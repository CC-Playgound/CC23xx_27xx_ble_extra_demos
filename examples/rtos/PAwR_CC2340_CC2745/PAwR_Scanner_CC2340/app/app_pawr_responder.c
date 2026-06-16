/******************************************************************************

 @file  app_pawr.c

 @brief This file contains the Periodic Advertising with Responses (PAwR)
        module implementation for BLE applications.

 The app_pawr module provides bidirectional communication using BLE Periodic
 Advertising with Responses. It supports two roles:

 PAwR Advertiser:
 - Creates periodic advertising with subevents
 - Provides subevent data via application callback
 - Receives responses from responders

 PAwR Responder:
 - Discovers and synchronizes to a PAwR advertiser by SID
 - Receives subevent data from the advertiser
 - Sends response data in designated response slots

 Architecture:
 - Uses BLEAppUtil as the abstraction layer for standard BLE operations
 - Uses direct GAP API calls for PAwR-specific functionality
 - V2 periodic advertising events used (include subevent field)
 - Application provides data via callbacks

 Group: WCS, BTS
 Target Device: cc23xx

******************************************************************************

 Copyright (c) 2024-2026, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************/


/*****************************************************************************
 Includes
 *****************************************************************************/

/* Module header first */
#include "app_pawr.h"

/* Standard C library headers */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* TI driver headers */
#include <ti/ble/app_util/framework/bleapputil_api.h>
#include <ti/ble/host/gap/gap_scanner.h>
#include <ti/ble/host/gap/gap_past.h>
#include <ti/ble/stack_util/bcomdef.h>

/* Project headers */
#include "app_main.h"
#include "ti_ble_config.h"
#include "ti/ble/app_util/menu/menu_module.h"


/*****************************************************************************
 * Local Function Prototypes
 *****************************************************************************/
static void AppPawr_ResponderPeriodicEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

static bStatus_t AppPawr_sendResponse(
    uint16_t syncHandle,
    uint8_t requestSubevent,
    uint8_t responseSubevent,
    uint8_t responseSlot,
    uint8_t dataLen,
    uint8_t *pData
);

bStatus_t AppPawr_setSubeventSync(
    uint16_t syncHandle,
    uint8_t numSubevents,
    uint8_t *subEvents
);
static uint8_t AppPawr_exampleResponseDataCallback(
    uint16_t syncHandle,
    uint8_t requestSubevent,
    uint8_t responseSubevent,
    uint8_t responseSlot,
    uint8_t *pDataLen,
    uint8_t *pData);

static void AppPawr_exampleSubeventReceivedCallback(
    uint16_t syncHandle,
    uint8_t subevent,
    uint8_t dataLen,
    uint8_t *pData);
/*****************************************************************************
 * Local Variables
 *****************************************************************************/

 /* Responder static variables */

static AppPawr_ResponderState_t gResponderState = APP_PAWR_RESPONDER_STATE_IDLE;
static uint16_t gSyncHandle = 0xFFFF;
static uint8_t gSubeventList[APP_PAWR_MAX_SUBEVENTS] = {0, 1, 2, 3};  /* Subevent indices to sync to */
static uint16_t gPeriodicEventCounter;


/* Responder periodic event handler for sync, data reception, and PAST */
static BLEAppUtil_EventHandler_t appPawrResponderPeriodicHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_PERIODIC_TYPE,
    .pEventHandler  = AppPawr_ResponderPeriodicEventHandler,
    .eventMask      = BLEAPPUTIL_SCAN_PERIODIC_ADV_REPORT_EVENT_V2   |
                      BLEAPPUTIL_SCAN_PERIODIC_ADV_SYNC_LOST_EVENT   |
                      BLEAPPUTIL_PAST_RECEIVED_V2_EVENT
};

/* Counter to include in response */
static uint8_t gResponseCounter = 0;

/* Example PAwR Responder Configuration */
static AppPawr_ResponderConfig_t gResponderConfig =
{
    .advSID = APP_PAWR_DEFAULT_ADV_SID,  /* Primary filter: sync to advertisers with this SID */
    .skip =APP_PAWR_DEFAULT_SKIP,
    .syncTimeout = APP_PAWR_DEFAULT_SYNC_TIMEOUT,  /* 2 seconds (N * 10ms) */
    .options = 0,

    /* Subevent subscription */
    .numSubevents = APP_PAWR_DEFAULT_NUM_SUBEVENTS,
    .subEvents = gSubeventList,

    /* Response slot assignment */
    /* Change this per device */
    .assignedResponseSlot = 0, 
    .deviceId = 0,              

    /* Application callbacks */
    .subeventCb = AppPawr_exampleSubeventReceivedCallback,
    .responseDataCb = AppPawr_exampleResponseDataCallback
};

/* Example callback: Provide response data when requested */
static uint8_t AppPawr_exampleResponseDataCallback(
    uint16_t syncHandle,
    uint8_t requestSubevent,
    uint8_t responseSubevent,
    uint8_t responseSlot,
    uint8_t *pDataLen,
    uint8_t *pData)
{
    /* Only respond in the assigned slot for this device */
    if (responseSlot != gResponderConfig.assignedResponseSlot)
    {
        return (0);
    }

    /* Build simple test response: [0xAA][deviceID][counter][subevent][0x55] */
    pData[0] = 0xAA;                        /* Response marker */
    pData[1] = gResponderConfig.deviceId;   /* Device identifier */
    pData[2] = gResponseCounter++;          /* Incrementing counter */
    pData[3] = requestSubevent;             /* Echo back subevent number */
    pData[4] = 0x55;                        /* Test byte */
    *pDataLen = 5;

    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                     "TX:slot=%d id=%d cnt=%d", responseSlot, gResponderConfig.deviceId, gResponseCounter - 1);

    return (1);  /* Send response */
}

static void AppPawr_exampleSubeventReceivedCallback(
    uint16_t syncHandle,
    uint8_t subevent,
    uint8_t dataLen,
    uint8_t *pData)
{
    /* Display received data */
    if (dataLen > 0 && pData != NULL)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE2, 0,
                         "RX:SE=%d len=%d", subevent, dataLen);

        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE3, 0,
                         "  data[0]=0x%02x", pData[0]);
    }
    else
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE2, 0,
                         "RX:SE=%d Invalid", subevent);
    }
}


/*********************************************************************
 * @fn      AppPawr_pastReceiverStart
 *
 * @brief   Initialize the PAwR responder for PAST-based sync.
 *
 *          Registers only the periodic event handler — no scanning.
 *          The controller auto-syncs when a PAST transfer arrives from
 *          the peer (mode=2 configured in app_connection.c); the handler
 *          subscribes to subevents via AppPawr_setSubeventSync().
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t AppPawr_pastReceiverStart(void)
{
    bStatus_t status;

    if (gResponderConfig.numSubevents == 0 ||
        gResponderConfig.numSubevents > APP_PAWR_MAX_SUBEVENTS ||
        gResponderConfig.subEvents == NULL ||
        gResponderConfig.subeventCb == NULL)
    {
        return (INVALIDPARAMETER);
    }

    status = BLEAppUtil_registerEventHandler(&appPawrResponderPeriodicHandler);
    if (status != SUCCESS)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "PAST: Periodic reg fail 0x%02x", status);
    }

    /* Configure default PAST receive parameters.
    * mode=2: controller auto-syncs on PAST receive and generates
    * periodic advertising reports (no duplicate filtering).
    * syncTimeout: time the controller will keep trying to find
    * the first packet of the advertised train after PAST arrives.
    * 1000 units * 10 ms = 10 seconds — 20 attempts at 500 ms interval. */
    Gap_PASTParams_t pastParams =
    {
        .mode        = 2,     /* Sync + reports, dup filtering disabled */
        .skip        = 0,
        .syncTimeout = 1000,  /* 10 s (N * 10 ms)                       */
        .cteType     = 0      /* Accept all CTE types                   */
    };
    Gap_SetDefaultPeriodicAdvSyncTransferParams(&pastParams);

    return (status);
}


/*********************************************************************
 * @fn      AppPawr_sendResponse
 *
 * @brief   Send response data (Responder only)
 *
 *          Sends response data in a response slot.
 *
 * @param   syncHandle         - Sync handle
 * @param   requestSubevent    - Subevent being responded to
 * @param   responseSubevent   - Subevent containing the response slot
 * @param   responseSlot       - Response slot index
 * @param   dataLen            - Length of response data (0-251)
 * @param   pData              - Pointer to response data
 *
 * @return  SUCCESS if response sent successfully
 * @return  INVALIDPARAMETER if parameters are invalid
 */
bStatus_t AppPawr_sendResponse(
    uint16_t syncHandle,
    uint8_t requestSubevent,
    uint8_t responseSubevent,
    uint8_t responseSlot,
    uint8_t dataLen,
    uint8_t *pData)
{
    uint8_t status;

    /* Validate sync handle */
    if (syncHandle != gSyncHandle)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                         "ERR: Invalid sync hdl");
        return (INVALIDPARAMETER);
    }

    /* Validate state */
    if (gResponderState != APP_PAWR_RESPONDER_STATE_SUBSCRIBED)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                         "ERR: Not subscribed");
        return (INVALIDPARAMETER);
    }

    /* Validate parameters */
    if (dataLen > APP_PAWR_MAX_RESPONSE_DATA_LEN)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                         "ERR: Data too long %d", dataLen);
        return (INVALIDPARAMETER);
    }

    if (dataLen > 0 && pData == NULL)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                         "ERR: NULL data ptr");
        return (INVALIDPARAMETER);
    }

    /* Build response parameters */
    GapScan_SetPeriodicResponseDataParams_t responseParams;

    responseParams.syncHandle   = syncHandle;
    responseParams.rqtEvent     = gPeriodicEventCounter;
    responseParams.rqtSubevent  = requestSubevent;
    responseParams.rspSubevent  = responseSubevent;
    responseParams.rspSlot      = responseSlot;
    responseParams.rspDataLen   = dataLen;
    responseParams.pData        = pData;

    /* Send response via GAP API */
    status = GapScan_SetPeriodicAdvResponseData(&responseParams);

    return (status);
}

/*********************************************************************
 * @fn      AppPawr_setSubeventSync
 *
 * @brief   Update subevent subscription (Responder only)
 *
 *          Changes which subevents the responder synchronizes to.
 *
 * @param   syncHandle    - Sync handle
 * @param   numSubevents  - Number of subevents to sync (1-128)
 * @param   subEvents     - Array of subevent indices to sync
 *
 * @return  SUCCESS if update successful
 * @return  INVALIDPARAMETER if parameters are invalid
 */
bStatus_t AppPawr_setSubeventSync(
    uint16_t syncHandle,
    uint8_t numSubevents,
    uint8_t *subEvents)
{
    uint8_t status;

    /* Validate sync handle */
    if (syncHandle != gSyncHandle)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                         "ERR: Invalid sync hdl");
        return (INVALIDPARAMETER);
    }

    /* Validate parameters */
    if (numSubevents == 0 || numSubevents > APP_PAWR_MAX_SUBEVENTS)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                         "ERR: Invalid num subevents %d", numSubevents);
        return (INVALIDPARAMETER);
    }

    if (subEvents == NULL)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                         "ERR: NULL subevent array");
        return (INVALIDPARAMETER);
    }

    /* Validate subevent indices are within bounds */
    for (uint8_t i = 0; i < numSubevents; i++)
    {
        if (subEvents[i] >= APP_PAWR_MAX_SUBEVENTS)
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                             "ERR: Subevent idx %d invalid", subEvents[i]);
            return (INVALIDPARAMETER);
        }
    }

    /* Copy subevent list to local buffer */
    memcpy(gSubeventList, subEvents, numSubevents);

    /* Build sync subevent parameters */
    GapScan_PeriodicSyncSubeventParams_t syncSubeventParams;

    syncSubeventParams.syncHandle   = syncHandle;
    syncSubeventParams.perAdvProps  = 0;  /* No special properties */
    syncSubeventParams.numSubevents = numSubevents;
    syncSubeventParams.subEvents    = gSubeventList;

    /* Configure subevent synchronization via GAP API */
    status = GapScan_SetPeriodicSyncSubevent(&syncSubeventParams);

    return (status);
}


/*********************************************************************
 * @fn      AppPawr_ResponderPeriodicEventHandler
 *
 * @brief   GAP Periodic Event Handler for PAwR Responder
 *
 *          Handles periodic advertising sync and data reception:
 *          - Sync established: Configure subevent synchronization
 *          - Periodic report: Receive subevent data, optionally send response
 *          - Sync lost: Restart scanning
 *
 * @param   event    - GAP periodic event type
 * @param   pMsgData - Pointer to event-specific data
 *
 * @return  None
 */
static void AppPawr_ResponderPeriodicEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{

    switch (event)
    {
        case BLEAPPUTIL_SCAN_PERIODIC_ADV_REPORT_EVENT_V2:
        {
            GapScan_Evt_PeriodicAdvRptV2_t *pEvt = (GapScan_Evt_PeriodicAdvRptV2_t *)pMsgData;

            /* Validate we're in correct state */
            if (gResponderState != APP_PAWR_RESPONDER_STATE_SUBSCRIBED)
            {
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                                 "WARN: Report in wrong state %d", gResponderState);
                break;
            }

            /* Validate sync handle matches */
            if (pEvt->syncHandle != gSyncHandle)
            {
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                                 "WARN: Wrong sync hdl 0x%04x", pEvt->syncHandle);
                break;
            }

            /* Save periodic event counter for response sending */
            gPeriodicEventCounter = pEvt->periodicEventCounter;

            /* Call application callback with received subevent data */
            if (gResponderConfig.subeventCb != NULL)
            {
                gResponderConfig.subeventCb(
                    pEvt->syncHandle,
                    pEvt->subEvent,
                    pEvt->dataLen,
                    pEvt->pData
                );
            }

            /* Only attempt to send response if we received valid data */
            /* Sending responses with NULL/0len data causes controller buffer exhaustion */
            if (pEvt->dataLen > 0 && pEvt->pData != NULL && gResponderConfig.responseDataCb != NULL)
            {
                uint8_t responseDataLen = 0;
                static uint8_t responseData[APP_PAWR_MAX_RESPONSE_DATA_LEN];

                /* Clear response buffer to ensure no stale data */
                memset(responseData, 0, sizeof(responseData));

                /* Call callback to check if response should be sent */
                /* Response happens in the SAME subevent's response slots */
                /* Response slot delay (20ms) gives responder time to process */
                uint8_t shouldRespond = gResponderConfig.responseDataCb(
                    pEvt->syncHandle,
                    pEvt->subEvent,
                    pEvt->subEvent,
                    gResponderConfig.assignedResponseSlot,
                    &responseDataLen,
                    responseData
                );

                if (shouldRespond != 0 && responseDataLen > 0)
                {
                    /* Validate response data length */
                    if (responseDataLen > APP_PAWR_MAX_RESPONSE_DATA_LEN)
                    {
                        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                                         "ERR: Response too long %d", responseDataLen);
                        break;
                    }

                    bStatus_t status = AppPawr_sendResponse(
                        pEvt->syncHandle,
                        pEvt->subEvent,
                        pEvt->subEvent,
                        gResponderConfig.assignedResponseSlot,
                        responseDataLen,
                        responseData
                    );
                    if (status != SUCCESS)
                    {
                        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                                         "ERR: Rsp send 0x%02x", status);
                    }
                }
            }
            /* Needed until stack fix applied */
            if(pEvt->dataLen > 0 && pEvt->pData != NULL)
            {
                BLEAppUtil_free(pEvt->pData);
            }
            break;
        }

        case BLEAPPUTIL_SCAN_PERIODIC_ADV_SYNC_LOST_EVENT:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE2, 0,
                             "PAwR: Sync LOST");

            /* Sync lost - reset back to idle and restart scanning */
            gResponderState = APP_PAWR_RESPONDER_STATE_IDLE;
            gSyncHandle = 0xFFFF;
            break;
        }

        case BLEAPPUTIL_PAST_RECEIVED_V2_EVENT:
        {
            uint16_t PASTconnHandle;
            GapPAST_PeriodicAdvSyncTransRcvV2_t *pEvt =
                (GapPAST_PeriodicAdvSyncTransRcvV2_t *)pMsgData;

            /* hdr.status carries the HCI LE Meta Event code (0x3E), NOT
             * the BLE synchronization result.  Per the BLE spec, a failed
             * sync is indicated by syncHandle == 0xFFFF; a valid handle
             * means the controller successfully established the sync. */
            if (pEvt->syncHandle == 0xFFFF)
            {
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                                 "PAST: Failed (no valid sync handle)");
                break;
            }

            /* Only accept sync if not already subscribed */
            if (gResponderState != APP_PAWR_RESPONDER_STATE_IDLE)
            {
                break;
            }

            gSyncHandle     = pEvt->syncHandle;
            gResponderState = APP_PAWR_RESPONDER_STATE_SYNCED;

            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                             "PAST: sync=0x%04x sid=%d", pEvt->syncHandle, pEvt->sid);

            /* Subscribe to the configured subevents */
            bStatus_t syncStatus = AppPawr_setSubeventSync(gSyncHandle,
                                                            gResponderConfig.numSubevents,
                                                            gResponderConfig.subEvents);
            if (syncStatus == SUCCESS)
            {
                gResponderState = APP_PAWR_RESPONDER_STATE_SUBSCRIBED;
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE2, 0,
                                 "PAST: Subscribed %d subevents", gResponderConfig.numSubevents);
            }
            else
            {
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE2, 0,
                                 "PAST: SubEvt fail 0x%02x", syncStatus);
                gResponderState = APP_PAWR_RESPONDER_STATE_IDLE;
                gSyncHandle     = 0xFFFF;
            }
            PASTconnHandle = App_Pawr_getConnHandle();  /* Update global connection handle for PAwR */
            BLEAppUtil_disconnect(PASTconnHandle);  /* Disconnect from sync to trigger new sync with subevent config */
            break;
        }

        default:
            break;
    }
}
