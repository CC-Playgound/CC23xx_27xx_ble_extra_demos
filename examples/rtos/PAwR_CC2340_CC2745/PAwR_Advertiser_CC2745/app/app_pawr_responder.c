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
#include <ti/ble/stack_util/bcomdef.h>

/* Project headers */
#include "app_main.h"
#include "ti_ble_config.h"
#include "ti/ble/app_util/menu/menu_module.h"


/*****************************************************************************
 * Local Function Prototypes
 *****************************************************************************/

static void AppPawr_ResponderScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void AppPawr_ResponderPeriodicEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static bStatus_t AppPawr_createSync(uint8_t addrType, uint8_t *advAddress);
static bStatus_t AppPawr_scanStart(void);

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

/* Responder scan event handler for advertiser discovery */
static BLEAppUtil_EventHandler_t appPawrResponderScanHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_SCAN_TYPE,
    .pEventHandler  = AppPawr_ResponderScanEventHandler,
    .eventMask      = BLEAPPUTIL_SCAN_ENABLED  |
                      BLEAPPUTIL_SCAN_DISABLED |
                      BLEAPPUTIL_ADV_REPORT
};

/* Responder periodic event handler for sync and data reception */
static BLEAppUtil_EventHandler_t appPawrResponderPeriodicHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_PERIODIC_TYPE,
    .pEventHandler  = AppPawr_ResponderPeriodicEventHandler,
    .eventMask      = BLEAPPUTIL_SCAN_PERIODIC_ADV_SYNC_EST_EVENT_V2 |
                      BLEAPPUTIL_SCAN_PERIODIC_ADV_REPORT_EVENT_V2 |
                      BLEAPPUTIL_SCAN_PERIODIC_ADV_SYNC_LOST_EVENT
};

/* Counter to include in response */
static uint8_t gResponseCounter = 0;

/* Retry tracking for sync failures */
static uint8_t gSyncRetryCount = 0;
#define APP_PAWR_MAX_SYNC_RETRIES (5)
#define APP_PAWR_SYNC_RETRY_DELAY_MS (1000)  /* 1 second base delay */

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
    .assignedResponseSlot = 2, 
    .deviceId = 2,              

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
                         "RX:SE=%d NULL/0len", subevent);
    }
}


/*
 * @fn      AppPawr_start
 *
 * @brief   Start the PAwR module
 *
 *          This function is called after stack initialization to start the
 *          PAwR module based on the specified role. Uses the global pawrConfig.
 *          For Advertiser: Creates advertising set, configures PAwR parameters
 *          For Responder: Starts scanning for advertiser by SID
 *
 * @return  SUCCESS if initialization successful
 * @return  INVALIDPARAMETER if configuration contains invalid values
 */
bStatus_t AppPawr_responder_start(void)
{
    bStatus_t status;

    /* Validate configuration parameters */
    if (gResponderConfig.advSID > APP_PAWR_MAX_SID)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Invalid SID %d", gResponderConfig.advSID);
        return (INVALIDPARAMETER);
    }

    if (gResponderConfig.numSubevents == 0 ||
        gResponderConfig.numSubevents > APP_PAWR_MAX_SUBEVENTS)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Invalid subevent cnt");
        return (INVALIDPARAMETER);
    }

    if (gResponderConfig.subEvents == NULL)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: NULL subevent array");
        return (INVALIDPARAMETER);
    }

    if (gResponderConfig.subeventCb == NULL)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: NULL subevent callback");
        return (INVALIDPARAMETER);
    }

    /* Reset retry counter */
    gSyncRetryCount = 0;

    status = BLEAppUtil_registerEventHandler(&appPawrResponderPeriodicHandler);
    if (status != SUCCESS)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Periodic reg fail 0x%02x", status);
        return (status);
    }

    status = BLEAppUtil_registerEventHandler(&appPawrResponderScanHandler);
    if (status != SUCCESS)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Scan reg fail 0x%02x", status);
        return (status);
    }

    status = AppPawr_scanStart();
    if (status != SUCCESS)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Scan start fail 0x%02x", status);
    }

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

static bStatus_t AppPawr_scanStart(void)
{
    bStatus_t status;

    BLEAppUtil_ScanInit_t scanInitParams =
    {
        .primPhy            = SCAN_PRIM_PHY_1M,
        .scanType           = SCAN_TYPE_PASSIVE,
        .scanInterval       = 160,
        .scanWindow         = 160,
        .advReportFields    = SCAN_ADVRPT_FLD_ADDRTYPE | SCAN_ADVRPT_FLD_ADDRESS |
                            SCAN_ADVRPT_FLD_ADVSID | SCAN_ADVRPT_FLD_RSSI |
                            SCAN_ADVRPT_FLD_DATALEN,
        .scanPhys           = SCAN_PRIM_PHY_1M,
        .fltPolicy          = SCAN_FLT_POLICY_ALL,
        .fltPduType         = SCAN_FLT_PDU_EXTENDED_ONLY | SCAN_FLT_PDU_COMPLETE_ONLY,
        .fltMinRssi         = SCAN_FLT_RSSI_ALL,
        .fltDiscMode        = SCAN_FLT_DISC_NONE,
        .fltDup             = SCAN_FLT_DUP_DISABLE
    };

    /* Initialize scanner with parameters */
    status = BLEAppUtil_scanInit(&scanInitParams);

    if (status == SUCCESS)
    {
        /* Start scanning with unlimited duration */
        BLEAppUtil_ScanStart_t scanStartParams = {0, 0, 0};

        status = BLEAppUtil_scanStart(&scanStartParams);
    }

    return (status);
}

/*********************************************************************
 * @fn      AppPawr_ResponderScanEventHandler
 *
 * @brief   Scan Event Handler for Responder advertiser discovery
 *
 *          Handles GAP scan events during advertiser discovery:
 *          - BLEAPPUTIL_ADV_REPORT: Check if advertisement matches advertiser SID
 *          - BLEAPPUTIL_SCAN_ENABLED: Update state to scanning
 *          - BLEAPPUTIL_SCAN_DISABLED: Retry scanning if not synced
 *
 * @param   event    - GAP scan event type
 * @param   pMsgData - Pointer to event-specific data
 *
 * @return  None
 */
static void AppPawr_ResponderScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    BLEAppUtil_ScanEventData_t *scanMsg = (BLEAppUtil_ScanEventData_t *)pMsgData;

    switch (event)
    {
        case BLEAPPUTIL_ADV_REPORT:
        {
            /* Only process if we're actively scanning for advertiser */
            if (gResponderState != APP_PAWR_RESPONDER_STATE_SCANNING)
            {
                break;
            }

            BLEAppUtil_GapScan_Evt_AdvRpt_t *pAdvRpt = &scanMsg->pBuf->pAdvReport;

            /* Match by SID only - allows multiple devices to sync if they use the same SID
             * Also verify that periodic advertising is present (periodicAdvInt != 0) */
            if (gResponderConfig.advSID == pAdvRpt->advSid &&
                pAdvRpt->periodicAdvInt != 0)
            {

                bStatus_t syncStatus = AppPawr_createSync(pAdvRpt->addrType, pAdvRpt->addr);

                if (syncStatus == SUCCESS)
                {
                    /* Transition to syncing state */
                    gResponderState = APP_PAWR_RESPONDER_STATE_SYNCING;
                }
                else
                {
                    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                                     "PAwR: Sync create FAIL");
                }
            }
            break;
        }

        case BLEAPPUTIL_SCAN_ENABLED:
        {
            gResponderState = APP_PAWR_RESPONDER_STATE_SCANNING;
            break;
        }

        case BLEAPPUTIL_SCAN_DISABLED:
        {
            /* Handle scan timeout based on current state */
            if (gResponderState == APP_PAWR_RESPONDER_STATE_SCANNING ||
                gResponderState == APP_PAWR_RESPONDER_STATE_SYNCING)
            {
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE2, 0,
                                 "PAwR: Scan timeout");
                /* Restart scan to keep looking for advertiser */
                AppPawr_scanStart();
            }
            break;
        }

        default:
            break;
    }
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
        case BLEAPPUTIL_SCAN_PERIODIC_ADV_SYNC_EST_EVENT_V2:
        {
            GapScan_Evt_PeriodicAdvSyncEstV2_t *pEvt = (GapScan_Evt_PeriodicAdvSyncEstV2_t *)pMsgData;

            /* Only process successful sync establishment */
            if (pEvt->status == SUCCESS)
            {

                /* Update state and save sync handle */
                if (gResponderState == APP_PAWR_RESPONDER_STATE_SYNCING)
                {
                    gResponderState = APP_PAWR_RESPONDER_STATE_SYNCED;
                    gSyncHandle = pEvt->syncHandle;

                    /* Reset retry counter on success */
                    gSyncRetryCount = 0;

                    /* Stop scanning - no longer needed after sync is established */
                    BLEAppUtil_scanStop();

                    /* Configure subevent synchronization */
                    bStatus_t syncStatus = AppPawr_setSubeventSync(gSyncHandle,
                                                                    gResponderConfig.numSubevents,
                                                                    gResponderConfig.subEvents);
                    if (syncStatus == SUCCESS)
                    {
                        gResponderState = APP_PAWR_RESPONDER_STATE_SUBSCRIBED;
                    }
                    else
                    {
                        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE3, 0,
                                        "ERR: SubEvt fail 0x%02x", syncStatus);
                        /* Sync failed - restart process */
                        gResponderState = APP_PAWR_RESPONDER_STATE_IDLE;
                        gSyncHandle = 0xFFFF;
                        AppPawr_scanStart();
                    }
                }
            }
            else
            {
                /* Sync establish failed - implement retry with exponential backoff */
                gSyncRetryCount++;
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE2, 0,
                                 "PAwR: Sync FAIL 0x%02x retry %d/%d",
                                 pEvt->status, gSyncRetryCount, APP_PAWR_MAX_SYNC_RETRIES);

                if (gSyncRetryCount >= APP_PAWR_MAX_SYNC_RETRIES)
                {
                    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE3, 0,
                                     "PAwR: Max retries reached");
                    gResponderState = APP_PAWR_RESPONDER_STATE_IDLE;
                    gSyncRetryCount = 0;

                }
                else
                {
                    /* Reset back to scanning for retry */
                    gResponderState = APP_PAWR_RESPONDER_STATE_SCANNING;
                }
            }
            break;
        }

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

            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE3, 0,
                             "PAwR: Restarting scan");
            AppPawr_scanStart();
            break;
        }

        default:
            break;
    }
}

/*********************************************************************
 * @fn      AppPawr_createSync
 *
 * @brief   Create periodic advertising sync with discovered advertiser address
 *
 * @param   addrType    - Address type
 * @param   advAddress  - Advertiser address
 *
 * @return  SUCCESS if sync creation initiated, error code otherwise
 */
static bStatus_t AppPawr_createSync(uint8_t addrType, uint8_t *advAddress)
{
    uint8_t status;

    /* Validate parameters */
    if (advAddress == NULL)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                         "ERR: NULL address");
        return (INVALIDPARAMETER);
    }

    /* Validate address type */
    if (addrType > APP_PAWR_MAX_ADDR_TYPE)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
                         "ERR: Invalid addr type %d", addrType);
        return (INVALIDPARAMETER);
    }

    GapScan_PeriodicAdvCreateSyncParams_t syncParams =
    {
        .options     = gResponderConfig.options,
        .advAddrType = addrType,
        .skip        = gResponderConfig.skip,
        .syncTimeout = gResponderConfig.syncTimeout,
        .syncCteType = 0  /* Accept all CTE types */
    };

    /* Copy discovered advertiser address */
    memcpy(syncParams.advAddress, advAddress, B_ADDR_LEN);

    
    status = GapScan_PeriodicAdvCreateSync(gResponderConfig.advSID, &syncParams);

    return (status);
}