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

******************************************************************************


*****************************************************************************/

/*****************************************************************************
 * Includes
 *****************************************************************************/

/* Module header first */
#include "app_pawr.h"

/* Standard C library headers */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* TI driver headers */
#include <ti/ble/app_util/framework/bleapputil_api.h>
#include <ti/ble/host/gap/gap_advertiser.h>
#include <ti/ble/stack_util/bcomdef.h>

/* Project headers */
#include "app_main.h"
#include "ti_ble_config.h"
#include "ti/ble/app_util/menu/menu_module.h"

static void AppPawr_AdvertiserPeriodicEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void AppPawr_handleSubeventDataRequest(GapAdv_PeriodicAdvASubeventDataRequest_t *pEvt);
static void AppPawr_handleResponseReport(GapAdv_PeriodicAdvAResponseReport_t *pEvt);
static bStatus_t AppPawr_buildSubeventDataBuffer(uint8_t numSubevents, AppPawr_SubeventData_t *pSubevents,
                                               uint8_t *pBuffer, uint16_t *pBufLen);
static void AppPawr_parseResponseBuffer(uint8_t *pBuffer, uint8_t numResponses,
                                        AppPawr_ResponseInfo_t *pResponses);

/* Advertiser static variables */
uint8_t gAdvHandle;

/* Global buffers for subevent data handling */
static AppPawr_SubeventData_t gSubeventDataArray[APP_PAWR_MAX_SUBEVENTS];
static uint8_t gSubeventBuffer[APP_PAWR_SUBEVENT_BUFFER_SIZE];

/* Advertiser event handler for periodic advertising events */
static BLEAppUtil_EventHandler_t appPawrAdvertiserPeriodicHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_PERIODIC_TYPE,
    .pEventHandler  = AppPawr_AdvertiserPeriodicEventHandler,
    .eventMask      = BLEAPPUTIL_ADV_PERIODIC_ADV_SUBEVENT_DATA_REQ_EVENT |
                      BLEAPPUTIL_ADV_PERIODIC_ADV_RSP_HDR_REPORT_EVENT    
};

/* Static buffers for subevent data - automatically sized based on configuration */
static uint8_t gSubeventDataBuffers[APP_PAWR_SUBEVENT_BUFFER_COUNT][APP_PAWR_MAX_SUBEVENT_DATA_LEN];

/* Counter for test data */
static uint8_t gSubeventCounter = 0;


uint8_t App_Pawr_getAdvHandle(void)
{
    return gAdvHandle;
}

/* Example callback: Provide subevent data when requested by controller */
static void AppPawr_exampleSubeventDataCallback(
    uint8_t advHandle,
    uint8_t subeventStart,
    uint8_t subeventCount,
    AppPawr_SubeventData_t *pSubeventData)
{

    /* Provide simple test data for each requested subevent */
    for (uint8_t i = 0; i < subeventCount; i++)
    {
        uint8_t subeventNum = subeventStart + i;

        /* Validate subevent index is within buffer bounds */
        if (subeventNum >= APP_PAWR_SUBEVENT_BUFFER_COUNT)
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                             "ERR: Subevent %d exceeds buffer max %d", subeventNum, APP_PAWR_SUBEVENT_BUFFER_COUNT);

            /* Must still initialize this entry to prevent sending garbage data */
            pSubeventData[i].subeventNum = subeventNum;
            pSubeventData[i].rspSlotStart = 0;
            pSubeventData[i].rspSlotCount = 0;
            pSubeventData[i].pData = NULL;
            pSubeventData[i].dataLen = 0;
            continue;
        }

        /* Build simple test data: [0x55][counter][subevent_num] */
        gSubeventDataBuffers[subeventNum][0] = 0x55;              /* Data marker */
        gSubeventDataBuffers[subeventNum][1] = gSubeventCounter;
        gSubeventDataBuffers[subeventNum][2] = subeventNum;

        /* Fill subevent data structure - each subevent points to its own buffer */
        pSubeventData[i].subeventNum = subeventNum;
        pSubeventData[i].rspSlotStart = 0;
        pSubeventData[i].rspSlotCount = 4;
        pSubeventData[i].pData = gSubeventDataBuffers[subeventNum];
        pSubeventData[i].dataLen = 3;

        /* Increment counter AFTER filling buffer to avoid race condition */
        gSubeventCounter++;
    }
}


static void AppPawr_exampleResponseCallback(
    uint8_t advHandle,
    uint8_t subevent,
    uint8_t numResponses,
    AppPawr_ResponseInfo_t *pResponses)
{
    /* Validate parameters */
    if (pResponses == NULL)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: NULL response ptr");
        return;
    }

    if (numResponses > APP_PAWR_MAX_RESPONSES_PER_EVENT)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "WARN: Too many responses %d", numResponses);
        numResponses = APP_PAWR_MAX_RESPONSES_PER_EVENT;  /* Limit to array size */
    }

    /* Note: numResponses includes both successful (dataStatus=0x00) and failed (dataStatus=0xFF)
     * response slots. Failed slots have dataLen=0 and should be filtered out. */

    /* Process each received response and display on separate lines */
    for (uint8_t i = 0; i < numResponses; i++)
    {
        /* Determine which display line to use based on response slot number */
        uint8_t displayLine;
        switch (pResponses[i].responseSlot)
        {
            case 0:
                displayLine = APP_MENU_PROFILE_STATUS_LINE;
                break;
            case 1:
                displayLine = APP_MENU_PROFILE_STATUS_LINE2;
                break;
            case 2:
                displayLine = APP_MENU_PROFILE_STATUS_LINE3;
                break;
            case 3:
                displayLine = APP_MENU_PROFILE_STATUS_LINE4;
                break;
            default:
                displayLine = APP_MENU_PROFILE_STATUS_LINE4;
                break;
        }

        /* Filter out failed response slots - only process responses with valid data */
        if (pResponses[i].dataLen >= 5 && pResponses[i].dataStatus == 0x00)
        {
            /* Parse response: [0xAA][deviceID][counter][subevent][test_byte] */
            uint8_t *pData = pResponses[i].pData;

            /* Validate data pointer */
            if (pData == NULL)
            {
                MenuModule_printf(displayLine, 0,
                                 "Slot%d:NULL data ptr", pResponses[i].responseSlot);
                continue;
            }

            if (pData[0] == 0xAA)  /* Check response marker */
            {
                MenuModule_printf(displayLine, 0,
                                 "Slot%d:ID=%d cnt=%d SE=%d",
                                 pResponses[i].responseSlot,
                                 pData[1],  /* Device ID */
                                 pData[2],  /* Counter */
                                 pData[3]); /* Subevent echo */
            }
            else
            {
                MenuModule_printf(displayLine, 0,
                                 "Slot%d:Bad marker 0x%02x",
                                 pResponses[i].responseSlot,
                                 pData[0]);
            }
        }
        else {
            MenuModule_printf(displayLine, 0,
                                 "datalen %d. :status 0x%02x",
                                 pResponses[i].dataLen,
                                 pResponses[i].dataStatus);
        }
    }
}

/* Example PAwR Advertiser Configuration */
static AppPawr_AdvertiserConfig_t gAdvertiserConfig =
{
        .advSID = APP_PAWR_DEFAULT_ADV_SID,
        .periodicAdvIntervalMin = APP_PAWR_DEFAULT_PERIODIC_ADV_INTERVAL_MIN,  
        .periodicAdvIntervalMax = APP_PAWR_DEFAULT_PERIODIC_ADV_INTERVAL_MAX,
        .periodicAdvProp = 0x0040,      /* Include TxPower */

        /* PAwR parameters */
        .numSubevents = APP_PAWR_DEFAULT_NUM_SUBEVENTS,
        .subeventInterval = APP_PAWR_DEFAULT_SUBEVENT_INTERVAL,         
        .responseSlotDelay = APP_PAWR_DEFAULT_RESPONSE_SLOT_DELAY,          
        .responseSlotSpacing = APP_PAWR_DEFAULT_RESPONSE_SLOT_SPACING,        
        .numOfResponseSlots = APP_PAWR_DEFAULT_NUM_RESPONSE_SLOTS,

        /* Application callbacks */
        .subeventDataCb = AppPawr_exampleSubeventDataCallback,
        .responseCb = AppPawr_exampleResponseCallback   
};

/*********************************************************************
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
bStatus_t AppPawr_advertiser_start(void)
{
    bStatus_t status;

    /* Validate configuration parameters */
    if (gAdvertiserConfig.advSID > APP_PAWR_MAX_SID)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Invalid SID %d", gAdvertiserConfig.advSID);
        return (INVALIDPARAMETER);
    }

    if (gAdvertiserConfig.numSubevents == 0 ||
        gAdvertiserConfig.numSubevents > APP_PAWR_MAX_SUBEVENTS)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Invalid subevent cnt %d", gAdvertiserConfig.numSubevents);
        return (INVALIDPARAMETER);
    }

    if (gAdvertiserConfig.numOfResponseSlots == 0)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Zero response slots");
        return (INVALIDPARAMETER);
    }

    if (gAdvertiserConfig.subeventDataCb == NULL)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: NULL subevent callback");
        return (INVALIDPARAMETER);
    }

    /* Create extended advertising parameters for periodic advertising */
    BLEAppUtil_AdvParams_t advParams =
    {
        .eventProps     = APP_PAWR_EXTENDED_ADV_PROP, 
        .primIntMin     = 160,  
        .primIntMax     = 160,  
        .primChanMap    = BLEAPPUTIL_ADV_CHAN_ALL,
        .peerAddrType   = BLEAPPUTIL_PEER_ADDRTYPE_PUBLIC_OR_PUBLIC_ID,
        .peerAddr       = { 0, 0, 0, 0, 0, 0 },
        .filterPolicy   = BLEAPPUTIL_ADV_AL_POLICY_ANY_REQ,
        .txPower        = BLEAPPUTIL_ADV_TX_POWER_NO_PREFERENCE,
        .primPhy        = BLEAPPUTIL_ADV_PRIM_PHY_1_MBPS,
        .secPhy         = BLEAPPUTIL_ADV_SEC_PHY_1_MBPS,
        .sid            = gAdvertiserConfig.advSID,
        .zeroDelay      = 0
    };

    
    BLEAppUtil_AdvInit_t advInitInfo =
    {
        .advDataLen      = 0,
        .advData         = NULL,
        .scanRespDataLen = 0,
        .scanRespData    = NULL,
        .advParam        = &advParams
    };

    
    status = BLEAppUtil_initAdvSet(&gAdvHandle, &advInitInfo);

    if (status != SUCCESS)
    {
        return (status);
    }

    
    GapAdv_periodicAdvParams_t periodicParams;
    periodicParams.periodicAdvIntervalMin = gAdvertiserConfig.periodicAdvIntervalMin;
    periodicParams.periodicAdvIntervalMax = gAdvertiserConfig.periodicAdvIntervalMax;
    periodicParams.periodicAdvProp        = gAdvertiserConfig.periodicAdvProp;

    
    GapAdv_PAwRParams_t pawrParams;
    pawrParams.numSubevents         = gAdvertiserConfig.numSubevents;
    pawrParams.subeventInterval     = gAdvertiserConfig.subeventInterval;
    pawrParams.responseSlotDelay    = gAdvertiserConfig.responseSlotDelay;
    pawrParams.responseSlotSpacing  = gAdvertiserConfig.responseSlotSpacing;
    pawrParams.numOfResponseSlots   = gAdvertiserConfig.numOfResponseSlots;

    /* Set both periodic and PAwR parameters using V2 API */
    status = GapAdv_SetPeriodicAdvParamsV2(gAdvHandle, &periodicParams, &pawrParams);

    if (status != SUCCESS)
    {
        return (status);
    }

    /* Register periodic event handler for subevent data requests and response reports */
    status = BLEAppUtil_registerEventHandler(&appPawrAdvertiserPeriodicHandler);

    if (status != SUCCESS)
    {
        return (status);
    }

    /* Enable extended advertising first (required before enabling periodic) */
    BLEAppUtil_AdvStart_t advStartInfo =
    {
        .enableOptions      = BLEAPPUTIL_ADV_START_ENABLE_OPTIONS_USE_MAX,
        .durationOrMaxEvents = 0
    };

    status = BLEAppUtil_advStart(gAdvHandle, &advStartInfo);

    if (status != SUCCESS)
    {
        return (status);
    }

    
    status = BLEAppUtil_setPeriodicAdvEnable(1, gAdvHandle);

    return (status);
}

/*********************************************************************
 * @fn      AppPawr_AdvertiserPeriodicEventHandler
 *
 * @brief   GAP Periodic Event Handler for PAwR Advertiser
 *
 *          Handles periodic advertising events:
 *          - Subevent data requests from controller
 *          - Response reports from responders
 *
 * @param   event    - GAP periodic event type
 * @param   pMsgData - Pointer to event-specific data
 *
 * @return  None
 */

static void AppPawr_AdvertiserPeriodicEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_ADV_PERIODIC_ADV_SUBEVENT_DATA_REQ_EVENT:
        {
            GapAdv_PeriodicAdvASubeventDataRequest_t *pReq =
            (GapAdv_PeriodicAdvASubeventDataRequest_t *)pMsgData;
            AppPawr_handleSubeventDataRequest(pReq);
            break;
        }
        case BLEAPPUTIL_ADV_PERIODIC_ADV_RSP_HDR_REPORT_EVENT:
        {
            GapAdv_PeriodicAdvAResponseReport_t *pReq =
            (GapAdv_PeriodicAdvAResponseReport_t *)pMsgData;
            AppPawr_handleResponseReport(pReq);

            break;
        }
        default:
        {
            break;
        }
    }
}

/*********************************************************************
 * @fn      AppPawr_handleSubeventDataRequest
 *
 * @brief   Handle subevent data request from controller
 *
 *          Calls application callback to get subevent data, formats
 *          it into the required interleaved buffer format, and sends
 *          it to the controller.
 *
 * @param   pEvt - Pointer to periodic advertising event
 *
 * @return  None
 */
static void AppPawr_handleSubeventDataRequest(GapAdv_PeriodicAdvASubeventDataRequest_t *pEvt)
{
    /* Use global static storage */
    uint16_t bufLen = 0;
    bStatus_t status;

    /* Validate event parameters */
    if (pEvt->subeventCount == 0 || pEvt->subeventCount > APP_PAWR_MAX_SUBEVENTS)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Invalid subevent req %d", pEvt->subeventCount);
        return;
    }

    /* Clear buffers before use */
    memset(gSubeventDataArray, 0, sizeof(gSubeventDataArray));
    memset(gSubeventBuffer, 0, sizeof(gSubeventBuffer));

    /* Call application callback to get subevent data */
    if (gAdvertiserConfig.subeventDataCb != NULL)
    {
        gAdvertiserConfig.subeventDataCb(
            pEvt->advHandle,
            pEvt->subeventStart,
            pEvt->subeventCount,
            gSubeventDataArray
        );
    }
    else
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: NULL subevent callback");
        return;
    }

    /* Build interleaved buffer format */
    status = AppPawr_buildSubeventDataBuffer(pEvt->subeventCount, gSubeventDataArray, gSubeventBuffer, &bufLen);
    if (status != SUCCESS)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Build buffer fail 0x%02x", status);
        return;
    }

    /* Send subevent data to controller using global buffer */
    GapAdv_padvSubEvtData_t padvSubEvtData;

    padvSubEvtData.advHandle        = pEvt->advHandle;
    padvSubEvtData.numSubEvtWithData = pEvt->subeventCount;
    padvSubEvtData.pSubEvtData      = gSubeventBuffer;

    status = GapAdv_SetPeriodicAdvSubeventData(&padvSubEvtData);
    if (status != SUCCESS)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Set subevent fail 0x%02x", status);
    }

}

/*********************************************************************
 * @fn      AppPawr_handleResponseReport
 *
 * @brief   Handle response report from controller
 *
 *          Parses the response buffer and calls application callback
 *          with the parsed response information.
 *
 * @param   pEvt - Pointer to periodic advertising event
 *
 * @return  None
 */
static void AppPawr_handleResponseReport(GapAdv_PeriodicAdvAResponseReport_t *pEvt)
{
    AppPawr_ResponseInfo_t responses[APP_PAWR_MAX_RESPONSES_PER_EVENT];

    /* Parse the response buffer - we know how many responses to expect */
    AppPawr_parseResponseBuffer(pEvt->responses, pEvt->numResponses, responses);

    /* Call application callback with parsed responses */
    gAdvertiserConfig.responseCb(
        pEvt->advHandle,
        pEvt->subevent,
        pEvt->numResponses,
        responses
    );
}

/*********************************************************************
 * @fn      AppPawr_buildSubeventDataBuffer
 *
 * @brief   Build interleaved buffer format for subevent data
 *
 *          Format: For each subevent:
 *          [subeventNum][rspSlotStart][rspSlotCount][dataLen][data...]
 *
 * @param   numSubevents - Number of subevents
 * @param   pSubevents   - Array of subevent data structures
 * @param   pBuffer      - Output buffer
 * @param   pBufLen      - Output buffer length
 *
 * @return  SUCCESS if buffer built successfully
 */
static bStatus_t AppPawr_buildSubeventDataBuffer(
    uint8_t numSubevents,
    AppPawr_SubeventData_t *pSubevents,
    uint8_t *pBuffer,
    uint16_t *pBufLen)
{
    uint16_t idx = 0;

    /* Validate parameters */
    if (pSubevents == NULL || pBuffer == NULL || pBufLen == NULL)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: NULL ptr in build");
        return (INVALIDPARAMETER);
    }

    if (numSubevents == 0 || numSubevents > APP_PAWR_MAX_SUBEVENTS)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: Invalid subevent cnt");
        return (INVALIDPARAMETER);
    }

    for (uint8_t i = 0; i < numSubevents; i++)
    {
        /* Validate buffer won't overflow */
        if (idx + APP_PAWR_SUBEVENT_HEADER_SIZE + pSubevents[i].dataLen > APP_PAWR_SUBEVENT_BUFFER_SIZE)
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                             "ERR: Buffer overflow prevented");
            return (INVALIDPARAMETER);
        }

        /* Validate data length */
        if (pSubevents[i].dataLen > APP_PAWR_MAX_SUBEVENT_DATA_LEN)
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                             "ERR: Subevent data too long");
            return (INVALIDPARAMETER);
        }

        pBuffer[idx++] = pSubevents[i].subeventNum;
        pBuffer[idx++] = pSubevents[i].rspSlotStart;
        pBuffer[idx++] = pSubevents[i].rspSlotCount;
        pBuffer[idx++] = pSubevents[i].dataLen;

        if (pSubevents[i].dataLen > 0)
        {
            if (pSubevents[i].pData == NULL)
            {
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                                 "ERR: NULL data with len>0");
                return (INVALIDPARAMETER);
            }
            memcpy(&pBuffer[idx], pSubevents[i].pData, pSubevents[i].dataLen);
            idx += pSubevents[i].dataLen;
        }
    }

    *pBufLen = idx;

    return (SUCCESS);
}

/*********************************************************************
 * @fn      AppPawr_parseResponseBuffer
 *
 * @brief   Parse response buffer from controller
 *
 *          Format per response:
 *          [txPower][rssi][cteType][slot][status][len][data...]
 *
 * @param   pBuffer      - Input buffer with response data
 * @param   numResponses - Number of responses in buffer
 * @param   pResponses   - Output: array of response info structures
 *
 * @return  None
 */
static void AppPawr_parseResponseBuffer(
    uint8_t *pBuffer,
    uint8_t numResponses,
    AppPawr_ResponseInfo_t *pResponses)
{
    uint16_t idx = 0;

    /* Validate parameters */
    if (pBuffer == NULL || pResponses == NULL)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                         "ERR: NULL ptr in parse");
        return;
    }

    /* Limit to array size */
    if (numResponses > APP_PAWR_MAX_RESPONSES_PER_EVENT)
    {
        numResponses = APP_PAWR_MAX_RESPONSES_PER_EVENT;
    }

    /* Parse each response from the buffer */
    for (uint8_t i = 0; i < numResponses; i++)
    {
        /* Validate we have enough bytes for header */
        if (idx + APP_PAWR_RESPONSE_HEADER_SIZE > APP_PAWR_SUBEVENT_BUFFER_SIZE)
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                             "ERR: Parse overflow at rsp %d", i);
            return;
        }

        pResponses[i].txPower      = (int8_t)pBuffer[idx++];
        pResponses[i].rssi         = (int8_t)pBuffer[idx++];
        pResponses[i].cteType      = pBuffer[idx++];
        pResponses[i].responseSlot = pBuffer[idx++];
        pResponses[i].dataStatus   = pBuffer[idx++];
        pResponses[i].dataLen      = pBuffer[idx++];

        /* Validate data length */
        if (pResponses[i].dataLen > APP_PAWR_MAX_RESPONSE_DATA_LEN)
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                             "ERR: Response data too long %d", pResponses[i].dataLen);
            pResponses[i].dataLen = 0;
            pResponses[i].pData = NULL;
            continue;
        }

        /* Validate we have enough bytes for data */
        if (idx + pResponses[i].dataLen > APP_PAWR_SUBEVENT_BUFFER_SIZE)
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                             "ERR: Data overflow at rsp %d", i);
            pResponses[i].dataLen = 0;
            pResponses[i].pData = NULL;
            return;
        }

        pResponses[i].pData = &pBuffer[idx];

        /* Advance index by the response data length */
        idx += pResponses[i].dataLen;
    }
}