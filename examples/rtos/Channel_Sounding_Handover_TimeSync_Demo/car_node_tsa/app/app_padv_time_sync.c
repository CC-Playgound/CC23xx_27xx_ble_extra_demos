/******************************************************************************

@file  app_padv_time_sync.c

@brief This file contains the Periodic Advertising Time Synchronization module
       implementation for BLE applications.

The app_padv_time_sync module provides time synchronization using BLE periodic
advertising. It supports two roles:

Time Sync Advertiser (TSA):
- Creates periodic advertising to broadcast the local system time
- Other devices can sync to this node's time reference

Time Sync Observer (TSO):
- Listens for periodic advertising from a TSA
- Receives PADV events with absStartTime
- Updates app_time_sync with the synchronized time

Architecture:
- Uses BLEAppUtil as the abstraction layer for all BLE stack operations
- TSA: BLEAppUtil APIs for periodic advertising setup
- TSO: BLEAppUtil APIs for periodic scan sync creation
- Time updates flow through AppTimeSync_setTimeOffset()

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
#ifdef TIME_SYNC
//*****************************************************************************
//! Includes
//*****************************************************************************
#include "app_padv_time_sync.h"
#include "app_time_sync.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti/ble/host/gap/gap.h"
#include "ti/ble/host/gap/gap_scanner.h"
#include "ti_ble_config.h"
#include <ti/drivers/UART2.h>
#include <stdio.h>

//*****************************************************************************
//! Local Function Prototypes
//*****************************************************************************
static uint8_t *AppPadvTimeSync_findNameInAdvData(uint8_t *pData, uint16_t dataLen, uint8_t *pNameLen);
static uint8_t AppPadvTimeSync_matchTsaByName(BLEAppUtil_GapScan_Evt_AdvRpt_t *pAdvRpt);
static uint8_t AppPadvTimeSync_createSync(uint8_t AddrType, uint8_t *AdvAddress);
static uint8_t AppPadvTimeSync_scanStart(void);
static void AppPadvTimeSync_PeriodicEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Local Variables
//*****************************************************************************

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG ) )
/**
 * @brief Handle of the TSA extended advertising set; saved for terminated-event logging.
 */
static uint8_t gTsaAdvHandle = 0xFF;

static void AppPadvTimeSync_AdvEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

static BLEAppUtil_EventHandler_t appPadvTimeSyncAdvHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_ADV_TYPE,
    .pEventHandler  = AppPadvTimeSync_AdvEventHandler,
    .eventMask      = BLEAPPUTIL_ADV_SET_TERMINATED
};
#endif // HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG )

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
/**
 * @brief Current TSO state for name-based discovery
 */
//static AppPadvTimeSync_TsoState_t gTsoState = APP_PADV_TIME_SYNC_TSO_STATE_IDLE;

/**
 * @brief Copy of TSO configuration for use during discovery
 */
//static AppPadvTimeSync_TsoConfig_t gTsoConfig;

/**
 * @brief Sync handle from the last successful GapScan_PeriodicAdvCreateSync;
 *        used to terminate the sync in stopTso() when state is SYNCED.
 */
static uint16_t gSyncHandle = 0xFFFF;

/**
 * @brief Scan event handler for name-based TSA discovery
 */
/*
static BLEAppUtil_EventHandler_t appPadvTimeSyncScanHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_SCAN_TYPE,
    .pEventHandler  = AppPadvTimeSync_ScanEventHandler,
    .eventMask      = BLEAPPUTIL_SCAN_ENABLED  |
                      BLEAPPUTIL_SCAN_DISABLED |
                      BLEAPPUTIL_ADV_REPORT
};
*/

/**
 * @brief GAP Periodic event handler for periodic events
 *
 */
static BLEAppUtil_EventHandler_t appPadvTimeSyncPeriodicHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_PERIODIC_TYPE,
    .pEventHandler  = AppPadvTimeSync_PeriodicEvtHandler,
    .eventMask      = BLEAPPUTIL_SCAN_PERIODIC_ADV_SYNC_EST_EVENT_V1 |
                      BLEAPPUTIL_SCAN_PERIODIC_ADV_REPORT_EVENT_V1 |
                      BLEAPPUTIL_SCAN_PERIODIC_ADV_SYNC_LOST_EVENT
};

#endif // HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG )

//*****************************************************************************
//! Functions
//*****************************************************************************

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
/*********************************************************************
 * @fn      AppPadvTimeSync_findNameInAdvData
 *
 * @brief   Find the device name in advertisement data
 *
 *          Parses the advertising data to find either a complete or
 *          shortened local name AD type.
 *
 * @param   pData    - Pointer to advertising data
 * @param   dataLen  - Length of advertising data
 * @param   pNameLen - Output: length of the found name
 *
 * @return  Pointer to the name string (not null-terminated), or NULL if not found
 */
static uint8_t *AppPadvTimeSync_findNameInAdvData(uint8_t *pData, uint16_t dataLen, uint8_t *pNameLen)
{
    uint16_t idx = 0;
    *pNameLen = 0;

    while (idx < dataLen)
    {
        uint8_t adLen = pData[idx];

        // Check for invalid length or end of data
        if (adLen == 0 || idx + adLen >= dataLen)
        {
            break;
        }

        uint8_t adType = pData[idx + 1];

        // Check for complete or shortened local name
        if (adType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
            adType == GAP_ADTYPE_LOCAL_NAME_SHORT)
        {
            *pNameLen = adLen - 1;  // Exclude the type byte
            return &pData[idx + 2]; // Return pointer to name data
        }

        idx += adLen + 1;
    }

    return NULL;
}

/*********************************************************************
 * @fn      AppPadvTimeSync_matchTsaByName
 *
 * @brief   Check if an advertisement matches the target TSA name
 *
 * @param   pAdvRpt - Pointer to advertisement report
 *
 * @return  TRUE if name matches, FALSE otherwise
 */
/*
static uint8_t AppPadvTimeSync_matchTsaByName(BLEAppUtil_GapScan_Evt_AdvRpt_t *pAdvRpt)
{
    uint8_t nameLen = 0;
    uint8_t *pAdvName;
    uint8_t targetLen;

    // No advertising data in this report — the SID already matched, so
    // accept it. The TSA's extended advertising PDU often carries no data
    // (dl=0); waiting for a data-bearing report wastes scan cycles.
    if (pAdvRpt->pData == NULL || pAdvRpt->dataLen == 0)
    {
        return TRUE;
    }

    // Find the name in the advertisement data
    pAdvName = AppPadvTimeSync_findNameInAdvData(pAdvRpt->pData, pAdvRpt->dataLen, &nameLen);

    if (pAdvName == NULL || gTsoConfig.tsaName == NULL)
    {
        return FALSE;
    }

    // Compare name lengths
    targetLen = strlen((const char *)gTsoConfig.tsaName);
    if (nameLen != targetLen)
    {
        return FALSE;
    }
    // Compare name contents (exact match, case-sensitive)
    if (memcmp(pAdvName, gTsoConfig.tsaName, targetLen) == 0)
    {
        return TRUE;
    }

    return FALSE;
}
*/

/*********************************************************************
 * @fn      AppPadvTimeSync_createSync
 *
 * @brief   Create periodic advertising sync with discovered TSA address
 *
 *          Uses the address discovered during scanning to create a
 *          periodic advertising sync.
 *
 * @return  SUCCESS if sync creation initiated, error code otherwise
 */
/*
static uint8_t AppPadvTimeSync_createSync(uint8_t AddrType, uint8_t *AdvAddress)
{
    uint8_t status = SUCCESS;

    // Set up periodic advertising sync parameters
    GapScan_PeriodicAdvCreateSyncParams_t syncParams =
    {
        .options     = gTsoConfig.options,
        .advAddrType = AddrType,
        .skip        = gTsoConfig.skip,
        .syncTimeout = gTsoConfig.syncTimeout,
        .syncCteType = 0  // Accept all CTE types
    };

    // Copy discovered advertiser address
    memcpy(syncParams.advAddress, AdvAddress, B_ADDR_LEN);

    // Create periodic advertising sync
    status = GapScan_PeriodicAdvCreateSync(gTsoConfig.advSID, &syncParams);

    return status;
}
*/

/*********************************************************************
 * @fn      AppPadvTimeSync_scanStart
 *
 * @brief   Start scanning for TSA device
 *
 *
 * @return  SUCCESS if scan started, error code otherwise
 */
static uint8_t AppPadvTimeSync_scanStart(void)
{
    uint8_t status;

    BLEAppUtil_ScanInit_t scanInitParams =
    {
        .primPhy            = DEFAULT_SCAN_PHY,
        .scanType           = SCAN_TYPE_PASSIVE,
        .scanInterval       = APP_PADV_TIME_SYNC_INTERVAL,
        .scanWindow         = APP_PADV_TIME_SYNC_SCAN_WINDOW,
        .advReportFields    = SCAN_ADVRPT_FLD_ADDRTYPE | SCAN_ADVRPT_FLD_ADDRESS |
                            SCAN_ADVRPT_FLD_ADVSID | SCAN_ADVRPT_FLD_RSSI |
                            SCAN_ADVRPT_FLD_DATALEN,
        .scanPhys           = DEFAULT_SCAN_PHY,
        .fltPolicy          = SCANNER_FILTER_POLICY,
        .fltPduType         = SCAN_FLT_PDU_COMPLETE_ONLY,
        .fltMinRssi         = SCANNER_FILTER_MIN_RSSI,
        .fltDiscMode        = SCANNER_FILTER_DISC_MODE,
        .fltDup             = SCANNER_DUPLICATE_FILTER
    };

    // Initialize scanner with parameters
    status = BLEAppUtil_scanInit(&scanInitParams);

    if (status == SUCCESS)
    {
        // Start scanning with unlimited params
        BLEAppUtil_ScanStart_t scanStartParams = {0, 0, 0};

        status = BLEAppUtil_scanStart(&scanStartParams);
    }
    return status;
}

/*********************************************************************
 * @fn      AppPadvTimeSync_ScanEventHandler
 *
 * @brief   Scan Event Handler for TSO name-based discovery
 *
 *          Handles GAP scan events during TSA discovery:
 *          - BLEAPPUTIL_ADV_REPORT: Check if advertisement matches TSA name
 *          - BLEAPPUTIL_SCAN_ENABLED: Update state to scanning
 *          - BLEAPPUTIL_SCAN_DISABLED: Retry with aggressive params if not found
 *
 * @param   event    - GAP scan event type
 * @param   pMsgData - Pointer to event-specific data
 *
 * @return  none
 */
 /*
void AppPadvTimeSync_ScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    BLEAppUtil_ScanEventData_t *scanMsg = (BLEAppUtil_ScanEventData_t *)pMsgData;

    switch (event)
    {
        case BLEAPPUTIL_ADV_REPORT:
        {
            // Count every report regardless of state — log every 50 to confirm
            // the scanner is actually receiving extended advertising packets.
            // If this counter never moves while state=SCANNING the problem is
            // upstream (TSA not advertising, or controller not delivering reports).
            {
                static uint32_t sRptCount = 0;
                sRptCount++;
                if ((sRptCount % 50) == 0)
                {
                    extern UART2_Handle uart;
                    char b[40] = {0};
                    sprintf(b, "PADV_rpt_cnt=%lu\r\n", (unsigned long)sRptCount);
                    UART2_write(uart, b, sizeof(b), NULL);
                }
            }

            // Only process if we're actively scanning for TSA
            if (gTsoState != APP_PADV_TIME_SYNC_TSO_STATE_SCANNING)
            {
                break;
            }

            BLEAppUtil_GapScan_Evt_AdvRpt_t *pAdvRpt = &scanMsg->pBuf->pAdvReport;

            if (gTsoConfig.advSID == pAdvRpt->advSid)
            {
                extern UART2_Handle uart;
                char b[64] = {0};
                sprintf(b, "PADV_sid_match dl=%u pd=%d et=0x%x\r\n",
                        (unsigned)pAdvRpt->dataLen, (pAdvRpt->pData != NULL) ? 1 : 0,
                        (unsigned)pAdvRpt->evtType);
                UART2_write(uart, b, sizeof(b), NULL);

                if (AppPadvTimeSync_matchTsaByName(pAdvRpt))
                {
                    uint8_t syncSt = AppPadvTimeSync_createSync(pAdvRpt->addrType, pAdvRpt->addr);
                    if (syncSt == SUCCESS)
                    {
                        gTsoState = APP_PADV_TIME_SYNC_TSO_STATE_SYNCING;
                        UART2_write(uart, "PADV_syncing\r\n", 14, NULL);
                    }
                    else
                    {
                        char b[48] = {0};
                        sprintf(b, "PADV_cs_fail st=%u\r\n", (unsigned)syncSt);
                        UART2_write(uart, b, sizeof(b), NULL);
                    }
                }
                else
                {
                    char b[80] = {0};
                    uint8_t d0 = 0, d1 = 0, d2 = 0, d3 = 0;
                    if (pAdvRpt->pData != NULL && pAdvRpt->dataLen >= 4)
                    {
                        d0 = pAdvRpt->pData[0]; d1 = pAdvRpt->pData[1];
                        d2 = pAdvRpt->pData[2]; d3 = pAdvRpt->pData[3];
                    }
                    sprintf(b, "PADV_nm_fail dl=%u et=0x%x b=%02x%02x%02x%02x\r\n",
                            (unsigned)pAdvRpt->dataLen, (unsigned)pAdvRpt->evtType,
                            d0, d1, d2, d3);
                    UART2_write(uart, b, sizeof(b), NULL);
                }
            }
            break;
        }

        case BLEAPPUTIL_SCAN_ENABLED:
        {
            // Only transition to SCANNING if PADV initiated the scan (from IDLE).
            // Avoid overwriting SYNCED state when Central starts its own scan.
            if (gTsoState == APP_PADV_TIME_SYNC_TSO_STATE_IDLE)
            {
                gTsoState = APP_PADV_TIME_SYNC_TSO_STATE_SCANNING;
                {
                    extern UART2_Handle uart;
                    char b[40] = {0};
                    sprintf(b, "PADV_scan_en st=%u\r\n", (unsigned)gTsoState);
                    UART2_write(uart, b, sizeof(b), NULL);
                }
            }
            else
            {
                extern UART2_Handle uart;
                char b[40] = {0};
                sprintf(b, "PADV_scan_en st=%u (no change)\r\n", (unsigned)gTsoState);
                UART2_write(uart, b, sizeof(b), NULL);
            }
            break;
        }

        case BLEAPPUTIL_SCAN_DISABLED:
        {
            // Only restart scan when still searching (SCANNING state).
            // Do NOT restart when SYNCING — createSync is pending in the
            // controller; calling scanInit here cancels the pending sync.
            if (gTsoState == APP_PADV_TIME_SYNC_TSO_STATE_SCANNING)
            {
                extern UART2_Handle uart;
                char b[40] = {0};
                sprintf(b, "PADV_scan_disabled st=%u -> restarting\r\n", (unsigned)gTsoState);
                UART2_write(uart, b, sizeof(b), NULL);
                AppPadvTimeSync_scanStart();
            }
            else
            {
                extern UART2_Handle uart;
                char b[40] = {0};
                sprintf(b, "PADV_scan_disabled st=%u (no restart)\r\n", (unsigned)gTsoState);
                UART2_write(uart, b, sizeof(b), NULL);
            }
            break;
        }

        default:
            break;
    }
}
*/

/*********************************************************************
 * @fn      AppPadvTimeSync_PeriodicEvtHandler
 *
 * @brief   GAP Periodic Event Handler for Time Sync
 *
 *          Handles periodic advertising sync established events through
 *          the standard GAP event path. This replaces the direct LL
 *          callback (BLEAppUtil_registerPadvSyncEstCallback) and provides:
 *          - Standard BLE event pattern (same as other GAP events)
 *          - More information (status, SID, address, PHY, interval)
 *          - Consistent architecture (no special callback path from LL)
 *
 * @param   event    - GAP periodic event type
 * @param   pMsgData - Pointer to event-specific data
 *
 * @return  none
 */
/*
static void AppPadvTimeSync_PeriodicEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch (event)
    {
        case BLEAPPUTIL_SCAN_PERIODIC_ADV_SYNC_EST_EVENT_V1:
        {
            GapScan_Evt_PeriodicAdvSyncEstV1_t *pEvt = (GapScan_Evt_PeriodicAdvSyncEstV1_t *)pMsgData;

            {
                extern UART2_Handle uart;
                char b[48] = {0};
                sprintf(b, "PADV_sync_est st=%u sh=%u\r\n",
                        (unsigned)pEvt->status, (unsigned)pEvt->syncHandle);
                UART2_write(uart, b, sizeof(b), NULL);
            }

            // Only process successful sync establishment
            if (pEvt->status == SUCCESS)
            {
                if (gTsoState == APP_PADV_TIME_SYNC_TSO_STATE_SYNCING)
                {
                    gTsoState = APP_PADV_TIME_SYNC_TSO_STATE_SYNCED;

                    // Stop scanning - no longer needed after sync is established
                    BLEAppUtil_scanStop();

                    // Provide the LL PADV TS the sync handle created
                    AppPadvTimeSync_start(pEvt->syncHandle);
                }
                else
                {
                    // Stale SYNC_EST arrived despite CreateSyncCancel (or cancel
                    // was not called). Terminate the orphaned handle immediately.
                    uint8_t termSt = GapScan_PeriodicAdvTerminateSync(pEvt->syncHandle);
                    extern UART2_Handle uart;
                    char b[56] = {0};
                    sprintf(b, "PADV_stale_est sh=%u st=%u tsoSt=%u\r\n",
                            (unsigned)pEvt->syncHandle, (unsigned)termSt,
                            (unsigned)gTsoState);
                    UART2_write(uart, b, sizeof(b), NULL);
                }
            }
            else
            {
                extern UART2_Handle uart;
                char b[48] = {0};
                sprintf(b, "PADV_sync_fail status=%u tsoSt=%u\r\n",
                        (unsigned)pEvt->status, (unsigned)gTsoState);
                UART2_write(uart, b, sizeof(b), NULL);

                if (gTsoState == APP_PADV_TIME_SYNC_TSO_STATE_SYNCING)
                {
                    // Normal sync fail (not cancelled by stopTso): restart scan.
                    // BLEAppUtil_scanStop() is async; SCAN_DISABLED fires when the
                    // controller confirms the stop. The SCAN_DISABLED handler checks
                    // gTsoState == SCANNING and calls AppPadvTimeSync_scanStart().
                    gTsoState = APP_PADV_TIME_SYNC_TSO_STATE_SCANNING;
                    BLEAppUtil_scanStop();
                }
                // else state == IDLE: stopTso already cleaned up; the controller
                // fired this SYNC_FAIL in response to our CreateSyncCancel. The app
                // scan will trigger SCAN_ENABLED which transitions IDLE→SCANNING.
            }
            break;
        }

        case BLEAPPUTIL_SCAN_PERIODIC_ADV_REPORT_EVENT_V1:
        {
            GapScan_Evt_PeriodicAdvRptV1_t *pEvt = (GapScan_Evt_PeriodicAdvRptV1_t *)pMsgData;

            // The first 4 bytes of pData contain the time offset
            if (pEvt->pData != NULL && pEvt->dataLen >= 4)
            {
                uint32_t offset = BUILD_UINT32(pEvt->pData[0], pEvt->pData[1], pEvt->pData[2], pEvt->pData[3]);
                AppTimeSync_setTimeOffset(offset);
            }
            break;
        }

        case BLEAPPUTIL_SCAN_PERIODIC_ADV_SYNC_LOST_EVENT:
        {
            // Sync lost - restart scanning to find TSA again.
            // After a successful SYNC_EST the controller stops the scanner
            // automatically (SCAN_DISABLED st=3 fires). When SYNC_LOST fires
            // the scan is therefore already stopped, so we call scanStart()
            // directly. Calling scanStop() first would be a no-op and
            // SCAN_DISABLED would never fire, leaving the TSO stuck in
            // SCANNING state with no active scan.
            gTsoState = APP_PADV_TIME_SYNC_TSO_STATE_SCANNING;
            gSyncHandle = 0xFFFF;
            {
                extern UART2_Handle uart;
                char b[40] = {0};
                sprintf(b, "PADV_sync_lost st=%u\r\n", (unsigned)gTsoState);
                UART2_write(uart, b, sizeof(b), NULL);
            }
            AppPadvTimeSync_scanStart();
            break;
        }

        default:
            break;
    }
}
*/

/*
void AppPadvTimeSync_stopTso(void)
{
    extern UART2_Handle uart;
    char b[32] = {0};
    sprintf(b, "PADV_stopTso st=%u\r\n", (unsigned)gTsoState);
    UART2_write(uart, b, sizeof(b), NULL);

    if (gTsoState == APP_PADV_TIME_SYNC_TSO_STATE_SYNCED)
    {
        // Terminate the active sync so the controller stops delivering
        // PADV reports and SYNC_LOST events after the handover.
        GapScan_PeriodicAdvTerminateSync(gSyncHandle);
        gSyncHandle = 0xFFFF;
    }
    else if (gTsoState == APP_PADV_TIME_SYNC_TSO_STATE_SYNCING)
    {
        // Cancel the pending createSync so the controller never fires a stale
        // SYNC_EST. Without the cancel, the stale SYNC_EST arrives after the
        // next handover's createSync has re-used handle 0, and the subsequent
        // TerminateSync(0) call terminates the WRONG (new) sync, corrupting
        // the controller's extended-scan state so no further SID_MATCH fires.
        // GapScan_PeriodicAdvCreateSyncCancel() has an SDK macro bug:
        // the macro uses ##__VA_ARGS__ leaving a trailing comma when called
        // with no args. Pass a dummy 0 argument to satisfy the preprocessor;
        // the underlying icall dispatcher ignores it for a zero-param function.
        uint8_t cancelSt = GapScan_PeriodicAdvCreateSyncCancel(0);
        {
            extern UART2_Handle uart;
            char b[48] = {0};
            sprintf(b, "PADV_createSync_cancel st=%u\r\n", (unsigned)cancelSt);
            UART2_write(uart, b, sizeof(b), NULL);
        }
        // Also stop the scan; the controller may still be running it.
        BLEAppUtil_scanStop();
    }
    else if (gTsoState == APP_PADV_TIME_SYNC_TSO_STATE_SCANNING)
    {
        BLEAppUtil_scanStop();
    }
    gTsoState = APP_PADV_TIME_SYNC_TSO_STATE_IDLE;
}
*/

/*********************************************************************
 * @fn      AppPadvTimeSync_start
 *
 * @brief   Save syncHandle to LL to send to application the expected data
 *
 *          BLEAppUtil_startTSO internally handles:
 *          1. Enabling PADV time sync by saving the required syncHandle
 *
 * @return  SUCCESS if started successful
 */
uint8_t AppPadvTimeSync_start(uint16_t syncHandle)
{
    gSyncHandle = syncHandle;
    return BLEAppUtil_startTSO(syncHandle);
}
#endif // HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG )

/*********************************************************************
 * @fn      AppPadvTimeSync_onCnEstablished
 *
 * @brief   Notify PADV TSA that a CN connection was just established.
 *
 *          Handover_StartCN() disrupts the controller's PADV time sync
 *          state so absStartTime is no longer embedded in periodic
 *          advertising packets. Re-enable it after every CN establishment
 *          so the TSO always receives valid clock data on the next cycle.
 *
 * @return  none
 */
uint8_t Handoverstatus;
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG ) )
void AppPadvTimeSync_onCnEstablished(void)
{
    if (gTsaAdvHandle != 0xFF)
    {
        // Handover_StartCN() can disrupt the extended advertising set.
        // Stop first (ignore errors — it may already be stopped), then
        // restart so TSO can rediscover the TSA beacon.
        BLEAppUtil_advStop(gTsaAdvHandle);

        BLEAppUtil_AdvStart_t advStartInfo = {
            .enableOptions       = BLEAPPUTIL_ADV_START_ENABLE_OPTIONS_USE_MAX,
            .durationOrMaxEvents = 0
        };

        extern UART2_Handle uart;
        char c[40] = {0};
        Handoverstatus = BLEAppUtil_advStart(gTsaAdvHandle, &advStartInfo);
        sprintf(c, "TSA adv start st=%u\r\n", (unsigned)Handoverstatus);
        UART2_write(uart, c, sizeof(c), NULL);

        bStatus_t status = BLEAppUtil_setPeriodicAdvTimeSyncEnable(gTsaAdvHandle);
        //extern UART2_Handle uart;
        char b[40] = {0};
        sprintf(b, "TSA timeSyncEn st=%u\r\n", (unsigned)status);
        UART2_write(uart, b, sizeof(b), NULL);

    }
}
#endif // HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG )

/*********************************************************************
 * @fn      AppPadvTimeSync_initTsa
 *
 * @brief   Initialize Time Sync Advertiser (TSA)
 *
 *          Sets up periodic advertising for broadcasting time:
 *          1. Creates an extended advertising set (non-connectable, non-scannable)
 *          2. Sets periodic advertising parameters
 *          3. Enables periodic advertising
 *
 * @param   pConfig - Pointer to TSA configuration
 *
 * @return  SUCCESS if initialization successful
 * @return  Error code otherwise
 */
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG ) )
static uint8_t AppPadvTimeSync_initTsa(AppPadvTimeSync_TsaConfig_t *pConfig)
{
    uint8_t status;
    uint8_t advHandle;

    // Validate that tsaName is provided (required for TSO to discover this TSA)
    if (pConfig->tsaName == NULL)
    {
        return FALSE;
    }

    // Build advertising data with complete local name for TSO discovery
    // Format: [length][type=0x09][name bytes...]
    uint8_t nameLen = strlen((const char *)pConfig->tsaName);
    static uint8_t advData[32];  // Max 31 bytes of advertising data
    uint8_t advDataLen = 0;

    if (nameLen > 0 && nameLen <= 29)  // 29 = 31 - 2 (length + type bytes)
    {
        advData[advDataLen++] = nameLen + 1;                    // Length (name + type)
        advData[advDataLen++] = GAP_ADTYPE_LOCAL_NAME_COMPLETE; // Type: Complete Local Name
        memcpy(&advData[advDataLen], pConfig->tsaName, nameLen);
        advDataLen += nameLen;
    }

    // Create extended advertising parameters for periodic advertising
    // Must be non-connectable and non-scannable for periodic advertising
    BLEAppUtil_AdvParams_t advParams =
    {
        .eventProps = pConfig->periodicAdvProp,                          // Non-connectable, non-scannable (required for periodic adv)
        .primIntMin = pConfig->periodicAdvIntervalMin,
        .primIntMax = pConfig->periodicAdvIntervalMax,
        .primChanMap = BLEAPPUTIL_ADV_CHAN_ALL,
        .peerAddrType = BLEAPPUTIL_PEER_ADDRTYPE_PUBLIC_OR_PUBLIC_ID,
        .peerAddr = { 0, 0, 0, 0, 0, 0 },
        .filterPolicy = BLEAPPUTIL_ADV_AL_POLICY_ANY_REQ,
        .txPower = BLEAPPUTIL_ADV_TX_POWER_NO_PREFERENCE,
        .primPhy = BLEAPPUTIL_ADV_PRIM_PHY_1_MBPS,
        .secPhy = BLEAPPUTIL_ADV_SEC_PHY_1_MBPS,
        .sid = pConfig->advSID,
        .zeroDelay = 0
    };

    // Initialize advertising set info with device name for TSO discovery
    BLEAppUtil_AdvInit_t advInitInfo =
    {
        .advDataLen = advDataLen,
        .advData = advData,
        .scanRespDataLen = 0,
        .scanRespData = NULL,
        .advParam = &advParams
    };

    // Create the advertising set using BLEAppUtil
    status = BLEAppUtil_initAdvSet(&advHandle, &advInitInfo);

    if (status != SUCCESS)
    {
        return status;
    }

    // Save handle globally so the terminated-event handler can identify it
    gTsaAdvHandle = advHandle;

    // Set periodic advertising parameters
    BLEAppUtil_PeriodicAdvParams_t periodicParams =
    {
        .periodicAdvIntervalMin = pConfig->periodicAdvIntervalMin,
        .periodicAdvIntervalMax = pConfig->periodicAdvIntervalMax,
        .periodicAdvProp = pConfig->periodicAdvProp
    };

    status = BLEAppUtil_setPeriodicAdvParams(advHandle, &periodicParams);

    if (status != SUCCESS)
    {
        return status;
    }

    // Set periodic advertising data (required before enabling periodic advertising)
    // The actual time data will be populated by the controller when time sync is enabled
    BLEAppUtil_PeriodicAdvData_t periodicAdvData =
    {
        .operation = 0x03,   // GAPADV_PERIODIC_ADV_DATA_COMPLETE
        .dataLength = 0,
        .pData = NULL
    };

    status = BLEAppUtil_setPeriodicAdvData(advHandle, &periodicAdvData);

    if (status != SUCCESS)
    {
        return status;
    }

    // Enable extended advertising first (required before enabling periodic)
    BLEAppUtil_AdvStart_t advStartInfo =
    {
        .enableOptions = BLEAPPUTIL_ADV_START_ENABLE_OPTIONS_USE_MAX,
        .durationOrMaxEvents = 0
    };

    status = BLEAppUtil_advStart(advHandle, &advStartInfo);

    if (status != SUCCESS)
    {
        return status;
    }

    // Enable periodic advertising
    status = BLEAppUtil_setPeriodicAdvEnable(1, advHandle);

    if (status != SUCCESS)
    {
        return status;
    }

    // Enable TSA time sync to include absStartTime in periodic advertising data
    status = BLEAppUtil_setPeriodicAdvTimeSyncEnable(advHandle);

    if (status != SUCCESS)
    {
        return status;
    }

    // Register handler to detect if the advertising set is terminated
    status = BLEAppUtil_registerEventHandler(&appPadvTimeSyncAdvHandler);

    return status;
}
#endif // HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG )

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG ) )
static void AppPadvTimeSync_AdvEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    if (event == BLEAPPUTIL_ADV_SET_TERMINATED)
    {
        BLEAppUtil_AdvEventData_t *pAdvData = (BLEAppUtil_AdvEventData_t *)pMsgData;
        GapAdv_setTerm_t *pTerm = &pAdvData->pBuf->pSetTerm;
        extern UART2_Handle uart;
        char b[56] = {0};
        sprintf(b, "TSA_adv_term ah=%u st=%u conn=%u\r\n",
                (unsigned)pTerm->handle,
                (unsigned)pTerm->status,
                (unsigned)pTerm->connHandle);
        UART2_write(uart, b, sizeof(b), NULL);
    }else if (event == BLEAPPUTIL_ADV_END_AFTER_DISABLE) {
        while(1);
    
    }
}
#endif // HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG )

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
/*********************************************************************
 * @fn      AppPadvTimeSync_initTso
 *
 * @brief   Initialize Time Sync Observer (TSO)
 *
 *          Sets up name-based discovery to find a TSA and receive
 *          time updates:
 *          1. Registers scan event handler
 *          2. Starts scanning for TSA by advertised name
 *          3. When found, extracts address and creates periodic sync
 *          4. Time updates flow automatically via PADV event callback
 *
 * @param   pConfig - Pointer to TSO configuration (tsaName must be set)
 *
 * @return  SUCCESS if initialization successful
 * @return  INVALIDPARAMETER if tsaName is NULL
 * @return  Error code otherwise
 */
/*
static uint8_t AppPadvTimeSync_initTso(AppPadvTimeSync_TsoConfig_t *pConfig)
{
    uint8_t status;

    // Validate that tsaName is provided (name-based discovery is required)
    if (pConfig->tsaName == NULL)
    {
        return INVALIDPARAMETER;
    }

    // Register GAP Periodic event handler to receive sync established events
    // through the standard GAP event path (instead of direct LL callback)
    status = BLEAppUtil_registerEventHandler(&appPadvTimeSyncPeriodicHandler);

    if (status == SUCCESS)
    {
        // Store configuration for use during discovery
        memcpy(&gTsoConfig, pConfig, sizeof(AppPadvTimeSync_TsoConfig_t));
    
        // Reset state
        gTsoState = APP_PADV_TIME_SYNC_TSO_STATE_IDLE;
    
        // Register scan event handler for name-based discovery
        status = BLEAppUtil_registerEventHandler(&appPadvTimeSyncScanHandler);
    }


    if (status == SUCCESS)
    {
        // Start scanning
        status = AppPadvTimeSync_scanStart();
    }


    return status;
}
*/
#endif // HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG )

/*********************************************************************
 * @fn      AppPadvTimeSync_init
 *
 * @brief   Initialize the PADV Time Sync module
 *
 *          Initializes the module based on the specified role:
 *          - TSA: Sets up periodic advertising to broadcast time
 *          - TSO: Sets up periodic scanning to receive time updates
 *
 * @param   pConfig - Pointer to configuration structure
 *
 * @return  SUCCESS if initialization successful
 * @return  INVALIDPARAMETER if pConfig is NULL
 * @return  Error code otherwise
 */
uint8_t AppPadvTimeSync_init(AppPadvTimeSync_Config_t *pConfig)
{
    uint8_t status;

    if (pConfig == NULL)
    {
        return INVALIDPARAMETER;
    }

    // Initialize based on role
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG ) )
    if (pConfig->role == APP_PADV_TIME_SYNC_ROLE_TSA)
    {
        status = AppPadvTimeSync_initTsa(&pConfig->tsa);
    }
    else
#endif // HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG )
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
    if (pConfig->role == APP_PADV_TIME_SYNC_ROLE_TSO)
    {
        //status = AppPadvTimeSync_initTso(&pConfig->config.tso);
    }
    else
#endif // HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG )
    {
        status = INVALIDPARAMETER;
    }

    return status;
}
#endif // TIME_SYNC