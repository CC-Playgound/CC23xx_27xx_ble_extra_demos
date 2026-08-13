/******************************************************************************

@file  app_car_node.c

@brief This file contains the car node functionalities for the BLE application.
       It handles CS events and estimates distance using both initiator and
       reflector results using the CSTransceiver and CSProcess modules.

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2025-2026, Texas Instruments Incorporated
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

// The CHANNEL_SOUNDING define added to prevent warnings, because the file uses
// Channel sounding logic only
#ifdef CHANNEL_SOUNDING

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti_ble_config.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti/ble/host/cs/cs.h"
#include "app_car_node.h"
#include "app_cs_api.h"
#include "app_cs_transceiver_api.h"
#include "app_ranging_client_api.h"

#ifdef CONNECTION_HANDOVER
#include "app_handover_api.h"
bStatus_t HandoverServingStart(uint16_t connHandle);
#endif

#ifdef CS_MEASURE_DISTANCE
#include "ti/ble/app_util/framework/bleapputil_timers.h"
#include "ti/ble/controller/ll/ll_rat.h"
#include "ti/ble/stack_util/osal/osal_list.h"
#include "app_cs_process_api.h"
#include "app_btcs_api.h"
#endif


#include "app_peripheral_api.h"
#include "app_gatt_api.h"
#include "app_padv_time_sync.h"
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"
#include <ti/drivers/UART2.h>
#include <ti/drivers/dpl/ClockP.h>
#include "ti/ble/host/gap/gap.h"
#include "ti/ble/host/gap/gap_scanner.h"
#include "app_peripheral_api.h"
#include "app_central_api.h"
#include <string.h>

/*********************************************************************
 * MACROS
 */

// Swaps bytes for uint16
#define SWAP_BYTES16(a) (((uint32_t)a << 8) | ((a >> 8) & 0xFFFF))

// Compare first 12 bits
#define CAR_NODE_COMPARE_12_BITS(a, b)  ((((uint16_t) a) & 0xFFF) == (((uint16_t) b) & 0xFFF))

// Checks if a specific flag is set
#define CAR_NODE_STATE_IS_FLAG_SET(state, flag) ((state & ((uint8_t) flag)) == ((uint8_t) flag))

// Set state flag
#define CAR_NODE_STATE_SET_FLAG(state, flag) (state |= ((uint8_t) flag))

// Clears a state
#define CAR_NODE_STATE_CLEAR(state) (state = ((uint8_t) CAR_NODE_STATE_IDLE))

// Checks if the results are ready for a given source mode and state
#define CAR_NODE_IS_RESULTS_READY(sourceMode, state) \
    (((sourceMode == CS_RESULTS_MODE_LOCAL) && CAR_NODE_STATE_IS_FLAG_SET(state, CAR_NODE_STATE_LOCAL_DATA_READY)) || \
    ((sourceMode != CS_RESULTS_MODE_LOCAL && sourceMode < CS_RESULTS_MODE_END) && CAR_NODE_STATE_IS_FLAG_SET(state, CAR_NODE_STATE_REMOTE_DATA_READY)))

// Timeout threshold in seconds \ RAT ticks to remove a pending procedure from the procedures list.
#define CAR_NODE_TIMEOUT_THRESHOLD_S ((uint32_t) 3)         // Configure this value for tuning the timeout threshold.
#define CAR_NODE_TIMEOUT_THRESHOLD_TICKS ((uint32_t) (RAT_TICKS_IN_1S * CAR_NODE_TIMEOUT_THRESHOLD_S))

// Timeout in milliseconds of the the timer set for the procedures list cleanup.
// It is used as a watchdog for cases where the system doesn't get any new CS results event,
// therefore the procedures list cleanup won't be triggered.
// As long as there are procedures in the list, the timer will be running.
// Should be greater than the time of @ref CAR_NODE_TIMEOUT_THRESHOLD_S for performance reasons.
#define CAR_NODE_TIMER_TIMEOUT_MS (TIMER_SEC_TO_MS(CAR_NODE_TIMEOUT_THRESHOLD_S) * 2)

// Max size of procedure elements data linked list
// Used to control the procedure elements aggregation
#define CAR_NODE_PROCEDURE_LIST_MAX_SIZE 10U

//*****************************************************************************
//! Defines
//*****************************************************************************

// Number of connections supported
#define CAR_NODE_MAX_CONNS      MAX_NUM_BLE_CONNS

// Already done collecting the local subevent results or there are no expected results on the way
#define CAR_NODE_PROC_COUNTER_IDLE         ((uint32_t) 0xFFFFFFFF)

//*****************************************************************************
//! Typedefs
//*****************************************************************************

#ifdef CS_MEASURE_DISTANCE


// Measure Distance Modes
#define CAR_NODE_MEASURE_DISTANCE_MODE_DISTANCE_RAW 0x01

// Procedure element state flags. Used by @ref CarNode_procedureInfoElem_t
typedef enum
{
    CAR_NODE_STATE_IDLE                     =   (uint8_t)BV(0),   //!< Waiting for data to arrive
    CAR_NODE_STATE_LOCAL_DATA_READY         =   (uint8_t)BV(1),   //!< Finished collecting local data
    CAR_NODE_STATE_REMOTE_DATA_READY        =   (uint8_t)BV(2),   //!< Finished collecting remote data
    CAR_NODE_STATE_LOCAL_PROCEDURE_ABORTED  =   (uint8_t)BV(3),   //!< Local procedure was aborted
} CarNode_stateFlags_e;

// General element to hold CS results data for received from the local \ remote devices
typedef struct
{
    osal_list_elem elem;

    // CS results data, one of the following types:
    // @ref CarNode_resultsElemDataProp_t (for Local \ Prop)
    // @ref CarNode_resultsElemDataRAS_t (RAS)
    // @ref @ref CarNode_resultsElemDataBTCS_t (BTCS)
    uint8_t resultsData[];  //!< Pointer to the results
} CarNode_resultsElem_t;

// Generic callback function to free an element in a results list.
// This function is called when providing it as a parameter to @ref CarNode_clearResultsList.
// Consider defining it when a resultsData param in @ref CarNode_resultsElem_t
// contains dynamic memory allocations fields.
typedef void (*CarNode_freeResultsElemCB_t)(CarNode_resultsElem_t* pResultsElem);

// Element data for Local \ Prop CS results
typedef struct
{
    int8_t referencePowerLevel;     //!< Reference Power Level, should be between @ref CS_MIN_TX_POWER_VALUE and @ref CS_MAX_TX_POWER_VALUE.
                                    //!< Set this value to @ref CS_INVALID_TX_POWER if this parameter is not relevant
    int16_t frequencyCompensation;  //!< Frequency compensation (CFO) in units of 0.01 ppm. 0 for continuation events.
    uint8_t subeventDoneStatus;     //!< Status of the Subevent Done.
    uint8_t procedureDoneStatus;    //!< Status of the Procedure Done.
    uint8_t numAntennaPath;         //!< Number of antenna paths reported.
    uint8_t numStepsReported;       //!< Number of steps reported in the given subevent.
    uint16_t dataLen;               //!< Data length
    uint8_t data[];                 //!< Pointer to the subevent results steps.
} CarNode_resultsElemDataProp_t;

#ifdef RANGING_CLIENT
// Element data for CS results received from RAS profile
typedef struct
{
    RangingDBClient_procedureSegmentsReader_t segmentsReader; //!< Segments reader for the Ranging DB data.
} CarNode_resultsElemDataRAS_t;
#endif

// Element data for CS results received from a BTCS message
typedef struct {
    uint8_t subeventDoneStatus;     //!< Status of the Subevent Done.
    uint8_t procedureDoneStatus;    //!< Status of the Procedure Done.
    uint8_t numStepsReported;       //!< Number of steps reported in the given subevent.
    uint16_t dataLen;               //!< Data length
    uint8_t data[];                 //!< Pointer to the subevent results steps.
} CarNode_resultsElemDataBTCS_t;

// Procedure information element, should be part of a list of type @ref osal_list_list
typedef struct
{
    osal_list_elem elem;

    uint8_t state;      //!< Current state of the procedure. Bitmap of @ref CarNode_stateFlags_e flags.
    uint32_t timestamp; //!< Timestamp of the procedure start in system ticks.

    // Current procedure parameters, determined by @ref CS_SUBEVENT_RESULT and @ref CS_SUBEVENT_CONTINUE_RESULT
    uint16_t procedureCounter;  //!< Procedure counter of the current procedure. Only the 16 LSB bit matters.
    uint8_t configId;           //!< Config ID for this procedure (from first subevent result)

    // NOTE: Procedure parameters (role, timing params) are accessed via CS_GetConfiguration(connHandle, configId)
    // selectedTxPower, numAntPath, finalTsw are accessed from gSessionsDb[connHandle]

    // BTCS module parameters workspace
    uint8_t totalSubeventSteps;     //!< Total number of steps expected to be reported for the current subevent
    uint8_t subeventStepsProcessed; //!< Number of steps already processed for the current subevent
    uint8_t procedureDoneStatus;    //!< ProcedureDoneStatus to be saved as the first subevent message arrives (not subeventCont)
    uint8_t subeventDoneStatus;     //!< SubeventDoneStatus to be saved as the first subevent message arrives (not subeventCont)

    osal_list_list localResults;    //!< List of local results collected during the procedure. elem type: @ref CarNode_resultsElem_t.
    osal_list_list remoteResults;   //!< List of remote results collected during the procedure. elem type: @ref CarNode_resultsElem_t.
    ChannelSounding_resultsSourceMode_e remoteResultsMode; //!< Source of the remote results

    // Generic callback function to free an element in remoteResults list
    void (*pFreeRemoteResultsElemCB)(CarNode_resultsElem_t* pResultsElem);

} CarNode_procedureInfoElem_t;

// Procedure params for @ref CarNode_procElemAddResultsProp
typedef struct
{
    int8_t referencePowerLevel;                             //!< Reference Power Level used for the results.
    int16_t frequencyCompensation;                          //!< Frequency compensation, units: 0.01 ppm (local: measured; remote: 0)
    uint8_t procedureDoneStatus;                            //!< Procedure Done Status.
    uint8_t subeventDoneStatus;                             //!< Subevent Done Status.
    uint8_t numAntennaPath;                                 //!< Number of antenna paths reported in the subevent.
    uint8_t numStepsReported;                               //!< Number of steps reported in the subevent.
    uint16_t dataLen;                                       //!< Length of the data in bytes.
    uint8_t* pData;                                         //!< Pointer to the steps data of the subevent results.
} CarNode_procElemAddResultsPropParams_t;

// Session structure to hold a single connection CS results data
typedef struct
{
    // CS Process module parameters
    uint16_t sessionId;  //!< Indicates if there is an associated cs_process session

    // Procedure counter of the current procedure that is running in real time.
    // This is required because the SubeventResultsCont event raised by the local device doesn't
    // contain a procedure counter parameter.
    // It is used only to determine local procedure counter.
    uint32_t currProcedureCounter;

    // Current procedure Tx Power (dBm) used by the local device for the current CS procedure.
    // Used as a temporary parameter until the first subevent will arrive.
    // To be passed into CS Process module when initializing a procedure
    int8_t selectedTxPower;

    // Counter of procedure elements in the procedures list
    // Used to limit the size of the list while aggregating results
    uint8_t procedureListSize;

    // Number of antenna paths for current procedure (computed from ACI at procedure enable)
    uint8_t numAntPath;

    // Required for csCofing struct for BleCsRanging
    uint8_t aci;

    // Final antenna switching period (computed from gLocalTsw + remoteTsw + ACI at procedure enable)
    uint8_t finalTsw;

    // Remote device T_SW capability (cached per-connection from capability exchange)
    uint8_t remoteTsw;

    osal_list_list procedureList; //!< List of procedures currently active for this session. elem type: @ref CarNode_procedureInfoElem_t.
} CarNode_session;
#endif

//*****************************************************************************
//! Prototypes
//*****************************************************************************

void CarNode_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
void CarNode_handleEvents(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

#ifdef CS_MEASURE_DISTANCE

void CS_procedureStart(uint16_t connHandle);
void CarNode_invokeProcedureEnableCmd(uint16_t connHandle);

/********** Procedures list handlers **********/
void CarNode_clearProceduresList(osal_list_list* pProcedureList);
CarNode_procedureInfoElem_t* CarNode_getProcedureElem(uint16_t connHandle, uint16_t procedureCounter);
void CarNode_procElemRemove(uint16_t connHandle, CarNode_procedureInfoElem_t* pProcedureElem);
void CarNode_procElemFree(CarNode_procedureInfoElem_t *pProcedureElem);
void CarNode_clearResultsList(osal_list_list* pResultsList, CarNode_freeResultsElemCB_t freeCb);
void CarNode_clearTimeoutProcedures();
void CarNode_timerCB(BLEAppUtil_timerHandle timerHandle, BLEAppUtil_timerTermReason_e reason, void *pData);
bStatus_t CarNode_procElemAddResultsProp(CarNode_procedureInfoElem_t* pProcedureElem, ChannelSounding_resultsSourceMode_e resultsSourceMode, CarNode_procElemAddResultsPropParams_t* pParams);
#ifdef RANGING_CLIENT
void CarNode_freeRemoteResultsElemRAS(CarNode_resultsElem_t* pResultsElem);
bStatus_t CarNode_procElemAddResultsRAS(CarNode_procedureInfoElem_t* pProcedureElem, RangingDBClient_procedureSegmentsReader_t segmentsReader);
#endif // RANGING_CLIENT
void CarNode_procElemAddResultsPostProcess(bStatus_t status, uint16_t connHandle, ChannelSounding_resultsSourceMode_e resultsMode,
                                           CarNode_procedureInfoElem_t* pProcedureElem);
void CarNode_procElemHandleStateReady(uint16_t connHandle, CarNode_procedureInfoElem_t* pProcedureElem);

/********** CS Process Session handlers **********/
void CarNode_clearSession(uint16_t connHandle);
void CarNode_clearConfigurationParams(uint16_t connHandle);
csProcessStatus_e CarNode_CSProcessAddResults(uint16_t connHandle, ChannelSounding_resultsSourceMode_e resultsMode, osal_list_list* pResultsList, uint8_t role);
csProcessStatus_e  CarNode_CSProcessAddResultsProp(ChannelSounding_resultsSourceMode_e resultsMode, CarNode_resultsElemDataProp_t* pElemResultsData, uint8_t totalStepsInSubevent);
#ifdef RANGING_CLIENT
csProcessStatus_e CarNode_CSProcessAddResultsRAS(uint16_t connHandle, CarNode_resultsElemDataRAS_t* pElemResultsData, uint8_t peerRole);
void CarNode_sendRASDataToExtCtrl(uint16_t connHandle, uint8_t role, RangingDBClient_procedureSegmentsReader_t segmentsReader);
#endif // RANGING_CLIENT
bool CarNode_handleCSProcessAddResultsStatus(uint16_t connHandle, csProcessStatus_e status);
csProcessStatus_e CarNode_handleCSProcessDistancePendingStatus(uint16_t connHandle);
void CarNode_sendDistanceResultsError(uint16_t connHandle, bStatus_t status);
void CarNode_handleDistanceResults(uint16_t connHandle, CSProcess_Results_t distanceResults, bStatus_t status);

/********** BTCS related handles **********/
bStatus_t CarNode_procElemAddResultsBTCS(CarNode_procedureInfoElem_t* pProcedureElem, uint8_t* pSubeventData, uint16_t dataLen,
                                         uint8_t numStepsReported, uint8_t procedureDoneStatus, uint8_t subeventDoneStatus);
bStatus_t CarNode_procElemHandleBTCSSubevent(uint16_t connHandle, CarNode_procedureInfoElem_t* pProcedureElem, uint8_t* pSubeventHdr, uint16_t dataLen);
bStatus_t CarNode_procElemHandleBTCSSubeventCont(uint16_t connHandle, CarNode_procedureInfoElem_t* pProcedureElem, BTCS_SubeventContHdr_t* pSubeventContHdr, uint16_t dataLen);
bStatus_t CarNode_procElemHandleResultsBTCS(uint16_t connHandle, CarNode_procedureInfoElem_t* pProcedureElem, uint8_t msgId, uint8_t* pMsgData, uint16_t dataLen);
void CarNode_handleBTCSMsg(uint16_t connHandle, uint8_t msgId, uint8_t* pMsgData, uint16_t dataLen);
void CarNode_handleDKMsg(uint16_t connHandle, DK_Message_t* pDKMessage);

#endif // CS_MEASURE_DISTANCE

#ifdef RANGING_CLIENT
/********** RAS events handlers **********/
void CarNode_RREQ_EventHandler(uint16_t connHandle, uint16_t rangingCount, uint8_t rangingStatus, RangingDBClient_procedureSegmentsReader_t segmentsReader);
#endif // RANGING_CLIENT

/********** CS Procedure events handlers **********/
void CarNode_handleReadRemoteCapsComplete(ChannelSounding_readRemoteCapabEvent_t* pCsReadRemoteCapsEvt);
void CarNode_handleCsProcEnableComplete(ChannelSounding_procEnableComplete_t *pProcEnableCompleteEvt);
bool CarNode_handleCsSubeventResultsEvt(uint16_t connHandle, ChannelSounding_resultsSourceMode_e resultsSourceMode,
                                        ChannelSounding_subeventResults_t *pSubeventResultsEvt);
bool CarNode_handleCsSubeventResultsEvtContEvt(uint16_t connHandle, ChannelSounding_resultsSourceMode_e resultsSourceMode,
                                               uint16_t procedureCounter, ChannelSounding_subeventResultsContinue_t *pSubeventResultsContEvt);

/********** CS events handlers **********/
void CarNode_csEvtHandler(csEvtHdr_t *pCsEvt);
void CarNode_transceiverEventHandler(BLEAppUtil_eventHandlerType_e , uint32 , BLEAppUtil_msgHdr_t *);
void CarNode_connEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

/********** Scan / Adv / Connect **********/
static uint8_t CarNode_startScan(void);
static uint8_t CarNode_adv(void);
static uint8_t CarNode_advstop(void);
static void CarNode_connect(uint8_t peerAddrType, uint8_t *peerAddr, uint8_t phy);
void CarNode_handleScanEvent(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData);


//*****************************************************************************
//! Globals
//*****************************************************************************

// Events handler struct for connection events
BLEAppUtil_EventHandler_t carNodeConnHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_CONN_TYPE,
    .pEventHandler  = CarNode_connEventHandler,
    .eventMask      = BLEAPPUTIL_LINK_ESTABLISHED_EVENT |
                      BLEAPPUTIL_LINK_TERMINATED_EVENT
};

//! The external event handler
ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

// UART handle for sending state and results to host computer
UART2_Handle uart;

#define HANDOVER_SERVING_NODE 1

#define KEY_NODE_NAME   "Key Node 0216"
#define PEER_NAME_LEN   (sizeof(KEY_NODE_NAME) - 1)

#define ADV_FIELD_LEN_OFFSET  0
#define ADV_FIELD_TYPE_OFFSET 1
#define ADV_FIELD_DATA_OFFSET 2


// Connection handle tracking (required by CarNode_invokeProcedureEnableCmd)
uint16_t cyclic_handle = 0xFFFF;
// Connection role (GAP_PROFILE_CENTRAL or GAP_PROFILE_PERIPHERAL)
uint8_t connectionRole = 0;

// ClockP watchdog timeout in ticks (1-second period, re-armed each CS activity)
#define CAR_NODE_WATCHDOG_TIMEOUT_MS    (TIMER_SEC_TO_MS(1))
#define CAR_NODE_WATCHDOG_TIMEOUT_TICKS ((CAR_NODE_WATCHDOG_TIMEOUT_MS * 1000UL) / ClockP_getSystemTickPeriod())


// CS procedure parameters configured at startup
static CS_setProcedureParamsCmdParams_t csSetProcedureParams = {
    .configID = 0,
    .maxProcedureDur = 0xFFFF,
    .minProcedureInterval = 0,
    .maxProcedureInterval = 0,
    .maxProcedureCount = 1,
    .minSubEventLen = 0xD6D8,//10000,//0xD6D8,
    .maxSubEventLen = 0xD6D8,//10000,//0xD6D8,
    .aci = 7,
    .phy = 1,
    .txPwrDelta = 0x00,
    .preferredPeerAntenna = 0b0011,
    .snrCtrlI = 0xFF,
    .snrCtrlR = 0xFF,
    .enable = 0,
};

static CS_createConfigCmdParams_t csConfigParams = {
    .configID = 0,
    .createContext = 1,
    .mainMode = 2,
    .subMode = 0xFF,
    .mainModeMinSteps = 0,
    .mainModeMaxSteps = 0,
    .mainModeRepetition = 0,
    .modeZeroSteps = 3,
    .role = 0,
    .rttType = 0,
    .csSyncPhy = 1,
    .channelMap = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    .chMRepetition = 1,
    .chSel = 0,
    .ch3cShape = 0,
    .ch3CJump = 0,
};

static CS_setDefaultSettingsCmdParams_t csDefaultSettings = {
    .roleEnable = 3,
    .csSyncAntennaSelection = 1,
    .maxTxPower = 10,
};

#ifdef CS_MEASURE_DISTANCE

// Set the Measure Distance mode
#if CS_MEASURE_DISTANCE == CAR_NODE_MEASURE_DISTANCE_MODE_DISTANCE_RAW
#define CAR_NODE_SEND_SUBEVENT_TO_EXT_CTRL TRUE
#else
#define CAR_NODE_SEND_SUBEVENT_TO_EXT_CTRL FALSE
#endif

// Sessions DB
CarNode_session gSessionsDb[CAR_NODE_MAX_CONNS];

// Timer handle for clearing the procedures list
BLEAppUtil_timerHandle gTimerHandle = BLEAPPUTIL_TIMER_INVALID_HANDLE;

// Local device T_SW capability (device-level, read once at capability exchange)
static uint8_t gLocalTsw = 0;

//*****************************************************************************
//! Functions
//*****************************************************************************

// CS Procedure State Machine — prevents double-scheduling and race conditions
typedef enum
{
    CS_PROC_STATE_IDLE,             // Ready to schedule a new procedure
    CS_PROC_STATE_ACTIVE,           // Procedure submitted / running
    CS_PROC_STATE_WAITING_RESULTS   // Procedure ended, awaiting distance results
} CarNode_CsProcState_e;

static CarNode_CsProcState_e gCsProcState = CS_PROC_STATE_IDLE;
static bool     gDistanceProcessed  = false;
static uint16_t gProcedureRepeatCount = 0;

// ClockP-based watchdog timer (separate from the BLEAppUtil timeout-cleanup timer)
static ClockP_Struct gCsWatchdogClockStruct;
static ClockP_Handle gCsWatchdogHandle = NULL;
uint16_t gTimerParamConnHandle = LL_INVALID_CONNECTION_ID;

#endif // CS_MEASURE_DISTANCE

// ============================================================================
// UART helper functions
// ============================================================================

static void uartInit(void)
{
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 3000000;
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) { while (1) {} }
}

// ============================================================================
// CS scheduling: state machine helpers and procedure enable
// ============================================================================

uint8_t CarNode_getConfiguredCsRole(void)
{
    return csConfigParams.role;
}

void csProcedureRestart(uint16_t connHandle)
{
    if (gCsProcState != CS_PROC_STATE_IDLE)
    {
        return;
    }
    CarNode_invokeProcedureEnableCmd(connHandle);
}

void CarNode_stopCsScheduling(void)
{
    cyclic_handle = 0xFFFF;
    if (gCsWatchdogHandle != NULL)
    {
        ClockP_stop(gCsWatchdogHandle);
    }
}

void CS_procedureStart(uint16_t connHandle)
{
    ChannelSounding_setProcedureEnableCmdParams_t pParams = {
        .connHandle = connHandle,
        .configID = 0,
        .enable = 1
    };
    ChannelSounding_procedureEnable(&pParams);
}

#ifdef CS_MEASURE_DISTANCE
void CarNode_setCsProcStateActive(void)
{
    gCsProcState       = CS_PROC_STATE_ACTIVE;
    gDistanceProcessed = false;
}
#endif

static void CarNode_timerRecoveryHandler(char *pData)
{
    CarNode_clearTimeoutProcedures();
    if (gCsProcState != CS_PROC_STATE_ACTIVE)
    {
        gCsProcState = CS_PROC_STATE_IDLE;
        csProcedureRestart(gTimerParamConnHandle);
    }
    else
    {
        gProcedureRepeatCount = 0;
    }
    ClockP_setTimeout(gCsWatchdogHandle, CAR_NODE_WATCHDOG_TIMEOUT_TICKS);
    ClockP_start(gCsWatchdogHandle);
}

static void CarNode_timerClockFxn(uintptr_t arg)
{
    BLEAppUtil_invokeFunction(CarNode_timerRecoveryHandler, NULL);
}

void CarNode_invokeProcedureEnableCmd(uint16_t connHandle)
{
    if (cyclic_handle != 0xFFFF)
    {
        csSetProcedureParams.connHandle = cyclic_handle;
        csStatus_e status = CS_SetProcedureParameters(&csSetProcedureParams);

        if (status == CS_STATUS_SUCCESS)
        {
            CS_setProcedureEnableCmdParams_t params;
            params.connHandle = csSetProcedureParams.connHandle;
            params.configID   = csSetProcedureParams.configID;
            params.enable     = TRUE;

            status = CS_ProcedureEnable(&params);
            
        }

        if (status == CS_STATUS_SUCCESS)
        {
            gCsProcState       = CS_PROC_STATE_ACTIVE;
            gDistanceProcessed = false;
        }
        else if (status == CS_STATUS_COMMAND_DISALLOWED)
        {
            // Procedure already running at the controller — no results produced, leave state unchanged.
        }
        else
        {
            gCsProcState = CS_PROC_STATE_IDLE;
        }
    }
}

// ============================================================================
// Scan / Adv / Connect functions
// ============================================================================

static uint8_t CarNode_startScan(void)
{
    Central_scanStartCmdParams_t scanParams;
    scanParams.scanPeriod = 0;
    scanParams.scanDuration = 0;
    scanParams.maxNumReport = 0;
    return Central_scanStart(&scanParams);
}

static uint8_t CarNode_adv(void)
{
    Peripheral_advStartCmdParams_t pParams;
    pParams.advHandle = 0;
    pParams.durationOrMaxEvents = GAP_ADV_ENABLE_OPTIONS_USE_MAX;
    pParams.enableOptions = 0;
    return Peripheral_advStart(&pParams);
}

static uint8_t CarNode_advstop(void)
{
    uint8_t advHandle = 0;
    return Peripheral_advStop(&advHandle);
}

static void CarNode_connect(uint8_t peerAddrType, uint8_t *peerAddr, uint8_t phy)
{
    Central_connectCmdParams_t connParams;
    connParams.addrType = peerAddrType;
    memcpy(connParams.addr, peerAddr, B_ADDR_LEN);
    connParams.phy = phy;
    connParams.timeout = 3000;
    Central_connect(&connParams);
}

void CarNode_handleScanEvent(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
    if ( event == BLEAPPUTIL_ADV_REPORT )
    {
        BLEAppUtil_ScanEventData_t *pScanEvt = (BLEAppUtil_ScanEventData_t *)pMsgData;
        GapScan_data_t *pScanData = (GapScan_data_t *)(pScanEvt->pBuf);
        BLEAppUtil_GapScan_Evt_AdvRpt_t *pAdvRpt = (BLEAppUtil_GapScan_Evt_AdvRpt_t *)(&(pScanData->pAdvReport));
        uint8_t *pAdvData = (uint8_t *)(pAdvRpt->pData);
        uint16_t dataLen = pAdvRpt->dataLen;
        uint8_t fieldLen = 0;
        uint8_t fieldType;

        if ( (pAdvRpt->evtType & ADV_RPT_EVT_TYPE_CONNECTABLE) != 0 )
        {
            while ( dataLen != 0 )
            {
                fieldLen  = pAdvData[ADV_FIELD_LEN_OFFSET];
                fieldType = pAdvData[ADV_FIELD_TYPE_OFFSET];

                if ( ( fieldType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ) &&
                     ( PEER_NAME_LEN == fieldLen - 1 ) )
                {
                    if ( memcmp(&pAdvData[ADV_FIELD_DATA_OFFSET], (uint8_t *)KEY_NODE_NAME, PEER_NAME_LEN) == 0 )
                    {
                        Central_scanStop();
                        CarNode_connect(pAdvRpt->addrType, pAdvRpt->addr, pAdvRpt->primPhy);
                    }
                }
                dataLen = dataLen - (fieldLen + 1);
                if ( dataLen != 0 )
                {
                    pAdvData = pAdvData + (fieldLen + 1);
                }
            }
        }
    }
#endif
}

/*******************************************************************************
 * This API starts the car node application.
 *
 * Public function defined in app_car_node.h.
 */
uint8_t CarNode_start(void)
{
    uint8_t status = SUCCESS;

    // Register to the Channel Sounding events
    ChannelSounding_registerEvtHandler(&CarNode_handleEvents);

    // Register to the events of the Transceiver
    ChannelSoundingTransceiver_registerEvtHandler(&CarNode_transceiverEventHandler);

    // Start the Transceiver which creates the PSM of the L2cap
    if( status == USUCCESS )
    {
        status = ChannelSoundingTransceiver_start();
    }

#ifdef CS_MEASURE_DISTANCE
    // Start CS Process module
    if (status == USUCCESS)
    {
        status = (uint8_t) CSProcess_Start();
    }

    // Clear all car node sessions DB
    if (status == USUCCESS)
    {
        // Clear all sessions DB
        for (uint16_t i = 0; i < CAR_NODE_MAX_CONNS; i++)
        {
            CarNode_clearSession(i);
            CarNode_clearConfigurationParams(i);
        }

        // Cache local T_SW capability (device-level, read once and reused for all connections)
        csCapabilities_t localCaps;
        status = CS_ReadLocalSupportedCapabilities(&localCaps);
        if (status == USUCCESS)
        {
            gLocalTsw = localCaps.tSwCap;
        }
    }

#endif // CS_MEASURE_DISTANCE


#ifdef RANGING_CLIENT
    // Start the ranging requester
    if (status == USUCCESS)
    {
        status = AppRREQ_start();
        AppRREQ_registerDataEvtHandler(&CarNode_RREQ_EventHandler);
    }
#endif

    if (status == USUCCESS)
    {
        status = BLEAppUtil_registerEventHandler(&carNodeConnHandler);
    }

    #if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
    if ( status == USUCCESS )
    {
        status = CarNode_adv();
    }
#endif
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
    if ( status == USUCCESS )
    {
        Central_registerEvtHandler(&CarNode_handleScanEvent);
        status = CarNode_startScan();
    }
#endif

    uartInit();

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
    UART2_write(uart, "\r\nAdvertising as: ", 18, NULL);
    UART2_write(uart, attDeviceName, strlen((const char*)attDeviceName), NULL);
#endif
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
    UART2_write(uart, "\r\nScanning for: ", 16, NULL);
    UART2_write(uart, KEY_NODE_NAME, strlen((const char*)KEY_NODE_NAME), NULL);
#endif

    GPIO_init();

    return status;
}

/*******************************************************************************
 * This API lets the application register an external event handler
 * function to the car_node module, in order to receive events.
 *
 * Public function defined in app_car_node.h.
 */
void CarNode_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
    gExtEvtHandler = fEventHandler;
}

/***********************************************************************
** Internal Functions
*/

/*********************************************************************
 * @fn      CarNode_extEvtHandler
 *
 * @brief   Forwards the event to the registered external event handler.
 *
 * @param   eventType - the event type of the event @ref BLEAppUtil_eventHandlerType_e
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
void CarNode_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    // Send the event to the upper layer if its handler exists
    if (gExtEvtHandler != NULL)
    {
        gExtEvtHandler(eventType, event, pMsgData);
    }
}

/*********************************************************************
 * @fn      CarNode_handleEvents
 *
 * @brief   This function handles Channel Sounding event sent from
 *          a lower layer module, Ensures that the raised event are as expected
 *          and pass the event message data to the right handler.
 *
 * @param   eventType - the type of the events @ref BLEAppUtil_eventHandlerType_e.
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
void CarNode_handleEvents(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    if ( eventType == BLEAPPUTIL_CS_TYPE && event == BLEAPPUTIL_CS_EVENT_CODE )
    {
        CarNode_csEvtHandler((csEvtHdr_t *)pMsgData);
    }
}

#ifdef CS_MEASURE_DISTANCE

/*********************************************************************
 * @fn      CarNode_clearProceduresList
 *
 * @brief Clears a given procedures list by freeing all its elements.
 *
 * @param pProcedureList - Pointer to the procedures list to be cleared.
 *
 * @return None
 */
void CarNode_clearProceduresList(osal_list_list* pProcedureList)
{
    if (NULL != pProcedureList)
    {
        osal_list_elem* pElem = osal_list_get(pProcedureList);

        while(pElem != NULL)
        {
            CarNode_procElemFree((CarNode_procedureInfoElem_t *) pElem);

            // Get the next element
            pElem = osal_list_get(pProcedureList);
        }

        // Clear the procedure list for the given connection
        osal_list_clearList(pProcedureList);
    }
}

/*********************************************************************
 * @fn      CarNode_getProcedureElem
 *
 * @brief Retrieves a procedure element associated with a connection handle
 *        and a procedure counter.
 *
 * @param connHandle - Connection handle
 * @param procedureCounter - Procedure counter to match against the elements in the list.
 *
 * @return Pointer to the procedure element if found, NULL otherwise.
 */
CarNode_procedureInfoElem_t* CarNode_getProcedureElem(uint16_t connHandle, uint16_t procedureCounter)
{
    CarNode_procedureInfoElem_t* procedureElem = NULL;

    if (connHandle < CAR_NODE_MAX_CONNS)
    {
        // Get the procedure element from the session's procedure list
        osal_list_elem* pElem = osal_list_head(&gSessionsDb[connHandle].procedureList);

        while (pElem != NULL)
        {
            // Check if the procedure counter matches
            if (CAR_NODE_COMPARE_12_BITS(((CarNode_procedureInfoElem_t*) pElem)->procedureCounter, procedureCounter) == TRUE)
            {
                procedureElem = (CarNode_procedureInfoElem_t *)pElem;
                break;
            }

            // Get the next element
            pElem = osal_list_next(pElem);
        }
    }

    return procedureElem;
}

/*********************************************************************
 * @fn      CarNode_procElemRemove
 *
 * @brief Removes a procedure element from a procedure list associated
 *        with a connection handle
 *
 * @param connHandle - Connection handle
 * @param pProcedureElem - Pointer to the procedure element to be removed.
 *
 * @return None
 */
void CarNode_procElemRemove(uint16_t connHandle, CarNode_procedureInfoElem_t* pProcedureElem)
{
    if (connHandle < CAR_NODE_MAX_CONNS && pProcedureElem != NULL)
    {
        if(gSessionsDb[connHandle].procedureListSize > 0)
        {
            // Free the procedure element
            osal_list_remove(&gSessionsDb[connHandle].procedureList, (osal_list_elem*) pProcedureElem);
            gSessionsDb[connHandle].procedureListSize--;
            CarNode_procElemFree(pProcedureElem);
        }
    }
}

/*********************************************************************
 * @fn      CarNode_procElemFree
 *
 * @brief Frees a procedure element and clears its results lists.
 *
 * @param pProcedureElem  - Pointer to the procedure element to be freed.
 *                          If NULL, the function does nothing.
 *
 * @return None
 */
void CarNode_procElemFree(CarNode_procedureInfoElem_t *pProcedureElem)
{
    if (pProcedureElem != NULL)
    {
        // Clear the local results list, no free callback needed
        CarNode_clearResultsList(&(pProcedureElem->localResults), NULL);

        // Clear the remote results list, providing the free callback if exists
        CarNode_clearResultsList(&(pProcedureElem->remoteResults), pProcedureElem->pFreeRemoteResultsElemCB);

        // Free the procedure element
        ICall_free(pProcedureElem);
    }
}

/*********************************************************************
 * @fn      CarNode_clearResultsList
 *
 * @brief Clears a given results list by freeing all its elements.
 *
 * @param pResultsList  - Pointer to the results list to be cleared.
 *                        If NULL, the function does nothing.
 * @param freeCb        - Callback function to free an element in remoteResults list.
 *                        If NULL, the elements will be freed directly.
 *
 * @return None
 */
void CarNode_clearResultsList(osal_list_list* pResultsList, CarNode_freeResultsElemCB_t freeCb)
{
    if (NULL != pResultsList)
    {
        osal_list_elem* pElem = osal_list_get(pResultsList);

        while(pElem != NULL)
        {
            if (freeCb != NULL)
            {
                // Use the provided callback to free the element
                freeCb((CarNode_resultsElem_t*) pElem);
            }
            else
            {
                // Free the element directly
                ICall_free(pElem);
            }

            // Get the next element
            pElem = osal_list_get(pResultsList);
        }

        // Clear the results list
        osal_list_clearList(pResultsList);
    }
}

/*********************************************************************
 * @fn      CarNode_clearTimeoutProcedures
 *
 * @brief Traverse through all connections and their procedures lists,
 *        Removing any procedures that have timed out based on the
 *        @ref CAR_NODE_TIMEOUT_THRESHOLD_TICKS.
 *        After the cleanup, if there are still active procedures,
 *        a timer will activated in order to call this function again.
 *        the timer will be rescheduled each time this function is called
 *        and there are still active procedures in the list.
 *
 * @note  There is no risk of race conditions because this function is called
 *        in BLEAppUtil context (both as a direct call and as a timer callback).
 *
 * @param None
 *
 * @return None
 */
void CarNode_clearTimeoutProcedures()
{
    // Get current time
    uint32_t currTime = llGetCurrentTime();

    // Count the number of procedures in the procedures list
    uint16_t numActiveProcedures = 0;

    // Iterate through all connections
    for (uint16_t connHandle = 0; connHandle < CAR_NODE_MAX_CONNS; connHandle++)
    {
        // Get the current session
        CarNode_session* pSession = &gSessionsDb[connHandle];

        // Check if there are any procedures in the list
        if (!osal_list_empty(&pSession->procedureList))
        {
            osal_list_elem* pElem = osal_list_head(&pSession->procedureList);

            while (pElem != NULL)
            {
                numActiveProcedures++;
                CarNode_procedureInfoElem_t* pProcElem = (CarNode_procedureInfoElem_t*) pElem;

                // Get the next element as a temporary variable
                osal_list_elem* pElemNextTemp = osal_list_next(pElem);

                // Check if the procedure has timed out
                if (llTimeAbs(pProcElem->timestamp, currTime) > CAR_NODE_TIMEOUT_THRESHOLD_TICKS)
                {
                    // Remove and free the procedure element
                    CarNode_procElemRemove(connHandle, pProcElem);
                    numActiveProcedures--;

                    // TODO: Report to the upper layer about the procedure timeout
                }

                // Set the current element to the next one
                pElem = pElemNextTemp;
            }
        }
    }

    // If there is an active timer - abort it
    if (gTimerHandle != BLEAPPUTIL_TIMER_INVALID_HANDLE)
    {
        BLEAppUtil_abortTimer(gTimerHandle);
        gTimerHandle = BLEAPPUTIL_TIMER_INVALID_HANDLE;
    }

    // If there are still active procedures - activate a new timer
    // If not - it will be activated here once there will be a new procedure in the list
    if (numActiveProcedures > 0)
    {
        gTimerHandle = BLEAppUtil_startTimer(CarNode_timerCB, CAR_NODE_TIMER_TIMEOUT_MS, FALSE, NULL);
    }
}

/*********************************************************************
 * @fn      CAClient_timerCB
 *
 * @brief   General timer callback function of type @ref BLEAppUtil_timerCB_t.
 *          This function is called when the associated timer expires.
 *          It calls @ref CarNode_clearTimeoutProcedures function in order
 *          to clear pending procedures that have timed out.
 *
 * @param   timerHandle - The handle of the timer that expired.
 * @param   reason - The reason for the timer expiration.
 * @param   pData - Pointer to the data associated with the timer. Not used.
 *
 * @return None
 */
void CarNode_timerCB(BLEAppUtil_timerHandle timerHandle, BLEAppUtil_timerTermReason_e reason, void *pData)
{
    // If we expect for a timer to expire
    if (gTimerHandle == timerHandle)
    {
        // Reset the global timer handle
        gTimerHandle = BLEAPPUTIL_TIMER_INVALID_HANDLE;

        if (reason == BLEAPPUTIL_TIMER_TIMEOUT)
        {
            // Clear the procedures that have timed out
            CarNode_clearTimeoutProcedures();
        }
        else if (reason == BLEAPPUTIL_TIMER_ABORTED)
        {
            // Timer was aborted, no action needed
        }
    }
}

/*********************************************************************
 * @fn      CarNode_clearSession
 *
 * @brief   Clears a CS Process session for a specific connection and
 *          its procedures list.
 *
 * @note    Does not clear CS Configuration parameters
 *
 * @param   connHandle - Connection handle bound to the session to be cleared
 *
 * @return  None
 */
void CarNode_clearSession(uint16_t connHandle)
{
    if (connHandle < CAR_NODE_MAX_CONNS)
    {
        CarNode_clearProceduresList(&gSessionsDb[connHandle].procedureList);

        gSessionsDb[connHandle].sessionId = CS_PROCESS_INVALID_SESSION;
        gSessionsDb[connHandle].currProcedureCounter = CAR_NODE_PROC_COUNTER_IDLE;
        gSessionsDb[connHandle].selectedTxPower = CS_INVALID_TX_POWER;
        gSessionsDb[connHandle].procedureListSize = 0;

#ifdef CS_MEASURE_DISTANCE
        gCsProcState = CS_PROC_STATE_IDLE;
#endif
    }
}

/*********************************************************************
 * @fn      CarNode_clearConfigurationParams
 *
 * @brief   Clears CS Configuration and remote capabilities parameters
 *          for a specific connection.
 *
 * @param   connHandle - Connection handle bound to the session to be cleared
 *
 * @return  None
 */
void CarNode_clearConfigurationParams(uint16_t connHandle)
{
    if (connHandle < CAR_NODE_MAX_CONNS)
    {
        gSessionsDb[connHandle].remoteTsw = 0;
    }
}

uint8_t CarNode_getRemoteTsw(uint16_t connHandle)
{
    if (connHandle < CAR_NODE_MAX_CONNS)
    {
        return gSessionsDb[connHandle].remoteTsw;
    }
    return 0;
}

void CarNode_setRemoteTsw(uint16_t connHandle, uint8_t tsw)
{
    if (connHandle < CAR_NODE_MAX_CONNS)
    {
        gSessionsDb[connHandle].remoteTsw = tsw;
    }
}

/*********************************************************************
 * @fn      CarNode_handleCSProcessAddResultsStatus
 *
 * @brief   This function handles the status returned from
 *          @ref CSProcess_AddSubeventResults function.
 *
 * @param   connHandle - Connection handle associated with the procedure that
 *                       @ref CSProcess_AddSubeventResults was called for.
 * @param   status - Status to be handled
 *
 * @return  TRUE - If distance was estimated.
 *          FALSE - otherwise.
 */
bool CarNode_handleCSProcessAddResultsStatus(uint16_t connHandle, csProcessStatus_e statusToHandle)
{
    bool isDistanceEstimated = FALSE;

    switch (statusToHandle)
    {
        case CS_PROCESS_SUCCESS:
        {
            // Successfully processed the subevent results, no need to do anything else
            break;
        }
        case CS_PROCESS_DISTANCE_ESTIMATION_PENDING:
        {
            if (CarNode_handleCSProcessDistancePendingStatus(connHandle) == CS_PROCESS_SUCCESS)
            {
                isDistanceEstimated = TRUE;
            }
            break;
        }
        default:
        {
            // Terminate CS Process procedure if active
            CSProcess_TerminateProcedure();

            // Send an error event to the upper layer
            CarNode_sendDistanceResultsError(connHandle, (bStatus_t) statusToHandle);

            break;
        }
    }

    return isDistanceEstimated;
}

/*********************************************************************
 * @fn      CarNode_handleCSProcessDistancePendingStatus
 *
 * @brief   This function handles @ref CS_PROCESS_DISTANCE_ESTIMATION_PENDING
 *          status returned from @ref CSProcess_AddSubeventResults function.
 *
 * @param   connHandle - Connection handle associated with the procedure that
 *                       @ref CSProcess_AddSubeventResults was called for.
 *
 * @return  The status returned from @ref CSProcess_EstimateDistance
 */
csProcessStatus_e CarNode_handleCSProcessDistancePendingStatus(uint16_t connHandle)
{
    CSProcess_Results_t distanceResults;
    csProcessStatus_e status;

    /***********************************************************************
     Critical point - The following function might take some time to finish
    ***********************************************************************/

    status = CSProcess_EstimateDistance(&distanceResults);

    switch (status)
    {
        case CS_PROCESS_SUCCESS:
        case CS_PROCESS_DISTANCE_ESTIMATION_FAILED:
        {
            // Send the results as they came from the CS Process module
            CarNode_handleDistanceResults(connHandle, distanceResults, (bStatus_t) status);
            break;
        }
        case CS_PROCESS_PROCEDURE_PROCESSING_PENDING:
        {
            // Shouldn't be happening here, but if it does - terminate the procedure in the
            // CS Process module and send an error
            CSProcess_TerminateProcedure();
            CarNode_sendDistanceResultsError(connHandle, (bStatus_t) status);
            break;
        }
        case CS_PROCESS_PROCEDURE_NOT_ACTIVE:
        {
            // Shouldn't be happening here, but if it does - send an error event
            CarNode_sendDistanceResultsError(connHandle, (bStatus_t) status);
            break;
        }
        default:
        {
            break;
        }
    }

    return status;
}

/*********************************************************************
 * @fn      CarNode_sendDistanceResultsError
 *
 * @brief   This function sends an error distance event to the upper layer
 *
 * @param   connHandle - Connection handle associated with the reported error
 * @param   status     - Status to be reported in the distance event
 *
 * @return  None
 */
void CarNode_sendDistanceResultsError(uint16_t connHandle, bStatus_t status)
{
    CSProcess_Results_t distanceResults = {0};

    CarNode_handleDistanceResults(connHandle, distanceResults, status);
}

/*********************************************************************
 * @fn      CarNode_handleDistanceResults
 *
 * @brief   This function receives distance results from the app_cs_process
 *          module and sends the results to the upper layer.
 *
 * @param   connHandle      - Connection handle associated with the reported results
 * @param   distanceResults - Distance results to send
 * @param   status          - Status of the results
 *
 * @return  none
 */
void CarNode_handleDistanceResults(uint16_t connHandle, CSProcess_Results_t distanceResults, bStatus_t status)
{
    // At first, take the results as is
    CSProcess_Results_t finalResults = distanceResults;

    // If fail - override with an invalid results
    if (status != CS_PROCESS_SUCCESS)
    {
        finalResults.distance = CAR_NODE_INVALID_RESULTS;
        finalResults.quality = CAR_NODE_INVALID_RESULTS;
        finalResults.confidence = CAR_NODE_INVALID_RESULTS;
        finalResults.velocity = CAR_NODE_INVALID_RESULTS;
    }

    ChannelSounding_appDistanceResultsEvent_t distResultsEvt;
    distResultsEvt.csEvtOpcode = CS_APP_DISTANCE_RESULTS_EVENT;
    distResultsEvt.status      = (uint8_t) status;
    distResultsEvt.connHandle  = connHandle;
    distResultsEvt.distance    = finalResults.distance;
    distResultsEvt.quality     = finalResults.quality;
    distResultsEvt.confidence  = finalResults.confidence;
    distResultsEvt.velocity    = finalResults.velocity;

    //CarNode_extEvtHandler(BLEAPPUTIL_CS_TYPE, BLEAPPUTIL_CS_APP_EVENT_CODE, (BLEAppUtil_msgHdr_t *) (&distResultsEvt));
    if (distResultsEvt.distance != (uint32_t)UINT_MAX)
    {
        char dist[5];
        dist[4] = 0;
        dist[3] = (char)(distResultsEvt.distance % 10 + '0');
        dist[2] = (char)(distResultsEvt.distance / 10   % 10 + '0');
        dist[1] = (char)(distResultsEvt.distance / 100  % 10 + '0');
        dist[0] = (char)(distResultsEvt.distance / 1000 % 10 + '0');

        char velocity[6];
        int32_t vel = finalResults.velocity;
        int idx = 0;
        if (vel < 0) { velocity[idx++] = '-'; vel = -vel; }
        velocity[idx++] = (char)(vel / 1000 % 10 + '0');
        velocity[idx++] = (char)(vel / 100  % 10 + '0');
        velocity[idx++] = (char)(vel / 10   % 10 + '0');
        velocity[idx++] = (char)(vel % 10 + '0');
        velocity[idx] = 0;

        char conf[4];
        uint32_t c = distResultsEvt.confidence;
        conf[0] = (char)(c / 100 % 10 + '0');
        conf[1] = (char)(c / 10  % 10 + '0');
        conf[2] = (char)(c       % 10 + '0');
        conf[3] = 0;

        int16_t rssiRaw = (int16_t)finalResults.rssi;
        char rssi_str[5];
        int rssi_idx = 0;
        if (rssiRaw < 0) { rssi_str[rssi_idx++] = '-'; rssiRaw = -rssiRaw; }
        rssi_str[rssi_idx++] = (char)(rssiRaw / 100 % 10 + '0');
        rssi_str[rssi_idx++] = (char)(rssiRaw / 10  % 10 + '0');
        rssi_str[rssi_idx++] = (char)(rssiRaw       % 10 + '0');
        rssi_str[rssi_idx] = 0;

        UART2_write(uart, dist, 4, NULL);
        UART2_write(uart, "  ", 2, NULL);
        UART2_write(uart, velocity, idx, NULL);
        UART2_write(uart, "  ", 2, NULL);
        UART2_write(uart, conf, 3, NULL);
        UART2_write(uart, "  ", 2, NULL);
        UART2_write(uart, rssi_str, rssi_idx, NULL);
        UART2_write(uart, "\r\n", 2, NULL);
    }

    cyclic_handle = connHandle;

    gProcedureRepeatCount++;
    gDistanceProcessed = true;

    if ((csSetProcedureParams.maxProcedureCount != 0 &&
         gProcedureRepeatCount >= csSetProcedureParams.maxProcedureCount) ||
        (csSetProcedureParams.maxProcedureCount == 1))
    {
        if (CarNode_getConfiguredCsRole() == CS_ROLE_INITIATOR)
        {
            gCsProcState = CS_PROC_STATE_IDLE;
            gDistanceProcessed = false;
            if (gProcedureRepeatCount >= 70)
            {
                gProcedureRepeatCount = 0;
                HandoverServingStart(connHandle);
            }
            else
            {
                CarNode_invokeProcedureEnableCmd(connHandle);
            }
        }
    }
}

/*********************************************************************
 * @fn      CarNode_procElemAddResultsProp
 *
 * @brief   This function adds a subevent results element to a given procedure
 *          element. It only handles results for the Local and Prop modes.
 *          It checks if the procedure is aborted or if the subevent is aborted,
 *          and updates the procedure state accordingly.
 *          If the given statuses are not of type 'aborted' - it will add the
 *          results to the given procedure element results list.
 *
 * @param   pProcedureElem - Pointer to the procedure element to add the results to.
 * @param   resultsSourceMode - Indicates the source of the subevent. type @ref ChannelSounding_resultsSourceMode_e
 * @param   pParams - Pointer to the procedure element params
 *
 * @return  SUCCESS - If the results were added successfully
 *          FAILURE - If one of the pointers is NULL
 *                  - If the results source mode is not supported
 *                  - If the procedure \ subevent is aborted
 *                  - If already have results ready for the given source mode
 *                  - If allocating memory failed
 */
bStatus_t CarNode_procElemAddResultsProp(CarNode_procedureInfoElem_t* pProcedureElem, ChannelSounding_resultsSourceMode_e resultsSourceMode, CarNode_procElemAddResultsPropParams_t* pParams)
{
    bStatus_t status = SUCCESS;

    if (pProcedureElem == NULL || pParams == NULL || pParams->pData == NULL || resultsSourceMode > CS_RESULTS_MODE_PROP)
    {
        status = FAILURE;
    }

    if (status == SUCCESS)
    {
        // If processing local results and the procedure is aborted - mark it
        if (resultsSourceMode == CS_RESULTS_MODE_LOCAL && pParams->procedureDoneStatus == CS_PROCEDURE_ABORTED)
        {
            CAR_NODE_STATE_SET_FLAG(pProcedureElem->state, CAR_NODE_STATE_LOCAL_PROCEDURE_ABORTED);
            status = FAILURE;
        }

        // Ensure we don't add data for a list which already has results ready
        if (CAR_NODE_IS_RESULTS_READY(resultsSourceMode, pProcedureElem->state))
        {
            status = FAILURE;
        }
    }

    if (status == SUCCESS)
    {
        // If processing remote results and this is the first subevent received
        if (resultsSourceMode == CS_RESULTS_MODE_PROP && osal_list_empty(&pProcedureElem->remoteResults))
        {
            pProcedureElem->remoteResultsMode = CS_RESULTS_MODE_PROP;
        }

        // Allocate a new results element
        uint16_t resultsElemSize = sizeof(CarNode_resultsElem_t) + sizeof(CarNode_resultsElemDataProp_t) + pParams->dataLen;
        CarNode_resultsElem_t* resultsElem = (CarNode_resultsElem_t*) ICall_malloc(resultsElemSize);

        // If failed to allocate memory for the results element
        if (NULL == resultsElem)
        {
            status = FAILURE;
        }
        else
        {
            // Initiate the results element data
            CarNode_resultsElemDataProp_t* resultsElemData = (CarNode_resultsElemDataProp_t*) resultsElem->resultsData;

            // Copy the data
            resultsElemData->referencePowerLevel   = pParams->referencePowerLevel;
            resultsElemData->frequencyCompensation = pParams->frequencyCompensation;
            resultsElemData->procedureDoneStatus   = pParams->procedureDoneStatus;
            resultsElemData->subeventDoneStatus    = pParams->subeventDoneStatus;
            resultsElemData->numAntennaPath        = pParams->numAntennaPath;
            resultsElemData->numStepsReported      = pParams->numStepsReported;
            resultsElemData->dataLen               = pParams->dataLen;
            memcpy(resultsElemData->data, pParams->pData, pParams->dataLen);

            // Add the results element to the end of the relevant procedure's results list
            if (resultsSourceMode == CS_RESULTS_MODE_LOCAL)
            {
                osal_list_put(&pProcedureElem->localResults, (osal_list_elem*) resultsElem);
            }
            else
            {
                osal_list_put(&pProcedureElem->remoteResults, (osal_list_elem*) resultsElem);
            }

            // Update procedure state flags if done
            if(pParams->procedureDoneStatus == CS_PROCEDURE_DONE)
            {
                if (resultsSourceMode == CS_RESULTS_MODE_LOCAL)
                {
                    CAR_NODE_STATE_SET_FLAG(pProcedureElem->state, CAR_NODE_STATE_LOCAL_DATA_READY);
                }
                else
                {
                    CAR_NODE_STATE_SET_FLAG(pProcedureElem->state, CAR_NODE_STATE_REMOTE_DATA_READY);
                }
            }
            else if (resultsSourceMode == CS_RESULTS_MODE_PROP && pParams->procedureDoneStatus == CS_PROCEDURE_ABORTED)
            {
                CAR_NODE_STATE_SET_FLAG(pProcedureElem->state, CAR_NODE_STATE_REMOTE_DATA_READY);
            }
        }
    }

    return status;
}

/*********************************************************************
 * @fn      CarNode_procElemAddResultsPostProcess
 *
 * @brief   This function is used as a post process function to be executed
 *          after subevent results are added to a procedure element.
 *          It should be called right after calling to the following functions:
 *          @ref CarNode_procElemAddResultsProp
 *          @ref CarNode_procElemAddResultsRAS
 *
 *          It checks the procedure state and results mode, and the status
 *          returned from the above functions.
 *
 * @param   status - Status returned from @ref CarNode_procElemAddResultsProp
 *                   or @ref CarNode_procElemAddResultsRAS.
 * @param   connHandle - Connection handle associated with the procedure element.
 * @param   resultsMode - Source mode of the results that were added to the procedure element.
 * @param   pProcedureElem - Pointer to the procedure element to post process.
 *
 * @return  none
 */
void CarNode_procElemAddResultsPostProcess(bStatus_t status, uint16_t connHandle, ChannelSounding_resultsSourceMode_e resultsMode,
                                           CarNode_procedureInfoElem_t* pProcedureElem)
{
    // If the given params are valid
    if (connHandle < CAR_NODE_MAX_CONNS && pProcedureElem != NULL)
    {
        // If this is a post process of a local results and one of the following is true:
        // 1. failure occurred
        // 2. done processing the local results
        // 3. local procedure is aborted
        if (resultsMode == CS_RESULTS_MODE_LOCAL &&
            (status == FAILURE ||
             CAR_NODE_STATE_IS_FLAG_SET(pProcedureElem->state, CAR_NODE_STATE_LOCAL_DATA_READY) ||
             CAR_NODE_STATE_IS_FLAG_SET(pProcedureElem->state, CAR_NODE_STATE_LOCAL_PROCEDURE_ABORTED)))
        {
            // Reset Current Procedure Counter
            gSessionsDb[connHandle].currProcedureCounter = CAR_NODE_PROC_COUNTER_IDLE;
        }

        // If local procedure was aborted - remove the procedure element and send distance error
        if (CAR_NODE_STATE_IS_FLAG_SET(pProcedureElem->state, CAR_NODE_STATE_LOCAL_PROCEDURE_ABORTED))
        {
            /**** Abort procedure results collection ****/

            // Remove and free the procedure results
            CarNode_procElemRemove(connHandle, pProcedureElem);

            // Send distance error
            CarNode_sendDistanceResultsError(connHandle, (bStatus_t) CS_PROCESS_PROCEDURE_ABORTED);
        }
        else if (CAR_NODE_STATE_IS_FLAG_SET(pProcedureElem->state, CAR_NODE_STATE_LOCAL_DATA_READY) &&
                 CAR_NODE_STATE_IS_FLAG_SET(pProcedureElem->state, CAR_NODE_STATE_REMOTE_DATA_READY))
        {
            // Local and Remote results are ready - handle by adding the results to CS Process module
            CarNode_procElemHandleStateReady(connHandle, pProcedureElem);
        }
        else
        {

        }
    }

    // Now check if there are procedures that have timed out for all connections and clear them
    CarNode_clearTimeoutProcedures();
}

/*********************************************************************
 * @fn      CarNode_procElemHandleStateReady
 *
 * @brief   This function handles the procedure element when both local
 *          and remote results are ready. It Initialize a CS Process procedure,
 *          adds the results to it and if possible - estimates the distance.
 *          the function removes the given procedure from the procedures list,
 *          and if the distance is estimated successfully - also removes all
 *          previous procedures that came before it.
 *
 * @param   connHandle - Connection handle associated with the procedure element.
 * @param   pProcedureElem - Pointer to the procedure element to post process.
 *
 * @return  none
 */
void CarNode_procElemHandleStateReady(uint16_t connHandle, CarNode_procedureInfoElem_t* pProcedureElem)
{
    csProcessStatus_e csProcessStatus = CS_PROCESS_SUCCESS;
    bool isDistanceEstimated = FALSE;

    // Get const pointer to CS configuration (zero-copy direct access to controller DB)
    const struct csConfig_t* pConfig = CS_GetConfiguration(connHandle, pProcedureElem->configId);

    if (pConfig == NULL)
    {
        // Config not found - should not happen, indicates internal error
        return;
    }

    // Init CS Process procedure
    CSProcess_InitProcedureParams_t initProcParams;
    initProcParams.handle                     = gSessionsDb[connHandle].sessionId;
    initProcParams.numAntPath                 = gSessionsDb[connHandle].numAntPath;
    initProcParams.localRole                  = pConfig->role;
    // Convert timing indices to microseconds (pConfig contains indices, ranging lib expects microseconds)
    initProcParams.tIP1                       = (uint8_t)CS_ConvertTip(pConfig->tIP1);
    initProcParams.tIP2                       = (uint8_t)CS_ConvertTip(pConfig->tIP2);
    initProcParams.tFCS                       = (uint8_t)CS_ConvertTfcs(pConfig->tFCs);
    initProcParams.tPM                        = (uint8_t)CS_ConvertTpm(pConfig->tPM);
    initProcParams.tSW                        = gSessionsDb[connHandle].finalTsw;
    initProcParams.csSync                     = pConfig->csSyncPhy;
    initProcParams.rttType                    = pConfig->rttType;
    initProcParams.mode0Steps                 = pConfig->modeZeroSteps;
    initProcParams.mainModeType               = pConfig->mainMode;
    initProcParams.mainModeRepetition         = pConfig->mainModeRepetition;
    initProcParams.toneAntennaConfigSelection = gSessionsDb[connHandle].aci;

    // Initialize CS Process procedure
    csProcessStatus = CSProcess_InitProcedure(&initProcParams);

    // Add results to CS Process module
    if (csProcessStatus == CS_PROCESS_SUCCESS)
    {
        // Add local results list
        csProcessStatus = CarNode_CSProcessAddResults(connHandle, CS_RESULTS_MODE_LOCAL, &pProcedureElem->localResults, pConfig->role);
        isDistanceEstimated = CarNode_handleCSProcessAddResultsStatus(connHandle, csProcessStatus);

        // If local results were added successfully, add remote results list
        if (csProcessStatus == CS_PROCESS_SUCCESS)
        {
            csProcessStatus = CarNode_CSProcessAddResults(connHandle, pProcedureElem->remoteResultsMode, &pProcedureElem->remoteResults, CS_GET_OPPOSITE_ROLE(pConfig->role));
            isDistanceEstimated = CarNode_handleCSProcessAddResultsStatus(connHandle, csProcessStatus);
        }
    }

    // Remove and free all of the procedures came before the procedure we are working on
    if (isDistanceEstimated == TRUE)
    {
        osal_list_elem* pElem = osal_list_head(&gSessionsDb[connHandle].procedureList);
        while (pElem != NULL)
        {
            CarNode_procedureInfoElem_t* pCurrProcElem = (CarNode_procedureInfoElem_t*) pElem;

            // Get the next element as a temporary variable
            osal_list_elem* pElemNextTemp = osal_list_next(pElem);

            // If the procedure element is before the current procedure element - remove it.
            // Note: the list is time ordered, so each element before the current procedure element
            // is guaranteed to be older
            if (pCurrProcElem != pProcedureElem)
            {
                // Remove and free the procedure element
                CarNode_procElemRemove(connHandle, pCurrProcElem);

                // TODO: Report to the upper layer that a procedure has been removed
            }
            else
            {
                // Break when we got to the current procedure element
                break;
            }

            // Set the current element to the next one
            pElem = pElemNextTemp;
        }
    }

    // Remove and free the procedure
    CarNode_procElemRemove(connHandle, pProcedureElem);
}

/*********************************************************************
 * @fn      CarNode_countSubeventTotalSteps
 *
 * @brief   This function traverses a results list and sums the numStepsReported
 *          of each subevent until it encounters an element with subeventDoneStatus
 *          of CS_SUBEVENT_ABORTED or CS_SUBEVENT_DONE.
 *
 * @param   pResultsList - Pointer to the results list to traverse.
 *                         Assumed not NULL.
 *
 * @return  The total number of steps reported across all subevents until a terminal status is reached.
 */
static uint8_t CarNode_countSubeventTotalSteps(osal_list_elem* pElem)
{
    uint8_t totalSteps = 0;

    // Traverse through the results list and sum numStepsReported
    while (pElem != NULL)
    {
        CarNode_resultsElemDataProp_t* pElemData = (CarNode_resultsElemDataProp_t*) (((CarNode_resultsElem_t*) pElem)->resultsData);

        // Add the steps from this element
        totalSteps += pElemData->numStepsReported;

        // Check if we've reached a terminal status
        if (pElemData->subeventDoneStatus == CS_SUBEVENT_ABORTED || pElemData->subeventDoneStatus == CS_SUBEVENT_DONE)
        {
            break;
        }

        pElem = osal_list_next(pElem);
    }

    return totalSteps;
}

/*********************************************************************
 * @fn      CarNode_CSProcessAddResults
 *
 * @brief   This function adds a given list of subevents results to the
 *          CS Process module.
 *
 * @param   connHandle - Connection handle for the session.
 * @param   resultsMode - Source mode of the results to be added.
 * @param   pResultsList - Pointer to the results list to be added.
 *                         Assumed not NULL.
 * @param   role - Role of the node for which the results are being added.
 *
 * @return  CS_PROCESS_GENERAL_FAILURE - if not all of the data in the results list
 *                                       has been processed by the CS Process module.
 * @return  The last status returned by @ref CSProcess_AddSubeventResults otherwise.
 */
csProcessStatus_e CarNode_CSProcessAddResults(uint16_t connHandle, ChannelSounding_resultsSourceMode_e resultsMode, osal_list_list* pResultsList, uint8_t role)
{
    csProcessStatus_e status = CS_PROCESS_INVALID_PARAM;
    osal_list_elem* pElem = (osal_list_elem*) osal_list_head(pResultsList);
    osal_list_elem* pNextElem = NULL;
    uint8_t* pElemData = NULL;
    uint8_t totalStepsCount = 0;

    // Traverse through the results list and add each results element to CS Process module
    while (pElem != NULL)
    {
        pNextElem = (osal_list_elem*) osal_list_next(pElem);

        // Take the current element data section for the different modules parse
        pElemData = ((CarNode_resultsElem_t*) pElem)->resultsData;

        if (resultsMode == CS_RESULTS_MODE_LOCAL || resultsMode == CS_RESULTS_MODE_PROP)
        {
            CarNode_resultsElemDataProp_t* pPropData = (CarNode_resultsElemDataProp_t*) pElemData;

            // Count total steps for the first subevent fragment
            if(totalStepsCount == 0)
            {
                totalStepsCount = CarNode_countSubeventTotalSteps(pElem);
            }

            status = CarNode_CSProcessAddResultsProp(resultsMode, pPropData, totalStepsCount);
            if (pPropData->subeventDoneStatus == CS_SUBEVENT_ABORTED || pPropData->subeventDoneStatus == CS_SUBEVENT_DONE)
            {
                totalStepsCount = 0;
            }

            osal_list_remove(pResultsList, pElem);
            ICall_free(pElem);
        }
        else if (resultsMode == CS_RESULTS_MODE_RAS)
        {
#ifdef RANGING_CLIENT
            status = CarNode_CSProcessAddResultsRAS(connHandle, (CarNode_resultsElemDataRAS_t*) pElemData, role);
#endif
        }
        else
        {
            // TODO: handle BTCS
        }

        if (status != CS_PROCESS_SUCCESS)
        {
            // Distance can be estimated or error occurred, break the loop and return
            break;
        }

        pElem = pNextElem;
    }

    return status;
}

/*********************************************************************
 * @fn      CarNode_CSProcessAddResultsProp
 *
 * @brief   This function adds a single subevent results to the
 *          CS Process module. It handles only results for the Local and Prop modes.
 *
 * @param   resultsMode - Source mode of the results to be added, either @ref CS_RESULTS_MODE_LOCAL
 *                        or @ref CS_RESULTS_MODE_PROP.
 * @param   pElemResultsData - Pointer to the results data of the subevent to be added.
 * @param   totalStepsInSubevent - Number of steps in this subevent, only counted in the first element
 *
 * @return  CS_PROCESS_GENERAL_FAILURE - if not all of the data in the results list
 *                                       has been processed by the CS Process module.
 * @return  The last status returned by @ref CSProcess_AddSubeventResults otherwise.
 */
csProcessStatus_e CarNode_CSProcessAddResultsProp(ChannelSounding_resultsSourceMode_e resultsMode, CarNode_resultsElemDataProp_t* pElemResultsData, uint8_t totalStepsInSubevent)
{
    csProcessStatus_e status = CS_PROCESS_INVALID_PARAM;
    uint32_t totalBytesProcessed = 0;

    // Prepare function parameters
    CSProcess_AddSubeventResultsParams_t subeventParams;
    subeventParams.resultsSourceMode       = resultsMode;
    subeventParams.referencePowerLevel     = pElemResultsData->referencePowerLevel;
    subeventParams.frequencyCompensation   = resultsMode == CS_RESULTS_MODE_LOCAL ? pElemResultsData->frequencyCompensation : 0; // local: measured; remote: 0
    subeventParams.subeventDoneStatus      = pElemResultsData->subeventDoneStatus;
    subeventParams.procedureDoneStatus     = pElemResultsData->procedureDoneStatus;
    subeventParams.numAntennaPath          = pElemResultsData->numAntennaPath;
    subeventParams.numStepsReported        = pElemResultsData->numStepsReported;
    subeventParams.totalSubeventStepsCount = totalStepsInSubevent;
    subeventParams.data                    = pElemResultsData->data;
    subeventParams.totalBytesProcessed     = &totalBytesProcessed; // Output parameter

    // Add results to CS Process
    status = CSProcess_AddSubeventResults(&subeventParams);

    // Ensure all of the data has been processed by the CS Process module
    // Note: we do this only when CS Process module returned SUCCESS, just to make sure
    // that all of the data has been processed. If another status was returned - we don't care
    // about the total bytes processed
    if (status == CS_PROCESS_SUCCESS && totalBytesProcessed != pElemResultsData->dataLen)
    {
        status = CS_PROCESS_GENERAL_FAILURE;
    }

    return status;
}

#ifdef RANGING_CLIENT
/*********************************************************************
 * @fn      CarNode_CSProcessAddResultsRAS
 *
 * @brief   This function adds a single RAS Ranging Data to the
 *          CS Process module.
 *
 * @param   connHandle - Connection handle for the session.
 * @param   pElemResultsData - Pointer to the element that contains the ranging data.
 * @param   role - Role of the device for which the results are being added.
 *
 * @return  CS_PROCESS_GENERAL_FAILURE - if not all of the data has been processed
 *                                        by the CS Process module.
 * @return  The last status returned by @ref CSProcess_AddSubeventResults otherwise.
 */
csProcessStatus_e CarNode_CSProcessAddResultsRAS(uint16_t connHandle, CarNode_resultsElemDataRAS_t* pElemResultsData, uint8_t role)
{
    csProcessStatus_e status = CS_PROCESS_INVALID_PARAM; // Status to be returned from CS Process module
    uint8_t rangingStatus = SUCCESS;                     // Status to be returned from Ranging DB Client module
    uint16_t rangingDbLen;                               // Holds the total length of the Ranging DB
    uint16_t totalBytesProcessed;                        // Holds the total number of bytes to be processed by the CS Process module.
    Ranging_RangingHeader_t rangingHeader;               // Holds the Ranging Header
    uint8_t numAntPath = 0;                              // Holds the number of antenna paths, implied by ranging header
    Ranging_subEventHeader_t subeventHeader;             // Holds the current subevent header
    uint8_t* subeventData = NULL;                        // Pointer to hold the current subevent data

    // Get the total length of the ranging Db
    rangingDbLen = pElemResultsData->segmentsReader.totalSegmentsSize;

    // Get the Ranging Header first
    rangingStatus = RangingDBClient_getRangingHeader(&pElemResultsData->segmentsReader, &rangingHeader);

    if (rangingStatus == SUCCESS)
    {
        // Initialize totalBytesProcessed with the size of the ranging header, as it is considered processed
        totalBytesProcessed = sizeof(Ranging_RangingData_t);

        // Validate antenna path mask matches configured procedure
        // Use numAntPath from connection level (computed at procedure enable from ACI)
        // Expected mask = (1 << numAntPath) - 1, e.g., 4 paths → 0xF, 2 paths → 0x3
        numAntPath = gSessionsDb[connHandle].numAntPath;
        uint8_t expectedMask = (1 << numAntPath) - 1;

        if (rangingHeader.antennaPathsMask != expectedMask)
        {
            // Data integrity error: Ranging header doesn't match procedure configuration
            // This indicates protocol error, data corruption, or configuration mismatch
            status = CS_PROCESS_INVALID_PARAM;
            rangingStatus = FAILURE;
        }
    }

    if (rangingStatus == SUCCESS && numAntPath != 0)
    {
        // Go over each subevent in the Ranging DB and add it to the CS Process module
        do
        {
            // Reset subevent variables
            subeventData = NULL;

            // Get subevent header and data, this function will allocate memory for the data if execution is successful
            rangingStatus = RangingDBClient_getNextSubevent(&pElemResultsData->segmentsReader, numAntPath, role, &subeventHeader, &subeventData, NULL);

            // If no more valid subevents are available (e.g. truncated RAS data),
            // break out of the loop to avoid spinning forever while totalBytesProcessed
            // never advances past rangingDbLen.
            if (rangingStatus != SUCCESS || subeventData == NULL)
            {
                status = CS_PROCESS_GENERAL_FAILURE;
                break;
            }

            // Holds the current subevent total bytes to be processed by the CS Process module
            uint32_t subeventBytesProcessed = 0;

            // Prepare function parameters
            CSProcess_AddSubeventResultsParams_t subeventParams;
            subeventParams.resultsSourceMode       = CS_RESULTS_MODE_RAS;
            subeventParams.referencePowerLevel     = subeventHeader.referencePowerLvl;
            subeventParams.frequencyCompensation   = 0; // remote: 0
            subeventParams.subeventDoneStatus      = subeventHeader.subeventDoneStatus;
            subeventParams.procedureDoneStatus     = subeventHeader.rangingDoneStatus;
            subeventParams.numAntennaPath          = numAntPath;
            subeventParams.numStepsReported        = subeventHeader.numStepsReported;
            subeventParams.totalSubeventStepsCount = subeventHeader.numStepsReported; // RAS subevent is provided in a single subevent
            subeventParams.data                    = subeventData;
            subeventParams.totalBytesProcessed     = &subeventBytesProcessed; // Output parameter

            // Add results to CS Process
            status = CSProcess_AddSubeventResults(&subeventParams);

            // Increment the total number of bytes processed
            totalBytesProcessed += sizeof(subeventHeader) + subeventBytesProcessed;

            // Ensure all of the data has been processed by the CS Process module and that we are not overflowing
            if (status == CS_PROCESS_SUCCESS && (subeventBytesProcessed == 0 || totalBytesProcessed > rangingDbLen))
            {
                status = CS_PROCESS_GENERAL_FAILURE;
            }

            // Free the subevent data
            if (subeventData != NULL)
            {
                ICall_free(subeventData);
                subeventData = NULL;
            }

        } while (status == CS_PROCESS_SUCCESS && (totalBytesProcessed < rangingDbLen));
    }

    return status;
}
#endif // RANGING_CLIENT

#endif // CS_MEASURE_DISTANCE

/*********************************************************************
 * @fn      CarNode_handleReadRemoteCapsComplete
 *
 * @brief   Handles the completion of reading remote capabilities for a
 *          specific connection.
 *          Reads and caches local T_SW capability (device-level).
 *          Caches remote T_SW capability (per-connection).
 *
 * @param   pCsReadRemoteCapsEvt - pointer to the event parameters
 *
 * @return  None
 */
void CarNode_handleReadRemoteCapsComplete(ChannelSounding_readRemoteCapabEvent_t* pCsReadRemoteCapsEvt)
{
#ifdef CS_MEASURE_DISTANCE
    if (pCsReadRemoteCapsEvt != NULL &&
        pCsReadRemoteCapsEvt->connHandle < CAR_NODE_MAX_CONNS &&
        pCsReadRemoteCapsEvt->csStatus == CS_STATUS_SUCCESS)
    {
        uint16_t connHandle = pCsReadRemoteCapsEvt->connHandle;

        // Cache remote T_SW capability from the event (per-connection)
        gSessionsDb[connHandle].remoteTsw = pCsReadRemoteCapsEvt->tSwCap;
    }
#endif
}

/*********************************************************************
 * @fn      CarNode_handleCsProcEnableComplete
 *
 * @brief   This function receives a CS Procedure Complete event and prepares the sessions
 *          DB for the upcoming procedure.
 *          The function will mark the procedure in the session DB as inactive as long as the
 *          given parameters are valid.
 *
 * @param   pProcEnableCompleteEvt - pointer to the event parameters as reported by the stack
 *
 * @return  None
 */
void CarNode_handleCsProcEnableComplete(ChannelSounding_procEnableComplete_t *pProcEnableCompleteEvt)
{
#ifdef CS_MEASURE_DISTANCE

    if (pProcEnableCompleteEvt == NULL || pProcEnableCompleteEvt->connHandle >= CAR_NODE_MAX_CONNS)
    {
        return;
    }

    if (pProcEnableCompleteEvt->enable == CS_ENABLE)
    {
        if (pProcEnableCompleteEvt->csStatus == CS_STATUS_SUCCESS)
        {
            gCsProcState = CS_PROC_STATE_ACTIVE;
            gDistanceProcessed = false;

            // Reset the Current Procedure Counter just in case
            gSessionsDb[pProcEnableCompleteEvt->connHandle].currProcedureCounter = CAR_NODE_PROC_COUNTER_IDLE;

            // Set the current selected Tx Power for this connection
            gSessionsDb[pProcEnableCompleteEvt->connHandle].selectedTxPower = pProcEnableCompleteEvt->selectedTxPower;

            // Compute and store numAntPath and finalTsw from ACI (compute once, use later)
            gSessionsDb[pProcEnableCompleteEvt->connHandle].numAntPath = CS_calcNumPaths(pProcEnableCompleteEvt->ACI);
            gSessionsDb[pProcEnableCompleteEvt->connHandle].aci = pProcEnableCompleteEvt->ACI;

            // Compute finalTsw using cached T_SW capabilities and the negotiated ACI
            gSessionsDb[pProcEnableCompleteEvt->connHandle].finalTsw = CS_GetTswByACI(
                pProcEnableCompleteEvt->ACI,
                gLocalTsw,
                gSessionsDb[pProcEnableCompleteEvt->connHandle].remoteTsw);


            // If no session is opened for this connection, open a new session
            if (gSessionsDb[pProcEnableCompleteEvt->connHandle].sessionId == CS_PROCESS_INVALID_SESSION)
            {
                uint16_t sessionId = CSProcess_OpenSession();

                if (sessionId != CS_PROCESS_INVALID_SESSION)
                {
                    gSessionsDb[pProcEnableCompleteEvt->connHandle].sessionId = sessionId;
                }
                else
                {
                    CarNode_clearSession(pProcEnableCompleteEvt->connHandle);
                }
            }
        }
        else
        {
            gCsProcState = CS_PROC_STATE_IDLE;
        }
    }
    else // CS_DISABLE - procedure ended or all repeats done
    {
        switch (pProcEnableCompleteEvt->csStatus)
        {
            case CS_STATUS_INSUFFICIENT_SECURITY:
            case CS_STATUS_INVALID_LL_PARAM:
            case CS_STATUS_FEATURE_NOT_SUPPORTED:
                gCsProcState = CS_PROC_STATE_IDLE;
                break;

            case CS_STATUS_PROCEDURE_IN_PROGRESS:
                break;

            case CS_STATUS_SUCCESS:
            case CS_STATUS_PEER_TERM:
            case CS_STATUS_HOST_TERM:
                // Procedure disabled normally. If handleDistanceResults already ran
                // while the procedure was still active (race: RAS arrived before this
                // DISABLE event), it got COMMAND_DISALLOWED and set gDistanceProcessed.
                // Retry the restart now that the stack is truly idle.
                gCsProcState = CS_PROC_STATE_IDLE;
                if (gDistanceProcessed && CarNode_getConfiguredCsRole() == CS_ROLE_INITIATOR)
                {
                    gDistanceProcessed = false;
                    CarNode_invokeProcedureEnableCmd(pProcEnableCompleteEvt->connHandle);
                }
                break;

            case CS_STATUS_INSTANT_PASSED:
            default:
                gCsProcState = CS_PROC_STATE_IDLE;
                if (CarNode_getConfiguredCsRole() == CS_ROLE_INITIATOR && gCsProcState != CS_PROC_STATE_ACTIVE)
                {
                    CS_procedureStart(pProcEnableCompleteEvt->connHandle);
                }
                break;
        }
    }

    gTimerParamConnHandle = pProcEnableCompleteEvt->connHandle;
    if (gCsWatchdogHandle == NULL)
    {
        ClockP_Params params;
        ClockP_Params_init(&params);
        params.startFlag = false;
        params.period    = 0;
        params.arg       = 0;
        gCsWatchdogHandle = ClockP_construct(&gCsWatchdogClockStruct, CarNode_timerClockFxn,
                                             CAR_NODE_WATCHDOG_TIMEOUT_TICKS, &params);
        ClockP_start(gCsWatchdogHandle);
    }
    else
    {
        ClockP_stop(gCsWatchdogHandle);
        ClockP_setTimeout(gCsWatchdogHandle, CAR_NODE_WATCHDOG_TIMEOUT_TICKS);
        ClockP_start(gCsWatchdogHandle);
    }

#endif // CS_MEASURE_DISTANCE

#ifdef RANGING_SERVER
    // Send the procedure enable complete event to the Ranging module
    ChannelSoundingTransceiver_sendResults((uint8_t*) pProcEnableCompleteEvt, sizeof(ChannelSounding_procEnableComplete_t), APP_CS_PROCEDURE_ENABLE_COMPLETE_EVENT);
#endif // RANGING_SERVER
}

/*********************************************************************
 * @fn      CarNode_handleCsSubeventResultsEvt
 *
 * @brief   This function receives CS Subevent Results event and pass  the relevant
 *          parameters to the Process module.
 *          When local results are given, it will mark that a procedure is active
 *          in the sessions DB, as long as the results are successfully added to the
 *          Process module.
 *          If adding the results to the Process module has been failed - the function
 *          will mark that a procedure is inactive in the sessions DB.
 *
 * @param   connHandle          - Connection handle associated with the reported results
 * @param   resultsSourceMode   - Indicates the source of the subevent. type @ref ChannelSounding_resultsSourceMode_e ChannelSounding_resultsSourceMode_e resultsSourceMode
 * @param   pSubeventResultsEvt - Pointer to subevent results data
 *
 * @return  TRUE    - When it is determined that the event should be sent to the upper layer
 *          FALSE   - When it is determined that the event shouldn't be sent to the upper layer
 */
bool CarNode_handleCsSubeventResultsEvt(uint16_t connHandle, ChannelSounding_resultsSourceMode_e resultsSourceMode,
                                        ChannelSounding_subeventResults_t *pSubeventResultsEvt)
{
    bool sendToExtHandler = TRUE;

#ifdef CS_MEASURE_DISTANCE
    bStatus_t status = SUCCESS;
    CarNode_procedureInfoElem_t* procedureElem = NULL;

    // Check for invalid parameters
    if (NULL == pSubeventResultsEvt ||
        resultsSourceMode > CS_RESULTS_MODE_PROP ||
        connHandle >= CAR_NODE_MAX_CONNS)
    {
        status = FAILURE;
    }

    sendToExtHandler = CAR_NODE_SEND_SUBEVENT_TO_EXT_CTRL;

    // If the given data is owned by the local device and there is no procedure currently running - Initialize relevant DB parameters
    if (status == SUCCESS)
    {
        // Now search for the procedure element using the right procedure counter
        procedureElem = CarNode_getProcedureElem(connHandle, pSubeventResultsEvt->procedureCounter);
    }

    // If the procedure element is not found, the results source mode is from the local device -
    // initiate a new procedure element as this is the first event we got for this procedure
    if (status == SUCCESS &&
        procedureElem == NULL
        && resultsSourceMode == CS_RESULTS_MODE_LOCAL)
    {
        // Allocate a new Procedure Info element
        procedureElem = (CarNode_procedureInfoElem_t*) ICall_malloc(sizeof(CarNode_procedureInfoElem_t));

        if (procedureElem != NULL)
        {
            // Initiate it
            CAR_NODE_STATE_CLEAR(procedureElem->state);
            procedureElem->timestamp                = llGetCurrentTime();
            procedureElem->procedureCounter         = pSubeventResultsEvt->procedureCounter;
            procedureElem->configId                 = pSubeventResultsEvt->configID;

            // NOTE: Procedure parameters (role, timing params) accessed via CS_GetConfiguration()
            // Session parameters (selectedTxPower, numAntPath, finalTsw) from gSessionsDb[]

            procedureElem->totalSubeventSteps       = 0;
            procedureElem->subeventStepsProcessed   = 0;
            procedureElem->procedureDoneStatus      = CS_PROCEDURE_ABORTED;
            procedureElem->subeventDoneStatus       = CS_SUBEVENT_ABORTED;
            procedureElem->remoteResultsMode        = CS_RESULTS_MODE_END;                      // Will be determined when remote results will be received
            procedureElem->pFreeRemoteResultsElemCB = NULL;                                     // Will be determined when remote results will be received
            osal_list_clearList(&procedureElem->localResults);
            osal_list_clearList(&procedureElem->remoteResults);

            // Remove old procedure elements if we reached the max size
            while(gSessionsDb[connHandle].procedureListSize >= CAR_NODE_PROCEDURE_LIST_MAX_SIZE && osal_list_empty(&gSessionsDb[connHandle].procedureList) == FALSE)
            {
                CarNode_procElemRemove(connHandle, (CarNode_procedureInfoElem_t*) osal_list_head(&gSessionsDb[connHandle].procedureList));
            }

            // Now add the new procedure element to the end of the procedures list
            osal_list_put(&gSessionsDb[connHandle].procedureList, (osal_list_elem*) procedureElem);
            gSessionsDb[connHandle].procedureListSize++;

            // Mark that results are on the way
            gSessionsDb[connHandle].currProcedureCounter = (uint32_t) pSubeventResultsEvt->procedureCounter;
        }
    }

    if (NULL != procedureElem)
    {
        CarNode_procElemAddResultsPropParams_t resultsParam;
        resultsParam.referencePowerLevel = pSubeventResultsEvt->referencePowerLevel;
        resultsParam.frequencyCompensation = pSubeventResultsEvt->frequencyCompensation;
        resultsParam.procedureDoneStatus = pSubeventResultsEvt->procedureDoneStatus;
        resultsParam.subeventDoneStatus = pSubeventResultsEvt->subeventDoneStatus;
        resultsParam.numAntennaPath = pSubeventResultsEvt->numAntennaPath;
        resultsParam.numStepsReported = pSubeventResultsEvt->numStepsReported;
        resultsParam.dataLen = pSubeventResultsEvt->dataLen;
        resultsParam.pData = pSubeventResultsEvt->data;
        status = CarNode_procElemAddResultsProp(procedureElem, resultsSourceMode, &resultsParam);

        // Post process the addition of results
        CarNode_procElemAddResultsPostProcess(status, connHandle, resultsSourceMode, procedureElem);
    }
#endif // CS_MEASURE_DISTANCE

#ifdef RANGING_SERVER
    if (resultsSourceMode == CS_RESULTS_MODE_LOCAL)
    {
        // Send the results to the Ranging module only if the results are local
        ChannelSoundingTransceiver_sendResults((uint8_t*) pSubeventResultsEvt, sizeof(ChannelSounding_subeventResults_t) + pSubeventResultsEvt->dataLen, APP_CS_SUBEVENT_RESULT);
    }
#endif // RANGING_SERVER

    return sendToExtHandler;
}

/*********************************************************************
 * @fn      CarNode_handleCsSubeventResultsEvtContEvt
 *
 * @brief   This function receives CS Subevent Results Continue event and pass the relevant
 *          parameters to the Process module, as long as there is an active procedure marked
 *          in the sessions DB.
 *          If adding the results to the Process module has been failed - the function will
 *          mark that a procedure is inactive in the sessions DB.
 *          If the results source mode is of type @ref CS_RESULTS_MODE_RAS - the results
 *          won't be processed.
 *
 * @param   connHandle          - Connection handle associated with the reported results
 * @param   resultsSourceMode   - Indicates the source of the subevent. type @ref ChannelSounding_resultsSourceMode_e
 * @param   procedureCounter    - Procedure counter of the current procedure. Only relevant for remote results.
 * @param   pSubeventResultsEvt - Pointer to subevent results data
 *
 * @return  TRUE    - When it is determined that the event should be sent to the upper layer
 *          FALSE   - When it is determined that the event shouldn't be sent to the upper layer
 */
bool CarNode_handleCsSubeventResultsEvtContEvt(uint16_t connHandle, ChannelSounding_resultsSourceMode_e resultsSourceMode,
                                               uint16_t procedureCounter, ChannelSounding_subeventResultsContinue_t *pSubeventResultsContEvt)
{
    bool sendToExtHandler = TRUE;

#ifdef CS_MEASURE_DISTANCE
    bStatus_t status = SUCCESS;
    CarNode_procedureInfoElem_t* procedureElem = NULL;

    if (pSubeventResultsContEvt == NULL || resultsSourceMode > CS_RESULTS_MODE_PROP)
    {
        status = FAILURE;
    }

    sendToExtHandler = CAR_NODE_SEND_SUBEVENT_TO_EXT_CTRL;

    // Procedure counter to be used when searching the Procedure element
    uint16_t procedureCounterToSearch = procedureCounter;

    if (status == SUCCESS && connHandle < CAR_NODE_MAX_CONNS)
    {
        // If processing local results, search for the procedure with the Procedure Counter saved in the DB
        // (since we don't have a Procedure Counter reported in this event)
        if (resultsSourceMode == CS_RESULTS_MODE_LOCAL)
        {
            // Take only the lower 16 bits since the rest are for indicating an invalid value @ref CAR_NODE_PROC_COUNTER_IDLE
            procedureCounterToSearch = (uint16_t) gSessionsDb[connHandle].currProcedureCounter;
        }

        // Search for the procedure element in the DB
        procedureElem = CarNode_getProcedureElem(connHandle, procedureCounterToSearch);
    }

    // If found a procedure element, add the results to it
    if (NULL != procedureElem)
    {
        CarNode_procElemAddResultsPropParams_t resultsParam;
        resultsParam.referencePowerLevel = CS_INVALID_TX_POWER;
        resultsParam.frequencyCompensation = 0;
        resultsParam.procedureDoneStatus = pSubeventResultsContEvt->procedureDoneStatus;
        resultsParam.subeventDoneStatus = pSubeventResultsContEvt->subeventDoneStatus;
        resultsParam.numAntennaPath = pSubeventResultsContEvt->numAntennaPath;
        resultsParam.numStepsReported = pSubeventResultsContEvt->numStepsReported;
        resultsParam.dataLen = pSubeventResultsContEvt->dataLen;
        resultsParam.pData = pSubeventResultsContEvt->data;
        status = CarNode_procElemAddResultsProp(procedureElem, resultsSourceMode, &resultsParam);

        // Post process the addition of the results
        CarNode_procElemAddResultsPostProcess(status, connHandle, resultsSourceMode, procedureElem);
    }
#endif // CS_MEASURE_DISTANCE

#ifdef RANGING_SERVER
  if(resultsSourceMode == CS_RESULTS_MODE_LOCAL)
  {
      // Send the results to the Ranging module only if the results are local
      ChannelSoundingTransceiver_sendResults((uint8_t*) pSubeventResultsContEvt, sizeof(ChannelSounding_subeventResultsContinue_t) + pSubeventResultsContEvt->dataLen, APP_CS_SUBEVENT_CONTINUE_RESULT);
  }
#endif // RANGING_SERVER

    return sendToExtHandler;
}

#ifdef RANGING_CLIENT

#ifdef CS_MEASURE_DISTANCE

/*********************************************************************
 * @fn      CarNode_freeRemoteResultsElemRAS
 *
 * @brief   This function frees a ranging DB results element.
 *          It is called when providing it as a parameter to @ref CarNode_clearResultsList.
 *
 * @param   pResultsElem - Pointer to the results element to be freed.
 *
 * @return  None
 */
void CarNode_freeRemoteResultsElemRAS(CarNode_resultsElem_t* pResultsElem)
{
    if (pResultsElem != NULL)
    {
        CarNode_resultsElemDataRAS_t* resultsData = (CarNode_resultsElemDataRAS_t*) pResultsElem->resultsData;

        // Free all of the segments in the segments reader
        RangingDBClient_freeSegmentsReader(&resultsData->segmentsReader);

        // Free the results element itself
        ICall_free(pResultsElem);
    }
}

/*********************************************************************
 * @fn      CarNode_procElemAddResultsRAS
 *
 * @brief   This function adds a ranging DB results to a given procedure element.
 *          It updates the procedure state to indicate that remote results are ready.
 *
 * @param   pProcedureElem  - Pointer to the procedure element to add the results to.
 * @param   segmentsReader  - segments reader structure
 *
 * @return  FAILURE - If the given parameters are invalid
 *                    If the procedure element already has remote results ready
 *                    If memory allocation failed
 *          SUCCESS - If the results were added successfully to the procedure element.
 */
bStatus_t CarNode_procElemAddResultsRAS(CarNode_procedureInfoElem_t* pProcedureElem, RangingDBClient_procedureSegmentsReader_t segmentsReader)
{
    bStatus_t status = SUCCESS;

    // Ensure valid parameters and that we don't add data for a list which already has results ready
    if (pProcedureElem == NULL ||
        CAR_NODE_IS_RESULTS_READY(CS_RESULTS_MODE_RAS, pProcedureElem->state))
    {
        status = FAILURE;
    }

    if (status == SUCCESS)
    {
        // Update the procedure's Remote Results mode to RAS
        pProcedureElem->remoteResultsMode = CS_RESULTS_MODE_RAS;

        // Set the free callback for the RAS results element
        pProcedureElem->pFreeRemoteResultsElemCB = CarNode_freeRemoteResultsElemRAS;

        // Allocate a new results element
        CarNode_resultsElem_t* resultsElem = (CarNode_resultsElem_t*) ICall_malloc(sizeof(CarNode_resultsElem_t) + sizeof(CarNode_resultsElemDataRAS_t));

        // If failed to allocate memory for the results element
        if (NULL == resultsElem)
        {
            status = FAILURE;
        }
        else
        {
            // Initiate the results element data
            CarNode_resultsElemDataRAS_t* resultsData = (CarNode_resultsElemDataRAS_t*) resultsElem->resultsData;

            // Copy the segment reader
            resultsData->segmentsReader = segmentsReader;

            // Add the results element to the end of the relevant procedure's results list
            osal_list_put(&pProcedureElem->remoteResults, (osal_list_elem*) resultsElem);

            // Update procedure state flags
            CAR_NODE_STATE_SET_FLAG(pProcedureElem->state, CAR_NODE_STATE_REMOTE_DATA_READY);
        }
    }
    return status;
}
#endif // CS_MEASURE_DISTANCE

/*********************************************************************
 * @fn      CarNode_RREQ_EventHandler
 *
 * @brief   Callback function to be called when the RAS client profile
 *          receives a ranging results.
 *          If measuring distance, this function adds the ranging results
 *          to the relevant procedure element.
 *          If not measuring distance, this function frees the results without
 *          processing them.
 *
 * @param   connHandle     - Connection handle related to the reported results
 * @param   rangingCount   - Ranging procedure counter
 * @param   rangingStatus  - Status of the ranging DB results
 * @param   segmentsReader - Segments reader structure
 *
 * @return  None
 */
void CarNode_RREQ_EventHandler(uint16_t connHandle, uint16_t rangingCount, uint8_t rangingStatus, RangingDBClient_procedureSegmentsReader_t segmentsReader)
{
    bStatus_t status = (bStatus_t) rangingStatus;
    const struct csConfig_t* pConfig;

    // If the connection handle is invalid, free the segments if needed and return
    if (connHandle > CAR_NODE_MAX_CONNS)
    {
        if (status == SUCCESS)
        {
            RangingDBClient_freeSegmentsReader(&segmentsReader);
        }

        return;
    }

#ifdef CS_MEASURE_DISTANCE
    CarNode_procedureInfoElem_t* pProcedureElem = CarNode_getProcedureElem(connHandle, rangingCount);

    // If the status is not SUCCESS and procedureElem exist - remove it.
    // No need to free the segment reader in this case
    if (status != SUCCESS && pProcedureElem != NULL)
    {
        CarNode_procElemRemove(connHandle, pProcedureElem);
    }

    if (status == SUCCESS)
    {
        // If not found, consider it a failure
        if (pProcedureElem == NULL)
        {
            status = FAILURE;
        }

        if (status == SUCCESS)
        {
            pConfig = CS_GetConfiguration(connHandle, pProcedureElem->configId);
            if (pConfig == NULL)
            {
                status = FAILURE;
            }
        }

        if (status == SUCCESS)
        {
#if ( CAR_NODE_SEND_SUBEVENT_TO_EXT_CTRL )
            // Send all RAS subevents data to extctrl
            // Note: should send the data before post-process as it might delete the segments
            CarNode_sendRASDataToExtCtrl(connHandle, CS_GET_OPPOSITE_ROLE(pConfig->role), segmentsReader);
#endif // CAR_NODE_SEND_SUBEVENT_TO_EXT_CTRL

            // Add the results to the procedure element
            status = CarNode_procElemAddResultsRAS(pProcedureElem, segmentsReader);

            // Post process the addition of results.
            // Note that if the results failed to be added, this function won't free the segments reader
            CarNode_procElemAddResultsPostProcess(status, connHandle, CS_RESULTS_MODE_RAS, pProcedureElem);
        }

        if (status != SUCCESS)
        {
            // Failed to add the results - free the segments reader
            RangingDBClient_freeSegmentsReader(&segmentsReader);
        }
    }

#else
    // If we are not measuring distance, free the segments reader directly
    if (status == SUCCESS)
    {
        RangingDBClient_freeSegmentsReader(&segmentsReader);
    }
#endif // !CS_MEASURE_DISTANCE
}

#endif // RANGING_CLIENT

#ifdef CS_MEASURE_DISTANCE
/*********************************************************************
 * @fn      CarNode_procElemAddResultsBTCS
 *
 * @brief   This function adds a subevent results element to a given procedure
 *          element. It only handles results for the BTCS modes.
 *
 * @param   pProcedureElem - Pointer to the procedure element to add the results to
 * @param   pSubeventData - Pointer to the subevent steps data
 * @param   dataLen - Length of the given pointer
 * @param   numStepsReported - Number of steps reported for this subevent
 * @param   procedureDoneStatus - Procedure Done Status of this subevent
 * @param   subeventDoneStatus - Subevent Done status
 *
 * @return  FAILURE - If memory allocation failed
 * @return  SUCCESS - Results added successfully
 */
bStatus_t CarNode_procElemAddResultsBTCS(CarNode_procedureInfoElem_t* pProcedureElem, uint8_t* pSubeventData, uint16_t dataLen,
                                         uint8_t numStepsReported, uint8_t procedureDoneStatus, uint8_t subeventDoneStatus)
{
    bStatus_t status = SUCCESS;

    // Update the procedure's Remote Results mode to BTCS
    pProcedureElem->remoteResultsMode = CS_RESULTS_MODE_BTCS;

    // Allocate a new results element
    uint16_t resultsElemSize = sizeof(CarNode_resultsElem_t) + sizeof(CarNode_resultsElemDataBTCS_t) + dataLen;
    CarNode_resultsElem_t* resultsElem = (CarNode_resultsElem_t*) ICall_malloc(resultsElemSize);

    // If failed to allocate memory for the results element
    if (NULL == resultsElem)
    {
        status = FAILURE;
    }
    else
    {
        // Initiate the results element data
        CarNode_resultsElemDataBTCS_t* resultsElemData = (CarNode_resultsElemDataBTCS_t*) resultsElem->resultsData;

        // Copy the data
        resultsElemData->procedureDoneStatus = procedureDoneStatus;
        resultsElemData->subeventDoneStatus  = subeventDoneStatus;
        resultsElemData->numStepsReported    = numStepsReported;
        resultsElemData->dataLen             = dataLen;
        memcpy(resultsElemData->data, pSubeventData, dataLen);

        // Add the results element to the end of the relevant procedure's results list
        osal_list_put(&pProcedureElem->remoteResults, (osal_list_elem*) resultsElem);
    }

    // If procedure done or aborted - mark that the remote data is ready
    if(procedureDoneStatus == CS_PROCEDURE_DONE || procedureDoneStatus == CS_PROCEDURE_ABORTED)
    {
        CAR_NODE_STATE_SET_FLAG(pProcedureElem->state, CAR_NODE_STATE_REMOTE_DATA_READY);
    }

    return status;
}

/*********************************************************************
 * @fn      CarNode_procElemHandleBTCSSubevent
 *
 * @brief   This function adds a BTCS subevent results element to a
 *          given procedure element.
 *
 * @param   connHandle - Connection handle for accessing session DB
 * @param   pProcedureElem - Pointer to the procedure element to add the results to
 * @param   pSubeventHdr - Pointer to a header of the Subevent.
 *                         One of the following types:
 *                         @ref BTCS_SubeventHdrRefl_t
 *                         @ref BTCS_SubeventHdrInit_t
 * @param   dataLen - Length of the given pointer (subevent header + steps data)
 *
 * @return  SUCCESS - The subevent results were added successfully.
 * @return  FAILURE - Failed to parse the subevent header
 */
bStatus_t CarNode_procElemHandleBTCSSubevent(uint16_t connHandle, CarNode_procedureInfoElem_t* pProcedureElem, uint8_t* pSubeventHdr, uint16_t dataLen)
{
    bStatus_t status = SUCCESS;

    // Get const pointer to CS configuration for this procedure
    const struct csConfig_t* pConfig = CS_GetConfiguration(connHandle, pProcedureElem->configId);

    if (pConfig == NULL)
    {
        // Config not found - should not happen
        return FAILURE;
    }

    // role of the remote device (opposite of local role)
    uint8_t role = CS_GET_OPPOSITE_ROLE(pConfig->role);

    // Parameters to be set according to the remote role, will be passed to the next handler
    uint8_t* subeventData;
    uint8_t totalSubeventSteps;
    uint8_t numStepsReported;
    uint8_t procedureDoneStatus;
    uint8_t subeventDoneStatus;
    uint8_t subeventHdrLen;
    uint8_t subeventDataLen;

    if (role == CS_ROLE_INITIATOR)
    {
        BTCS_SubeventHdrInit_t* subeventHdrInit = (BTCS_SubeventHdrInit_t*)pSubeventHdr;

        totalSubeventSteps = subeventHdrInit->totalSubeventSteps;
        numStepsReported = subeventHdrInit->numStepsReported;
        procedureDoneStatus = BTCS_EXTRACT_PROCDONE_STATUS(subeventHdrInit->status);
        subeventDoneStatus = BTCS_EXTRACT_SUBEVENTDONE_STATUS(subeventHdrInit->status);

        subeventData = subeventHdrInit->data;
        subeventHdrLen = sizeof(BTCS_SubeventHdrInit_t);
    }
    else
    {
        BTCS_SubeventHdrRefl_t* subeventHdrRefl = (BTCS_SubeventHdrRefl_t*)pSubeventHdr;

        totalSubeventSteps = subeventHdrRefl->totalSubeventSteps;
        numStepsReported = subeventHdrRefl->numStepsReported;
        procedureDoneStatus = BTCS_EXTRACT_PROCDONE_STATUS(subeventHdrRefl->status);
        subeventDoneStatus = BTCS_EXTRACT_SUBEVENTDONE_STATUS(subeventHdrRefl->status);

        subeventData = subeventHdrRefl->data;
        subeventHdrLen = sizeof(BTCS_SubeventHdrRefl_t);
    }

    // Compare the given length against the header length before subtracting
    if (dataLen >= subeventHdrLen)
    {
        subeventDataLen = dataLen - subeventHdrLen;
    }
    else
    {
        status = FAILURE;
    }

    if (status == SUCCESS)
    {
        // Initiate DB steps counters since its a new subevent
        pProcedureElem->totalSubeventSteps = totalSubeventSteps;   // Save the total steps expected for this subevent
        pProcedureElem->subeventStepsProcessed = numStepsReported; // Save the number of steps processed for this subevent
        pProcedureElem->procedureDoneStatus = procedureDoneStatus; // Save procedureDoneStatus for later
        pProcedureElem->subeventDoneStatus = subeventDoneStatus;   // Save subeventDoneStatus for later

        // If one of the statuses type is 'Aborted' - we will leave it as is and let the CS Process module handle it.
        // Otherwise:
        // 1. If the sum of this and all previous NSR values reported for this subevent is less than the
        //    totalSubeventSteps then additional BTCS_SubeventContHdr_t with more data may be sent, therefore
        //    set the subeventDoneStatus to 'Active'
        // 2. Else - leave both statuses as it came from the BTCS message
        if (pProcedureElem->procedureDoneStatus != CS_PROCEDURE_ABORTED &&
            pProcedureElem->subeventDoneStatus != CS_SUBEVENT_ABORTED &&
            pProcedureElem->totalSubeventSteps > pProcedureElem->subeventStepsProcessed)
        {
            procedureDoneStatus = CS_PROCEDURE_ACTIVE;
            subeventDoneStatus = CS_SUBEVENT_ACTIVE;
        }

        status = CarNode_procElemAddResultsBTCS(pProcedureElem, subeventData, subeventDataLen, numStepsReported,
                                                procedureDoneStatus, subeventDoneStatus);
    }

    return status;
}

/*********************************************************************
 * @fn      CarNode_procElemHandleBTCSSubeventCont
 *
 * @brief   This function adds a BTCS subevent continue results element to a
 *          given procedure element.
 *
 * @param   connHandle - Connection handle for accessing session DB
 * @param   pProcedureElem - Pointer to the procedure element to add the results to
 * @param   pSubeventContHdr - Pointer to a header of the Subevent Continue,
 *                            should contain the subevent data.
 * @param   dataLen - Length of the given pointer (subevent header + steps data)
 *
 * @return  SUCCESS - The subevent results were added successfully.
 * @return  FAILURE - Failed to parse the subevent header
 */
bStatus_t CarNode_procElemHandleBTCSSubeventCont(uint16_t connHandle, CarNode_procedureInfoElem_t* pProcedureElem, BTCS_SubeventContHdr_t* pSubeventContHdr, uint16_t dataLen)
{
    bStatus_t status = SUCCESS;

    // Consider a procedureDoneStatus and subeventDoneStatus as active
    uint8_t procedureDoneStatus = CS_PROCEDURE_ACTIVE;
    uint8_t subeventDoneStatus = CS_SUBEVENT_ACTIVE;

    // If these are the final steps of the subevent - send the real statuses to CS Process module
    if (pProcedureElem->totalSubeventSteps == pProcedureElem->subeventStepsProcessed + pSubeventContHdr->numStepsReported)
    {
        procedureDoneStatus = pProcedureElem->procedureDoneStatus;
        subeventDoneStatus = pProcedureElem->subeventDoneStatus;
    }

    // Consider steps as processed, and if a failure will occur, we will reset the relevant data later
    pProcedureElem->subeventStepsProcessed += pSubeventContHdr->numStepsReported;

    // Compare the given length against the header length before subtracting
    if (dataLen < sizeof(BTCS_SubeventContHdr_t))
    {
        status = FAILURE;
    }

    if (status == SUCCESS)
    {
        status = CarNode_procElemAddResultsBTCS(pProcedureElem, pSubeventContHdr->data, dataLen - sizeof(BTCS_SubeventContHdr_t),
                                                pSubeventContHdr->numStepsReported, procedureDoneStatus, subeventDoneStatus);
    }

    return status;
}

/*********************************************************************
 * @fn      CarNode_procElemHandleResultsBTCS
 *
 * @brief   This function handles a BTCS message that contains subevent
 *          results. It adds the results to the given procedure element.
 *
 * @param   connHandle - Connection handle for accessing session DB
 * @param   pProcedureElem - Pointer to the procedure element to add the results to
 * @param   msgId - Message ID of types: @ref BTCS_RANGING_PROCEDURE_RESULTS_START
 *                  or @ref BTCS_RANGING_PROCEDURE_RESULTS_CONTINUE
 * @param   pMsgData - Pointer to a message data of types: @ref BTCS_ProcHdr_t
 *                     or @ref BTCS_ProcContHdr_t
 * @param   dataLen - Length of the given pointer
 *
 * @return  SUCCESS - The subevent results were added successfully.
 * @return  FAILURE - If remote results are already marked as ready for this procedure
 *                  - Failed to parse the subevent header
 */
bStatus_t CarNode_procElemHandleResultsBTCS(uint16_t connHandle, CarNode_procedureInfoElem_t* pProcedureElem, uint8_t msgId, uint8_t* pMsgData, uint16_t dataLen)
{
    bStatus_t status = SUCCESS;

    // Ensure valid parameters and that we don't add data for a list which already has results ready
    if (pProcedureElem == NULL || pMsgData == NULL ||
        CAR_NODE_IS_RESULTS_READY(CS_RESULTS_MODE_BTCS, pProcedureElem->state))
    {
        status = FAILURE;
    }

    uint8_t* subeventHdr; // Might be subevent or subeventCont
    uint16_t subeventDataLen;
    bool isSubeventCont = FALSE;

    if (status == SUCCESS && msgId == BTCS_RANGING_PROCEDURE_RESULTS_START)
    {
        // If a subevent is currently in process or the data length is not valid
        if (pProcedureElem->subeventStepsProcessed != 0 ||
            dataLen < sizeof(BTCS_ProcHdr_t))
        {
            // Consider failure because we are not expecting this message in the middle of a subevent processing
            status = FAILURE;
        }
        else
        {
            // Set subevent header using @ref BTCS_ProcHdr_t and subtract its data length
            subeventHdr = ((BTCS_ProcHdr_t*)pMsgData)->data;
            subeventDataLen = dataLen - sizeof(BTCS_ProcHdr_t);
        }
    }
    else if (status == SUCCESS)
    {
        // If a subevent is currently in process or the data length is not valid
        if (dataLen < sizeof(BTCS_ProcContHdr_t))
        {
            status = FAILURE;
        }
        else
        {
            // Set subevent header using @ref BTCS_ProcContHdr_t and subtract its data length
            subeventHdr = ((BTCS_ProcContHdr_t*)pMsgData)->data;
            subeventDataLen = dataLen - sizeof(BTCS_ProcContHdr_t);

            // If a subevent is currently in process, consider this message as a continuation of the subevent
            if (pProcedureElem->subeventStepsProcessed > 0)
            {
                isSubeventCont = TRUE;
            }
        }
    }

    if (status == SUCCESS)
    {
        if (isSubeventCont == FALSE)
        {
            status = CarNode_procElemHandleBTCSSubevent(connHandle, pProcedureElem, subeventHdr, subeventDataLen);
        }
        else
        {
            status = CarNode_procElemHandleBTCSSubeventCont(connHandle, pProcedureElem, (BTCS_SubeventContHdr_t*)subeventHdr, subeventDataLen);
        }
    }

    // TODO: Loop over the message for additional subevents if needed

    return status;
}

/*********************************************************************
 * @fn      CarNode_handleBTCSMsg
 *
 * @brief   Handles BTCS messages.
 *
 * @param   connHandle - Connection handle associated with the reported results
 * @param   msgId - One of the following:
 *                  @ref BTCS_RANGING_PROCEDURE_RESULTS_START
 *                  @ref BTCS_RANGING_PROCEDURE_RESULTS_CONTINUE
 * @param   pMsgData - Message data, matches the message id
 * @param   dataLen - Length of the given message data
 *
 * @return  None
 */
void CarNode_handleBTCSMsg(uint16_t connHandle, uint8_t msgId, uint8_t* pMsgData, uint16_t dataLen)
{
    bStatus_t status = SUCCESS;
    CarNode_procedureInfoElem_t* pProcedureElem = NULL;
    uint8_t procedureCounter;

    if (pMsgData != NULL && connHandle < CAR_NODE_MAX_CONNS &&
        (msgId == BTCS_RANGING_PROCEDURE_RESULTS_START || msgId == BTCS_RANGING_PROCEDURE_RESULTS_CONTINUE))
    {
        // Get squence number (procedure counter)
        procedureCounter = pMsgData[0];

        // Search for the procedure associated with the procedure counter in the list
        pProcedureElem = CarNode_getProcedureElem(connHandle, (uint16_t) procedureCounter);

        if (NULL != pProcedureElem)
        {
            // Add the results to the procedure element
            status = CarNode_procElemHandleResultsBTCS(connHandle, pProcedureElem, msgId, pMsgData, dataLen);

            // Post process the addition of results
            CarNode_procElemAddResultsPostProcess(status, connHandle, CS_RESULTS_MODE_BTCS, pProcedureElem);
        }
    }
}

/*********************************************************************
 * @fn      CarNode_handleDKMsg
 *
 * @brief   Handles Digital Key messages.
 *
 * @param   connHandle - Connection handle associated with the reported results
 * @param   pDKMessage - Pointer to DK message
 *
 * @return  None
 */
void CarNode_handleDKMsg(uint16_t connHandle, DK_Message_t* pDKMessage)
{
    if (NULL != pDKMessage)
    {
        if (pDKMessage->msgHdr.msgType == BTCS_RANGING_SERVICE_MESSAGE_TYPE)
        {
            uint16_t msgLen = SWAP_BYTES16(pDKMessage->msgLen);

            CarNode_handleBTCSMsg(connHandle, pDKMessage->payloadHdr, pDKMessage->payloadData, msgLen);
        }
    }
}

#endif

/*********************************************************************
 * @fn      CarNode_csEvtHandler
 *
 * @brief   This function handles @ref BLEAPPUTIL_CS_EVENT_CODE events.
 *
 * @param   pCsEvt - Pointer to CS event
 *
 * @return  None
 */
void CarNode_csEvtHandler(csEvtHdr_t *pCsEvt)
{
  uint8_t opcode = pCsEvt->opcode;
  uint8_t sendToExtHandler = TRUE;

  switch( opcode )
  {
    case CS_READ_REMOTE_SUPPORTED_CAPABILITIES_COMPLETE_EVENT:
    {
      CarNode_handleReadRemoteCapsComplete((ChannelSounding_readRemoteCapabEvent_t*) pCsEvt);

      ChannelSounding_readRemoteCapabEvent_t *pCapsEvt = (ChannelSounding_readRemoteCapabEvent_t *)pCsEvt;
      if (pCapsEvt->csStatus == CS_STATUS_SUCCESS &&
          CarNode_getConfiguredCsRole() == CS_ROLE_INITIATOR)
      {
        csDefaultSettings.connHandle = pCapsEvt->connHandle;
        bStatus_t csSettingsStatus = CS_SetDefaultSettings(&csDefaultSettings);
        if (csSettingsStatus == SUCCESS)
        {
          csConfigParams.connHandle = pCapsEvt->connHandle;
          CS_CreateConfig(&csConfigParams);
        }
      }
      break;
    }

    case CS_CONFIG_COMPLETE_EVENT:
    {
      CS_configCompleteEvt_t *pConfigEvt = (CS_configCompleteEvt_t *)pCsEvt;
      if (pConfigEvt->csStatus == CS_STATUS_SUCCESS &&
          connectionRole == GAP_PROFILE_CENTRAL)
      {
        csSetProcedureParams.connHandle = ((CS_configCompleteEvt_t *)pCsEvt)->connHandle;
        ChannelSounding_securityEnableCmdParams_t secParams;
        secParams.connHandle = pConfigEvt->connHandle;
        ChannelSounding_securityEnable(&secParams);
      }
      break;
    }

    case CS_READ_REMOTE_FAE_TABLE_COMPLETE_EVENT:
    {
      break;
    }

    case CS_SECURITY_ENABLE_COMPLETE_EVENT:
    {
      CS_securityEnableCompleteEvt_t *pSecEvt = (CS_securityEnableCompleteEvt_t *)pCsEvt;

      if (pSecEvt->csStatus == CS_STATUS_SUCCESS &&
          CarNode_getConfiguredCsRole() == CS_ROLE_INITIATOR)
      {
        csStatus_e status = CS_SetProcedureParameters(&csSetProcedureParams);
        if(status == CS_STATUS_SUCCESS)
        {
            CS_procedureStart(pSecEvt->connHandle);
        }
      }
      break;
    }

    case CS_PROCEDURE_ENABLE_COMPLETE_EVENT:
    {
      // Call Procedure Enable Complete handler
      CarNode_handleCsProcEnableComplete((ChannelSounding_procEnableComplete_t *) pCsEvt);
      break;
    }

    case CS_SUBEVENT_RESULT:
    {
      ChannelSounding_subeventResults_t* subeventResultsEvt = (ChannelSounding_subeventResults_t *) pCsEvt;

      sendToExtHandler = CarNode_handleCsSubeventResultsEvt(subeventResultsEvt->connHandle, CS_RESULTS_MODE_LOCAL, subeventResultsEvt);
      break;
    }

    case CS_SUBEVENT_CONTINUE_RESULT:
    {
      ChannelSounding_subeventResultsContinue_t* subeventResultsContEvt = (ChannelSounding_subeventResultsContinue_t *) pCsEvt;

      sendToExtHandler = CarNode_handleCsSubeventResultsEvtContEvt(subeventResultsContEvt->connHandle, CS_RESULTS_MODE_LOCAL,
                                                                   0xFFFF, // Procedure Counter is not used in local results
                                                                   subeventResultsContEvt);
    // {
    // /*
    //  *    Embedded CS-Handover sequence TODO:
    //  *        1. Uncomment this block
    //  *
    //  *      The purpose of this section is to re-enable Channel Sounding procedure
    //  *      automatically after it has been aborted.
    //  */
    //   if (subeventResultsContEvt->procedureDoneStatus == CS_PROCEDURE_ABORTED)
    //   {
    //     ChannelSounding_setProcedureEnableCmdParams_t csEnableParams;
    //     csEnableParams.connHandle = subeventResultsContEvt->connHandle;
    //     csEnableParams.configID = 0;
    //     csEnableParams.enable = CS_ENABLE;
    //     ChannelSounding_procedureEnable(&csEnableParams);
    //   }
    // }
      break;
    }

    default:
    {
      break;
    }
  }

  if (sendToExtHandler == TRUE)
  {
    // Send event to the upper layer
    CarNode_extEvtHandler(BLEAPPUTIL_CS_TYPE, BLEAPPUTIL_CS_EVENT_CODE, (BLEAppUtil_msgHdr_t *) pCsEvt);
  }
}

/*********************************************************************
 * @fn      CarNode_transceiverEventHandler
 *
 * @brief   This function handles the events raised from the transceiver
 *          module.
 *
 * @param   eventType - The type of the events @ref BLEAppUtil_eventHandlerType_e.
 * @param   event     - Message event.
 * @param   pMsgData  - Pointer to message data.
 *
 * @return  None
 */
void CarNode_transceiverEventHandler(BLEAppUtil_eventHandlerType_e eventType,
                                     uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if ( pMsgData != NULL )
  {
    switch ( eventType )
    {
      case BLEAPPUTIL_L2CAP_DATA_TYPE:
      {
         l2capDataEvent_t *pDataPkt = (l2capDataEvent_t *)pMsgData;
         uint8* pPayload = pDataPkt->pkt.pPayload;
         if (pPayload != NULL)
         {
              uint8_t opcode = ((csEvtHdr_t*) pPayload)->opcode;
              uint8_t sendToExtHandler = FALSE;

              switch(opcode)
              {
                  case CS_SUBEVENT_RESULT:
                  {
                    // Call results handler
                    sendToExtHandler = CarNode_handleCsSubeventResultsEvt(pDataPkt->pkt.connHandle, CS_RESULTS_MODE_PROP,
                                                                          (ChannelSounding_subeventResults_t *) pPayload);
                    break;
                  }

                  case CS_SUBEVENT_CONTINUE_RESULT:
                  {
                    ChannelSounding_subeventResultsContinueExt_t* pAppSubeventResultsContExt =
                                                    (ChannelSounding_subeventResultsContinueExt_t *) pPayload;

                    // Calculate the size of the unextended event
                    uint16_t resultsSize = sizeof(ChannelSounding_subeventResultsContinue_t) + pAppSubeventResultsContExt->dataLen;

                    // Allocate memory for the unextended event
                    ChannelSounding_subeventResultsContinue_t* pAppSubeventResultsCont =
                                                    (ChannelSounding_subeventResultsContinue_t*)ICall_malloc(resultsSize);

                    // Check if memory allocation was successful, fill the unextended event and call the results continue handler
                    if (NULL != pAppSubeventResultsCont)
                    {
                        pAppSubeventResultsCont->csEvtOpcode           = pAppSubeventResultsContExt->csEvtOpcode;
                        pAppSubeventResultsCont->connHandle            = pAppSubeventResultsContExt->connHandle;
                        pAppSubeventResultsCont->configID              = pAppSubeventResultsContExt->configID;
                        pAppSubeventResultsCont->procedureDoneStatus   = pAppSubeventResultsContExt->procedureDoneStatus;
                        pAppSubeventResultsCont->subeventDoneStatus    = pAppSubeventResultsContExt->subeventDoneStatus;
                        pAppSubeventResultsCont->abortReason           = pAppSubeventResultsContExt->abortReason;
                        pAppSubeventResultsCont->numAntennaPath        = pAppSubeventResultsContExt->numAntennaPath;
                        pAppSubeventResultsCont->numStepsReported      = pAppSubeventResultsContExt->numStepsReported;
                        pAppSubeventResultsCont->dataLen               = pAppSubeventResultsContExt->dataLen;

                        // Copy subevent data
                        memcpy(pAppSubeventResultsCont->data, pAppSubeventResultsContExt->data, pAppSubeventResultsContExt->dataLen);

                        // Call results continue handler with extended parameters separated
                        sendToExtHandler = CarNode_handleCsSubeventResultsEvtContEvt(pDataPkt->pkt.connHandle, CS_RESULTS_MODE_PROP,
                                                                                     pAppSubeventResultsContExt->procedureCounter,
                                                                                     pAppSubeventResultsCont);

                        // If the results continue handler has returned TRUE, we will send unextended event to the upper layer
                        if (sendToExtHandler == TRUE)
                        {
                            CarNode_extEvtHandler(BLEAPPUTIL_CS_TYPE, BLEAPPUTIL_CS_EVENT_CODE, (BLEAppUtil_msgHdr_t *) pAppSubeventResultsCont);

                            // Set the sendToExtHandler to FALSE so we won't send the extended event
                            sendToExtHandler = FALSE;
                        }

                        ICall_free(pAppSubeventResultsCont);
                    }

                    break;
                  }

                  default:
                  {
                    break;
                  }
              }

              if (sendToExtHandler == TRUE)
              {
                // Send event to the upper layer
                CarNode_extEvtHandler(BLEAPPUTIL_CS_TYPE, BLEAPPUTIL_CS_EVENT_CODE, (BLEAppUtil_msgHdr_t *) pPayload);
              }

              osal_bm_free(pPayload);
         }

        break;
      }

      default:
        break;
    }
  }
}

/*********************************************************************
 * @fn      CarNode_connEventHandler
 *
 * @brief   The purpose of this function is to handle connection-related
 *          events that arise from the GAP and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - Message event.
 * @param   pMsgData - Pointer to message data.
 *
 * @return  None
 */
void CarNode_connEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    if (pMsgData == NULL)
    {
        return;
    }

    switch(event)
    {
        case BLEAPPUTIL_LINK_ESTABLISHED_EVENT:
        {
            gapEstLinkReqEvent_t *pGapEstMsg = (gapEstLinkReqEvent_t *)pMsgData;
            cyclic_handle = pGapEstMsg->connectionHandle;
            connectionRole = pGapEstMsg->connRole;
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
            CarNode_advstop();
            //CarNode_adv();
#endif
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
            Central_scanStop();
#endif
            UART2_write(uart, "\r\nconnected\r\n", 13, NULL);
            break;
        }

        case BLEAPPUTIL_LINK_TERMINATED_EVENT:
        {
#ifdef CS_MEASURE_DISTANCE
            gapTerminateLinkEvent_t *gapTermMsg = (gapTerminateLinkEvent_t *)pMsgData;

            // If a session is opened in the CS Process module - Clear the relevant data
            if (gapTermMsg->connectionHandle < CAR_NODE_MAX_CONNS &&
                gSessionsDb[gapTermMsg->connectionHandle].sessionId != CS_PROCESS_INVALID_SESSION)
            {
                {
                //     /*
                //     *   Embedded CS-Handover sequence TODO:
                //     *       1. Uncomment this block
                //     *
                //     *      The purpose of this section is to handle distance calculation
                //     *      before closing the session in CS Process module.
                //     */
                //     // Before closing the connection, calculate the distance if pending
                    CarNode_handleCSProcessDistancePendingStatus(gapTermMsg->connectionHandle);
                }

                // Close the session in the CS Process module
                CSProcess_CloseSession(gSessionsDb[gapTermMsg->connectionHandle].sessionId);

                // Clear the session data
                CarNode_clearSession(gapTermMsg->connectionHandle);

                // Clear configuration parameters
                CarNode_clearConfigurationParams(gapTermMsg->connectionHandle);
            }


            // Reset CS procedure state
            gCsProcState = CS_PROC_STATE_IDLE;
            gDistanceProcessed = false;

            // Stop watchdog timer
            if (gCsWatchdogHandle != NULL)
            {
                ClockP_stop(gCsWatchdogHandle);
            }

            // Erase bonds to force fresh pairing on reconnection
            //GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, 0, NULL);

            cyclic_handle = 0xFFFF;

            {
                char b[40] = {0};
                sprintf(b, "disconnected reason=%u\r\n", (unsigned)gapTermMsg->reason);
                UART2_write(uart, b, sizeof(b), NULL);
            }
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
            CarNode_adv();
#endif
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG ) )
            //AppPadvTimeSync_onCnEstablished();
#endif
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
            CarNode_startScan();
#endif
#endif
            break;
        }

        default:
        {
            break;
        }
    }
}

#if defined(RANGING_CLIENT) && defined(CS_MEASURE_DISTANCE)
/*********************************************************************
 * @fn      CarNode_sendRASDataToExtCtrl
 *
 * @brief   The purpose of this function is to handle the sending of
 *          RAS data to the upper layer when in distance measurement mode,
 *          by extracting the relevant data from the segments reader and
 *          sending it through the extctrl event handler.
 *
 * @param   connHandle - Connection handle associated with the reported results.
 * @param   role - CS role of the local device for the procedure.
 * @param   segmentsReader - Segments reader structure.
 *
 * @return  None.
 *
 */
void CarNode_sendRASDataToExtCtrl(uint16_t connHandle, uint8_t role, RangingDBClient_procedureSegmentsReader_t segmentsReader)
{
    uint8_t rangingStatus = FAILURE;
    Ranging_RangingHeader_t rangingHeader;
    uint8_t numAntPath = 0;
    ChannelSounding_appRASSubeventResultsEvent_t* subevent;
    Ranging_subEventHeader_t subeventHeader;
    uint8_t* subeventData = NULL;
    uint32_t subeventDataLen = 0;

    // Get the ranging header to obtain the antenna paths mask, advancing the reader past the header
    rangingStatus = RangingDBClient_getRangingHeader(&segmentsReader, &rangingHeader);

    if (rangingStatus == SUCCESS)
    {
        // Calculate the number of antenna paths from the antenna paths mask in the header
        numAntPath = gSessionsDb[connHandle].numAntPath;
    }

    if (numAntPath != 0)
    {
        while(rangingStatus == SUCCESS)
        {
            // Extract the next subevent header and data from the segments reader.
            rangingStatus = RangingDBClient_getNextSubevent(&segmentsReader, numAntPath, role, &subeventHeader, &subeventData, &subeventDataLen);

            if (rangingStatus == SUCCESS && subeventData != NULL)
            {
                subevent = (ChannelSounding_appRASSubeventResultsEvent_t*) ICall_malloc(sizeof(ChannelSounding_appRASSubeventResultsEvent_t) + subeventDataLen);
                if (subevent != NULL)
                {
                    // Fill the subevent information to the extctrl struct
                    subevent->csEvtOpcode           = CS_APP_RAS_SUBEVENT_RESULTS_EVENT;
                    subevent->status                = rangingStatus;
                    subevent->connHandle            = connHandle;
                    subevent->startAclConnEvt       = subeventHeader.startAclConnEvt;
                    subevent->freqCompensation      = subeventHeader.freqCompensation;
                    subevent->rangingDoneStatus     = subeventHeader.rangingDoneStatus;
                    subevent->subeventDoneStatus    = subeventHeader.subeventDoneStatus;
                    subevent->rangingAbortReason    = subeventHeader.rangingAbortReason;
                    subevent->subeventAbortReason   = subeventHeader.subeventAbortReason;
                    subevent->referencePowerLvl     = subeventHeader.referencePowerLvl;
                    subevent->numStepsReported      = subeventHeader.numStepsReported;
                    subevent->dataLen               = subeventDataLen;
                    memcpy(subevent->data, subeventData, subeventDataLen);

                    // Forward to extCtrl handler (through CsExtCtrl_csTypeEvtHandler)
                    CarNode_extEvtHandler(BLEAPPUTIL_CS_TYPE, BLEAPPUTIL_CS_APP_EVENT_CODE, (BLEAppUtil_msgHdr_t *) subevent);

                    // Free the extctrl struct
                    ICall_free(subevent);
                    subevent = NULL;
                }

                // Free the subevent data allocated by RangingDBClient_getNextSubevent after forwarding it to the upper layer
                ICall_free(subeventData);
                subeventData = NULL;
            }
        }
    }
}

#endif

#endif //CHANNEL_SOUNDING
