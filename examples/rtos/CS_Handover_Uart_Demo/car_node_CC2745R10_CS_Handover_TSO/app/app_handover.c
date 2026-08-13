/******************************************************************************

@file  app_handover.c

@brief This example file demonstrates how to activate the handover with
the help of BLEAppUtil APIs.

More details on the functions and structures can be seen next to the usage.

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

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
#ifdef CONNECTION_HANDOVER

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <stdarg.h>
#include "ti/ble/stack_util/comdef.h"
#include "ti/ble/host/handover/handover.h"
#include "app_handover.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "app_main.h"
#include "app_time_sync.h"
#include "app_uart.h"

#ifdef TIME_SYNC
#include "app_time_sync.h"
#include "app_padv_time_sync.h"
#endif

#include "ti/log/Log.h"

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"


#ifdef RANGING_CLIENT
#include "app_ranging_client_api.h"
#endif

#include "app_central_api.h"
#include "app_peripheral_api.h"

//*****************************************************************************
//! Prototypes
//*****************************************************************************
void Handover_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

void handoverDataReceive(char *param);
void handoverCnStatusDataReceive(char *param);
bStatus_t HandoverServingStart(uint16_t connHandle); 
void CS_procedureStart(uint16_t connHandle);
void CarNode_invokeProcedureEnableCmd(uint16_t connHandle);
void UartInit(void); 

uint32_t AppTimeSync_getCurrentSharedTime(void);
uint8_t AppTimeSync_init(uint8_t usePadv);
uint32_t AppTimeSync_getTimeDeltaInUs(uint32_t startTime);
uint32_t AppTimeSync_getTimeSyncSize(void);
bool AppTimeSync_isEnabled(void);
extern int uart_printf(const char *fmt, ...);
/*********************************************************************
 * CONSTANTS
 */
#define UART_MAX_READ_SIZE 1300

#define HANDOVER_APP_START_SERVING_NODE   0x00
#define HANDOVER_APP_START_CANDIDATE_NODE 0x01
#define HANDOVER_APP_CLOSE_SERVING_NODE   0x02

//*****************************************************************************
//! Globals
//*****************************************************************************

/**
 * @brief Handover data buffer header structure
 *
 * @ref Handover_EventHandler()
 */
typedef struct
{
  uint32_t type;                //!< The type of the handover data (RREQ/Stack)
  uint32_t length;              //!< The length of the handover data
} handoverBufferHeader_t;

/**
 * @brief Handover data buffer structure
 *
 * @ref Handover_startCandidateNode()
 */
typedef struct
{
  uint32_t length;                       //!< The header of handover RREQ data
  uint8_t *pHandoverRREQData;            //!< The RREQ handover data buffer
} handoverCNRREQParams_t;

// Serving node parameters
handoverSNParams_t gSnParams;

// Candidate node parameters
handoverCNParams_t gCnParams;

// Candidate node RREQ parameters
handoverCNRREQParams_t gCnRREQParams;

// True while this device has an active serving node session (StartSN succeeded, CloseSN not yet called).
// Used to reject looped-back cmdType=1 packets on the SN side.
static bool gSnSessionActive = false;

// True from the moment cmdType=1 is accepted until BLEAPPUTIL_HANDOVER_START_CANDIDATE_EVENT_CODE fires.
// Prevents a second cmdType=1 (e.g. TSO's next SN cycle) from calling registerCNCB+StartCN a second time
// before CN_EVT fires, which would suppress the event and prevent CS from starting.
static bool gCnSessionActive = false;

// Aligned copy of the stack payload passed to Handover_StartCN.
// The stack reads pHandoverData asynchronously after StartCN returns (same as SN side).
// Must stay alive until BLEAPPUTIL_HANDOVER_START_CANDIDATE_EVENT_CODE fires.
static uint8_t *gCnStackDataBuf = NULL;

// Key Node's antenna switching period, parsed from TYPE_CS_CAPS_HEADER.
// Applied to gSessionsDb[connHandle].remoteTsw in the CN_EVT handler.
static uint8_t gCnRemoteTsw = 0;

extern uint16_t globalConnHandle;

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref Handover_registerEvtHandler
//*****************************************************************//

// Events handlers struct, contains the handlers and event masks
BLEAppUtil_EventHandler_t handoverHandler =
{
  .handlerType    = BLEAPPUTIL_HANDOVER_TYPE,
  .pEventHandler  = Handover_EventHandler,
  .eventMask      = BLEAPPUTIL_HANDOVER_START_SERVING_EVENT_CODE   |
                    BLEAPPUTIL_HANDOVER_START_CANDIDATE_EVENT_CODE
};


//*****************************************************************************
//! Functions
//*****************************************************************************

bStatus_t  HandoverServingStart(uint16_t connHandle)
{
  Handover_snParams_t snParams;

  snParams.connHandle = connHandle;
  snParams.minGattHandle = 0;
  snParams.maxGattHandle = 0;
  snParams.snMode = 1;

  return Handover_startServingNode(snParams);
}

void Handover_commandParser(char *pData)
{
  uint8_t status = SUCCESS;
  Handover_dataTransfer_t *dataTransfer = (Handover_dataTransfer_t *)pData;

  switch (dataTransfer->cmdType)
  {
    case HANDOVER_APP_START_CANDIDATE_NODE:
    {
      if (gSnSessionActive)
      {
        break;
      }
      if (gCnSessionActive)
      {
        // Tell the SN to close so it doesn't wait for a reply that will never come.
        uint32_t totSize = sizeof(uint16_t) + sizeof(uint32_t);
        Handover_dataTransfer_t *closeSkip = (Handover_dataTransfer_t *)
                ICall_malloc(sizeof(Handover_dataTransfer_t) + totSize);
        if (closeSkip != NULL)
        {
          uint16_t snConnHandle;
          uint32_t closeStatus = SUCCESS;
          memcpy(&snConnHandle, dataTransfer->pData, sizeof(uint16_t));
          closeSkip->cmdType = HANDOVER_APP_CLOSE_SERVING_NODE;
          memcpy(&closeSkip->pData[0], &snConnHandle, sizeof(uint16_t));
          memcpy(&closeSkip->pData[2], &closeStatus,  sizeof(uint32_t));
          uartWriteData((uint8_t *)closeSkip, sizeof(Handover_dataTransfer_t) + totSize);
          ICall_free(closeSkip);
        }
        break;
      }
      gCnSessionActive = true;

      // Stop scan and advertising before taking over as CN so the BLE
      // controller is not contending with the handover connection takeover.
      //Central_scanStop();
      //{
      //  uint8_t advHandle = 0;
      //  Peripheral_advStop(&advHandle);
      //}

      Handover_cnParams_t cnParams;

      cnParams.timeDelta = 0;
      cnParams.timeDeltaErr = 0;
      cnParams.maxFailedConnEvents = 15;
      cnParams.txBurstRatio = 1;
      cnParams.pHandoverData = dataTransfer->pData + 2 + 4 + 4; // connHandle(uint16_t), status(uint32_t) and totSize(uint32_t)

      status = Handover_startCandidateNode(cnParams);

      if (status == SUCCESS)
      {
        // Signal the SN to close immediately, before the CN's first connection
        // event fires (~68ms away). The SN must stop transmitting at the Key
        // Node's anchor so the CN can take over without interference.
        uint32_t totSize = sizeof(uint16_t) + sizeof(uint32_t);
        Handover_dataTransfer_t *closeReply = (Handover_dataTransfer_t *)
                ICall_malloc(sizeof(Handover_dataTransfer_t) + totSize);
        if (closeReply != NULL)
        {
          uint16_t snConnHandle;
          uint32_t closeStatus = SUCCESS;
          memcpy(&snConnHandle, dataTransfer->pData, sizeof(uint16_t));
          closeReply->cmdType = HANDOVER_APP_CLOSE_SERVING_NODE;
          memcpy(&closeReply->pData[0], &snConnHandle, sizeof(uint16_t));
          memcpy(&closeReply->pData[2], &closeStatus,  sizeof(uint32_t));
          uartWriteData((uint8_t *)closeReply, sizeof(Handover_dataTransfer_t) + totSize);
          ICall_free(closeReply);
        }

      }
      else
      {
        gCnSessionActive = false;
      }
      break;
    }

    case HANDOVER_APP_CLOSE_SERVING_NODE:
    {
      Handover_closeSnParams_t closeSn;
      memcpy(&closeSn.connHandle,     dataTransfer->pData,                      sizeof(uint16_t));
      memcpy(&closeSn.handoverStatus, dataTransfer->pData + sizeof(uint16_t),   sizeof(uint32_t));

      status = Handover_closeServingNode(closeSn);
      break;
    }

    default:
    {
      status = FAILURE;
      break;
    }
  }
}


/*********************************************************************
 * @fn      Handover_startServingNode
 *
 * @brief   This function will handle the start of the handover process
 *          on the serving node side. This includes, getting the needed
 *          buffer size from the stack and registering to module's CB
 *
 * @param   snParams - Serving node start parameters
 *
 * @return  SUCCESS, FAILURE, bleMemAllocError, bleIncorrectMode
 *          INVALIDPARAMETER
 */
bStatus_t Handover_startServingNode(Handover_snParams_t snParams)
{
  uint8_t status = SUCCESS;

  status = BLEAppUtil_registerSNCB();

  if ( status == SUCCESS )
  {
    // Initialize the serving node parameters
    status = Handover_InitSNParams(&gSnParams);

    // Fill the given parameters
    gSnParams.connHandle = snParams.connHandle;
    gSnParams.handoverSnMode = snParams.snMode;

    // Set the minimum and maximum GATT handles
    gSnParams.minGattHandle = snParams.minGattHandle;
    gSnParams.maxGattHandle = snParams.maxGattHandle;

    if ( status == SUCCESS )
    {
      // Get the stack data size needed
      gSnParams.handoverDataSize = Handover_GetSNDataSize(&gSnParams);

      if ( gSnParams.handoverDataSize != 0 )
      {
        // Allocate the data buffer
        gSnParams.pHandoverData = (uint8 *) ICall_malloc(gSnParams.handoverDataSize);
        if ( gSnParams.pHandoverData == NULL )
        {
            // Allocation failed
            status = bleMemAllocError;
        }
        else
        {
          status = Handover_StartSN(&gSnParams);
          if ( status == SUCCESS )
          {
            gSnSessionActive = true;
            gCnSessionActive = false;
          }
          else
          {
            // The stack returned an error code
            // Release the allocated data
            ICall_free(gSnParams.pHandoverData);
            gSnParams.pHandoverData = NULL;
          }
        }
      }
      else
      {
        status = FAILURE;
      }
    }
  }

  return status;
}

/*********************************************************************
 * @fn      Handover_startCandidateNode
 *
 * @brief   This function will handle the start of the handover process
 *          on the candidate node side. This includes registering to the
 *          module's CB
 *
 * @param   cnParams - Candidate node start parameters
 *
 * @return  none
 */
bStatus_t Handover_startCandidateNode(Handover_cnParams_t cnParams)
{
  bStatus_t status = SUCCESS;
  handoverBufferHeader_t header;

  // Register to receive CN CB
  status = BLEAppUtil_registerCNCB();

  if ( status == SUCCESS )
  {
    status = Handover_InitCNParams(&gCnParams);

    gCnParams.pHandoverData = cnParams.pHandoverData;
    gCnParams.timeDeltaInUs = cnParams.timeDelta;
    gCnParams.timeDeltaMaxErrInUs = cnParams.timeDeltaErr;
    gCnParams.maxFailedConnEvents = cnParams.maxFailedConnEvents;
    gCnParams.txBurstRatio = cnParams.txBurstRatio;
    memcpy(&header, gCnParams.pHandoverData, sizeof(handoverBufferHeader_t));

  }

#ifdef TIME_SYNC
  if ( status == SUCCESS && AppTimeSync_isEnabled() && header.type == TYPE_TIME_SYNC_HEADER )
  {
    // Increment the pointer to the time sync data
    gCnParams.pHandoverData += sizeof(handoverBufferHeader_t);

    // Process time sync data and calculate time delta.
    // Requires PADV sync (setupTimeOffset != 0) to be meaningful; without it
    // the two nodes' RAT clocks are unrelated and the subtraction wraps.
    uint32_t startTime = BUILD_UINT32(gCnParams.pHandoverData[0], gCnParams.pHandoverData[1], gCnParams.pHandoverData[2], gCnParams.pHandoverData[3]);
    //if ( AppTimeSync_isSyncValid() && AppTimeSync_getSharedTimeOffset() != 0 )
    //if ( AppTimeSync_isSyncValid())
    {
      gCnParams.timeDeltaInUs = AppTimeSync_getTimeDeltaInUs(startTime);
    }


    // Increment the pointer to the next header
    gCnParams.pHandoverData += header.length;
    memcpy(&header, gCnParams.pHandoverData, sizeof(handoverBufferHeader_t));
  }
#endif

#ifdef RANGING_CLIENT
  if ( status == SUCCESS && header.type == TYPE_RREQ_HEADER )
  {
    uint8_t slotSt = AppRREQ_isAvailableSlot();

    // Check if there is available slot for RREQ data, if not - abort handover
    if( slotSt == SUCCESS )
    {
      // Get RREQ data length and increment the pointer
      gCnRREQParams.length = header.length;
      gCnParams.pHandoverData += sizeof(handoverBufferHeader_t);

      // Allocate RREQ data buffer
      gCnRREQParams.pHandoverRREQData = (uint8_t *) ICall_malloc(gCnRREQParams.length);

      if ( gCnRREQParams.pHandoverRREQData != NULL )
      {
        // Copy the RREQ data buffer to global temp buffer
        memcpy(gCnRREQParams.pHandoverRREQData, gCnParams.pHandoverData, gCnRREQParams.length);

        // Increment the pointer to the stack data
        gCnParams.pHandoverData += gCnRREQParams.length;
        memcpy(&header, gCnParams.pHandoverData, sizeof(handoverBufferHeader_t));
      }
      else
      {
        // Allocation failed
        status = bleMemAllocError;
        gCnRREQParams.length = 0;
      }
    }
    else
    {
      status = FAILURE;
    }
  }
#endif
  if ( status == SUCCESS && header.type == TYPE_CS_CAPS_HEADER )
  {
    gCnParams.pHandoverData += sizeof(handoverBufferHeader_t);
    gCnRemoteTsw = gCnParams.pHandoverData[0];
    gCnParams.pHandoverData += header.length;
    memcpy(&header, gCnParams.pHandoverData, sizeof(handoverBufferHeader_t));
  }

  if ( status == SUCCESS && (header.type == TYPE_STACK_HEADER) )
  {
    // Increment the pointer to the stack data
    gCnParams.pHandoverData += sizeof(handoverBufferHeader_t);

    // The UART receive buffer lands at heap_base+83 (4k+3 alignment).
    // llCsDbSetProcedureParams() inside Handover_StartCN reads uint32_t values
    // from pHandoverData and will fault if the pointer is not 4-byte aligned.
    // Copy to a fresh heap block (ICall_malloc always returns 4-byte aligned).
    // The stack continues reading pHandoverData asynchronously after StartCN returns
    // (same pattern as SN side) — do NOT free here; freed in the CN event handler.
    gCnStackDataBuf = (uint8_t *)ICall_malloc(header.length);
    if ( gCnStackDataBuf == NULL )
    {
      status = bleMemAllocError;
    }
    else
    {
      memcpy(gCnStackDataBuf, gCnParams.pHandoverData, header.length);
      gCnParams.pHandoverData = gCnStackDataBuf;

      
      // Process stack handover data
      status = Handover_StartCN(&gCnParams);
    }
  }
  else if ( status == SUCCESS )
  {

  }

  return status;
}

/*********************************************************************
 * @fn      Handover_closeServingNode
 *
 * @brief   The function will close the serving node side
 *
 * @param   closeSn - handover close SN parameters
 *
 * @return  status
 */
bStatus_t Handover_closeServingNode(Handover_closeSnParams_t closeSn)
{
#if defined(RANGING_CLIENT)
  // Close RREQ side without notifying the peer.
  AppRREQ_localDisable(closeSn.connHandle);
#endif

  // CloseSN stack side.
  bStatus_t status = Handover_CloseSN(&gSnParams, closeSn.handoverStatus);
  gSnSessionActive = false;
  return status;
}

/*********************************************************************
 * @fn      Handover_EventHandler
 *
 * @brief   The purpose of this function is to handle handover events
 *
 * @param   event    - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Handover_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (event)
    {
      case BLEAPPUTIL_HANDOVER_START_SERVING_EVENT_CODE:
      {
        uint8_t *pStat = (uint8_t *)pMsgData;
        uint32_t status = INVALID_32BIT_STATUS;
        uint16_t connHandle = INVALID_CONN_HANDLE;
        uint32_t extHeaderSize = sizeof(connHandle) + sizeof(status);
        uint32_t handoverDataBufferSize = 0;
        uint8_t headersCount = 1;

        handoverBufferHeader_t bufferRREQHeader = {TYPE_RREQ_HEADER, 0};
        handoverBufferHeader_t bufferStackHeader = {TYPE_STACK_HEADER, gSnParams.handoverDataSize};
        handoverBufferHeader_t bufferTimeSyncHeader = {TYPE_TIME_SYNC_HEADER, 0};
        handoverBufferHeader_t bufferCsCapsHeader = {TYPE_CS_CAPS_HEADER, sizeof(uint8_t)};

        if ( gSnParams.pHandoverData != NULL )
        {
          connHandle = BUILD_UINT16(pStat[0], pStat[1]);
          status = BUILD_UINT32(pStat[2], pStat[3], pStat[4], pStat[5]);

          // If status is success - send data
          if ( status == SUCCESS )
          {
  #ifdef TIME_SYNC
            if ( AppTimeSync_isEnabled() )
            {
              bufferTimeSyncHeader.length = AppTimeSync_getTimeSyncSize();
              headersCount++;
            }
  #endif

  #ifdef RANGING_CLIENT
            // RREQ - get the profile struct data size 
            bufferRREQHeader.length = AppRREQ_getConnInfoSize(connHandle);

            if( bufferRREQHeader.length != 0 )
            {
              headersCount++;
            }
  #endif
            // CS Caps header always included (1 byte: remoteTsw)
            headersCount++;

            // Buffers size + actual handover data size ( RREQ.length = 0 and headersCount = 1 if not enabled )
            handoverDataBufferSize = (headersCount*sizeof(handoverBufferHeader_t)) +
                                      bufferStackHeader.length +
                                      bufferRREQHeader.length +
                                      bufferTimeSyncHeader.length +
                                      bufferCsCapsHeader.length;

            // Extctrl expect - [connHandle, status, length of handover data, handover data...]
            extHeaderSize += sizeof(handoverDataBufferSize);
          }
        }

        // Allocate the event
        Handover_dataTransfer_t *dataTransfer = (Handover_dataTransfer_t *) ICall_malloc(sizeof(Handover_dataTransfer_t) + extHeaderSize + handoverDataBufferSize);

        if ( dataTransfer != NULL )
        {
          dataTransfer->cmdType = HANDOVER_APP_START_CANDIDATE_NODE;

          uint8_t *pEvt = dataTransfer->pData;

          // Copy the connection handle
          memcpy(pEvt, &connHandle, sizeof(connHandle));
          pEvt += sizeof(connHandle);

          // Copy the status
          memcpy(pEvt, &status, sizeof(status));
          pEvt += sizeof(status);

          if ( status == SUCCESS )
          {
            // Copy the handover data buffer size
            memcpy(pEvt, &handoverDataBufferSize, sizeof(handoverDataBufferSize));
            pEvt += sizeof(handoverDataBufferSize);

  #ifdef TIME_SYNC
            if ( AppTimeSync_isEnabled() )
            {
              // Copy the Time Sync header
              memcpy(pEvt, &bufferTimeSyncHeader, sizeof(bufferTimeSyncHeader));
              pEvt += sizeof(bufferTimeSyncHeader);

              // Copy the Time Sync data value
              uint32_t currSharedTime = AppTimeSync_getCurrentSharedTime();
              memcpy(pEvt, &currSharedTime, bufferTimeSyncHeader.length);
              pEvt += bufferTimeSyncHeader.length;
            }
  #endif

  #ifdef RANGING_CLIENT
            if ( bufferRREQHeader.length != 0 )
            {
              // Copy the RREQ header
              memcpy(pEvt, &bufferRREQHeader, sizeof(bufferRREQHeader));
              pEvt += sizeof(bufferRREQHeader);

              // Copy the RREQ data value
              AppRREQ_getConnInfoData(connHandle, pEvt);
              pEvt += bufferRREQHeader.length;
            }
  #endif
            // Copy the CS Caps header + remoteTsw (Key Node's antenna switching period)
            {
              extern uint8_t CarNode_getRemoteTsw(uint16_t connHandle);
              uint8_t remoteTsw = CarNode_getRemoteTsw(connHandle);
              memcpy(pEvt, &bufferCsCapsHeader, sizeof(bufferCsCapsHeader));
              pEvt += sizeof(bufferCsCapsHeader);
              memcpy(pEvt, &remoteTsw, bufferCsCapsHeader.length);
              pEvt += bufferCsCapsHeader.length;
            }

            // Copy the Stack header
            memcpy(pEvt, &bufferStackHeader, sizeof(bufferStackHeader));
            pEvt += sizeof(bufferStackHeader);

            // Copy the handover stack data value
            memcpy(pEvt, gSnParams.pHandoverData, bufferStackHeader.length);
          }
        }

          uartWriteData((uint8_t *)dataTransfer, sizeof(Handover_dataTransfer_t) + extHeaderSize + handoverDataBufferSize);

          ICall_free(dataTransfer);
        
        // Done with the handover. Reset the parameters
        if ( gSnParams.pHandoverData != NULL )
        {
          ICall_free(gSnParams.pHandoverData);
          gSnParams.pHandoverData = NULL;
        }
        gSnParams.handoverDataSize = HANDOVER_INVALID_DATA_SIZE;

        break;
      }

      case BLEAPPUTIL_HANDOVER_START_CANDIDATE_EVENT_CODE:
      {
        gCnSessionActive = false;
        // Stack is done with pHandoverData — release the aligned stack buffer now.
        if ( gCnStackDataBuf != NULL )
        {
          ICall_free(gCnStackDataBuf);
          gCnStackDataBuf = NULL;
        }

        uint8_t *pStat = (uint8_t *)pMsgData;
        uint32_t status = INVALID_32BIT_STATUS;
        uint16_t connHandle = INVALID_CONN_HANDLE;

        connHandle = BUILD_UINT16(pStat[0], pStat[1]);
        status = BUILD_UINT32(pStat[2], pStat[3], pStat[4], pStat[5]);

  #ifdef RANGING_CLIENT
        // Apply RREQ data to the actual RREQ table
        if ( gCnRREQParams.pHandoverRREQData != NULL && gCnRREQParams.length > 0 )
        {
          uint8_t rreqStatus = AppRREQ_populateConnInfoData(connHandle, gCnRREQParams.pHandoverRREQData, gCnRREQParams.length);
          if ( rreqStatus != SUCCESS )
          {
            // RREQ handover apply failed - update status to reflect partial failure
            status = rreqStatus;
          }

          // Free the temporary RREQ data buffer
          ICall_free(gCnRREQParams.pHandoverRREQData);
          gCnRREQParams.pHandoverRREQData = NULL;
          gCnRREQParams.length = 0;
        }
  #endif

        if (status == SUCCESS)
        {
          // Restore the Key Node's antenna switching capability so finalTsw
          // is computed correctly in CarNode_handleCsProcEnableComplete.
          {
            extern void CarNode_setRemoteTsw(uint16_t connHandle, uint8_t tsw);
            CarNode_setRemoteTsw(connHandle, gCnRemoteTsw);
          }

          // Set cyclic_handle so CarNode_invokeProcedureEnableCmd finds the
          // connection. LINK_ESTABLISHED_EVENT does this too but fires after
          // CN_EVT, so set it explicitly here for the handover path.
          extern uint16_t cyclic_handle;
          cyclic_handle = connHandle;

          // CS_procedureStart only calls ProcedureEnable; after handover the
          // procedure parameters are not yet applied to the new connHandle.
          // CarNode_invokeProcedureEnableCmd calls SetProcedureParameters first.
          CarNode_invokeProcedureEnableCmd(connHandle);

#ifdef TIME_SYNC
          // Handover_StartCN() causes the controller to silently drop any
          // pending createSync. Restart TSA discovery so PADV sync establishes.
          // AppPadvTimeSync_onCnEstablished();
#endif
        }
        else
        {

        }
        break;
      }
    }
  }
}

/*********************************************************************
 * @fn      Handover_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize
 *          register the specific events handlers of the handover
 *          application module and initiate the parser module.
 *
 * @param   none
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Handover_start(void)
{

  handoverUartInit();
  uart_printf("UART_CS_Handover_Ready!\r\n");
#ifdef TIME_SYNC
  // Initialize the time sync module.
  // TRUE = use PADV time sync,
  // FALSE = use external time source by calling AppTimeSync_setTimeOffset().
  AppTimeSync_init(TRUE);
#endif // TIME_SYNC

  // Register to both the Serving node and the candidate node callbacks
  return BLEAppUtil_registerEventHandler(&handoverHandler);
}

#endif // CONNECTION_HANDOVER
#endif // ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
