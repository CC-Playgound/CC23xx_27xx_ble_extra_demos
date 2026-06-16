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

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
#ifdef CONNECTION_HANDOVER

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <stdarg.h>
#include "ti/ble/stack_util/comdef.h"
#include "ti/ble/host/handover/handover.h"
#include "app_handover_api.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "app_main.h"
#include "app_time_sync.h"

#ifdef TIME_SYNC
#include "app_time_sync.h"
#endif

#ifdef RANGING_CLIENT
#include "app_ranging_client_api.h"
#endif

//*****************************************************************************
//! Prototypes
//*****************************************************************************
static void Handover_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
void Handover_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

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

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref Handover_registerEvtHandler
//*****************************************************************//
static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

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
          if ( status != SUCCESS )
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
    header = *(handoverBufferHeader_t *)(gCnParams.pHandoverData);
  }

#ifdef TIME_SYNC
  if ( status == SUCCESS && AppTimeSync_isEnabled() && header.type == TYPE_TIME_SYNC_HEADER )
  {
    // Increment the pointer to the time sync data
    gCnParams.pHandoverData += sizeof(handoverBufferHeader_t);

    // Process time sync data and calculate time delta
    uint32_t startTime = BUILD_UINT32(gCnParams.pHandoverData[0], gCnParams.pHandoverData[1], gCnParams.pHandoverData[2], gCnParams.pHandoverData[3]);
    gCnParams.timeDeltaInUs = AppTimeSync_getTimeDeltaInUs(startTime);

    // Increment the pointer to the next header
    gCnParams.pHandoverData += header.length;
    header = *(handoverBufferHeader_t *)(gCnParams.pHandoverData);
  }
#endif

#ifdef RANGING_CLIENT
  if ( status == SUCCESS && header.type == TYPE_RREQ_HEADER )
  {
    // Check if there is available slot for RREQ data, if not - abort handover
    if( AppRREQ_isAvailableSlot() == SUCCESS )
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
        header = *(handoverBufferHeader_t *)(gCnParams.pHandoverData);
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
  if ( status == SUCCESS && (header.type == TYPE_STACK_HEADER) )
  {
    // Increment the pointer to the stack data
    gCnParams.pHandoverData += sizeof(handoverBufferHeader_t);

    // Process stack handover data
    status = Handover_StartCN(&gCnParams);
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
  return Handover_CloseSN(&gSnParams, closeSn.handoverStatus);
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
        uint8_t *pSnEvent = NULL;
        uint8_t headersCount = 1;

        handoverBufferHeader_t bufferRREQHeader = {TYPE_RREQ_HEADER, 0};
        handoverBufferHeader_t bufferStackHeader = {TYPE_STACK_HEADER, gSnParams.handoverDataSize};
        handoverBufferHeader_t bufferTimeSyncHeader = {TYPE_TIME_SYNC_HEADER, 0};

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
            // Buffers size + actual handover data size ( RREQ.length = 0 and headersCount = 1 if not enabled )
            handoverDataBufferSize = (headersCount*sizeof(handoverBufferHeader_t)) +
                                      bufferStackHeader.length +
                                      bufferRREQHeader.length +
                                      bufferTimeSyncHeader.length;

            // Extctrl expect - [connHandle, status, length of handover data, handover data...]
            extHeaderSize += sizeof(handoverDataBufferSize);
          }
        }

        // Allocate the event
        pSnEvent = (uint8_t *) ICall_malloc(extHeaderSize + handoverDataBufferSize);

        if ( pSnEvent != NULL )
        {
          memset(pSnEvent, 0, extHeaderSize + handoverDataBufferSize);
          uint8_t *pEvt = pSnEvent;

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
            // Copy the Stack header
            memcpy(pEvt, &bufferStackHeader, sizeof(bufferStackHeader));
            pEvt += sizeof(bufferStackHeader);

            // Copy the handover stack data value
            memcpy(pEvt, gSnParams.pHandoverData, bufferStackHeader.length);
          }
        }

        Handover_extEvtHandler(BLEAPPUTIL_HANDOVER_TYPE, event, (BLEAppUtil_msgHdr_t *)pSnEvent);

        ICall_free(pSnEvent);

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
          uint8_t *pStat = (uint8_t *)pMsgData;
          uint32_t status = INVALID_32BIT_STATUS;
          uint16_t connHandle = INVALID_CONN_HANDLE;
          uint32_t totSize = 0;
          uint8_t *pCnEvent = NULL;

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
          // If status is success - send data
          totSize = sizeof(connHandle) + sizeof(status);

          // Allocate the event
          pCnEvent = (uint8_t *) ICall_malloc(totSize);

          if ( pCnEvent != NULL )
          {
            uint8_t *pEvt = pCnEvent;

            // Copy the connection handle
            memcpy(pEvt, &connHandle, sizeof(connHandle));
            pEvt += sizeof(connHandle);

            // Copy the status
            memcpy(pEvt, &status, sizeof(status));
            pEvt += sizeof(status);

            Handover_extEvtHandler(BLEAPPUTIL_HANDOVER_TYPE, event, (BLEAppUtil_msgHdr_t *)pCnEvent);
            ICall_free(pCnEvent);
          }

          break;
      }

      default:
          break;
    }
  }
}

/*********************************************************************
 * @fn      Handover_extEvtHandler
 *
 * @brief   The purpose of this function is to forward the event to the external
 *          event handler that registered to handle the events.
 *
 * @param   eventType - the event type of the event @ref BLEAppUtil_eventHandlerType_e
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  none
 */
static void Handover_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if ( gExtEvtHandler != NULL )
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      Handover_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void Handover_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
  gExtEvtHandler = fEventHandler;
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
