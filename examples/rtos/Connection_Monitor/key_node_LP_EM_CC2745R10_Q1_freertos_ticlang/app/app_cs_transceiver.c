/******************************************************************************

@file app_cs_transceiver.c

@brief This file contains the implementation of the transceiver control logic
       for the Channel Sounding application.
       It transmits and receives data over L2CAP.

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

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti/ble/host/cs/cs.h"
#include <app_l2cap_coc_api.h>
#include "string.h"
#include "app_cs_api.h"
#include "app_ranging_server_api.h"

//*****************************************************************************
//! Defines
//*****************************************************************************
#define CS_PSM             0x01  // TODO
#define CS_TRANSCEIVER_BUFFER_SIZE  L2CAP_MAX_MTU

// Pending CS results queue configuration
#define CS_PENDING_RESULTS_QUEUE_LEN 10

//*****************************************************************************
//! TYPEDEFS
//*****************************************************************************
typedef struct
{
  uint16_t connHandle;
  uint16_t CID;
} ChannelSoundingTransceiverRemoteId_t;

// A pending CS result element
typedef struct
{
    List_Elem element;
    L2capCoc_sendSduCmdParams_t *pParams;
} csPendingResultElement;

// Pending CS results queue
typedef struct
{
    List_List queue;      // The pending CS results queue
    uint16_t  count;      // Number of elements in the queue
} csPendingResultsQ;

//*****************************************************************************
//! Prototypes
//*****************************************************************************

#if !defined( RANGING_SERVER )
/**
 * @note !defined( RANGING_SERVER ) use:
 * If RANGING_SERVER is defined, this function is never called,
 * which may lead to compilation warnings due to unused functions or variables.
 */
static void ChannelSoundingTransceiver_l2capEventHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t *);
static void ChannelSoundingTransceiver_connEventHandler(uint32, BLEAppUtil_msgHdr_t *);
static void ChannelSoundingTransceiver_signalTypeEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void ChannelSoundingTransceiver_dataTypeEvtHandler(uint32 event, l2capDataEvent_t *pDataPkt);
static bStatus_t ChannelSoundingTransceiver_l2capSendResults(uint8_t *pMsgData, uint16_t dataLen);
static csPendingResultElement *ChannelSoundingTransceiver_csPendingResultElemNext(csPendingResultElement *currentElem);
#endif // !defined( RANGING_SERVER )

//*****************************************************************************
//! Globals
//*****************************************************************************

#if !defined( RANGING_SERVER )
static ExtCtrl_eventHandler_t gExtEvtHandler= NULL;
static ChannelSoundingTransceiverRemoteId_t *pRemoteDevice = NULL;

BLEAppUtil_EventHandler_t CSConnHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_CONN_TYPE,
    .pEventHandler  = ChannelSoundingTransceiver_connEventHandler,
    .eventMask      = BLEAPPUTIL_LINK_ESTABLISHED_EVENT
};

// A queue for handling L2CAP flow control events when sending CS results
csPendingResultsQ gCsPendingResultsQueue = {0};
#endif // !defined( RANGING_SERVER )

//*****************************************************************************
//! Functions
//*****************************************************************************

/*******************************************************************************
 * Public function defined in app_cs_transceiver_api.h
 */
bStatus_t ChannelSoundingTransceiver_sendResults(uint8_t *pMsgData, uint16_t dataLen, ChannelSounding_eventOpcodes_e eventOpcode)
{
  bStatus_t status = SUCCESS;

    switch( eventOpcode )
  {
    case APP_CS_PROCEDURE_ENABLE_COMPLETE_EVENT:
    {
#ifdef RANGING_SERVER
      ChannelSounding_procEnableComplete_t *pAppProcedureEnableComplete = (ChannelSounding_procEnableComplete_t *) pMsgData;
      status = AppRRSP_CsProcedureEnable(pAppProcedureEnableComplete);
#endif // RANGING_SERVER
      break;
    }

    case APP_CS_SUBEVENT_RESULT:
    {
#ifdef RANGING_SERVER
      ChannelSounding_subeventResults_t *pAppSubeventResults = (ChannelSounding_subeventResults_t *) pMsgData;
      // send event to the ranging server application
      status = AppRRSP_CsSubEvent(pAppSubeventResults);
#else // !defined( RANGING_SERVER )
      // Send the results over L2CAP
      status = ChannelSoundingTransceiver_l2capSendResults(pMsgData, dataLen);
#endif // !defined( RANGING_SERVER )
      break;
    }

    case APP_CS_SUBEVENT_CONTINUE_RESULT:
    {
#ifdef RANGING_SERVER
      ChannelSounding_subeventResultsContinue_t *pAppSubeventResultsCont = (ChannelSounding_subeventResultsContinue_t *) pMsgData;
      // send event to the ranging server application
      status = AppRRSP_CsSubContEvent(pAppSubeventResultsCont);
#else // !defined( RANGING_SERVER )
      // Send the results over L2CAP
      status= ChannelSoundingTransceiver_l2capSendResults(pMsgData, dataLen);
#endif // !defined( RANGING_SERVER )
      break;
    }

    case APP_CS_SUBEVENT_CONTINUE_RESULT_EXT:
    {
#ifdef RANGING_SERVER
      ChannelSounding_subeventResultsContinueExt_t *pAppSubeventResultsContExt = (ChannelSounding_subeventResultsContinueExt_t *) pMsgData;

      uint16_t eventSize = sizeof(ChannelSounding_subeventResultsContinue_t) + pAppSubeventResultsContExt->dataLen;
      ChannelSounding_subeventResultsContinue_t* appSubeventResultsCont = (ChannelSounding_subeventResultsContinue_t*) ICall_malloc(eventSize);
      if (appSubeventResultsCont != NULL)
      {
        appSubeventResultsCont->csEvtOpcode = pAppSubeventResultsContExt->csEvtOpcode;
        appSubeventResultsCont->connHandle = pAppSubeventResultsContExt->connHandle;
        appSubeventResultsCont->configID = pAppSubeventResultsContExt->configID;
        appSubeventResultsCont->procedureDoneStatus = pAppSubeventResultsContExt->procedureDoneStatus;
        appSubeventResultsCont->subeventDoneStatus = pAppSubeventResultsContExt->subeventDoneStatus;
        appSubeventResultsCont->abortReason = pAppSubeventResultsContExt->abortReason;
        appSubeventResultsCont->numAntennaPath = pAppSubeventResultsContExt->numAntennaPath;
        appSubeventResultsCont->numStepsReported = pAppSubeventResultsContExt->numStepsReported;
        appSubeventResultsCont->dataLen = pAppSubeventResultsContExt->dataLen;
        memcpy(appSubeventResultsCont->data, pAppSubeventResultsContExt->data, pAppSubeventResultsContExt->dataLen);

        // send event to the ranging server application
        status = AppRRSP_CsSubContEvent(appSubeventResultsCont);

        ICall_free(appSubeventResultsCont);
      }
#else // !defined( RANGING_SERVER )
        // Send the results over L2CAP
        status = ChannelSoundingTransceiver_l2capSendResults(pMsgData, dataLen);
#endif // !defined( RANGING_SERVER )
      break;
    }

    default:
    {
        // Unsupported event opcode
        status = FAILURE;
        break;
    }
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_transceiver_api.h
 */
bStatus_t ChannelSoundingTransceiver_start( void )
{
  bStatus_t status = FAILURE;

#ifdef RANGING_SERVER
  // Start the Ranging Server application
  status = AppRRSP_start();
#else // !defined( RANGING_SERVER )
  // Register to BLE APP Util event handler
  status = BLEAppUtil_registerEventHandler(&CSConnHandler);
  if( status == SUCCESS)
  {
    // Register the application callback function for L2CAP Flow Control
    L2CAP_RegisterFlowCtrlTask(BLEAppUtil_getSelfEntity());

    // Initialize the CS pending results queue and reset the element count
    List_clearList(&gCsPendingResultsQueue.queue);
    gCsPendingResultsQueue.count = 0;

    // Register to the L2CAP events
    L2CAPCOC_registerEvtHandler(&ChannelSoundingTransceiver_l2capEventHandler);
    // Create the PSM for the Channel Sounding Transceiver
    status = L2CAPCOC_createPsm(CS_PSM);
    // For BTCS - use another PSM
  }
#endif // !defined( RANGING_SERVER )

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_transceiver_api.h
 */
void ChannelSoundingTransceiver_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
#if !defined( RANGING_SERVER )
  // Register the external event handler
  gExtEvtHandler= fEventHandler;
#endif // !defined( RANGING_SERVER )
}

/***********************************************************************
** Internal Functions
*/

#if !defined( RANGING_SERVER )
/*********************************************************************
 * @fn      ChannelSoundingTransceiver_csPendingResultElemNext
 *
 * @brief   Function to return the next csPendingResultElement element in a linked list.
 *
 * @param   currentElem - a pointer to an element in the list.
 *
 * @return  Pointer to the next csPendingResultElement element in the linked list or NULL if at the end.
 */
static csPendingResultElement *ChannelSoundingTransceiver_csPendingResultElemNext(csPendingResultElement *currentElem)
{
  return (csPendingResultElement *)List_next(&currentElem->element);
}

/*********************************************************************
 * @fn      ChannelSoundingTransceiver_eventHandler
 *
 * @brief   This function handles the events raised from the application that
 * this module registered to.
 *
 * @param   eventType - the type of the events @ref BLEAppUtil_eventHandlerType_e.
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void ChannelSoundingTransceiver_l2capEventHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{

  if (pMsgData != NULL)
  {
    switch (eventType)
    {
      case BLEAPPUTIL_L2CAP_SIGNAL_TYPE:
      {
        ChannelSoundingTransceiver_signalTypeEvtHandler(event, pMsgData);
        break;
      }

      case BLEAPPUTIL_L2CAP_DATA_TYPE:
      {
        ChannelSoundingTransceiver_dataTypeEvtHandler(event, (l2capDataEvent_t *)pMsgData);
        break;
      }

      default :
      {
        break;
      }
    }
  }
}
#endif // !defined( RANGING_SERVER )

#if !defined( RANGING_SERVER )
/*********************************************************************
 * @fn      ChannelSoundingTransceiver_extEvtHandler
 *
 * @brief   Forwards the event to the registered external event handler.
 *
 * @param   eventType - the event type of the event @ref BLEAppUtil_eventHandlerType_e
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  none
 */
static void ChannelSoundingTransceiver_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer if its handle exists
  if ( gExtEvtHandler!= NULL )
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}
#endif // !defined( RANGING_SERVER )

#if !defined( RANGING_SERVER )
/*********************************************************************
 * @fn      ChannelSoundingTransceiver_signalTypeEvtHandler
 *
 * @brief   This function handles events from type @ref BLEAPPUTIL_L2CAP_SIGNAL_TYPE
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void ChannelSoundingTransceiver_signalTypeEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    l2capSignalEvent_t *pL2capSignalEvt = (l2capSignalEvent_t *)pMsgData;
    uint16_t connHandle = pL2capSignalEvt->connHandle;
    bStatus_t status = FAILURE;
    csPendingResultElement *tempCsRes = NULL;
    csPendingResultElement *nextCsRes = NULL;

    switch (event)
    {
      case BLEAPPUTIL_L2CAP_NUM_CTRL_DATA_PKT_EVT:
      {
        // Send pending CS results if any
        if (pRemoteDevice != NULL)
        {
          tempCsRes = (csPendingResultElement *) List_head(&gCsPendingResultsQueue.queue);
          while (tempCsRes != NULL)
          {
            nextCsRes = ChannelSoundingTransceiver_csPendingResultElemNext(tempCsRes);
            // Verify connection handle and CID
            if ( (tempCsRes->pParams->connHandle == pRemoteDevice->connHandle) &&
                 (tempCsRes->pParams->CID == pRemoteDevice->CID) )
            {
              status = L2CAPCOC_sendSDU(tempCsRes->pParams);
              if ( status == blePending || status == bleMemAllocError )
              {
                // Unable to send, exit the loop without removing the item from the queue
                break;
              }
            }
            
            // Remove the item from the queue and free the resources
            List_remove(&gCsPendingResultsQueue.queue, &tempCsRes->element);
            gCsPendingResultsQueue.count--;
            ICall_free(tempCsRes->pParams);
            ICall_free(tempCsRes);
            tempCsRes = nextCsRes;
          }
        }
        break;
      }

      case BLEAPPUTIL_L2CAP_CHANNEL_ESTABLISHED_EVT:
      {
        if( pRemoteDevice == NULL )
        {
          pRemoteDevice = BLEAppUtil_malloc(sizeof(ChannelSoundingTransceiverRemoteId_t));
          if( pRemoteDevice != NULL )
          {
            pRemoteDevice->connHandle = connHandle;
            pRemoteDevice->CID        = (pL2capSignalEvt->cmd).channelEstEvt.CID;
          }
        }
        break;
      }

      case BLEAPPUTIL_L2CAP_CHANNEL_TERMINATED_EVT:
      {
        if( pRemoteDevice != NULL && pRemoteDevice->connHandle == connHandle)
        {
          BLEAppUtil_free(pRemoteDevice);
          pRemoteDevice = NULL;
        }
        // Clear the CS pending results queue and free resources
        while ((tempCsRes = (csPendingResultElement *)List_get(&gCsPendingResultsQueue.queue)) != NULL)
        {
          if (tempCsRes->pParams != NULL)
          {
            ICall_free(tempCsRes->pParams);
          }
          ICall_free(tempCsRes);
        }
        List_clearList(&gCsPendingResultsQueue.queue);
        gCsPendingResultsQueue.count = 0;
        break;
      }

      case BLEAPPUTIL_L2CAP_PEER_CREDIT_THRESHOLD_EVT:
      {
        if( pRemoteDevice != NULL && pRemoteDevice->connHandle == connHandle)
        {
          L2capCoc_flowCtrlCredits_t params;
          params.connHandle = pRemoteDevice->connHandle;
          params.CID        = pRemoteDevice->CID;
          params.peerCredits= L2CAP_NOF_CREDITS - L2CAP_CREDITS_THRESHOLD;
          L2CAPCOC_FlowCtrlCredit(&params);
        }
        break;
      }

      default:
      {
        break;
      }
    }
  }
}
#endif // !defined( RANGING_SERVER )

#if !defined( RANGING_SERVER )
/*********************************************************************
 * @fn      ChannelSoundingTransceiver_dataTypeEvtHandler
 *
 * @brief   This function handles events from type @ref BLEAPPUTIL_L2CAP_DATA_TYPE
 *
 * @param   event     - message event.
 * @param   pDataPkt  - pointer to message data.
 *
 * @return  None
 */
static void ChannelSoundingTransceiver_dataTypeEvtHandler(uint32 event, l2capDataEvent_t *pDataPkt)
{
  l2capPacket_t pkt = pDataPkt->pkt;

  if ( pRemoteDevice != NULL &&
       pRemoteDevice->CID == pkt.CID &&
       pRemoteDevice->connHandle == pkt.connHandle)
  {
    // Send to the to the upper layer
    ChannelSoundingTransceiver_extEvtHandler(BLEAPPUTIL_L2CAP_DATA_TYPE, event, (BLEAppUtil_msgHdr_t *)pDataPkt);
  }
}
#endif // !defined( RANGING_SERVER )

#if !defined( RANGING_SERVER )
/*********************************************************************
 * @fn      ChannelSoundingTransceiver_connEventHandler
 *
 * @brief   The purpose of this function is to handle connection related
 *          events that rise from the GAP and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void ChannelSoundingTransceiver_connEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch(event)
    {
      case BLEAPPUTIL_LINK_ESTABLISHED_EVENT:
      {
        gapEstLinkReqEvent_t *pGapEstMsg = (gapEstLinkReqEvent_t *)pMsgData;
        if ( pRemoteDevice == NULL && pGapEstMsg->connRole == GAP_PROFILE_CENTRAL)
        {
          L2capCoc_connectReqCmdParams_t params;
          params.connHandle = ((gapEstLinkReqEvent_t *)pMsgData)->connectionHandle;
          params.psm        = CS_PSM;
          params.peerPsm    = CS_PSM;
          L2CAPCOC_connectReq(&params);
        }
        break;
      }

      default:
      {
        break;
      }
    }
  }
}
#endif // !defined( RANGING_SERVER )

#if !defined( RANGING_SERVER )
/*********************************************************************
 * @fn      ChannelSoundingTransceiver_l2capSendResults
 *
 * @brief   This function sends the results over L2CAP.
 *
 * @param   pMsgData - pointer to message data.
 * @param   dataLen  - length of the data to send.
 *
 * @return  bStatus_t - SUCCESS, FAILURE, or blePending if the data was queued into
 *                      the pending CS results queue.
 */
static bStatus_t ChannelSoundingTransceiver_l2capSendResults(uint8_t *pMsgData, uint16_t dataLen)
{
  bStatus_t status = FAILURE;
  bool safeToDealloc = TRUE; // Indicates whether it's safe to free the resources
  L2capCoc_sendSduCmdParams_t *pParams = NULL;
  csPendingResultElement *elem = NULL;

  if ( pRemoteDevice != NULL && dataLen <= CS_TRANSCEIVER_BUFFER_SIZE &&
       gCsPendingResultsQueue.count < CS_PENDING_RESULTS_QUEUE_LEN )
  {
    // Send data
    pParams = ICall_malloc(sizeof(L2capCoc_sendSduCmdParams_t) + dataLen);
    if (pParams != NULL)
    {
      pParams->connHandle = pRemoteDevice->connHandle;
      pParams->CID        = pRemoteDevice->CID;
      pParams->len        = dataLen;
      memcpy( pParams->pPayload, pMsgData, dataLen );

      // If there is a pending result in the queue, set the status as blePending so it will be enqueued
      if ( !List_empty( &gCsPendingResultsQueue.queue ) )
      {
        status = blePending;
      }
      else
      {
        // No pending results, send directly
        status = L2CAPCOC_sendSDU( pParams );
      }

      // Enqueue the CS result if it is pending
      if ( status == blePending )
      {
        elem = ICall_malloc(sizeof(csPendingResultElement));
        if (elem != NULL)
        {
          memset(elem, 0, sizeof(csPendingResultElement));
          elem->pParams = pParams;
          List_put(&gCsPendingResultsQueue.queue, &elem->element);
          gCsPendingResultsQueue.count++;
          safeToDealloc = FALSE;
        }
        else
        {
          // Unable to allocate memory for the queue element, set status to FAILURE
          status = FAILURE;
          safeToDealloc = TRUE;
        }
      }
      // Free resources if needed
      if ( safeToDealloc == TRUE )
      {
        // Free the resources
        ICall_free(pParams);
        if (elem != NULL)
        {
          ICall_free(elem);
        }
      }
    }
  }
  return status;
}
#endif // !defined( RANGING_SERVER )
