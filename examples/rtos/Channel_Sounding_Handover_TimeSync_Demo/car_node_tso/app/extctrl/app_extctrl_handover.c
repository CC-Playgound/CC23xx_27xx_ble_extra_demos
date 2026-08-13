/******************************************************************************

@file  app_extctrl_handover.c

@brief This example file demonstrates how to activate the handover command and
event parser.

using the structures defined in the file.

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

/*********************************************************************
 * INCLUDES
 */
#include "app_extctrl_handover.h"
#include "app_extctrl_common.h"
#include "ti/ble/app_util/framework/bleapputil_extctrl_dispatcher.h"

#ifdef CS_MEASURE_DISTANCE
#include <app_cs_process_api.h>
#endif

/*********************************************************************
 * PROTOTYPES
 */
static void HandoverExtCtrl_commandParser(uint8_t *pData);
static void HandoverExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t *);
static void HandoverExtCtrl_startSnEvtHandler(uint8_t *pHandoverMsg);
static void HandoverExtCtrl_startCnEvtHandler(uint8_t *pHandoverMsg);
static void HandoverExtCtrl_evtCodeHandler(uint32_t event, BLEAppUtil_msgHdr_t *pMsgData);
static void HandoverExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen);
static void HandoverExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp);

/*********************************************************************
 * CONSTANTS
 */
#define HANDOVER_APP_START_SERVING_NODE   0x00
#define HANDOVER_APP_START_CANDIDATE_NODE 0x01
#define HANDOVER_APP_CLOSE_SERVING_NODE   0x02

/*********************************************************************
 * GLOBALS
 */
// The external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      HandoverExtCtrl_eventHandler
 *
 * @brief   This function handles the events raised from the application that
 *          this module registered to
 *
 * @param   eventType - the type of the events @ref BLEAppUtil_eventHandlerType_e.
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void HandoverExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (eventType)
    {
      case BLEAPPUTIL_HANDOVER_TYPE:
      {
        // Call the function that will build the relevant event
        HandoverExtCtrl_evtCodeHandler(event, pMsgData);
        break;
      }

      default:
      {
        break;
      }
    }
  }
}

/*********************************************************************
 * @fn      HandoverExtCtrl_evtCodeHandler
 *
 * @brief   This function handles events from type @ref BLEAppUtil_HandoverEventMaskFlags_e
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void HandoverExtCtrl_evtCodeHandler(uint32_t event, BLEAppUtil_msgHdr_t *pMsgData)
{
  switch(event)
  {
    case BLEAPPUTIL_HANDOVER_START_SERVING_EVENT_CODE:
    {
      HandoverExtCtrl_startSnEvtHandler((uint8_t *)(pMsgData));
      break;
    }

    case BLEAPPUTIL_HANDOVER_START_CANDIDATE_EVENT_CODE:
    {
      HandoverExtCtrl_startCnEvtHandler((uint8_t*)(pMsgData));
      break;
    }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      HandoverExtCtrl_startSnEvtHandler
 *
 * @brief   This function handles the serving node start event
 *
 * @param   pHandoverMsg - pointer to the start serving node data
 *
 * @return  None
 */
static void HandoverExtCtrl_startSnEvtHandler(uint8_t *pHandoverMsg)
{
  if ( pHandoverMsg != NULL )
  {
    uint32_t totLen = 0;
    uint32_t dataLen = 0;
    AppExtCtrlHandoverSnStartEvent_t *pEvt = NULL;

    // Extract the connection handle
    uint16_t connHandle = BUILD_UINT16(pHandoverMsg[0], pHandoverMsg[1]);
    pHandoverMsg += sizeof(uint16_t);

    // Extract the status
    uint32_t status = BUILD_UINT32(pHandoverMsg[0], pHandoverMsg[1], pHandoverMsg[2], pHandoverMsg[3]);
    pHandoverMsg += sizeof(uint32_t);

    if ( status == SUCCESS )
    {
      dataLen = BUILD_UINT32(pHandoverMsg[0], pHandoverMsg[1], pHandoverMsg[2], pHandoverMsg[3]);
      pHandoverMsg += sizeof(uint32_t);

      // Allocate the full data size
      totLen = sizeof(AppExtCtrlHandoverSnStartEvent_t) + dataLen;
    }
    else
    {
      dataLen = 0;
      totLen = sizeof(AppExtCtrlHandoverSnStartEvent_t);
    }

    // Allocate the necessary size
    pEvt = (AppExtCtrlHandoverSnStartEvent_t *) ICall_malloc(totLen);

    if ( pEvt != NULL )
    {
      pEvt->event = APP_EXTCTRL_HANDOVER_SN_DATA;
      pEvt->connHandle = connHandle;
      pEvt->status = status;
      pEvt->dataLen = dataLen;

      // If we have handover data
      if ( (dataLen != 0) && (status == SUCCESS) )
      {
        memcpy(pEvt->pData, pHandoverMsg, dataLen);
      }

      /*
       *   Embedded CS-Handover sequence TODO:
       *       1. Take time stamp here for the handover offset calculation = t0.
       */
      // Send the event forward
      HandoverExtCtrl_extHostEvtHandler((uint8_t *)pEvt, totLen);
      // Free the resources
      ICall_free(pEvt);
    }
  }
}

/*********************************************************************
 * @fn      HandoverExtCtrl_startCnEvtHandler
 *
 * @brief   This function handles the candidate node start event
 *
 * @param   pHandoverMsg - pointer to the start candidate node connection
 *                         information
 *
 * @return  None
 */
static void HandoverExtCtrl_startCnEvtHandler(uint8_t *pHandoverMsg)
{
  if ( pHandoverMsg != NULL )
  {
    uint32_t totLen = 0;
    AppExtCtrlHandoverCnStartEvent_t *pEvt = NULL;

    // Extract the connection handle
    uint16_t connHandle = BUILD_UINT16(pHandoverMsg[0], pHandoverMsg[1]);
    pHandoverMsg += sizeof(uint16_t);
    // {
    //   /*
    //    *   Embedded CS-Handover sequence TODO:
    //    *       1. Uncomment this block
    //    *
    //    *      The purpose of this section is to trigger CS procedure enable
    //    *      once the handover process is completed and the Candidate Node is ready.
    //    */
    //   ChannelSounding_setProcedureEnableCmdParams_t csEnableParams;
    //   csEnableParams.connHandle = connHandle;
    //   csEnableParams.configID = 0;
    //   csEnableParams.enable = CS_ENABLE;
    //   ChannelSounding_procedureEnable(&csEnableParams);
    // }

    // Extract the status
    uint32_t status = BUILD_UINT32(pHandoverMsg[0],
                                   pHandoverMsg[1],
                                   pHandoverMsg[2],
                                   pHandoverMsg[3]);

    // The size of the event code, connection handle (uint16_t) and status (uint32_t)
    totLen = sizeof(AppExtCtrlHandoverCnStartEvent_t);

    // Allocate the necessary size
    pEvt = (AppExtCtrlHandoverCnStartEvent_t *) ICall_malloc(totLen);

    if ( pEvt != NULL )
    {
      pEvt->event = APP_EXTCTRL_HANDOVER_CN_STATUS;
      pEvt->connHandle = connHandle;
      pEvt->status = status;

      // Send the event forward
      HandoverExtCtrl_extHostEvtHandler((uint8_t *)pEvt, totLen);
      // Free the resources
      ICall_free(pEvt);
    }
  }
}

/*********************************************************************
 * @fn      HandoverExtCtrl_commandParser
 *
 * @brief   This function parse the received host message, and call the
 *          the relevant API with the data.
 *
 * @param   pData - pointer to the data message.
 *
 * @return  None
 */
static void HandoverExtCtrl_commandParser(uint8_t *pData)
{
  if (pData != NULL)
  {
    appMsg_t* appMsg = (appMsg_t*)pData;
    bStatus_t status = SUCCESS;

    switch (appMsg->cmdOp)
    {
      case HANDOVER_APP_START_SERVING_NODE:
      {
        Handover_snParams_t snParams;
        AppExtCtrlHandoverStartSnCmd_t *pMsgData = (AppExtCtrlHandoverStartSnCmd_t *)appMsg->pData;

        snParams.connHandle = pMsgData->connHandle;
        snParams.minGattHandle = pMsgData->minGattHandle;
        snParams.maxGattHandle = pMsgData->maxGattHandle;
        snParams.snMode = pMsgData->snMode;

        status = Handover_startServingNode(snParams);
        break;
      }

      case HANDOVER_APP_START_CANDIDATE_NODE:
      {
        Handover_cnParams_t cnParams;
        AppExtCtrlHandoverStartCnCmd_t *pMsgData = (AppExtCtrlHandoverStartCnCmd_t *)appMsg->pData;

      /*
       *   Embedded CS-Handover sequence TODO:
       *       1. Take time stamp here for the handover offset calculation = t1.
       *       2. Remove:
       *                  cnParams.timeDelta = pMsgData->timeDelta;
       *       3. Add:
       *                  cnParams.timeDelta = (t1 - t0)/4;
       *
       * (Time CN received the command) - (Time SN sent the data) = Actual time delta
       */
        cnParams.timeDelta = pMsgData->timeDelta;
        cnParams.timeDeltaErr = pMsgData->timeDeltaErr;
        cnParams.maxFailedConnEvents = pMsgData->maxNumFailedEvents;
        cnParams.txBurstRatio = (uint8_t)pMsgData->txBurstRatio;
        cnParams.pHandoverData = pMsgData->pData;

        status = Handover_startCandidateNode(cnParams);
        break;
      }

      case HANDOVER_APP_CLOSE_SERVING_NODE:
      {
        Handover_closeSnParams_t closeSn;
        AppExtCtrlHandoverCloseSnCmd_t *pMsgData = (AppExtCtrlHandoverCloseSnCmd_t *)appMsg->pData;

        closeSn.connHandle = pMsgData->connHandle;
        closeSn.handoverStatus = pMsgData->status;

        status = Handover_closeServingNode(closeSn);
        break;
      }

      default:
      {
        status = FAILURE;
        break;
      }
    }

    if (status != SUCCESS)
    {
      // Send error message
      HandoverExtCtrl_sendErrEvt(status, appMsg->cmdOp);
    }
  }
}

/*********************************************************************
 * @fn      HandoverExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void HandoverExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
  // Create error event structure
  AppExtCtrlErrorEvent_t errEvt;
  errEvt.event        = APP_EXTCTRL_FAILURE;
  errEvt.cmdOp        = cmdOp;
  errEvt.appSpecifier = APP_SPECIFIER_CHO;
  errEvt.errCode      = errCode;

  // Send error event
  HandoverExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

/*********************************************************************
 * @fn      HandoverExtCtrl_extHostEvtHandler
 *
 * @brief   The purpose of this function is to forward the event to the external
 *          control host event handler that the external control returned once
 *          this module registered to it.
 *
 * @param   pData    - pointer to the event data
 * @param   dataLen  - data length.
 *
 * @return  None
 */
static void HandoverExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if (gExtHostEvtHandler != NULL && pData != NULL)
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      HandoverExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the handover module
 *          to the external control.
 *
 * @param   None
 *
 * @return  SUCCESS, FAILURE
 */
bStatus_t HandoverExtCtrl_start(void)
{
  bStatus_t status = SUCCESS;

  // Register to the Dispatcher module
  gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_CHO,
                                                     &HandoverExtCtrl_commandParser,
                                                     APP_CAP_HANDOVER);

  // If the registration succeed, register event handler call back to the handover application
  if (gExtHostEvtHandler != NULL)
  {
    Handover_registerEvtHandler(&HandoverExtCtrl_eventHandler);
  }
  return status;
}

#endif // CONNECTION_HANDOVER
#endif // ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
