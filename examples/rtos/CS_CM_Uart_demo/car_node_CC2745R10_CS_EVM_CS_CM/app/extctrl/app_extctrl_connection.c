/******************************************************************************

@file  app_extctrl_connection.c

@brief This file parse and process the messages comes form the external control module
 dispatcher module, and build the events from the app_connection.c application and
 send it to the external control dispatcher module back.

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


/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_extctrl_dispatcher.h"
#include "app_extctrl_connection.h"
#include <string.h>

/*********************************************************************
 * PROTOTYPES
 */
static void ConnectionExtCtrl_sendErrEvt(uint8_t, uint8_t);
static void ConnectionExtCtrl_commandParser(uint8_t*);
static void ConnectionExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);
static void ConnectionExtCtrl_extHostEvtHandler(uint8_t*, uint16_t);

/*********************************************************************
 * CONSTANTS
 */
#define CONNECTION_CMD_REGISTER_CONN_EVENT      0x00
#define CONNECTION_CMD_UNREGISTER_CONN_EVENT    0x01
#define CONNECTION_CMD_SET_PHY                  0x02
#define CONNECTION_CMD_TERMINATE_LINK           0x03
#define CONNECTION_CMD_PARAMETER_UPDATE         0x04
#define CONNECTION_CMD_CHANNEL_MAP_UPDATE       0x05
#define CONNECTION_CMD_SET_DATA_LENGTH          0x06

/*********************************************************************
 * GLOBALS
 */
// The external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      ConnectionExtCtrl_commandParser
 *
 * @brief   This function parse the received host message, and call the
 *          the relevant API with the data.
 *
 * @param   pData - pointer to the data message.
 *
 * @return  None
 */
static void ConnectionExtCtrl_commandParser(uint8_t *pData)
{
  if (pData != NULL)
  {
    appMsg_t* appMsg = (appMsg_t*)pData;
    bStatus_t status = SUCCESS;

    switch (appMsg->cmdOp)
    {
      case CONNECTION_CMD_REGISTER_CONN_EVENT:
      {
        AppExtCtrlRegisterConnEventCmdParams_t *pExtParams = (AppExtCtrlRegisterConnEventCmdParams_t *)appMsg->pData;
        Connection_registerConnEventCmdParams_t params;

        // Parse external parameters into Connection_registerConnEventCmdParams_t
        params.connHandle = pExtParams->connHandle;
        params.eventType = (GAP_CB_Event_e)pExtParams->eventType;
        params.reportFrequency = pExtParams->reportFrequency;

        status = Connection_registerConnEventCmd(&params);
        break;
      }

      case CONNECTION_CMD_UNREGISTER_CONN_EVENT:
      {
        status = Connection_unregisterConnEventCmd();
        break;
      }

      case CONNECTION_CMD_SET_PHY:
      {
        AppExtCtrlSetPhyCmdParams_t *pExtParams = (AppExtCtrlSetPhyCmdParams_t *)appMsg->pData;
        Connection_setPhyCmdParams_t params;

        // Parse external parameters into Connection_setPhyCmdParams_t
        params.connHandle = pExtParams->connHandle;
        params.phyOpts = pExtParams->phyOpts;
        params.allPhys = pExtParams->allPhys;
        params.txPhy = pExtParams->txPhy;
        params.rxPhy = pExtParams->rxPhy;

        status = Connection_setPhyCmd(&params);
        break;
      }

      case CONNECTION_CMD_PARAMETER_UPDATE:
      {
        AppExtCtrlParamUpdateCmdParams_t *pExtParams = (AppExtCtrlParamUpdateCmdParams_t *)appMsg->pData;
        Connection_paramUpdateCmdParams_t params;

        // Parse external parameters into Connection_paramUpdateCmdParams_t
        params.connectionHandle = pExtParams->connHandle;
        params.intervalMin = pExtParams->intervalMin;
        params.intervalMax = pExtParams->intervalMax;
        params.connLatency = pExtParams->connLatency;
        params.connTimeout = pExtParams->connTimeout;
        params.signalIdentifier = pExtParams->signalIdentifier;

        status = Connection_paramUpdateCmd(&params);
        break;
      }

      case CONNECTION_CMD_CHANNEL_MAP_UPDATE:
      {
        AppExtCtrlChannelMapUpdateCmdParams_t *pExtParams = (AppExtCtrlChannelMapUpdateCmdParams_t *)appMsg->pData;
        Connection_channelMapUpdateCmdParams_t params;

        // Parse external parameters into Connection_channelMapUpdateCmdParams_t
        params.connectionHandle = pExtParams->connHandle;
        memcpy(params.channelMap, pExtParams->channelMap, sizeof(params.channelMap));

        status = Connection_channelMapUpdateCmd(&params);
        break;
      }

      case CONNECTION_CMD_SET_DATA_LENGTH:
      {
        AppExtCtrlSetDataLenCmdParams_t *pExtParams = (AppExtCtrlSetDataLenCmdParams_t *)appMsg->pData;
        Connection_dataLenUpdateCmdParams_t params;

        // Parse external parameters into Connection_dataLenUpdateCmdParams_t
        params.connectionHandle = pExtParams->connHandle;
        params.txOctets = pExtParams->maxTxOctets;
        params.txTime = pExtParams->maxTxTime;

        status = Connection_dataLenUpdateCmd(&params);
        break;
      }

      case CONNECTION_CMD_TERMINATE_LINK:
      {
        AppExtCtrlConnTermLinkReq_t *pExtParams = (AppExtCtrlConnTermLinkReq_t *)appMsg->pData;
        status = Connection_terminateLinkCmd(pExtParams->connHandle);
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
      ConnectionExtCtrl_sendErrEvt(status, appMsg->cmdOp);
    }
  }
}

/*********************************************************************
 * @fn      ConnectionExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void ConnectionExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
  // Create error event structure
  AppExtCtrlErrorEvent_t errEvt;
  errEvt.event        = APP_EXTCTRL_FAILURE;
  errEvt.cmdOp        = cmdOp;
  errEvt.appSpecifier = APP_SPECIFIER_CONNECTION;
  errEvt.errCode      = errCode;

  // Send error event
  ConnectionExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

/*********************************************************************
 * @fn      ConnectionExtCtrl_linkTermEvtHandler
 *
 * @brief   This function handles the link termination event.
 *
 * @param   pTermMsg - pointer to the termination message
 *
 * @return  None
 */
static void ConnectionExtCtrl_linkTermEvtHandler(gapTerminateLinkEvent_t *pTermMsg)
{
  if (pTermMsg != NULL)
  {
    AppExtCtrlTerminateLinkEvent_t termLinkEvt;

    termLinkEvt.event            =  APP_EXTCTRL_LINK_TERMINATED_EVENT;
    termLinkEvt.status           =  pTermMsg->hdr.status;
    termLinkEvt.opcode           =  pTermMsg->opcode;
    termLinkEvt.connectionHandle =  pTermMsg->connectionHandle;
    termLinkEvt.reason           =  pTermMsg->reason;

    // Send the event forward
    ConnectionExtCtrl_extHostEvtHandler((uint8_t *)&termLinkEvt,
                             sizeof(AppExtCtrlTerminateLinkEvent_t));
  }
}

/*********************************************************************
 * @fn      ConnectionExtCtrl_paramUpdateEvtHandler
 *
 * @brief   This function handles the connection parameter update event.
 *
 * @param   pTermMsg - pointer to the param update message
 *
 * @return  None
 */
static void ConnectionExtCtrl_paramUpdateEvtHandler(gapLinkUpdateEvent_t *pParamUpdateMsg, AppExtCtrlEvent_e eventType)
{
  if (pParamUpdateMsg != NULL)
  {
    AppExtCtrlParamUpdateEvent_t paramUpdateEvt;
    
    paramUpdateEvt.event            =  eventType;
    paramUpdateEvt.status           =  pParamUpdateMsg->hdr.status;
    paramUpdateEvt.opcode           =  pParamUpdateMsg->opcode;
    paramUpdateEvt.connectionHandle =  pParamUpdateMsg->connectionHandle;
    paramUpdateEvt.connInterval     =  pParamUpdateMsg->connInterval;
    paramUpdateEvt.connLatency      =  pParamUpdateMsg->connLatency;
    paramUpdateEvt.connTimeout      =  pParamUpdateMsg->connTimeout;

    // Send the event forward
    ConnectionExtCtrl_extHostEvtHandler((uint8_t *)&paramUpdateEvt,
                             sizeof(AppExtCtrlParamUpdateEvent_t));
  }
}

/*********************************************************************
 * @fn      ConnectionExtCtrl_linkEstEvtHandler
 *
 * @brief   This function handles the link termination event.
 *
 * @param   pEstMsg - pointer to the establish link message
 *
 * @return  None
 */
static void ConnectionExtCtrl_linkEstEvtHandler(gapEstLinkReqEvent_t *pEstMsg)
{
  if (pEstMsg != NULL)
  {
    AppExtCtrlEstLinkEvent_t estLinkEvt;

    estLinkEvt.event             = APP_EXTCTRL_LINK_ESTABLISHED_EVENT;
    estLinkEvt.status            = pEstMsg->hdr.status;
    estLinkEvt.opcode            = pEstMsg->opcode;
    estLinkEvt.devAddrType       = pEstMsg->devAddrType;
    memcpy(estLinkEvt.devAddr, pEstMsg->devAddr, B_ADDR_LEN);
    estLinkEvt.connectionHandle  = pEstMsg->connectionHandle;
    estLinkEvt.connRole          = pEstMsg->connRole;
    estLinkEvt.connInterval      = pEstMsg->connInterval;
    estLinkEvt.connLatency       = pEstMsg->connLatency;
    estLinkEvt.connTimeout       = pEstMsg->connTimeout;
    estLinkEvt.clockAccuracy     = pEstMsg->clockAccuracy;

    // Send the event forward
    ConnectionExtCtrl_extHostEvtHandler((uint8_t *)&estLinkEvt,
                             sizeof(AppExtCtrlEstLinkEvent_t));
  }
}

/*********************************************************************
 * @fn      ConnectionExtCtrl_connNotiConnEvtHandler
 *
 * @brief   This function handles the connection notification event.
 *
 * @param   pConnEvt - pointer to the connection notification message
 *
 * @return  None
 */
static void ConnectionExtCtrl_connNotiConnEvtHandler(Gap_ConnEventRpt_t *pConnEvt)
{
  if (pConnEvt != NULL)
  {
    AppExtCtrlConnCbEvt_t eventCbReport;

    eventCbReport.event = APP_EXTCTRL_CONN_NOTI_CONN_EVENT;

    eventCbReport.status         =  pConnEvt->status;
    eventCbReport.handle         =  pConnEvt->handle;
    eventCbReport.channel        =  pConnEvt->channel;
    eventCbReport.phy            =  pConnEvt->phy;
    eventCbReport.lastRssi       =  pConnEvt->lastRssi;
    eventCbReport.packets        =  pConnEvt->packets;
    eventCbReport.errors         =  pConnEvt->errors;
    eventCbReport.nextTaskType   =  pConnEvt->nextTaskType;
    eventCbReport.nextTaskTime   =  pConnEvt->nextTaskTime;
    eventCbReport.eventCounter   =  pConnEvt->eventCounter;
    eventCbReport.timeStamp      =  pConnEvt->timeStamp;
    eventCbReport.eventType      =  pConnEvt->eventType;

    // Send the event forward
    ConnectionExtCtrl_extHostEvtHandler((uint8_t *)&eventCbReport,
                                        sizeof(AppExtCtrlConnCbEvt_t));
  }
}

/*********************************************************************
 * @fn      ConnectionExtCtrl_connNotiTypeEvtHandler
 *
 * @brief   This function handles events from type @ref BLEAPPUTIL_CONN_NOTI_TYPE
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void ConnectionExtCtrl_connNotiTypeEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  switch (event)
  {
    case BLEAPPUTIL_CONN_NOTI_CONN_ESTABLISHED:
    case BLEAPPUTIL_CONN_NOTI_PHY_UPDATE:
    case BLEAPPUTIL_CONN_NOTI_CONN_EVENT_ALL:
    {
      ConnectionExtCtrl_connNotiConnEvtHandler((Gap_ConnEventRpt_t *)pMsgData);
      break;
    }

    default:
    {
      break;
    }
  }
}

/*********************************************************************
 * @fn      ConnectionExtCtrl_gapConnTypeEvtHandler
 *
 * @brief   This function handles events from type @ref BLEAPPUTIL_GAP_CONN_TYPE
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void ConnectionExtCtrl_gapConnTypeEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  switch (event)
  {
    case BLEAPPUTIL_LINK_ESTABLISHED_EVENT:
    {
      ConnectionExtCtrl_linkEstEvtHandler((gapEstLinkReqEvent_t *)pMsgData);
      break;
    }

    case BLEAPPUTIL_LINK_TERMINATED_EVENT:
    {
      ConnectionExtCtrl_linkTermEvtHandler((gapTerminateLinkEvent_t *)pMsgData);
      break;
    }

    case BLEAPPUTIL_LINK_PARAM_UPDATE_EVENT:
    case BLEAPPUTIL_LINK_PARAM_UPDATE_REJECT_EVENT:
    case BLEAPPUTIL_LINK_PARAM_UPDATE_REQ_EVENT:
    {
      AppExtCtrlEvent_e mappedEvent;

      switch(event) 
      {
        case BLEAPPUTIL_LINK_PARAM_UPDATE_EVENT:
          mappedEvent = APP_EXTCTRL_LINK_PARAM_UPDATE_EVENT;
          break;
        case BLEAPPUTIL_LINK_PARAM_UPDATE_REJECT_EVENT:
          mappedEvent = APP_EXTCTRL_LINK_PARAM_UPDATE_REJECT_EVENT;
          break;
        case BLEAPPUTIL_LINK_PARAM_UPDATE_REQ_EVENT:
          mappedEvent = APP_EXTCTRL_LINK_PARAM_UPDATE_REQ_EVENT;
          break;
        default:
          mappedEvent = APP_EXTCTRL_LINK_PARAM_UPDATE_EVENT;
          break;
      }
      ConnectionExtCtrl_paramUpdateEvtHandler((gapLinkUpdateEvent_t *)pMsgData, (AppExtCtrlEvent_e)mappedEvent);
      break;
    }
    
    default:
    {
      break;
    }
  }
}

/*********************************************************************
 * @fn      ConnectionExtCtrl_eventHandler
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
static void ConnectionExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{

  if (pMsgData != NULL)
  {
    switch (eventType)
    {
      case BLEAPPUTIL_GAP_CONN_TYPE:
      {
        ConnectionExtCtrl_gapConnTypeEvtHandler(event, pMsgData);
        break;
      }

      case BLEAPPUTIL_CONN_NOTI_TYPE:
      {
        ConnectionExtCtrl_connNotiTypeEvtHandler(event, pMsgData);
        break;
      }

      default :
      {
        break;
      }
    }
  }
}

/*********************************************************************
 * @fn      ConnectionExtCtrl_extHostEvtHandler
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
static void ConnectionExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if (gExtHostEvtHandler != NULL && pData != NULL)
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      ConnectionExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the connection module
 *          to the external control dispatcher, and register the call back
 *          event handler function to the connection application.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t ConnectionExtCtrl_start(void)
{
  bStatus_t status = SUCCESS;

  // Register to the Dispatcher module
  gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_CONNECTION,
                                                     &ConnectionExtCtrl_commandParser,
                                                     APP_CAP_CONNECTION);

  // If the registration succeed, register event handler call back to the connection application
  if (gExtHostEvtHandler != NULL)
  {
    Connection_registerEvtHandler(&ConnectionExtCtrl_eventHandler);
  }
  return status;
}
