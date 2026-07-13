/******************************************************************************

@file  app_extctrl_ca_server.c

@brief This file parse and process the messages comes form the external control
 dispatcher module, build the events from the app_ca_server.c application and
 send it back to the external control dispatcher module.

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

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_extctrl_dispatcher.h"
#include "app_extctrl_ca_server.h"
#include <string.h>

/*********************************************************************
 * PROTOTYPES
 */
static void CAServerExtCtrl_sendErrEvt(uint8_t, uint8_t);
static void CAServerExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);
static void CAServerExtCtrl_commandParser(uint8_t *pData);
static void CAServerExtCtrl_extHostEvtHandler(uint8_t*, uint16_t);

/*********************************************************************
 * CONSTANTS
 */
#define CASERVER_APP_SEND_INDICATION   0x00

/*********************************************************************
 * GLOBALS
 */
// The external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      CAServerExtCtrl_commandParser
 *
 * @brief   This function parse the received host message, and call the
 *          the relevant API with the data.
 *
 * @param   pData - pointer to the data message.
 *
 * @return  None
 */
static void CAServerExtCtrl_commandParser(uint8_t *pData)
{
  if (pData != NULL)
  {
    appMsg_t* appMsg = (appMsg_t*)pData;
    bStatus_t status = SUCCESS;

    switch (appMsg->cmdOp)
    {
      case CASERVER_APP_SEND_INDICATION:
      {
        uint16_t connHandle = BUILD_UINT16(appMsg->pData[0],
                                           appMsg->pData[1]);

        status = CAServer_sendIndication(connHandle);
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
        CAServerExtCtrl_sendErrEvt(status, appMsg->cmdOp);
    }
  }
}

/*********************************************************************
 * @fn      CAServerExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void CAServerExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
  // Create error event structure
  AppExtCtrlErrorEvent_t errEvt;
  errEvt.event        = APP_EXTCTRL_FAILURE;
  errEvt.cmdOp        = cmdOp;
  errEvt.appSpecifier = APP_SPECIFIER_CA_SERVER;
  errEvt.errCode      = errCode;

  // Send error event
  CAServerExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

/*********************************************************************
 * @fn      CAServerExtCtrl_CCCUpdateEvtHandler
 *
 * @brief   This function handles the Car Access Service CCC Update event
 *
 * @param   pCCCUpdate   - pointer to the CCC update message
 * @param   extCtrlEvent - the event that should be sent to the external control
 *
 * @return  None
 */
static void CAServerExtCtrl_CCCUpdateEvtHandler(CAServer_CCCpdateEvent_t *pCCCUpdate, AppExtCtrlEvent_e extCtrlEvent)
{
  if (pCCCUpdate != NULL)
  {
    AppExtCtrlCAServerCCCpdateEvent_t cccUpdateEvt;

    cccUpdateEvt.event      = extCtrlEvent;
    cccUpdateEvt.connHandle = pCCCUpdate->connHandle;
    cccUpdateEvt.pValue     = pCCCUpdate->pValue;

    CAServerExtCtrl_extHostEvtHandler((uint8_t *)&cccUpdateEvt, sizeof(AppExtCtrlCAServerCCCpdateEvent_t));
  }
}

/*********************************************************************
 * @fn      CAServerExtCtrl_incomingDataEvtHandler
 *
 * @brief   This function handles the Car Access Service incoming data event
 *
 * @param   pIncData     - pointer to the incoming data message
 * @param   extCtrlEvent - the event that should be sent to the external control
 *
 * @return  None
 */
static void CAServerExtCtrl_incomingDataEvtHandler(CAServer_incomingDataEvent_t *pIncData, AppExtCtrlEvent_e extCtrlEvent)
{
  if (pIncData != NULL)
  {
    AppExtCtrlCAServerIncomingDataEvent_t *incDataEvt = (AppExtCtrlCAServerIncomingDataEvent_t *)ICall_malloc(sizeof(AppExtCtrlCAServerIncomingDataEvent_t) + pIncData->len);

    incDataEvt->event      = extCtrlEvent;
    incDataEvt->connHandle = pIncData->connHandle;
    incDataEvt->len        = pIncData->len;
    memcpy(incDataEvt->pValue, pIncData->pValue, pIncData->len);

    CAServerExtCtrl_extHostEvtHandler((uint8_t *)incDataEvt, sizeof(AppExtCtrlCAServerIncomingDataEvent_t) + pIncData->len);
    ICall_free(incDataEvt);
  }
}

/*********************************************************************
 * @fn      CAServerExtCtrl_indCnfEvtHandler
 *
 * @brief   This function handles the Car Access Service indication confirmation event
 *
 * @param   pIndCnf      - pointer to the CCC update message
 * @param   extCtrlEvent - the event that should be sent to the external control
 *
 * @return  None
 */
static void CAServerExtCtrl_indCnfEvtHandler(CAServer_indCnfEvent_t *pIndCnf, AppExtCtrlEvent_e extCtrlEvent)
{
  if (pIndCnf != NULL)
  {
    AppExtCtrlCAServerIndCnfEvent_t indCnfEvt;

    indCnfEvt.event      = extCtrlEvent;
    indCnfEvt.connHandle = pIndCnf->connHandle;
    indCnfEvt.status     = pIndCnf->status;

    CAServerExtCtrl_extHostEvtHandler((uint8_t *)&indCnfEvt, sizeof(AppExtCtrlCAServerIndCnfEvent_t));
  }
}

/*********************************************************************
 * @fn      CAServerExtCtrl_genericTypeEvtHandler
 *
 * @brief   This function handles events from type @ref BLEAPPUTIL_GATT_TYPE
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void CAServerExtCtrl_genericTypeEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (event)
    {
      case APP_EXTCTRL_CA_SERVER_CCC_UPDATE:
      {
          CAServerExtCtrl_CCCUpdateEvtHandler((CAServer_CCCpdateEvent_t *)pMsgData,
                                         APP_EXTCTRL_CA_SERVER_CCC_UPDATE);
        break;
      }
      case APP_EXTCTRL_CA_SERVER_INDICATION_CNF:
      {
          CAServerExtCtrl_indCnfEvtHandler((CAServer_indCnfEvent_t *)pMsgData,
                                           APP_EXTCTRL_CA_SERVER_INDICATION_CNF);
        break;
      }
      case APP_EXTCTRL_CA_SERVER_LONG_WRITE_DONE:
      {
          CAServerExtCtrl_incomingDataEvtHandler((CAServer_incomingDataEvent_t *)pMsgData,
                                                 APP_EXTCTRL_CA_SERVER_LONG_WRITE_DONE);
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
 * @fn      CAServerExtCtrl_eventHandler
 *
 * @brief   This function handles the events raised from the application that
 *          this module registered to.
 *
 * @param   eventType - the type of the events @ref BLEAppUtil_eventHandlerType_e.
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void CAServerExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (eventType)
    {
      case BLEAPPUTIL_GENERIC_TYPE:
      {
        CAServerExtCtrl_genericTypeEvtHandler(event, pMsgData);
      }

      default :
      {
        break;
      }
    }
  }
}

/*********************************************************************
 * @fn      CAServerExtCtrl_extHostEvtHandler
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
static void CAServerExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if (gExtHostEvtHandler != NULL && pData != NULL)
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      CAServerExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the CA Server module
 *          to the external control dispatcher, and register the call back
 *          event handler function to the Car Access server application.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t CAServerExtCtrl_start(void)
{
  bStatus_t status = SUCCESS;

  // Register to the Dispatcher module
  gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_CA_SERVER,
                                                     &CAServerExtCtrl_commandParser,
                                                     APP_CAP_CA_SERVER);

  // If the registration succeed, register event handler call back to the data application
  if (gExtHostEvtHandler != NULL)
  {
    CAServer_registerEvtHandler(&CAServerExtCtrl_eventHandler);
  }
  return status;
}
