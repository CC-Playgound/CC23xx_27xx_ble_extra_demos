/******************************************************************************

@file  app_extctrl_ranging_client.c

@brief This file parse and process the messages comes form the external control
 dispatcher module, and build the events from the app_ranging_client.c application and
 send it to the external control dispatcher module back.

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

#if defined(RANGING_CLIENT_EXTCTRL_APP) && defined(RANGING_CLIENT)
/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_extctrl_dispatcher.h"
#include "app_extctrl_ranging_client.h"
#include <string.h>

/*********************************************************************
 * PROTOTYPES
 */
static void AppRREQExtCtrl_sendErrEvt(uint8_t, uint8_t);
static void AppRREQExtCtrl_eventHandler(AppRREQEventType_e event, BLEAppUtil_msgHdr_t *pMsgData);
static void AppRREQExtCtrl_commandParser(uint8_t *pData);
static void AppRREQExtCtrl_extHostEvtHandler(uint8_t*, uint16_t);

/*********************************************************************
 * TYPEDEFS
 */
typedef struct __attribute__((packed))
{
  uint16_t connHandle;                  //!< Connection handle
  uint16_t mtu;                          //!< New MTU size
} AppGATTExtCtrlMTUUExchangeCMD_t;

/*********************************************************************
 * CONSTANTS
 */
#define APP_RREQ_START                  0x00
#define APP_RREQ_ENABLE                 0x01
#define APP_RREQ_DISABLE                0x02
#define APP_RREQ_GET_RANGING_DATA       0x03
#define APP_RREQ_ABORT                  0x04
#define APP_RREQ_CONFIGURE_CHAR         0x05
#define APP_RREQ_CS_PROCEDURE_STARTED   0x06

/*********************************************************************
 * GLOBALS
 */
// The external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      AppRREQExtCtrl_commandParser
 *
 * @brief   This function parse the received host message, and call the
 *          the relevant API with the data.
 *
 * @param   pData - pointer to the data message.
 *
 * @return  None
 */
static void AppRREQExtCtrl_commandParser(uint8_t *pData)
{
  if (pData != NULL)
  {
    appMsg_t *appMsg = (appMsg_t*)pData;
    bStatus_t status = SUCCESS;

    switch (appMsg->cmdOp)
    {
      case APP_RREQ_START:
      {
        status = AppRREQ_start();
        break;
      }

      case APP_RREQ_ENABLE:
      {
        AppExtCtrlRREQEnableCmd_t *pEnableCmdData = (AppExtCtrlRREQEnableCmd_t *)appMsg->pData;
        status = AppRREQ_enable(pEnableCmdData->connHandle, (RREQEnableModeType_e) pEnableCmdData->enableMode);
        break;
      }

      case APP_RREQ_DISABLE:
      {
        AppExtCtrlRREQEnableCmd_t *pDisableCmdData = (AppExtCtrlRREQEnableCmd_t *)appMsg->pData;
        status = AppRREQ_disable(pDisableCmdData->connHandle);
        break;
      }

      case APP_RREQ_GET_RANGING_DATA:
      {
        AppExtCtrlRREQGetRangingDataCmd_t *pGetDataCmdData = (AppExtCtrlRREQGetRangingDataCmd_t *)appMsg->pData;
        status = AppRREQ_GetRangingData(pGetDataCmdData->connHandle, pGetDataCmdData->rangingCount);
        break;
      }

      case APP_RREQ_ABORT:
      {
        AppExtCtrlRREQAbortCmd_t *pAbortCmdData = (AppExtCtrlRREQAbortCmd_t *)appMsg->pData;
        status = AppRREQ_abort(pAbortCmdData->connHandle);
        break;
      }

      case APP_RREQ_CONFIGURE_CHAR:
      {
        AppExtCtrlRREQConfigureCharCmd_t *pConfigureCharCmdData = (AppExtCtrlRREQConfigureCharCmd_t *)appMsg->pData;
        status = AppRREQ_configureCharRegistration(pConfigureCharCmdData->connHandle, pConfigureCharCmdData->charUUID, pConfigureCharCmdData->subscribeMode);
        break;
      }

      case APP_RREQ_CS_PROCEDURE_STARTED:
      {
        AppExtCtrlRREQCSProcedureStartCmd_t *pCSEventCmdData = (AppExtCtrlRREQCSProcedureStartCmd_t *)appMsg->pData;
        status = AppRREQ_procedureStarted(pCSEventCmdData->connHandle, pCSEventCmdData->procedureCounter);
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
      AppRREQExtCtrl_sendErrEvt(status, appMsg->cmdOp);
    }
  }
}

/*********************************************************************
 * @fn      AppRREQExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void AppRREQExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
  // Create error event structure
  AppExtCtrlErrorEvent_t errEvt;
  errEvt.event        = APP_EXTCTRL_FAILURE;
  errEvt.cmdOp        = cmdOp;
  errEvt.appSpecifier = APP_SPECIFIER_RREQ;
  errEvt.errCode      = errCode;

  // Send error event
  AppRREQExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

/*********************************************************************
 * @fn      AppRREQExtCtrl_dataReadyEvtHandler
 *
 * @brief   This function handles the data ready event
 *
 * @param   pMsg    - pointer to the gatt message
 * @param   extCtrlEvent - the event that should be sent to the external control
 *
 * @return  None
 */
static void AppRREQExtCtrl_dataReadyEvtHandler(AppRREQDataReady_t *pMsg, AppExtCtrlEvent_e extCtrlEvent)
{
  if (pMsg != NULL)
  {
    AppExtCtrlRREQDataReady_t dataReadyEvt;

    dataReadyEvt.event        = extCtrlEvent;
    dataReadyEvt.connHandle   = pMsg->connHandle;
    dataReadyEvt.rangingCount = pMsg->rangingCount;

    AppRREQExtCtrl_extHostEvtHandler((uint8_t *)&dataReadyEvt, sizeof(AppExtCtrlRREQDataReady_t));
  }
}

/*********************************************************************
 * @fn      AppRREQExtCtrl_statusEvtHandle
 *
 * @brief   This function handles the status event
 *
 * @param   pMsg - pointer to the status message
 * @param   extCtrlEvent - the event that should be sent to the external control
 *
 * @return  None
 */
static void AppRREQExtCtrl_statusEvtHandle(AppRREQStatus_t *pMsg, AppExtCtrlEvent_e extCtrlEvent)
{
  if (pMsg != NULL)
  {
    AppExtCtrlRREQStatus_t *pStatusEvt;

    pStatusEvt = ICall_malloc( sizeof(AppExtCtrlRREQStatus_t) + pMsg->statusDataLen);
    if(pStatusEvt != NULL)
    {
        pStatusEvt->event          = extCtrlEvent;
        pStatusEvt->connHandle     = pMsg->connHandle;
        pStatusEvt->status         = pMsg->statusCode;
        pStatusEvt->statusDataLen  = pMsg->statusDataLen;

        memcpy( pStatusEvt->statusData, pMsg->statusData, pMsg->statusDataLen);

        AppRREQExtCtrl_extHostEvtHandler((uint8_t *)pStatusEvt, sizeof(AppExtCtrlRREQStatus_t) + pMsg->statusDataLen );

        ICall_free(pStatusEvt);
    }
  }
}

/*********************************************************************
 * @fn      AppRREQExtCtrl_eventHandler
 *
 * @brief   This function handles the events raised from the application that
 *          this module registered to.
 *
 * @param   event     - rreq message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void AppRREQExtCtrl_eventHandler(AppRREQEventType_e event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (event)
    {
      case APP_RREQ_EVENT_DATA_READY:
      {
          AppRREQExtCtrl_dataReadyEvtHandler((AppRREQDataReady_t *)pMsgData, APP_EXTCTRL_RREQ_DATA_READY);
        break;
      }

      case APP_RREQ_EVENT_STATUS:
      {
        AppRREQExtCtrl_statusEvtHandle((AppRREQStatus_t *)pMsgData, APP_EXTCTRL_RREQ_STATUS);
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
 * @fn      AppRREQExtCtrl_extHostEvtHandler
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
static void AppRREQExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if ((gExtHostEvtHandler != NULL) && (pData != NULL))
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      AppRREQExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the data module
 *          to the external control dispatcher, and register the call back
 *          event handler function to the data application.
 *
 * @return  SUCCESS/FAILURE
 */
uint8_t AppRREQExtCtrl_start(void)
{
  uint8_t status = SUCCESS;

  // Register to the Dispatcher module
  gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_RREQ,
                                                     &AppRREQExtCtrl_commandParser,
                                                     APP_CAP_RREQ);

  // If the registration succeed, register event handler call back to the data application
  if (gExtHostEvtHandler != NULL)
  {
    AppRREQ_registerEvtHandler(&AppRREQExtCtrl_eventHandler);
  }
  return status;
}

#endif // defined(RANGING_CLIENT_EXTCTRL_APP) && defined(RANGING_CLIENT)
