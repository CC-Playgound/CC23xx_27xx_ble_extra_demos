/******************************************************************************

@file  app_extctrl_gatt.c

@brief This file parse and process the messages comes form the external control
 dispatcher module, build the events from the app_gatt.c application and
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
#include "app_extctrl_gatt.h"
#include <string.h>

/*********************************************************************
 * PROTOTYPES
 */
static void AppGATTExtCtrl_sendErrEvt(uint8_t, uint8_t);
static void AppGATTExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);
static void AppGATTExtCtrl_commandParser(uint8_t *pData);
static void AppGATTExtCtrl_extHostEvtHandler(uint8_t*, uint16_t);

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
#define APP_GATT_MTU_EXCHANGE   0x00

/*********************************************************************
 * GLOBALS
 */
// The external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      AppGATTExtCtrl_commandParser
 *
 * @brief   This function parse the received host message, and call the
 *          the relevant API with the data.
 *
 * @param   pData - pointer to the data message.
 *
 * @return  None
 */
static void AppGATTExtCtrl_commandParser(uint8_t *pData)
{
  if (pData != NULL)
  {
    appMsg_t* appMsg = (appMsg_t*)pData;
    bStatus_t status = SUCCESS;

    switch (appMsg->cmdOp)
    {
      case APP_GATT_MTU_EXCHANGE:
      {
        AppGATTExtCtrlMTUUExchangeCMD_t *mtuExchangeCMD = (AppGATTExtCtrlMTUUExchangeCMD_t *)appMsg->pData;

        status = AppGATT_exchangeMTU(mtuExchangeCMD->connHandle, mtuExchangeCMD->mtu);
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
        AppGATTExtCtrl_sendErrEvt(status, appMsg->cmdOp);
    }
  }
}

/*********************************************************************
 * @fn      AppGATTExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void AppGATTExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
  // Create error event structure
  AppExtCtrlErrorEvent_t errEvt;
  errEvt.event        = APP_EXTCTRL_FAILURE;
  errEvt.cmdOp        = cmdOp;
  errEvt.appSpecifier = APP_SPECIFIER_GATT;
  errEvt.errCode      = errCode;

  // Send error event
  AppGATTExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

/*********************************************************************
 * @fn      AppGATTExtCtrl_mtuUpdatedEvtHandler
 *
 * @brief   This function handles the GATT MTU updated event
 *
 * @param   pGATTMsg    - pointer to the gatt message
 * @param   extCtrlEvent - the event that should be sent to the external control
 *
 * @return  None
 */
static void AppGATTExtCtrl_mtuUpdatedEvtHandler(gattMsgEvent_t *pGATTMsg, AppExtCtrlEvent_e extCtrlEvent)
{
  if (pGATTMsg != NULL)
  {
    AppExtCtrlMTUUpdatedEvent_t dataStateEvt;

    dataStateEvt.event      = extCtrlEvent;
    dataStateEvt.connHandle = pGATTMsg->connHandle;
    dataStateEvt.mtu        = pGATTMsg->msg.mtuEvt.MTU;

    AppGATTExtCtrl_extHostEvtHandler((uint8_t *)&dataStateEvt, sizeof(AppExtCtrlMTUUpdatedEvent_t));
  }
}

/*********************************************************************
 * @fn      AppGATTExtCtrl_gattTypeEvtHandler
 *
 * @brief   This function handles events from type @ref BLEAPPUTIL_GATT_TYPE
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void AppGATTExtCtrl_gattTypeEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (event)
    {
      case BLEAPPUTIL_ATT_MTU_UPDATED_EVENT:
      {
        AppGATTExtCtrl_mtuUpdatedEvtHandler((gattMsgEvent_t *)pMsgData,
                                         APP_EXTCTRL_GATT_MTU_UPDATED_EVENT);
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
 * @fn      AppGATTExtCtrl_eventHandler
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
static void AppGATTExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (eventType)
    {
      case BLEAPPUTIL_GATT_TYPE:
      {
        AppGATTExtCtrl_gattTypeEvtHandler(event, pMsgData);
      }

      default :
      {
        break;
      }
    }
  }
}

/*********************************************************************
 * @fn      AppGATTExtCtrl_extHostEvtHandler
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
static void AppGATTExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if (gExtHostEvtHandler != NULL && pData != NULL)
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      AppGATTExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the data module
 *          to the external control dispatcher, and register the call back
 *          event handler function to the data application.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t AppGATTExtCtrl_start(void)
{
  bStatus_t status = SUCCESS;

  // Register to the Dispatcher module
  gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_GATT,
                                                     &AppGATTExtCtrl_commandParser,
                                                     APP_CAP_GATT);

  // If the registration succeed, register event handler call back to the data application
  if (gExtHostEvtHandler != NULL)
  {
    AppGATT_registerEvtHandler(&AppGATTExtCtrl_eventHandler);
  }
  return status;
}
