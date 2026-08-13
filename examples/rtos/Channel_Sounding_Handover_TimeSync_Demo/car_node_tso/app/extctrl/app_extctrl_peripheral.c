/******************************************************************************

@file  app_extctrl_peripheral.c

@brief This file parse and process the messages comes form the external control
 dispatcher module, and build the events from the app_peripheral.c application and
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
#include "app_extctrl_peripheral.h"
#include <string.h>

/*********************************************************************
 * PROTOTYPES
 */
static void PeripheralExtCtrl_sendErrEvt(uint8_t, uint8_t);
static void PeripheralExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);
static void PeripheralExtCtrl_commandParser(uint8_t *pData);
static void PeripheralExtCtrl_extHostEvtHandler(uint8_t*, uint16_t);

/*********************************************************************
 * CONSTANTS
 */
#define PERI_CMD_ADVERTISE_START    0x00
#define PERI_CMD_ADVERTISE_STOP     0x01

/*********************************************************************
 * GLOBALS
 */
// The external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      Connection_commandParser
 *
 * @brief   This function parse the received host message, and call the
 *          the relevant API with the data.
 *
 * @param   pData - pointer to the data message.
 *
 * @return  None
 */
static void PeripheralExtCtrl_commandParser(uint8_t *pData)
{
  if (pData != NULL)
  {
    appMsg_t* appMsg = (appMsg_t*)pData;
    bStatus_t status = SUCCESS;

    switch (appMsg->cmdOp)
    {
      case PERI_CMD_ADVERTISE_START:
      {
        Peripheral_advStartCmdParams_t *pParams = (Peripheral_advStartCmdParams_t *)appMsg->pData;
        status = Peripheral_advStart(pParams);
        break;
      }
      case PERI_CMD_ADVERTISE_STOP:
      {
        uint8_t *pHandle = (uint8_t *)appMsg->pData;
        status = Peripheral_advStop(pHandle);
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
      PeripheralExtCtrl_sendErrEvt(status, appMsg->cmdOp);
    }
  }
}

/*********************************************************************
 * @fn      PeripheralExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void PeripheralExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
  // Create error event structure
  AppExtCtrlErrorEvent_t errEvt;
  errEvt.event        = APP_EXTCTRL_FAILURE;
  errEvt.cmdOp        = cmdOp;
  errEvt.appSpecifier = APP_SPECIFIER_PERIPHERAL;
  errEvt.errCode      = errCode;

  // Send error event
  PeripheralExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

/*********************************************************************
 * @fn      PeripheralExtCtrl_advEvtHandler
 *
 * @brief   This function handles the advertise event.
 *
 * @param   pAdvData - pointer to the advertise message
 * @param   extCtrlEvent - the event that should be sent to the external control
 *
 * @return  None
 */
static void PeripheralExtCtrl_advEvtHandler(BLEAppUtil_AdvEventData_t *pAdvData, AppExtCtrlEvent_e extCtrlEvent)
{
  if (pAdvData != NULL)
  {
    AppExtCtrlAdvStatusEvt_t advStatusEvt;
    advStatusEvt.event = extCtrlEvent;
    advStatusEvt.advHandle = *((uint8_t *)pAdvData->pBuf);

    PeripheralExtCtrl_extHostEvtHandler((uint8_t *)&advStatusEvt, sizeof(AppExtCtrlAdvStatusEvt_t));
  }
}

/*********************************************************************
 * @fn      PeripheralExtCtrl_gapAdvTypeEvtHandler
 *
 * @brief   This function handles events from type @ref BLEAPPUTIL_GAP_ADV_TYPE
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void PeripheralExtCtrl_gapAdvTypeEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    BLEAppUtil_AdvEventData_t *pAdvData = (BLEAppUtil_AdvEventData_t *)pMsgData;
    switch (event)
    {
      case BLEAPPUTIL_ADV_START_AFTER_ENABLE:
      {
        PeripheralExtCtrl_advEvtHandler(pAdvData, APP_EXTCTRL_ADV_START_AFTER_ENABLE);
        break;
      }

      case BLEAPPUTIL_ADV_END_AFTER_DISABLE:
      {
        PeripheralExtCtrl_advEvtHandler(pAdvData, APP_EXTCTRL_ADV_END_AFTER_DISABLE);
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
 * @fn      PeripheralExtCtrl_eventHandler
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
static void PeripheralExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (eventType)
    {
      case BLEAPPUTIL_GAP_ADV_TYPE:
      {
        PeripheralExtCtrl_gapAdvTypeEvtHandler(event, pMsgData);
      }

      default :
      {
        break;
      }
    }
  }
}

/*********************************************************************
 * @fn      PeripheralExtCtrl_extHostEvtHandler
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
static void PeripheralExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if (gExtHostEvtHandler != NULL && pData != NULL)
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      PeripheralExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the peripheral module
 *          to the external control dispatcher, and register the call back
 *          event handler function to the peripheral application.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t PeripheralExtCtrl_start(void)
{
  bStatus_t status = SUCCESS;

  // Register to the Dispatcher module
  gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_PERIPHERAL,
                                                     &PeripheralExtCtrl_commandParser,
                                                     APP_CAP_PERIPHERAL);

  // If the registration succeed, register event handler call back to the peripheral application
  if (gExtHostEvtHandler != NULL)
  {
    Peripheral_registerEvtHandler(&PeripheralExtCtrl_eventHandler);
  }
  return status;
}
