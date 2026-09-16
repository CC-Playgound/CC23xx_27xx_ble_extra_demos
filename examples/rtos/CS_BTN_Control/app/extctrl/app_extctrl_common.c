/******************************************************************************

@file  app_extctrl_common.c

@brief This file parse and process the messages comes form the external control module
 dispatcher module, and build the events from the app_common.c application and
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
#include "app_extctrl_common.h"
#include "app_common_api.h"
#include <string.h>

/*********************************************************************
 * PROTOTYPES
 */
static void CommonExtCtrl_sendErrEvt(uint8_t, uint8_t);
static void CommonExtCtrl_commandParser(uint8_t*);
static void CommonExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);
static void CommonExtCtrl_extHostEvtHandler(uint8_t*, uint16_t);
static void CommonExtCtrl_deviceAddressEvt(GAP_Addr_Modes_t addrMode, uint8_t *pIdAddr, uint8_t *pRpAddr);

/*********************************************************************
 * CONSTANTS
 */
#define COMMON_CMD_GET_DEV_ADDR    0x00
#define COMMON_CMD_RESET_DEVICE    0x01

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBALS
 */
// The external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      CommonExtCtrl_commandParser
 *
 * @brief   This function parse the received host message, and call the
 *          the relevant API with the data.
 *
 * @param   pData - pointer to the data message.
 *
 * @return  None
 */
static void CommonExtCtrl_commandParser(uint8_t *pData)
{
  if (pData != NULL)
  {
    appMsg_t* appMsg = (appMsg_t*)pData;
    bStatus_t status = SUCCESS;

    switch (appMsg->cmdOp)
    {
      case COMMON_CMD_GET_DEV_ADDR:
      {
        GAP_Addr_Modes_t devAddrMode;
        uint8_t idAddr[B_ADDR_LEN] = {0};
        uint8_t rpAddr[B_ADDR_LEN] = {0};

        status = Common_getDevAddress(&devAddrMode, idAddr, rpAddr);

        if (status == SUCCESS)
        {
          // Send the event
          CommonExtCtrl_deviceAddressEvt(devAddrMode, idAddr, rpAddr);
        }
        break;
      }

      case COMMON_CMD_RESET_DEVICE:
      {
        // Reset the device
        Common_resetDevice();
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
      CommonExtCtrl_sendErrEvt(status, appMsg->cmdOp);
    }
  }
}

/*********************************************************************
 * @fn      CommonExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void CommonExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
  // Create error event structure
  AppExtCtrlErrorEvent_t errEvt;
  errEvt.event        = APP_EXTCTRL_FAILURE;
  errEvt.cmdOp        = cmdOp;
  errEvt.appSpecifier = APP_SPECIFIER_COMMON;
  errEvt.errCode      = errCode;

  // Send error event
  CommonExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

/*********************************************************************
 * @fn      CommonExtCtrl_deviceAddressEvt
 *
 * @brief   This function sends the ID and RP addresses as event to the external control.
 *
 * @return  None
 */
static void CommonExtCtrl_deviceAddressEvt(GAP_Addr_Modes_t addrMode, uint8_t *pIdAddr, uint8_t *pRpAddr)
{
    // Create error event structure
    AppExtCtrlConnDevAddressEvt_t deviceAddressEvt;

    deviceAddressEvt.event          = APP_EXTCTRL_COMMON_DEVICE_ADDRESS;
    deviceAddressEvt.devAddressMode = addrMode;
    memcpy(deviceAddressEvt.idAddr, pIdAddr, B_ADDR_LEN);
    memcpy(deviceAddressEvt.rpAddr, pRpAddr, B_ADDR_LEN);

    // Send event
    CommonExtCtrl_extHostEvtHandler((uint8_t *)&deviceAddressEvt, sizeof(AppExtCtrlConnDevAddressEvt_t));
}

/*********************************************************************
 * @fn      CommonExtCtrl_cmdStatusEvtHandler
 *
 * @brief   This function handles the command status event.
 *
 * @param   pCmdStatus - pointer to the command status event
 *
 * @return  None
 */
static void CommonExtCtrl_cmdStatusEvtHandler(hciEvt_CommandStatus_t *pCmdStatus)
{
  if (pCmdStatus != NULL)
  {
    AppExtCtrlCommandStatusEvt_t cmdStatusEvent;

    cmdStatusEvent.event = APP_EXTCTRL_COMMON_COMMAND_STATUS_EVENT;
    cmdStatusEvent.cmdStatus    =  pCmdStatus->cmdStatus;
    cmdStatusEvent.numHciCmdPkt =  pCmdStatus->numHciCmdPkt;
    cmdStatusEvent.cmdOpcode    =  pCmdStatus->cmdOpcode;
    // Send the event forward
    CommonExtCtrl_extHostEvtHandler((uint8_t *)&cmdStatusEvent,
                                    sizeof(AppExtCtrlCommandStatusEvt_t));
  }
}

/*********************************************************************
 * @fn      CommonExtCtrl_hciLeEvtHandler
 *
 * @brief   This function handles @ref BLEAPPUTIL_HCI_LE_EVENT_CODE
 *
 * @param   pMsgData - pointer message data
 *
 * @return  None
 *
 */
static void CommonExtCtrl_hciLeEvtHandler(BLEAppUtil_msgHdr_t *pMsgData)
{
  uint8_t sendTheEvent = FALSE;

  if (pMsgData != NULL)
  {
    uint8_t *pPtr;
    AppExtCtrlHciLeEvent_t *pHciLeEvent;
    hciEvtHdr_t *pHciHdr = (hciEvtHdr_t *)pMsgData;

    switch(pHciHdr->BLEEventCode)
    {
      case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
      {
        hciEvt_BLEPhyUpdateComplete_t *pPhyUpdateEvt  = (hciEvt_BLEPhyUpdateComplete_t*) pMsgData;
        pHciLeEvent = ICall_malloc(sizeof(AppExtCtrlCommandStatusEvt_t) + EXTCTRL_PHY_UPDATE_COMPLETE_EVENT_DATA_LEN);

        if (pHciLeEvent != NULL)
        {
          pHciLeEvent->BLEEventCode = pPhyUpdateEvt ->BLEEventCode;
          pHciLeEvent->len          = EXTCTRL_PHY_UPDATE_COMPLETE_EVENT_DATA_LEN;

          // Point to the allocated data
          pPtr = pHciLeEvent->pData;

          // Copy the attributes
          memcpy(pPtr, &(pPhyUpdateEvt ->status), sizeof(uint8_t));
          pPtr++;
          memcpy(pPtr, &(pPhyUpdateEvt ->connHandle), sizeof(uint16_t));
          pPtr += sizeof(uint16_t);
          memcpy(pPtr, &(pPhyUpdateEvt ->txPhy), sizeof(uint8_t));
          pPtr++;
          memcpy(pPtr, &(pPhyUpdateEvt ->rxPhy), sizeof(uint8_t));

          sendTheEvent = TRUE;
        }
        break;
      }

      case HCI_BLE_DATA_LENGTH_CHANGE_EVENT:
      {
        hciEvt_BLEDataLengthChange_t *pDataLengthChangeEvt  = (hciEvt_BLEDataLengthChange_t*) pMsgData;
        pHciLeEvent = ICall_malloc(sizeof(AppExtCtrlCommandStatusEvt_t) + EXTCTRL_DATA_LENGTH_CHANGE_EVENT_DATA_LEN);

        if (pHciLeEvent != NULL)
        {
          pHciLeEvent->BLEEventCode = pDataLengthChangeEvt ->BLEEventCode;
          pHciLeEvent->len          = EXTCTRL_DATA_LENGTH_CHANGE_EVENT_DATA_LEN;

          // Point to the allocated data
          pPtr = pHciLeEvent->pData;

          // Copy the attributes
          memcpy(pPtr, &(pDataLengthChangeEvt ->connHandle), sizeof(uint16_t));
          pPtr += sizeof(uint16_t);
          memcpy(pPtr, &(pDataLengthChangeEvt ->maxTxOctets), sizeof(uint16_t));
          pPtr += sizeof(uint16_t);
          memcpy(pPtr, &(pDataLengthChangeEvt ->maxTxTime), sizeof(uint16_t));
          pPtr += sizeof(uint16_t);
          memcpy(pPtr, &(pDataLengthChangeEvt ->maxRxOctets), sizeof(uint16_t));
          pPtr += sizeof(uint16_t);
          memcpy(pPtr, &(pDataLengthChangeEvt ->maxRxTime), sizeof(uint16_t));

          sendTheEvent = TRUE;
        }
        break;
      }

      case HCI_BLE_CHANNEL_MAP_UPDATE_EVENT:
      {
        hciEvt_BLEChannelMapUpdate_t *pChannelMapUpdateEvt  = (hciEvt_BLEChannelMapUpdate_t*) pMsgData;
        pHciLeEvent = ICall_malloc(sizeof(AppExtCtrlCommandStatusEvt_t) + EXTCTRL_CHANNEL_MAP_UPDATE_EVENT_DATA_LEN);

        if(pHciLeEvent != NULL)
        {
          pHciLeEvent->BLEEventCode = pChannelMapUpdateEvt ->BLEEventCode;
          pHciLeEvent->len          = EXTCTRL_CHANNEL_MAP_UPDATE_EVENT_DATA_LEN;

          // Point to the allocated data
          pPtr = pHciLeEvent->pData;

          // Copy the attributes
          memcpy(pPtr, &(pChannelMapUpdateEvt ->connHandle), sizeof(uint16_t));
          pPtr += sizeof(uint16_t);
          memcpy(pPtr, &(pChannelMapUpdateEvt ->nextDataChan), sizeof(uint8_t));
          pPtr++;
          memcpy(pPtr, &(pChannelMapUpdateEvt ->newChannelMap), sizeof(uint16_t) * LL_NUM_BYTES_FOR_CHAN_MAP);

          sendTheEvent = TRUE;
        }
        break;
      }

      case HCI_BLE_CONN_UPDATE_COMPLETE_EVENT:
      case HCI_BLE_CONN_UPDATE_REJECT_EVENT:
      {
        hciEvt_BLEConnUpdateComplete_t *pConnParamEvt  = (hciEvt_BLEConnUpdateComplete_t*) pMsgData;
        pHciLeEvent = ICall_malloc(sizeof(AppExtCtrlCommandStatusEvt_t) + EXTCTRL_CONN_PARAM_UPDATE_EVENT_DATA_LEN);

        if(pHciLeEvent != NULL)
        {
          pHciLeEvent->BLEEventCode = pConnParamEvt ->BLEEventCode;
          pHciLeEvent->len          = EXTCTRL_CONN_PARAM_UPDATE_EVENT_DATA_LEN;

          // Point to the allocated data
          pPtr = pHciLeEvent->pData;

          // Copy the attributes
          memcpy(pPtr, &(pConnParamEvt ->status), sizeof(uint8_t));
          pPtr++;
          memcpy(pPtr, &(pConnParamEvt ->connectionHandle), sizeof(uint16_t));
          pPtr += sizeof(uint16_t);
          memcpy(pPtr, &(pConnParamEvt ->connInterval), sizeof(uint16_t));
          pPtr += sizeof(uint16_t);
          memcpy(pPtr, &(pConnParamEvt ->connLatency), sizeof(uint16_t));
          pPtr += sizeof(uint16_t);
          memcpy(pPtr, &(pConnParamEvt ->connTimeout), sizeof(uint16_t));

          sendTheEvent = TRUE;
        }
        break;
      }

      default:
      {
          break;
      }
    }

    if (sendTheEvent == TRUE)
    {
      // The event was allocated and should be forwarded.
      pHciLeEvent->event = APP_EXTCTRL_COMMON_HCI_COMPLETE_EVENT;
      // Send the event forward
      CommonExtCtrl_extHostEvtHandler((uint8_t *)pHciLeEvent,
                               sizeof(AppExtCtrlCommandStatusEvt_t) + pHciLeEvent->len);
      // Free the resources
      ICall_free(pHciLeEvent);
    }
  }
}

/*********************************************************************
 * @fn      CommonExtCtrl_hciGapTypeEvtHandler
 *
 * @brief   This function handles events from type @ref BLEAPPUTIL_HCI_GAP_TYPE
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void CommonExtCtrl_hciGapTypeEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if ( pMsgData != NULL )
  {
    switch (event)
    {
      case BLEAPPUTIL_HCI_COMMAND_STATUS_EVENT_CODE:
      {
        CommonExtCtrl_cmdStatusEvtHandler((hciEvt_CommandStatus_t *)pMsgData);
        break;
      }

      case BLEAPPUTIL_HCI_LE_EVENT_CODE:
      {
        CommonExtCtrl_hciLeEvtHandler(pMsgData);
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
 * @fn      CommonExtCtrl_eventHandler
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
static void CommonExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{

  if (pMsgData != NULL)
  {
    switch (eventType)
    {
      case BLEAPPUTIL_HCI_GAP_TYPE:
      {
        CommonExtCtrl_hciGapTypeEvtHandler(event, pMsgData);
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
 * @fn      CommonExtCtrl_extHostEvtHandler
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
static void CommonExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if (gExtHostEvtHandler != NULL && pData != NULL)
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      CommonExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the common module
 *          to the external control dispatcher, and register the call back
 *          event handler function to the common application.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t CommonExtCtrl_start(void)
{
  bStatus_t status = SUCCESS;

  // Register to the Dispatcher module
  gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_COMMON,
                                                     &CommonExtCtrl_commandParser,
                                                     APP_CAP_COMMON);

  // If the registration succeed, register event handler call back to the common application
  if (gExtHostEvtHandler != NULL)
  {
    Common_registerEvtHandler(&CommonExtCtrl_eventHandler);
  }
  return status;
}
