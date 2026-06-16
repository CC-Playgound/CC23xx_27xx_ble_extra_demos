/******************************************************************************

@file  app_l2cap_coc.c

@brief This file contains the L2CAP COC flow control application
functionality.

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

#if defined (BLE_V41_FEATURES) && (BLE_V41_FEATURES & L2CAP_COC_CFG)

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti/ble/host/l2cap/l2cap.h"
#include "app_l2cap_coc_api.h"
#include "ti/ble/stack_util/icall/app/icall.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti_ble_config.h"
#include "app_main.h"
#include <string.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Prototypes
//*****************************************************************************

static void L2CAPCOC_dataHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void L2CAPCOC_signalHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Globals
//*****************************************************************************

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref L2CAPCOC_registerEvtHandler
//*****************************************************************//
static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

// Events handlers struct, contains the handlers and event masks
// of the L2CAP data packets
BLEAppUtil_EventHandler_t DSL2CAPdataHandler =
{
    .handlerType    = BLEAPPUTIL_L2CAP_DATA_TYPE,
    .pEventHandler  = L2CAPCOC_dataHandler,
    .eventMask      = 0,
};

// Events handlers struct, contains the handlers and event masks
// of the L2CAP signal packets
BLEAppUtil_EventHandler_t DSL2CAPsignalHandler =
{
    .handlerType    = BLEAPPUTIL_L2CAP_SIGNAL_TYPE,
    .pEventHandler  = L2CAPCOC_signalHandler,
    .eventMask      = BLEAPPUTIL_L2CAP_CHANNEL_ESTABLISHED_EVT       |
                      BLEAPPUTIL_L2CAP_CHANNEL_TERMINATED_EVT        |
                      BLEAPPUTIL_L2CAP_OUT_OF_CREDIT_EVT             |
                      BLEAPPUTIL_L2CAP_PEER_CREDIT_THRESHOLD_EVT     |
                      BLEAPPUTIL_L2CAP_NUM_CTRL_DATA_PKT_EVT
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
* @fn      L2CAPCOC_createPsm
*
* @brief   Register a Protocol/Service Multiplexer with L2CAP.
*
* @param   psm - the PSM number wants to register with.
*
* @return  SUCCESS: Registration was successful.
 *         INVALIDPARAMETER: Max number of channels is greater than total supported.
 *         bleInvalidRange: PSM value is out of range.
 *         bleInvalidMtuSize: MTU size is out of range.
 *         bleNoResources: Out of resources.
 *         bleAlreadyInRequestedMode: PSM already registered.
*/
bStatus_t L2CAPCOC_createPsm(uint16_t psm)
{
  uint8_t status = SUCCESS;
  l2capPsm_t psmInstance;

  l2capUserCfg_t l2capUserConfig;
  L2CAP_GetUserConfig(&l2capUserConfig);

  // Prepare the PSM parameters
  psmInstance.psm                 = psm;
  psmInstance.mtu                 = L2CAP_MAX_MTU;
  psmInstance.mps                 = L2CAP_MAX_MPS;
  psmInstance.initPeerCredits     = L2CAP_NOF_CREDITS;
  psmInstance.peerCreditThreshold = L2CAP_CREDITS_THRESHOLD;
  psmInstance.maxNumChannels      = l2capUserConfig.maxNumCoChannels;
  psmInstance.taskId              = ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, BLEAppUtil_getSelfEntity());
  psmInstance.pfnVerifySecCB      = NULL;

  // Register PSM with L2CAP task
  status = L2CAP_RegisterPsm(&psmInstance);

  return status;
}

/*********************************************************************
* @fn      L2CAPCOC_closePsm
*
* @brief   Deregister a Protocol/Service Multiplexer with L2CAP.
*
* @param   psm - the PSM number wants to deregister.
*
* @return  SUCCESS: Registration was successful.
*          INVALIDPARAMETER: PSM or task Id is invalid.
*          bleIncorrectMode: PSM is in use.
*/
bStatus_t L2CAPCOC_closePsm(uint16_t psm)
{
  bStatus_t status = SUCCESS;

  uint8_t taskId = ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, BLEAppUtil_getSelfEntity());

  status =  L2CAP_DeregisterPsm( taskId, psm);

  return status;
}

/*********************************************************************
* @fn      L2CAPCOC_connectReq
*
 * @brief   Send Connection Request to open a channel.
 *
 * @param   pParams- pointer to parameters @ref L2capCoc_connectReqCmdParams_t
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: PSM is not registered.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleIncorrectMode: PSM not registered.
 *          bleNotConnected: Connection is down.
 *          bleNoResources: No available resource.
 *          bleMemAllocError: Memory allocation error occurred.
 */
bStatus_t L2CAPCOC_connectReq(L2capCoc_connectReqCmdParams_t *pParams)
{
  bStatus_t status = SUCCESS;
  // Send the connection request to the peer
  status = L2CAP_ConnectReq(pParams->connHandle, pParams->psm, pParams->peerPsm);

  return status;
}

/*********************************************************************
 * @fn      L2CAPCOC_disconnectReq
 *
 * @brief   Send Disconnection Request.
 *
 * @param   CID - local CID to disconnect
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Channel id is invalid.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          bleNoResources: No available resource.
 *          bleMemAllocError: Memory allocation error occurred.
 */
bStatus_t L2CAPCOC_disconnectReq(uint16_t CID)
{
  bStatus_t status = SUCCESS;
  // Send the disconnection request to the peer
  status = L2CAP_DisconnectReq(CID);

  return status;
}

/*********************************************************************
 * @fn      L2CAPCOC_sendSDU
 *
 * @brief   Send data packet over an L2CAP connection oriented channel
 *          established over a physical connection.
 *
 * @param   pParams- pointer to parameters @ref L2capCoc_sendSduCmdParams_t
 *
 * @return  SUCCESS: Data was sent successfully.
 *          INVALIDPARAMETER: SDU payload is null.
 *          bleNotConnected: Connection or Channel is down.
 *          bleMemAllocError: Memory allocation error occurred.
 *          blePending: In the middle of another transmit.
 *          bleInvalidMtuSize: SDU size is larger than peer MTU.
 */
bStatus_t L2CAPCOC_sendSDU(L2capCoc_sendSduCmdParams_t *pParams)
{
  bStatus_t  status = SUCCESS;

  l2capPacket_t packet;

  packet.connHandle = pParams->connHandle;
  packet.CID = pParams->CID;
  packet.len = pParams->len;
  packet.pPayload = L2CAP_bm_alloc( pParams->len );

  memcpy(packet.pPayload, pParams->pPayload, packet.len);

  status = L2CAP_SendSDU( &packet );

  if (status != SUCCESS)
  {
    osal_bm_free(packet.pPayload);
  }


  return status;
}

/*********************************************************************
 * @fn      L2CAPCOC_FlowCtrlCredit
 *
 * @brief   Send Flow Control Credit.
 *
 * @param   pParams- pointer to parameters @ref L2capCoc_flowCtrlCredits_t
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Channel is not open.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          bleInvalidRange - Credits is out of range.
 *          bleMemAllocError: Memory allocation error occurred.
 */
bStatus_t L2CAPCOC_FlowCtrlCredit(L2capCoc_flowCtrlCredits_t *pParams)
{
  bStatus_t  status = SUCCESS;

  if( pParams!= NULL )
  {
      status = L2CAP_FlowCtrlCredit(pParams->connHandle,
                                    pParams->CID,
                                    pParams->peerCredits);
  }

  return status;
}

/*********************************************************************
 * @fn      L2CAPCOC_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void L2CAPCOC_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
  gExtEvtHandler = fEventHandler;
}

/*********************************************************************
 * @fn      L2CAPCOC_extEvtHandler
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
static void L2CAPCOC_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer if its handle exists
  if (gExtEvtHandler != NULL)
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      L2CAPCOC_signalHandler
 *
 * @brief   Handles the signals received on the L2CAP channel
 *
 * @param   event - event to handle
 * @param   pMsgData - request/response data to handle
 *
 * @return  none
 */
void L2CAPCOC_signalHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    // Send to the to the upper layer that the scan started
    L2CAPCOC_extEvtHandler(BLEAPPUTIL_L2CAP_SIGNAL_TYPE, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      L2CAPCOC_dataHandler
 *
 * @brief   Handles the data received on the L2CAP channel
 *
 * @param   event - event to handle
 * @param   pMsgData - data to handle
 *
 * @return  none
 */
void L2CAPCOC_dataHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{

  if (pMsgData != NULL)
  {
    // Send to the to the upper layer
    L2CAPCOC_extEvtHandler(BLEAPPUTIL_L2CAP_DATA_TYPE, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      L2CAPCOC_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the L2CAP COC profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t L2CAPCOC_start( void )
{
  bStatus_t status = SUCCESS;

  status = BLEAppUtil_registerEventHandler(&DSL2CAPdataHandler);
  if(status != SUCCESS)
  {
    // Return status value
    return(status);
  }

  status = BLEAppUtil_registerEventHandler(&DSL2CAPsignalHandler);
  if(status != SUCCESS)
  {
    // Return status value
    return(status);
  }

  return ( status );
}

#endif // (BLE_V41_FEATURES & L2CAP_COC_CFG)
