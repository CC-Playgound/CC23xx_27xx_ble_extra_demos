/******************************************************************************

@file  app_gatt.c

@brief This file contains the application GATT functionality

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

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "app_gatt_api.h"
#include <app_main.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Prototypes
//*****************************************************************************

static void AppGATT_GATTEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void AppGATT_extEvtHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);

//*****************************************************************************
//! Globals
//*****************************************************************************

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref AppGATT_registerEvtHandler
//*****************************************************************//
static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

// Events handlers struct, contains the handlers and event masks
// of the application data module
BLEAppUtil_EventHandler_t appGATTGATTHandler =
{
    .handlerType    = BLEAPPUTIL_GATT_TYPE,
    .pEventHandler  = AppGATT_GATTEventHandler,
    .eventMask      = BLEAPPUTIL_ATT_MTU_UPDATED_EVENT
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      AppGATT_GATTEventHandler
 *
 * @brief   The purpose of this function is to handle GATT events
 *          that rise from the GATT and were registered in
 *          @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void AppGATT_GATTEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer
  AppGATT_extEvtHandler(BLEAPPUTIL_GATT_TYPE, event, pMsgData);
}

/*********************************************************************
 * @fn      AppGATT_extEvtHandler
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
static void AppGATT_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer if its handle exists
  if (gExtEvtHandler != NULL)
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      AppGATT_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void AppGATT_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
  gExtEvtHandler = fEventHandler;
}

/*********************************************************************
 * @fn      AppGATT_exchangeMTU
 *
 * @brief   This function is called to exchange the MTU size with the peer
 *
 * @param   connHandle - connection handle.
 * @param   mtuSize - new MTU.
 *
 * @return  @ref SUCCESS : The command was sent successfully.
 * @return  @ref FAILURE : The command wasn't sent.
 */
uint8_t AppGATT_exchangeMTU(uint16_t connHandle, uint16_t mtuSize)
{
  uint8_t status = SUCCESS;
  attExchangeMTUReq_t req;

  req.clientRxMTU = mtuSize - L2CAP_HDR_SIZE;
  status = GATT_ExchangeMTU(connHandle, &req, BLEAppUtil_getSelfEntity());

  return status;
}

/*********************************************************************
 * @fn      AppGATT_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the data
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t AppGATT_start( void )
{
  bStatus_t status = SUCCESS;

  // Register the handlers
  status = BLEAppUtil_registerEventHandler( &appGATTGATTHandler );

  // Return status value
  return( status );
}
