/******************************************************************************

@file  app_common.c

@brief This example file is for common app util

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
#include "app_common_api.h"

//*****************************************************************************
//! Defines
//*****************************************************************************


//*****************************************************************************
//! Prototypes
//*****************************************************************************
static void Common_HciGapEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Globals
//*****************************************************************************

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref Common_registerEvtHandler
//*****************************************************************//
static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

BLEAppUtil_EventHandler_t commonHciGapHandler =
{
    .handlerType    = BLEAPPUTIL_HCI_GAP_TYPE,
    .pEventHandler  = Common_HciGapEventHandler,
    .eventMask      = BLEAPPUTIL_HCI_COMMAND_STATUS_EVENT_CODE |
                      BLEAPPUTIL_HCI_LE_EVENT_CODE
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Common_getDevAddress
 *
 * @brief   Extract the device address.
 *
 * @param  pAddrMode - pointer to the address mode
 * @param  pIdAddr   - pointer to the IDA
 * @param  pRpAddr   - pointer to the RPA
 *
 * @return @ref SUCCESS
 */
bStatus_t Common_getDevAddress(GAP_Addr_Modes_t *pAddrMode, uint8_t *pIdAddr, uint8_t *pRpAddr)
{
  bStatus_t status = SUCCESS;

  uint8_t *pGapIdAddr =  GAP_GetDevAddress(TRUE);
  uint8_t *pGapRpAddr =  GAP_GetDevAddress(FALSE);

  *pAddrMode = BLEAppUtil_getDevAddrMode();

  memcpy(pIdAddr, pGapIdAddr, B_ADDR_LEN);
  if (*pAddrMode > ADDRMODE_RANDOM)
  {
     memcpy(pRpAddr, pGapRpAddr, B_ADDR_LEN);
  }

  return status;
}

/*********************************************************************
 * @fn      Common_resetDevice
 *
 * @brief   Resets device
 *
 * @param   none
 *
 * @return  none
 */

void Common_resetDevice(void)
{
  Power_reset();
}

/*********************************************************************
 * @fn      Common_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void Common_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
  gExtEvtHandler = fEventHandler;
}

/*********************************************************************
 * @fn      Common_extEvtHandler
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
void Common_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer if its handle exists
  if (gExtEvtHandler != NULL)
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      Common_HciGapEventHandler
 *
 * @brief   The purpose of this function is to handle HCI GAP events
 *          that rise from the HCI and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Common_HciGapEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    // Send the event to the upper layer
    Common_extEvtHandler(BLEAPPUTIL_HCI_GAP_TYPE, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      Common_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the connection
 *          application module and initiate the parser module.
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Common_start()
{
  bStatus_t status = SUCCESS;

  status = BLEAppUtil_registerEventHandler(&commonHciGapHandler);
  if (status != SUCCESS)
  {
    return(status);
  }

  // Return status value
  return(status);
}
