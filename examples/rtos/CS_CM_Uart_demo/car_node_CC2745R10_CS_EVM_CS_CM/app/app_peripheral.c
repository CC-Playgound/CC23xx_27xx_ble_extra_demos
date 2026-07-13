/******************************************************************************

@file  app_peripheral.c

@brief This example file demonstrates how to activate the broadcaster role as peripheral with
the help of BLEAppUtil APIs.

peripheralAdvHandler structure is used for define advertising event handling
callback function and eventMask is used to specify the events that
will be received and handled.
In addition, fill the BLEAppUtil_AdvInit_t structure with variables generated
by the Sysconfig.

In the Peripheral_start() function at the bottom of the file, registration,
initialization and activation are done using the BLEAppUtil API functions,
using the structures defined in the file.

The example provides also APIs to be called from upper layer and support sending events
to an external control module.

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
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "app_peripheral_api.h"
#include "ti_ble_config.h"
#include "app_main.h"

//*****************************************************************************
//! Prototypes
//*****************************************************************************
static void Peripheral_AdvEventHandler(uint32, BLEAppUtil_msgHdr_t*);
static void Peripheral_extEvtHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);

//*****************************************************************************
//! Globals
//*****************************************************************************

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref Central_registerEvtHandler
//*****************************************************************//
static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

//! Stores adv handles
uint8_t peripheralAdvHandles[BLE_CONFIG_NUM_ADV_SETS];

BLEAppUtil_EventHandler_t peripheralAdvHandler =
{
  .handlerType    = BLEAPPUTIL_GAP_ADV_TYPE,
  .pEventHandler  = Peripheral_AdvEventHandler,
  .eventMask      = BLEAPPUTIL_ADV_START_AFTER_ENABLE |
                    BLEAPPUTIL_ADV_END_AFTER_DISABLE
};

BLEAppUtil_AdvStart_t advSetStartParamsSet =
{
  /* Use the maximum possible value. This is the spec-defined maximum for */
  /* directed advertising and infinite advertising for all other types */
  .enableOptions         = GAP_ADV_ENABLE_OPTIONS_USE_MAX,
  .durationOrMaxEvents   = 0
};

//*****************************************************************************
//! Functions
//*****************************************************************************

// The application APIs

/******************************************************************************
 * @fn      Peripheral_advStart
 *
 * @brief   Peripheral Advertise Start Api, This Api will attempt to enable
 *          advertising for a set identified by the handle.
 *
 * @param   pAdvParams - pointer to cmd struct params.
 *
 * @return @ref bleIncorrectMode : incorrect profile role or an update / prepare
 *         is in process
 * @return @ref bleGAPNotFound : advertising set does not exist
 * @return @ref bleNotReady : the advertising set has not yet been loaded with
 *         advertising data
 * @return @ref bleAlreadyInRequestedMode : device is already advertising
 * @return @ref SUCCESS
 */
bStatus_t Peripheral_advStart(Peripheral_advStartCmdParams_t *pAdvParams)
{
  bStatus_t status = FAILURE;

  if (pAdvParams != NULL)
  {
    advSetStartParamsSet.durationOrMaxEvents = pAdvParams->durationOrMaxEvents;
    advSetStartParamsSet.enableOptions = pAdvParams->enableOptions;
    status = BLEAppUtil_advStart(pAdvParams->advHandle, &advSetStartParamsSet);
  }

  return status;
}

/*********************************************************************
 * @fn      Peripheral_advStop
 *
 * @brief   Peripheral Advertise Stop Api, This Api will attempt to stop
 *          the advertise set by handle.
 *
 * @param   pHandle - pointer to the advSet handle.
 *
* @return @ref bleIncorrectMode : incorrect profile role or an update / prepare
*         is in process
* @return @ref bleGAPNotFound : advertising set does not exist
* @return @ref bleAlreadyInRequestedMode : advertising set is not currently
*         advertising
* @return @ref SUCCESS
*/
bStatus_t Peripheral_advStop(uint8_t *pHandle)
{
  bStatus_t status = FAILURE;

  if (pHandle != NULL)
  {
    status = BLEAppUtil_advStop(*pHandle);
  }

  return status;
}

/*********************************************************************
 * @fn      Peripheral_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void Peripheral_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
  gExtEvtHandler = fEventHandler;
}

// The application functions

/*********************************************************************
 * @fn      Peripheral_extEvtHandler
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
static void Peripheral_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer if its handle exists
  if (gExtEvtHandler != NULL)
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      Peripheral_AdvEventHandler
 *
 * @brief   The purpose of this function is to handle peripheral state
 *          events that rise from the GAP and were registered
 *          in @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void Peripheral_AdvEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    // Send the event to the upper layer
    Peripheral_extEvtHandler(BLEAPPUTIL_GAP_ADV_TYPE, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      Peripheral_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the peripheral
 *          application module.
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Peripheral_start()
{
  bStatus_t status = SUCCESS;

  status = BLEAppUtil_registerEventHandler(&peripheralAdvHandler);
  if (status != SUCCESS)
  {
    return(status);
  }

  // Initiate the advertise sets
  uint8_t peripheralAdvHandlesStatuses[BLE_CONFIG_NUM_ADV_SETS];
  BleConfig_initAdvSets(peripheralAdvHandles, peripheralAdvHandlesStatuses);
  for (int i = 0; i < BLE_CONFIG_NUM_ADV_SETS; i++)
  {
    // Return the first FAILURE status value if any
    if (peripheralAdvHandlesStatuses[i] != SUCCESS)
    {
      return(peripheralAdvHandlesStatuses[i]);
    }
  }

  // Return status value
  return(status);
}
#endif  // ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
