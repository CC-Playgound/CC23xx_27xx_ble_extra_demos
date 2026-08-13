/******************************************************************************

@file  app_ca_server.c

@brief This file contains the Car Access service server application
       functionality

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
#include "ti/ble/profiles/car_access/car_access_profile.h"
#include <app_ca_server_api.h>
#include <app_main.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref CAServer_registerEvtHandler
//*****************************************************************//
static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

//*****************************************************************************
//!LOCAL FUNCTIONS
//*****************************************************************************

static void CAServer_onCCCUpdateCB( uint16_t connHandle, uint16_t pValue );
static void CAServer_incomingDataCB( uint16_t connHandle, uint8_t *pValue, uint16_t len );
static void CAServer_indCnfReceivedCB( uint16_t connHandle, uint8_t status );
static void CAServer_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//!APPLICATION CALLBACK
//*****************************************************************************
// Car Access application callback function for incoming data
static CAP_cb_t ca_profileCB =
{
  CAServer_onCCCUpdateCB,
  CAServer_incomingDataCB,
  CAServer_indCnfReceivedCB
};

//*****************************************************************************
//! Functions
//*****************************************************************************
/*********************************************************************
 * @fn      CAServer_onCCCUpdateCB
 *
 * @brief   Callback from Car Access Profile indicating ccc update
 *
 * @param   connHandle - the connection the CCC was updated for
 * @param   pValue - the new CCC value
 *
 * @return  SUCCESS or stack call status
 */
static void CAServer_onCCCUpdateCB( uint16_t connHandle, uint16_t pValue )
{
  // GATT_CLIENT_CFG_INDICATE
  CAServer_CCCpdateEvent_t cccUpdate;
  cccUpdate.connHandle = connHandle;
  cccUpdate.pValue = pValue;
  // Send the event to the upper layer
  CAServer_extEvtHandler(BLEAPPUTIL_GENERIC_TYPE, APP_EXTCTRL_CA_SERVER_CCC_UPDATE, (BLEAppUtil_msgHdr_t *)&cccUpdate);
}

/*********************************************************************
 * @fn      CAServer_incomingDataCB
 *
 * @brief   Callback from Car Access Profile indicating incoming data
 *
 * @param   dataIn - pointer to data structure used to store incoming data
 *
 * @return  SUCCESS or stack call status
 */
static void CAServer_incomingDataCB( uint16_t connHandle, uint8_t *pValue, uint16_t len )
{
  CAServer_incomingDataEvent_t incData;
  incData.connHandle = connHandle;
  incData.len = len;
  incData.pValue = pValue;
  // Send the event to the upper layer
  CAServer_extEvtHandler(BLEAPPUTIL_GENERIC_TYPE, APP_EXTCTRL_CA_SERVER_LONG_WRITE_DONE, (BLEAppUtil_msgHdr_t *)&incData);
}

/*********************************************************************
 * @fn      CAServer_indCnfReceivedCB
 *
 * @brief   Callback from Car Access Profile indicating indication
 *          confirmation was received
 *
 * @param   connHandle - the connection the indication was sent to
 * @param   status - the status of the indication
 *
 * @return  SUCCESS or stack call status
 */
static void CAServer_indCnfReceivedCB( uint16_t connHandle, uint8_t status )
{
  CAServer_indCnfEvent_t indCnf;
  indCnf.connHandle = connHandle;
  indCnf.status = status;
  // Send the event to the upper layer
  CAServer_extEvtHandler(BLEAPPUTIL_GENERIC_TYPE, APP_EXTCTRL_CA_SERVER_INDICATION_CNF, (BLEAppUtil_msgHdr_t *)&indCnf);
}

/*********************************************************************
 * @fn      CAServer_extEvtHandler
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
static void CAServer_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer if its handle exists
  if (gExtEvtHandler != NULL)
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      CAServer_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void CAServer_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
  gExtEvtHandler = fEventHandler;
}

/*********************************************************************
 * @fn      CAServer_sendIndication
 *
 * @brief   Send indication to a specific connHandle
 *
 * @param   connHandle - the connection handle to send indication to
 *
 * @return  SUCCESS or stack call status
 */
uint8_t CAServer_sendIndication( uint16_t connHandle )
{
  return ( CAP_sendInd( connHandle ) );
}

/*********************************************************************
 * @fn      CAServer_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Car Access profile.
 *
 * @return  SUCCESS or stack call status
 */
uint8_t CAServer_start( void )
{
  uint8_t status = SUCCESS;

  status = CAP_start( &ca_profileCB );

  return ( status );
}
