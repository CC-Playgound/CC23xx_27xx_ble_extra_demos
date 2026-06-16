/******************************************************************************

@file  app_cs_transceiver_api.h

@brief This file contains the CS APIs and structures.


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

#ifndef APP_CS_TRANSCEIVER_API_H
#define APP_CS_TRANSCEIVER_API_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_extctrl_common.h>
#include "ti/ble/host/cs/cs.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*
 * Command Structures
 */

/*
 * Event Structures
 */


/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      ChannelSoundingTransceiver_sendResults
 *
 * @brief   This function is called to send the results of the Channel Sounding.
 *
 * @param   pMsgData - pointer to the message data.
 * @param   dataLen - length of the message data.
 * @param   eventOpcode  - specifies the type of event or message structure that is received from pMsgData.
 *
 * @return  None
 */
bStatus_t ChannelSoundingTransceiver_sendResults(uint8_t *pMsgData, uint16_t dataLen, ChannelSounding_eventOpcodes_e eventOpcode);

/*********************************************************************
 * @fn      ChannelSoundingTransceiver_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @param   fEventHandler - pointer to the external event handler function.
 *
 * @return  None
 */
void ChannelSoundingTransceiver_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler);

/*********************************************************************
 * @fn      ChannelSoundingTransceiver_start
 *
 * @brief   This function is called to start the Channel Sounding Transceiver.
 *          register the call back event handler function to the l2cap_coc application.
 *          and creates a new PSM.
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t ChannelSoundingTransceiver_start( void );

#ifdef __cplusplus
}
#endif

#endif /* APP_CS_TRANSCEIVER_API_H */
