/******************************************************************************

@file  app_l2cap_coc_api.h

@brief This file contains the L2cap COC APIs and structures.


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

#ifndef APP_L2CAP_COC_API_H
#define APP_L2CAP_COC_API_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_extctrl_common.h>

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

/*********************************************************************
 * Enumerators
 */

/*********************************************************************
 * Structures
 */

typedef struct
{
  uint16_t connHandle; //!< Connection handle
  uint16_t psm;        //!< Protocol/Service Multiplexer
  uint16_t peerPsm;    //!< Peer Protocol/Service Multiplexer
} L2capCoc_connectReqCmdParams_t;

/// @brief L2CAP packet structure.
typedef struct
{
  uint16_t connHandle; //!< Connection handle
  uint16_t CID;        //!< Connection Identifier
  uint16_t len;        //!< Length of the payload
  uint8_t  pPayload[];  //!< Pointer to the payload
} L2capCoc_sendSduCmdParams_t;

typedef struct
{
  uint16_t connHandle; //!< Connection handle
  uint16_t CID;        //!< Connection Identifier
  uint16 peerCredits;  //!< The credits to send
} L2capCoc_flowCtrlCredits_t;

/*********************************************************************
 * FUNCTIONS
 */

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
bStatus_t L2CAPCOC_createPsm(uint16_t psm);

/*********************************************************************
* @fn      L2CAPCOC_closePsm
*
* @brief   Close a communication channel between peers
*
* @param   connHandle - connection handle
*
* @return  SUCCESS: Registration was successful.
*          INVALIDPARAMETER: PSM or task Id is invalid.
*          bleIncorrectMode: PSM is in use.
*/
bStatus_t L2CAPCOC_closePsm(uint16_t psm);

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
bStatus_t L2CAPCOC_connectReq(L2capCoc_connectReqCmdParams_t *pParams);

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
bStatus_t L2CAPCOC_disconnectReq(uint16_t CID);

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
bStatus_t L2CAPCOC_sendSDU(L2capCoc_sendSduCmdParams_t *pParams);

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
bStatus_t L2CAPCOC_FlowCtrlCredit(L2capCoc_flowCtrlCredits_t *pParams);

/*********************************************************************
 * @fn      L2CAPCOC_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void L2CAPCOC_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler);

#ifdef __cplusplus
}
#endif

#endif /* APP_L2CAP_COC_API_H */
