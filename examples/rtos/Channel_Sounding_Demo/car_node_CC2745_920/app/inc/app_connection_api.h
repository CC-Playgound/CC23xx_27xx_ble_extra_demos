/******************************************************************************

@file  app_connection_api.h

@brief This file contains the Connection APIs and structures.


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

#ifndef APP_CONNECTION_API_H
#define APP_CONNECTION_API_H

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

/**
 * @brief Event Call Back Enable Command Parmas Structure
 *
 * Should be created and passed to
 * @ref Connection_registerConnEventCmd function.
 */
typedef struct
{
  uint16_t       connHandle;      //!< Connection handle
  GAP_CB_Event_e eventType;       //!< Type of event, @ref GAP_CB_Event_e
  uint8_t        reportFrequency; //!< Frequency of event reporting
} Connection_registerConnEventCmdParams_t;

/**
 * @brief Set Phy Command Parmas Structure
 *
 * Should be created and passed to
 * @ref Connection_setPhyCmd function.
 */
typedef struct
{
  uint16_t connHandle; //!< Connection handle
  uint16_t phyOpts;    //!< Bit field of Host preferred PHY options
  uint8_t  allPhys;    //!< Host preference on how to handle txPhy and rxPhy
  uint8_t  txPhy;      //!< Bit field of Host preferred Tx PHY
  uint8_t  rxPhy;      //!< Bit field of Host preferred Rx PHY
} Connection_setPhyCmdParams_t;

/**
 * @brief Connection Param Update Command Parmas Structure
 *
 * Should be created and passed to
 * @ref Connection_paramUpdateCmd function.
 */
typedef struct
{
  uint16_t connectionHandle; //!< Connection handle of the update
  uint16_t intervalMin;      //!< Minimum Connection Interval
  uint16_t intervalMax;      //!< Maximum Connection Interval
  uint16_t connLatency;      //!< Connection Latency
  uint16_t connTimeout;      //!< Connection Timeout
  uint8_t  signalIdentifier; //!< L2CAP Signal Identifier. Must be 0 for LL Update
} Connection_paramUpdateCmdParams_t;

/**
 * @brief Connection Channel Map Update Command Parameters Structure
 *
 * Should be created and passed to
 * @ref Connection_channelMapUpdateCmd function.
 */
typedef struct
{
  uint16_t connectionHandle; //!< Connection handle of the update
  uint8_t  channelMap[5];    //!< New channel map
} Connection_channelMapUpdateCmdParams_t;

/**
 * @brief  Set Data Length Command Parameters Structure
 *
 * Should be created and passed to
 * @ref Connection_dataLenUpdateCmd function.
 */
typedef struct
{
  uint16_t connectionHandle; //!< Connection handle of the update
  uint16_t txOctets;      //!< Maximum Transmission Octets
  uint16_t txTime;        //!< Maximum Transmission Time
} Connection_dataLenUpdateCmdParams_t;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      Connection_registerConnEventCmd
 *
 * @brief   Register connection event call back Api, This Api enables reporting the event
 *          call back of the connection handle in every connection event depends on
 *          the frequency rate that been asked for.
 *
 * @param   pParams - pointer to cmd struct params.
 *
* @return @ref SUCCESS
* @return @ref bleGAPNotFound : connection handle not found
* @return @ref bleInvalidRange : the callback function was NULL or action is invalid
* @return @ref bleMemAllocError : there is not enough memory to register the callback.
 */
bStatus_t Connection_registerConnEventCmd(Connection_registerConnEventCmdParams_t *pParams);

/*********************************************************************
 * @fn      Connection_unregisterConnEventCmd
 *
 * @brief   unregister connection event call back Api, This Api disables reporting the event
 *  for all the connections.
 *
 * @param   None
 *
* @return @ref SUCCESS
* @return @ref bleGAPNotFound : connection handle not found
* @return @ref bleInvalidRange : the callback function was NULL or action is invalid
* @return @ref bleMemAllocError : there is not enough memory to register the callback.
 */
bStatus_t Connection_unregisterConnEventCmd(void);

/*********************************************************************
 * @fn      Connection_unregisterConnEventCmd
 *
 * @brief   Set the phy connection.
 *
 * @param   pParams - pointer to cmd struct params.
 *
 * @return  @ref HCI_SUCCESS
 */
bStatus_t Connection_setPhyCmd(Connection_setPhyCmdParams_t *pParams);

/*********************************************************************
 * @fn      Connection_paramUpdateCmd
 *
 * @brief   Send connection parameters update.
 *
 * @param   pParams - pointer to cmd struct params.
 *
 * @return  @ref HCI_SUCCESS
 */
bStatus_t Connection_paramUpdateCmd(Connection_paramUpdateCmdParams_t *pParams);

/*********************************************************************
 * @fn      Connection_channelMapUpdateCmd
 *
 * @brief   Send channel map update.
 *
 * @param   pParams - pointer to cmd struct params.
 *
 * @return  @ref HCI_SUCCESS
 */
bStatus_t Connection_channelMapUpdateCmd(Connection_channelMapUpdateCmdParams_t *pParams);

/*********************************************************************
 * @fn      Connection_dataLenUpdateCmd
 *
 * @brief   Send data length update.
 *
 * @param   pParams - pointer to cmd struct params.
 *
 * @return  @ref HCI_SUCCESS
 */
bStatus_t Connection_dataLenUpdateCmd(Connection_dataLenUpdateCmdParams_t *pParams);

/*********************************************************************
 * @fn      Connection_terminateLinkCmd
 *
 * @brief   Terminate a BLE connection.
 *
 * @param   connHandle - Connection handle to terminate.
 *
 * @return @ref SUCCESS : termination request sent to stack
 * @return @ref bleIncorrectMode : No Link to terminate
 * @return @ref bleInvalidTaskID : not app that established link
 */
bStatus_t Connection_terminateLinkCmd(uint16_t connHandle);

/*********************************************************************
 * @fn      Connection_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void Connection_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONNECTION_API_H */
