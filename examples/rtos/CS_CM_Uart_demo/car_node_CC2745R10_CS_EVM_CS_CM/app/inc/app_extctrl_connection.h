/******************************************************************************

@file  app_extctrl_connection.h

@brief This file parse and process the messages comes form the external control module
 dispatcher module, and build the events from the app_connection.c application and
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

#ifndef APP_EXTCTRL_CONNECTION_H
#define APP_EXTCTRL_CONNECTION_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_extctrl_common.h>
#include <app_connection_api.h>

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

/**
 * @brief Structure for AppExtCtrl Connection Termination Link Request parameters
 */
typedef struct __attribute__((packed))
{
  uint16_t connHandle;   //!< Connection Handle
} AppExtCtrlConnTermLinkReq_t;

/**
 * @brief Structure for AppExtCtrl Set Phy Command parameters
 */
typedef struct __attribute__((packed))
{
  uint16_t connHandle; //!< Connection handle
  uint16_t phyOpts;    //!< Bit field of Host preferred PHY options
  uint8_t  allPhys;    //!< Host preference on how to handle txPhy and rxPhy
  uint8_t  txPhy;      //!< Bit field of Host preferred Tx PHY
  uint8_t  rxPhy;      //!< Bit field of Host preferred Rx PHY
} AppExtCtrlSetPhyCmdParams_t;

/**
 * @brief Structure for AppExtCtrl Connection Param Update Command parameters
 */
typedef struct __attribute__((packed))
{
  uint16_t connHandle;       //!< Connection handle of the update
  uint16_t intervalMin;      //!< Minimum Connection Interval
  uint16_t intervalMax;      //!< Maximum Connection Interval
  uint16_t connLatency;      //!< Connection Latency
  uint16_t connTimeout;      //!< Connection Timeout
  uint8_t  signalIdentifier; //!< L2CAP Signal Identifier. Must be 0 for LL Update
} AppExtCtrlParamUpdateCmdParams_t;

/**
 * @brief Structure for AppExtCtrl Connection Param Update Command parameters
 */
typedef struct __attribute__((packed))
{
  uint16_t connHandle;       //!< Connection handle of the update
  uint8_t  channelMap[5];    //!< New channel map
} AppExtCtrlChannelMapUpdateCmdParams_t;

/**
 * @brief Structure for AppExtCtrl Connection Param Update Command parameters
 */
typedef struct __attribute__((packed))
{
  uint16_t connHandle;       //!< Connection handle of the update
  uint16_t maxTxOctets;      //!< Maximum Transmission Octets
  uint16_t maxTxTime;        //!< Maximum Transmission Time
} AppExtCtrlSetDataLenCmdParams_t;

/**
 * @brief Structure for AppExtCtrl Register Connection Event Command parameters
 */
typedef struct __attribute__((packed))
{
  uint16_t connHandle;      //!< Connection handle
  uint8_t  eventType;       //!< Type of event @ref GAP_CB_Event_e
  uint8_t  reportFrequency; //!< Frequency of event reporting
} AppExtCtrlRegisterConnEventCmdParams_t;

/*********************************************************************
 * Enumerators
 */

/*********************************************************************
 * Structures
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      ConnectionExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the connection module
 *          to the external control dispatcher, and register the call back
 *          event handler function to the connection application.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t ConnectionExtCtrl_start(void);


#ifdef __cplusplus
}
#endif

#endif /* APP_EXTCTRL_CONNECTION_H */
