/******************************************************************************

@file  app_central_api.h

@brief This file contains the Central APIs and structures.


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

#ifndef APP_CENTRAL_API_H
#define APP_CENTRAL_API_H

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
 * @brief Scan Start Command Params Structure
 *
 * Should be created and passed to
 * @ref Central_scanStart function.
 */
typedef struct
{
  uint16_t scanPeriod;         //!< Scan period in 1.28 sec units. 0x00 for continuous.
  uint16_t scanDuration;       //!< Scan duration in 10 ms units. 0x00 for continuous.
  uint8_t  maxNumReport;       //!< Max number of reports. 0 for unlimited.
} Central_scanStartCmdParams_t;

/**
 * @brief Create Connection Command Params Structure
 *
 * Should be created and passed to
 * @ref Central_connect function.
 */
typedef struct
{
    uint8_t addrType;           //!< Advertiser address type
    uint8_t addr[B_ADDR_LEN];   //!< Advertiser address
    uint8_t phy;                //!< PHY
    uint16_t timeout;           //!< Timeout
} Central_connectCmdParams_t;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      Central_scanStart
 *
 * @brief   Central Scan Start Api, This Api starts the scan with the given params.
 *          If duration is zero period shall be ignored and the scanner
 *          will continue scanning until @ref BLEAppUtil_scanStop is called.
 *          If both period is zero and duration is non-zero, the scanner will
 *          scan once until duration has expired or @ref BLEAppUtil_scanStop
 *          is called. If both the duration and period are non-zero, the
 *          scanner will continue canning periodically until @ref BLEAppUtil_scanStop
 *          is called.
 *
 * @param   pScanParams - pointer to the start scan command params.
 *
 * @return  @ref SUCCESS
 * @return  @ref bleNotReady
 * @return  @ref bleInvalidRange
 * @return  @ref bleMemAllocError
 * @return  @ref bleAlreadyInRequestedMode
 * @return  @ref bleIncorrectMode
 */
bStatus_t Central_scanStart(Central_scanStartCmdParams_t *pScanParams);

/*********************************************************************
 * @fn      Central_connect
 *
 * @brief   This function initiates a connection with the peer device specified by the
 *          connection parameters provided. It sets up the connection parameters and
 *          calls the @ref BLEAppUtil_connect function to establish the connection.
 *
 * @param pConnParams Pointer to the connection command parameters.
 *
 * @return @ref SUCCESS
 * @return @ref bleNotReady
 * @return @ref bleInvalidRange
 * @return @ref bleMemAllocError
 * @return @ref bleAlreadyInRequestedMode
 * @return @ref FAILURE
 */
bStatus_t Central_connect(Central_connectCmdParams_t *pConnParams);

/*********************************************************************
 * @fn      Central_scanStop
 *
 * @brief   Central Scan Stop Api, This Api stops currently running scanning operation.
 *
 * @return @ref SUCCESS
 * @return @ref FAILURE
 * @return @ref bleIncorrectMode
 */
bStatus_t Central_scanStop(void);

/*********************************************************************
 * @fn      Central_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void Central_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler);

#ifdef __cplusplus
}
#endif

#endif /* APP_CENTRAL_API_H */
