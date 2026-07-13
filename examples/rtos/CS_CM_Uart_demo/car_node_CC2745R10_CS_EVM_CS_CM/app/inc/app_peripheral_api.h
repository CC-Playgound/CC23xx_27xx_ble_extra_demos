/******************************************************************************

@file  app_peripheral_api.h

@brief This file contains the Peripheral APIs and structures.


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

#ifndef APP_PERIPHERAL_API_H
#define APP_PERIPHERAL_API_H

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
 * @brief Advertise Start Command Parmas Structure
 *
 * Should be created and passed to the
 * @ref Peripheral_advStart function.
 */
typedef struct
{
  uint16_t                durationOrMaxEvents; //!< Duration or maximum number of advertising events
  GapAdv_enableOptions_t  enableOptions;       //!< Advertising enable options
  uint8_t                 advHandle;           //!< Advertising handle
} Peripheral_advStartCmdParams_t;

/*********************************************************************
 * Functions
 */

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
bStatus_t Peripheral_advStart(Peripheral_advStartCmdParams_t *advParams);

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
bStatus_t Peripheral_advStop(uint8_t *pHandle);

/*********************************************************************
 * @fn      Peripheral_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void Peripheral_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler);

#ifdef __cplusplus
}
#endif

#endif /* APP_PERIPHERAL_API_H */
