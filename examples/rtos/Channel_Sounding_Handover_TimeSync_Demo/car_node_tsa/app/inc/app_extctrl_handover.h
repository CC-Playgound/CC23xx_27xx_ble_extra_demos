/******************************************************************************

@file  app_handover_parser.h

@brief This file parse and process the messages comes form the NWP module,
 and build the events from the app_handover.c application and send it
 to the NWP module back.

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

#ifndef APP_EXTCTRL_HANDOVER_H
#define APP_EXTCTRL_HANDOVER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_handover_api.h>

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
// Handover Commands
typedef struct __attribute__((packed))
{
  uint16_t connHandle;
  uint16_t minGattHandle;
  uint16_t maxGattHandle;
  uint8_t  snMode;
} AppExtCtrlHandoverStartSnCmd_t;

typedef struct __attribute__((packed))
{
  uint16_t connHandle;
  uint32_t status;
} AppExtCtrlHandoverCloseSnCmd_t;

typedef struct __attribute__((packed))
{
  uint32_t timeDelta;
  uint32_t timeDeltaErr;
  uint32_t maxNumFailedEvents;
  uint32_t txBurstRatio;
  uint8_t  pData[];
} AppExtCtrlHandoverStartCnCmd_t;

// Handover Events
typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;
  uint16_t        connHandle;
  uint32_t        status;
  uint32_t        dataLen;
  uint8_t         pData[];
} AppExtCtrlHandoverSnStartEvent_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;
  uint16_t        connHandle;
  uint32_t        status;
} AppExtCtrlHandoverCnStartEvent_t;

/*********************************************************************
 * Enumerators
 */

/*********************************************************************
 * Structures
 */

/*********************************************************************
 * @fn      HandoverExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the handover module
 *          to the external control.
 *
 * @param   None
 *
 * @return  SUCCESS, FAILURE
 */
bStatus_t HandoverExtCtrl_start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_EXTCTRL_HANDOVER_H */
