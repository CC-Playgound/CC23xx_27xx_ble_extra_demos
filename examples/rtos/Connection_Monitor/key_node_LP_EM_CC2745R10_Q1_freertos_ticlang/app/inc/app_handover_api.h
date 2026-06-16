/******************************************************************************

@file  app_handover_api.h

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2026, Texas Instruments Incorporated
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

#ifndef APP_HANDOVER_API_H
#define APP_HANDOVER_API_H

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_extctrl_common.h>

//*****************************************************************************
//! Defines
//*****************************************************************************
#define INVALID_32BIT_STATUS 0xFFFFFFFF
#define INVALID_CONN_HANDLE  0xFFFF

/*
 * Serving node   - will insert a Type header at the beginning of each data block.
 * Candidate node - would be able to parse the data correctly, without additional
 *                  arguments in StartCN. Due to type header it knows which data
 *                  belongs to which module (Stack/RREQ).
 */
#define TYPE_RREQ_HEADER              0x01
#define TYPE_STACK_HEADER             0x02
#define TYPE_TIME_SYNC_HEADER         0x03

// Size of the type header - uint32
#define TYPE_HEADER_SIZE          4
//*****************************************************************************
//! Typedefs
//*****************************************************************************
/**
 * @brief Handover serving node parameters structure
 *
 * Should be created and passed to Handover_startServingNode
 */
typedef struct
{
  uint16_t connHandle;          //!< Connection handle
  uint16_t minGattHandle;       //!< Minimum GATT handle
  uint16_t maxGattHandle;       //!< Maximum GATT handle
  uint8_t  snMode;              //!< Serving node mode
} Handover_snParams_t;

/**
 * @brief Handover serving node close SN parameters structure
 *
 * Should be created and passed to Handover_closeServingNode
 */
typedef struct
{
  uint16_t connHandle;          //!< Connection handle
  uint32_t handoverStatus;      //!< Handover status
} Handover_closeSnParams_t;

/**
 * @brief Handover candidate node parameters structure
 *
 * Should be created and passed to Handover_startCandidateNode
 */
typedef struct
{
  uint8_t  *pHandoverData;      //!< Pointer to handover data
  uint32_t  timeDelta;          //!< Time delta
  uint32_t  timeDeltaErr;       //!< Time delta error
  uint32_t  maxFailedConnEvents;//!< Maximum number of failed connection events
  uint8_t   txBurstRatio;       //!< Central Only: Percentage of the connection event the TxBurst will be active. when this value is 0, TxBurst won't be used
} Handover_cnParams_t;

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Handover_startServingNode
 *
 * @brief   This function will handle the start of the handover process
 *          on the serving node side. This includes, getting the needed
 *          buffer size from the stack and registering to module's CB
 *
 * @param   snParams - Serving node start parameters
 *
 * @return  SUCCESS, FAILURE, bleMemAllocError, bleIncorrectMode
 *          INVALIDPARAMETER
 */
bStatus_t Handover_startServingNode(Handover_snParams_t snParams);

/*********************************************************************
 * @fn      Handover_startCandidateNode
 *
 * @brief   This function will handle the start of the handover process
 *          on the candidate node side. This includes registering to the
 *          module's CB
 *
 * @param   cnParams - Candidate node start parameters
 *
 * @return  none
 */
bStatus_t Handover_startCandidateNode(Handover_cnParams_t cnParams);

/*********************************************************************
 * @fn      Handover_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void Handover_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler);

/*********************************************************************
 * @fn      Handover_closeServingNode
 *
 * @brief   This function handle the serving node close command
 *
 * @return  status
 */
bStatus_t Handover_closeServingNode(Handover_closeSnParams_t closeSn);

#endif // APP_HANDOVER_API_H
