/******************************************************************************

@file  app_extctrl_cm.h

@brief This file parse and process the messages comes form the external control module,
 and build the events from the app_cm.c application and send it
 to the external control module back.


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

#ifndef APP_EXTCTRL_CM_H
#define APP_EXTCTRL_CM_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_extctrl_common.h>
#include <app_central_api.h>

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
 * @brief Structure for incoming Connection Monitor Start Command parameters
 */
typedef struct __attribute__((packed))
{
  uint32_t timeDelta;          //!< Time delta in microseconds
  uint32_t timeDeltaErr;       //!< Maximum deviation time in microseconds
  uint32_t connTimeout;        //!< The supervision connection timeout
  uint8_t  maxSyncAttempts;    //!< Maximum attempts the device will try until the sync happens
  uint8_t  dataLen;            //!< Length of the connection data
  uint8_t  data[];             //!< Connection data
} AppExtCtrlCmStart_t;

/**
 * @brief Structure for incoming Connection Monitor Update Command parameters
 */
typedef struct __attribute__((packed))
{
  uint32_t accessAddr;         //!< Access address of the monitored connection
  uint16_t connUpdateEvtCnt;   //!< Connection update event counter
  uint8_t  updateType;         //!< Update type (PHY, Channel Map, or Parameter)
  uint8_t  dataLen;            //!< Length of the update data
  uint8_t  data[];             //!< Update data
} AppExtCtrlCmUpdate_t;

/**
 * @brief Structure for incoming Connection Monitor Stop Command parameters
 */
typedef struct __attribute__((packed))
{
  uint16_t connHandle;         //!< Connection handle of the monitored connection
} AppExtCtrlCmStop_t;

/*********************************************************************
 * Enumerators
 */

/*********************************************************************
 * Structures
 */

/**
 * @brief Structure for Connection Monitor Serving start event with
 *        the connection data
 */
typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;
  uint32_t accessAddr;
  uint16_t connHandle;
  uint8_t  dataLen;
  uint8_t  pData[];
} AppExtCtrlCmsStartEvent_t;

typedef struct __attribute__((packed))
{
  uint32_t timestamp;
  uint8_t  status;
  int8_t   rssi;
  uint8_t  pktLength;
  uint8_t  sn:1;
  uint8_t  nesn:1;
  uint8_t  pad:6;
} AppExtCtrlCmPacketData_t;

/**
 * @brief Structure for Connection Monitor packet report event
 */
typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;
  uint32_t accessAddr;
  uint16_t connHandle;
  uint16_t connEvtCnt;
  uint8_t  channel;
  AppExtCtrlCmPacketData_t packets[2];
} AppExtCtrlCmReportEvent_t;

/**
 * @brief Structure for Connection Monitor Connection update event
 */
typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;     //!< Event type
  uint16_t connHandle;         //!< Connection handle
  uint32_t accessAddr;         //!< Access address of the monitored connection
  uint16_t connUpdateEvtCnt;   //!< Connection update event counter
  uint8_t  updateType;         //!< Update type (PHY, Channel Map, or Parameter)
  uint8_t  dataLen;            //!< Length of the update data
  uint8_t  data[];             //!< Update data
} AppExtCtrlCmConnUpdateEvent_t;

/**
 * @brief Structure for Connection Monitor start event sent to the
 *        external control
 */
typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;     //!< Event type
  uint32_t accessAddr;         //!< Access address of the monitored connection
  uint16_t connHandle;         //!< Connection handle
  uint8_t  addrType;           //!< Address type of the monitored device
  uint8_t  addr[B_ADDR_LEN];   //!< Address of the monitored device
} AppExtCtrlCmStartEvent_t;

/**
 * @brief Structure for Connection Monitor stop event sent to the
 *        external control
 */
typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;     //!< Event type
  uint32_t accessAddr;         //!< Access address of the monitored connection
  uint16_t connHandle;         //!< Connection handle
  uint8_t  addrType;           //!< Address type of the monitored device
  uint8_t  addr[B_ADDR_LEN];   //!< Address of the monitored device
  uint8_t  stopReason;         //!< Reason for stopping the monitoring
} AppExtCtrlCmStopEvent_t;

/*********************************************************************
 * FUNCTIONS
 */
/*********************************************************************
 * @fn      ConnectionMonitorExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the connection
 *          monitor module to the external control.
 *
 * @param   None
 *
 * @return  SUCCESS, FAILURE
 */
bStatus_t ConnectionMonitorExtCtrl_start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_EXTCTRL_CM_H */
