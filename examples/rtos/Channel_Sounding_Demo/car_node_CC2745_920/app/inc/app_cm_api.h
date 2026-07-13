/******************************************************************************

@file  app_cm_api.h

@brief This file contains the Connection Monitor APIs and structures.


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

#ifndef APP_CM_API_H
#define APP_CM_API_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti/ble/stack_util/connection_monitor_types.h"
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
 * @brief Connection Monitor start serving command structure
 *
 * Should be created and passed to
 * @ref ConnectionMonitor_GetConnData function.
 */
typedef struct
{
    uint32_t accessAddr;    // The Access Address will be filled once @ref ConnnectionMonitor_GetConData called
    uint16_t connHandle;    // The connection should be assigned before calling @ref ConnectionMonitor_GetConnData
    uint8_t  dataLen;       // The size of the data buffer will be filled once @ref ConnectionMonitor_GetConnData called
    uint8_t *pData;         // The data buffer will be filled once @ref ConnectionMonitor_GetConnData called
} ConnectionMonitor_ServingStartCmdParams_t;

/**
 * @brief Connection Monitor - start monitoring command structure
 *
 * Should be created and passed to
 * @ref ConnectionMonitor_StartMonitor function.
 */
typedef struct
{
    uint32_t timeDeltaInUs;       //!< The time in us it took for the data to be transferred from node to another in the system
    uint32_t timeDeltaMaxErrInUs; //!< The maximum deviation time in us
    uint32_t connTimeout;         //!< The supervision connection timeout
    uint8_t  maxSyncAttempts;     //!< Number of attempts the device will try to follow before determining the monitoring
                                  //!< process failed or not.
    uint8_t  cmDataSize;          //!< The stack CM data size
    uint8_t  *pCmData;            //!< Pointer to the buffer the application allocated
} ConnectionMonitor_StartCmdParams_t;

/**
 * @brief Connection Monitor - stop monitoring command structure
 *
 * Should be created and passed to
 * @ref ConnectionMonitor_StopMonitor function.
 */
typedef struct
{
    uint32_t connHandle;          //!< Connection handle of the connection monitor to stop
} ConnectionMonitor_StopCmdParams_t;

/**
 * @brief Connection Monitor - update monitoring command structure
 *
 * Should be created and passed to
 * @ref ConnectionMonitor_UpdateConn function.
 */
typedef struct
{
    uint32_t              accessAddr;            //!< Access address of the monitored connection
    uint16_t              connUpdateEvtCnt;      //!< The connection Event counter of the monitored connection
    cmConnUpdateEvtType_e updateType;            //!< The event type @ref cmConnUpdateEvtType_e
    union
    {
        cmPhyUpdateEvt_t     phyUpdateEvt;     //!< Phy update event
        cmChanMapUpdateEvt_t chanMapUpdateEvt; //!< Channel map update event
        cmParamUpdateEvt_t   paramUpdateEvt;   //!< Parameter update event
    } updateEvt;
} ConnectionMonitor_UpdateCmdParams_t;

/*********************************************************************
 * FUNCTIONS
 */
/*********************************************************************
 * @fn      ConnectionMonitor_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *
 * @return  fEventHandler - Pointer to the event handler function
 */
void ConnectionMonitor_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler);

/*********************************************************************
 * @fn      ConnectionMonitor_GetConnData
 *
 * @brief   This function get the needed information from the connection
 *          monitor - serving side. This information should be passed by
 *          the Serving node to the Candidate node. The Candidate Node
 *          will use this information to monitor the requested connection
 *
 * @param   pCmsParams - Pointer for the CMS parameters
 *
 * @return  CM_SUCCESS, CM_INCORRECT_MODE, CM_NOT_CONNECTED,
 *          CM_NO_RESOURCE, CM_UNSUPPORTED @ref cmErrorCodes_e
 */
cmErrorCodes_e ConnectionMonitor_GetConnData(ConnectionMonitor_ServingStartCmdParams_t *pCmsParams);

/*********************************************************************
 * @fn      ConnectionMonitor_StartMonitor
 *
 * @brief   This function used to start monitoring a connection by the
 *          candidate based on the given connection data
 *
 * @param   pNewCmData - Pointer to a new connection data needed
 *                       to be monitored
 *
 * @return  CM_SUCCESS, CM_NOT_CONNECTED, CM_ALREADY_REQUESTED,
 *          CM_INVALID_PARAMS, CM_NO_RESOURCE,
 *          CM_CONNECTION_LIMIT_EXCEEDED, CM_UNSUPPORTED @ref cmErrorCodes_e
 *
 * @output  This function will send a @ref CM_TRACKING_START_EVT event to the registered
 *          callback @ref pfnCmConnStatusEvtCB in case of a success monitoring start, otherwise it will send a
 *          @ref CM_TRACKING_STOP_EVT event after the end of the trying to sync up to
 *          maxSyncAttempts in the @ref cmStartMonitorParams_t.
 *
 * @note    If the return code is CM_SUCCESS if there any running advertise, it will be disabled,
 *          and an event of @ref GAP_EVT_ADV_END_AFTER_DISABLE
 *
 * @note    After a success monitoring start, the application should expect
 *          @ref cmReportEvt_t event with the RSSI by calling @ref pfnCmReportEvtCB.
 */
cmErrorCodes_e ConnectionMonitor_StartMonitor(ConnectionMonitor_StartCmdParams_t *pNewCmData);

/*********************************************************************
 * @fn      ConnectionMonitor_StopMonitor
 *
 * @brief   This function used to stop monitoring a connection
 *
 * @param   pStopParams - Pointer to a stop command parameters
 *
 * @return  CM_SUCCESS, CM_NOT_CONNECTED, CM_UNSUPPORTED @ref cmErrorCodes_e
 *
 * @output This function will send a @ref CM_TRACKING_STOP_EVT event to the registered
 *         callback @ref pfnCmConnStatusEvtCB.
 */
cmErrorCodes_e ConnectionMonitor_StopMonitor(ConnectionMonitor_StopCmdParams_t *pStopParams);

/*********************************************************************
 * @fn      ConnectionMonitor_UpdateConn
 *
 * @brief   This function used to update monitored connection given by the CMS
 *          with the given connection data
 *
 * @param   pUpdateParams  - pointer to the update command parameters
 *
 * @return  CM_SUCCESS, CM_NOT_CONNECTED, CM_ALREADY_REQUESTED,
 *          CM_INVALID_PARAMS,CM_UNSUPPORTED,CM_FAILURE @ref cmErrorCodes_e
 *
 * @output  This function will adjust the monitored connection with the update event.
 *          but if the update event can't be applied with return code @ref CM_FAILURE,
 *          it will terminate the monitored connection with @ref CM_TRACKING_STOP_EVT and
 *          reason @ref CM_BAD_UPDATE_EVENT
 */
cmErrorCodes_e ConnectionMonitor_UpdateConn(ConnectionMonitor_UpdateCmdParams_t *pUpdateParams);

#ifdef __cplusplus
}
#endif

#endif /* APP_CM_API_H */
