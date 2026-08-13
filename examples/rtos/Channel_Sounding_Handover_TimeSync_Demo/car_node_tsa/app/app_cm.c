/******************************************************************************

@file  app_cm.c

@brief This example file demonstrates how to activate the connection monitor role with
the help of BLEAppUtil APIs.

In the ConnectionMonitor_start() function at the bottom of the file, registration,
initialization and activation are done using the BLEAppUtil API functions,
using the structures defined in the file.

The example provides also APIs to be called from upper layer and support sending events
to an external control module.

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

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti/ble/host/connection_monitor/connection_monitor.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "app_cm_api.h"
#include "ti_ble_config.h"
#include "app_main.h"

//*****************************************************************************
//! Prototypes
//*****************************************************************************
static void ConnectionMonitor_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void ConnectionMonitor_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void ConnectionMonitorServing_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Globals
//*****************************************************************************

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref ConnectionMonitor_registerEvtHandler
//*****************************************************************//
static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

// Events handlers struct, contains the handlers and event masks for
// the connection monitor serving
BLEAppUtil_EventHandler_t cmsHandler =
{
  .handlerType    = BLEAPPUTIL_CM_TYPE,
  .pEventHandler  = ConnectionMonitorServing_EventHandler,
  .eventMask      = BLEAPPUTIL_CM_CONN_UPDATE_EVENT_CODE
};

// Events handlers struct, contains the handlers and event masks for
// the connection monitor role
BLEAppUtil_EventHandler_t cmHandler =
{
  .handlerType    = BLEAPPUTIL_CM_TYPE,
  .pEventHandler  = ConnectionMonitor_EventHandler,
  .eventMask      = BLEAPPUTIL_CM_REPORT_EVENT_CODE   |
                    BLEAPPUTIL_CM_CONN_STATUS_EVENT_CODE
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*******************************************************************************
 * This function is called to register the external event handler
 *
 * Public function defined in app_cm_api.h
 */
void ConnectionMonitor_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
  gExtEvtHandler = fEventHandler;
}

/*********************************************************************
 * @fn      ConnectionMonitor_extEvtHandler
 *
 * @brief   The purpose of this function is to forward the event to the external
 *          event handler that registered to handle the events.
 *
 * @param   eventType - the event type of the event @ref BLEAppUtil_eventHandlerType_e
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  none
 */
static void ConnectionMonitor_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer if its handle exists
  if (gExtEvtHandler != NULL)
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}

/*******************************************************************************
 * This function get the needed information from the connection monitor - serving
 * side
 *
 * Public function defined in app_cm_api.h
 */
cmErrorCodes_e ConnectionMonitor_GetConnData(ConnectionMonitor_ServingStartCmdParams_t *pCmsParams)
{
    cmErrorCodes_e status = CM_FAILURE;
    cmsConnDataParams_t cmSrvParams;

    if ( pCmsParams != NULL )
    {
        status = CMS_InitConnDataParams(&cmSrvParams);

        if ( status == SUCCESS )
        {
            cmSrvParams.dataSize = CMS_GetConnDataSize();
            // The caller is responsible to call "ConnectionMonitor_FreeConnData" to release the buffer
            cmSrvParams.pCmData = (uint8_t *)ICall_malloc(cmSrvParams.dataSize);

            // Check the allocation is successful and request the CMS data from the stack
            if ( cmSrvParams.pCmData != NULL )
            {
                status = CMS_GetConnData(pCmsParams->connHandle, &cmSrvParams);

                if ( ( status != SUCCESS ) &&
                     ( cmSrvParams.pCmData != NULL ) )
                {
                    // Free the resources
                    ICall_free(cmSrvParams.pCmData);
                }
                else
                {
                    // Give the caller the access address
                    pCmsParams->accessAddr = cmSrvParams.accessAddr;
                    // Give the caller the data size and pointer
                    pCmsParams->dataLen = cmSrvParams.dataSize;
                    pCmsParams->pData = cmSrvParams.pCmData;
                }
            }
            else
            {
                // The allocation failed
                status = CM_NO_RESOURCE;
            }
        }
    }

    return(status);
}

/*******************************************************************************
 * This function used to start monitoring a connection by the candidate based
 * on the given connection data
 *
 * Public function defined in app_cm_api.h
 */
cmErrorCodes_e ConnectionMonitor_StartMonitor(ConnectionMonitor_StartCmdParams_t *pNewCmData)
{
    cmErrorCodes_e status = CM_SUCCESS;
    cmStartMonitorParams_t cmParams;

    if ( pNewCmData != NULL )
    {
        // Initialize the CM parameters
        status = CM_InitStartMonitorParams(&cmParams);

        if ( status == SUCCESS )
        {
            cmParams.timeDeltaInUs = pNewCmData->timeDeltaInUs;
            cmParams.timeDeltaMaxErrInUs = pNewCmData->timeDeltaMaxErrInUs;
            cmParams.connTimeout = pNewCmData->connTimeout;
            cmParams.maxSyncAttempts = pNewCmData->maxSyncAttempts;
            cmParams.cmDataSize = pNewCmData->cmDataSize;
            cmParams.pCmData = pNewCmData->pCmData;

            status = CM_StartMonitor(&cmParams);
        }
    }
    else
    {
        status = CM_FAILURE;
    }

    return(status);
}

/*******************************************************************************
 * This function used to stop monitoring a connection
 *
 * Public function defined in app_cm_api.h
 */
cmErrorCodes_e ConnectionMonitor_StopMonitor(ConnectionMonitor_StopCmdParams_t *pStopParams)
{
    return CM_StopMonitor(pStopParams->connHandle);
}

/*******************************************************************************
 * This function used to update monitored connection given by the CMS
 * with the given connection data
 *
 * Public function defined in app_cm_api.h
 */

cmErrorCodes_e ConnectionMonitor_UpdateConn(ConnectionMonitor_UpdateCmdParams_t *pUpdateParams)
{
    cmErrorCodes_e status = CM_SUCCESS;
    cmConnUpdateEvt_t connUpdateEvt;

    if ( pUpdateParams != NULL )
    {
        connUpdateEvt.accessAddr         =  pUpdateParams->accessAddr;
        connUpdateEvt.connUpdateEvtCnt   =  pUpdateParams->connUpdateEvtCnt;
        connUpdateEvt.updateType         =  pUpdateParams->updateType;

        switch(connUpdateEvt.updateType)
        {
            case CM_PHY_UPDATE_EVT:
            {
                connUpdateEvt.updateEvt.phyUpdateEvt = pUpdateParams->updateEvt.phyUpdateEvt;
                break;
            }

            case CM_CHAN_MAP_UPDATE_EVT:
            {
                connUpdateEvt.updateEvt.chanMapUpdateEvt = pUpdateParams->updateEvt.chanMapUpdateEvt;
                break;
            }

            case CM_PARAM_UPDATE_EVT:
            {
                connUpdateEvt.updateEvt.paramUpdateEvt = pUpdateParams->updateEvt.paramUpdateEvt;
                break;
            }

            default:
            {
                status = CM_FAILURE;
                break;
            }
        }

        if( status == CM_SUCCESS )
        {
          status = CM_UpdateConn(&connUpdateEvt);
        }
    }
    else
    {
        status = CM_FAILURE;
    }

    return(status);
}

/*********************************************************************
 * @fn      ConnectionMonitorServing_EventHandler
 *
 * @brief   The purpose of this function is to handle connection monitor serving events
 *
 * @param   event    - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void ConnectionMonitorServing_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    if (pMsgData != NULL)
    {
        switch (event)
        {
            case BLEAPPUTIL_CM_CONN_UPDATE_EVENT_CODE:
            {
                ConnectionMonitor_extEvtHandler(BLEAPPUTIL_CM_TYPE, event, pMsgData);
                break;
            }

            default:
            {
                break;
            }
        }
    }
}

/*********************************************************************
 * @fn      ConnectionMonitor_EventHandler
 *
 * @brief   The purpose of this function is to handle connection monitor events
 *
 * @param   event    - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void ConnectionMonitor_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    if (pMsgData != NULL)
    {
      switch (event)
      {
          case BLEAPPUTIL_CM_REPORT_EVENT_CODE:
          case BLEAPPUTIL_CM_CONN_STATUS_EVENT_CODE:
          {
              ConnectionMonitor_extEvtHandler(BLEAPPUTIL_CM_TYPE, event, pMsgData);
              break;
          }

          default:
          {
              break;
          }
      }
    }
}

/*******************************************************************************
 * This function handle the initialization of the connection monitor module.
 *
 * Public function defined in app_main.h.
 */
bStatus_t ConnectionMonitor_start()
{
    bStatus_t status = SUCCESS;

    // Register to BLE APP Util event handler CMS
    status = BLEAppUtil_registerEventHandler(&cmsHandler);

    if ( status == SUCCESS )
    {
        // Register to BLE APP Util event handler CM
        status = BLEAppUtil_registerEventHandler(&cmHandler);
    }

    if ( status == SUCCESS )
    {
        // Register to receive connection monitor serving scallbacks
        status = BLEAppUtil_registerCMSCBs();
    }

    if ( status == SUCCESS )
    {
        // Register to receive connection monitor role callbacks
        status = BLEAppUtil_registerCMCBs();
    }

  return(status);
}
