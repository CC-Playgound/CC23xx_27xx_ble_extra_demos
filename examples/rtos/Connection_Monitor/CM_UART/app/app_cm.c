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

 Copyright (c) 2025, Texas Instruments Incorporated
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

#include "ti/ble/stack_util/connection_monitor_types.h"
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include "ti/log/Log.h"
#include "ti/ble/controller/ll/ll_connection_monitor.h"
#include "ti/ble/app_util/menu/menu_module.h"

#define CM_SERVING_CMD_START 0x00
#define CM_CMD_START         0x01
#define CM_CMD_STOP          0x02
#define CM_CMD_UPDATE_CONN   0x03

#define FIRST_PACKET_INDX    0x00
#define SECOND_PACKET_INDX   0x01

/**
 * @brief Structure for Connection Monitor Connection update event
 */
typedef struct __attribute__((packed))
{
  uint16_t event;              //!< Event type
  uint16_t connHandle;         //!< Connection handle
  uint32_t accessAddr;         //!< Access address of the monitored connection
  uint16_t connUpdateEvtCnt;   //!< Connection update event counter
  uint8_t  updateType;         //!< Update type (PHY, Channel Map, or Parameter)
  uint8_t  dataLen;            //!< Length of the update data
  uint8_t  data[];             //!< Update data
} AppCmConnUpdateEvent_t;

uint8_t cmStatus[MAX_NUM_BLE_CONNS];
ConnectionMonitor_UpdateCmdParams_t cmUpdateParams;
//*****************************************************************************
//! Prototypes
//*****************************************************************************
//static void ConnectionMonitor_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void ConnectionMonitor_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void ConnectionMonitorServing_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
void uartWriteData(uint8_t *data, uint16_t len);
//*****************************************************************************
//! Globals
//*****************************************************************************

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref ConnectionMonitor_registerEvtHandler
//*****************************************************************//
// static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

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
// void ConnectionMonitor_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
// {
//   gExtEvtHandler = fEventHandler;
// }

/*********************************************************************
 * @fn      // ConnectionMonitor_extEvtHandler
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
// static void // ConnectionMonitor_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
// {
//   // Send the event to the upper layer if its handle exists
//   if (gExtEvtHandler != NULL)
//   {
//     gExtEvtHandler(eventType, event, pMsgData);
//   }
// }

void cmDataReceive(char *param)
{
    ConnectionMonitor_StartCmdParams_t cmParams;

    // Set the start parameters
    cmParams.cmDataSize = CMS_GetConnDataSize();
    cmParams.connTimeout = 2000;
    cmParams.maxSyncAttempts = 3;
    cmParams.timeDeltaInUs = 6000;
    cmParams.timeDeltaMaxErrInUs = 0;

    cmParams.pCmData = (uint8_t *)ICall_malloc(cmParams.cmDataSize);
    if ( cmParams.pCmData != NULL )
    {
        memcpy(cmParams.pCmData, (uint8_t *)param, cmParams.cmDataSize);

        uint8_t status = ConnectionMonitor_StartMonitor(&cmParams);
        // Free the resources
        ICall_free(cmParams.pCmData);
        if (status != SUCCESS)
        {
            while(1);
        }
    }
}

extern uint16_t numBytesRead;

void cmUpdateDataReceive(char *param)
{
    ConnectionMonitor_UpdateCmdParams_t cmParams;
    AppCmConnUpdateEvent_t pMsgData;
    memcpy(&pMsgData, param, sizeof(AppCmConnUpdateEvent_t));

    cmParams.accessAddr       = pMsgData.accessAddr;
    cmParams.connUpdateEvtCnt = pMsgData.connUpdateEvtCnt;
    cmParams.updateType       = (cmConnUpdateEvtType_e)pMsgData.updateType;
    memcpy(&(cmParams.updateEvt), param + sizeof(AppCmConnUpdateEvent_t), pMsgData.dataLen);
    
    // if (cmStatus[pMsgData.connHandle] == 1)
    {
        uint8_t status = ConnectionMonitor_UpdateConn(&cmParams);
        if (status != SUCCESS)
        {
            // if CM is not active, ignore the error
        }
    }
}

void cmServingStart(uint16_t connHandle)
{
    ConnectionMonitor_ServingStartCmdParams_t pParams;
    // Extract the connection handle
    pParams.connHandle = connHandle;

    // Call the connection monitor application to receive the data
    bStatus_t status = ConnectionMonitor_GetConnData(&pParams);
    if ( status == SUCCESS )
    {
        uartWriteData(pParams.pData, pParams.dataLen);
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
                GPIO_write(CONFIG_GPIO_LED_GREEN, 1);
                GPIO_write(CONFIG_GPIO_LED_GREEN, 0);
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

            GPIO_write(CONFIG_GPIO_LED_GREEN, 1);
            GPIO_write(CONFIG_GPIO_LED_GREEN, 0);
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
                BLEAppUtil_cmConnUpdateEvt_t *pEvent = (BLEAppUtil_cmConnUpdateEvt_t *)pMsgData;
                AppCmConnUpdateEvent_t *pEvtToSend;
                cmConnUpdateEvt_t *pUpdateEvt;
                uint8_t dataLen;
                uint8_t *pDataToCopy;
                
                uint8_t totalLen = sizeof(AppCmConnUpdateEvent_t);
               
                // Extract the event
                pUpdateEvt = &(pEvent->connUpdateEvt);

                if (pUpdateEvt->updateType == CM_PHY_UPDATE_EVT)
                {
                    dataLen = sizeof(cmPhyUpdateEvt_t);
                    pDataToCopy = (uint8_t *)&(pUpdateEvt->updateEvt.phyUpdateEvt);

                }
                else if (pUpdateEvt->updateType == CM_CHAN_MAP_UPDATE_EVT)
                {
                    dataLen = sizeof(cmChanMapUpdateEvt_t);
                    pDataToCopy = (uint8_t *)&(pUpdateEvt->updateEvt.chanMapUpdateEvt);
                }
                else if (pUpdateEvt->updateType == CM_PARAM_UPDATE_EVT)
                {
                    dataLen = sizeof(cmParamUpdateEvt_t);
                    pDataToCopy = (uint8_t *)&(pUpdateEvt->updateEvt.chanMapUpdateEvt);
                }
                else
                {
                    // Unknown type.
                    return;
                }


                // Add the dataLen to the total
                totalLen += dataLen;
                pEvtToSend = (AppCmConnUpdateEvent_t *)ICall_malloc(totalLen);
                if (pEvtToSend != NULL)
                {
                    pEvtToSend->event            = 0xA0;
                    pEvtToSend->connHandle       = pEvent->connHandle;
                    pEvtToSend->accessAddr       = pUpdateEvt->accessAddr;
                    pEvtToSend->connUpdateEvtCnt = pUpdateEvt->connUpdateEvtCnt;
                    pEvtToSend->updateType       = pUpdateEvt->updateType;
                    pEvtToSend->dataLen          = dataLen;
                    memcpy(pEvtToSend->data, pDataToCopy, dataLen);

                    uartWriteData((uint8_t *)pEvtToSend, totalLen);

                    // Free the resources
                    ICall_free(pEvtToSend);
                }
                // ConnectionMonitor_extEvtHandler(BLEAPPUTIL_CM_TYPE, event, pMsgData);
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
          {
            cmReportEvt_t *pReport = (cmReportEvt_t *)pMsgData;
            cmReportEvt_t evt;

            evt.accessAddr = pReport->accessAddr;
            evt.connHandle = pReport->connHandle;
            evt.connEvtCnt = pReport->connEvtCnt;
            evt.channel    = pReport->channel;
            // Copy the first packet data
            evt.packets[FIRST_PACKET_INDX].timestamp = pReport->packets[FIRST_PACKET_INDX].timestamp;
            evt.packets[FIRST_PACKET_INDX].status = pReport->packets[FIRST_PACKET_INDX].status;
            evt.packets[FIRST_PACKET_INDX].rssi = pReport->packets[FIRST_PACKET_INDX].rssi;
            evt.packets[FIRST_PACKET_INDX].pktLength = pReport->packets[FIRST_PACKET_INDX].pktLength;
            evt.packets[FIRST_PACKET_INDX].sn = pReport->packets[FIRST_PACKET_INDX].sn;
            evt.packets[FIRST_PACKET_INDX].nesn = pReport->packets[FIRST_PACKET_INDX].nesn;
            evt.packets[FIRST_PACKET_INDX].pad = 0;
            // Copy the second packet data
            evt.packets[SECOND_PACKET_INDX].timestamp = pReport->packets[SECOND_PACKET_INDX].timestamp;
            evt.packets[SECOND_PACKET_INDX].status = pReport->packets[SECOND_PACKET_INDX].status;
            evt.packets[SECOND_PACKET_INDX].rssi = pReport->packets[SECOND_PACKET_INDX].rssi;
            evt.packets[SECOND_PACKET_INDX].pktLength = pReport->packets[SECOND_PACKET_INDX].pktLength;
            evt.packets[SECOND_PACKET_INDX].sn = pReport->packets[SECOND_PACKET_INDX].sn;
            evt.packets[SECOND_PACKET_INDX].nesn = pReport->packets[SECOND_PACKET_INDX].nesn;
            evt.packets[SECOND_PACKET_INDX].pad = 0;

            // Log_printf(LogModule0, Log_DEBUG, "first status: %d,second status: %d", evt.packets[SECOND_PACKET_INDX].status, evt.packets[FIRST_PACKET_INDX].status);
            // Log_printf(LogModule0, Log_DEBUG, "connHandle:%d, connEvtInstant:%u, CM RSSI: %d %d", pReport->connHandle, pReport->connEvtCnt, (uint8_t)(evt.packets[FIRST_PACKET_INDX].rssi), (uint8_t)(evt.packets[SECOND_PACKET_INDX].rssi));

            // MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "CM report-"
            //                   "connHandle = " MENU_MODULE_COLOR_GREEN "%u" MENU_MODULE_COLOR_RESET
            //                   " connEvtInstant = " MENU_MODULE_COLOR_GREEN "%u" MENU_MODULE_COLOR_RESET
            //                   " Central RSSI = " MENU_MODULE_COLOR_GREEN "%d" MENU_MODULE_COLOR_RESET 
            //                   " Peri RSSI = " MENU_MODULE_COLOR_GREEN "%d" MENU_MODULE_COLOR_RESET,
            //                    pReport->connHandle, pReport->connEvtCnt, evt.packets[FIRST_PACKET_INDX].rssi, evt.packets[SECOND_PACKET_INDX].rssi);
                               
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "CM report-"
                            "connHandle = " MENU_MODULE_COLOR_GREEN "%u" MENU_MODULE_COLOR_RESET
                            " connEvtInstant = " MENU_MODULE_COLOR_GREEN "%u" MENU_MODULE_COLOR_RESET
                            " Central RSSI = " MENU_MODULE_COLOR_GREEN "%d" MENU_MODULE_COLOR_RESET 
                            " Peri RSSI = " MENU_MODULE_COLOR_GREEN "%d" MENU_MODULE_COLOR_RESET
                            " Central status = " MENU_MODULE_COLOR_GREEN "%d" MENU_MODULE_COLOR_RESET 
                            " Peri status = " MENU_MODULE_COLOR_GREEN "%d" MENU_MODULE_COLOR_RESET,
                            pReport->connHandle, pReport->connEvtCnt, evt.packets[FIRST_PACKET_INDX].rssi, evt.packets[SECOND_PACKET_INDX].rssi, evt.packets[FIRST_PACKET_INDX].status, evt.packets[SECOND_PACKET_INDX].status);
            break;
          }

          case BLEAPPUTIL_CM_CONN_STATUS_EVENT_CODE:
          {
            cmStatusEvtType_e eventType = ((cmStatusEvt_t *)pMsgData)->evtType;
            if ( eventType == CM_TRACKING_START_EVT )
            {
                // AppCtrlCmStartEvent_t startEvt;
                cmStartEvt_t *pStart = (cmStartEvt_t *)(((cmStatusEvt_t *)pMsgData)->pEvtData);

                // startEvt.accessAddr = pStart->accessAddr;
                // startEvt.connHandle = pStart->connHandle;
                // startEvt.addrType = pStart->addrType;
                // memcpy(startEvt.addr, pStart->addr, B_ADDR_LEN);

                // Log_printf(LogModule0, Log_DEBUG, "CM_TRACKING_START_EVT!");
                
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "TRACKING START connection handle:"
                            MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                            pStart->connHandle);
                // cmStatus[pStart->connHandle] = 1;
                // ConnectionMonitorExtCtrl_extHostEvtHandler((uint8_t *)&startEvt, sizeof(AppExtCtrlCmStartEvent_t));
            }
            else
            {
                // AppExtCtrlCmStopEvent_t stopEvt;
                cmStopEvt_t *pStop = (cmStopEvt_t *)((cmStatusEvt_t *)pMsgData)->pEvtData;

                // stopEvt.event = APP_EXTCTRL_CM_STOP_EVENT;
                // stopEvt.accessAddr = pStop->accessAddr;
                // stopEvt.connHandle = pStop->connHandle;
                // stopEvt.addrType = pStop->addrType;
                // memcpy(stopEvt.addr, pStop->addr, B_ADDR_LEN);
                // stopEvt.stopReason = pStop->stopReason;

                cmStatus[pStop->connHandle] = 0;
                // Log_printf(LogModule0, Log_DEBUG, "CM_TRACKING_STOP_EVT- stop reason: %d", pStop->stopReason);

                  MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "CM_TRACKING_STOP_EVT- stop reason:"
                            MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                            pStop->stopReason);
                // ConnectionMonitorExtCtrl_extHostEvtHandler((uint8_t *)&stopEvt, sizeof(AppExtCtrlCmStopEvent_t));
            }
              // ConnectionMonitor_extEvtHandler(BLEAPPUTIL_CM_TYPE, event, pMsgData);
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

    uartInit();

  return(status);
}
