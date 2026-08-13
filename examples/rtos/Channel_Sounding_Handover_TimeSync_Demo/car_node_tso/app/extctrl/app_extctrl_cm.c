/******************************************************************************

@file  app_extctrl_cm.c

@brief This file parse and process the messages comes form the external control module
 dispatcher module, and build the events from the app_cm.c application and
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


/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_extctrl_dispatcher.h"
#include "app_extctrl_cm.h"
#include "app_cm_api.h"
#include <string.h>

/*********************************************************************
 * PROTOTYPES
 */
static void ConnectionMonitorExtCtrl_sendErrEvt(uint8_t, uint8_t);
static void ConnectionMonitorExtCtrl_commandParser(uint8_t*);
static void ConnectionMonitorExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);
static void ConnectionMonitorExtCtrl_extHostEvtHandler(uint8_t*, uint16_t);
static void ConnectionMonitorExtCtrl_evtCodeHandler(uint32_t event, BLEAppUtil_msgHdr_t *pMsgData);
static void ConnectionMonitorExtCtrl_connUpdateEvtHandler(BLEAppUtil_cmConnUpdateEvt_t *pEvent);
static void ConnectionMonitorExtCtrl_reportEvtHandler(cmReportEvt_t *pReport);
static void ConnectionMonitorExtCtrl_connStatusEvtHandler(cmStatusEvt_t *pEvent);

/*********************************************************************
 * CONSTANTS
 */
#define CM_SERVING_CMD_START 0x00
#define CM_CMD_START         0x01
#define CM_CMD_STOP          0x02
#define CM_CMD_UPDATE_CONN   0x03

#define FIRST_PACKET_INDX    0x00
#define SECOND_PACKET_INDX   0x01

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBALS
 */
// The external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      ConnectionMonitorExtCtrl_commandParser
 *
 * @brief   This function parse the received host message, and call the
 *          the relevant API with the data.
 *
 * @param   pData - pointer to the data message.
 *
 * @return  None
 */
static void ConnectionMonitorExtCtrl_commandParser(uint8_t *pData)
{
    if (pData != NULL)
    {
        bStatus_t status = SUCCESS;
        appMsg_t* appMsg = (appMsg_t*)pData;

        switch (appMsg->cmdOp)
        {
            case CM_SERVING_CMD_START:
            {
                ConnectionMonitor_ServingStartCmdParams_t pParams;

                // Extract the connection handle
                pParams.connHandle = BUILD_UINT16(appMsg->pData[0],
                                                  appMsg->pData[1]);

                // Call the connection monitor application to receive the data
                status = ConnectionMonitor_GetConnData(&pParams);
                if ( status == SUCCESS )
                {
                    uint16_t totLen = sizeof(AppExtCtrlCmsStartEvent_t) + pParams.dataLen;

                    // Prepare the event with the data
                    AppExtCtrlCmsStartEvent_t *pEvt = (AppExtCtrlCmsStartEvent_t *)ICall_malloc(totLen);
                    if ( pEvt != NULL )
                    {
                        pEvt->event = APP_EXTCTRL_CM_SRV_DATA;
                        pEvt->accessAddr = pParams.accessAddr;
                        pEvt->connHandle = pParams.connHandle;
                        pEvt->dataLen = pParams.dataLen;
                        memcpy(pEvt->pData, pParams.pData, pParams.dataLen);

                        ConnectionMonitorExtCtrl_extHostEvtHandler((uint8_t *)pEvt, totLen);

                        // Free the resources
                        ICall_free(pEvt);
                    }
                    // Free the allocated recourse when called @ConnectionMonitor_GetConnData
                    ICall_free(pParams.pData);
                }
                break;
            }

            case CM_CMD_START:
            {
                ConnectionMonitor_StartCmdParams_t cmParams;
                AppExtCtrlCmStart_t *pMsgData = (AppExtCtrlCmStart_t *)appMsg->pData;

                // Extract the start parameters
                cmParams.cmDataSize = pMsgData->dataLen;
                cmParams.connTimeout = pMsgData->connTimeout;
                cmParams.maxSyncAttempts = pMsgData->maxSyncAttempts;
                cmParams.timeDeltaInUs = pMsgData->timeDelta;
                cmParams.timeDeltaMaxErrInUs = pMsgData->timeDeltaErr;

                cmParams.pCmData = (uint8_t *)ICall_malloc(cmParams.cmDataSize);
                if ( cmParams.pCmData != NULL )
                {
                    memcpy(cmParams.pCmData, pMsgData->data, cmParams.cmDataSize);
                    status = ConnectionMonitor_StartMonitor(&cmParams);
                    // Free the resources
                    ICall_free(cmParams.pCmData);
                }

                break;
            }

            case CM_CMD_STOP:
            {
                AppExtCtrlCmStop_t *pMsgData = (AppExtCtrlCmStop_t *)appMsg->pData;
                ConnectionMonitor_StopCmdParams_t params;
                params.connHandle = pMsgData->connHandle;
                status = ConnectionMonitor_StopMonitor(&params);
                break;
            }

            case CM_CMD_UPDATE_CONN:
            {
                ConnectionMonitor_UpdateCmdParams_t params;
                AppExtCtrlCmUpdate_t *pMsgData = (AppExtCtrlCmUpdate_t *)appMsg->pData;
                params.accessAddr       = pMsgData->accessAddr;
                params.connUpdateEvtCnt = pMsgData->connUpdateEvtCnt;
                params.updateType       = (cmConnUpdateEvtType_e)pMsgData->updateType;
                memcpy(&(params.updateEvt), pMsgData->data, pMsgData->dataLen);

                status = ConnectionMonitor_UpdateConn(&params);
                break;
            }

            default:
            {
                status = FAILURE;
                break;
            }
        }

        if (status != SUCCESS)
        {
          // Send error message
          ConnectionMonitorExtCtrl_sendErrEvt(status, appMsg->cmdOp);
        }
    }
}

/*********************************************************************
 * @fn      ConnectionMonitorExtCtrl_eventHandler
 *
 * @brief   This function handles the events raised from the application that
 *          this module registered to
 *
 * @param   eventType - the type of the events @ref BLEAppUtil_eventHandlerType_e.
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void ConnectionMonitorExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    if (pMsgData != NULL)
    {
      switch (eventType)
      {
        case BLEAPPUTIL_CM_TYPE:
        {
          // Call the function that will build the relevant event
            ConnectionMonitorExtCtrl_evtCodeHandler(event, pMsgData);
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
 * @fn      ConnectionMonitorExtCtrl_evtCodeHandler
 *
 * @brief   This function handles events from type @ref BLEAppUtil_CmEventMaskFlags_e
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void ConnectionMonitorExtCtrl_evtCodeHandler(uint32_t event, BLEAppUtil_msgHdr_t *pMsgData)
{
    if ( pMsgData != NULL )
    {
        switch(event)
        {
            case BLEAPPUTIL_CM_CONN_UPDATE_EVENT_CODE:
            {
                ConnectionMonitorExtCtrl_connUpdateEvtHandler((BLEAppUtil_cmConnUpdateEvt_t *)(pMsgData));
                break;
            }

            case BLEAPPUTIL_CM_REPORT_EVENT_CODE:
            {
                ConnectionMonitorExtCtrl_reportEvtHandler((cmReportEvt_t *)pMsgData);
                break;
            }

            case BLEAPPUTIL_CM_CONN_STATUS_EVENT_CODE:
            {
                ConnectionMonitorExtCtrl_connStatusEvtHandler((cmStatusEvt_t *)(pMsgData));
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
 * @fn      ConnectionMonitorExtCtrl_connUpdateEvtHandler
 *
 * @brief   This function handles the connection monitor serving connection update event
 *
 * @param   pEvent - Pointer to the connection status data
 *
 * @return  None
 */
static void ConnectionMonitorExtCtrl_connUpdateEvtHandler(BLEAppUtil_cmConnUpdateEvt_t *pEvent)
{
    cmConnUpdateEvt_t *pUpdateEvt;
    AppExtCtrlCmConnUpdateEvent_t *pEvtToSend;
    uint8_t totalLen = sizeof(AppExtCtrlCmConnUpdateEvent_t);
    uint8_t dataLen;
    uint8_t *pDataToCopy;

    if (pEvent != NULL)
    {
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
            pDataToCopy = (uint8_t *)&(pUpdateEvt->updateEvt.paramUpdateEvt);
        }
        else
        {
            // Unknown type.
            return;
        }

        // Add the dataLen to the total
        totalLen += dataLen;
        pEvtToSend = (AppExtCtrlCmConnUpdateEvent_t *)ICall_malloc(totalLen);
        if (pEvtToSend != NULL)
        {
            pEvtToSend->event            = APP_EXTCTRL_CM_CONN_UPDATE_EVENT;
            pEvtToSend->connHandle       = pEvent->connHandle;
            pEvtToSend->accessAddr       = pUpdateEvt->accessAddr;
            pEvtToSend->connUpdateEvtCnt = pUpdateEvt->connUpdateEvtCnt;
            pEvtToSend->updateType       = pUpdateEvt->updateType;
            pEvtToSend->dataLen          = dataLen;
            memcpy(pEvtToSend->data, pDataToCopy, dataLen);

            ConnectionMonitorExtCtrl_extHostEvtHandler((uint8_t *)pEvtToSend, totalLen);

            // Free the resources
            ICall_free(pEvtToSend);
        }
    }
}

/*********************************************************************
 * @fn      ConnectionMonitorExtCtrl_reportEvtHandler
 *
 * @brief   This function handles the connection monitor reports
 *
 * @param   pReport - Pointer to the connection monitor report
 *
 * @return  None
 */
static void ConnectionMonitorExtCtrl_reportEvtHandler(cmReportEvt_t *pReport)
{
    if ( pReport != NULL )
    {
        AppExtCtrlCmReportEvent_t evt;

        evt.event = APP_EXTCTRL_CM_REPORT;
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

        ConnectionMonitorExtCtrl_extHostEvtHandler((uint8_t *)&evt, sizeof(AppExtCtrlCmReportEvent_t));
    }
}

/*********************************************************************
 * @fn      ConnectionMonitorExtCtrl_connStatusEvtHandler
 *
 * @brief   This function handles the connection monitor connection status event
 *
 * @param   pEvent - Pointer to the connection status data
 *
 * @return  None
 */
static void ConnectionMonitorExtCtrl_connStatusEvtHandler(cmStatusEvt_t *pEvent)
{
    cmStatusEvtType_e eventType;

    if ( pEvent != NULL )
    {
        // Extract the event type
        eventType = pEvent->evtType;

        if ( eventType == CM_TRACKING_START_EVT )
        {
            AppExtCtrlCmStartEvent_t startEvt;
            cmStartEvt_t *pStart = (cmStartEvt_t *)(pEvent->pEvtData);

            startEvt.event = APP_EXTCTRL_CM_START_EVENT;
            startEvt.accessAddr = pStart->accessAddr;
            startEvt.connHandle = pStart->connHandle;
            startEvt.addrType = pStart->addrType;
            memcpy(startEvt.addr, pStart->addr, B_ADDR_LEN);
            ConnectionMonitorExtCtrl_extHostEvtHandler((uint8_t *)&startEvt, sizeof(AppExtCtrlCmStartEvent_t));
        }
        else
        {
            AppExtCtrlCmStopEvent_t stopEvt;
            cmStopEvt_t *pStop = (cmStopEvt_t *)(pEvent->pEvtData);

            stopEvt.event = APP_EXTCTRL_CM_STOP_EVENT;
            stopEvt.accessAddr = pStop->accessAddr;
            stopEvt.connHandle = pStop->connHandle;
            stopEvt.addrType = pStop->addrType;
            memcpy(stopEvt.addr, pStop->addr, B_ADDR_LEN);
            stopEvt.stopReason = pStop->stopReason;

            ConnectionMonitorExtCtrl_extHostEvtHandler((uint8_t *)&stopEvt, sizeof(AppExtCtrlCmStopEvent_t));
        }
    }
}

/*********************************************************************
 * @fn      ConnectionMonitorExtCtrl_extHostEvtHandler
 *
 * @brief   The purpose of this function is to forward the event to the external
 *          control host event handler that the external control returned once
 *          this module registered to it.
 *
 * @param   pData    - pointer to the event data
 * @param   dataLen  - data length.
 *
 * @return  None
 */
static void ConnectionMonitorExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if (gExtHostEvtHandler != NULL && pData != NULL)
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      ConnectionMonitorExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void ConnectionMonitorExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
    // Create error event structure
    AppExtCtrlErrorEvent_t errEvt;
    errEvt.event        = APP_EXTCTRL_FAILURE;
    errEvt.cmdOp        = cmdOp;
    errEvt.appSpecifier = APP_SPECIFIER_CM;
    errEvt.errCode      = errCode;

    // Send error event
    ConnectionMonitorExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

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
bStatus_t ConnectionMonitorExtCtrl_start(void)
{
    bStatus_t status = SUCCESS;

    // Register to the Dispatcher module
    gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_CM,
                                                       &ConnectionMonitorExtCtrl_commandParser,
                                                       APP_CAP_CM);

    // If the registration succeed, register event handler call back to the handover application
    if (gExtHostEvtHandler != NULL)
    {
      ConnectionMonitor_registerEvtHandler(&ConnectionMonitorExtCtrl_eventHandler);
    }

    return status;
}
