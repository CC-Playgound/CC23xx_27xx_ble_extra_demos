/******************************************************************************

@file  app_connection.c

@brief This example file demonstrates how to work with connections with
the help of BLEAppUtil APIs.

The example provides also APIs to be called from upper layer and support sending events
to an external control module.

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
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "app_connection_api.h"
#include "app_gatt_api.h"
#include "app_main.h"
#include <string.h>

#if defined(RANGING_CLIENT) && !defined(RANGING_CLIENT_EXTCTRL_APP)
#include "app_ranging_client_api.h"
#endif

//*****************************************************************************
//! Defines
//*****************************************************************************
#define DEFAULT_EVT_REPORT_FREQUENCY         1  // <! The default report frequency of the event notification

//*****************************************************************************
//! Prototypes
//*****************************************************************************
static void Connection_ConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void Connection_ConnNotifyEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static uint8_t Connection_addConnInfo(uint16_t connHandle, uint8_t *pAddr);
static uint8_t Connection_removeConnInfo(uint16_t connHandle);

//*****************************************************************************
//! Globals
//*****************************************************************************

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref Connection_registerEvtHandler
//*****************************************************************//
static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

//! Holds the connection handles
static App_connInfo gConnectionConnList[MAX_NUM_BLE_CONNS];

//! Hold the rssi frequency report for each connection handle
static uint16_t gEventReportFrequency[MAX_NUM_BLE_CONNS];

BLEAppUtil_EventHandler_t connectionConnHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_CONN_TYPE,
    .pEventHandler  = Connection_ConnEventHandler,
    .eventMask      = BLEAPPUTIL_LINK_ESTABLISHED_EVENT |
                      BLEAPPUTIL_LINK_TERMINATED_EVENT |
                      BLEAPPUTIL_LINK_PARAM_UPDATE_EVENT |
                      BLEAPPUTIL_LINK_PARAM_UPDATE_REQ_EVENT |
                      BLEAPPUTIL_CONNECTING_CANCELLED_EVENT
};

BLEAppUtil_EventHandler_t connectionConnNotifyHandler =
{
    .handlerType    = BLEAPPUTIL_CONN_NOTI_TYPE,
    .pEventHandler  = Connection_ConnNotifyEventHandler,
    .eventMask      = BLEAPPUTIL_CONN_NOTI_CONN_ESTABLISHED |
                      BLEAPPUTIL_CONN_NOTI_PHY_UPDATE |
                      BLEAPPUTIL_CONN_NOTI_CONN_EVENT_ALL
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Connection_registerConnEventCmd
 *
 * @brief   Register connection event call back Api, This Api enables reporting the event
 *          call back of the connection handle in every connection event depends on
 *          the frequency rate that been asked for.
 *
 * @param   pParams - pointer to cmd struct params.
 *
* @return @ref SUCCESS
* @return @ref bleGAPNotFound : connection handle not found
* @return @ref bleInvalidRange : the callback function was NULL or action is invalid
* @return @ref bleMemAllocError : there is not enough memory to register the callback.
 */
bStatus_t Connection_registerConnEventCmd(Connection_registerConnEventCmdParams_t *pParams)
{
  bStatus_t status = FAILURE;
  uint16_t connIndx;

  if (pParams != NULL)
  {
    status = BLEAppUtil_registerConnNotifHandler(pParams->connHandle, (GAP_CB_Event_e) pParams->eventType);

    if (status == SUCCESS)
    {
      // Update the counters of the report frequency
      for ( connIndx = 0 ; connIndx < MAX_NUM_BLE_CONNS ; connIndx++ )
      {
        if ( pParams->connHandle == LINKDB_CONNHANDLE_ALL ||
             gConnectionConnList[connIndx].connHandle == pParams->connHandle)
        {
          // Update the frequency rate of the reporting
          if (pParams->reportFrequency >= DEFAULT_EVT_REPORT_FREQUENCY)
          {
            gEventReportFrequency[connIndx] = pParams->reportFrequency;
          }
          else
          {
            gEventReportFrequency[connIndx] = DEFAULT_EVT_REPORT_FREQUENCY;
          }
        }
        // Reset the counter
        gConnectionConnList[connIndx].notifyCbCnt = 0;
      }
    }
  }

  return status;
}

/*********************************************************************
 * @fn      Connection_unregisterConnEventCmd
 *
 * @brief   unregister connection event call back Api, This Api disables reporting the event
 *  for all the connections.
 *
 * @param   None
 *
* @return @ref SUCCESS
* @return @ref bleGAPNotFound : connection handle not found
* @return @ref bleInvalidRange : the callback function was NULL or action is invalid
* @return @ref bleMemAllocError : there is not enough memory to register the callback.
 */
bStatus_t Connection_unregisterConnEventCmd(void)
{
  bStatus_t status = FAILURE;

  // Note: BLEAppUtil_unRegisterConnNotifHandler api unregister all the connection notification
  // from all the connection.
  status = BLEAppUtil_unRegisterConnNotifHandler();

  uint8_t i = 0;

  // reset the counters for all the connections
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (gConnectionConnList[i].connHandle == LINKDB_CONNHANDLE_INVALID )
    {
      break;
    }
    gConnectionConnList[i].notifyCbCnt = 0;
  }

  return status;
}

/*********************************************************************
 * @fn      Connection_setPhyCmd
 *
 * @brief   Set the phy connection.
 *
 * @param   pParams - pointer to cmd struct params.
 *
 * @return  @ref HCI_SUCCESS
 */
bStatus_t Connection_setPhyCmd(Connection_setPhyCmdParams_t *pParams)
{
  bStatus_t status = FAILURE;

  if (pParams != NULL)
  {
     BLEAppUtil_ConnPhyParams_t phyParams;
     phyParams.connHandle   = pParams->connHandle;
     phyParams.allPhys      = pParams->allPhys;
     phyParams.txPhy        = pParams->txPhy;
     phyParams.rxPhy        = pParams->rxPhy;
     phyParams.phyOpts      = pParams->phyOpts;

     status = BLEAppUtil_setConnPhy(&phyParams);
  }

  return status;
}

/*********************************************************************
 * @fn      Connection_paramUpdateCmd
 *
 * @brief   Send connection parameters update request.
 *
 * @param   pParams - pointer to cmd struct params.
 *
 * @return  @ref HCI_SUCCESS
 */
bStatus_t Connection_paramUpdateCmd(Connection_paramUpdateCmdParams_t *pParams)
{
  bStatus_t status = FAILURE;

  if (pParams != NULL)
  {
     gapUpdateLinkParamReq_t connParams;
     connParams.connectionHandle = pParams->connectionHandle;
     connParams.intervalMin      = pParams->intervalMin;
     connParams.intervalMax      = pParams->intervalMax;
     connParams.connLatency      = pParams->connLatency;
     connParams.connTimeout      = pParams->connTimeout;
     connParams.signalIdentifier = pParams->signalIdentifier;

     status = BLEAppUtil_paramUpdateReq(&connParams);
  }

  return status;
}

/*********************************************************************
 * @fn      Connection_channelMapUpdateCmd
 *
 * @brief   Send channel map update request.
 *
 * @param   pParams - pointer to cmd struct params.
 *
 * @return  @ref HCI_SUCCESS
 */
bStatus_t Connection_channelMapUpdateCmd(Connection_channelMapUpdateCmdParams_t *pParams)
{
  bStatus_t status = FAILURE;

  if (pParams != NULL)
  {
    status = HCI_EXT_SetHostConnChanClassificationCmd( pParams->channelMap,
                                                       pParams->connectionHandle );
  }

  return status;
}

/*********************************************************************
 * @fn      Connection_dataLenUpdateCmd
 *
 * @brief   Set the data length connection.
 *
 * @param   pParams - pointer to cmd struct params.
 *
 * @return  @ref HCI_SUCCESS
 */
bStatus_t Connection_dataLenUpdateCmd(Connection_dataLenUpdateCmdParams_t *pParams)
{
  bStatus_t status = FAILURE;

  if (pParams != NULL)
  {
    status = HCI_LE_SetDataLenCmd( pParams->connectionHandle,
                                   pParams->txOctets,
                                   pParams->txTime );
  }

  return status;
}

/*********************************************************************
 * @fn      Connection_terminateLinkCmd
 *
 * @brief   Terminate a BLE connection.
 *
 * @param   connHandle - Connection handle to terminate.
 *
 * @return @ref SUCCESS : termination request sent to stack
 * @return @ref bleIncorrectMode : No Link to terminate
 * @return @ref bleInvalidTaskID : not app that established link
 */
bStatus_t Connection_terminateLinkCmd(uint16_t connHandle)
{
    return BLEAppUtil_disconnect(connHandle);
}

/*********************************************************************
 * @fn      Connection_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void Connection_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
  gExtEvtHandler = fEventHandler;
}

/*********************************************************************
 * @fn      Connection_addConnInfo
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          If there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t Connection_addConnInfo(uint16_t connHandle, uint8_t *pAddr)
{
  uint8_t i = 0;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (gConnectionConnList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      gConnectionConnList[i].connHandle = connHandle;
      memcpy(gConnectionConnList[i].peerAddress, pAddr, B_ADDR_LEN);
      gConnectionConnList[i].notifyCbCnt = 0;

      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      Connection_removeConnInfo
 *
 * @brief   Remove a device from the connected device list
 *
 * @param   connHandle - the connection handle
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          If connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t Connection_removeConnInfo(uint16_t connHandle)
{
  uint8_t i = 0;
  uint8_t index = 0;
  uint8_t maxNumConn = MAX_NUM_BLE_CONNS;

  for (i = 0; i < maxNumConn; i++)
  {
    if (gConnectionConnList[i].connHandle == connHandle)
    {
      // Mark the entry as deleted
      gConnectionConnList[i].connHandle = LINKDB_CONNHANDLE_INVALID;

      break;
    }
  }

  // Save the index to return
  index = i;

  // Shift the items in the array
  for (i = 0; i < maxNumConn - 1; i++)
  {
      if (gConnectionConnList[i].connHandle == LINKDB_CONNHANDLE_INVALID &&
          gConnectionConnList[i + 1].connHandle == LINKDB_CONNHANDLE_INVALID)
      {
        break;
      }
      if (gConnectionConnList[i].connHandle == LINKDB_CONNHANDLE_INVALID &&
          gConnectionConnList[i + 1].connHandle != LINKDB_CONNHANDLE_INVALID)
      {
        memmove(&gConnectionConnList[i], &gConnectionConnList[i+1], sizeof(App_connInfo));
        gConnectionConnList[i + 1].connHandle = LINKDB_CONNHANDLE_INVALID;
      }
  }

  return index;
}

/*********************************************************************
 * @fn      Connection_getConnList
 *
 * @brief   Get the connection list
 *
 * @return  connection list
 */
App_connInfo *Connection_getConnList(void)
{
  return gConnectionConnList;
}

/*********************************************************************
 * @fn      Connection_getConnhandle
 *
 * @brief   Find connHandle in the connected device list by index
 *
 * @param   index - the index of the connection in the @ref gConnectionConnList
 *
 * @return  the connHandle found. If there is no match,
 *          MAX_NUM_BLE_CONNS will be returned.
 */
uint16_t Connection_getConnhandle(uint8_t index)
{

  if (index < MAX_NUM_BLE_CONNS)
  {
    return gConnectionConnList[index].connHandle;
  }
  return MAX_NUM_BLE_CONNS;
}


/*********************************************************************
 * @fn      Connection_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @param   connHandle - the connection handle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, LL_INACTIVE_CONNECTIONS will be returned.
 */
uint16_t Connection_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (gConnectionConnList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return LL_INACTIVE_CONNECTIONS;
}

/*********************************************************************
 * @fn      Connection_extEvtHandler
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
static void Connection_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer if its handle exists
  if (gExtEvtHandler != NULL)
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      Connection_ConnEventHandler
 *
 * @brief   The purpose of this function is to handle connection related
 *          events that rise from the GAP and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void Connection_ConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (event)
    {
      case BLEAPPUTIL_LINK_ESTABLISHED_EVENT:
      {
        gapEstLinkReqEvent_t *pGapEstMsg = (gapEstLinkReqEvent_t *)pMsgData;

        // Add the connection to the connected device list
        Connection_addConnInfo(pGapEstMsg->connectionHandle, pGapEstMsg->devAddr);

#if defined(RANGING_CLIENT) && !defined(RANGING_CLIENT_EXTCTRL_APP)
        // Call MTU exchange
        AppGATT_exchangeMTU(pGapEstMsg->connectionHandle, MAX_PDU_SIZE);
#endif

        break;
      }

      case BLEAPPUTIL_LINK_TERMINATED_EVENT:
      {
        gapTerminateLinkEvent_t *pGapTermMsg = (gapTerminateLinkEvent_t *)pMsgData;

        // Remove the connection from the connected device list
        Connection_removeConnInfo(pGapTermMsg->connectionHandle);

#if defined(RANGING_CLIENT) && !defined(RANGING_CLIENT_EXTCTRL_APP)
        // Disable the RREQ
        AppRREQ_disable(pGapTermMsg->connectionHandle);
#endif

        break;
      }

      case BLEAPPUTIL_LINK_PARAM_UPDATE_REQ_EVENT:
      {
        gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsgData;

        // Only accept connection intervals with slave latency of 0
        // This is just an example of how the application can send a response
        if(pReq->req.connLatency == 0)
        {
            BLEAppUtil_paramUpdateRsp(pReq,TRUE);
        }
        else
        {
            BLEAppUtil_paramUpdateRsp(pReq,FALSE);
        }

        break;
      }

      case BLEAPPUTIL_LINK_PARAM_UPDATE_EVENT:
      {
        break;
      }

      default:
      {
        break;
      }
    }
    // Send the event to the upper layer
    Connection_extEvtHandler(BLEAPPUTIL_GAP_CONN_TYPE, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      Connection_ConnNotifyEventHandler
 *
 * @brief   The purpose of this function is to handle Connection Notification type events
 *          that rise from the GAP and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void Connection_ConnNotifyEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    Gap_ConnEventRpt_t * pConnEvt = (Gap_ConnEventRpt_t *)pMsgData;

    // Get the connection index
    uint16_t connIndx = Connection_getConnIndex(pConnEvt->handle);
    if (connIndx < MAX_NUM_BLE_CONNS)
    {
      // Promote the notifyCbCnt for the connection
      gConnectionConnList[connIndx].notifyCbCnt++;
    }

    switch (event)
    {
      case BLEAPPUTIL_CONN_NOTI_CONN_EVENT_ALL:
      {
        // If the notifyCbCnt meets the frequency rate then report it to the upper layer
        // Otherwise - don't send the event
        if (gConnectionConnList[connIndx].notifyCbCnt % gEventReportFrequency[connIndx] != 0)
        {
          break;
        }
      }
      // Always send the event if it is from the type :
      // BLEAPPUTIL_CONN_NOTI_CONN_ESTABLISHED / BLEAPPUTIL_CONN_NOTI_PHY_UPDATE
      case BLEAPPUTIL_CONN_NOTI_CONN_ESTABLISHED:
      case BLEAPPUTIL_CONN_NOTI_PHY_UPDATE:
      {
        // Send the event to the upper layer
        Connection_extEvtHandler(BLEAPPUTIL_CONN_NOTI_TYPE, event, pMsgData);
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
 * @fn      Connection_HciGAPEventHandler
 *
 * @brief   The purpose of this function is to handle HCI GAP events
 *          that rise from the HCI and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Connection_HciGAPEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    // Send the event to the upper layer
    Connection_extEvtHandler(BLEAPPUTIL_HCI_GAP_TYPE, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      Connection_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the connection
 *          application module and initiate the parser module.
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Connection_start()
{
  bStatus_t status = SUCCESS;

  status = BLEAppUtil_registerEventHandler(&connectionConnHandler);
  if (status != SUCCESS)
  {
    return(status);
  }

  status = BLEAppUtil_registerEventHandler(&connectionConnNotifyHandler);
  if (status != SUCCESS)
  {
    return(status);
  }

  // Return status value
  return(status);
}
#endif // ( HOST_CONFIG & (CENTRAL_CFG | PERIPHERAL_CFG) )
