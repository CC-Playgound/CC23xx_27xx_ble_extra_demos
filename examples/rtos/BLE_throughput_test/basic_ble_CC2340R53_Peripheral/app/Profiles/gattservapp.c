
/******************************************************************************

 @file  gattservapp.c

 @brief This file contains the GATT Server Application.

 Group: WCS, BTS
 Target Device: cc23xx

 ******************************************************************************
 
 Copyright (c) 2009-2026, Texas Instruments Incorporated

 All rights reserved not granted herein.
 Limited License.

 Texas Instruments Incorporated grants a world-wide, royalty-free,
 non-exclusive license under copyrights and patents it now or hereafter
 owns or controls to make, have made, use, import, offer to sell and sell
 ("Utilize") this software subject to the terms herein. With respect to the
 foregoing patent license, such license is granted solely to the extent that
 any such patent is necessary to Utilize the software alone. The patent
 license shall not apply to any combinations which include this software,
 other than combinations with devices manufactured by or for TI ("TI
 Devices"). No hardware patent is licensed hereunder.

 Redistributions must preserve existing copyright notices and reproduce
 this license (including the above copyright notice and the disclaimer and
 (if applicable) source code license limitations below) in the documentation
 and/or other materials provided with the distribution.

 Redistribution and use in binary form, without modification, are permitted
 provided that the following conditions are met:

   * No reverse engineering, decompilation, or disassembly of this software
     is permitted with respect to any software provided in binary form.
   * Any redistribution and use are licensed by TI for use only with TI Devices.
   * Nothing shall obligate TI to provide you with source code for the software
     licensed and provided to you in object code.

 If software source code is provided to you, modification and redistribution
 of the source code are permitted provided that the following conditions are
 met:

   * Any redistribution and use of the source code, including any resulting
     derivative works, are licensed by TI for use only with TI Devices.
   * Any redistribution and use of any object code compiled from the source
     code and any resulting derivative works, are licensed by TI for use
     only with TI Devices.

 Neither the name of Texas Instruments Incorporated nor the names of its
 suppliers may be used to endorse or promote products derived from this
 software without specific prior written permission.

 DISCLAIMER.

 THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )

/*******************************************************************************
 * INCLUDES
 */
#include "ti/ble/stack_util/bcomdef.h"
#include "ti/ble/host/common/linkdb.h"
#include "ti/ble/host/common/linkdb_internal.h"
#include "ti/ble/stack_util/osal/osal_bufmgr.h"
#include "ti/ble/stack_util/osal/osal.h"
#include "ti/ble/stack_util/osal/osal_list.h"
#include "ti/ble/host/l2cap/l2cap_internal.h"
#include "ti/ble/host/att/att_internal.h"
#include "ti/ble/host/gap/gap.h"
#include "ti/ble/host/gatt/gatt.h"
#include "ti/ble/host/gatt/gatt_uuid.h"
#include "ti/ble/host/gatt/gattservapp.h"
#include "ti/ble/host/gatt/gatt_internal.h"
#include "ti/ble/stack_util/lib_opt/map_direct.h"
#include <ti/drivers/utils/Math.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#if defined ( TESTMODES )
#define ATT_TEST_MODE_MAX_MTU_SIZE       517                  //!< Maximum ATT MTU size, specific for test modes.
#endif /* TESTMODES */

/*********************************************************************
 * TYPEDEFS
 */

// List element for parameter update and PHY command status lists
typedef struct
{
  osal_list_elem elem;
  uint16 connHandle;         //!< Connection message was received on
  uint8 method;              //!< Type of message
  gattMsg_t msg;             //!< Attribute protocol/profile message
} gattReTx_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 appTaskID = INVALID_TASK_ID; // The task ID of an app/profile that
                                   // wants GATT Server event messages

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 GATTServApp_TaskID;   // Task ID for internal task/event processing

static uint8 GATTServApp_att_delayed_req = 0;
static uint8 GATTServApp_gatt_no_service_changed = 1;

// Server Prepare Write table (one entry per each physical link)
static prepareWrites_t *prepareWritesTbl;

// Callbacks for services
static serviceCBsList_t *serviceCBsList = NULL;

// Maximum number of attributes that Server can prepare for writing per Client
static uint8 maxNumPrepareWrites = 0;

// Globals to be used for processing an incoming request
static uint16 attrLen;
static uint8 *pAttrValue = NULL;
static attMsg_t rsp;
//#ifdef ATT_DELAYED_REQ
static gattMsgEvent_t req;
//#endif // ATT_DELAYED_REQ

/*** Defined GATT Attributes ***/

// GATT Service attribute
const gattAttrType_t gattService = { ATT_BT_UUID_SIZE, gattServiceUUID };

//#ifndef GATT_NO_SERVICE_CHANGED
// Service Changed Characteristic Properties
static uint8 serviceChangedCharProps = GATT_PROP_INDICATE;

// Service Changed attribute (hidden). Set the affected Attribute Handle range
// to 0x0001 to 0xFFFF to indicate to the client to rediscover the entire set
// of Attribute Handles on the server.

// Client Characteristic configuration. Each client has its own instantiation
// of the Client Characteristic Configuration. Reads of the Client Characteristic
// Configuration only shows the configuration for that client and writes only
// affect the configuration of that client.
static gattCharCfg_t *indCharCfg;

//#endif // GATT_NO_SERVICE_CHANGED

#if defined ( TESTMODES )
  static uint16 gattservapp_paramValue = 0;
#endif

// List to store GATT retransmission
static osal_list_list gattReTxList;

/*********************************************************************
 * Profile Attributes - Table
 */

// GATT Attribute Table
static gattAttribute_t gattAttrTbl[] = {
  // Generic Attribute Profile
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&gattService                     /* pValue */
  },

//#ifndef GATT_NO_SERVICE_CHANGED
    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &serviceChangedCharProps
    },

      // Attribute Service Changed
      {
        { ATT_BT_UUID_SIZE, serviceChangedUUID },
        0,
        0,
        NULL
      },

      // Client Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&indCharCfg
      }
//#endif // GATT_NO_SERVICE_CHANGED
};
//else // GATT_NO_SERVICE_CHANGED
static gattAttribute_t gattAttrTbl_gatt_no_service_changed[] = {
  // Generic Attribute Profile
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&gattService                     /* pValue */
  },
};
// #endif // GATT_NO_SERVICE_CHANGED

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static uint8                    gattServApp_ProcessMsg( gattMsgEvent_t *pMsg );
static bStatus_t                gattServApp_ProcessFindByTypeValueReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle );
static bStatus_t                gattServApp_ProcessReadReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle );
static bStatus_t                gattServApp_ProcessReadBlobReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle );
static bStatus_t                gattServApp_ProcessReadMultiReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle );
static bStatus_t                gattServApp_ProcessReadByGrpTypeReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle );
static prepareWrites_t *        gattServApp_FindPrepareWriteQ( uint16 connHandle );
static bStatus_t                gattServApp_buildReadByTypeRsp( uint16 connHandle, uint8 *pAttrValue, uint16 attrLen, uint16 attrHandle );
static bStatus_t                gattServApp_ProcessWriteReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle, uint8 *pSafeToDealloc );
static uint8                    gattServApp_IsWriteLong( attExecuteWriteReq_t *pReq, prepareWrites_t *pQueue );
static bStatus_t                gattServApp_ProcessPrepareWriteReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle, uint8 *pSafeToDealloc );
static bStatus_t                gattServApp_SetNumPrepareWrites( uint8 numPrepareWrites );
static bStatus_t                gattServApp_ProcessReadByTypeReq( gattMsgEvent_t *pMsg, bStatus_t status, uint16 *pErrHandle );
static bStatus_t                gattServApp_ProcessExecuteWriteReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle );
static bStatus_t                gattServApp_RegisterServiceCBs( uint16 handle, const gattServiceCBs_t *pServiceCBs );
static bStatus_t                gattServApp_DeregisterServiceCBs( uint16 handle );
static pfnGATTReadAttrCB_t      gattServApp_FindReadAttrCB( uint16 handle );
static pfnGATTWriteAttrCB_t     gattServApp_FindWriteAttrCB( uint16 handle );
static uint8                    gattServApp_PrepareWriteQInUse( void );
static const gattServiceCBs_t  *gattServApp_FindServiceCBs( uint16 service );
pfnGATTAuthorizeAttrCB_t gattServApp_FindAuthorizeAttrCB( uint16 handle );
bStatus_t                gattServApp_EnqueueReTx( uint16 connHandle, uint8 method, gattMsg_t *pMsg );
static void                     gattServApp_DequeueReTx( void );
static bStatus_t                gattServApp_ProcessWriteLong( gattMsgEvent_t *pMsg, prepareWrites_t *pQueue, uint16 *pErrHandle );
static void                     gattServApp_ClearPrepareWriteQ( prepareWrites_t *pQueue );
static bStatus_t                gattServApp_ProcessExchangeMTUReq( gattMsgEvent_t *pMsg );
static bStatus_t                gattServApp_EnqueuePrepareWriteReq( uint16 connHandle, attPrepareWriteReq_t *pReq );
static bStatus_t                gattServApp_ProcessReliableWrites( gattMsgEvent_t *pMsg, prepareWrites_t *pQueue, uint16 *pErrHandle );
static void                     gattServApp_ResetCharCfg( uint16 connHandle );

/*********************************************************************
 * API FUNCTIONS
 */

// GATT App Callback functions
static void gattServApp_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

//#ifndef GATT_NO_SERVICE_CHANGED
static bStatus_t gattServApp_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                          uint8 *pValue, uint16 len, uint16 offset,
                                          uint8 method );
//#endif // GATT_NO_SERVICE_CHANGED

/*********************************************************************
 * PROFILE CALLBACKS
 */

//#ifndef GATT_NO_SERVICE_CHANGED
// GATT Service Callbacks
static const gattServiceCBs_t gattServiceCBs =
{
  NULL,                    // Read callback function pointer
  gattServApp_WriteAttrCB, // Write callback function pointer
  NULL                     // Authorization callback function pointer
};
//#endif // GATT_NO_SERVICE_CHANGED
/*********************************************************************
 * @fn      GATTServApp_RegisterForMsgs
 *
 * @brief   Register your task ID to receive event messages from
 *          the GATT Server Application.
 *
 * @param   taskId - Default task ID to send events
 *
 * @return  none
 */
void GATTServApp_RegisterForMsg( uint8 taskID )
{
  uint32_t status;

  // TODO: improve documentation around this fxn and who can call it.  basic_ble
  // app threads never call this, but it is called from the stack thread (via
  // GAPBondMgr_Register()).  If this is a purely internal fxn, only callable
  // from within the stack thread, then no need to BLE_invokeIfRequired()
  if (!BLE_invokeIfRequired((void *)&GATTServApp_RegisterForMsg, &status,
    ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, taskID)))
  {

  appTaskID = taskID;

  }
}

/*********************************************************************
 * @fn      GATTServApp_Init
 *
 * @brief   Initialize the GATT Server Application.
 *
 * @param   taskId - Task identifier for the desired task
 *
 * @return  none
 */
void GATTServApp_Init( uint8 taskId, uint8_t cfg_GATTServApp_att_delayed_req, uint8_t cfg_GATTServApp_gatt_no_service_changed, uint8_t cfg_gatt_max_num_prepare_writes )
{
  GATTServApp_TaskID = taskId;

  GATTServApp_att_delayed_req = cfg_GATTServApp_att_delayed_req;
  GATTServApp_gatt_no_service_changed = cfg_GATTServApp_gatt_no_service_changed;

  // Allocate Server Info table
  prepareWritesTbl = MAP_osal_mem_alloc( sizeof(prepareWrites_t) * linkDBNumConns );
  if ( prepareWritesTbl == NULL )
  {
    return;
  }


  if (!GATTServApp_gatt_no_service_changed) // #ifndef GATT_NO_SERVICE_CHANGED
  {
    // Allocate Client Characteristic Configuration table
    indCharCfg = (gattCharCfg_t *)MAP_osal_mem_alloc( sizeof(gattCharCfg_t) * linkDBNumConns );
    if ( indCharCfg == NULL )
    {
      // Free already allocated data
      MAP_osal_mem_free( prepareWritesTbl );

      return;
    }

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, indCharCfg );
  } // #endif // GATT_NO_SERVICE_CHANGED


  // Initialize Prepare Write Table
  for ( uint8 i = 0; i < linkDBNumConns; i++ )
  {
    // Initialize connection handle
    prepareWritesTbl[i].connHandle = LINKDB_CONNHANDLE_INVALID;

    // Initialize the prepare write queue
    prepareWritesTbl[i].pPrepareWriteQ = NULL;
  }

  // Set up the initial prepare write queues
  if (!cfg_gatt_max_num_prepare_writes ) // #ifndef GATT_MAX_PREPARE_WRITES
  {
    VOID gattServApp_SetNumPrepareWrites( GATT_MAX_NUM_PREPARE_WRITES );
  }
  else
  {
    VOID gattServApp_SetNumPrepareWrites( cfg_gatt_max_num_prepare_writes );
  }

  // Register to receive incoming ATT Requests
  GATT_RegisterForReq( GATTServApp_TaskID );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( gattServApp_HandleConnStatusCB );

  // Register for l2cap to know what task to check against for ATT
  // retransmissions
  l2capRegisterATTReTxTask( GATTServApp_TaskID );
}

#include <ti/drivers/GPIO.h>
/*********************************************************************
 * @fn      GATTServApp_ProcessEvent
 *
 * @brief   GATT Server Application Task event processor. This function
 *          is called to process all events for the task. Events include
 *          timers, messages and any other user defined events.
 *
 * @param   task_id - The OSAL assigned task ID.
 * @param   events - events to process. This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint32 GATTServApp_ProcessEvent( uint8 task_id, uint32 events )
{
  bool safeToDealloc = TRUE;

  if ( events & SYS_EVENT_MSG )
  {
    osal_event_hdr_t *pMsg;

    if ( (pMsg = ( osal_event_hdr_t *)osal_msg_receive( GATTServApp_TaskID )) != NULL )
    {
      // Process incoming messages
      switch ( pMsg->event )
      {
        // Incoming GATT message
        case GATT_MSG_EVENT:
          {
            GPIO_toggle(24);
            gattMsgEvent_t *pEvent = (gattMsgEvent_t *)pMsg;
            
            if ( gattServApp_ProcessMsg( pEvent ) )
            {
              // Safe to free the payload buffer (if present)
              GATT_bm_free( &pEvent->msg, pEvent->method );
            }
          }
          break;

          // Incoming L2CAP message
          case L2CAP_SIGNAL_EVENT:
          {
            l2capSignalEvent_t* pEvent = (l2capSignalEvent_t *)pMsg;

            if (pEvent->opcode == L2CAP_NUM_CTRL_DATA_PKT_EVT)
            {
              // Dequeue retransmission if it exists and free if sent successfully
              gattServApp_DequeueReTx();
            }

            // If this is to be forwarded to another task
            if (flowCtrlFwdTaskId != INVALID_TASK_ID)
            {
              // Forward to desired task
              if(MAP_osal_msg_send(flowCtrlFwdTaskId, (uint8_t *)pMsg) == SUCCESS)
              {
                // Don't deallocate the message as it is now the flow control
                // task's responsibility.
                safeToDealloc = FALSE;
              }
            }
          }
          break;

        default:
          // Unsupported message
          break;
      }

      // Deallocate OSAL message unless it was forwarded
      if (safeToDealloc == TRUE)
      {
        // Release the OSAL message
        VOID MAP_osal_msg_deallocate((uint8 *)pMsg);
      }
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Discard unknown events
  return 0;
}

/******************************************************************************
 * @fn      GATTServApp_RegisterService
 *
 * @brief   Register a service's attribute list and callback functions with
 *          the GATT Server Application.
 *
 * @param   pAttrs - Array of attribute records to be registered
 * @param   numAttrs - Number of attributes in array
 * @param   encKeySize - Minimum encryption key size required by service (7-16 bytes)
 * @param   pServiceCBs - Service callback function pointers
 *
 * @return  SUCCESS: Service registered successfully.
 *          INVALIDPARAMETER: Invalid service fields.
 *          FAILURE: Not enough attribute handles available.
 *          bleMemAllocError: Memory allocation error occurred.
 *          bleInvalidRange: Encryption key size's out of range.
 */
bStatus_t GATTServApp_RegisterService( gattAttribute_t *pAttrs,
                                       uint16 numAttrs, uint8 encKeySize,
                                       const gattServiceCBs_t *pServiceCBs )
{
  uint8 status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATTServApp_RegisterService, &invokeStatus, pAttrs,
      numAttrs, encKeySize, pServiceCBs))
  {
    status = (uint8)invokeStatus;
  }
  else
  {

  // First register the service attribute list with GATT Server
  if ( pAttrs != NULL )
  {
    gattService_t service;

    service.attrs = pAttrs;
    service.numAttrs = numAttrs;
    service.encKeySize = encKeySize;

    status = GATT_RegisterService( &service );
    if ( ( status == SUCCESS ) && ( pServiceCBs != NULL ) )
    {
      // Register the service CBs with GATT Server Application
      status = gattServApp_RegisterServiceCBs( GATT_SERVICE_HANDLE( pAttrs ),
                                               pServiceCBs );
    }
  }
  else
  {
    status = INVALIDPARAMETER;
  }
  }
  return ( status );
}

/******************************************************************************
 * @fn      GATTServApp_DeregisterService
 *
 * @brief   Deregister a service's attribute list and callback functions from
 *          the GATT Server Application.
 *
 *          NOTE: It's the caller's responsibility to free the service attribute
 *          list returned from this API.
 *
 * @param   handle - handle of service to be deregistered
 * @param   p2pAttrs - pointer to array of attribute records (to be returned)
 *
 * @return  SUCCESS: Service deregistered successfully.
 *          FAILURE: Service not found.
 */
bStatus_t GATTServApp_DeregisterService( uint16 handle, gattAttribute_t **p2pAttrs )
{
  uint8 status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATTServApp_DeregisterService, &invokeStatus, handle, p2pAttrs))
  {
    status = (uint8)invokeStatus;
  }
  else
  {

  // First deregister the service CBs with GATT Server Application
  status = gattServApp_DeregisterServiceCBs( handle );
  if ( status == SUCCESS )
  {
    gattService_t service;

    // Deregister the service attribute list with GATT Server
    status = GATT_DeregisterService( handle, &service );
    if ( status == SUCCESS )
    {
      if ( p2pAttrs != NULL )
      {
        *p2pAttrs = service.attrs;
      }
    }
  }
  }
  return ( status );
}

/*********************************************************************
 * @fn      GATTServApp_SetParameter
 *
 * @brief   Set a GATT Server parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   pValue - pointer to data to write.  This is dependent on the
 *                   the parameter ID and WILL be cast to the appropriate
 *                   data type (example: data type of uint16 will be cast
 *                   to uint16 pointer).
 *
 * @return  SUCCESS: Parameter set successful
 *          FAILURE: Parameter in use
 *          INVALIDPARAMETER: Invalid parameter
 *          bleInvalidRange: Invalid value
 *          bleMemAllocError: Memory allocation failed
 */
bStatus_t GATTServApp_SetParameter( uint8 param, uint8 len, void *pValue )
{
  bStatus_t status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATTServApp_SetParameter, &invokeStatus, param, len, pValue))
  {
    status = (bStatus_t)invokeStatus;
  }
  else
  {

  switch ( param )
  {
    case GATT_PARAM_NUM_PREPARE_WRITES:
      if ( len == sizeof ( uint8 ) )
      {
        if ( !MAP_gattServApp_PrepareWriteQInUse() )
        {
          // Set the new nunber of prepare writes
          status = MAP_gattServApp_SetNumPrepareWrites( *((uint8*)pValue) );
        }
        else
        {
          status = FAILURE;
        }
      }
      else
      {
        status = bleInvalidRange;
      }
      break;

    default:
      status = INVALIDPARAMETER;
      break;
  }
  }
  return ( status );
}

/*********************************************************************
 * @fn      GATTServApp_GetParameter
 *
 * @brief   Get a GATT Server parameter.
 *
 * @param   param - Profile parameter ID
 * @param   pValue - pointer to data to put. This is dependent on the
 *                   parameter ID and WILL be cast to the appropriate
 *                   data type (example: data type of uint16 will be
 *                   cast to uint16 pointer).
 *
 * @return  SUCCESS: Parameter get successful
 *          INVALIDPARAMETER: Invalid parameter
 */
bStatus_t GATTServApp_GetParameter( uint8 param, void *pValue )
{
  bStatus_t status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATTServApp_GetParameter, &invokeStatus, param, pValue))
  {
    status = (bStatus_t)invokeStatus;
  }
  else
  {
    status = SUCCESS;

    switch ( param )
    {
      case GATT_PARAM_NUM_PREPARE_WRITES:
        *((uint8*)pValue) = maxNumPrepareWrites;
        break;

      default:
        status = INVALIDPARAMETER;
        break;
    }
  }
  return ( status );
}

//#ifdef ATT_DELAYED_REQ
/*********************************************************************
 * @fn      GATTServApp_ReadRsp
 *
 * @brief   If a service returns blePending to the read attribute call back
 *          invoked from GATTServApp, the service can later respond to with the
 *          value to be read using this API
 *
 * @param   connHandle - connection read request was received on
 * @param   pAttrValue - pointer to data read
 * @param   attrLen - length of data read
 * @param   attrHandle - attribute handle read
 *
 * @return  SUCCESS: Read was successfully added to response
 *          bleNotConnected: Connection associated with read req is down
 */
bStatus_t GATTServApp_ReadRsp( uint16 connHandle, uint8 *pAttrValue,
                               uint16 attrLen, uint16 attrHandle )
{
  bStatus_t status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATTServApp_ReadRsp, &invokeStatus, connHandle, pAttrValue, attrLen, attrHandle))
  {
    status = (bStatus_t)invokeStatus;
  }
  else
  {

  uint16 errHandle = attrHandle;
  uint16 rspOpcode = req.method + 1;
  status = bleInvalidRange;

  // Verify the link is still up, the last request is valid and the response
  // connection handle matches the last request connection handle
  if ( MAP_linkDB_State ( connHandle, LINK_CONNECTED ) == FALSE )
  {
    return bleNotConnected;
  }
  else if ( connHandle == req.connHandle )
  {
    switch ( req.method )
    {
      case ATT_READ_BY_TYPE_REQ:
        // Verify that the response attribute handle matches the attribute
        // handle to be read.
        if ( attrHandle == req.msg.readByTypeReq.startHandle )
        {
          // Add attribute value received from service to the Read By Type Response
          status = gattServApp_buildReadByTypeRsp( connHandle, pAttrValue,
                                                       attrLen, attrHandle );

          // Update start handle to find next attribute
          req.msg.readByTypeReq.startHandle++;

          // Process Updated Read By Type Request
          status = gattServApp_ProcessReadByTypeReq( &req, status, &errHandle );
        }
        break;

      default:
        break;
    }
  }

  // See if we need to send an error response back
  if ( ( status != SUCCESS ) &&
       ( status != blePending ) &&
       ( status != bleInvalidRange ) )
  {
    // Make sure the request was not sent locally
    if ( req.hdr.status != bleNotConnected )
    {
      // Free buffer allocated for response (if any)
      GATT_bm_free( (gattMsg_t *)&rsp, rspOpcode );

      attErrorRsp_t *pRsp = &rsp.errorRsp;

      pRsp->reqOpcode = req.method;
      pRsp->handle = errHandle;
      pRsp->errCode = status;

      if ( ATT_ErrorRsp( req.connHandle, pRsp ) != SUCCESS )
      {
        // Failed to send an error response
        rspOpcode = ATT_ERROR_RSP;

        status = blePending;
      }

      // Deallocate and clear stored request
      GATT_bm_free( (gattMsg_t *)&req.msg, req.method );
      VOID MAP_osal_memset( &req, 0, sizeof( gattMsgEvent_t ) );
    }
  }

  // See if we've failed to send a response back
  if ( status == blePending )
  {
    // Failed to send a response, enqueue it to retry later
    if ( MAP_gattServApp_EnqueueReTx( req.connHandle, rspOpcode,
                                      (gattMsg_t *)&rsp ) != SUCCESS )
    {
      // Free buffer just allocated for response (if any)
      GATT_bm_free( (gattMsg_t *)&rsp, rspOpcode );
    }

    // Deallocate and clear stored request
    GATT_bm_free( (gattMsg_t *)&req.msg, req.method );
    VOID MAP_osal_memset( &req, 0, sizeof( gattMsgEvent_t ) );
  }
  }
  return ( status );
}
//#endif // ATT_DELAYED_REQ

/*********************************************************************
 * @fn      gattServApp_SetNumPrepareWrites
 *
 * @brief   Set the maximum number of the prepare writes.
 *
 * @param   numPrepareWrites - number of prepare writes
 *
 * @return  SUCCESS: New number set successfully.
 *          bleMemAllocError: Memory allocation failed.
 */
bStatus_t gattServApp_SetNumPrepareWrites( uint8 numPrepareWrites )
{
  uint8 *pQueue;
  uint16 queueSize = (numPrepareWrites * sizeof( attPrepareWriteReq_t ));

  // First make sure no one can get access to the Prepare Write Table
  maxNumPrepareWrites = 0;

  // Free the existing prepare write queues
  if ( prepareWritesTbl[0].pPrepareWriteQ != NULL )
  {
    MAP_osal_mem_free( prepareWritesTbl[0].pPrepareWriteQ );

    // Null out the prepare writes queues
    for ( uint8 i = 0; i < linkDBNumConns; i++ )
    {
      prepareWritesTbl[i].pPrepareWriteQ = NULL;
    }
  }

  // Allocate the prepare write queues
  pQueue = MAP_osal_mem_alloc( (linkDBNumConns * queueSize) );
  if ( pQueue != NULL )
  {
    // Initialize the prepare write queues
    VOID MAP_osal_memset( pQueue, 0, (linkDBNumConns * queueSize) );

    // Set up the prepare write queue for each client (i.e., connection)
    for ( uint8 i = 0; i < linkDBNumConns; i++ )
    {
      prepareWrites_t *pClientQ = &(prepareWritesTbl[i]);

      pClientQ->pPrepareWriteQ = (attPrepareWriteReq_t *)pQueue;

      // Mark the prepare write request items as empty
      for ( uint8 j = 0; j < numPrepareWrites; j++ )
      {
        pClientQ->pPrepareWriteQ[j].handle = GATT_INVALID_HANDLE;
        pClientQ->pPrepareWriteQ[j].pValue = NULL;
      }

      pQueue += queueSize; // pointer of next available client queue
    }

    // Set the new number of prepare writes
    maxNumPrepareWrites = numPrepareWrites;

    return ( SUCCESS );
  }

  return ( bleMemAllocError );
}

/******************************************************************************
 * @fn      GATTServApp_AddService
 *
 * @brief   Add function for the GATT Service.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  SUCCESS: Service added successfully.
 *          INVALIDPARAMETER: Invalid service field.
 *          FAILURE: Not enough attribute handles available.
 *          bleMemAllocError: Memory allocation error occurred.
 */
bStatus_t GATTServApp_AddService( uint32 services )
{
  uint8 status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATTServApp_AddService,&invokeStatus, services))
  {
    status = (uint8)invokeStatus;
  }
  else {
  status = SUCCESS;

  if ( services & GATT_SERVICE )
  {
    if (!GATTServApp_gatt_no_service_changed) // #ifndef GATT_NO_SERVICE_CHANGED
    {
      // Register GATT attribute list and CBs with GATT Server Application
      status = GATTServApp_RegisterService( gattAttrTbl, GATT_NUM_ATTRS( gattAttrTbl ),
                                            GATT_MAX_ENCRYPT_KEY_SIZE,
                                            &gattServiceCBs );
    }
    else // #else // GATT_NO_SERVICE_CHANGED
    {
      // Register GATT attribute list and CBs with GATT Server Application
      status = GATTServApp_RegisterService( gattAttrTbl_gatt_no_service_changed, GATT_NUM_ATTRS( gattAttrTbl_gatt_no_service_changed ),
                                            GATT_MAX_ENCRYPT_KEY_SIZE,
                                            NULL );
    }
  } // #endif // GATT_NO_SERVICE_CHANGED
  }
  return ( status );
}

/******************************************************************************
 * @fn      GATTServApp_DelService
 *
 * @brief   Delete function for the GATT Service.
 *
 * @param   services - services to delete. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  SUCCESS: Service deleted successfully.
 *          FAILURE: Service not found.
 */
bStatus_t GATTServApp_DelService( uint32 services )
{
  uint8 status = SUCCESS;

  if ( services & GATT_SERVICE )
  {
    // Deregister GATT attribute list and CBs from GATT Server Application
    status = GATTServApp_DeregisterService( GATT_SERVICE_HANDLE( gattAttrTbl ), NULL );
  }

  return ( status );
}

/******************************************************************************
 * @fn      gattServApp_RegisterServiceCBs
 *
 * @brief   Register callback functions for a service.
 *
 * @param   handle - handle of service being registered
 * @param   pServiceCBs - pointer to service CBs to be registered
 *
 * @return  SUCCESS: Service CBs were registered successfully.
 *          INVALIDPARAMETER: Invalid service CB field.
 *          bleMemAllocError: Memory allocation error occurred.
 */
bStatus_t gattServApp_RegisterServiceCBs( uint16 handle,
                                          const gattServiceCBs_t *pServiceCBs )
{
  serviceCBsList_t *pNewItem;

  // Make sure the service handle is specified
  if ( handle == GATT_INVALID_HANDLE )
  {
    return ( INVALIDPARAMETER );
  }

  // Fill in the new service list
  pNewItem = (serviceCBsList_t *)MAP_osal_mem_alloc( sizeof( serviceCBsList_t ) );
  if ( pNewItem == NULL )
  {
    // Not enough memory
    return ( bleMemAllocError );
  }

  // Set up new service CBs item
  pNewItem->next = NULL;
  pNewItem->serviceInfo.handle = handle;
  pNewItem->serviceInfo.pCBs = pServiceCBs;

  // Find spot in list
  if ( serviceCBsList == NULL )
  {
    // First item in list
    serviceCBsList = pNewItem;
  }
  else
  {
    serviceCBsList_t *pLoop = serviceCBsList;

    // Look for end of list
    while ( pLoop->next != NULL )
    {
      pLoop = pLoop->next;
    }

    // Put new item at end of list
    pLoop->next = pNewItem;
  }

  return ( SUCCESS );
}

/******************************************************************************
 * @fn      gattServApp_DeregisterServiceCBs
 *
 * @brief   Deregister callback functions for a service.
 *
 * @param   handle - handle of service CBs to be deregistered
 *
 * @return  SUCCESS: Service CBs were deregistered successfully.
 *          FAILURE: Service CBs were not found.
 */
bStatus_t gattServApp_DeregisterServiceCBs( uint16 handle )
{
  serviceCBsList_t *pLoop = serviceCBsList;
  serviceCBsList_t *pPrev = NULL;

  // Look for service
  while ( pLoop != NULL )
  {
    if ( pLoop->serviceInfo.handle == handle )
    {
      // Service CBs found; unlink it
      if ( pPrev == NULL )
      {
        // First item in list
        serviceCBsList = pLoop->next;
      }
      else
      {
        pPrev->next = pLoop->next;
      }

      // Free the service CB record
      MAP_osal_mem_free( pLoop );

      return ( SUCCESS );
    }

    pPrev = pLoop;
    pLoop = pLoop->next;
  }

  // Service CBs not found
  return ( FAILURE );
}

/*********************************************************************
 * @fn      gattServApp_FindServiceCBs
 *
 * @brief   Find service's callback record.
 *
 * @param   handle - owner of service
 *
 * @return  Pointer to service record. NULL, otherwise.
 */
const gattServiceCBs_t *gattServApp_FindServiceCBs( uint16 handle )
{
  serviceCBsList_t *pLoop = serviceCBsList;

  while ( pLoop != NULL )
  {
    if ( pLoop->serviceInfo.handle == handle )
    {
      return ( pLoop->serviceInfo.pCBs );
    }

    // Try next service
    pLoop = pLoop->next;
  }

  return ( (gattServiceCBs_t *)NULL );
}

uint32_t _processCnt = 0;
uint32_t _processStatus = 0;
/*********************************************************************
 * @fn          gattServApp_ProcessMsg
 *
 * @brief       GATT Server App message processing function.
 *
 * @param       pMsg - pointer to received message
 *
 * @return      TRUE when it's safe for caller to dealloc packet. FALSE, otherwise.
 */
uint8 gattServApp_ProcessMsg( gattMsgEvent_t *pMsg )
{
  uint16 errHandle = GATT_INVALID_HANDLE;
  uint8 safeToDealloc = TRUE;
  uint8 rspOpcode;
  uint8 status;

#if defined ( TESTMODES )
  if ( gattservapp_paramValue == GATT_TESTMODE_NO_RSP )
  {
    // Notify GATT that a message has been processed
    // Note: This call is optional if flow control is not used.
    MAP_GATT_AppCompletedMsg( pMsg );

    // Just ignore the incoming request messages
    return ( safeToDealloc );
  }
#endif

  // Allocate buffer to process an incoming request - done only once
  if ( pAttrValue == NULL )
  {
    pAttrValue = MAP_osal_mem_alloc( MAP_L2CAP_GetMTU() );
    if ( pAttrValue == NULL )
    {
      // Couldn't get buffer
      return ( safeToDealloc );
    }
  }

  // Initialize response data
  VOID MAP_osal_memset( &rsp, 0 , sizeof( attMsg_t ) );
  rspOpcode = pMsg->method + 1;

  // Process the GATT server message
  switch ( pMsg->method )
  {
    case ATT_EXCHANGE_MTU_REQ:
      status = MAP_gattServApp_ProcessExchangeMTUReq( pMsg );
      break;

    case ATT_FIND_BY_TYPE_VALUE_REQ:
      status = MAP_gattServApp_ProcessFindByTypeValueReq( pMsg, &errHandle );
      break;

    case ATT_READ_BY_TYPE_REQ:
      status = MAP_gattServApp_ProcessReadByTypeReq( pMsg, SUCCESS, &errHandle );
      break;

    case ATT_READ_REQ:
      status = MAP_gattServApp_ProcessReadReq( pMsg, &errHandle );
      break;

    case ATT_READ_BLOB_REQ:
      status = MAP_gattServApp_ProcessReadBlobReq( pMsg, &errHandle );
      break;

    case ATT_READ_MULTI_REQ:
      status = MAP_gattServApp_ProcessReadMultiReq( pMsg, &errHandle );
      break;

    case ATT_READ_BY_GRP_TYPE_REQ:
      status = MAP_gattServApp_ProcessReadByGrpTypeReq( pMsg, &errHandle );
      break;

    case ATT_WRITE_REQ:
      status = MAP_gattServApp_ProcessWriteReq( pMsg, &errHandle, &safeToDealloc );
      break;

    case ATT_PREPARE_WRITE_REQ:
      status = MAP_gattServApp_ProcessPrepareWriteReq( pMsg, &errHandle, &safeToDealloc );
      break;

    case ATT_EXECUTE_WRITE_REQ:
      status = MAP_gattServApp_ProcessExecuteWriteReq( pMsg, &errHandle );
      break;

    default:
      // Unknown request - ignore it!
      status = SUCCESS;
      break;
  }


  // See if we need to send an error response back
  if ( ( status != SUCCESS ) && ( status != blePending ) )
  {
    // Make sure the request was not sent locally
    if ( pMsg->hdr.status != bleNotConnected )
    {
      attErrorRsp_t *pRsp = &rsp.errorRsp;

      pRsp->reqOpcode = pMsg->method;
      pRsp->handle = errHandle;
      pRsp->errCode = status;

      if ( MAP_ATT_ErrorRsp( pMsg->connHandle, pRsp ) != SUCCESS )
      {
        // Failed to send an error response
        rspOpcode = ATT_ERROR_RSP;

        status = blePending;
      }
    }
  }

  // See if we've failed to send a response back
  if ( status == blePending )
  {
    // Failed to send a response, enqueue it to retry later
    if ( MAP_gattServApp_EnqueueReTx( pMsg->connHandle, rspOpcode,
                                      (gattMsg_t *)&rsp ) != SUCCESS )
    {
      // Free buffer just allocated for response (if any)
      MAP_GATT_bm_free( (gattMsg_t *)&rsp, rspOpcode );
    }
  }

  // Notify GATT that a message has been processed
  // Note: This call is optional if flow control is not used.
  // Note: For ATT_DELAYED_REQ, the only delayed request is ATT_READ_BY_TYPE_REQ
  //       which has built in flow control based on the ATT Response. Thus
  //       L2CAP flow control is not used and this function can be called
  //       regardless of whether the request has been completely processed. If
  //       in the future more ATT Requests are supported/needed this may change
  MAP_GATT_AppCompletedMsg( pMsg );

  return ( safeToDealloc );
}

/*********************************************************************
 * @fn          gattServApp_ProcessExchangeMTUReq
 *
 * @brief       Process Exchange MTU Request.
 *
 * @param       pMsg - pointer to received message
 *
 * @return      Success
 */
bStatus_t gattServApp_ProcessExchangeMTUReq( gattMsgEvent_t *pMsg )
{
  attExchangeMTUReq_t *pReq = &pMsg->msg.exchangeMTUReq;
  attExchangeMTURsp_t *pRsp = &rsp.exchangeMTURsp;

  // ATT_MTU shall be set to the minimum of the Client Rx MTU and Server Rx MTU values

  // Set the Server Rx MTU parameter to the maximum MTU that this server can receive
#if defined ( TESTMODES )
  if ( gattservapp_paramValue == GATT_TESTMODE_MAX_MTU_SIZE )
  {
    pRsp->serverRxMTU = ATT_TEST_MODE_MAX_MTU_SIZE;
  }
  else
#endif
  {
    pRsp->serverRxMTU = MAP_L2CAP_GetMTU();
  }

  // Send response back
  if ( MAP_ATT_ExchangeMTURsp( pMsg->connHandle, pRsp ) != SUCCESS )
  {
    // Set ATT_MTU to the minimum of the Client Rx MTU and Server Rx MTU values

    // Pass the new MTU size up to app so that our negotiated MTU size gets
    // set correctly when the response is transmitted
    pRsp->serverRxMTU = Math_MIN( pReq->clientRxMTU, pRsp->serverRxMTU );

    return ( blePending );
  }

  // Set ATT_MTU to the minimum of the Client Rx MTU and Server Rx MTU values
  MAP_GATT_UpdateMTU( pMsg->connHandle, Math_MIN( pReq->clientRxMTU, pRsp->serverRxMTU ) );

  return ( SUCCESS );
}

/*********************************************************************
 * @fn          gattServApp_ProcessFindByTypeValueReq
 *
 * @brief       Process Find By Type Value Request.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessFindByTypeValueReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle )
{
  attFindByTypeValueReq_t *pReq = &pMsg->msg.findByTypeValueReq;
  attFindByTypeValueRsp_t *pRsp = &rsp.findByTypeValueRsp;
  uint16 service, mtuSize = MAP_ATT_GetMTU( pMsg->connHandle );
  gattAttribute_t *pAttr;
  uint16 infoLen = 0;

  // Only attributes with attribute handles between and including the Starting
  // Handle parameter and the Ending Handle parameter that match the requested
  // attribute type and the attribute value will be returned.

  // All attribute types are effectively compared as 128-bit UUIDs,
  // even if a 16-bit UUID is provided in this request or defined
  // for an attribute.
  pAttr = MAP_GATT_FindHandleUUID( pReq->startHandle, pReq->endHandle,
                                   pReq->type.uuid, pReq->type.len, &service );

  while ( ( pAttr != NULL ) && ( infoLen < (mtuSize-5) ) )
  {
    uint16 handle = GATT_INVALID_HANDLE, grpEndHandle;

    // It is not possible to use this request on an attribute that has a value
    // that is longer than (ATT_MTU - 7).
    if ( MAP_GATTServApp_ReadAttr( pMsg->connHandle, pAttr, service, pAttrValue,
                                   &attrLen, 0, (mtuSize-7), pMsg->method ) == SUCCESS )
    {
      // Attribute values should be compared in terms of length and binary representation.
      if ( ( pReq->len == attrLen ) && MAP_osal_memcmp( pReq->pValue, pAttrValue, attrLen) )
      {
        // New attribute found

        if ( pRsp->pHandlesInfo == NULL )
        {
          // Allocate space for handles information
          pRsp->pHandlesInfo = (uint8 *)MAP_GATT_bm_alloc( pMsg->connHandle,
                                                           ATT_FIND_BY_TYPE_VALUE_RSP,
                                                           GATT_MAX_MTU, NULL );
          if ( pRsp->pHandlesInfo == NULL )
          {
            // Couldn't get buffer for response
            *pErrHandle = pReq->startHandle;

            return ( ATT_ERR_INSUFFICIENT_RESOURCES );
          }
        }

        // Attribute found
        handle = pAttr->handle;
      }
    }

    // Try to find the next attribute
    pAttr = MAP_GATT_FindNextAttr( pAttr, pReq->endHandle, service, &grpEndHandle );

    // Set Group End Handle for Attribute Handle found
    if ( handle != GATT_INVALID_HANDLE )
    {
      // Set the Found Handle to the attribute that has the exact attribute
      // type and attribute value from the request.
      pRsp->pHandlesInfo[infoLen++] = LO_UINT16( handle );
      pRsp->pHandlesInfo[infoLen++] = HI_UINT16( handle );

      // If the attribute type is a grouping attribute, the Group End Handle
      // shall be defined by that higher layer specification. If the attribute
      // type is not a grouping attribute, the Group End Handle shall be equal
      // to the Found Attribute Handle.
      if ( pAttr != NULL )
      {
        pRsp->pHandlesInfo[infoLen++] = LO_UINT16( grpEndHandle );
        pRsp->pHandlesInfo[infoLen++] = HI_UINT16( grpEndHandle );
      }
      else
      {
        // If no other attributes with the same attribute type exist after the
        // Found Attribute Handle, the Group End Handle shall be set to 0xFFFF.
        pRsp->pHandlesInfo[infoLen++] = LO_UINT16( GATT_MAX_HANDLE );
        pRsp->pHandlesInfo[infoLen++] = HI_UINT16( GATT_MAX_HANDLE );
      }
    }
  } // while

  if ( infoLen > 0 )
  {
    pRsp->numInfo = infoLen / 4;

    // Send a response back
    if ( MAP_ATT_FindByTypeValueRsp( pMsg->connHandle, pRsp ) != SUCCESS )
    {
      // Failed to send response back
      return ( blePending );
    }

    return ( SUCCESS );
  }

  *pErrHandle = pReq->startHandle;

  return ( ATT_ERR_ATTR_NOT_FOUND );
}

/*********************************************************************
 * @fn          gattServApp_ProcessReadByTypeReq
 *
 * @brief       Process Read By Type Request.
 *
 * @param       pMsg - pointer to received message
 * @param       status - current status of read by type processing. Must be
 *                       SUCCESS if pMsg is not a previous request
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessReadByTypeReq( gattMsgEvent_t *pMsg,
                                            bStatus_t status,
                                            uint16 *pErrHandle )
{
  attReadByTypeReq_t *pReq;
  attReadByTypeRsp_t *pRsp = &rsp.readByTypeRsp;
  uint16 mtuSize = MAP_ATT_GetMTU( pMsg->connHandle );
  uint16 maxDataLen = mtuSize - sizeof(pRsp->len);

  if (GATTServApp_att_delayed_req) //#ifdef ATT_DELAYED_REQ
  {
    // If pMsg is a new reqeust, store incase request processing is delayed
    // Note: This shallow copy works because there are no pointer members of
    //       attReadByTypeReq_t. With pointer members must deep copy
    if ( pMsg != &req )
    {
      MAP_osal_memcpy( &req, pMsg, sizeof( gattMsgEvent_t ) );
    }

    // Update local request pointer to point to request message
    pReq = &req.msg.readByTypeReq;
  }
  else  // #else
  {
    // Update local request pointer to point to request message
    pReq = &pMsg->msg.readByTypeReq;
  }     // #endif // ATT_DELAYED_REQ

  // Only the attributes with attribute handles between and including the
  // Starting Handle and the Ending Handle with the attribute type that is
  // the same as the Attribute Type given will be returned.

  // Make sure there's enough room at least for an attribute handle (no value)
  while ( pRsp->dataLen <= ( maxDataLen - GATT_ATTR_HANDLE_SIZE ) &&
          status == SUCCESS )
  {
    uint16 service;
    gattAttribute_t *pAttr;

    // All attribute types are effectively compared as 128-bit UUIDs, even if
    // a 16-bit UUID is provided in this request or defined for an attribute.
    pAttr = MAP_GATT_FindHandleUUID( pReq->startHandle, pReq->endHandle, pReq->type.uuid,
                                     pReq->type.len, &service );
    if ( pAttr == NULL )
    {
      break; // No more attribute found
    }

    // Update start handle so it has the right value if we break from the loop
    pReq->startHandle = pAttr->handle;

    // Make sure the attribute has sufficient permissions to allow reading
    status = MAP_GATT_VerifyReadPermissions( pMsg->connHandle, pAttr, service );
    if ( status != SUCCESS )
    {
      break;
    }

    // Read the attribute value. If the attribute value is longer than
    // (ATT_MTU - 4) or 253 octets, whichever is smaller, then the first
    // (ATT_MTU - 4) or 253 octets shall be included in this response.
    status = MAP_GATTServApp_ReadAttr( pMsg->connHandle, pAttr, service, pAttrValue,
                                       &attrLen, 0,
                                       ( maxDataLen - GATT_ATTR_HANDLE_SIZE ),
                                       pMsg->method );

    if (GATTServApp_att_delayed_req) //#ifdef ATT_DELAYED_REQ
    {
      if ( status == blePending )
      {
        // Service unable to read attribute during call back.
        return ( SUCCESS );
      }
      else if ( status != SUCCESS )
      {
        break; // Cannot read the attribute value
      }
    }
    else // #else
    {
      if ( status != SUCCESS )

      {
        break; // Cannot read the attribute value
      }
    } // #endif // ATT_DELAYED_REQ

    // Add attribute value received from service to the Read By Type Response
    status = MAP_gattServApp_buildReadByTypeRsp( pMsg->connHandle, pAttrValue,
                                                 attrLen, pAttr->handle );
    if ( status != SUCCESS )
    {
      break;
    }

    if ( pReq->startHandle == GATT_MAX_HANDLE )
    {
      break; // We're done
    }

    // Update start handle and search again
    pReq->startHandle++;
  } // while

  // See what to respond
  if ( pRsp->dataLen > 0 )
  {
    // Set the number of attribute handle-value pairs found
    pRsp->numPairs = pRsp->dataLen / pRsp->len;

    if (GATTServApp_att_delayed_req) //#ifdef ATT_DELAYED_REQ
    {
      // Deallocate and clear stored request
      MAP_GATT_bm_free( (gattMsg_t *)&req.msg, req.method );
      VOID MAP_osal_memset( &req, 0, sizeof( gattMsgEvent_t ) );
    } // #endif // ATT_DELAYED_REQ


    // Send a response back
    if ( MAP_ATT_ReadByTypeRsp( pMsg->connHandle, pRsp ) != SUCCESS )
    {
      // Failed to send response back
      return ( blePending );
    }

    return ( SUCCESS );
  }

  if ( status == SUCCESS )
  {
    // Attribute not found -- dataLen must be 0
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  *pErrHandle = pReq->startHandle;

  return ( status );
}

/*********************************************************************
 * @fn          gattServApp_buildReadByTypeRsp
 *
 * @brief       Process attribute value received from service to build
 *              Read by Type Response message.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttrValue - pointer to data read
 * @param       attrLen - length of data read
 * @param       attrHandle - handle of attribute read
 *
 * @return      Success, Failure, or ATT_ERR_INSUFFICIENT_RESOURCES
 */
bStatus_t gattServApp_buildReadByTypeRsp( uint16 connHandle,
                                          uint8 *pAttrValue,
                                          uint16 attrLen,
                                          uint16 attrHandle )
{
  attReadByTypeRsp_t *pRsp = &rsp.readByTypeRsp;
  uint16 mtuSize = MAP_ATT_GetMTU( connHandle );

  // See if this is the first attribute found
  if ( pRsp->dataLen == 0 )
  {
    // Allocate space for handle and value pairs
    pRsp->pDataList = (uint8 *)MAP_GATT_bm_alloc( connHandle, ATT_READ_BY_TYPE_RSP,
                                                  GATT_MAX_MTU, NULL );
    if ( pRsp->pDataList == NULL )
    {
      return ( ATT_ERR_INSUFFICIENT_RESOURCES );
    }

    // Use the length of the first attribute value for the length field
    pRsp->len = GATT_ATTR_HANDLE_SIZE + attrLen;
  }
  else
  {
    // If the attributes have attribute values that have the same length
    // then these attributes can all be read in a single request.
    if ( pRsp->len != GATT_ATTR_HANDLE_SIZE + attrLen )
    {
      return ( FAILURE );
    }
  }

  // Make sure there's enough room for this attribute handle and value
  if ( ( pRsp->dataLen + attrLen ) >
         (mtuSize - GATT_ATTR_HANDLE_SIZE - sizeof(pRsp->len) ) )
  {
    return ( FAILURE );
  }

  // Add the handle value pair to the response
  pRsp->pDataList[pRsp->dataLen++] = LO_UINT16( attrHandle );
  pRsp->pDataList[pRsp->dataLen++] = HI_UINT16( attrHandle );

  VOID MAP_osal_memcpy( &(pRsp->pDataList[pRsp->dataLen]), pAttrValue, attrLen );
  pRsp->dataLen += attrLen;

  return ( SUCCESS );
}

/*********************************************************************
 * @fn          gattServApp_ProcessReadReq
 *
 * @brief       Process Read Request.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessReadReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle )
{
  attReadReq_t *pReq = &pMsg->msg.readReq;
  gattAttribute_t *pAttr;
  uint16 service;
  uint8 status;

  pAttr = MAP_GATT_FindHandle( pReq->handle, &service );
  if ( pAttr != NULL )
  {
    attReadRsp_t *pRsp = &rsp.readRsp;
    uint8 safeToDealloc = TRUE;
    uint16 len;

    // Allocate space for attribute value
    pRsp->pValue = (uint8 *)MAP_GATT_bm_alloc( pMsg->connHandle, ATT_READ_RSP,
                                               GATT_MAX_MTU, &len );
    if ( pRsp->pValue == NULL )
    {
      // Couldn't get buffer for response
      *pErrHandle = pReq->handle;

      return ( ATT_ERR_INSUFFICIENT_RESOURCES );
    }

    // Build and send a response back. If the attribute value is longer
    // than (ATT_MTU - 1) then (ATT_MTU - 1) octets shall be included
    // in this response.
    status = MAP_GATTServApp_ReadAttr( pMsg->connHandle, pAttr, service, pRsp->pValue,
                                       &pRsp->len, 0, len, pMsg->method );
    if ( status == SUCCESS )
    {
      // Send a response back
      if ( MAP_ATT_ReadRsp( pMsg->connHandle, pRsp ) !=  SUCCESS )
      {
        // Failed to send response back
        return ( blePending );
      }

      safeToDealloc = FALSE;
    }
    else if ( status == blePending )
    {
      // GATT_ServApp will send the response when it is able so consider this
      // transaction completed
      status = SUCCESS;
    }

    if ( safeToDealloc )
    {
      // Free buffer just allocated
      MAP_osal_bm_free( pRsp->pValue );
    }
  }
  else
  {
    status = ATT_ERR_INVALID_HANDLE;
  }

  if ( status != SUCCESS )
  {
    *pErrHandle = pReq->handle;
  }

  return ( status );
}

/*********************************************************************
 * @fn          gattServApp_ProcessReadBlobReq
 *
 * @brief       Process Read Blob Request.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessReadBlobReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle )
{
  attReadBlobReq_t *pReq = &pMsg->msg.readBlobReq;
  gattAttribute_t *pAttr;
  uint16 service;
  uint8 status;

  pAttr = MAP_GATT_FindHandle( pReq->handle, &service );
  if ( pAttr != NULL )
  {
    attReadBlobRsp_t *pRsp = &rsp.readBlobRsp;
    uint8 safeToDealloc = TRUE;
    uint16 len;

    // Allocate space for attribute value
    pRsp->pValue = (uint8 *)MAP_GATT_bm_alloc( pMsg->connHandle, ATT_READ_BLOB_RSP,
                                               GATT_MAX_MTU, &len );
    if ( pRsp->pValue == NULL )
    {
      // Couldn't get buffer for response
      *pErrHandle = pReq->handle;

      return ( ATT_ERR_INSUFFICIENT_RESOURCES );
    }

    // Read part attribute value. If the attribute value is longer than
    // (Value Offset + ATT_MTU - 1) then (ATT_MTU - 1) octets from Value
    // Offset shall be included in this response.
    status = MAP_GATTServApp_ReadAttr( pMsg->connHandle, pAttr, service,
                                       pRsp->pValue, &pRsp->len, pReq->offset,
                                       len, pMsg->method );
    if ( status == SUCCESS )
    {
      // Send a response back
      if ( MAP_ATT_ReadBlobRsp( pMsg->connHandle, pRsp ) != SUCCESS )
      {
        // Failed to send response back
        return ( blePending );
      }

      safeToDealloc = FALSE;
    }
    else if ( status == blePending )
    {
      // GATT_ServApp will send the response when it is able so consider this
      // transaction completed
      status = SUCCESS;
    }

    if ( safeToDealloc )
    {
      // Free buffer just allocated
      MAP_osal_bm_free( pRsp->pValue );
    }
  }
  else
  {
    status = ATT_ERR_INVALID_HANDLE;
  }

  if ( status != SUCCESS )
  {
    *pErrHandle = pReq->handle;
  }

  return ( status );
}

/*********************************************************************
 * @fn          gattServApp_ProcessReadMultiReq
 *
 * @brief       Process Read Multiple Request.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessReadMultiReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle )
{
  attReadMultiReq_t *pReq = &pMsg->msg.readMultiReq;
  attReadMultiRsp_t *pRsp = &rsp.readMultiRsp;
  uint16 i, mtuSize = MAP_ATT_GetMTU( pMsg->connHandle );
  uint8 status = SUCCESS;

  for ( i = 0; ( i < pReq->numHandles ) && ( pRsp->len < (mtuSize-1) ); i++ )
  {
    gattAttribute_t *pAttr;
    uint16 service;

    pAttr = MAP_GATT_FindHandle( ATT_HANDLE( pReq->pHandles, i ), &service );
    if ( pAttr == NULL )
    {
      // Should never get here!
      status = ATT_ERR_INVALID_HANDLE;

      // The handle of the first attribute causing the error
      *pErrHandle = ATT_HANDLE( pReq->pHandles, i );
      break;
    }

    // If the Set Of Values parameter is longer than (ATT_MTU - 1) then only
    // the first (ATT_MTU - 1) octets shall be included in this response.
    status = MAP_GATTServApp_ReadAttr( pMsg->connHandle, pAttr, service, pAttrValue,
                                       &attrLen, 0, (mtuSize-1), pMsg->method );
    if ( status != SUCCESS )
    {
      // The handle of the first attribute causing the error
      *pErrHandle = ATT_HANDLE( pReq->pHandles, i );
      break;
    }

    if ( pRsp->pValues == NULL )
    {
      // Allocate space for set of two or more attribute values
      pRsp->pValues = (uint8 *)MAP_GATT_bm_alloc( pMsg->connHandle, ATT_READ_MULTI_RSP,
                                                  GATT_MAX_MTU, NULL );
      if ( pRsp->pValues == NULL )
      {
        // Couldn't get buffer for response
        *pErrHandle = ATT_HANDLE( pReq->pHandles, i );

        return ( ATT_ERR_INSUFFICIENT_RESOURCES );
      }
    }

    // Make sure there's enough room in the response for this attribute value
    if ( pRsp->len + attrLen > (mtuSize-1) )
    {
      attrLen = (mtuSize-1) - pRsp->len;
    }

    // Append this value to the end of the response
    VOID MAP_osal_memcpy( &(pRsp->pValues[pRsp->len]), pAttrValue, attrLen );
    pRsp->len += attrLen;
  }

  if ( status == SUCCESS )
  {
    // Send a response back
    if ( MAP_ATT_ReadMultiRsp( pMsg->connHandle, pRsp ) !=  SUCCESS )
    {
      // Failed to send response back
      return ( blePending );
    }

    return ( SUCCESS );
  }

  if ( pRsp->pValues != NULL )
  {
    // Free buffer just allocated
    MAP_osal_bm_free( pRsp->pValues );
  }

  return ( status );
}

/*********************************************************************
 * @fn          gattServApp_ProcessReadByGrpTypeReq
 *
 * @brief       Process Read By Group Type Request.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessReadByGrpTypeReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle )
{
  attReadByGrpTypeReq_t *pReq = &pMsg->msg.readByGrpTypeReq;
  attReadByGrpTypeRsp_t *pRsp = &rsp.readByGrpTypeRsp;
  uint16 service, mtuSize = MAP_ATT_GetMTU( pMsg->connHandle );
  gattAttribute_t *pAttr;
  uint16 dataLen = 0;
  uint8 status = SUCCESS;

  // Only the attributes with attribute handles between and including the
  // Starting Handle and the Ending Handle with the attribute type that is
  // the same as the Attribute Type given will be returned.

  // All attribute types are effectively compared as 128-bit UUIDs,
  // even if a 16-bit UUID is provided in this request or defined
  // for an attribute.
  pAttr = MAP_GATT_FindHandleUUID( pReq->startHandle, pReq->endHandle,
                                   pReq->type.uuid, pReq->type.len, &service );
  while ( pAttr != NULL )
  {
    uint16 endGrpHandle;

    // The service, include and characteristic declarations are readable and
    // require no authentication or authorization, therefore insufficient
    // authentication or read not permitted errors shall not occur.
    status = MAP_GATT_VerifyReadPermissions( pMsg->connHandle, pAttr, service );
    if ( status != SUCCESS )
    {
      *pErrHandle = pAttr->handle;

      break;
    }

    // Read the attribute value. If the attribute value is longer than
    // (ATT_MTU - 6) or 251 octets, whichever is smaller, then the first
    // (ATT_MTU - 6) or 251 octets shall be included in this response.
    status = MAP_GATTServApp_ReadAttr( pMsg->connHandle, pAttr, service, pAttrValue,
                                       &attrLen, 0, (mtuSize-6), pMsg->method );
    if ( status != SUCCESS )
    {
      // Cannot read the attribute value
      *pErrHandle = pAttr->handle;

      break;
    }

    // See if this is the first attribute found
    if ( dataLen == 0 )
    {
      // Allocate space for handles information
      pRsp->pDataList = (uint8 *)MAP_GATT_bm_alloc( pMsg->connHandle, ATT_READ_BY_GRP_TYPE_RSP,
                                                    GATT_MAX_MTU, NULL );
      if ( pRsp->pDataList == NULL )
      {
        // Couldn't get buffer for response
        *pErrHandle = pAttr->handle;

        return ( ATT_ERR_INSUFFICIENT_RESOURCES );
      }

      // Use the length of the first attribute value for the length field
      pRsp->len = 2 + 2 + attrLen;
    }
    else
    {
      // If the attributes have attribute values that have the same length
      // then these attributes can all be read in a single request.
      if ( pRsp->len != 2 + 2 + attrLen )
      {
        break; // We're done here
      }

      // Make sure there's enough room for this attribute handle, end group handle and value
      if ( dataLen + attrLen > (mtuSize-6) )
      {
        break; // We're done here
      }
    }

    // Add Attribute Handle to the response
    pRsp->pDataList[dataLen++] = LO_UINT16( pAttr->handle );
    pRsp->pDataList[dataLen++] = HI_UINT16( pAttr->handle );

    // Try to find the next attribute
    pAttr = MAP_GATT_FindNextAttr( pAttr, pReq->endHandle, service, &endGrpHandle );

    // Add End Group Handle to the response
    if ( pAttr != NULL )
    {
      // The End Group Handle is the handle of the last attribute within the
      // service definition
      pRsp->pDataList[dataLen++] = LO_UINT16( endGrpHandle );
      pRsp->pDataList[dataLen++] = HI_UINT16( endGrpHandle );
    }
    else
    {
      // The ending handle of the last service can be 0xFFFF
      pRsp->pDataList[dataLen++] = LO_UINT16( GATT_MAX_HANDLE );
      pRsp->pDataList[dataLen++] = HI_UINT16( GATT_MAX_HANDLE );
    }

    // Add Attribute Value to the response
    VOID MAP_osal_memcpy( &(pRsp->pDataList[dataLen]), pAttrValue, attrLen );
    dataLen += attrLen;
  } // while

  // See what to respond
  if ( dataLen > 0 )
  {
    // Set the number of attribute handle, end group handle and value sets found
    pRsp->numGrps = dataLen / pRsp->len;

    // Send a response back
    if ( MAP_ATT_ReadByGrpTypeRsp( pMsg->connHandle, pRsp ) != SUCCESS )
    {
      // Failed to send response back
      return ( blePending );
    }

    return ( SUCCESS );
  }

  if ( status == SUCCESS )
  {
    // No grouping attribute found -- dataLen must be 0
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  *pErrHandle = pReq->startHandle;

  return ( status );
}



uint32_t _g_test = 0;
bStatus_t g_status;
/*********************************************************************
 * @fn          gattServApp_ProcessWriteReq
 *
 * @brief       Process Write Request or Command.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 * @param       pSafeToDealloc - whether it's safe for caller to dealloc packet
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessWriteReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle,
                                       uint8 *pSafeToDealloc )
{
  attWriteReq_t *pReq;
  gattAttribute_t *pAttr;
  uint16 service;
  uint8 status = SUCCESS;

  if ( ( pMsg == NULL ) || ( pErrHandle == NULL ) || ( pSafeToDealloc == NULL ) )
  {
	  return FAILURE;
  }

  pReq = &(pMsg->msg.writeReq);

  // Check if authenticated write shall be used.
  if ( pReq->sig != ATT_SIG_NOT_INCLUDED )
  {
	  // Must be GATT Signed Write Without Response.
	  // Make sure we're allowed to execute authenticated write.
	  if ( ( pReq->sig == ATT_SIG_INVALID ) && ( pReq->cmd == TRUE ) )
	  {
	     status = ATT_ERR_WRITE_NOT_PERMITTED;
	  }
  }
  // GPIO_write(24, 0);
  // No Error Response or Write Response shall be sent in response to Write
  // Command. If the server cannot write this attribute for any reason the
  // command shall be ignored.
  pAttr = MAP_GATT_FindHandle( pReq->handle, &service );
  // GPIO_write(24, 1);
  if ( ( pAttr != NULL ) && ( status == SUCCESS ) )
  {
    // Authorization is handled by the application/profile
    if ( gattPermitAuthorWrite( pAttr->permissions ) )
    {
      // Use Service's authorization callback to authorize the request
      pfnGATTAuthorizeAttrCB_t pfnCB = MAP_gattServApp_FindAuthorizeAttrCB( service );
      if ( pfnCB != NULL )
      {
          status = (*pfnCB)( pMsg->connHandle, pAttr, ATT_WRITE_REQ );
      }
      else
      {
        status = ATT_ERR_UNLIKELY;
      }
    }
    

    // If everything is fine then try to write the new value
    if ( status == SUCCESS )
    {
      // Use Service's write callback to write the request
      status = MAP_GATTServApp_WriteAttr( pMsg->connHandle, pReq->handle,
                                          pReq->pValue, pReq->len, 0,
                                          (pReq->cmd ? ATT_WRITE_CMD : pMsg->method) );
      if ( status == SUCCESS )
      {
        // Make sure this is not a Write Command before sending a response back
        if ( pReq->cmd == FALSE )
        {
          // Send a response back
          if ( MAP_ATT_WriteRsp( pMsg->connHandle ) != SUCCESS )
          {
            // Failed to send response back
            return ( blePending );
          }
        }
      }
      else if ( status == blePending )
      {
        // GATT_ServApp will send the response when it is able so consider this
        // transaction completed
        status = SUCCESS;
        *pSafeToDealloc = FALSE; // payload pass to app
      }
    }
  }
  else if ( status == SUCCESS )
  {
    
    status = ATT_ERR_INVALID_HANDLE;
  }

  if ( status != SUCCESS )
  {
    _g_test++;
    *pErrHandle = pReq->handle;
  }

  return ( pReq->cmd ? SUCCESS : status );
}

/*********************************************************************
 * @fn          gattServApp_ProcessPrepareWriteReq
 *
 * @brief       Process Prepare Write Request.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 * @param       pSafeToDealloc - safe for a caller to deallocate
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessPrepareWriteReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle,
                                              uint8 *pSafeToDealloc)
{
  attPrepareWriteReq_t *pReq = &pMsg->msg.prepareWriteReq;
  gattAttribute_t *pAttr;
  uint16 service;
  uint8 status = SUCCESS;

  // Verify that the connection was not terminated prior to processing pMsg
  if ( MAP_linkDB_State( pMsg->connHandle, LINK_CONNECTED ) == FALSE )
  {
    pMsg->hdr.status = bleNotConnected;
    return ATT_ERR_UNLIKELY;
  }

  pAttr = MAP_GATT_FindHandle( pReq->handle, &service );
  if ( pAttr != NULL )
  {
    // Authorization is handled by the application/profile
    if ( gattPermitAuthorWrite( pAttr->permissions ) )
    {
      // Use Service's authorization callback to authorize the request
      pfnGATTAuthorizeAttrCB_t pfnCB = MAP_gattServApp_FindAuthorizeAttrCB( service );
      if ( pfnCB != NULL )
      {
          status = (*pfnCB)( pMsg->connHandle, pAttr, ATT_WRITE_REQ );
      }
      else
      {
        status = ATT_ERR_UNLIKELY;
      }
    }

    if ( status == SUCCESS )
    {
#if defined ( TESTMODES )
      if ( gattservapp_paramValue == GATT_TESTMODE_CORRUPT_PW_DATA )
      {
        pReq->pValue[0] = ~(pReq->pValue[0]);
      }
#endif
      // Enqueue the request for now
      status = MAP_gattServApp_EnqueuePrepareWriteReq( pMsg->connHandle, pReq );
      if ( status == SUCCESS )
      {
        // pReq enqueued, notify caller not to deallocate
        *pSafeToDealloc = FALSE;

        attPrepareWriteRsp_t *pRsp = &rsp.prepareWriteRsp;

        // Echo the received request back
        VOID MAP_osal_memcpy( pRsp, pReq, sizeof( attPrepareWriteRsp_t ) );

        pRsp->pValue = (uint8 *)MAP_GATT_bm_alloc( pMsg->connHandle, ATT_PREPARE_WRITE_RSP,
                                                   pReq->len, NULL );
        if ( pRsp->pValue != NULL )
        {
          // Copy over the attribute value
          VOID MAP_osal_memcpy( pRsp->pValue, pReq->pValue, pReq->len );

          // Send a response back
          if ( MAP_ATT_PrepareWriteRsp( pMsg->connHandle, pRsp ) != SUCCESS )
          {
            // Failed to send response back
            return ( blePending );
          }
        }
        else
        {
          status = ATT_ERR_INSUFFICIENT_RESOURCES;
        }
      }
    }
  }
  else
  {
    status = ATT_ERR_INVALID_HANDLE;
  }

  if ( status != SUCCESS )
  {
    *pErrHandle = pReq->handle;
  }

  return ( status );
}

/*********************************************************************
 * @fn          gattServApp_ProcessExecuteWriteReq
 *
 * @brief       Process Execute Write Request.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessExecuteWriteReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle )
{
  attExecuteWriteReq_t *pReq = &pMsg->msg.executeWriteReq;
  prepareWrites_t *pQueue;
  uint8 status = SUCCESS;

  // See if this client has a prepare write queue
  pQueue = MAP_gattServApp_FindPrepareWriteQ( pMsg->connHandle );
  if ( pQueue != NULL )
  {
    // See if we're asked to write the values
    if ( pReq->flags == ATT_WRITE_PREPARED_VALUES )
    {
      if ( MAP_gattServApp_IsWriteLong( pReq, pQueue ) )
      {
        // Process the Write Long operation
        status = MAP_gattServApp_ProcessWriteLong( pMsg, pQueue, pErrHandle );
      }
      else
      {
        // Process the Reliable Writes operation
        status = MAP_gattServApp_ProcessReliableWrites( pMsg, pQueue, pErrHandle );
      }
    }
    else // ATT_CANCEL_PREPARED_WRITES
    {
      // Cancel all prepared writes - just ignore the Prepare Write Requests
    }

    // Clear the queue for this client
    MAP_gattServApp_ClearPrepareWriteQ( pQueue );
  }

  if ( status == SUCCESS )
  {
    // Send a response back
    if ( MAP_ATT_ExecuteWriteRsp( pMsg->connHandle ) != SUCCESS )
    {
      // Failed to send response back
      return ( blePending );
    }
  }
  else if ( status == blePending )
  {
    // GATT_ServApp will send the response when it is able so consider this
    // transaction completed
    status = SUCCESS;
  }

  return ( status );
}

/*********************************************************************
 * @fn          gattServApp_IsWriteLong
 *
 * @brief       Check for a Write Long operation.
 *
 * @param       pMsg - pointer to received message
 * @param       pQueue - pointer to client prepare write queue
 *
 * @return      TRUE or FALSE
 */
uint8 gattServApp_IsWriteLong( attExecuteWriteReq_t *pReq, prepareWrites_t *pQueue )
{
  uint8 *pLongValue, numReqs;
  uint16 nextOffset, handle = GATT_INVALID_HANDLE, totalLen = 0;

  // Check for a Write Long operation
  for ( numReqs = 0; numReqs < maxNumPrepareWrites; numReqs++ )
  {
    attPrepareWriteReq_t *pWriteReq = &(pQueue->pPrepareWriteQ[numReqs]);

    // See if there're any prepared write requests in the queue
    if ( pWriteReq->handle == GATT_INVALID_HANDLE )
    {
      break; // We're done
    }

    if ( handle == GATT_INVALID_HANDLE )
    {
      // Remember attribute info of the first prepare write request
      handle = pWriteReq->handle;
      totalLen = pWriteReq->len;
      nextOffset = pWriteReq->offset + totalLen;
    }
    else if ( ( handle == pWriteReq->handle ) && ( nextOffset == pWriteReq->offset ) )
    {
      // Same attribute handles and continuous value offsets
      totalLen += pWriteReq->len;
      nextOffset += pWriteReq->len;
    }
    else
    {
      // Different attribute handles or non-continuous value offsets
      return ( FALSE );
    }
  } // for loop

  if ( numReqs > 1 )
  {
    uint16 offset = 0; // offset for single value

    // Allocate buffer to concatenate all parts of attribute value
    if ( ( totalLen == 0 ) || ( ( pLongValue = MAP_osal_bm_alloc( totalLen ) ) == NULL ) )
    {
      // Each Prepare Write Request must be executed separately
      return ( FALSE );
    }

    // Concatenate all parts of attribute value into a single one
    for ( uint8 i = 0; i < numReqs; i++ )
    {
      attPrepareWriteReq_t *pWriteReq = &(pQueue->pPrepareWriteQ[i]);

      // Concatenate this part of attribute value to long value
      if ( pWriteReq->pValue != NULL )
      {
        VOID MAP_osal_memcpy( &pLongValue[offset], pWriteReq->pValue, pWriteReq->len );

        // Update offset into single value
        offset += pWriteReq->len;

        // Free buffer payload, not needed any more
        MAP_osal_bm_free( pWriteReq->pValue );
      }

      if ( i > 0 )
      {
        // Keep only the info of the first request
        MAP_osal_memset( pWriteReq, 0, sizeof( attPrepareWriteReq_t ) );
      }
    }

    // It's a Long Write operation. Set the attribute value and its total length
    pQueue->pPrepareWriteQ[0].pValue = pLongValue;
    pQueue->pPrepareWriteQ[0].len = totalLen;
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn          gattServApp_ProcessWriteLong
 *
 * @brief       Process a Write Long operation.
 *
 * @param       pMsg - pointer to received message
 * @param       pQueue - pointer to client prepare write queue
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessWriteLong( gattMsgEvent_t *pMsg,
                                        prepareWrites_t *pQueue,
                                        uint16 *pErrHandle )
{
  attPrepareWriteReq_t *pWriteReq = &(pQueue->pPrepareWriteQ[0]);
  uint8 status;

  // Prepare the request
  status = MAP_GATTServApp_WriteAttr( pMsg->connHandle, pWriteReq->handle,
                                      pWriteReq->pValue, pWriteReq->len,
                                      pWriteReq->offset, ATT_EXECUTE_WRITE_REQ );
  // If the prepare write requests can not be written, the queue shall
  // be cleared and then an Error Response shall be sent with a high
  // layer defined error code.
  if ( status == blePending )
  {
    pWriteReq->pValue = NULL; // payload passed to app
  }
  else if ( status != SUCCESS )
  {
    // The Attribute Handle in Error shall be set to the attribute handle
    // of the attribute from the prepare write queue that caused this
    // application error
    *pErrHandle = pWriteReq->handle;
  }

  return ( status );
}

/*********************************************************************
 * @fn          gattServApp_ProcessReliableWrites
 *
 * @brief       Process a Reliable Writes operation.
 *
 * @param       pMsg - pointer to received message
 * @param       pQueue - pointer to client prepare write queue
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessReliableWrites( gattMsgEvent_t *pMsg,
                                             prepareWrites_t *pQueue,
                                             uint16 *pErrHandle )
{
  uint8 status = SUCCESS;

  for ( uint8 i = 0; i < maxNumPrepareWrites; i++ )
  {
    uint8 method;
    attPrepareWriteReq_t *pWriteReq = &(pQueue->pPrepareWriteQ[i]);

    // See if there're any prepared write requests in the queue
    if ( pWriteReq->handle == GATT_INVALID_HANDLE )
    {
      break; // We're done
    }

    // See if this is the last Prepare Write Request
    if ( ( (i+1) == maxNumPrepareWrites ) ||
         ( pQueue->pPrepareWriteQ[i+1].handle == GATT_INVALID_HANDLE ) )
    {
      method = ATT_EXECUTE_WRITE_REQ;
    }
    else
    {
      method = ATT_PREPARE_WRITE_REQ;
    }

    status = MAP_GATTServApp_WriteAttr( pMsg->connHandle, pWriteReq->handle,
                                        pWriteReq->pValue, pWriteReq->len,
                                        pWriteReq->offset, method );
    // If the prepare write requests can not be written, the queue shall
    // be cleared and then an Error Response shall be sent with a high
    // layer defined error code.
    if ( status == blePending )
    {
      pWriteReq->pValue = NULL; // payload passed to app
    }
    else if ( status != SUCCESS )
    {
      // Cancel the remaining prepared writes

      // The Attribute Handle in Error shall be set to the attribute handle
      // of the attribute from the prepare write queue that caused this
      // application error
      *pErrHandle = pWriteReq->handle;
      break;
    }
  } // for loop

  return ( status );
}

/*********************************************************************
 * @fn          gattServApp_EnqueuePrepareWriteReq
 *
 * @brief       Enqueue Prepare Write Request.
 *
 * @param       connHandle - connection packet was received on
 * @param       pReq - pointer to request
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_EnqueuePrepareWriteReq( uint16 connHandle, attPrepareWriteReq_t *pReq )
{
  prepareWrites_t *pQueue;

  // See if there's queue already assocaited with this client
  pQueue = MAP_gattServApp_FindPrepareWriteQ( connHandle );
  if ( pQueue == NULL )
  {
    // Find a queue for this client
    pQueue = MAP_gattServApp_FindPrepareWriteQ( LINKDB_CONNHANDLE_INVALID );
    if ( pQueue != NULL )
    {
      pQueue->connHandle = connHandle;
    }
  }

  // If a queue is found for this client then enqueue the request
  if ( pQueue != NULL )
  {
    for ( uint8 i = 0; i < maxNumPrepareWrites; i++ )
    {
      attPrepareWriteReq_t *pWriteReq = &(pQueue->pPrepareWriteQ[i]);

      // See if this prepare write request is empty
      if ( pWriteReq->handle == GATT_INVALID_HANDLE )
      {
        // Store the request info
        VOID MAP_osal_memcpy( pWriteReq, pReq, sizeof( attPrepareWriteReq_t ) );

        return ( SUCCESS );
      }
    }
  }

  return ( ATT_ERR_PREPARE_QUEUE_FULL );
}

/*********************************************************************
 * @fn          gattServApp_FindPrepareWriteQ
 *
 * @brief       Find client's queue.
 *
 * @param       connHandle - connection used by client
 *
 * @return      Pointer to queue. NULL, otherwise.
 */
prepareWrites_t *gattServApp_FindPrepareWriteQ( uint16 connHandle )
{
  // First see if this client has already a queue
  for ( uint8 i = 0; i < linkDBNumConns; i++ )
  {
    if ( prepareWritesTbl[i].connHandle == connHandle )
    {
      // Queue found
      return ( &(prepareWritesTbl[i]) );
    }
  }

  return ( (prepareWrites_t *)NULL );
}

/*********************************************************************
 * @fn          gattServApp_PrepareWriteQInUse
 *
 * @brief       Check to see if the prepare write queue is in use.
 *
 * @param       void
 *
 * @return      TRUE if queue in use. FALSE, otherwise.
 */
uint8 gattServApp_PrepareWriteQInUse( void )
{
  // See if any prepare write queue is in use
  for ( uint8 i = 0; i < linkDBNumConns; i++ )
  {
    if ( prepareWritesTbl[i].connHandle != LINKDB_CONNHANDLE_INVALID )
    {
      for ( uint8 j = 0; j < maxNumPrepareWrites; j++ )
      {
        if ( prepareWritesTbl[i].pPrepareWriteQ[j].handle != GATT_INVALID_HANDLE )
        {
          // Queue item is in use
          return ( TRUE );
        }
      } // for
    }
  } // for

  return ( FALSE );
}

/*********************************************************************
 * @fn          gattServApp_ClearPrepareWriteQ
 *
 * @brief       Clear the prepare write queue for a GATT Client.
 *
 * @param       pQueue - pointer to client's queue
 *
 * @return      none.
 */
void gattServApp_ClearPrepareWriteQ( prepareWrites_t *pQueue )
{
  // Clear the queue for this client
  for ( uint8 i = 0; i < maxNumPrepareWrites; i++ )
  {
    attPrepareWriteReq_t *pWriteReq = &(pQueue->pPrepareWriteQ[i]);

    // See if there're any prepared write requests in the queue
    if ( pWriteReq->handle == GATT_INVALID_HANDLE )
    {
      break; // We're done
    }

    if ( pWriteReq->pValue != NULL )
    {
      // Free the payload
      VOID MAP_osal_bm_free( pWriteReq->pValue );
    }

    // Clear the queue item
    VOID MAP_osal_memset( pWriteReq, 0, sizeof( attPrepareWriteReq_t ) );

    // Mark this item as empty
    pWriteReq->handle = GATT_INVALID_HANDLE;
  } // for loop

  // Mark this queue as empty
  pQueue->connHandle = LINKDB_CONNHANDLE_INVALID;
}

/*********************************************************************
 * @fn      gattServApp_FindReadAttrCB
 *
 * @brief   Find the Read Attribute CB function pointer for a given service.
 *
 * @param   handle - service attribute handle
 *
 * @return  pointer to the found CB. NULL, otherwise.
 */
pfnGATTReadAttrCB_t gattServApp_FindReadAttrCB( uint16 handle )
{
  const gattServiceCBs_t *pCBs = MAP_gattServApp_FindServiceCBs( handle );

  return ( ( pCBs == NULL ) ? NULL : pCBs->pfnReadAttrCB );
}

/*********************************************************************
 * @fn      gattServApp_FindWriteAttrCB
 *
 * @brief   Find the Write CB Attribute function pointer for a given service.
 *
 * @param   handle - service attribute handle
 *
 * @return  pointer to the found CB. NULL, otherwise.
 */
pfnGATTWriteAttrCB_t gattServApp_FindWriteAttrCB( uint16 handle )
{
  const gattServiceCBs_t *pCBs = MAP_gattServApp_FindServiceCBs( handle );

  return ( ( pCBs == NULL ) ? NULL : pCBs->pfnWriteAttrCB );
}

/*********************************************************************
 * @fn      gattServApp_FindAuthorizeAttrCB
 *
 * @brief   Find the Authorize Attribute CB function pointer for a given service.
 *
 * @param   handle - service attribute handle
 *
 * @return  pointer to the found CB. NULL, otherwise.
 */
pfnGATTAuthorizeAttrCB_t gattServApp_FindAuthorizeAttrCB( uint16 handle )
{
  const gattServiceCBs_t *pCBs = MAP_gattServApp_FindServiceCBs( handle );

  return ( ( pCBs == NULL ) ? NULL : pCBs->pfnAuthorizeAttrCB );
}

//#ifndef GATT_NO_SERVICE_CHANGED
/*********************************************************************
 * @fn      gattServApp_ValidateWriteAttrCB
 *
 * @brief   Validate and/or Write attribute data
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  Success or Failure
 */
bStatus_t gattServApp_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                   uint8 *pValue, uint16 len, uint16 offset,
                                   uint8 method )
{
  bStatus_t status = SUCCESS;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_INDICATE );
        break;

      default:
        // Should never get here!
        status = ATT_ERR_INVALID_HANDLE;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}
//#endif // GATT_NO_SERVICE_CHANGED

/*********************************************************************
 * @fn          GATTServApp_ReadAttr
 *
 * @brief       Read an attribute. If the format of the attribute value
 *              is unknown to GATT Server, use the callback function
 *              provided by the Service.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       service - handle of owner service
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read
 *
 * @return      Success or Failure
 */
uint8 GATTServApp_ReadAttr( uint16 connHandle, gattAttribute_t *pAttr,
                            uint16 service, uint8 *pValue, uint16 *pLen,
                            uint16 offset, uint16 maxLen, uint8 method )
{
  uint8 useCB = FALSE;
  bStatus_t status = SUCCESS;

  // Authorization is handled by the application/profile
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Use Service's authorization callback to authorize the request
    pfnGATTAuthorizeAttrCB_t pfnCB = MAP_gattServApp_FindAuthorizeAttrCB( service );
    if ( pfnCB != NULL )
    {
      status = (*pfnCB)( connHandle, pAttr, ATT_READ_REQ );
    }
    else
    {
      status = ATT_ERR_UNLIKELY;
    }

    if ( status != SUCCESS )
    {
      // Read operation failed!
      return ( status );
    }
  }

  // Check the UUID length
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case GATT_PRIMARY_SERVICE_UUID:
      case GATT_SECONDARY_SERVICE_UUID:
        // Make sure it's not a blob operation
        if ( offset == 0 )
        {
          gattAttrType_t *pType = (gattAttrType_t *)(pAttr->pValue);

          *pLen = pType->len;
          VOID MAP_osal_memcpy( pValue, pType->uuid, pType->len );
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        break;

      case GATT_CHARACTER_UUID:
        // Make sure it's not a blob operation
        if ( offset == 0 )
        {
          gattAttribute_t *pCharValue;

          // The Attribute Value of a Characteristic Declaration includes the
          // Characteristic Properties, Characteristic Value Attribute Handle
          // and UUID.
          *pLen = 1;
          pValue[0] = *pAttr->pValue; // Properties

          // The Characteristic Value Attribute exists immediately following
          // the Characteristic Declaration.
          pCharValue = GATT_FindHandle( pAttr->handle+1, NULL );
          if ( pCharValue != NULL )
          {
            // It can be a 128-bit UUID
            *pLen += (2 + pCharValue->type.len);

            // Attribute Handle
            pValue[1] = LO_UINT16( pCharValue->handle );
            pValue[2] = HI_UINT16( pCharValue->handle );

            // Attribute UUID
            VOID MAP_osal_memcpy( &(pValue[3]), pCharValue->type.uuid, pCharValue->type.len );
          }
          else
          {
            // Should never get here!
            *pLen += (2 + ATT_BT_UUID_SIZE);

            // Set both Attribute Handle and UUID to 0
            VOID MAP_osal_memset( &(pValue[1]), 0, (2 + ATT_BT_UUID_SIZE) );
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        break;

      case GATT_INCLUDE_UUID:
        // Make sure it's not a blob operation
        if ( offset == 0 )
        {
          uint16 servHandle;
          uint16 endGrpHandle;
          gattAttribute_t *pIncluded;
          uint16 handle = *((uint16 *)(pAttr->pValue));

          // The Attribute Value of an Include Declaration is set the
          // included service Attribute Handle, the End Group Handle,
          // and the service UUID. The Service UUID shall only be present
          // when the UUID is a 16-bit Bluetooth UUID.
          *pLen = 4;
          pValue[0] = LO_UINT16( handle );
          pValue[1] = HI_UINT16( handle );

          // Find the included service attribute record
          pIncluded = GATT_FindHandle( handle, &servHandle );
          if ( pIncluded != NULL )
          {
            gattAttrType_t *pServiceUUID = (gattAttrType_t *)pIncluded->pValue;

            // Find out the End Group handle
            if ( ( GATT_FindNextAttr( pIncluded, GATT_MAX_HANDLE,
                                      servHandle, &endGrpHandle ) == NULL ) &&
                 ( !gattSecondaryServiceType( pIncluded->type ) ) )
            {
              // The ending handle of the last service can be 0xFFFF
              endGrpHandle = GATT_MAX_HANDLE;
            }

            // Include only 16-bit Service UUID
            if ( pServiceUUID->len == ATT_BT_UUID_SIZE )
            {
              VOID MAP_osal_memcpy( &(pValue[4]), pServiceUUID->uuid, ATT_BT_UUID_SIZE );
              *pLen += ATT_BT_UUID_SIZE;
            }
          }
          else
          {
            // Should never get here!
            endGrpHandle = handle;
          }

          // End Group Handle
          pValue[2] = LO_UINT16( endGrpHandle );
          pValue[3] = HI_UINT16( endGrpHandle );
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        // Make sure it's not a blob operation
        if ( offset == 0 )
        {
          uint16 value = MAP_GATTServApp_ReadCharCfg( connHandle,
                                                      GATT_CCC_TBL(pAttr->pValue) );
          *pLen = 2;
          pValue[0] = LO_UINT16( value );
          pValue[1] = HI_UINT16( value );
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        break;

      case GATT_CHAR_EXT_PROPS_UUID:
      case GATT_SERV_CHAR_CFG_UUID:
        // Make sure it's not a blob operation
        if ( offset == 0 )
        {
          uint16 value = *((uint16 *)(pAttr->pValue));

          *pLen = 2;
          pValue[0] = LO_UINT16( value );
          pValue[1] = HI_UINT16( value );
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        break;

      case GATT_CHAR_USER_DESC_UUID:
        {
          uint16 len = MAP_osal_strlen( (char *)(pAttr->pValue) ); // Could be a long attribute

          // If the value offset of the Read Blob Request is greater than the
          // length of the attribute value, an Error Response shall be sent with
          // the error code Invalid Offset.
          if ( offset <= len )
          {
            // If the value offset is equal than the length of the attribute
            // value, then the length of the part attribute value shall be zero.
            if ( offset == len )
            {
              len = 0;
            }
            else
            {
              // If the attribute value is longer than (Value Offset + maxLen)
              // then maxLen octets from Value Offset shall be included in
              // this response.
              if ( len > ( offset + maxLen ) )
              {
                len = maxLen;
              }
              else
              {
                len -= offset;
              }
            }

            *pLen = len;
            VOID MAP_osal_memcpy( pValue, &(pAttr->pValue[offset]), len );
          }
          else
          {
            status = ATT_ERR_INVALID_OFFSET;
          }
        }
        break;

      case GATT_CHAR_FORMAT_UUID:
        // Make sure it's not a blob operation
        if ( offset == 0 )
        {
          gattCharFormat_t *pFormat = (gattCharFormat_t *)(pAttr->pValue);

          *pLen = 7;
          pValue[0] = pFormat->format;
          pValue[1] = pFormat->exponent;
          pValue[2] = LO_UINT16( pFormat->unit );
          pValue[3] = HI_UINT16( pFormat->unit );
          pValue[4] = pFormat->nameSpace;
          pValue[5] = LO_UINT16( pFormat->desc );
          pValue[6] = HI_UINT16( pFormat->desc );
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        break;

      default:
        useCB = TRUE;
        break;
    }
  }
  else
  {
    useCB = TRUE;
  }

  if ( useCB == TRUE )
  {
    // Use Service's read callback to process the request
    pfnGATTReadAttrCB_t pfnCB = MAP_gattServApp_FindReadAttrCB( service );
    if ( pfnCB != NULL )
    {
      // Read the attribute value
      status = (*pfnCB)( connHandle, pAttr, pValue, pLen, offset, maxLen, method );
    }
    else
    {
      status = ATT_ERR_UNLIKELY;
    }
  }

  return ( status );
}

/*********************************************************************
 * @fn      GATTServApp_WriteAttr
 *
 * @brief   Write attribute data
 *
 * @param   connHandle - connection message was received on
 * @param   handle - attribute handle
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
uint8 GATTServApp_WriteAttr( uint16 connHandle, uint16 handle,
                             uint8 *pValue, uint16 len, uint16 offset,
                             uint8 method )
{
  uint16 service;
  gattAttribute_t *pAttr;
  bStatus_t status;

  // Find the owner of the attribute
  pAttr = GATT_FindHandle( handle, &service );
  if ( pAttr != NULL )
  {
    // Find out the owner's callback functions
    pfnGATTWriteAttrCB_t pfnCB = MAP_gattServApp_FindWriteAttrCB( service );
    if ( pfnCB != NULL )
    {
      // Try to write the new value
      status = (*pfnCB)( connHandle, pAttr, pValue, len, offset, method );

      // If Client Characteristic Configuration is being updated by the
      // Client then notify the application
      if ( ( status == SUCCESS )          &&
           ( method != GATT_LOCAL_WRITE ) &&
           ( pAttr->type.len == ATT_BT_UUID_SIZE ) )
      {
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1] );
        if ( uuid == GATT_CLIENT_CHAR_CFG_UUID )
        {
          uint16 value = BUILD_UINT16( pValue[0], pValue[1] );

          // CCC value has been updated, notify the application
          MAP_GATTServApp_SendCCCUpdatedEvent( connHandle, handle, value );
        }
      }
    }
    else
    {
      status = ATT_ERR_UNLIKELY;
    }
  }
  else
  {
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

#if defined ( TESTMODES )
/*********************************************************************
 * @fn      GATTServApp_SetParamValue
 *
 * @brief   Set a GATT Server Application Parameter value. Use this
 *          function to change the default GATT parameter values.
 *
 * @param   value - new param value
 *
 * @return  void
 */
void GATTServApp_SetParamValue( uint16 value )
{
  uint32_t status;

  if (!BLE_invokeIfRequired((void *)&GATTServApp_SetParamValue, &status, value))
  {
    gattservapp_paramValue = value;
  }
}

/*********************************************************************
 * @fn      GATTServApp_GetParamValue
 *
 * @brief   Get a GATT Server Application Parameter value.
 *
 * @param   none
 *
 * @return  GATT Parameter value
 */
uint16 GATTServApp_GetParamValue( void )
{
  uint16 status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATTServApp_GetParamValue, &invokeStatus))
  {
    status = (uint16)invokeStatus;
  }
  else
  {
    status = gattservapp_paramValue;
  }
  return (status);
}
#endif

/*********************************************************************
 * @fn      GATTServApp_UpdateCharCfg
 *
 * @brief   Update the Client Characteristic Configuration for a given
 *          Client.
 *
 *          Note: This API should only be called from the Bond Manager.
 *
 * @param   connHandle - connection handle.
 * @param   attrHandle - attribute handle.
 * @param   value - characteristic configuration value (from NV).
 *
 * @return  Success or Failure
 */
bStatus_t GATTServApp_UpdateCharCfg( uint16 connHandle, uint16 attrHandle, uint16 value )
{
  uint8 buf[2];

  buf[0] = LO_UINT16( value );
  buf[1] = HI_UINT16( value );

  return ( MAP_GATTServApp_WriteAttr( connHandle, attrHandle, buf, 2, 0 , GATT_LOCAL_WRITE ) );
}

//#ifndef GATT_NO_SERVICE_CHANGED
/*********************************************************************
 * @fn      GATTServApp_SendServiceChangedInd
 *
 * @brief   Send out a Service Changed Indication.
 *
 * @param   connHandle - connection to use
 * @param   taskId - task to be notified of confirmation
 *
 * @return  SUCCESS: Indication was sent successfully.
 *          FAILURE: Service Changed attribute not found.
 *          INVALIDPARAMETER: Invalid connection handle or request field.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          blePending: A confirmation is pending with this client.
 */
bStatus_t GATTServApp_SendServiceChangedInd( uint16 connHandle, uint8 taskId )
{
  bStatus_t status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATTServApp_SendServiceChangedInd, &invokeStatus,
    connHandle, ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, taskId)))
  {
    status = (bStatus_t)invokeStatus;
  }
  else {

  uint16 value = GATTServApp_ReadCharCfg( connHandle, indCharCfg );

  if ( value & GATT_CLIENT_CFG_INDICATE )
  {
    status = GATT_ServiceChangedInd(connHandle, taskId);
  }
  else {
    status = FAILURE;
  }
  }
  return (status);
}
//#endif // GATT_NO_SERVICE_CHANGED

/*********************************************************************
 * @fn          gattServApp_ResetCharCfg
 *
 * @brief       Reset all Client Characteristic Configuration attributes.
 *
 * @param       connHandle - connection handle
 *
 * @return      none
 */
void gattServApp_ResetCharCfg( uint16 connHandle )
{
  uint16 service;
  gattAttribute_t *pAttr;

  pAttr = GATT_FindHandleUUID( GATT_MIN_HANDLE, GATT_MAX_HANDLE,
                               clientCharCfgUUID, ATT_BT_UUID_SIZE, &service );
  while ( pAttr != NULL )
  {
    // Reset Client Char Config
    GATTServApp_InitCharCfg( connHandle, GATT_CCC_TBL(pAttr->pValue) );

    // Try to find the next Client Char Config attribute
    pAttr = GATT_FindNextAttr( pAttr, GATT_MAX_HANDLE, service, NULL );
  }
}

/*********************************************************************
 * @fn          gattServApp_HandleConnStatusCB
 *
 * @brief       GATT Server Application link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
void gattServApp_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Check to see if the connection has dropped
  if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
       ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
         ( !MAP_linkDB_State( connHandle, LINK_CONNECTED ) ) ) )
  {
    prepareWrites_t *pQueue = gattServApp_FindPrepareWriteQ( connHandle );

    // See if this client has a prepare write queue
    if ( pQueue != NULL )
    {
      // Clear the queue for this client
      gattServApp_ClearPrepareWriteQ( pQueue );
    }

    // Reset all Client Char Config when connection drops
    gattServApp_ResetCharCfg( connHandle );

    if (GATTServApp_att_delayed_req) //#ifdef ATT_DELAYED_REQ
    {
      // Free any response that was being created to be sent to the connection
      // handle that disconnected. Also clear request.
      if ( req.connHandle == connHandle )
      {
        // Free buffer any buffer allocated in response struct
        GATT_bm_free( (gattMsg_t *)&rsp, req.method + 1 );

        // Deallocate and clear stored request
        GATT_bm_free( (gattMsg_t *)&req.msg, req.method );
        VOID MAP_osal_memset( &req, 0, sizeof( gattMsgEvent_t ) );
      }
    } //#endif // ATT_DELAYED_REQ


    // Remove any queued retransmissions
    gattReTx_t *pGattReTx = (gattReTx_t *)MAP_osal_list_head(&gattReTxList);
    while(pGattReTx != NULL)
    {
      // Save the next element in list
      gattReTx_t *pGattReTxNext = (gattReTx_t *)MAP_osal_list_next((osal_list_elem *)pGattReTx);

      // If this is a relevant retransmission
      if (pGattReTx->connHandle)
      {
        // Remove element from list
        MAP_osal_list_remove(&gattReTxList, (osal_list_elem *)pGattReTx);

        // Free GATT payload
        MAP_GATT_bm_free(&(pGattReTx->msg), pGattReTx->method);

        // Free retransmission element
        MAP_osal_mem_free(pGattReTx);
        pGattReTx = NULL;
      }

      // Go to next element in list
      pGattReTx = pGattReTxNext;
    }
  }
}

/*********************************************************************
 * @fn      GATTServApp_SendCCCUpdatedEvent
 *
 * @brief   Build and send the GATT_CLIENT_CHAR_CFG_UPDATED_EVENT to
 *          the app.
 *
 * @param   connHandle - connection handle
 * @param   attrHandle - attribute handle
 * @param   value - attribute new value
 *
 * @return  none
 */
void GATTServApp_SendCCCUpdatedEvent( uint16 connHandle, uint16 attrHandle, uint16 value )
{
  uint32_t status;

  if (!BLE_invokeIfRequired((void *)&GATTServApp_SendCCCUpdatedEvent, &status,
    connHandle, attrHandle, value))

  {

  if ( appTaskID != INVALID_TASK_ID )
  {
    // Allocate, build and send event
    gattClientCharCfgUpdatedEvent_t *pEvent =
      (gattClientCharCfgUpdatedEvent_t *)MAP_osal_msg_allocate( (uint16)(sizeof ( gattClientCharCfgUpdatedEvent_t )) );
    if ( pEvent )
    {
      pEvent->hdr.event = GATT_SERV_MSG_EVENT;
      pEvent->hdr.status = SUCCESS;

      pEvent->method = GATT_CLIENT_CHAR_CFG_UPDATED_EVENT;
      pEvent->connHandle = connHandle;
      pEvent->attrHandle = attrHandle;
      pEvent->value = value;

      VOID MAP_osal_msg_send( appTaskID, (uint8 *)pEvent );
    }
  }
  }
}

/*********************************************************************
 * @fn      gattServApp_EnqueueReTx
 *
 * @brief   Enqueue an ATT response retransmission
 *
 * Enable L2CAP signaling to GATT Serv App if it is not already enabled.
 *
 * @param       connHandle - connection event belongs to
 * @param       method - type of message
 * @param       pMsg - pointer to message to be sent
 *
 * @return      SUCCESS
 * @return      @ref bleMemAllocError
 */
bStatus_t gattServApp_EnqueueReTx( uint16 connHandle, uint8 method, gattMsg_t *pMsg )
{
  gattReTx_t *pGattReTx = NULL;

  // Only enqueue when the connection is still valid
  if (MAP_linkDB_State( connHandle, LINK_CONNECTED ))
  {
    // Allocate retransmission list element
    pGattReTx = MAP_osal_mem_alloc(sizeof(gattReTx_t));

    if(pGattReTx)
    {
      // If there aren't currently any queued retransmissions
      if(MAP_osal_list_empty(&gattReTxList))
      {
        // If there is already a task registered to receive L2CAP signal events,
        // set the L2CAP forwarding task
        if (flowCtrlTaskId != INVALID_TASK_ID)
        {
          flowCtrlFwdTaskId = flowCtrlTaskId;
        }

        // Register with L2CAP to receive flow control packets
        flowCtrlTaskId = GATTServApp_TaskID;

        // Set flag to only send ATT responses
        att_sendRspOnly = TRUE;
      }

      // Fill up retransmission list element
      pGattReTx->connHandle = connHandle;
      pGattReTx->method = method;
      if(pMsg != NULL)
      {
        VOID MAP_osal_memcpy(&(pGattReTx->msg), pMsg, sizeof(gattMsg_t));
      }
      else
      {
        VOID MAP_osal_memset(&(pGattReTx->msg), 0, sizeof(gattMsg_t));
      }

      // Add element to list
      MAP_osal_list_put(&gattReTxList, (osal_list_elem *)pGattReTx);

      return ( SUCCESS );
    }
    else
    {
      return ( bleMemAllocError );
    }
  }
  return ( bleNotConnected );
}

/*********************************************************************
 * @fn      gattServApp_DequeueReTx
 *
 * @brief  Dequeue and try to send an ATT response retransmission
 *
 * Disable L2CAP signaling to GATT Serv App if the queue is empty
 *
 * @param       connHandle - connection event belongs to
 * @param       method - type of message
 * @param       pMsg - pointer to message to be sent
 */
void gattServApp_DequeueReTx( void )
{
  uint8 status = USUCCESS;

  // Check if there are any queued retransmissions
  gattReTx_t *pGattReTx = (gattReTx_t *)MAP_osal_list_head(&gattReTxList);
  if(pGattReTx != NULL)
  {
    // Try to retransmit
    status = GATT_SendRsp(pGattReTx->connHandle, pGattReTx->method, &(pGattReTx->msg));

    // Need to handle only those cases. In any other cases, try to retransmit the packet.
    if ((status == USUCCESS) || (status == bleNotConnected))
    {
      // Remove element from list
      MAP_osal_list_remove(&gattReTxList, (osal_list_elem *)pGattReTx);

      if (status == bleNotConnected)
      {
        // Free GATT payload
        MAP_GATT_bm_free(&(pGattReTx->msg), pGattReTx->method);
      }

      // Free retransmission element
      MAP_osal_mem_free(pGattReTx);
    }
  }

  if(MAP_osal_list_empty(&gattReTxList))
  {
    // List is empty, Stop L2CAP signaling. Restore the forwarding task
    // assuming it is set.
    flowCtrlTaskId = flowCtrlFwdTaskId;

    // Clear flag to only send ATT responses
    att_sendRspOnly = UFALSE;
  }

  return;
}
#endif // ( CENTRAL_CFG | PERIPHERAL_CFG )

/****************************************************************************
****************************************************************************/
