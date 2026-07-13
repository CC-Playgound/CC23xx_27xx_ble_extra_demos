/******************************************************************************

 @file  gatt_server.c

 @brief This file contains the Generic Attribute Profile Server.

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


/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/stack_util/bcomdef.h"
#include "ti/ble/host/common/linkdb.h"
#include "ti/ble/host/common/linkdb_internal.h"
#include "ti/ble/stack_util/osal/osal_bufmgr.h"
#include "ti/ble/host/gatt/gatt.h"
#include "ti/ble/host/gatt/gatt_uuid.h"
#include "ti/ble/host/gatt/gatt_internal.h"
#include "ti/ble/stack_util/lib_opt/map_direct.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Value passed into GATT_Indication's and GATT_Notification's authentication
// parameter to define which level of authentication is required.
#define GATT_LEGACY_PAIRING_AUTH     0x01
#define GATT_SECURE_CONNECTION_AUTH  0x02

/*********************************************************************
 * TYPEDEFS
 */

// Function prototype to parse an attribute protocol request message
typedef bStatus_t (*gattParseReq_t)( uint8 sig, uint8 cmd, uint8 *pParams, uint16 len, attMsg_t *pMsg );

// Function prototype to process an attribute protocol request message
typedef bStatus_t (*gattProcessReq_t)( uint16 connHandle,  attMsg_t *pMsg );

// Service record list item
typedef struct _attAttrList
{
  struct _attAttrList *next;  // pointer to next service record
  gattService_t service;      // service record
} gattServiceList_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// GATT Service List
static gattServiceList_t *pServiceList = NULL;

// Server Info table (one entry per each physical link)
static gattServerInfo_t *serverInfoTbl;

// Task to be notified of requests
static uint8 reqTaskId = INVALID_TASK_ID;

// Next available attribute handle
static uint16 nextHandle = GATT_MIN_HANDLE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static gattService_t *gattFindService(uint16 handle);
static gattServerInfo_t *gattFindServerInfo(uint16 connHandle);
static gattParseReq_t gattParseReq(uint8 method);
static bStatus_t gattProcessExchangeMTUReq(uint16 connHandle, attMsg_t *pMsg);
static bStatus_t gattProcessFindInfoReq(uint16 connHandle, attMsg_t *pMsg);
static bStatus_t gattProcessFindByTypeValueReq(uint16 connHandle, attMsg_t *pMsg);
static bStatus_t gattProcessReadByTypeReq(uint16 connHandle, attMsg_t *pMsg);
static bStatus_t gattProcessReadReq(uint16 connHandle, attMsg_t *pMsg);
static bStatus_t gattProcessReadMultiReq(uint16 connHandle, attMsg_t *pMsg);
static bStatus_t gattProcessReadByGrpTypeReq(uint16 connHandle, attMsg_t *pMsg);
static gattProcessReq_t gattProcessReq(uint8 method);
static bStatus_t gattProcessWriteReq(uint16 connHandle, attMsg_t *pMsg);
static bStatus_t gattProcessExecuteWriteReq(uint16 connHandle, attMsg_t *pMsg);
static void gattResetServerInfo(gattServerInfo_t *pServer);
static void gattServerHandleConnStatusCB(uint16 connHandle, uint8 changeType);
static void gattServerHandleTimerCB(uint8 *pData);
static void gattServerNotifyTxCB(uint16 connHandle, uint8 opcode);
static bStatus_t gattServerProcessMsgCB(uint16 connHandle, attPacket_t *pPkt, uint8 *pSafeToDealloc);
static void gattServerStartTimer(uint8 *pData, uint16 timeout, uint8 *pTimerId);
static uint16 gattServiceLastHandle(uint16 handle);
static void gattStoreServerInfo(gattServerInfo_t *pServer, uint8 taskId);

/*********************************************************************
 * API FUNCTIONS
 */

/*-------------------------------------------------------------------
 * GATT Server Public APIs
 */

/******************************************************************************
 * @fn      GATT_InitServer
 *
 * @brief   Initialize the Generic Attribute Profile Server.
 *
 * @return  SUCCESS: Server initialized successfully.
 *          bleMemAllocError: Memory allocation error occurred.
 */
bStatus_t GATT_InitServer( void )
{
  // Allocate Server Info table
  serverInfoTbl = MAP_osal_mem_alloc( sizeof(gattServerInfo_t) * gattNumConns );
  if ( serverInfoTbl == NULL )
  {
    return ( bleMemAllocError );
  }

  // Mark all Server records as unused
  for ( uint8 i = 0; i < gattNumConns; i++ )
  {
    gattServerInfo_t *pServer = &serverInfoTbl[i];

    // Initialize connection handle
    if ( i == 0 )
    {
      pServer->connHandle = LINKDB_CONNHANDLE_LOOPBACK;
    }
    else
    {
      pServer->connHandle = LINKDB_CONNHANDLE_INVALID;
    }

    // Initialize Handle Value Confirmation info
    pServer->taskId = INVALID_TASK_ID;
    pServer->timerId = INVALID_TIMER_ID;

    // Initialize request info
    pServer->pendingReq = 0;
  }

  // Set up the server's processing function
  MAP_gattRegisterServer( MAP_gattServerProcessMsgCB );

  // Set up the server's notify Tx function
  MAP_ATT_RegisterServer( MAP_gattServerNotifyTxCB );

  // Register with Link DB to receive link status change callback
  VOID MAP_linkDB_Register( MAP_gattServerHandleConnStatusCB );

  return ( SUCCESS );
}

/******************************************************************************
 * @fn      GATT_RegisterService
 *
 * @brief   Register a service attribute list with the GATT Server. A service
 *          is composed of characteristics or references to other services.
 *          Each characteristic contains a value and may contain optional
 *          information about the value. There are two types of services:
 *          primary service and secondary service.
 *
 *          A service definition begins with a service declaration and ends
 *          before the next service declaration or the maximum Attribute Handle.
 *
 *          A characteristic definition begins with a characteristic declaration
 *          and ends before the next characteristic or service declaration or
 *          maximum Attribute Handle.
 *
 *          The attribute server will only keep a pointer to the attribute
 *          list, so the calling application will have to maintain the code
 *          and RAM associated with this list.
 *
 * @param   pService - pointer to service attribute list to be registered
 *
 * @return  SUCCESS: Service registered successfully.
 *          INVALIDPARAMETER: Invalid service field.
 *          FAILURE: Not enough attribute handles available.
 *          bleMemAllocError: Memory allocation error occurred.
 *          bleInvalidRange: Encryption key size's out of range.
 */
bStatus_t GATT_RegisterService( gattService_t *pService )
{
  bStatus_t status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATT_RegisterService, &invokeStatus,
    pService))
  {
    status = (bStatus_t)invokeStatus;
  }
  else
  {
  gattServiceList_t *pNewItem;

  // Make sure the service attribute list begins with a service declaration
  if ( ( pService->numAttrs == 0 ) || !gattServiceType( pService->attrs[0].type ) )
  {
    return ( INVALIDPARAMETER );
  }

  if ( ( pService->encKeySize < GATT_MIN_ENCRYPT_KEY_SIZE ) ||
       ( pService->encKeySize > GATT_MAX_ENCRYPT_KEY_SIZE ) )
  {
    return ( bleInvalidRange );
  }

  // Make sure we have enough attribute handles available for this service
  if ( ( nextHandle == 0 ) || ( pService->numAttrs > ( GATT_MAX_HANDLE - nextHandle ) + 1 ) )
  {
    return ( FAILURE );
  }

  // Allocate space for the new service item
  pNewItem = (gattServiceList_t *)MAP_osal_mem_alloc( sizeof( gattServiceList_t ) );
  if ( pNewItem == NULL )
  {
    // Not enough memory
    return ( bleMemAllocError );
  }

  // Assign attribute handles
  for ( uint16 i = 0; i < pService->numAttrs; i++ )
  {
    pService->attrs[i].handle = nextHandle++;
  }

  // Set up new service item
  pNewItem->next = NULL;
  MAP_osal_memcpy( &(pNewItem->service), pService, sizeof( gattService_t ) );

  // Find spot in list
  if ( pServiceList == NULL )
  {
    // First item in list
    pServiceList = pNewItem;
  }
  else
  {
    gattServiceList_t *pLoop = pServiceList;

    // Look for end of list
    while ( pLoop->next != NULL )
    {
      pLoop = pLoop->next;
    }

    // Put new item at end of list
    pLoop->next = pNewItem;
  }
  status = SUCCESS;
  }
  return (status);
}

/******************************************************************************
 * @fn      GATT_DeregisterService
 *
 * @brief   Deregister a service attribute list with the GATT Server.
 *
 *          NOTE: It's the caller's responsibility to free the service attribute
 *          list returned from this API.
 *
 * @param   handle - handle of service to be deregistered
 * @param   pService - pointer to deregistered service (to be returned)
 *
 * @return  SUCCESS: Service deregistered successfully.
 *          FAILURE: Service not found.
 */
bStatus_t GATT_DeregisterService( uint16 handle, gattService_t *pService )
{
  bStatus_t status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATT_DeregisterService, &invokeStatus,
    handle, pService))
  {
    status = (bStatus_t)invokeStatus;
  }
  else
  {
  gattServiceList_t *pLoop = pServiceList;
  gattServiceList_t *pPrev = NULL;

  // Look for service
  while ( pLoop != NULL )
  {
    if ( pLoop->service.attrs[0].handle == handle )
    {
      // Service found; unlink it
      if ( pPrev == NULL )
      {
        // First item in list
        pServiceList = pLoop->next;
      }
      else
      {
        pPrev->next = pLoop->next;
      }

      // Application will free the service attribute list
      if ( pService != NULL )
      {
        VOID MAP_osal_memcpy( pService, &(pLoop->service), sizeof( gattService_t ) );
      }

      // Free the service record
      MAP_osal_mem_free( pLoop );

      return ( SUCCESS );
    }

    pPrev = pLoop;
    pLoop = pLoop->next;
  }

  // Service not found
  status = FAILURE;
  }
  return (status);
}

/******************************************************************************
 * @fn      GATT_RegisterForReq
 *
 * @brief   Register to receive incoming ATT Requests.
 *
 * @param   taskId - task to forward requests to
 *
 * @return  void
 */
void GATT_RegisterForReq( uint8 taskId )
{
  reqTaskId = taskId;
}

/*********************************************************************
 * @fn      GATT_VerifyReadPermissions
 *
 * @brief   Verify the permissions of an attribute for reading.
 *
 * @param   connHandle - connection to use
 * @param   pAttr - pointer to attribute
 * @param   service - service handle
 *
 * @return  SUCCESS: Attribute can be read
 *          ATT_ERR_READ_NOT_PERMITTED: Attribute cannot be read
 *          ATT_ERR_INSUFFICIENT_AUTHEN: Attribute requires authentication
 *          ATT_ERR_INSUFFICIENT_KEY_SIZE: Key Size used for encrypting is insufficient
 *          ATT_ERR_INSUFFICIENT_ENCRYPT: Attribute requires encryption
 */
bStatus_t GATT_VerifyReadPermissions( uint16 connHandle, gattAttribute_t *pAttr,
                                      uint16 service )
{
  // Make sure the requesting device has sufficient security
  if ( gattPermitAuthenRead( pAttr->permissions ) )
  {
    return ( MAP_linkDB_Authen( connHandle, MAP_GATT_ServiceEncKeySize( service ),
                            TRUE ) );
  }

  if ( gattPermitEncryptRead( pAttr->permissions ) )
  {
    // Read operation requires an encrypted link (unauthenticated)
    return ( MAP_linkDB_Authen( connHandle, MAP_GATT_ServiceEncKeySize( service ),
                            FALSE ) );
  }

  // Make sure the requesting device has sufficient authorization
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // According to the spec Vol 3 Part C , 10.5
    // A service may require authorization before allowing access. Authorization is a
    // confirmation by the user to continue with the procedure. Authentication does
    // not necessarily provide authorization. Authorization may be granted by user
    // confirmation after successful authentication.
    if ( MAP_linkDB_Authen( connHandle, MAP_GATT_ServiceEncKeySize( service ), gattPermitAuthenRead( pAttr->permissions ) ) == SUCCESS )
    {
      // Use Service's authorization callback to authorize the request
      pfnGATTAuthorizeAttrCB_t pfnCB = MAP_gattServApp_FindAuthorizeAttrCB( service );
      if ( pfnCB != NULL )
      {
        return((*pfnCB)( connHandle, pAttr, ATT_READ_REQ ));
      }
      else
      {
        return(ATT_ERR_UNLIKELY);
      }
    }
    else
    {
      return (ATT_ERR_INSUFFICIENT_AUTHOR);
    }
  }
  // Make sure the attribute has sufficient permissions to allow reading
  else if ( ! gattPermitRead( pAttr->permissions ) )
  {
    return ( ATT_ERR_READ_NOT_PERMITTED );
  }
  else
  {
        /* this else clause is required, even if the
           programmer expects this will never be reached
           Fix Misra-C Required: MISRA.IF.NO_ELSE */
  }

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      GATT_VerifyWritePermissions
 *
 * @brief   Verify the permissions of an attribute for writing.
 *
 * @param   connHandle - connection to use
 * @param   pAttr - pointer to attribute
 * @param   service - service handle
 * @param   pReq - pointer to write request
 *
 * @return  SUCCESS: Attribute can be written
 *          ATT_ERR_WRITE_NOT_PERMITTED: Attribute cannot be written
 *          ATT_ERR_INSUFFICIENT_AUTHEN: Attribute requires authentication
 *          ATT_ERR_INSUFFICIENT_KEY_SIZE: Key Size used for encrypting is insufficient
 *          ATT_ERR_INSUFFICIENT_ENCRYPT: Attribute requires encryption
 */
bStatus_t GATT_VerifyWritePermissions( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint16 service, attWriteReq_t *pReq )
{
  // Make sure the requesting device has sufficient authorization
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // According to the spec Vol 3 Part C , 10.5
    // A service may require authorization before allowing access. Authorization is a
    // confirmation by the user to continue with the procedure. Authentication does
    // not necessarily provide authorization. Authorization may be granted by user
    // confirmation after successful authentication.
    uint8 status = MAP_linkDB_Authen( connHandle, MAP_GATT_ServiceEncKeySize( service ), gattPermitAuthenWrite( pAttr->permissions ));
    // Write operation requires an encrypted link or an authenticated signed command
    if ( ( status != SUCCESS ) && ( ( pReq->cmd == FALSE ) || ( pReq->sig != ATT_SIG_VALID ) ) )
    {
      return (ATT_ERR_INSUFFICIENT_AUTHOR);
    }
    else
    {
      // Use Service's authorization callback to authorize the request
      pfnGATTAuthorizeAttrCB_t pfnCB = MAP_gattServApp_FindAuthorizeAttrCB( service );
      if ( pfnCB != NULL )
      {
        return((*pfnCB)( connHandle, pAttr, ATT_WRITE_REQ ));
      }
      else
      {
        return(ATT_ERR_UNLIKELY);
      }
    }
  }

  if ( gattPermitEncryptWrite( pAttr->permissions ) )
  {
    // Write operation requires an encrypted link (unauthenticated)
    return ( MAP_linkDB_Authen( connHandle, MAP_GATT_ServiceEncKeySize( service ), FALSE ));
  }

  if ( gattPermitAuthenWrite( pAttr->permissions ) )
  {
    uint8 status = MAP_linkDB_Authen( connHandle, MAP_GATT_ServiceEncKeySize( service ),TRUE );
    // Write operation requires an encrypted link or an authenticated signed command
    if ( ( status != SUCCESS ) && ( ( pReq->cmd == FALSE ) || ( pReq->sig != ATT_SIG_VALID ) ) )
    {
      return ( status );
    }
  }
  // Make sure the attribute has sufficient permissions to allow writing
  else if ( !gattPermitWrite( pAttr->permissions ) )
  {
    return ( ATT_ERR_WRITE_NOT_PERMITTED );
  }
  else
  {
        /* this else clause is required, even if the
           programmer expects this will never be reached
           Fix Misra-C Required: MISRA.IF.NO_ELSE */
  }

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      GATT_ServiceChangedInd
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
uint8 GATT_ServiceChangedInd( uint16 connHandle, uint8 taskId )
{
  gattAttribute_t *pAttr;
  uint8 status;

  // Find the Service Changed attribute record
  pAttr = MAP_GATT_FindHandleUUID( GATT_MIN_HANDLE, GATT_MAX_HANDLE,
                               serviceChangedUUID, ATT_BT_UUID_SIZE, NULL );
  if ( pAttr != NULL )
  {
    attHandleValueInd_t ind;

    ind.pValue = MAP_GATT_bm_alloc( connHandle, ATT_HANDLE_VALUE_IND, 4, NULL );
    if ( ind.pValue != NULL )
    {
      ind.handle = pAttr->handle;

      // Set the affected Attribute Handle range to 0x0001 to 0xFFFF to
      // indicate to the client to rediscover the entire set of Attribute
      // Handles on the server.
      ind.len = 4;
      ind.pValue[0] = LO_UINT16( GATT_MIN_HANDLE );
      ind.pValue[1] = HI_UINT16( GATT_MIN_HANDLE );
      ind.pValue[2] = LO_UINT16( GATT_MAX_HANDLE );
      ind.pValue[3] = HI_UINT16( GATT_MAX_HANDLE );

      status = MAP_GATT_Indication( connHandle, &ind, FALSE, taskId );
      if ( status != SUCCESS )
      {
        MAP_osal_bm_free( ind.pValue );
      }
    }
    else
    {
      status = bleNoResources;
    }
  }
  else
  {
    status = FAILURE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      GATT_FindHandleUUID
 *
 * @brief   Find the attribute record for a given handle and UUID.
 *
 * @param   startHandle - first handle to look for
 * @param   endHandle - last handle to look for
 * @param   pUUID - pointer to UUID to look for
 * @param   len - length of UUID
 * @param   pHandle - handle of owner of attribute (to be returned)
 *
 * @return  Pointer to attribute record. NULL, otherwise.
 */
gattAttribute_t *GATT_FindHandleUUID( uint16 startHandle, uint16 endHandle, const uint8 *pUUID,
                                      uint16 len, uint16 *pHandle )
{
  gattServiceList_t *pLoop = pServiceList;

  while ( pLoop != NULL )
  {
    for ( uint16 i = 0; i < pLoop->service.numAttrs; i++ )
    {
      gattAttribute_t *pAttr = &(pLoop->service.attrs[i]);

      // Check to see if this handle falls within the starting and ending handles
      if ( ( pAttr->handle >= startHandle ) && ( pAttr->handle <= endHandle ) )
      {
        // Compare UUIDs if one is provided
        if ( ( len == 0 ) ||
             ( MAP_ATT_CompareUUID( pAttr->type.uuid, pAttr->type.len,
                                pUUID, len ) ) )
        {
          // Entry found
          if ( pHandle != NULL )
          {
            // Handle of the service that the attribute belongs to
            *pHandle = pLoop->service.attrs[0].handle;
          }

          return ( pAttr );
        }
      }
    }

    // Try next service
    pLoop = pLoop->next;
  }

  return ( (gattAttribute_t *)NULL );
}
#define  xDEBUG_TEST
#ifdef DEBUG_TEST
uint8_t _found = 0;
gattAttribute_t *_g_pAttr = NULL;
uint16 _serviceHandle;
#endif
/*********************************************************************
 * @fn      GATT_FindHandle
 *
 * @brief   Find the attribute record for a given handle
 *
 * @param   handle - handle to look for
 * @param   pHandle - handle of owner of attribute (to be returned)
 *
 * @return  Pointer to attribute record. NULL, otherwise.
 */
gattAttribute_t *GATT_FindHandle( uint16 handle, uint16 *pHandle )
{
  gattServiceList_t *pLoop = pServiceList;
  #ifdef DEBUG_TEST
  if(_found)
  {
    *pHandle = _serviceHandle;
    return(_g_pAttr);
  }
  #endif

  while ( pLoop != NULL )
  {
    uint16 serviceHandle = pLoop->service.attrs[0].handle;

    // See if the handle falls within this service
    if ( ( handle >= serviceHandle ) && ( handle < serviceHandle + pLoop->service.numAttrs ) )
    {
      for ( uint16 i = 0; i < pLoop->service.numAttrs; i++ )
      {
        gattAttribute_t *pAttr = &(pLoop->service.attrs[i]);

        if ( pAttr->handle == handle )
        {
          // Entry found
          if ( pHandle != NULL )
          {
            #ifdef DEBUG_TEST
            _found = 1;
            _g_pAttr = pAttr;
            _serviceHandle = serviceHandle;
            #endif
            // Handle of the service that the attribute belongs to
            *pHandle = serviceHandle;
          }

          return ( pAttr );
        }
      }
    }

    // Try next service
    pLoop = pLoop->next;
  }

  return ( (gattAttribute_t *)NULL );
}

/*********************************************************************
 * @fn      GATT_FindNextAttr
 *
 * @brief   Find the next attribute of the same type for a given attribute.
 *
 * @param   pAttr - pointer to attribute to find a next for
 * @param   endHandle - last handle to look for
 * @param   service - handle of owner service
 * @param   pLastHandle - handle of last attribute (to be returned)
 *
 * @return  Pointer to next attribute record. NULL, otherwise.
 */
gattAttribute_t *GATT_FindNextAttr( gattAttribute_t *pAttr, uint16 endHandle,
                                    uint16 service, uint16 *pLastHandle )
{
  uint16 lastHandle;
  gattAttribute_t *pNext = NULL;
  uint16 owner = GATT_INVALID_HANDLE;

  // Try to find the next attribute of the same type

  // All attribute types are effectively compared as 128-bit UUIDs,
  // even if a 16-bit UUID is provided in this request or defined
  // for an attribute.
  if ( pAttr->handle != GATT_MAX_HANDLE )
  {
    pNext = MAP_GATT_FindHandleUUID( pAttr->handle+1, endHandle,
                                     pAttr->type.uuid, pAttr->type.len,
                                     &owner );
  }

  // Try to find the handle of the last attribute
  if ( gattServiceType( pAttr->type ) )
  {
    // Get the handle of the last attribute within this service
    lastHandle = MAP_gattServiceLastHandle( pAttr->handle );
  }
  else if ( gattCharacterType( pAttr->type ) )
  {
    // Check to see if this is the last characteristic within the service
    if ( ( pNext == NULL ) || ( owner != service ) )
    {
      lastHandle = MAP_gattServiceLastHandle( service );
    }
    else
    {
      lastHandle = pNext->handle - 1;
    }
  }
  else
  {
    // Not a grouping attribute -- return its handle
    lastHandle = pAttr->handle;
  }

  if ( pLastHandle != NULL )
  {
    *pLastHandle = lastHandle;
  }

  return ( pNext );
}

/*********************************************************************
 * @fn      GATT_ServiceNumAttrs
 *
 * @brief   Get the number of attributes for a given service.
 *
 * @param   handle - service handle to look for
 *
 * @return  Number of attributes if service found. 0, otherwise.
 */
uint16 GATT_ServiceNumAttrs( uint16 handle )
{
  gattService_t *pService = MAP_gattFindService( handle );
  if ( pService != NULL )
  {
    // Return number of attributes
    return ( pService->numAttrs );
  }

  // Service not found
  return ( 0 );
}

/*********************************************************************
 * @fn      GATT_ServiceEncKeySize
 *
 * @brief   Get the minimum encryption key size required by a given service.
 *
 * @param   handle - service handle to look for
 *
 * @return  Encryption key size if service found. Default key size, otherwise.
 */
uint8 GATT_ServiceEncKeySize( uint16 handle )
{
  gattService_t *pService = MAP_gattFindService( handle );
  if ( pService != NULL )
  {
    // Return encryption key size
    return ( pService->encKeySize );
  }

  // Service not found
  return ( GATT_MAX_ENCRYPT_KEY_SIZE );
}

/*********************************************************************
 * @fn      GATT_SendRsp
 *
 * @brief   Send an ATT Response message out.
 *
 * @param   connHandle - connection to use
 * @param   method - type of response message
 * @param   pRsp - pointer to ATT response to be sent
 *
 * @return  SUCCESS: Response was sent successfully.
 *          INVALIDPARAMETER: Invalid response field.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          bleMemAllocError: Memory allocation error occurred.
 *          blePending: In the middle of another transmit.
 *          bleInvalidMtuSize: Packet length is larger than connection's MTU size.
 */
bStatus_t GATT_SendRsp( uint16 connHandle, uint8 method, gattMsg_t *pRsp )
{
  uint8 status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATT_SendRsp, &invokeStatus, connHandle,
      method, pRsp))
  {
      status = (uint8)invokeStatus;
  }
  else
  {

  switch ( method )
  {
    case ATT_ERROR_RSP:
      status = MAP_ATT_ErrorRsp( connHandle, (attErrorRsp_t *)pRsp );
      break;

    case ATT_EXCHANGE_MTU_RSP:
      {
        attExchangeMTURsp_t rsp;

        // Set the Server Rx MTU parameter to the maximum MTU that this server
        // can receive
        rsp.serverRxMTU = MAP_L2CAP_GetMTU();

        status = MAP_ATT_ExchangeMTURsp( connHandle, &rsp );
        if ( status == SUCCESS )
        {
          // Update ATT_MTU size with what was passed up to app as serverRxMTU
          MAP_GATT_UpdateMTU( connHandle, ((attExchangeMTURsp_t *)pRsp)->serverRxMTU );
        }
      }
      break;

    case ATT_FIND_INFO_RSP:
      status = MAP_ATT_FindInfoRsp( connHandle, (attFindInfoRsp_t *)pRsp );
      break;

    case ATT_FIND_BY_TYPE_VALUE_RSP:
      status = MAP_ATT_FindByTypeValueRsp( connHandle, (attFindByTypeValueRsp_t *)pRsp );
      break;

    case ATT_READ_BY_TYPE_RSP:
      status = MAP_ATT_ReadByTypeRsp( connHandle, (attReadByTypeRsp_t *)pRsp );
      break;

    case ATT_READ_RSP:
      status = MAP_ATT_ReadRsp( connHandle, (attReadRsp_t *)pRsp );
      break;

    case ATT_READ_BLOB_RSP:
      status = MAP_ATT_ReadBlobRsp( connHandle, (attReadBlobRsp_t *)pRsp );
      break;

    case ATT_READ_MULTI_RSP:
      status = MAP_ATT_ReadMultiRsp( connHandle, (attReadMultiRsp_t *)pRsp );
      break;

    case ATT_READ_BY_GRP_TYPE_RSP:
      status = MAP_ATT_ReadByGrpTypeRsp( connHandle, (attReadByGrpTypeRsp_t *)pRsp );
      break;

    case ATT_WRITE_RSP:
      status = MAP_ATT_WriteRsp( connHandle );
      break;

    case ATT_PREPARE_WRITE_RSP:
      status = MAP_ATT_PrepareWriteRsp( connHandle, (attPrepareWriteRsp_t *)pRsp );
      break;

   case ATT_EXECUTE_WRITE_RSP:
      status = MAP_ATT_ExecuteWriteRsp( connHandle );
      break;

    default:
      status = INVALIDPARAMETER;
      break;
  }
  }
  return ( status );
}

/*-------------------------------------------------------------------
 * GATT Server Sub-Procedure APIs
 */

/*********************************************************************
 * @fn      GATT_Indication
 *
 * @brief   This sub-procedure is used when a server is configured to
 *          indicate a characteristic value to a client and expects an
 *          attribute protocol layer acknowledgement that the indication
 *          was successfully received.
 *
 *          The ATT Handle Value Indication is used in this sub-procedure.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive an OSAL GATT_MSG_EVENT message.
 *          The type of the message will be ATT_HANDLE_VALUE_CFM.
 *
 *          Note: This sub-procedure is complete when ATT_HANDLE_VALUE_CFM
 *                (with SUCCESS or bleTimeout status) is received by the
 *                calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pInd - pointer to indication to be sent
 * @param   taskId - task to be notified of confirmation
 * @param   authenticated - whether an authenticated link is required.
 *                          0x01: LE Legacy authenticated
 *                          0x02: Secure Connections authenticated
 *
 * @return  SUCCESS: Indication was sent successfully.
 *          INVALIDPARAMETER: Invalid connection handle or request field.
 *          ATT_ERR_INSUFFICIENT_AUTHEN: Link is not encrypted
 *          ATT_ERR_INSUFFICIENT_KEY_SIZE: Key size encrypted is not large enough
 *          ATT_ERR_INSUFFICIENT_ENCRYPT: Link is encrypted, but not authenticated
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          blePending: A confirmation is pending with this client.
 *          bleTimeout: Previous transaction timed out.
 */
bStatus_t GATT_Indication( uint16 connHandle, attHandleValueInd_t *pInd,
                           uint8 authenticated, uint8 taskId )
{
  bStatus_t status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATT_Indication, &invokeStatus,
      connHandle, pInd, authenticated, ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, taskId)))
  {
     status = (uint8)invokeStatus;
  }
  else
  {
    gattServerInfo_t *pServer;

    // Check the connection exists
    if (MAP_linkDB_Find( connHandle ) == NULL)
    {
      status  = bleNotConnected;
    }
    else
    {

      // Make sure we're allowed to send a new indication
      status = MAP_gattGetServerStatus( connHandle, &pServer );
      if ( status == SUCCESS )
      {
        // Make sure the link is authenticated if requested
        if ( authenticated )
        {
          uint16 service = GATT_INVALID_HANDLE;

          // Find service attribute belongs to
          VOID MAP_GATT_FindHandle( pInd->handle, &service );

          // Indication operation requires authentication
          status = MAP_linkDB_Authen( connHandle, MAP_GATT_ServiceEncKeySize( service ),
                                  ( authenticated == GATT_LEGACY_PAIRING_AUTH ? TRUE : FALSE ) );
        }

        if ( status == SUCCESS )
        {
          status = MAP_ATT_HandleValueInd( connHandle, pInd );
          if ( status == SUCCESS )
          {
            // Store server info
            MAP_gattStoreServerInfo( pServer, taskId );
          }
        }
      }
    }
  }
  return ( status );
}

/*********************************************************************
 * @fn      GATT_Notification
 *
 * @brief   This sub-procedure is used when a server is configured to
 *          notify a characteristic value to a client without expecting
 *          any attribute protocol layer acknowledgement that the
 *          notification was successfully received.
 *
 *          The ATT Handle Value Notification is used in this sub-procedure.
 *
 *          Note: A notification may be sent at any time and does not
 *          invoke a confirmation.
 *
 *          No confirmation will be sent to the calling application task for
 *          this sub-procedure.
 *
 * @param   connHandle - connection to use
 * @param   pNoti - pointer to notification to be sent
 * @param   authenticated - whether an authenticated link is required
 *                          0x01: LE Legacy authenticated
 *                          0x02: Secure Connections authenticated
 *
 * @return  SUCCESS: Notification was sent successfully.
 *          INVALIDPARAMETER: Invalid connection handle or request field.
 *          ATT_ERR_INSUFFICIENT_AUTHEN: Link is not encrypted
 *          ATT_ERR_INSUFFICIENT_KEY_SIZE: Key size encrypted is not large enough
 *          ATT_ERR_INSUFFICIENT_ENCRYPT: Link is encrypted, but not authenticated
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          bleTimeout: Previous transaction timed out.
 */
bStatus_t GATT_Notification( uint16 connHandle, attHandleValueNoti_t *pNoti,
                             uint8 authenticated )
{
  bStatus_t status;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATT_Notification, &invokeStatus,
      connHandle, pNoti, authenticated))
  {
     status = (uint8)invokeStatus;
  }
  else
  {
    gattServerInfo_t *pServer;

    // Check the connection exists
    if (MAP_linkDB_Find( connHandle ) == NULL)
    {
      status  = bleNotConnected;
    }
    else
    {

      // Make sure we're allowed to send a new notification
      status = MAP_gattGetServerStatus( connHandle, &pServer );
      if ( status != bleTimeout )
      {
        // Make sure the link is authenticated if requested
        if ( authenticated )
        {
          uint16 service = GATT_INVALID_HANDLE;

          // Find service attribute belongs to
          VOID MAP_GATT_FindHandle( pNoti->handle, &service );

          // Notification operation requires authentication
          status = MAP_linkDB_Authen( connHandle, MAP_GATT_ServiceEncKeySize( service ),
                                  ( authenticated == GATT_LEGACY_PAIRING_AUTH ? TRUE : FALSE ) );
          if ( status != SUCCESS )
          {
            return ( status );
          }
        }

        status = MAP_ATT_HandleValueNoti( connHandle, pNoti );
      }
    }
  }
  return ( status );
}

/*-------------------------------------------------------------------
 * GATT Server Internal Functions
 */

/*********************************************************************
 * @fn      gattServiceLastHandle
 *
 * @brief   Get the handle of the last attribute within a given service.
 *
 * @param   handle - service handle
 *
 * @return  Handle of last attribute for service. Handle, otherwise.
 */
static uint16 gattServiceLastHandle( uint16 handle )
{
  uint16 lastHandle;

  // Find out the handle of the last attribute withis this service
  lastHandle = MAP_GATT_ServiceNumAttrs( handle );
  if ( lastHandle != 0 )
  {
    lastHandle += (handle - 1);
  }
  else
  {
    lastHandle = handle;
  }

  return ( lastHandle );
}

/*********************************************************************
 * @fn      gattStoreServerInfo
 *
 * @brief   Store server info.
 *
 * @param   connHandle - connection to use
 * @param   taskId - task to be notified of response
 *
 * @return  void
 */
static void gattStoreServerInfo( gattServerInfo_t *pServer, uint8 taskId )
{
  if ( taskId != INVALID_TASK_ID )
  {
    // Start a timeout timer for the confirmation
    MAP_gattServerStartTimer( (uint8 *)pServer, ATT_MSG_TIMEOUT, &pServer->timerId );

    // Store task id to forward the confirmation to
    pServer->taskId = taskId;
  }
}

/*********************************************************************
 * @fn          gattProcessServerMsgCB
 *
 * @brief       GATT Server message processing function.
 *
 * @param       connHandle - connection packet was received on
 * @param       pPkt - pointer to received packet
 * @param       pSafeToDealloc - whether it's safe for caller to dealloc packet
 *
 * @return      SUCCESS: Message processed successfully
 *              ATT_ERR_UNSUPPORTED_REQ: Unsupported request or command
 *              ATT_ERR_INVALID_PDU: Invalid PDU
 *              bleMemAllocError: Memory allocation error occurred
 */
static bStatus_t gattServerProcessMsgCB( uint16 connHandle, attPacket_t *pPkt,
                                         uint8 *pSafeToDealloc )
{
  gattMsg_t msg;
  gattServerInfo_t *pServer = NULL;
  uint8 status;

  // See if this is a confirmation to an indication
  if ( pPkt->method == ATT_HANDLE_VALUE_CFM )
  {
    // Make sure we have the info about the Server that sent the indication
    pServer = MAP_gattFindServerInfo( connHandle );
    if ( ( pServer != NULL ) && ( TIMER_STATUS( pServer->timerId ) == blePending ) )
    {
      // Forward the indication up to the app
      VOID MAP_gattNotifyEvent( pServer->taskId, connHandle, SUCCESS, pPkt->method, NULL );

      // Reset server info
      MAP_gattResetServerInfo( pServer );
    }

    // We're done here
    return ( SUCCESS );
  }

  // Check sequential request-response protocol flow control. ATT commands
  // don't have any flow control.
  if ( !ATT_WRITE_COMMAND( pPkt->method, pPkt->cmd ) )
  {
    // Make sure there's no pending request to be processed by the app
    pServer = MAP_gattFindServerInfo( connHandle );
    if ( ( pServer != NULL ) && (  pServer->pendingReq > 0 ) )
    {
      // See if this is the first request that causes a flow control violation
      if ( ( pServer->pendingReq & ATT_FCV_BIT ) == 0 )
      {
        // Notify the upper layer app about the flow control violation
        MAP_gattSendFlowCtrlEvt( connHandle, pPkt->method, pServer->pendingReq );

        // Flow control violated
        pServer->pendingReq |= ATT_FCV_BIT;
      }

      // Drop the incoming request
      return ( SUCCESS );
    }
  }

  // Make sure the incoming request or command is supported
  if ( ( reqTaskId == INVALID_TASK_ID )                 ||
       ( pPkt->method < ATT_EXCHANGE_MTU_REQ )          ||
       ( pPkt->method > ATT_EXECUTE_WRITE_REQ )         ||
       ( MAP_gattParseReq( pPkt->method ) == NULL )     ||
       ( MAP_gattProcessReq( pPkt->method ) == NULL )   ||
       ( ( ( pPkt->sig != ATT_SIG_NOT_INCLUDED )  ||
           ( pPkt->cmd == TRUE ) )                &&
           ( pPkt->method != ATT_WRITE_REQ ) ) )
  {
    // Unsupported request or command
    return ( ATT_ERR_UNSUPPORTED_REQ );
  }

  // Parse the incoming request or command
  status = MAP_gattParseReq( pPkt->method )( pPkt->sig, pPkt->cmd, pPkt->pParams,
                                             pPkt->len, (attMsg_t *)&msg );
  if ( status == SUCCESS )
  {
    // Try to process the request or command
    status = MAP_gattProcessReq( pPkt->method )( connHandle, (attMsg_t *)&msg );
    if ( status == SUCCESS )
    {
      // Forward the request up to the application for further processing
      if ( pPkt->method != ATT_FIND_INFO_REQ )
      {
        status = MAP_gattNotifyEvent( reqTaskId, connHandle, SUCCESS, pPkt->method, &msg );
        if ( status == SUCCESS )
        {
          if ( MAP_gattGetPayload( &msg , pPkt->method ) != NULL )
          {
            *pSafeToDealloc = FALSE; // payload sent to app
          }

          if ( !ATT_WRITE_COMMAND( pPkt->method, pPkt->cmd ) && ( pServer != NULL ) )
          {
            // Set pending request
            pServer->pendingReq = pPkt->method;
          }
        }
      }
    }
    // Do not send an error response back for any command
    else if ( pPkt->cmd == FALSE )
    {
      attErrorRsp_t errorRsp;

      // Send an Error Response back
      errorRsp.reqOpcode = pPkt->method;
      errorRsp.errCode = status;

      // Set the handle
      if ( ( pPkt->method == ATT_FIND_INFO_REQ )          ||
           ( pPkt->method == ATT_FIND_BY_TYPE_VALUE_REQ ) ||
           ( pPkt->method == ATT_READ_BY_TYPE_REQ )       ||
           ( pPkt->method == ATT_READ_BY_GRP_TYPE_REQ ) )
      {
        // Set handle to the starting handle
        errorRsp.handle = msg.findInfoReq.startHandle;
      }
      else
      {
        // All requests share the handle field
        errorRsp.handle = msg.readReq.handle;
      }

      // Send an Error Response back
      if ( MAP_ATT_ErrorRsp( connHandle, &errorRsp ) != SUCCESS )
      {
        // Failed to send a response, enqueue it to retry later
        VOID MAP_gattServApp_EnqueueReTx( connHandle, ATT_ERROR_RSP,
                                          (gattMsg_t *) &errorRsp);
      }

      // We're done with this request
      status = SUCCESS;
    }
    else
    {
          /* this else clause is required, even if the
            programmer expects this will never be reached
            Fix Misra-C Required: MISRA.IF.NO_ELSE */
    }
  }

  return ( status );
}

/*********************************************************************
 * @fn      gattProcessExchangeMTUReq
 *
 * @brief   Process Exchange MTU Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pMsg - pointer to message structure
 *
 * @return  SUCCESS: Forward the request up to the application
 */
static bStatus_t gattProcessExchangeMTUReq( uint16 connHandle, attMsg_t *pMsg )
{
  VOID connHandle; // Not used here
  VOID pMsg; // Not used here

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      gattProcessFindInfoReq
 *
 * @brief   Process Find Information Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pMsg - pointer to message structure
 *
 * @return  SUCCESS: Forward the request up to the application
 *          ATT_ERR_INVALID_HANDLE: Invalid attribute handle
 *          ATT_ERR_ATTR_NOT_FOUND: Attribute not found
 *          ATT_ERR_INSUFFICIENT_RESOURCES: Out of resources
 */
static bStatus_t gattProcessFindInfoReq( uint16 connHandle, attMsg_t *pMsg )
{
  attFindInfoReq_t *pReq = &pMsg->findInfoReq;
  uint16 startHandle = pReq->startHandle;
  attFindInfoRsp_t rsp;
  uint16 maxNumPairs = 0;

  // If the starting handle greater than the ending handle or the starting
  // handle is 0x0000 then return the status code Invalid Handle
  if ( ( startHandle > pReq->endHandle ) || ( startHandle == 0 ) )
  {
    return ( ATT_ERR_INVALID_HANDLE );
  }

  // Initialize local variables
  rsp.numInfo = 0;
  rsp.pInfo = NULL;
  uint8 *pInfo = NULL;
  while ( TRUE )
  {
    // All attribute types are effectively compared as 128 bit UUIDs, even if
    // a 16 bit UUID is provided in this request or defined for an attribute.
    gattAttribute_t *pAttr = MAP_GATT_FindHandleUUID( startHandle, pReq->endHandle,
                                                  NULL, 0, NULL );
    // If the size of the UUID Filter parameter is 0 octets, then all attribute
    // types will be returned. If the size of the UUID Filter parameter is 2 or
    // 16 octets, then only attribute types with this UUID Filter will be returned.
    if ( pAttr == NULL )
    {
      // No more attributes found
      break;
    }

    // Is this the first UUID found?
    if ( rsp.numInfo == 0 )
    {
      uint16 len;

      // Allocate space for handle and UUID pairs
      pInfo = (uint8 *)MAP_GATT_bm_alloc( connHandle, ATT_FIND_INFO_RSP, GATT_MAX_MTU, &len );
      if ( pInfo == NULL )
      {
        // Couldn't get buffer for response
        return ( ATT_ERR_INSUFFICIENT_RESOURCES );
      }

      rsp.pInfo = pInfo;

      // Set the Format field using the first UUID's length
      if ( pAttr->type.len == ATT_BT_UUID_SIZE )
      {
        // A list of 1 or more handles with their 16 bit Bluetooth UUIDs
        rsp.format = ATT_HANDLE_BT_UUID_TYPE;

        // Max number of handle and 16-bit UUID pairs in a single Find Info Rsp
        maxNumPairs = len / ( 2 + ATT_BT_UUID_SIZE );
      }
      else
      {
        // A list of 1 or more handles with their 128 bit UUIDs
        rsp.format = ATT_HANDLE_UUID_TYPE;

        // Max number of handle and 128-bit UUID pairs in a single Find Info Rsp
        maxNumPairs = len / ( 2 + ATT_UUID_SIZE );
      }
    }

    // Copy handle and UUID into the response
    if ( rsp.format == ATT_HANDLE_BT_UUID_TYPE )
    {
      // Handle with its 16 bit Bluetooth UUID
      if ( pAttr->type.len != ATT_BT_UUID_SIZE )
      {
        // It's not possible to include attributes with differing UUID sizes
        // into a single response
        break;
      }

      *pInfo++ = LO_UINT16( pAttr->handle );
      *pInfo++ = HI_UINT16( pAttr->handle );

      VOID MAP_osal_memcpy( pInfo, pAttr->type.uuid, ATT_BT_UUID_SIZE );
      pInfo += ATT_BT_UUID_SIZE;
    }
    else // ATT_HANDLE_UUID_TYPE
    {
      // Handle with its 128 bit Bluetooth UUID
      if ( pAttr->type.len != ATT_UUID_SIZE )
      {
        // It's not possible to include attributes with differing UUID sizes
        // into a single response
        break;
      }

      *pInfo++ = LO_UINT16( pAttr->handle );
      *pInfo++ = HI_UINT16( pAttr->handle );

      VOID MAP_osal_memcpy( pInfo, pAttr->type.uuid, ATT_UUID_SIZE );
      pInfo += ATT_UUID_SIZE;
    }

    if ( ( ++rsp.numInfo >= maxNumPairs ) ||
         ( pAttr->handle == GATT_MAX_HANDLE ) )
    {
      break; // We're done successfully
    }

    // Update start handle and search again
    startHandle = pAttr->handle + 1;
  }

  // If no attribute is found then return the status code Attribute Not Found
  if ( rsp.numInfo == 0 )
  {
    return ( ATT_ERR_ATTR_NOT_FOUND );
  }

  // Send a Find Info Response back
  if ( MAP_ATT_FindInfoRsp( connHandle, &rsp ) != SUCCESS )
  {
    // Failed to send a response, enqueue it to retry later
    if ( MAP_gattServApp_EnqueueReTx( connHandle, ATT_FIND_INFO_RSP,
                                      (gattMsg_t *)&rsp ) != SUCCESS )
    {
      // Free buffer just allocated
      MAP_osal_bm_free( rsp.pInfo );
    }
  }

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      gattProcessFindByTypeVaueReq
 *
 * @brief   Process Find By Type Value Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pMsg - pointer to message structure
 *
 * @return  SUCCESS: Forward the request up to the application
 *          ATT_ERR_INVALID_HANDLE: Invalid attribute handle
 *          ATT_ERR_ATTR_NOT_FOUND: Attribute not found
 */
static bStatus_t gattProcessFindByTypeValueReq( uint16 connHandle, attMsg_t *pMsg )
{
  attFindByTypeValueReq_t *pReq = &pMsg->findByTypeValueReq;

  VOID connHandle; // Not used here

  // If the starting handle greater than the ending handle or the starting
  // handle is 0x0000 then return the status code Invalid Handle
  if ( ( pReq->startHandle > pReq->endHandle ) || ( pReq->startHandle == 0 ) )
  {
    return ( ATT_ERR_INVALID_HANDLE );
  }

  // Only attributes with attribute handles between and including the Starting
  // Handle parameter and the Ending Handle parameter that match the requested
  // attribute type and the attribute value will be returned.

  // All attribute types are effectively compared as 128 bit UUIDs, even if
  // a 16 bit UUID is provided in this request or defined for an attribute.
  if ( MAP_GATT_FindHandleUUID( pReq->startHandle, pReq->endHandle,
                            pReq->type.uuid, pReq->type.len, NULL ) == NULL )
  {
    // If no attribute with the given type exists within the handle range
    // then return the status code Attribute Not Found
    return ( ATT_ERR_ATTR_NOT_FOUND );
  }

  // Forward the request up to the application
  return ( SUCCESS );
}

/*********************************************************************
 * @fn      gattProcessReadByTypeReq
 *
 * @brief   Process Read By Type Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pMsg - pointer to message structure
 *
 * @return  SUCCESS: Forward the request up to the application
 *          ATT_ERR_INVALID_HANDLE: Invalid attribute handle
 *          ATT_ERR_ATTR_NOT_FOUND: Attribute not found
 */
static bStatus_t gattProcessReadByTypeReq( uint16 connHandle, attMsg_t *pMsg )
{
  attReadByTypeReq_t *pReq = &pMsg->readByTypeReq;

  VOID connHandle; // Not used here

  // If the starting handle greater than the ending handle or the starting
  // handle is 0x0000 then return the status code Invalid Handle
  if ( ( pReq->startHandle > pReq->endHandle ) || ( pReq->startHandle == 0 ) )
  {
    return ( ATT_ERR_INVALID_HANDLE );
  }

  // Only an attribute with attribute type that is the same as the Type
  // given will be returned. The attribute returned must be the attribute
  // with the lowest handle within the handle range.

  // All attribute types are effectively compared as 128 bit UUIDs, even if
  // a 16 bit UUID is provided in this request or defined for an attribute.
  if ( MAP_GATT_FindHandleUUID( pReq->startHandle, pReq->endHandle,
                            pReq->type.uuid, pReq->type.len, NULL ) == NULL )
  {
    // If no attribute with the given type exists within the handle range
    // then return the status code Attribute Not Found
    return ( ATT_ERR_ATTR_NOT_FOUND );
  }

  // Forward the request up to the application
  return ( SUCCESS );
}

/*********************************************************************
 * @fn      gattProcessReadReq
 *
 * @brief   Process Read Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pMsg - pointer to message structure
 *
 * @return  SUCCESS: Forward the request up to the application
 *          ATT_ERR_INVALID_HANDLE: Invalid attribute handle
 *          ATT_ERR_READ_NOT_PERMITTED: Attribute cannot be read
 *          ATT_ERR_INSUFFICIENT_AUTHEN: Attribute requires authentication
 *          ATT_ERR_INSUFFICIENT_KEY_SIZE: Key Size used for encrypting is insufficient
 *          ATT_ERR_INSUFFICIENT_ENCRYPT: Attribute requires encryption
 */
static bStatus_t gattProcessReadReq( uint16 connHandle, attMsg_t *pMsg )
{
  gattAttribute_t *pAttr;
  uint16 handle = pMsg->readReq.handle;
  uint16 service;

  // Make sure the handle is valid
  pAttr = MAP_GATT_FindHandle( handle, &service );
  if ( pAttr == NULL )
  {
    return ( ATT_ERR_INVALID_HANDLE );
  }

  // Forward the request up to the application (if reading allowed)
  return ( MAP_GATT_VerifyReadPermissions( connHandle, pAttr, service ) );
}

/*********************************************************************
 * @fn      gattProcessReadMultiReq
 *
 * @brief   Process Read Multiple Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pMsg - pointer to message structure
 *
 * @return  SUCCESS: Forward the request up to the application
 *          ATT_ERR_INVALID_HANDLE: Invalid attribute handle
 *          ATT_ERR_READ_NOT_PERMITTED: Attribute cannot be read
 *          ATT_ERR_INSUFFICIENT_AUTHEN: Attribute requires authentication
 *          ATT_ERR_INSUFFICIENT_KEY_SIZE: Key Size used for encrypting is insufficient
 *          ATT_ERR_INSUFFICIENT_ENCRYPT: Attribute requires encryption
 */
static bStatus_t gattProcessReadMultiReq( uint16 connHandle, attMsg_t *pMsg )
{
  attReadMultiReq_t *pReq = &pMsg->readMultiReq;

  // Make sure all the handles are valid and all attributes have sufficient
  // permissions to allow reading.
  for ( uint16 i = 0; i < pReq->numHandles; i++ )
  {
    gattAttribute_t *pAttr;
    uint16 service;
    uint8 status;

    // Make sure the handle is valid
    pAttr = MAP_GATT_FindHandle( ATT_HANDLE( pReq->pHandles, i ), &service );
    if ( pAttr == NULL )
    {
      // The handle of the first attribute causing the error
      pReq->pHandles[0] = pReq->pHandles[ATT_HANDLE_IDX( i )];
      pReq->pHandles[1] = pReq->pHandles[ATT_HANDLE_IDX( i ) + 1];

      return ( ATT_ERR_INVALID_HANDLE );
    }

    // Make sure the attribute has sufficient permissions to allow reading
    status = MAP_GATT_VerifyReadPermissions( connHandle, pAttr, service );
    if ( status != SUCCESS )
    {
      // The handle of the first attribute causing the error
      pReq->pHandles[0] = pReq->pHandles[ATT_HANDLE_IDX( i )];
      pReq->pHandles[1] = pReq->pHandles[ATT_HANDLE_IDX( i ) + 1];

      return ( status );
    }
  }

  // Forward the request up to the application
  return ( SUCCESS );
}

/*********************************************************************
 * @fn      gattProcessReadByGrpTypeReq
 *
 * @brief   Process Read By Group Type Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pMsg - pointer to message structure
 *
 * @return  SUCCESS: Forward the request up to the application
 *          ATT_ERR_INVALID_HANDLE: Invalid attribute handle
 *          ATT_ERR_UNSUPPORTED_GRP_TYPE: Group attribute type not supported
 *          ATT_ERR_ATTR_NOT_FOUND: Attribute not found
 */
static bStatus_t gattProcessReadByGrpTypeReq( uint16 connHandle, attMsg_t *pMsg )
{
  attReadByGrpTypeReq_t *pReq = &pMsg->readByGrpTypeReq;

  VOID connHandle; // Not used here

  // If the starting handle greater than the ending handle or the starting
  // handle is 0x0000 then return the status code Invalid Handle
  if ( ( pReq->startHandle > pReq->endHandle ) || ( pReq->startHandle == 0 ) )
  {
    return ( ATT_ERR_INVALID_HANDLE );
  }

  // If the Attribute Group Type is not a supported grouping attribute
  // the return the status code Unsupported Group Type
  if ( !gattPrimaryServiceType( pReq->type ) )
  {
    return ( ATT_ERR_UNSUPPORTED_GRP_TYPE );
  }

  // Only the attributes with attribute handles between and including the
  // Starting Handle and the Ending Handle with the attribute type that
  // is the same as the Attribute Group Type given will be returned.

  // All attribute types are effectively compared as 128 bit UUIDs, even if
  // a 16 bit UUID is provided in this request or defined for an attribute.
  if ( MAP_GATT_FindHandleUUID( pReq->startHandle, pReq->endHandle,
                            pReq->type.uuid, pReq->type.len, NULL ) == NULL )
  {
    // If no attribute with the given type exists within the handle range
    // then return the status code Attribute Not Found
    return ( ATT_ERR_ATTR_NOT_FOUND );
  }

  // Forward the request up to the application
  return ( SUCCESS );
}

/*********************************************************************
 * @fn      gattProcessWriteReq
 *
 * @brief   Process Write Request or Command.
 *
 * @param   connHandle - connection message was received on
 * @param   pMsg - pointer to message structure
 *
 * @return  SUCCESS: Forward the request up to the application
 *          ATT_ERR_READ_NOT_PERMITTED: Attribute cannot be written
 *          ATT_ERR_INSUFFICIENT_AUTHEN: Attribute requires authentication
 *          ATT_ERR_INSUFFICIENT_KEY_SIZE: Key Size used for encrypting is insufficient
 *          ATT_ERR_INSUFFICIENT_ENCRYPT: Attribute requires encryption
 */
static bStatus_t gattProcessWriteReq( uint16 connHandle, attMsg_t *pMsg )
{
  gattAttribute_t *pAttr;
  attWriteReq_t *pReq = &(pMsg->writeReq);
  uint16 service;

  // Make sure the handle is valid
  pAttr = MAP_GATT_FindHandle( pReq->handle, &service );
  if ( pAttr == NULL )
  {
    return ( ATT_ERR_INVALID_HANDLE );
  }

  // Forward the request up to the application (if writting allowed)
  return ( MAP_GATT_VerifyWritePermissions( connHandle, pAttr, service, pReq ) );
}

/*********************************************************************
 * @fn      gattParrseReq
 *
 * @brief   Given an ATT Method, return the appropriate parsing
 *          function
 *
 * @param   method - @ATT_METHOD_DEFINES
 *
 * @return  gattParseReq_t - pointer to parsing function
 */
static gattParseReq_t gattParseReq( uint8 method )
{
  switch( method )
  {
    case ATT_EXCHANGE_MTU_REQ:
      return MAP_ATT_ParseExchangeMTUReq;
    case ATT_FIND_INFO_REQ:
      return MAP_ATT_ParseFindInfoReq;
    case ATT_FIND_BY_TYPE_VALUE_REQ:
      return MAP_ATT_ParseFindByTypeValueReq;
    case ATT_READ_BY_TYPE_REQ:
      return MAP_ATT_ParseReadByTypeReq;
    case ATT_READ_REQ:
      return MAP_ATT_ParseReadReq;
    case ATT_READ_BLOB_REQ:
      return MAP_ATT_ParseReadBlobReq;
    case ATT_READ_MULTI_REQ:
      return MAP_ATT_ParseReadMultiReq;
    case ATT_READ_BY_GRP_TYPE_REQ:
      return MAP_ATT_ParseReadByTypeReq;
    case ATT_WRITE_REQ:
      return MAP_ATT_ParseWriteReq;
    case ATT_PREPARE_WRITE_REQ:
      return MAP_ATT_ParsePrepareWriteReq;
    case ATT_EXECUTE_WRITE_REQ:
      return MAP_ATT_ParseExecuteWriteReq;
    default:
      return NULL;
  }
}

/*********************************************************************
 * @fn      gattProcessReq
 *
 * @brief   Given an ATT Method, return the appropriate processing
 *          function
 *
 * @param   method - @ATT_METHOD_DEFINES
 *
 * @return  gattProcessReq_t - pointer to processing function
 */
static gattProcessReq_t gattProcessReq( uint8 method )
{
  switch( method )
  {
    case ATT_EXCHANGE_MTU_REQ:
      return MAP_gattProcessExchangeMTUReq;
    case ATT_FIND_INFO_REQ:
      return MAP_gattProcessFindInfoReq;
    case ATT_FIND_BY_TYPE_VALUE_REQ:
      return MAP_gattProcessFindByTypeValueReq;
    case ATT_READ_BY_TYPE_REQ:
      return MAP_gattProcessReadByTypeReq;
    case ATT_READ_REQ:
      return MAP_gattProcessReadReq;
    case ATT_READ_BLOB_REQ:
      return MAP_gattProcessReadReq;
    case ATT_READ_MULTI_REQ:
      return MAP_gattProcessReadMultiReq;
    case ATT_READ_BY_GRP_TYPE_REQ:
      return MAP_gattProcessReadByGrpTypeReq;
    case ATT_WRITE_REQ:
      return MAP_gattProcessWriteReq;
    case ATT_PREPARE_WRITE_REQ:
      return MAP_gattProcessWriteReq;
    case ATT_EXECUTE_WRITE_REQ:
      return MAP_gattProcessExecuteWriteReq;
    default:
      return NULL;
  }
}

/*********************************************************************
 * @fn      gattProcessExecuteWriteReq
 *
 * @brief   Process Execute Write Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pMsg - pointer to message structure
 *
 * @return  SUCCESS: Forward the request up to the application
 */
static bStatus_t gattProcessExecuteWriteReq( uint16 connHandle, attMsg_t *pMsg )
{
  VOID connHandle; // Not used here
  VOID pMsg; // Not used here

  // Forward the request up to the application
  return ( SUCCESS );
}

/*********************************************************************
 * @fn      gattFindService
 *
 * @brief   Find the record for a given service
 *
 * @param   handle - service handle to look for
 *
 * @return  Pointer to service record if found. NULL, otherwise.
 */
static gattService_t *gattFindService( uint16 handle )
{
  gattServiceList_t *pLoop = pServiceList;

  while ( pLoop != NULL )
  {
    if ( pLoop->service.attrs[0].handle == handle )
    {
      // Service found
      return ( &pLoop->service );
    }

    // Try next service
    pLoop = pLoop->next;
  }

  // Service not found
  return ( (gattService_t *)NULL );
}

/*********************************************************************
 * @fn          gattGetServerStatus
 *
 * @brief       Get the status for a given server.
 *
 * @param       connHandle - client connection to server
 * @param       p2pServer - pointer to server info (to be returned)
 *
 * @return      SUCCESS: No confirmation pending
 *              INVALIDPARAMETER: Invalid connection handle
 *              blePending: Confirmation pending
 *              bleTimeout: Previous transaction timed out
 */
bStatus_t gattGetServerStatus( uint16 connHandle, gattServerInfo_t **p2pServer )
{
  gattServerInfo_t *pServer;

  pServer = MAP_gattFindServerInfo( connHandle );
  if ( pServer != NULL )
  {
    if ( p2pServer != NULL )
    {
      *p2pServer = pServer;
    }

    // Make sure there's no confirmation pending or timed out with this client
    return ( TIMER_STATUS( pServer->timerId ) );
  }

  // Connection handle not found
  return ( INVALIDPARAMETER );
}

/*********************************************************************
 * @fn      attFindServerInfo
 *
 * @brief   Find the server info.  Uses the connection handle to search
 *          the server info table.
 *
 * @param   connHandle - connection handle.
 *
 * @return  a pointer to the found item. NULL, otherwise.
 */
static gattServerInfo_t *gattFindServerInfo( uint16 connHandle )
{
  uint8 i;

  for ( i = 0; i < gattNumConns; i++ )
  {
    if ( serverInfoTbl[i].connHandle == connHandle )
    {
      // Entry found
      return ( &serverInfoTbl[i] );
    }
  }

  return ( (gattServerInfo_t *)NULL );
}

/*********************************************************************
 * @fn      gattResetServerInfo
 *
 * @brief   Reset the server info.
 *
 * @param   pServer - pointer to server info.
 *
 * @return  void
 */
static void gattResetServerInfo( gattServerInfo_t *pServer )
{
  // Cancel the confirmation timer
  MAP_gattStopTimer( &pServer->timerId );

  // Reset confirmation info
  pServer->taskId = INVALID_TASK_ID;
}

/*********************************************************************
 * @fn      attServertStartTimer
 *
 * @brief   Start a server timer to expire in n seconds.
 *
 * @param   pData - data to be passed in to callback function
 * @param   timeout - in milliseconds.
 * @param   pTimerId - will point to new timer Id (if not null)
 *
 * @return  void
 */
static void gattServerStartTimer( uint8 *pData, uint16 timeout, uint8 *pTimerId )
{
  MAP_gattStartTimer( MAP_gattServerHandleTimerCB, pData, timeout, pTimerId );
}

/*********************************************************************
 * @fn      gattServerNotifyTxCB
 *
 * @brief   Notify GATT Server about an outgoing ATT message to a client.
 *
 * @param   connHandle - client's connection handle.
 * @param   opcode - opcode of outgoing message.
 *
 * @return  void
 */
static void gattServerNotifyTxCB( uint16 connHandle, uint8 opcode )
{
  gattServerInfo_t *pServer = MAP_gattFindServerInfo( connHandle );
  if ( ( pServer != NULL ) && ( opcode == ( pServer->pendingReq + 1 ) ) )
  {
    // Reset pending request without touching the FCV bit
    pServer->pendingReq &= ATT_FCV_BIT;
  }
}

/*********************************************************************
 * @fn      gattServerHandleTimerCB
 *
 * @brief   Handle a callback for a timer that has just expired.
 *
 * @param   pData - pointer to timer data
 *
 * @return  void
 */
static void gattServerHandleTimerCB( uint8 *pData )
{
  gattServerInfo_t *pServer = (gattServerInfo_t *)pData;

  // Response timer has expired
  if ( ( pServer != NULL ) && TIMER_VALID( pServer->timerId ) )
  {
    // Notify the application about the timeout
    VOID MAP_gattNotifyEvent( pServer->taskId, pServer->connHandle, bleTimeout,
                          ATT_HANDLE_VALUE_CFM, NULL );

    // Timer has expired. If a transaction has not completed before it times
    // out, then this transaction shall be considered to have failed. No more
    // attribute protocol requests, commands, indications or notifications
    // shall be sent to the target device on this ATT Bearer.
    pServer->timerId = TIMEOUT_TIMER_ID;

    // Reset confirmation info
    pServer->taskId = INVALID_TASK_ID;
  }
}

/*********************************************************************
 * @fn          gattServerHandleConnStatusCB
 *
 * @brief       GATT link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      void
 */
static void gattServerHandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  gattServerInfo_t *pServer = NULL;

  // Check to see if this is loopback connection
  if ( connHandle == LINKDB_CONNHANDLE_LOOPBACK )
  {
    return;
  }

  if ( changeType == LINKDB_STATUS_UPDATE_NEW )
  {
    // A new connection has been made
    pServer = MAP_gattFindServerInfo( connHandle );
    if ( pServer == NULL )
    {
      // Entry not found; add it to the server table
      pServer = MAP_gattFindServerInfo( LINKDB_CONNHANDLE_INVALID );
      if ( pServer != NULL )
      {
        // Empty entry found
        pServer->connHandle = connHandle;
      }
    }

    // We're done here!
    return;
  }

  if ( changeType == LINKDB_STATUS_UPDATE_REMOVED )
  {
    pServer = MAP_gattFindServerInfo( connHandle );
    if ( pServer != NULL )
    {
      // Entry found; remove it from the server table
      pServer->connHandle = LINKDB_CONNHANDLE_INVALID;
    }
  }
  else if ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS )
  {
    // Check to see if the connection has dropped
    if ( !MAP_linkDB_State( connHandle, LINK_CONNECTED ) )
    {
      pServer = MAP_gattFindServerInfo( connHandle );
    }
  }
  else
  {
        /* this else clause is required, even if the
           programmer expects this will never be reached
           Fix Misra-C Required: MISRA.IF.NO_ELSE */
  }

  // Connection has dropped; notify the application
  if ( pServer != NULL )
  {
    if ( pServer->timerId != INVALID_TIMER_ID )
    {
      if ( pServer->timerId != TIMEOUT_TIMER_ID )
      {
        // Notify the application about the link disconnect
        VOID MAP_gattNotifyEvent( pServer->taskId, connHandle, bleNotConnected,
                              ATT_HANDLE_VALUE_CFM, NULL );
      }

      // Reset server info
      MAP_gattResetServerInfo( pServer );

      // Just in case if we've timed out waiting for a confirmation
      pServer->timerId = INVALID_TIMER_ID;
    }

    // Reset pending request info
    pServer->pendingReq = 0;
  }
}

/*******************************************************************************
 * This function returns the number of attributes that has GATT_CLIENT_CHAR_CFG_UUID
 *
 * Public function defined in gatt.h.
 */
uint16_t GATT_GetNumIndNotiHandles(uint16_t minHandle, uint16_t maxHandle)
{
  gattServiceList_t *pServiceLoop = pServiceList;
  gattAttribute_t *pAttr;
  uint8_t counter = 0;
  uint8_t pUUID[ATT_BT_UUID_SIZE] = { LO_UINT16( GATT_CLIENT_CHAR_CFG_UUID ),
                                      HI_UINT16( GATT_CLIENT_CHAR_CFG_UUID ) };
  uint16_t len = ATT_BT_UUID_SIZE;
  uint16_t numAtt;

  if ( (minHandle == GATT_INVALID_HANDLE) || (maxHandle == GATT_INVALID_HANDLE) )
  {
      // At least one of the handles are invalid. Return 0 indicates there isn't any relevant attribute
      return 0;
  }

  // Loop through all services
  while ( pServiceLoop != NULL )
  {
    // Loop through all attributes
    for ( numAtt = 0; numAtt < pServiceLoop->service.numAttrs; numAtt++ )
    {
      pAttr = &(pServiceLoop->service.attrs[numAtt]);

      // Compare UUIDs with notification/indication UUID
      if( MAP_ATT_CompareUUID( pAttr->type.uuid, pAttr->type.len,
                               pUUID, len ) )
      {
        if ( (pAttr->handle >= minHandle) && (pAttr->handle <= maxHandle) )
        {
          counter++;
        }
      }
    }

    // Try next service
    pServiceLoop = pServiceLoop->next;
  }

  return ( counter );
}

/*******************************************************************************
 * This function fill a given buffer with the handles and values of characteristic
 * with GATT_CLIENT_CHAR_CFG_UUID
 *
 * Public function defined in gatt.h.
 */
uint8_t GATT_SetIndNotiRegData(uint16_t connHandle,
                               uint16_t minHandle,
                               uint16_t maxHandle,
                               uint16_t numAtts,
                               cccValues_t *pCccData)
{
  gattServiceList_t *pServiceLoop = pServiceList;
  gattCharCfg_t *cccTable;
  gattAttribute_t *pAttr;
  gattCharCfg_t *pItem;
  uint8_t maxNumConns = linkDB_NumConns();
  uint16_t numAtt;
  uint8_t numConn;
  uint16_t cccIndex = 0;
  uint8_t pUUID[ATT_BT_UUID_SIZE] = { LO_UINT16( GATT_CLIENT_CHAR_CFG_UUID ),
                                      HI_UINT16( GATT_CLIENT_CHAR_CFG_UUID ) };
  uint16_t len = ATT_BT_UUID_SIZE;

  if ( NULL == pCccData )
  {
      return FAILURE;
  }

  if ( (minHandle == GATT_INVALID_HANDLE) || (maxHandle == GATT_INVALID_HANDLE) )
  {
      // At least one of the handles are invalid. Return 0 indicates there isn't any relevant attribute
      return FAILURE;
  }

  // Loop through all services
  while ( pServiceLoop != NULL )
  {
    // Loop through all attributes
    for ( numAtt = 0; numAtt < pServiceLoop->service.numAttrs; numAtt++ )
    {
      pAttr = &(pServiceLoop->service.attrs[numAtt]);

      // Compare UUIDs with notification/indication UUID
      if( MAP_ATT_CompareUUID( pAttr->type.uuid, pAttr->type.len,
                               pUUID, len ) )
      {
        if ( (pAttr->handle >= minHandle) && (pAttr->handle <= maxHandle) )
        {
            cccTable = (gattCharCfg_t *)(*(uint8**)(pAttr->pValue));

            // Search for the correct connection handle
            for ( numConn = 0; numConn < maxNumConns; numConn++ )
            {
              pItem = &cccTable[numConn];
              // Check that this is the connection handle needed and that it doesn't
              // exceeds the expected number of attributes expected
              if ( (pItem->connHandle == connHandle) && (cccIndex != numAtts) )
              {
                pCccData[cccIndex].handle = pAttr->handle;
                pCccData[cccIndex].value = pItem->value;
                if (cccIndex < UINT16_MAX)
                {
                  cccIndex++;
                } 
              }
            }
        }
      }
    }

    // Try next service
    pServiceLoop = pServiceLoop->next;
  }

  return ( SUCCESS );
}

/*******************************************************************************
 * This function returns the server pending request value
 *
 * Public function defined in gatt.h.
 */
uint8_t GATT_GetServerPendingReq(uint16_t connHandle)
{
  gattServerInfo_t *pServer = NULL;

  // Make sure there's no pending request to be processed by the app
  pServer = MAP_gattFindServerInfo( connHandle );

  if ( pServer != NULL )
  {
    return pServer->pendingReq;
  }

  return GATT_SERVER_NOT_FOUND;
}

/*********************************************************************
 * @fn          GATT_GetNextHandle
 *
 * @brief       Return the next available attribute handle.
 *
 * @param       none
 *
 * @return      next attribute handle
 */
uint16 GATT_GetNextHandle( void )
{
  uint16 result;
  uint32_t invokeStatus;

  if (BLE_invokeIfRequired((void *)&GATT_GetNextHandle, &invokeStatus))
  {
    result = (uint16)invokeStatus;
  }
  else {
    result = nextHandle;
  }
  return (result);
}

/*********************************************************************
 * @fn          GATT_SetNextHandle
 *
 * @brief       Set the next available attribute handle.
 *
 * @param       handle - next attribute handle
 *
 * @return      void
 */
void GATT_SetNextHandle( uint16 handle )
{
  if ( handle >= nextHandle )
  {
    // Set next available attribute handle
    nextHandle = handle;
  }
}

/****************************************************************************
****************************************************************************/
