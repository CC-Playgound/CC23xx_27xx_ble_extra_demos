/******************************************************************************

 @file  simple_gatt_profile.c

 @brief This file contains the Simple GATT profile sample GATT service profile
 for use with the BLE sample application.

 Group: WCS, BTS
 Target Device: cc23xx

 ******************************************************************************

 Copyright (c) 2010-2024, Texas Instruments Incorporated
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
#include "ti/ble/stack_util/icall/app/icall.h"
#include <string.h>
/* This Header file contains all BLE API and icall structure definition */
#include "ti/ble/stack_util/icall/app/icall_ble_api.h"

#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <ti/ble/profiles/simple_gatt/simple_gatt_profile.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
void SimpleGattProfile_callback(uint8 paramID);
void SimpleGattProfile_invokeFromFWContext(char *pData);

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0xFFF0
GATT_BT_UUID(simpleGattProfile_ServUUID, SIMPLEGATTPROFILE_SERV_UUID);

// Characteristic 1 UUID: 0xFFF1
GATT_BT_UUID(simpleGattProfile_char1UUID, SIMPLEGATTPROFILE_CHAR1_UUID);

// Characteristic 2 UUID: 0xFFF2
GATT_BT_UUID(simpleGattProfile_char2UUID, SIMPLEGATTPROFILE_CHAR2_UUID);

// Characteristic 3 UUID: 0xFFF3
GATT_BT_UUID(simpleGattProfile_char3UUID, SIMPLEGATTPROFILE_CHAR3_UUID);

// Characteristic 4 UUID: 0xFFF4
GATT_BT_UUID(simpleGattProfile_char4UUID, SIMPLEGATTPROFILE_CHAR4_UUID);

// Characteristic 5 UUID: 0xFFF5
GATT_BT_UUID(simpleGattProfile_char5UUID, SIMPLEGATTPROFILE_CHAR5_UUID);

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static SimpleGattProfile_CBs_t *simpleGattProfile_appCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple GATT Profile Service attribute
static const gattAttrType_t simpleGattProfile_Service = {
    ATT_BT_UUID_SIZE, simpleGattProfile_ServUUID};

// Simple GATT Profile Characteristic 1 Properties
static uint8 simpleGattProfile_Char1Props =
    GATT_PROP_READ | GATT_PROP_WRITE_NO_RSP;

// Characteristic 1 Value
static uint8 simpleGattProfile_Char1[247];

// Simple GATT Profile Characteristic 1 User Description
static uint8 simpleGattProfile_Char1UserDesp[17] = "Characteristic 1";

// Simple GATT Profile Characteristic 2 Properties
static uint8 simpleGattProfile_Char2Props = GATT_PROP_READ;

// Characteristic 2 Value
static uint8 simpleGattProfile_Char2 = 0;

// Simple Profile Characteristic 2 User Description
static uint8 simpleGattProfile_Char2UserDesp[17] = "Characteristic 2";

// Simple GATT Profile Characteristic 3 Properties
static uint8 simpleGattProfile_Char3Props = GATT_PROP_WRITE;

// Characteristic 3 Value
static uint8 simpleGattProfile_Char3 = 0;

// Simple GATT Profile Characteristic 3 User Description
static uint8 simpleGattProfile_Char3UserDesp[17] = "Characteristic 3";

// Simple GATT Profile Characteristic 4 Properties
static uint8 simpleGattProfile_Char4Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic 4 Value
static uint8 simpleGattProfile_Char4 = 0;

// Simple GATT Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *simpleGattProfile_Char4Config;

// Simple GATT Profile Characteristic 4 User Description
static uint8 simpleGattProfile_Char4UserDesp[17] = "Characteristic 4";

// Simple GATT Profile Characteristic 5 Properties
static uint8 simpleGattProfile_Char5Props = GATT_PROP_READ;

// Characteristic 5 Value
static uint8 simpleGattProfile_Char5[SIMPLEGATTPROFILE_CHAR5_LEN] = {0, 0, 0, 0,
                                                                     0};

// Simple GATT Profile Characteristic 5 User Description
static uint8 simpleGattProfile_Char5UserDesp[17] = "Characteristic 5";

/*********************************************************************
 * Profile Attributes - Table
 */

#include <app_main.h>

static gattAttribute_t simpleGattProfile_attrTbl[] = {
    /*------------------type-----------------*/
    /*-----------permissions-----------*/ /*-----------------pValue----------------*/
    // Simple Profile Service
    GATT_BT_ATT(primaryServiceUUID, GATT_PERMIT_READ,
                (uint8 *)&simpleGattProfile_Service),

    // Characteristic 1 Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &simpleGattProfile_Char1Props),
    // Characteristic Value 1
    GATT_BT_ATT(simpleGattProfile_char1UUID,
                GATT_PERMIT_READ | GATT_PERMIT_WRITE, simpleGattProfile_Char1),
    // Characteristic 1 User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ,
                simpleGattProfile_Char1UserDesp),

    // Characteristic 2 Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &simpleGattProfile_Char2Props),
    // Characteristic Value 2
    GATT_BT_ATT(simpleGattProfile_char2UUID, GATT_PERMIT_READ,
                &simpleGattProfile_Char2),
    // Characteristic 2 User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ,
                simpleGattProfile_Char2UserDesp),

    // Characteristic 3 Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &simpleGattProfile_Char3Props),
    // Characteristic Value 3
    GATT_BT_ATT(simpleGattProfile_char3UUID, GATT_PERMIT_WRITE,
                &simpleGattProfile_Char3),
    // Characteristic 3 User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ,
                simpleGattProfile_Char3UserDesp),

    // Characteristic 4 Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &simpleGattProfile_Char4Props),
    // Characteristic Value 4
    GATT_BT_ATT(simpleGattProfile_char4UUID, 0, &simpleGattProfile_Char4),
    // Characteristic 4 configuration
    GATT_BT_ATT(clientCharCfgUUID,
                GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                (uint8 *)&simpleGattProfile_Char4Config),
    // Characteristic 4 User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ,
                simpleGattProfile_Char4UserDesp),

    // Characteristic 5 Declaration
    // GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &simpleGattProfile_Char5Props),
    // // Characteristic Value 5
    // GATT_BT_ATT(simpleGattProfile_char5UUID, GATT_PERMIT_AUTHEN_READ,
    //             simpleGattProfile_Char5),
    // // Characteristic 5 User Description
    // GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ,
    //             simpleGattProfile_Char5UserDesp),
};
/*********************************************************************
 * LOCAL FUNCTIONS
 */
bStatus_t SimpleGattProfile_readAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr, uint8_t *pValue,
                                       uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method);
bStatus_t SimpleGattProfile_writeAttrCB(uint16_t connHandle,
                                        gattAttribute_t *pAttr, uint8_t *pValue,
                                        uint16_t len, uint16_t offset,
                                        uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple GATT Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
const gattServiceCBs_t simpleGattProfile_CBs = {
    SimpleGattProfile_readAttrCB,  // Read callback function pointer
    SimpleGattProfile_writeAttrCB, // Write callback function pointer
    NULL                           // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleGattProfile_addService
 *
 * @brief   This function initializes the Simple GATT Server service
 *          by registering GATT attributes with the GATT server.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t SimpleGattProfile_addService(void) {
  uint8 status = SUCCESS;
  memset(simpleGattProfile_Char1, 0xAA, sizeof(simpleGattProfile_Char1));

  // Allocate Client Characteristic Configuration table
  simpleGattProfile_Char4Config =
      (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS);
  if (simpleGattProfile_Char4Config == NULL) {
    return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(LINKDB_CONNHANDLE_INVALID,
                          simpleGattProfile_Char4Config);

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(
      simpleGattProfile_attrTbl, GATT_NUM_ATTRS(simpleGattProfile_attrTbl),
      GATT_MAX_ENCRYPT_KEY_SIZE, &simpleGattProfile_CBs);

  // Return status value
  return (status);
}

/*********************************************************************
 * @fn      SimpleGattProfile_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   appCallbacks - pointer to application callback.
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t
SimpleGattProfile_registerAppCBs(SimpleGattProfile_CBs_t *appCallbacks) {
  if (appCallbacks) {
    simpleGattProfile_appCBs = appCallbacks;

    return (SUCCESS);
  } else {
    return (bleAlreadyInRequestedMode);
  }
}

/*********************************************************************
 * @fn      SimpleGattProfile_setParameter
 *
 * @brief   Set a Simple GATT Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *                  the parameter ID and WILL be cast to the appropriate
 *                  data type (example: data type of uint16 will be cast to
 *                  uint16 pointer).
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */

#include "ti/ble/app_util/menu/menu_module.h"

#define DSS_NOTI_HDR_SIZE (ATT_OPCODE_SIZE + 2)
bStatus_t SendNotification(uint8 *pValue, uint16 len, uint8_t charID) {
  bStatus_t status = SUCCESS;
  gattAttribute_t *pAttr = NULL;
  attHandleValueNoti_t noti = {0};
  linkDBInfo_t connInfo = {0};
  uint16 offset = 0;
  uint8 i = 0;

  // Verify input parameters
  if (pValue == NULL) {
    return (INVALIDPARAMETER);
  }

  if (1) {
    // Find the characteristic value attribute
    pAttr = GATTServApp_FindAttr(simpleGattProfile_attrTbl,
                                 GATT_NUM_ATTRS(simpleGattProfile_attrTbl),
                                 &simpleGattProfile_Char4);
  } else {
    // Find the characteristic value attribute
    // pAttr = GATTServApp_FindAttr(simpleGattProfile_attrTbl,
    //                              GATT_NUM_ATTRS(simpleGattProfile_attrTbl),
    //                              simpleGattProfile_Char1);
  }

  if (pAttr != NULL) {

    // Check the ccc value for each BLE connection
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++) {
      gattCharCfg_t *pItem = &(simpleGattProfile_Char4Config[i]);
      ;
      if (1) {
        pItem = &(simpleGattProfile_Char4Config[i]);
      } else {
        // pItem = &(simpleGattProfile_Char1Config[i]);
      }

      // If the connection has register for notifications
      if ((pItem->connHandle != LINKDB_CONNHANDLE_INVALID) &&
          (pItem->value == GATT_CLIENT_CFG_NOTIFY)) {
        // Find out what the maximum MTU size is for each connection
        status = linkDB_GetInfo(pItem->connHandle, &connInfo);
        offset = 0;

        while (status != bleTimeout && status != bleNotConnected &&
               len > offset) {
          // Determine allocation size
          uint16_t allocLen = (len - offset);
          if (allocLen > (connInfo.MTU - DSS_NOTI_HDR_SIZE)) {
            // If len > MTU split data to chunks of MTU size
            allocLen = connInfo.MTU - DSS_NOTI_HDR_SIZE;
          }
          // MenuModule_printf(0, 0, "allocLen: %d", allocLen);

          noti.len = allocLen;
          noti.pValue = (uint8 *)GATT_bm_alloc(
              pItem->connHandle, ATT_HANDLE_VALUE_NOTI, allocLen, 0);

          // *(noti.pValue + noti.len -1) = (uint8_t)noti.len;
          if (noti.pValue != NULL) {
            // If allocation was successful, copy out data and send it
            memcpy(noti.pValue, pValue + offset, noti.len);
            noti.handle = pAttr->handle;

            // Send the data over BLE notifications
            status = GATT_Notification(pItem->connHandle, &noti, FALSE);
            // status = GATT_Indication( pItem->connHandle, (attHandleValueInd_t
            // *)&noti, FALSE, BLEAppUtil_getSelfEntity());

            // If unable to send the data, free allocated buffers and return
            if (status != SUCCESS) {
              GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);

              // Failed to send notification, print error message
              // MenuModule_printf(
              //         APP_MENU_PROFILE_STATUS_LINE4,
              //         0,
              //         "Failed to send notification - Error: "
              //         MENU_MODULE_COLOR_RED "%d " MENU_MODULE_COLOR_RESET,
              //         status);
            } else {
              // Increment data offset
              offset += allocLen;

              // Notification sent
              // MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE4, 0,
              //                   "Notification sent");
            }
          } else {
            status = bleNoResources;
          }
        } // End of while
      }
    } // End of for
  } // End of if

  // Return status value
  return (status);
}

ICall_heapStats_t heapStats;
uint8_t _t = 0;

void simpleSendNotification(uint8_t dataLen) {
  attHandleValueNoti_t noti = {0};
  gattAttribute_t *pAttr = NULL;
  noti.len = dataLen; 
  noti.pValue = (uint8 *)GATT_bm_alloc(0, ATT_HANDLE_VALUE_NOTI, noti.len, 0);

  pAttr = GATTServApp_FindAttr(simpleGattProfile_attrTbl,
                                 GATT_NUM_ATTRS(simpleGattProfile_attrTbl),
                                 &simpleGattProfile_Char4);
  GPIO_toggle(15);
  if (noti.pValue != NULL) {
    memset(noti.pValue, _t, noti.len);
    *(noti.pValue+noti.len - 1) = _t++;
    noti.handle = pAttr->handle;
    bStatus_t status = GATT_Notification(0, &noti, FALSE);
    // If unable to send the data, free allocated buffers and return
    if (status != SUCCESS) {
      GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
    }
    GPIO_write(14, 0);
    return;
  }
  GPIO_write(14, 1);
}

uint8_t t = 2;


bStatus_t SimpleGattProfile_setParameter(uint8 param, uint8 len, void *value) {
  attHandleValueNoti_t noti = {0};
  gattAttribute_t *pAttr = NULL;

  bStatus_t status = SUCCESS;

  switch (param) {
  case SIMPLEGATTPROFILE_CHAR1:
    // if ( len == sizeof ( uint8 ) )
    // {
    //   simpleGattProfile_Char1 = *((uint8*)value);
    // }
    // else
    // {
    //   status = bleInvalidRange;
    // }
    break;

  case SIMPLEGATTPROFILE_CHAR2:
    if (len == sizeof(uint8)) {
      simpleGattProfile_Char2 = *((uint8 *)value);
    } else {
      status = bleInvalidRange;
    }
    break;

  case SIMPLEGATTPROFILE_CHAR3:
    if (len == sizeof(uint8)) {
      simpleGattProfile_Char3 = *((uint8 *)value);
    } else {
      status = bleInvalidRange;
    }
    break;

  case SIMPLEGATTPROFILE_CHAR4:
    if (len == sizeof(uint8)) {
      simpleGattProfile_Char4 = *((uint8 *)value);
      // See if Notification has been enabled
      GATTServApp_ProcessCharCfg(
          simpleGattProfile_Char4Config, &simpleGattProfile_Char4, FALSE,
          simpleGattProfile_attrTbl, GATT_NUM_ATTRS(simpleGattProfile_attrTbl),
          INVALID_TASK_ID, SimpleGattProfile_readAttrCB);
    } else {
      status = bleInvalidRange;
    }
    break;

  case SIMPLEGATTPROFILE_CHAR5:
    if (len == SIMPLEGATTPROFILE_CHAR5_LEN) {
      VOID memcpy(simpleGattProfile_Char5, value, SIMPLEGATTPROFILE_CHAR5_LEN);
    } else {
      status = bleInvalidRange;
    }
    break;

  default:
    status = INVALIDPARAMETER;
    break;
  }

  // Return status value
  return (status);
}

/*********************************************************************
 * @fn      SimpleGattProfile_getParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleGattProfile_getParameter(uint8 param, void *value) {
  bStatus_t status = SUCCESS;
  switch (param) {
  case SIMPLEGATTPROFILE_CHAR1:
    // *((uint8*)value) = simpleGattProfile_Char1;
    VOID memcpy(value, simpleGattProfile_Char1,
                sizeof(simpleGattProfile_Char1));
    break;

  case SIMPLEGATTPROFILE_CHAR2:
    *((uint8 *)value) = simpleGattProfile_Char2;
    break;

  case SIMPLEGATTPROFILE_CHAR3:
    *((uint8 *)value) = simpleGattProfile_Char3;
    break;

  case SIMPLEGATTPROFILE_CHAR4:
    *((uint8 *)value) = simpleGattProfile_Char4;
    break;

  case SIMPLEGATTPROFILE_CHAR5:
    VOID memcpy(value, simpleGattProfile_Char5, SIMPLEGATTPROFILE_CHAR5_LEN);
    break;

  default:
    status = INVALIDPARAMETER;
    break;
  }

  // Return status value
  return (status);
}

/*********************************************************************
 * @fn          SimpleGattProfile_readAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
bStatus_t SimpleGattProfile_readAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr, uint8_t *pValue,
                                       uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method) {
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0) {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  if (pAttr->type.len == ATT_BT_UUID_SIZE) {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch (uuid) {
    // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
    // gattserverapp handles those reads

    // characteristics 1 and 2 have read permissions
    // characteritisc 3 does not have read permissions; therefore it is not
    //   included here
    // characteristic 4 does not have read permissions, but because it
    //   can be sent as a notification, it is included here
    case SIMPLEGATTPROFILE_CHAR1_UUID:
      *pLen = sizeof(simpleGattProfile_Char1);
      VOID memcpy(pValue, pAttr->pValue, sizeof(simpleGattProfile_Char1));
      break;
    case SIMPLEGATTPROFILE_CHAR2_UUID:
    case SIMPLEGATTPROFILE_CHAR4_UUID:
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
      break;

    case SIMPLEGATTPROFILE_CHAR5_UUID:
      *pLen = SIMPLEGATTPROFILE_CHAR5_LEN;
      VOID memcpy(pValue, pAttr->pValue, SIMPLEGATTPROFILE_CHAR5_LEN);
      break;

    default:
      // Should never get here! (characteristics 3 and 4 do not have read
      // permissions)
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }
  } else {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  // Return status value
  return (status);
}
uint8_t isEnableNotify = 0;

uint32_t gNotiCounter = 0;
uint32_t gInstantBits = 0;
float gInstantSpeed = 0;
uint32_t gTempTimeTicks = 0;
uint8_t gValue = 0;
uint8_t gLen = 0;

/*********************************************************************
 * @fn      SimpleGattProfile_writeAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
bStatus_t SimpleGattProfile_writeAttrCB(uint16_t connHandle,
                                        gattAttribute_t *pAttr, uint8_t *pValue,
                                        uint16_t len, uint16_t offset,
                                        uint8_t method) {
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;

  if (pAttr->type.len == ATT_BT_UUID_SIZE) {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch (uuid) {
    case SIMPLEGATTPROFILE_CHAR1_UUID:
    {
      // MenuModule_printf(0, 0, "Write callback: %02x, Len: %d ", pValue[0], len);
      if (offset == 0) {
         if (status == SUCCESS) {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;
        // *pCurValue = pValue[0];
        memset(pCurValue, pValue[0], sizeof(simpleGattProfile_Char1));
         }
      } else {
        status = ATT_ERR_ATTR_NOT_LONG;
        MenuModule_printf(0, 0, "C->P: Error!");
      }
      gNotiCounter++;

      gInstantBits += len * 8;
      gValue = pValue[0];
      gLen = len;

      // if(! (gNotiCounter % 1000))
      // {

      //   uint32_t _nowTicks = ClockP_getSystemTicks();
      //   uint32_t _time = _nowTicks - gTempTimeTicks;
      //   float kbps = ((float)gInstantBits / (float)_time) * 1000.0f;
      //   // MenuModule_printf(0, 0, "_nowTicks: %d", _time);
      //   MenuModule_printf(0, 0, "C->P: Instant speed: %d bits %f kbps; Data Len: %d; time(us): %d Data: 0x%02x", gInstantBits, kbps, len, _time, pValue[0]);
      //   gInstantBits = 0;
      //   gTempTimeTicks = _nowTicks;
      // }
      notifyApp = SIMPLEGATTPROFILE_CHAR1;
    }
    break;
    case SIMPLEGATTPROFILE_CHAR3_UUID: {
      // Validate the value
      //  Make sure it's not a blob oper
      if (offset == 0) {
        if (len != 1) {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      } else {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if (status == SUCCESS) {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;
        *pCurValue = pValue[0];

        if (pAttr->pValue == simpleGattProfile_Char1) {
          notifyApp = SIMPLEGATTPROFILE_CHAR1;
        } else {
          notifyApp = SIMPLEGATTPROFILE_CHAR3;
        }
      }
    } break;

    case GATT_CLIENT_CHAR_CFG_UUID:

      status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                              offset, GATT_CLIENT_CFG_NOTIFY);
      isEnableNotify = pValue[0];
      
      // notify the App that a change has occurred in Char 4
      notifyApp = SIMPLEGATTPROFILE_CHAR4;
      break;

    default:
      // Should never get here! (characteristics 2 and 4 do not have write
      // permissions)
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }
  } else {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a characteristic value changed then callback function to notify
  // application of change
  if ((notifyApp != 0xFF) && simpleGattProfile_appCBs &&
      simpleGattProfile_appCBs->pfnSimpleGattProfile_Change) {
    SimpleGattProfile_callback(notifyApp);
  }

  // Return status value
  return (status);
}

/*********************************************************************
 * @fn      SimpleGattProfile_callback
 *
 * @brief   This function will be called from the BLE App Util module
 *          context.
 *          Calling the application callback
 *
 * @param   pData - data
 *
 * @return  None
 */
void SimpleGattProfile_callback(uint8 paramID) {
  char *pData = ICall_malloc(sizeof(char));

  if (pData == NULL) {
    return;
  }

  pData[0] = paramID;

  BLEAppUtil_invokeFunction(SimpleGattProfile_invokeFromFWContext, pData);
}

/*********************************************************************
 * @fn      SimpleGattProfile_invokeFromFWContext
 *
 * @brief   This function will be called from the BLE App Util module
 *          context.allback
 *
 * @param   pData - data
 *
 * @return  None
 */
void SimpleGattProfile_invokeFromFWContext(char *pData) {
  simpleGattProfile_appCBs->pfnSimpleGattProfile_Change(pData[0]);
}

/***************************************************
 *          Calling the application c******************
 *********************************************************************/
