/******************************************************************************

@file  app_data.c

@brief This file contains the application data functionality

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2025, Texas Instruments Incorporated
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
#include <string.h>
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti/ble/app_util/menu/menu_module.h"
#include <app_main.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************

static void GATT_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

#ifdef GATT_SERVICE_DISCOVERY
static uint8_t display_index = 1;
#endif // #ifdef GATT_SERVICE_DISCOVERY

// Events handlers struct, contains the handlers and event masks
// of the application data module
BLEAppUtil_EventHandler_t dataGATTHandler =
{
    .handlerType    = BLEAPPUTIL_GATT_TYPE,
    .pEventHandler  = GATT_EventHandler,
    .eventMask      = BLEAPPUTIL_ATT_FLOW_CTRL_VIOLATED_EVENT |
#ifdef GATT_SERVICE_DISCOVERY
                      BLEAPPUTIL_ATT_FIND_BY_TYPE_VALUE_RSP |
                      BLEAPPUTIL_ATT_READ_BY_TYPE_RSP |
#endif // #ifdef GATT_SERVICE_DISCOVERY
                      BLEAPPUTIL_ATT_MTU_UPDATED_EVENT
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      GATT_EventHandler
 *
 * @brief   The purpose of this function is to handle GATT events
 *          that rise from the GATT and were registered in
 *          @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void GATT_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  gattMsgEvent_t *gattMsg = ( gattMsgEvent_t * )pMsgData;
  switch ( gattMsg->method )
  {
      case ATT_FLOW_CTRL_VIOLATED_EVENT:
      {
          MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "GATT status: ATT flow control is violated");
      }
      break;

      case ATT_MTU_UPDATED_EVENT:
      {
          MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "GATT status: ATT MTU update to %d",
                            gattMsg->msg.mtuEvt.MTU);
#ifdef GATT_SERVICE_DISCOVERY
          /* SimpleLink Service **/
          uint8_t uuid[2] = {0xF0, 0xFF};
          /** GATT Service Discovery **/
          bStatus_t bstatus = GATT_DiscPrimaryServiceByUUID(linkDB_NumActive() - 1, // only support 1 connection
                                                            uuid,
                                                            2,
                                                            BLEAppUtil_getSelfEntity());
          MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: GATT_DiscPrimaryServiceByUUID = "
                            MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                            bstatus);
#endif // #ifdef GATT_SERVICE_DISCOVERY
      }
      break;

#ifdef GATT_SERVICE_DISCOVERY
    case ATT_FIND_BY_TYPE_VALUE_RSP:
    {
        static uint16_t lastStart;
        static uint16_t lastEnd;
        static uint8_t gattServiceFound = 0;
        attFindByTypeValueRsp_t att = (attFindByTypeValueRsp_t)gattMsg->msg.findByTypeValueRsp;
        // This event will be invoked twice, one with service found, the other with Attribute not found
        // Set gattServiceFound flag to 1 in first one, start characteristic discovery in second one
        // att.numInfo > 0 means target service is found
        if (att.numInfo > 0)
        {
            for (uint8_t i = 0; i < att.numInfo; i++)
            {
                uint16_t attHandle = BUILD_UINT16(att.pHandlesInfo[i * 4], att.pHandlesInfo[i * 4 + 1]);
                uint16_t endHandle = BUILD_UINT16(att.pHandlesInfo[i * 4 + 2], att.pHandlesInfo[i * 4 + 3]);
                lastStart = attHandle;
                lastEnd = endHandle;
                // This event will be invoked twice, one with service found, the other with Attribute not found
                // Set gattServiceFound flag to 1 in first one, start characteristic discovery in second one
                gattServiceFound = 1;
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "GATT service found: handle 0x%x", attHandle);
            }
        }
        // att.numInfo = 0 mean attribute not found, that's the end of service discovery
        else
        {
            // Target service is found in previous event
            if(gattServiceFound == 1)
            {
                bStatus_t bstatus = GATT_DiscAllChars(gattMsg->connHandle,
                                                      lastStart,
                                                      lastEnd,
                                                      BLEAppUtil_getSelfEntity());
                MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: GATT_DiscAllChars = "
                                  MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                                  bstatus);
            }
            // Target service not found
            else
            {
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "GATT service not found");
            }
        }
    }
    break;

    case ATT_READ_BY_TYPE_RSP:
    {
        attReadByTypeRsp_t att = (attReadByTypeRsp_t)gattMsg->msg.readByTypeRsp;

        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE + display_index, 0, "ATT_READ_BY_TYPE_RSP len: %d numPairs: %d", 
                          att.len, att.numPairs);
        display_index++;

        for (uint8_t i = 0; i < att.numPairs; i++)
        {
            /* This is the Attribute Handle */
            uint16_t attHandle = BUILD_UINT16(att.pDataList[i * att.len + 0], att.pDataList[i * att.len + 1]);
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE + display_index, 0,"ATT Handle: 0x%x", attHandle);
            display_index++;

            /* This is the index to the beginning of the value data AFTER the attribute handle */
            uint8_t index = i * att.len + 2;

            /* The characteristic value data */
            uint8_t data1 = att.pDataList[index];     // Bit field of characteristic properties

            uint8_t data2 = att.pDataList[index + 1]; // Second Byte of Characteristic Value Handle
            uint8_t data3 = att.pDataList[index + 2]; // First Byte of Characteristic Value Handle

            /* If we were using 128-bit Bluetooth UUIDs, then we would have 16 of these instead of 2. */
            uint8_t data4 = att.pDataList[index + 3]; // Second byte of UUID
            uint8_t data5 = att.pDataList[index + 4]; // First byte of UUID

            /* Since we are in little endian, we have to reverse the order to make the values make sense. */
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE + display_index, 0,"UUID: 0x%x%x Val Handle: 0x%x%x Properties: 0x%x", 
                              data5, data4, data3, data2, data1);
            display_index++;
        }
    }
    break;
#endif // #ifdef GATT_SERVICE_DISCOVERY

    default:
      break;
  }
}

/*********************************************************************
 * @fn      Data_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the data
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Data_start( void )
{
  bStatus_t status = SUCCESS;

  // Register the handlers
  status = BLEAppUtil_registerEventHandler( &dataGATTHandler );

  // Return status value
  return( status );
}
