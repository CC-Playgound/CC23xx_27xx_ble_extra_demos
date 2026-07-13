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
#include <ti/drivers/dpl/ClockP.h>


//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************
extern uint8_t charVal;
static void GATT_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

// Events handlers struct, contains the handlers and event masks
// of the application data module
BLEAppUtil_EventHandler_t dataGATTHandler =
{
    .handlerType    = BLEAPPUTIL_GATT_TYPE,
    .pEventHandler  = GATT_EventHandler,
    .eventMask      = BLEAPPUTIL_ATT_FLOW_CTRL_VIOLATED_EVENT |
                      BLEAPPUTIL_ATT_MTU_UPDATED_EVENT |
                      BLEAPPUTIL_ATT_READ_RSP |
                      BLEAPPUTIL_ATT_WRITE_RSP |
                      BLEAPPUTIL_ATT_WRITE_REQ |
                      BLEAPPUTIL_ATT_EXCHANGE_MTU_RSP|
                      BLEAPPUTIL_ATT_EXCHANGE_MTU_REQ|
                      BLEAPPUTIL_ATT_ERROR_RSP |
                      BLEAPPUTIL_ATT_HANDLE_VALUE_NOTI
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

uint32_t gNotiCounter = 0;
uint32_t gInstantBits = 0;
float gInstantSpeed = 0;
uint32_t gTempTimeTicks = 0;

#ifdef Display_DISABLE_ALL
#include "ti/drivers/UART2.h"
#include "stdio.h"

UART2_Handle uartHandle;

int uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0)
        return len;

    if (len > sizeof(buf))
        len = sizeof(buf);

    size_t written = 0;
    int_fast16_t ret = UART2_write(uartHandle, buf, len, &written);

    if (ret != 0)
        return ret;

    return written;
}
#endif

#include <FreeRTOS.h>
#include <task.h>
#include <stdarg.h>
#include <queue.h>


extern QueueHandle_t myQueue;


static uint8_t _tmp[244 * 3];
static uint8_t count = 0;
static uint16_t offset = 0;
size_t writeBytes;

#ifdef Display_DISABLE_ALL
/* Task function */
void taskFunction(void* a0)
{
  uint8_t frame_order = 0;
  uint8_t temp[244];
  
  while (1)
  {
    
    
    if(xQueueReceive(myQueue, (void*) temp, portMAX_DELAY) == pdPASS)
    {
      
      uint16_t len = 244;
      uint8_t *pData = temp;

      // 拷贝到缓存
      memcpy(_tmp + offset, pData, len);
      offset += len;
      count++;

      // 满 3 包才发
      if (count == 4)
      {
          UART2_write(uartHandle, _tmp, offset, &writeBytes);
 
          // 清状态
          count = 0;
          offset = 0;
      }
      
    }
  }
  vTaskDelete( NULL );
}
#endif




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
      }
      break;
    case ATT_READ_RSP:
        {

            MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Read rsp = 0x%02x",
                              gattMsg->msg.readRsp.pValue[0]);

             break;
        }
    case ATT_WRITE_REQ:
        {
          MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Write req sent = %d",
                              gattMsg->msg.writeReq.len);
          break;
        }
    case ATT_WRITE_RSP:
        {
            MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Write sent = 0x%02x",
                              charVal);
                              //gattMsg->msg.writeReq.pValue[0]);
             break;
        }
    case ATT_EXCHANGE_MTU_REQ:
    {
      MenuModule_printf(0, 0, "-- ATT_EXCHANGE_MTU_REQ");
    }
    break;

    case ATT_EXCHANGE_MTU_RSP:
        {
            MenuModule_printf(APP_MENU_CONN_EVENT, 0, "ATT_EXCHANGE_MTU_RSP: MTU max size client = %d MTU max size server = %d; status: %d",
                              gattMsg->msg.exchangeMTUReq.clientRxMTU, gattMsg->msg.exchangeMTURsp.serverRxMTU, gattMsg->hdr.status);
            break;
        }

    case   ATT_HANDLE_VALUE_NOTI:
        {
            // MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "Notification received = 0x%02x",
            //                               gattMsg->msg.handleValueNoti.pValue[0]);
            
            GPIO_toggle(15);
            #ifdef Display_DISABLE_ALL
            
            GPIO_write(14,1);
            
            uint16_t len = gattMsg->msg.handleValueNoti.len;
            uint8_t *pData = gattMsg->msg.handleValueNoti.pValue;

            // 拷贝到缓存
            memcpy(_tmp + offset, pData, len);
            offset += len;
            count++;
            
            if (count == 3)
            {
                UART2_write(uartHandle, _tmp, offset, &writeBytes);
      
                // 清状态
                count = 0;
                offset = 0;
            }
            
          
            // if(xQueueSend(myQueue, ( void * )gattMsg->msg.handleValueNoti.pValue, 0) != pdPASS)
            // {
            //   while(1) {}
            // }
			      GPIO_write(14,0);
            #endif
            gNotiCounter++;

            gInstantBits += gattMsg->msg.handleValueNoti.len * 8;

            if(! (gNotiCounter % 1000))
            {
              extern uint32_t _test;
              uint32_t _nowTicks = ClockP_getSystemTicks();
              uint32_t _time = _nowTicks - gTempTimeTicks;
              float kbps = ((float)gInstantBits / (float)_time) * 1000.0f;
              // MenuModule_printf(0, 0, "_nowTicks: %d", _time);
              #ifndef Display_DISABLE_ALL
              MenuModule_printf(0, 0, "P->C: Instant speed: %d bits %f kbps; Data Len:%d; time(us): %d.",  gInstantBits,  kbps, gattMsg->msg.handleValueNoti.len,_time);
              #else
              // uart_printf( "P->C: Instant speed: %d bits %f kbps; Data Len:%d.\r\n",  gInstantBits,  kbps, gattMsg->msg.handleValueNoti.len);
              
              #endif
              gInstantBits = 0;
              gTempTimeTicks = _nowTicks;
            }
            

            // MenuModule_printf(0, 0, "Noti: %d Len: %d", gattMsg->msg.handleValueNoti.pValue[0], gattMsg->msg.handleValueNoti.len);
            break;
        }
    case ATT_ERROR_RSP:
        {
            attErrorRsp_t  *pReq = (attErrorRsp_t  *)pMsgData;

            MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Error %d",
                              pReq->errCode);
           break;

        }

    default:
      break;
  }
}


#ifdef Display_DISABLE_ALL
/*
 *  ======== callbackFxn ========
 */
void callbackFxn(UART2_Handle handle, void *buffer, size_t count, void *userArg, int_fast16_t status)
{
    if (status != UART2_STATUS_SUCCESS)
    {
        /* TX error occured in UART2_write() */
        while (1) {}
    }

    // numBytesRead = count;
    // sem_post(&sem);
    // GPIO_toggle(24);
}
#endif


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

  #ifdef Display_DISABLE_ALL
  UART2_Params uartParams;
  /* Create a UART in CALLBACK read mode */
  UART2_Params_init(&uartParams);
  uartParams.writeMode     = UART2_Mode_NONBLOCKING;
  // uartParams.writeCallback = callbackFxn;
  uartParams.baudRate     = 3000000;
  
  uartHandle = UART2_open(0, &uartParams);

  #endif
  // Return status value
  return( status );
}
