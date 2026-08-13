/******************************************************************************

@file  app_common.c

@brief This example file is for common app util

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
//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "app_common_api.h"
#include "ti_drivers_config.h"
#include <ti/drivers/UART2.h>

#include "stdio.h"
#include <stdarg.h>

//*****************************************************************************
//! Defines
//*****************************************************************************


//*****************************************************************************
//! Prototypes
//*****************************************************************************
static void Common_HciGapEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Local Variables
//*****************************************************************************

UART2_Handle uart;

//*****************************************************************************
//! Globals
//*****************************************************************************


BLEAppUtil_EventHandler_t commonHciGapHandler =
{
    .handlerType    = BLEAPPUTIL_HCI_GAP_TYPE,
    .pEventHandler  = Common_HciGapEventHandler,
    .eventMask      = BLEAPPUTIL_HCI_COMMAND_STATUS_EVENT_CODE |
                      BLEAPPUTIL_HCI_LE_EVENT_CODE
};

//*****************************************************************************
//! Functions
//*****************************************************************************

int uart_print(const char *fmt, ...)
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
    int_fast16_t ret = UART2_write(uart, buf, len, &written);

    if (ret != 0)
        return ret;

    return written;
}

int uart_printf(const char *fmt, ...)
{
    char buf[256];
    char timestamp[32];
    va_list args;

    uint32_t ticks = ClockP_getSystemTicks();
    snprintf(timestamp, sizeof(timestamp), "[%lu] ", (unsigned long)ticks);

    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0)
        return len;

    if (len > sizeof(buf))
        len = sizeof(buf);

    char outbuf[288]; // 32 + 256
    int outlen = snprintf(outbuf, sizeof(outbuf), "%s%s", timestamp, buf);
    if (outlen > sizeof(outbuf))
        outlen = sizeof(outbuf);

    size_t written = 0;
    int_fast16_t ret = UART2_write(uart, outbuf, outlen, &written);

    if (ret != 0)
        return ret;

    return written;
}


void PrintUartInit(void) {
  UART2_Params uartParams;

  UART2_Params_init(&uartParams);
  uartParams.baudRate = 921600;

  uart = UART2_open(CONFIG_UART2_0, &uartParams);

  if (uart == NULL) {
    while (1) {
    }
  }
  uart_print("\r\n"
             "=====================================\r\n"
             "====    Texas Instruments        ====\r\n"
             "====          CFAE               ====\r\n"
             "====  Car Node CS_Handover Demo  ====\r\n"
             "=====================================\r\n");
#ifdef PHONE
  uart_printf("Car Node Ready for Phone!\r\n");
#else
  uart_printf("Car Node Ready for Keyfob!\r\n");
#endif

  GAP_Addr_Modes_t devAddrMode;
  uint8_t idAddr[B_ADDR_LEN] = {0};
  uint8_t rpAddr[B_ADDR_LEN] = {0};

  Common_getDevAddress(&devAddrMode, idAddr, rpAddr);

  uart_printf("BLE ID Address: %s\r\n", BLEAppUtil_convertBdAddr2Str(idAddr));
  if (devAddrMode > ADDRMODE_RANDOM) {
    uart_printf("BLE RP Address: %s\r\n", BLEAppUtil_convertBdAddr2Str(rpAddr));
  }
}

/*********************************************************************
 * @fn      Common_getDevAddress
 *
 * @brief   Extract the device address.
 *
 * @param  pAddrMode - pointer to the address mode
 * @param  pIdAddr   - pointer to the IDA
 * @param  pRpAddr   - pointer to the RPA
 *
 * @return @ref SUCCESS
 */
bStatus_t Common_getDevAddress(GAP_Addr_Modes_t *pAddrMode, uint8_t *pIdAddr, uint8_t *pRpAddr)
{
  bStatus_t status = SUCCESS;

  uint8_t *pGapIdAddr =  GAP_GetDevAddress(TRUE);
  uint8_t *pGapRpAddr =  GAP_GetDevAddress(FALSE);

  *pAddrMode = BLEAppUtil_getDevAddrMode();

  memcpy(pIdAddr, pGapIdAddr, B_ADDR_LEN);
  if (*pAddrMode > ADDRMODE_RANDOM)
  {
     memcpy(pRpAddr, pGapRpAddr, B_ADDR_LEN);
  }

  return status;
}

/*********************************************************************
 * @fn      Common_resetDevice
 *
 * @brief   Resets device
 *
 * @param   none
 *
 * @return  none
 */

void Common_resetDevice(void)
{
  Power_reset();
}


/*********************************************************************
 * @fn      Common_HciGapEventHandler
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
void Common_HciGapEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  switch (event)
  {
  case BLEAPPUTIL_HCI_COMMAND_STATUS_EVENT_CODE:
    break;
  case BLEAPPUTIL_HCI_LE_EVENT_CODE:
  {
    hciEvt_BLEPhyUpdateComplete_t *pPUC = (hciEvt_BLEPhyUpdateComplete_t *)pMsgData;

    if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
    {
      if (pPUC->status != SUCCESS)
      {
        uart_printf("Conn status: Phy update failure - connHandle = %d\r\n",
                    pPUC->connHandle);
      }
      else
      {
        char *currPhy =
            (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1 Mbps" : (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M)  ? "2 Mbps"
                                                                   : (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "CODED"
                                                                                                                      : "Unexpected PHY Value";
        uart_printf("Conn status: Phy update - connHandle = %d PHY = %s phy: %d\r\n",
                    pPUC->connHandle, currPhy, pPUC->rxPhy);
      }
    }
    break;
  }
  default:
  {
    break;
  }
  }
}

/*********************************************************************
 * @fn      Common_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the connection
 *          application module and initiate the parser module.
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Common_start()
{
  bStatus_t status = SUCCESS;

  status = BLEAppUtil_registerEventHandler(&commonHciGapHandler);
  if (status != SUCCESS)
  {
    return(status);
  }
  
  PrintUartInit();

  // Return status value
  return(status);
}
