#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "app_handover.h"

#define UART_MAX_READ_SIZE 1300

static UART2_Handle handoverUart;

// UART read buffer
uint8_t uartReadBuffer[UART_MAX_READ_SIZE] = {0};
uint16_t numBytesRead = 0;


 /*  ======== callbackFxn ========
 */
void callbackFxn(UART2_Handle handle, void *buffer, size_t count, void *userArg, int_fast16_t status)
{
  // This allocation will be freed by bleapp_util
  uint8_t *pHandoverData = (uint8_t *)ICall_malloc(count);

  if (pHandoverData != NULL)
  {
    memcpy(pHandoverData, buffer, count);
    BLEAppUtil_invokeFunction(Handover_commandParser, (char *)pHandoverData);
  }

  UART2_read(handle, (uint8_t *)uartReadBuffer, UART_MAX_READ_SIZE, NULL);
  
}

void handoverUartInit(void)
{
    UART2_Params uartParams;

    /* Create a UART in CALLBACK read mode */
    UART2_Params_init(&uartParams);
    uartParams.readMode     = UART2_Mode_CALLBACK;
    uartParams.readCallback = callbackFxn;
    uartParams.baudRate     = 921600;

    handoverUart = UART2_open(CONFIG_UART2, &uartParams);

    if (handoverUart == NULL)
    {
        /* UART2_open() failed */
        while (1) {}
    }
    
    UART2_read(handoverUart, uartReadBuffer, UART_MAX_READ_SIZE, NULL);
}

void uartWriteData(uint8_t *data, uint16_t len)
{
    UART2_write(handoverUart, data, len, NULL);
}

void uartReadData(uint8_t *data, uint16_t len)
{
    UART2_read(handoverUart,data, len, NULL);
}

