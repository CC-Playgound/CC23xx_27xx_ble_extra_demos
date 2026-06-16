#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include "ti/ble/app_util/framework/bleapputil_api.h"

#define UART_MAX_READ_SIZE 300
#define CM_DATA_SIZE 56

UART2_Handle uart;

// UART read buffer
uint8_t uartReadBuffer[UART_MAX_READ_SIZE] = {0};
uint16_t numBytesRead = 0;

void cmDataReceive(char *param);
void cmUpdateDataReceive(char *param);
/*
 *  ======== callbackFxn ========
 */
void callbackFxn(UART2_Handle handle, void *buffer, size_t count, void *userArg, int_fast16_t status)
{
    uint8_t *receiveData = (uint8_t *)buffer;
    numBytesRead = count;

    if (count == CM_DATA_SIZE) // TODO: read according count directly
    {
        GPIO_write(CONFIG_GPIO_LED_RED, CONFIG_GPIO_LED_ON);

        // This allocation will be free by bleapp_util
        uint8_t *cmData = (uint8_t *)ICall_malloc(CM_DATA_SIZE);
        memcpy(cmData, buffer, CM_DATA_SIZE);
        BLEAppUtil_invokeFunction(cmDataReceive, (char *)cmData); 
    }
    else if (receiveData[0] == 0xA0) // CM update
    {
        // This allocation will be free by bleapp_util
        uint8_t *connUpdateData = (uint8_t *)ICall_malloc(count);
        memcpy(connUpdateData, buffer, count);
        BLEAppUtil_invokeFunction(cmUpdateDataReceive, (char *)connUpdateData); // TODO: use only one BLEAppUtil_invokeFunction
    }
    else 
    {
        // TODO:
    }
    UART2_read(uart, uartReadBuffer, UART_MAX_READ_SIZE, NULL);
}

void uartInit(void)
{
    UART2_Params uartParams;

    /* Create a UART in CALLBACK read mode */
    UART2_Params_init(&uartParams);
    uartParams.readMode     = UART2_Mode_CALLBACK;
    uartParams.readCallback = callbackFxn;
    uartParams.baudRate     = 115200;

    uart = UART2_open(CONFIG_UART2_1, &uartParams);

    if (uart == NULL)
    {
        /* UART2_open() failed */
        while (1) {}
    }
    
    UART2_read(uart, uartReadBuffer, UART_MAX_READ_SIZE, NULL);
}

void uartWriteData(uint8_t *data, uint16_t len)
{
    UART2_write(uart, data, len, NULL);
}

void uartReadData(uint8_t *data, uint16_t len)
{
    UART2_read(uart,data, len, NULL);
}

