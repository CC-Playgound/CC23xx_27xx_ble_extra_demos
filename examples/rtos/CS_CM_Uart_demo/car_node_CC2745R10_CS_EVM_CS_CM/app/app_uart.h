#ifndef APP_UART_H
#define APP_UART_H

#include <stdint.h>
#include <stddef.h>

void uartInit(void);
void uartWriteData(uint8_t *data, uint16_t len);
void uartReadData(uint8_t *data, uint16_t len);

#endif  // APP_UART_H