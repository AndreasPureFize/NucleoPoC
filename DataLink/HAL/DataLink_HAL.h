#ifndef DATALINK_HAL_H
#define DATALINK_HAL_H

#include <stdint.h>
#include "usart.h"   /* UART_HandleTypeDef */

#ifdef __cplusplus
extern "C" {
#endif

/* Console helpers (UART2) */
void print_line(const char* s);
void print_error_hex(uint32_t err);

#ifdef __cplusplus
}
#endif

#endif /* DATALINK_HAL_H */
