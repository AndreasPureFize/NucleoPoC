

#include "DataLink_HAL.h"
#include "main.h"      /* HAL_GetTick/HAL_Delay if needed later */
#include <string.h>
#include <stdio.h>

/* CubeMX provides these in usart.c */
extern UART_HandleTypeDef huart2;

/* Prints a raw line/string over the VCOM console (USART2) */
void print_line(const char* s)
{
    if (!s) return;
    HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

/* Formats and prints "Error 0xXXXXXXXX\r\n" over the VCOM console */
void print_error_hex(uint32_t err)
{
    char line[32];
    int n = snprintf(line, sizeof line, "Error 0x%08lX\r\n", (unsigned long)err);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);
}
