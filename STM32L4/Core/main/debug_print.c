#include "debug_print.h"
#include "stm32l4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

extern UART_HandleTypeDef huart2;

void debug_vprint(const char *const fmt, va_list args)
{
    char buf[160];
    uint32_t now = HAL_GetTick();
    int prefix_len = snprintf(buf, sizeof(buf), "[%lu ms] ", (unsigned long)now);

    if ((prefix_len < 0) || (prefix_len >= (int)sizeof(buf)))
    {
        return;
    }

    int len = vsnprintf(&buf[prefix_len], sizeof(buf) - (size_t)prefix_len, fmt, args);

    if (len > 0)
    {
        len += prefix_len;
        if (len >= (int)sizeof(buf))
        {
            len = (int)sizeof(buf) - 1;
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    }
}

void debug_print(const char *const fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    debug_vprint(fmt, args);
    va_end(args);
}
