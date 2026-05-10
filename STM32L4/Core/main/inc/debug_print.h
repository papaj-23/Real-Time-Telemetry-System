#ifndef DEBUG_PRITN_H
#define DEBUG_PRITN_H

#include <stdarg.h>

void debug_vprint(const char *const fmt, va_list args);
void debug_print(const char *const fmt, ...);

#endif
