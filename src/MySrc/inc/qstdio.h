#ifndef __QSTDIO_H__
#define __QSTDIO_H__

#include <stdarg.h>

int qscanf(const char* str, const char* fmt, ...);
char* qprintf(char* str, const char* fmt, ...);
char* qprintfv(char* str, const char* fmt, va_list args);

#endif