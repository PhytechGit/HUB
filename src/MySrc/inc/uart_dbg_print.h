/*
 * uart_dbg_print.h
 *
 *  Created on: Nov 1, 2016
 *      Author: phytech
 */

#ifndef UART_DBG_PRINT_H_
#define UART_DBG_PRINT_H_
#include <stdint.h>
#define PRINT_DBG_CRLF // print crlf

//void uartDebugInit(void);
//void printDbg(const char* format, ...);
//void printDbgCRLF(void);
//void printDbgStr(unsigned char nUart, char * pStr);
void printMsg(const char* msg);
void printMsg1(uint8_t* msg, int len);
void PrintNum(uint32_t val);
void PrintIntNum(int32_t val);
void printParameterVal(const char* msg, uint8_t var);
#endif /* UART_DBG_PRINT_H_ */
