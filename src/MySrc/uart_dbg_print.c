/*
 * uart_dbg_print.c
 *
 *  Created on: Nov 1, 2016
 *      Author: phytech
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "uart_dbg_print.h"
#include "uart2.h"
#include "uart2_config.h"
#include "define.h"

//#include "MyGlobalVars.h"

/*void printDbg(const char* format, ...)
{
    char       msg[100];

    if(!uartInitialized[DBG_VCOM]){// initialized uart0 (work with VCOM) for print debug
    	uart0Init();
    }

    va_list    args;
    va_start(args, format);
    vsnprintf(msg, sizeof(msg), format, args); // do check return value
    va_end(args);
#ifdef PRINT_DBG_CRLF
    if(strcat(msg, "\r\n") != NULL){
#endif
    if(!uartSendBuffer(DBG_VCOM,(uint8_t *)msg, strlen(msg))){
    	while(1);
    }
#ifdef PRINT_DBG_CRLF
    }
#endif
}
*/
void printMsg(const char* msg)
{
//    char       msg[100];
#ifdef DEBUG_MODE

    if(!uartInitialized[DBG_VCOM]){// initialized uart0 (work with VCOM) for print debug
    	uart0Init();
    }
    uartSendBuffer(DBG_VCOM,(uint8_t *)msg, strlen(msg));
    uartSendBuffer(DBG_VCOM,"\r\n", 2);
#endif
}

void printParameterVal(const char* msg, uint8_t var)
{
//    char       msg[100];
#ifdef DEBUG_MODE

    if(!uartInitialized[DBG_VCOM]){// initialized uart0 (work with VCOM) for print debug
    	uart0Init();
    }
    uartSendBuffer(DBG_VCOM,(uint8_t *)msg, strlen(msg));
    PrintNum(var);
    //uartSendBuffer(DBG_VCOM,"\r\n", 2);
#endif
}

void PrintChar(const char* msg)
{
//    char       msg[100];
#ifdef DEBUG_MODE

    if(!uartInitialized[DBG_VCOM]){// initialized uart0 (work with VCOM) for print debug
    	uart0Init();
    }
    uartSendBuffer(DBG_VCOM,(uint8_t *)msg, 1);

#endif
}

void printMsg1(uint8_t* msg, int len)
{
//    char       msg[100];
#ifdef DEBUG_MODE

    if(!uartInitialized[DBG_VCOM]){// initialized uart0 (work with VCOM) for print debug
    	uart0Init();
    }
    uartSendBuffer(DBG_VCOM, msg, len);
    uartSendBuffer(DBG_VCOM,"\r\n", 2);
#endif
}

void printMsg2(uint8_t* msg, uint8_t len)
{
//    char       msg[100];
#ifdef DEBUG_MODE

    if(!uartInitialized[DBG_VCOM])
    {// initialized uart0 (work with VCOM) for print debug
    	uart0Init();
    }
    uartSendBuffer(DBG_VCOM, msg, len);
#endif
}
void PrintNum(uint32_t val)
{
#ifdef DEBUG_MODE
    char s[10];
    uint8_t i = 0;
    if (val > 999999 )
    {
    	//printMsg("too big number");
    	return;
    }

    do
    {
        s[i++] = (char)(val % 10);
        val = val / 10;
    }
    while (val > 0);
    for (; i > 0; i--)
    {
    	switch (s[i-1])
    	{
    	case 0:
    		PrintChar("0");
    		break;
    	case 1:
    		PrintChar("1");
    		break;
    	case 2:
    		PrintChar("2");
    		break;
    	case 3:
    		PrintChar("3");
    		break;
    	case 4:
    		PrintChar("4");
    		break;
    	case 5:
    		PrintChar("5");
    		break;
    	case 6:
    		PrintChar("6");
    		break;
    	case 7:
    		PrintChar("7");
    		break;
    	case 8:
    		PrintChar("8");
    		break;
    	case 9:
    		PrintChar("9");
    		break;
    	}
    }
    printMsg("\r\n");
//        putchar1(s[i-1] + 48);
//    putchar1('#');
//    putchar1('\n');
#endif
}

void PrintIntNum(int32_t val)
{
#ifdef DEBUG_MODE
    char s[10];
    uint8_t i = 0;
    if (val > 999999 )
    {
    	printMsg("too big number");
    	return;
    }
    if (val < 0)
    {
    	printMsg("-");
        val *= -1;
    }

    do
    {
        s[i++] = (char)(val % 10);
        val = val / 10;
    }
    while (val > 0);
    for (; i > 0; i--)
    {
    	switch (s[i-1])
    	{
    	case 0:
    		printMsg("0");
    		break;
    	case 1:
    		printMsg("1");
    		break;
    	case 2:
    		printMsg("2");
    		break;
    	case 3:
    		printMsg("3");
    		break;
    	case 4:
    		printMsg("4");
    		break;
    	case 5:
    		printMsg("5");
    		break;
    	case 6:
    		printMsg("6");
    		break;
    	case 7:
    		printMsg("7");
    		break;
    	case 8:
    		printMsg("8");
    		break;
    	case 9:
    		printMsg("9");
    		break;
    	}
    }
    printMsg("\r\n");
//        putchar1(s[i-1] + 48);
//    putchar1('#');
//    putchar1('\n');
#endif
}

/*
void printDbgCRLF(void)
{
	//printDbg("\r\n");
}
*/

/*
 * void printDbgStr(unsigned char nUart, char * pStr)
{
	while(*pStr)
	{
		uartSendByte(nUart, *pStr);
		pStr++;
	}
}
*/
