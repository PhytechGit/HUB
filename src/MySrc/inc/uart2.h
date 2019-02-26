/*
 * uart2.h
 *
 *  Created on: Nov 16, 2016
 *      Author: phytech
 */

#ifndef MY_SRC_INC_UART2_H_
#define MY_SRC_INC_UART2_H_

#include "stdint.h"
#include "stdbool.h"

#include "uart2_config.h"
#include "buffer.h"

#define DBG_VCOM		UART0
#define UART_GPS		UART1
#define UART_LOGGER		UART1

extern volatile bool uartInitialized[];

void uart0Init(void);
void uart1Init(void);

cBuffer* uartGetRxBuffer(uint8_t nUart);
cBuffer* uartGetTxBuffer(uint8_t nUart);

int uartGetByte(uint8_t nUart);
uint8_t uartReceiveByte(uint8_t nUart, uint8_t *rxData);
void uartFlushReceiveBuffer(uint8_t nUart);
uint8_t uartReceiveBufferIsEmpty(uint8_t nUart);
void uartAddToTxBuffer(uint8_t nUart, uint8_t data);

void uartSendByte(uint8_t nUart, uint8_t txData);
void uartSendTxBuffer(uint8_t nUart);
uint8_t uartSendBuffer(uint8_t nUart, uint8_t *buffer, uint16_t nBytes);

void uartSetRxHandler(uint8_t nUart, void (*rx_func)(uint8_t c));

#endif /* MY_SRC_INC_UART2_H_ */
