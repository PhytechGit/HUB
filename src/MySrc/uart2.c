/*
 * uart2.c
 *
 *  Created on: Nov 16, 2016
 *      Author: phytech
 */

#include <string.h>
#include "em_device.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_usart.h"
#include "em_leuart.h"

#include "buffer.h"
#include "uart2.h"
#include "uart2_config.h"


// automatically allocate space in ram for each buffer
static uint8_t uart0RxData[UART0_RX_BUFFER_SIZE];
static uint8_t uart0TxData[UART0_TX_BUFFER_SIZE];
static uint8_t uart1RxData[UART1_RX_BUFFER_SIZE];
static uint8_t uart1TxData[UART1_TX_BUFFER_SIZE];



// flag variables
volatile bool   uartReadyTx[UART_NUMB_MAX];
volatile bool   uartBufferedTx[UART_NUMB_MAX];
uint16_t uartRxOverflow[UART_NUMB_MAX];

// receive and transmit buffers
static cBuffer uartRxBuffer[UART_NUMB_MAX];
static cBuffer uartTxBuffer[UART_NUMB_MAX];


volatile bool uartInitialized[UART_NUMB_MAX] = {false,false};


typedef void (*voidFuncPtr)(uint8_t);
volatile static voidFuncPtr UartRxFunc[UART_NUMB_MAX];

// for EZR32 UART0 
void uart0Init(void)
{
	USART_InitAsync_TypeDef init = USART0_INITASYNC_DEFAULT;

	// initialize the LEUART0 buffers
	bufferInit(&uartRxBuffer[UART0], uart0RxData, UART0_RX_BUFFER_SIZE);
	bufferInit(&uartTxBuffer[UART0], uart0TxData, UART0_TX_BUFFER_SIZE);
	// initialize user receive handlers
	UartRxFunc[UART0] = 0;

	// initialize states
	uartReadyTx[UART0] = true;
	uartBufferedTx[UART0] = false;
	// clear overflow count
	uartRxOverflow[UART0] = 0;

	/* Enable peripheral clocks */
	CMU_ClockEnable(cmuClock_HFPER, true);
	/* Configure GPIO pins */
	CMU_ClockEnable(cmuClock_GPIO, true);
	/* To avoid false start, configure output as high */
	GPIO_PinModeSet(gpioPortE, 10, gpioModePushPull, 1);
	//unsigned int RETARGET_RXPIN;
	//GPIO_Port_TypeDef RETARGET_RXPORT;
	GPIO_PinModeSet(gpioPortE, 11, gpioModeInput, 0);

	CMU_ClockEnable(cmuClock_USART0, true);

	/* Configure USART for basic async operation */
	init.enable = usartDisable;
	USART_InitAsync(USART0, &init);

	/* Enable pins at correct UART/USART location. */
	USART0->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC0;

	/* Clear previous RX interrupts */
	USART_IntClear(USART0, USART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(USART0_RX_IRQn);

	/* Enable RX interrupts */
	USART_IntEnable(USART0, USART_IEN_RXDATAV);
	NVIC_EnableIRQ(USART0_RX_IRQn);

	/* Clear previous TX interrupts */
	USART_IntClear(USART0, USART_IF_TXC);
	NVIC_ClearPendingIRQ(USART0_TX_IRQn);

	/* Enable TX interrupts */
	USART_IntEnable(USART0, USART_IEN_TXC);
	NVIC_EnableIRQ(USART0_TX_IRQn);

	/* Finally enable it */
	USART_Enable(USART0, usartEnable);

	uartInitialized[UART0] = true;
}

// for EZR32 LEUART0

void uart1Init(void)
{
	// initialize the LEUART0 buffers
	bufferInit(&uartRxBuffer[UART1], uart1RxData, UART1_RX_BUFFER_SIZE);
	bufferInit(&uartTxBuffer[UART1], uart1TxData, UART1_TX_BUFFER_SIZE);
	// initialize user receive handlers
	UartRxFunc[UART1] = 0;

	// initialize states
	uartReadyTx[UART1] = true;
	uartBufferedTx[UART1] = false;
	// clear overflow count
	uartRxOverflow[UART1] = 0;

	/* Enable peripheral clocks */
	CMU_ClockEnable(cmuClock_HFPER, true);
	/* Configure GPIO pins */
	CMU_ClockEnable(cmuClock_GPIO, true);
	/* To avoid false start, configure output as high */
	GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 1);
	//unsigned int RETARGET_RXPIN;
	//GPIO_Port_TypeDef RETARGET_RXPORT;
	GPIO_PinModeSet(gpioPortD, 5, gpioModeInput, 0);

	LEUART_TypeDef      *leuart = LEUART0;
	LEUART_Init_TypeDef init    = LEUART_INIT_9600_DEFAULT;
	/* Enable CORE LE clock in order to access LE modules */
	CMU_ClockEnable(cmuClock_CORELE, true);

	/* Select LFXO for LEUARTs (and wait for it to stabilize) */
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

	CMU_ClockEnable(cmuClock_LEUART0, true);

	/* Do not prescale clock */
	CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1);

	/* Configure LEUART */
	init.enable = leuartDisable;

	LEUART_Init(leuart, &init);
	/* Enable pins at default location */
	leuart->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN | LEUART_ROUTE_LOCATION_LOC0;

	/* Clear previous RX interrupts */
	LEUART_IntClear(LEUART0, LEUART_IEN_RXDATAV);
	/* Clear previous TX interrupts */
	LEUART_IntClear(LEUART0, LEUART_IEN_TXC);
	NVIC_ClearPendingIRQ(LEUART0_IRQn);

	/* Enable RX interrupts */
	LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV);
	/* Enable TX interrupts */
	LEUART_IntEnable(LEUART0, LEUART_IEN_TXC);
	NVIC_EnableIRQ(LEUART0_IRQn);

	/* Finally enable it */
	LEUART_Enable(leuart, leuartEnable);

	uartInitialized[UART1] = true;

}


cBuffer* uartGetRxBuffer(uint8_t nUart)
{
	// return rx buffer pointer
	return &uartRxBuffer[nUart];
}


cBuffer* uartGetTxBuffer(uint8_t nUart)
{
	// return tx buffer pointer
	return &uartTxBuffer[nUart];
}


int uartGetByte(uint8_t nUart)
{
	// get single byte from receive buffer (if available)
	uint8_t c;

	if(uartReceiveByte(nUart, &c))
		return c;
	else
		return -1;
}


uint8_t uartReceiveByte(uint8_t nUart, uint8_t *rxData)
{
	// make sure we have a receive buffer
	if(uartRxBuffer[nUart].size)
	{
		// make sure we have data
		if(uartRxBuffer[nUart].datalength)
		{
			// get byte from beginning of buffer
			*rxData = bufferGetFromFront(&uartRxBuffer[nUart]);
			return true;
		}
		else // no data
			return false;
	}
	else // no buffer
		return false;
}


void uartFlushReceiveBuffer(uint8_t nUart)
{
	// flush all data from receive buffer
	bufferFlush(&uartRxBuffer[nUart]);
}

uint8_t uartReceiveBufferIsEmpty(uint8_t nUart)
{
	return (uartRxBuffer[nUart].datalength == 0);
}

void uartAddToTxBuffer(uint8_t nUart, uint8_t data)
{
	// add data byte to the end of the tx buffer
	bufferAddToEnd(&uartTxBuffer[nUart], data);
}

void uartSendByte(uint8_t nUart, uint8_t txData)
{
	// wait for the transmitter to be ready
	while(!uartReadyTx[nUart]);
	if(nUart){
		// send uart1 byte
		LEUART_Tx(LEUART0, txData);

	}else{
		// send uart0 byte
		USART_Tx(USART0, txData);

	}
	// set ready state to FALSE
	uartReadyTx[nUart] = false;
}


void uartSendTxBuffer(uint8_t nUart)
{
	// turn on buffered transmit
	uartBufferedTx[nUart] = true;
	// send the first byte to get things going by interrupts
	uartSendByte(nUart, bufferGetFromFront(&uartTxBuffer[nUart]));
}


uint8_t uartSendBuffer(uint8_t nUart, uint8_t *buffer, uint16_t nBytes)
{
	uint8_t first;
	uint16_t i;

	while(uartBufferedTx[nUart]); // add for me

	// check if there's space (and that we have any bytes to send at all)
	if((uartTxBuffer[nUart].datalength + nBytes < uartTxBuffer[nUart].size) && nBytes){

		// grab first character
		first = *buffer++;	// original

		// copy user buffer to uart transmit buffer
		for(i = 0; i < nBytes-1; i++)
		{
			// put data bytes at end of buffer
			bufferAddToEnd(&uartTxBuffer[nUart], *buffer++);
		}

		// send the first byte to get things going by interrupts
		uartBufferedTx[nUart] = true; 	// original
		uartSendByte(nUart, first);		// original

		// return success
		return true;
	}
	else
	{
		// return failure
		return false;
	}
}

void uartSetRxHandler(uint8_t nUart, void (*rx_func)(uint8_t c))
{
	// make sure the uart number is within bounds
	if(nUart < UART_NUMB_MAX)
	{
		// set the receive interrupt to run the supplied user function
		UartRxFunc[nUart] = rx_func;
	}
}


// UART Transmit Complete Interrupt Function
void uartTransmitService(uint8_t nUart)
{
	// check if buffered tx is enabled
	if(uartBufferedTx[nUart])
	{
		// check if there's data left in the buffer
		if(uartTxBuffer[nUart].datalength)
		{
			// send byte from top of buffer
			if(nUart)
			{
				LEUART_Tx(LEUART0, bufferGetFromFront(&uartTxBuffer[nUart]));
			}
			else
			{
				USART_Tx(USART0, bufferGetFromFront(&uartTxBuffer[nUart]));
			}
		}
		else
		{
			// no data left
			uartBufferedTx[nUart] = false;
			// return to ready state
			uartReadyTx[nUart] = true;

			if(nUart)
			{
				LEUART_IntClear(LEUART0, LEUART_IFC_TXC);
			}
			else
			{
				USART_IntClear(USART0, USART_IF_TXC);
			}
		}
	}
	else
	{
		// we're using single-byte tx mode
		// indicate transmit complete, back to ready
		uartReadyTx[nUart] = true;

		if(nUart){
			LEUART_IntClear(LEUART0, LEUART_IFC_TXC);
		}else{
			USART_IntClear(USART0, USART_IF_TXC);
		}
	}
}


// UART Receive Complete Interrupt Function
void uartReceiveService(uint8_t nUart)
{
	register uint8_t reg;

	// get received char
	if(nUart)
	{
		reg = LEUART_Rx(LEUART0);
	}
	else
	{
		reg = USART_Rx(USART0);
	}

	// if there's a user function to handle this receive event
	if(UartRxFunc[nUart])
	{// call it and pass the received data
		UartRxFunc[nUart](reg);
	}
	else
	{// otherwise do default processing
		// put received char in buffer
		// check if there's space
		if( !bufferAddToEnd(&uartRxBuffer[nUart], reg) )
		{// no space in buffer
			// count overflow
			uartRxOverflow[nUart]++;
		}
	}
}


void USART0_RX_IRQHandler(void)
{

	if (USART0->IF & USART_IF_RXDATAV)
	{
		uartReceiveService(UART0);
	}
}

void USART0_TX_IRQHandler(void)
{
	if (USART0->IF & USART_IF_TXC)
	{
		// service UART0 transmit interrupt
		uartTransmitService(UART0);
	}
}

void LEUART0_IRQHandler(void)
{
	uint32_t flag = LEUART0->IF;

	if (flag & LEUART_IF_RXDATAV){
		uartReceiveService(UART1);
	}
	else if(flag & LEUART_IF_TXC){
		uartTransmitService(UART1);

	}
}

