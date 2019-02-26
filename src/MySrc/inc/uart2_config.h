/*
 * uart2_config.h
 *
 *  Created on: Nov 16, 2016
 *      Author: phytech
 */

#ifndef SRC_UART2_CONFIG_H_
#define SRC_UART2_CONFIG_H_


#define UART_NUMB_MAX		2


#define UART0_RX_BUFFER_SIZE	200//80
#define UART0_TX_BUFFER_SIZE	200//80
#define UART1_RX_BUFFER_SIZE	200//80
#define UART1_TX_BUFFER_SIZE	200//80

#define UART0		0
#define UART1		1


#define USART0_INITASYNC_DEFAULT                                                            \
{                                                                                          \
  usartEnable,      /* Enable RX/TX when init completed. */                                \
  0,                /* Use current configured reference clock for configuring baudrate. */ \
  115200,           /* 115200 bits/s. */                                                   \
  usartOVS16,       /* 16x oversampling. */                                                \
  usartDatabits8,   /* 8 databits. */                                                      \
  usartNoParity,    /* No parity. */                                                       \
  usartStopbits1,   /* 1 stopbit. */                                                       \
  false,            /* Do not disable majority vote. */                                    \
  false,            /* Not USART PRS input mode. */                                        \
  usartPrsRxCh0     /* PRS channel 0. */                                                   \
}

/** Default config for LEUART init structure. */
#define LEUART_INIT_38400_DEFAULT                                                                 \
{                                                                                           \
  leuartEnable,      /* Enable RX/TX when init completed. */                                \
  0,                 /* Use current configured reference clock for configuring baudrate. */ \
  38400,               /* 38400 bits/s. */                                                     \
  leuartDatabits8,   /* 8 databits. */                                                      \
  leuartNoParity,    /* No parity. */                                                       \
  leuartStopbits1    /* 1 stopbit. */                                                       \
}
/** Default config for LEUART init structure. */
#define LEUART_INIT_9600_DEFAULT                                                                 \
{                                                                                           \
  leuartEnable,      /* Enable RX/TX when init completed. */                                \
  0,                 /* Use current configured reference clock for configuring baudrate. */ \
  9600,               /* 9600 bits/s. */                                                     \
  leuartDatabits8,   /* 8 databits. */                                                      \
  leuartNoParity,    /* No parity. */                                                       \
  leuartStopbits1    /* 1 stopbit. */                                                       \
}


#endif /* SRC_UART2_CONFIG_H_ */
