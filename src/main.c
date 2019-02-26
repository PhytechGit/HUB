/***************************************************************************//**
 * @file main.c
 * @brief EZRadio simple trx example
 *
 * This example shows how to easily implement a simple trx code for your
 * controller using EZRadio or EZRadioPRO devices.
 *
 * @version 5.5.0
 *******************************************************************************
 * # License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "spidrv.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "rtcdriver.h"
#include "sleep.h"

#include "ezradio_cmd.h"
#include "ezradio_api_lib.h"
#include "ezradio_plugin_manager.h"
#include "define.h"

/* Push button callback functionns. */
static void GPIO_PB1_IRQHandler(uint8_t pin);
static void GPIO_PB0_IRQHandler(uint8_t pin);

#if (defined EZRADIO_VARIABLE_DATA_START)
#define APP_PKT_DATA_START EZRADIO_VARIABLE_DATA_START
#else
#define APP_PKT_DATA_START 1u
#endif

#if (defined EZRADIO_PLUGIN_TRANSMIT)
static void appPacketTransmittedCallback (EZRADIODRV_Handle_t handle, Ecode_t status);
#endif //#if ( defined EZRADIO_PLUGIN_TRANSMIT )

#if (defined EZRADIO_PLUGIN_RECEIVE)
static void appPacketReceivedCallback (EZRADIODRV_Handle_t handle, Ecode_t status);
#endif //#if ( defined EZRADIO_PLUGIN_RECEIVE )

#if (defined EZRADIO_PLUGIN_CRC_ERROR)
static void appPacketCrcErrorCallback (EZRADIODRV_Handle_t handle, Ecode_t status);
#endif //#if ( defined EZRADIO_PLUGIN_CRC_ERROR )

#if !defined(__CROSSWORKS_ARM) && defined(__GNUC__)
/* sniprintf does not process floats, but occupy less flash memory ! */
#define snprintf    sniprintf
#endif

#if (defined EZRADIO_PLUGIN_TRANSMIT)

/* Defines the number of packets to send for one press of PB1.
 * Sends infinite number of packets if defined to 0xFFFF. */
#define APP_TX_PKT_SEND_NUM   0xFFFF

#define MAX_DATA	24

sensor MySensorsArr[MAX_DATA];
uint32_t MySlots = 0;
uint32_t MyID;

/* Tx packet data array, initialized with the default payload in the generated header file */
static uint8_t radioTxPkt[EZRADIO_FIFO_SIZE] = RADIO_CONFIG_DATA_CUSTOM_PAYLOAD;

/* Packet counter */
static volatile uint16_t appTxPktCntr = 0;

/* Sign tx active state */
static volatile bool appTxActive = false;

/* Data counter in transmitted packet */
static volatile uint16_t appDataCntr = 0;

#endif //#if ( defined EZRADIO_PLUGIN_TRANSMIT )

#if (defined EZRADIO_PLUGIN_RECEIVE)

/* Rx packet data array */
static uint8_t radioRxPkt[EZRADIO_FIFO_SIZE];

#endif //#if ( defined EZRADIO_PLUGIN_RECEIVE )

/* RTC frequency */
#if defined(_EZR32_HAPPY_FAMILY)
#define APP_RTC_FREQ_HZ 10u
#else
#define APP_RTC_FREQ_HZ 9u
#endif

/* RTC timeout */
#define APP_RTC_TIMEOUT_MS (1000u / APP_RTC_FREQ_HZ)
#define APP_RTC_TIMEOUT_1S (1000u)

/* RTC set time is expired */
static volatile bool rtcTick = false;

/** Timer used to issue time elapsed interrupt. */
static RTCDRV_TimerID_t rtcTickTimer;
static RTCDRV_TimerID_t rtcRepeateTimer;

static RTCDRV_TimerID_t rtc20SecTimer;

uint8_t fDataIn;
static uint8_t NewMsgStack[MAX_MSG_IN_STACK][MAX_MSG_LEN+2];
static int8_t gReadStack = 0;
static int8_t gWriteStack = 0;

WorkingMode curMode;
uint16_t	g_iBtr;
uint32_t	g_LoggerID;
uint8_t		g_nTimeSlot;
uint8_t		g_nHubSlot;
bool 		g_bflagWakeup;



/***************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons.
 ******************************************************************************/
static void GpioSetup(void)
{
  /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Initialize GPIO interrupt */
  GPIOINT_Init();

//  /* Configure PB0 as input and enable interrupt */
//  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPull, 1);
//  GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, true, true);
//  GPIOINT_CallbackRegister(BSP_GPIO_PB0_PIN, GPIO_PB0_IRQHandler);
//
//  /* Configure PB1 as input and enable interrupt */
//  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPull, 1);
//  GPIO_IntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, false, true, true);
//  GPIOINT_CallbackRegister(BSP_GPIO_PB1_PIN, GPIO_PB1_IRQHandler);
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler (PB1)
 *        Switches between analog and digital clock modes.
 ******************************************************************************/
static void GPIO_PB0_IRQHandler(uint8_t pin)
{
  (void)pin;

#if (defined EZRADIO_PLUGIN_TRANSMIT)
  /* Check if already transmitting some packets,
   * send one otherwise. */
  if ( !appTxPktCntr ) {
    appTxPktCntr += 1;
  }
#endif //#if ( defined EZRADIO_PLUGIN_TRANSMIT )
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler (PB0)
 *        Increments the time by one minute.
 ******************************************************************************/
static void GPIO_PB1_IRQHandler(uint8_t pin)
{
  (void)pin;

#if (defined EZRADIO_PLUGIN_TRANSMIT)
  /* Check if already transmitting some packets, stop them if so,
   * otherwise, send the APP_TX_PKT_SEND_NUM number of packets
   * (infinite is defined to 0xFFFF). */
  if (appTxPktCntr) {
    appTxPktCntr = 0;
  } else {
    appTxPktCntr += APP_TX_PKT_SEND_NUM;
  }
#endif //#if ( defined EZRADIO_PLUGIN_TRANSMIT )
}


/***************************************************************************//**
 * @brief   Register a callback function to be called repeatedly at the
 *          specified frequency.
 *
 * @param[in] pFunction  Pointer to function that should be called at the
 *                       given frequency.
 * @param[in] pParameter Pointer argument to be passed to the callback function.
 * @param[in] frequency  Frequency at which to call function at.
 *
 * @return  0 for successful or
 *         -1 if the requested frequency is not supported.
 ******************************************************************************/
int RepeatCallbackRegister(void(*pFunction)(void*),
                           void* pParameter,
                           unsigned int frequency)
{
  if (ECODE_EMDRV_RTCDRV_OK
      == RTCDRV_AllocateTimer(&rtcRepeateTimer)) {
    if (ECODE_EMDRV_RTCDRV_OK
        == RTCDRV_StartTimer(rtcRepeateTimer, rtcdrvTimerTypePeriodic, frequency,
                             (RTCDRV_Callback_t)pFunction, pParameter)) {
      return 0;
    }
  }

  return -1;
}

void PutRadio2Sleep()
{
	ezradio_change_state(EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
	RTCDRV_Delay(100);
	GPIO_PinOutClear(GPIO_TCXO_PORT, GPIO_TCXO_PIN);
}

void WakeupRadio()
{
	GPIO_PinOutSet(GPIO_TCXO_PORT, GPIO_TCXO_PIN);
	RTCDRV_Delay(5);
}

void GoToSleep()
{
	g_bflagWakeup = false;
    PutRadio2Sleep();
	while (g_bflagWakeup == false)
		EMU_EnterEM2(true);
}

bool IsBusySlot(uint8_t slot)
{
	uint32_t temp = 2^slot;

	if ((MySlots & temp) == 1)
		return true;
	return false;
}
/***************************************************************************//**
 * @brief GPIO Interrupt handler (PB0)
 *        Increments the time by one minute.
 ******************************************************************************/
void RTC_App_IRQHandler()
{
  rtcTick = true;
}

void RTC_TimeSlot()
{
	g_bflagWakeup = true;
	curMode = MODE_SLEEPING;
	g_nTimeSlot++;
	if (g_nTimeSlot == 180)
	  g_nTimeSlot = 0;
	// if time to listen for irrigation sensor
	if (((g_nTimeSlot % 15) >= 0) && ((g_nTimeSlot % 15) <= 2))
		curMode = MODE_LISTENING;
	// if time to listen to other sensors
	if ((g_nTimeSlot < 30) && (IsBusySlot(g_nTimeSlot)))
	  curMode = MODE_LISTENING;
	// if time to send data to logger
	if (g_nTimeSlot == g_nHubSlot)
	  curMode = MODE_SENDING;
	// if nothing to do now - gotosleep for another 20 sec
//	if (curMode == MODE_SLEEPING)
//	  ;
}

uint8_t GetFirstBusyCell()
{
	  for (int8_t i = 0; i < MAX_MSG_IN_STACK; i++)
		  if (NewMsgStack[i][INDEX_STATUS] == CELL_BUSY)
			  return i;
	  return MAX_MSG_IN_STACK;
}

uint8_t GetFirstEmptyCell()
{
	  for (int8_t i = 0; i < MAX_MSG_IN_STACK; i++)
		  if (NewMsgStack[i][INDEX_STATUS] == CELL_EMPTY)
			  return i;
	  return MAX_MSG_IN_STACK;
}

uint8_t GetCheckSum(uint8_t* buf, uint8_t len)
{
	uint8_t i, res = 0;
	for (i = 0; i < len; i++)
		res+= buf[i];
	return res;
}

uint32_t Bytes2Long(uint8_t* buf)
{
	 Uint32toBytes temp32bituint;

	for (uint8_t i = 0; i < 4; i++)
		temp32bituint.bVal[i] =  buf[i];

	return temp32bituint.iVal;
}

uint16_t Bytes2Int(uint8_t* buf)
{
	Uint16toBytes tmp;

	for (uint8_t i = 0; i < 2; i++)
		tmp.bVal[i] =  buf[i];

	return tmp.iVal;
}

void Copy(uint8_t* destBuf, uint8_t* srcBuf, uint8_t len)
{
	uint8_t i;
	for (i = 0; i < len; i++)
		destBuf[i] = srcBuf[i];
}

uint8_t InsertNewSensor(uint32_t senID)
{
	uint8_t i;
	bool bFoundEmpty = false;

	do
	{
		if (MySensorsArr[i].ID != 0)
			i++;
		else
			bFoundEmpty = true;
	}
	while ((i < MAX_DATA) && (!bFoundEmpty));

	if (bFoundEmpty)
	{
		MySensorsArr[i].ID = senID;
		return i;
	}
	return MAX_DATA;
}

uint8_t GetNextSensor(uint32_t senIndex)
{
	uint8_t i = senIndex;

	do
	{
		if (MySensorsArr[i].ID != 0)
			return i;
		else
			i++;
	}
	while (i < MAX_DATA);

	return MAX_DATA;
}

uint8_t GetNextFreeSlot(uint32_t allowSlot)
{
	uint8_t index = 0;
	uint32_t temp = 1;
	do
	{
		if ((MySlots & temp == 0) && (temp & allowSlot == 1))
		{
			MySlots = MySlots & temp;
				return index;
		}
		index++;
		temp = temp << 1;
	}
	while  (index < 30);
	return 30;
}

uint8_t GetSensorIndex(uint32_t senID)
{
	uint8_t i;
	do
	{
		if (MySensorsArr[i].ID == senID)
			return i;
		i++;
	}
	while(i < MAX_DATA);
	return MAX_DATA;
}

bool ParseMsg()
{
	uint8_t senIndex, size = NewMsgStack[gReadStack][INDEX_SIZE];

	if (NewMsgStack[gReadStack][size-1] != GetCheckSum(&NewMsgStack[gReadStack][0],size))
		//wrong CS
		return false;
	if (curMode == MODE_CONFIGURE)
	{
		if (NewMsgStack[gReadStack][INDEX_HEADER] != HEADER_GETID_ACK)
			return false;
		if (Bytes2Int(&NewMsgStack[gReadStack][2]) != g_iBtr)
			return false;
		radioTxPkt[INDEX_HEADER] = HEADER_ID_OK;
		radioTxPkt[INDEX_SIZE] = 7;
		radioTxPkt[2] = MyID;
		radioTxPkt[6] = GetCheckSum(radioTxPkt, 6);
	}
	if (curMode == MODE_LISTENING)
	{
		// if its data or history msg from sensor
		if ((NewMsgStack[gReadStack][INDEX_HEADER] == HEADER_MSR) || (NewMsgStack[gReadStack][INDEX_HEADER] == HEADER_HST))
		{
			if (Bytes2Long(&NewMsgStack[gReadStack][INDEX_TO_ADDRESS]) != MyID)
				// msg not for me
				return false;

			senIndex = GetSensorIndex((uint32_t)(&NewMsgStack[gReadStack][INDEX_FROM_ADDRESS]));
			if (senIndex == MAX_DATA)
				return false;
			if (NewMsgStack[gReadStack][INDEX_HEADER] == HEADER_MSR)
			{
				Copy(&MySensorsArr[senIndex].data[0], &NewMsgStack[gReadStack][INDEX_DATA], 6);
				MySensorsArr[senIndex].IsData = true;
			}
			if (NewMsgStack[gReadStack][INDEX_HEADER] == HEADER_HST)
			{
				Copy(&MySensorsArr[senIndex].HstrData[0], &NewMsgStack[gReadStack][INDEX_DATA], 10);
				MySensorsArr[senIndex].IsHstr = true;
			}

			//Build response
			radioTxPkt[INDEX_HEADER] = NewMsgStack[gReadStack][INDEX_HEADER]+1;
			radioTxPkt[INDEX_TO_ADDRESS] = MySensorsArr[senIndex].ID;
			radioTxPkt[INDEX_FROM_ADDRESS] = MyID;
			radioTxPkt[INDEX_SLOT] = MySensorsArr[senIndex].slot.index;
			radioTxPkt[INDEX_TX_COUNTER] = NewMsgStack[gReadStack][INDEX_RX_COUNTER];
			radioTxPkt[INDEX_SIZE] = 13;
			radioTxPkt[INDEX_TX_CS] = GetCheckSum(radioTxPkt, 12);
		}
	}
	if  (curMode == MODE_INSTALLATION)
	{
		// if its data or history msg from sensor
		if ((NewMsgStack[gReadStack][INDEX_HEADER] == HEADER_MSR) || (NewMsgStack[gReadStack][INDEX_HEADER] == HEADER_HST))
		{
			if (Bytes2Long(&NewMsgStack[gReadStack][INDEX_TO_ADDRESS]) == DEFAULT_ID)
			{
				senIndex = InsertNewSensor((uint32_t)(&NewMsgStack[gReadStack][INDEX_FROM_ADDRESS]));
				if (senIndex == MAX_DATA)
				// cant insert this sensor
					return false;

				if (NewMsgStack[gReadStack][INDEX_RSSI] == TYPE_IRRIGATION)
					MySensorsArr[senIndex].slot.index = GetNextFreeSlot(IRG_SLOTS);
				else
					MySensorsArr[senIndex].slot.index = GetNextFreeSlot(NOT_IRG_SLOTS);

				MySensorsArr[senIndex].slot.status = STATUS_STANDBY;
				//Build response
				radioTxPkt[INDEX_HEADER] = NewMsgStack[gReadStack][INDEX_HEADER]+1;
				radioTxPkt[INDEX_TO_ADDRESS] = MySensorsArr[senIndex].ID;
				radioTxPkt[INDEX_FROM_ADDRESS] = MyID;
				radioTxPkt[INDEX_SLOT] = MySensorsArr[senIndex].slot.index;
				radioTxPkt[INDEX_TX_COUNTER] = NewMsgStack[gReadStack][INDEX_RX_COUNTER];
				radioTxPkt[INDEX_SIZE] = 13;
				radioTxPkt[INDEX_TX_CS] = GetCheckSum(radioTxPkt, 12);
			}
		}
	}
	if (curMode == MODE_SENDING)
	{
		if (NewMsgStack[gReadStack][INDEX_HEADER] != HEADER_SND_DATA_ACK)
			return false;
		if (Bytes2Long(&NewMsgStack[gReadStack][INDEX_TO_ADDRESS]) != MyID)
			// msg not for me
			return false;
	}
	return true;
}

uint8_t GetSensorData(uint8_t senIndex, uint8_t* tmp)
{
	uint8_t res = 0;

	if ((MySensorsArr[senIndex].ID == 0) || (!(MySensorsArr[senIndex].IsData) && (!MySensorsArr[senIndex].IsHstr)))
		return 0;
	if (MySensorsArr[senIndex].IsData)
	{
		tmp[0] = 12;
		tmp[1] = TYPE_DATA;
		Copy(&tmp[2], (uint8_t*)MySensorsArr[senIndex].ID, 4);
		Copy(&tmp[6], MySensorsArr[senIndex].data,6);
		res = 12;
	}
	if (MySensorsArr[senIndex].IsHstr)
	{
		tmp[res] = res;
		tmp[res+1] = TYPE_HSTR;
		Copy(&tmp[res+2], (uint8_t*)MySensorsArr[senIndex].ID, 4);
		Copy(&tmp[res+6], MySensorsArr[senIndex].HstrData,10);
		res += 16;
	}
	return res;
}

void BuildDataMsg()
{
	uint8_t bufIndex = 0, senIndex = 0, i;
	uint8_t tmp[30];

	radioTxPkt[INDEX_HEADER] = HEADER_SND_DATA;
	radioTxPkt[INDEX_SIZE] = 100;
	radioTxPkt[INDEX_TO_ADDRESS] = g_LoggerID;
	radioTxPkt[INDEX_FROM_ADDRESS] = MyID;

	senIndex = GetNextSensor(senIndex);
	i = GetSensorData(senIndex, &tmp[0]);
	while ((bufIndex + i < MAX_TX_LEN) && (senIndex != MAX_DATA))
	{
		Copy(&radioTxPkt[bufIndex], &tmp[0], i);
		MySensorsArr[senIndex].Status = STATUS_SEND_DATA;
		senIndex = GetNextSensor(senIndex);
		if (senIndex != MAX_DATA)
			i = GetSensorData(senIndex, &tmp[0]);
	}
	radioTxPkt[bufIndex] = GetCheckSum(radioTxPkt, bufIndex);
}
/***************************************************************************//**
 * @brief  Main function of the example.
 ******************************************************************************/
int main(void)
{
  /* EZRadio driver init data and handler */
  EZRADIODRV_HandleData_t appRadioInitData = EZRADIODRV_INIT_DEFAULT;
  EZRADIODRV_Handle_t appRadioHandle = &appRadioInitData;

  /* EZRadio response structure union */
  ezradio_cmd_reply_t ezradioReply;

  /* Chip errata */
  CHIP_Init();

  /* HFXO 48MHz, divided by 1 */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Setup GPIO for pushbuttons. */
  GpioSetup();

  SLEEP_Init(NULL,NULL);

  /* Set RTC to generate interrupt 250ms. */
  RTCDRV_Init();
  if (ECODE_EMDRV_RTCDRV_OK
      != RTCDRV_AllocateTimer(&rtcTickTimer) ) {
    while (1) ;
  }
  if (ECODE_EMDRV_RTCDRV_OK
      != RTCDRV_StartTimer(rtcTickTimer, rtcdrvTimerTypePeriodic, APP_RTC_TIMEOUT_MS,
                           (RTCDRV_Callback_t)RTC_App_IRQHandler, NULL) ) {
    while (1) ;
  }

  if (ECODE_EMDRV_RTCDRV_OK
      != RTCDRV_AllocateTimer(&rtc20SecTimer) ) {
    while (1) ;
  }
  if (ECODE_EMDRV_RTCDRV_OK
      != RTCDRV_StartTimer(rtc20SecTimer, rtcdrvTimerTypePeriodic, APP_RTC_TIMEOUT_1S * 20,
                           (RTCDRV_Callback_t)RTC_TimeSlot, NULL) ) {
    while (1) ;
  }


#if (defined EZRADIO_PLUGIN_TRANSMIT)
  /* Configure packet transmitted callback. */
  appRadioInitData.packetTx.userCallback = &appPacketTransmittedCallback;
#endif

#if (defined EZRADIO_PLUGIN_RECEIVE)
  /* Configure packet received buffer and callback. */
  appRadioInitData.packetRx.userCallback = &appPacketReceivedCallback;
  appRadioInitData.packetRx.pktBuf = radioRxPkt;
#endif

#if (defined EZRADIO_PLUGIN_CRC_ERROR)
  /* Configure packet received with CRC error callback. */
  appRadioInitData.packetCrcError.userCallback = &appPacketCrcErrorCallback;
#endif

  /* Initialize EZRadio device. */
  ezradioInit(appRadioHandle);

  /* Reset radio fifos and start reception. */
  ezradioResetTRxFifo();
#if (defined EZRADIO_PLUGIN_RECEIVE)
  ezradioStartRx(appRadioHandle);
#endif

  if (MyID == 0)
	  curMode = MODE_CONFIGURE;
  else
	  curMode = MODE_LISTENING;
  /* Enter infinite loop that will take care of ezradio plugin manager and packet transmission. */
  while (1)
  {
    /* Run radio plug-in manager */
    ezradioPluginManager(appRadioHandle);

    if (fDataIn)
    {
    	fDataIn = false;
    	gReadStack = GetFirstBusyCell();
		do
		{
			if (ParseMsg() == true)
			{
				ezradioStartTransmitConfigured( appRadioHandle, radioTxPkt );
			}
			NewMsgStack[gReadStack][INDEX_STATUS] = CELL_EMPTY;
			gReadStack = GetFirstBusyCell();
		}
		while (gReadStack != MAX_MSG_IN_STACK);
    }
    if (curMode == MODE_SENDING)
    	BuildDataMsg();

    if (rtcTick)
    {
      rtcTick = false;

#if (defined EZRADIO_PLUGIN_TRANSMIT)
      /* Send a packet if requested */
      if (appTxPktCntr) {
        /* Try to send the packet */
        if ( !appTxActive ) {
          /* Sing tx active state */
          appTxActive = true;

          /* Add data cntr as the data to be sent to the packet */
          radioTxPkt[APP_PKT_DATA_START]   = (uint8_t)( ((uint16_t)appDataCntr) >> 8);
          radioTxPkt[APP_PKT_DATA_START + 1] = (uint8_t)( ((uint16_t)appDataCntr) & 0x00FF);

          /* Transmit packet */
          ezradioStartTransmitDefault(appRadioHandle, radioTxPkt);

          /* Increase data counter */
          appDataCntr++;

          /* Decrease number of requested packets,
           * if not configured to infinite. */
          if (appTxPktCntr != 0xFFFF)
          {
            /* Decrease request counter */
            if (appTxPktCntr)
            {
              appTxPktCntr--;
            }
          }
        }
        else
        {
       //   printf("---Data TX:  need to wait\n");
        }
      }
#endif //#if ( defined EZRADIO_PLUGIN_TRANSMIT )
    }
  }
}

#if (defined EZRADIO_PLUGIN_TRANSMIT)
/***************************************************************************//**
 * @brief  Packet transmitted callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 ******************************************************************************/
static void appPacketTransmittedCallback(EZRADIODRV_Handle_t handle, Ecode_t status)
{
  if ( status == ECODE_EMDRV_EZRADIODRV_OK ) {
    /* Sign tx passive state */
    appTxActive = false;

#if (defined EZRADIO_PLUGIN_RECEIVE)
    /* Change to RX state */
    ezradioStartRx(handle);
#endif
  }
}
#endif //#if ( defined EZRADIO_PLUGIN_TRANSMIT )

#if (defined EZRADIO_PLUGIN_RECEIVE)
/***************************************************************************//**
 * @brief  Packet received callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 ******************************************************************************/
static void appPacketReceivedCallback(EZRADIODRV_Handle_t handle, Ecode_t status)
{
  //Silent warning.
  (void)handle;

  if ( status == ECODE_EMDRV_EZRADIODRV_OK )
  {
	  fDataIn = true;
	  ezradio_cmd_reply_t ezradioReply;
	  	  //ezradio_part_info(&ezradioReply);
	  	  ezradio_get_modem_status(0 ,&ezradioReply);
	  	  uint8_t g_nRssi = ezradioReply.GET_MODEM_STATUS.LATCH_RSSI;
	  	  gWriteStack = GetFirstEmptyCell();
	  	  if (gWriteStack == MAX_MSG_IN_STACK)
	  	  {
	  		  //printMsg("doesn't have empty cell. missed msg \r\n");
	  		  return;
	  	  }

	  	  for (int8_t i = 0; i < MAX_MSG_LEN; i++)
	  		  NewMsgStack[gWriteStack][i] = radioRxPkt[i];
	  	  NewMsgStack[gWriteStack][INDEX_RSSI] = g_nRssi;
	  	  NewMsgStack[gWriteStack][INDEX_STATUS] = CELL_BUSY;
    /* Read out and print received packet data:
     *  - print 'ACK' in case of ACK was received
     *  - print the data if some other data was received. */
//    if ( (radioRxPkt[APP_PKT_DATA_START] == 'A')
//         && (radioRxPkt[APP_PKT_DATA_START + 1] == 'C')
//         && (radioRxPkt[APP_PKT_DATA_START + 2] == 'K') ) {
//      //printf("-->Data RX: ACK\n");
//    } else
//    {
//      uint16_t rxData;
//
//      rxData =  (uint16_t)(radioRxPkt[APP_PKT_DATA_START]) << 8;
//      rxData += (uint16_t)(radioRxPkt[APP_PKT_DATA_START + 1]);
//
//      //printf("-->Data RX: %05d\n", rxData);
//    }
  }
}
#endif //#if ( defined EZRADIO_PLUGIN_RECEIVE )

#if (defined EZRADIO_PLUGIN_CRC_ERROR)
/***************************************************************************//**
 * @brief  Packet received with CRC error callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 ******************************************************************************/
static void appPacketCrcErrorCallback(EZRADIODRV_Handle_t handle, Ecode_t status)
{
  if ( status == ECODE_EMDRV_EZRADIODRV_OK ) {
    //printf("-->Pkt  RX: CRC Error\n");

#if (defined EZRADIO_PLUGIN_RECEIVE)
    /* Change to RX state */
    ezradioStartRx(handle);
#endif //#if ( defined EZRADIO_PLUGIN_RECEIVE )
  }
}
#endif //#if ( defined EZRADIO_PLUGIN_CRC_ERROR )
