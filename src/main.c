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
#include "nvm.h"
#include "define.h"
#include "ProtocolDef.h"

/* Push button callback functionns. */
//static void GPIO_PB1_IRQHandler(uint8_t pin);
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

#define MAX_DATA	30//EZRADIO_FIFO_SIZE//24


/* Tx packet data array, initialized with the default payload in the generated header file */
static uint8_t radioTxPkt[EZRADIO_FIFO_SIZE] = RADIO_CONFIG_DATA_CUSTOM_PAYLOAD;

/* Packet counter */
//static volatile uint16_t appTxPktCntr = 0;

/* Sign tx active state */
static volatile bool appTxActive = false;

/* Data counter in transmitted packet */
//static volatile uint16_t appDataCntr = 0;

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

typedef enum
{
//  OBJ_GPS_ID,
//  OBJ_MSR_ID,
  OBJ_ID_TYPE
} NVM_Object_Ids;

Sen_Hub_Rec_Msg msg;
//Hub_Rec_Msg recMsg;

/* RTC set time is expired */
//static volatile bool rtcTick = false;

/** Timer used to issue time elapsed interrupt. */
static RTCDRV_TimerID_t rtcTickTimer;
//static RTCDRV_TimerID_t rtcRepeateTimer;

static RTCDRV_TimerID_t rtc20SecTimer;
static volatile uint16_t rtcTickCnt;
//static volatile uint16_t rtcTickCntEcho;

const uint8_t Version[] = {'H',5,19,1};

sensor MySensorsArr[MAX_DATA];
//uint32_t MyID;

uint8_t fDataIn;
static uint8_t NewMsgStack[MAX_MSG_IN_STACK][MAX_MSG_LEN+2];
static int8_t gReadStack = 0;
static int8_t gWriteStack = 0;

uint32_t 	g_lMySlots = 0;
WorkingMode g_wCurMode;
uint16_t	g_iBtr;
uint32_t	g_LoggerID = 0xFFFFFFFF;
uint8_t		g_nCurTimeSlot;
uint8_t		g_nHubSlot;
bool 		g_bflagWakeup;
bool 		g_bOnReset;
Task 		g_nCurTask = TASK_WAIT;
bool 		g_bIsMoreData;
bool 		g_bStartInstal;
uint8_t		g_nHour;
uint8_t		g_nMin;
uint8_t		g_nSec;
uint8_t		g_nRetryCnt;
/////////////for card 102
static uint16_t g_iBtnPressed = 0;

/////////////for card 102
static void GPIO_BTN_IRQHandler( uint8_t pin )
{
	if (g_iBtnPressed == 0)
		g_iBtnPressed = 1;
//	GPIO_PinOutSet(GPIO_LED1_PORT,GPIO_LED1_PIN);
	g_bflagWakeup = true;
}

void EnableBtnInt()
{
	g_iBtnPressed = 0;
	printMsg("EnableBtnInt");
   /* Configure PA1 as input and enable interrupt */
   GPIO_PinModeSet(GPIO_BTN_PORT, GPIO_BTN_PIN, gpioModeInput, 0);
   GPIO_IntConfig(GPIO_BTN_PORT, GPIO_BTN_PIN, false, true, true);
   GPIOINT_CallbackRegister( GPIO_BTN_PIN, GPIO_BTN_IRQHandler );
//   printMsg("enable btn intrpt\r\n");
}

void TurnOff()
{
	GPIO_PinOutClear(GPIO_LED2_PORT, GPIO_LED2_PIN);
#ifdef DEBUG_MODE
	printMsg("TurnOff");
#endif
	// TCXO Power Down
//	uartDebugDeInit();
	GPIO_PinOutClear(GPIO_TCXO_PORT, GPIO_TCXO_PIN);
//	SLEEP_ForceSleepInEM4();
	RTCDRV_DeInit();
	GPIO_PinOutClear(GPIO_LED1_PORT, GPIO_LED1_PIN);
	GPIO_PinOutClear(GPIO_LED2_PORT, GPIO_LED2_PIN);
	GPIO_PinOutClear(GPIO_MAIN_POWER_PORT, GPIO_MAIN_POWER_PIN);

}
/////////////end for card 102
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

  GPIO_PinModeSet(GPIO_MAIN_POWER_PORT, GPIO_MAIN_POWER_PIN, gpioModePushPull, 1);
   GPIO_DriveModeSet(GPIO_LED1_PORT,gpioDriveModeHigh);
   GPIO_PinModeSet(GPIO_LED1_PORT, GPIO_LED1_PIN, gpioModePushPullDrive, 0);
   GPIO_DriveModeSet(GPIO_LED2_PORT,gpioDriveModeHigh);
   GPIO_PinModeSet(GPIO_LED2_PORT, GPIO_LED2_PIN, gpioModePushPullDrive, 0);
  // power TCXO
  GPIO_PinModeSet(GPIO_TCXO_PORT, GPIO_TCXO_PIN, gpioModePushPull, 0);
   /* Configure GPS system on i/o */
   GPIO_PinModeSet(GPIO_BTN_PORT, GPIO_BTN_PIN, gpioModeInput, 0);
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler (PB1)
 *        Switches between analog and digital clock modes.
 ******************************************************************************/
static void GPIO_PB0_IRQHandler(uint8_t pin)
{
	g_bStartInstal = true;
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler (PB0)
 *        Increments the time by one minute.
 ******************************************************************************/
/*static void GPIO_PB1_IRQHandler(uint8_t pin)
{
  (void)pin;

#if (defined EZRADIO_PLUGIN_TRANSMIT)
//   Check if already transmitting some packets, stop them if so,
//   otherwise, send the APP_TX_PKT_SEND_NUM number of packets
//   (infinite is defined to 0xFFFF).
  if (appTxPktCntr) {
    appTxPktCntr = 0;
  } else {
    appTxPktCntr += APP_TX_PKT_SEND_NUM;
  }
#endif //#if ( defined EZRADIO_PLUGIN_TRANSMIT )
}
*/

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
******************************************************************************/

Ecode_t APP_StoreData(uint16_t page)
{
  Ecode_t result;

//  BSP_LedsSet(LED_STORE_ON);
  result = NVM_Write(page, NVM_WRITE_ALL_CMD);
//  BSP_LedsSet(LED_ALL_OFF);

//  if (result != ECODE_EMDRV_NVM_OK)
//    APP_Fatal(FATAL_AT_STORE);

  return result;
}

/**************************************************************************//**
 * @brief Read objects from page (volume)
 *****************************************************************************/
Ecode_t APP_ReadData(uint16_t page, uint16_t obj)
{
  Ecode_t result;

  result = NVM_Read(page, obj);

//  if (result != ECODE_EMDRV_NVM_OK)
//    APP_Fatal(FATAL_AT_READ);
  return result;
}
/**************************************************************************//**
 * @brief Initialize NVM and restore objects
 *****************************************************************************/
Ecode_t APP_RestoreData(void)
{
  Ecode_t result;

//   initialize NVM module
  result = NVM_Init(NVM_ConfigGet());

  if (result == ECODE_EMDRV_NVM_NO_PAGES_AVAILABLE)
  { // Ups, looks like no valid data in flash!
     //This could happen on first run after flashing.
     //So, we have to erase NVM
    result = NVM_Erase(0);
    // Store initial data/configuration
    if (result == ECODE_EMDRV_NVM_OK)
    {
//      result = APP_StoreData(PAGE_WEAR_ID);
    	printMsg("NVM_Erase OK");
    }
//     if wear page contains different data/object than normal page
//     it could be resonable to write wear page here too.
  }

//   if init phase went correctly, try to restore data.
  if (result == ECODE_EMDRV_NVM_OK)
  { // Try to restore data from wear page, if failed read it from normal page
//    result = NVM_Read(PAGE_WEAR_ID, TABLE_ID);
//    if (result == ECODE_EMDRV_NVM_PAGE_INVALID)

      result = APP_ReadData(PAGE_ID_TYPE, OBJ_ID_TYPE);
      if (result == ECODE_EMDRV_NVM_OK)
      {
    		printMsg("NVM_Read ID OK");
      }

  }
  return result;
}
void PutRadio2Sleep()
{
	printMsg("PutRadio2Sleep");
	ezradio_change_state(EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
	RTCDRV_Delay(100);
	GPIO_PinOutClear(GPIO_TCXO_PORT, GPIO_TCXO_PIN);
}

void WakeupRadio()
{
	printMsg("wakeup Radio");
	GPIO_PinOutSet(GPIO_TCXO_PORT, GPIO_TCXO_PIN);
	RTCDRV_Delay(5);
	ezradio_change_state(EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_READY);
	ezradio_change_state(EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_RX);
}

void GoToSleep()
{
	bool bTimerRun;
//	if (RTCDRV_IsRunning(rtc20SecTimer, &bTimerRun) !=  ECODE_EMDRV_RTCDRV_OK)
//		return;
//		exit(0);
//printMsg("timer run ok");
//	if (!bTimerRun)
//	//	exit(0);
//		return;
	//printMsg("timer run");

	logd("slot is: %d", g_nCurTimeSlot);
	if (RTCDRV_IsRunning(rtcTickTimer, &bTimerRun) == ECODE_EMDRV_RTCDRV_OK)
		if (bTimerRun)
			RTCDRV_StopTimer(rtcTickTimer);

	RTCDRV_Delay(50);
	GPIO_PinOutClear(GPIO_LED1_PORT, GPIO_LED1_PIN);
	PutRadio2Sleep();
	GPIO_PinOutSet(GPIO_LED2_PORT,GPIO_LED2_PIN);
	g_bflagWakeup = false;
	while (g_bflagWakeup == false)
		EMU_EnterEM2(true);

}

bool IsBusySlot(uint8_t slot)
{
	uint32_t temp = 2^slot;

	if ((g_lMySlots & temp) == 1)
		return true;
	return false;
}

void RemoveSensor(uint8_t i)
{
	uint32_t tmp;
	MySensorsArr[i].ID = DEFAULT_ID;
	MySensorsArr[i].slot.status = SLOT_STATUS_EMPTY;
	tmp = ~(1 << MySensorsArr[i].slot.index);
	g_lMySlots &= tmp;
}

void CheckSensorConnection()
{
	uint8_t i;
	for (i = 0; i < MAX_DATA; i++)
	{
		if (MySensorsArr[i].ID != DEFAULT_ID)
			//if sensor hasn't connected in the last 24 hours at all
			if (MySensorsArr[i].DailyCnct == false)
				RemoveSensor(i);
		MySensorsArr[i].DailyCnct = false;
	}
}
/***************************************************************************//**
 * @brief GPIO Interrupt handler (PB0)
 *        Increments the time by one minute.
 ******************************************************************************/
void RTC_App_IRQHandler()
{
  if (rtcTickCnt > 0)
	  rtcTickCnt--;
  if (g_iBtnPressed > 0)
	  g_iBtnPressed++;
  if ((g_nCurTask == TASK_SYNC) && (rtcTickCnt == 0))
	  g_nSec++;
}

void RTC_TimeSlot()
{
	g_nCurTimeSlot++;
	if  ((g_wCurMode == MODE_INSTALLATION) && (rtcTickCnt > 0))
		return;

	GPIO_PinOutClear(GPIO_LED2_PORT, GPIO_LED2_PIN);
	GPIO_PinOutSet(GPIO_LED1_PORT, GPIO_LED1_PIN);
	g_bflagWakeup = true;
	g_nCurTask = TASK_SLEEP;
	// end of hour
	if (g_nCurTimeSlot == 180)
	{
		g_nCurTimeSlot = 0;
		g_nHour++;
		//midnight
		if (g_nHour == 24)
		{
			g_nHour = 0;
			CheckSensorConnection();
		}
	}
/*==========================
	if (g_bStartInstal) //(g_nCurTimeSlot == 5)//
	{
		g_wCurMode = MODE_INSTALLATION;
		g_nCurTask = TASK_WAIT;
		g_bStartInstal = false;
		rtcTickCnt = 6000;// 10 minutes wait
	}
	======================*/
	// if time to listen for irrigation sensor
	//to-do check if there are irrigation sensors at all
//	if (((g_nCurTimeSlot % 15) >= 0) && ((g_nCurTimeSlot % 15) <= 2))
	{
		//to-do: light
		g_wCurMode = MODE_INSTALLATION;//MODE_LISTENING;
		g_nCurTask = TASK_WAIT;
		rtcTickCnt = 190;
//		rtcTickCntEcho = rtcTickCnt;
		return;
	}
	// if time to listen to other sensors
	if ((g_nCurTimeSlot < 30) && (IsBusySlot(g_nCurTimeSlot)))
	{
		g_wCurMode = MODE_LISTENING;
		g_nCurTask = TASK_WAIT;
		rtcTickCnt = 190;
	}
	// if time to send data to logger
	if (g_nCurTimeSlot == g_nHubSlot)
	{
	  g_wCurMode = MODE_SENDING;
	  g_nCurTask = TASK_DO_JOB;
	  g_nRetryCnt = 0;
	}

	// if not going to sleep = restart the 100ms timer
//	if (g_nCurTask != TASK_SLEEP)
//	{
//		if (ECODE_EMDRV_RTCDRV_OK
//		      != RTCDRV_StartTimer(rtcTickTimer, rtcdrvTimerTypePeriodic, APP_RTC_TIMEOUT_MS,
//		                           (RTCDRV_Callback_t)RTC_App_IRQHandler, NULL) )
//		{
//		    //while (1) ;
//		  }
//		//GPIO_PinOutSet(GPIO_LED2_PORT, GPIO_LED2_PIN);
//	}
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

/*
uint32_t Bytes2Long(uint8_t* buf)
{
	 Uint32toBytes temp32bituint;

	for (uint8_t i = 0; i < 4; i++)
		temp32bituint.bVal[i] =  buf[i];

	return temp32bituint.iVal;
}

uint8_t* Long2Bytes(uint32_t l)
{
	 Uint32toBytes temp32bituint;

	 temp32bituint.iVal = l;

	return temp32bituint.bVal;
}

uint16_t Bytes2UInt(uint8_t* buf)
{
	Uint16toBytes tmp;

	for (uint8_t i = 0; i < 2; i++)
		tmp.bVal[i] =  buf[i];

	return tmp.iVal;
}
*/
int16_t Bytes2Int(uint8_t* buf)
{
	Int16toBytes tmp;

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

uint16_t GetSecToConnect(uint8_t nSlot)
{
	return (20 * (180 - g_nCurTimeSlot + nSlot)) - (rtcTickCnt / 10);
}

uint8_t InsertNewSensor(uint32_t senID)
{
	uint8_t i = 0;
	bool bFoundEmpty = false;

	do
	{
		if (MySensorsArr[i].ID != 0)
			if (MySensorsArr[i].ID == senID)
				return i;
			else
				i++;
		else
			bFoundEmpty = true;
	}
	while ((i < MAX_DATA) && (!bFoundEmpty));

	if (bFoundEmpty)
	{
		MySensorsArr[i].ID = senID;
		logd("insert sensor: %d", senID);
		logd("to index: %d",i);
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

uint8_t GetNextFreeSlot(uint8_t senType)
{
	uint8_t index = 0;
	uint32_t temp = 1;
	uint32_t allowSlot;
	if (senType == TYPE_IRRIGATION)
		allowSlot = IRG_SLOTS;
	else
		allowSlot = NOT_IRG_SLOTS;

	do
	{
		if (((g_lMySlots & temp) == 0) && ((temp & allowSlot) != 0))
		{
			g_lMySlots = g_lMySlots & temp;
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
	uint8_t i = 0;
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
	//uint32_t l;
	uint8_t i;
	printMsg("ParseMsg");
	logd("index read in stack %d", gReadStack);
	uint8_t senIndex, size = NewMsgStack[gReadStack][INDEX_SIZE];
	uint8_t nSlot,res = false;

	if (NewMsgStack[gReadStack][size] != GetCheckSum(&NewMsgStack[gReadStack][1],size-1))
	{
		printMsg("wrong CS");
		//
		//return false;
	}
	//copy to struct
	for ( i = 0; i < size; i++)
		((( uint8_t *) &msg)[i]) = NewMsgStack[gReadStack][i+1];

	switch (g_wCurMode)
	{
	case MODE_CONFIGURE:
	{
		// to do: put back
//		if (NewMsgStack[gReadStack][INDEX_HEADER] != HEADER_GETID_ACK)
//			break;
//		if (Bytes2Int(&NewMsgStack[gReadStack][2]) != g_iBtr)
//			break;
		myGnrlInfo.m_ID = msg.Header.m_addressee;//Bytes2Long(&NewMsgStack[gReadStack][4]);
		APP_StoreData(PAGE_ID_TYPE);
		radioTxPkt[INDEX_HEADER] = HEADER_ID_OK;
		radioTxPkt[INDEX_SIZE] = 7;
		radioTxPkt[2] = myGnrlInfo.m_ID;
		res = true;
//		radioTxPkt[6] = GetCheckSum(radioTxPkt, 6);
		break;
	}
	case MODE_LISTENING:
	case MODE_INSTALLATION:
	{
		logd("Header is: %d", msg.Header.m_Header);//NewMsgStack[gReadStack][INDEX_HEADER]);
		// if its data or history msg from sensor
		if ((msg.Header.m_Header == HEADER_MSR) || (msg.Header.m_Header == HEADER_HST))
//		if ((NewMsgStack[gReadStack][INDEX_HEADER] == HEADER_MSR) || (NewMsgStack[gReadStack][INDEX_HEADER] == HEADER_HST))
		{
			if (msg.Header.m_addressee != myGnrlInfo.m_ID)
				if (msg.Header.m_addressee != DEFAULT_ID)
//			if (Bytes2Long(&NewMsgStack[gReadStack][INDEX_TO_ADDRESS]) != myGnrlInfo.m_ID)
//				if (Bytes2Long(&NewMsgStack[gReadStack][INDEX_TO_ADDRESS]) != DEFAULT_ID)
				{
					printMsg("message not for me");
					// msg not for me
					break;
				}

			//l = msg.Header.m_ID;// Bytes2Long(&NewMsgStack[gReadStack][INDEX_FROM_ADDRESS]);
			logd("sensor ID: %d", msg.Header.m_ID);
//			logd("data is: ", msg.DataPayload.m_data);//Bytes2Int(&NewMsgStack[gReadStack][INDEX_DATA]));
//			logd("btr is: ", msg.DataPayload.m_battery);//Bytes2Int(&NewMsgStack[gReadStack][INDEX_DATA]));
//			logd("index is: ", msg.DataPayload.m_index);//Bytes2Int(&NewMsgStack[gReadStack][INDEX_DATA]));
//			logd("type is: ", msg.DataPayload.m_type);//Bytes2Int(&NewMsgStack[gReadStack][INDEX_DATA]));

			senIndex = GetSensorIndex(msg.Header.m_ID);
			// not my sensor
			if (senIndex == MAX_DATA)
			{
				printMsg("not my sensor");
				if (g_wCurMode == MODE_INSTALLATION)
				{
					senIndex = InsertNewSensor(msg.Header.m_ID);
					if (senIndex == MAX_DATA)
					{
						printMsg("not enough soace for more sensor");
					// cant insert this sensor
						break;
					}
				}
				else
					break;
			}
			if (/*NewMsgStack[gReadStack][INDEX_HEADER]*/msg.Header.m_Header == HEADER_MSR)
			{
				Copy(&MySensorsArr[senIndex].data[0], ((uint8_t *) &msg.DataPayload), 5);//&NewMsgStack[gReadStack][INDEX_DATA], 5);
				MySensorsArr[senIndex].data[5] = NewMsgStack[gReadStack][INDEX_RSSI];
				MySensorsArr[senIndex].IsData = true;
				MySensorsArr[senIndex].Status = SEN_STATUS_GOT_DATA;
				printMsg("save data");
				printMsg1(MySensorsArr[senIndex].data, 6);
			}
			if (NewMsgStack[gReadStack][INDEX_HEADER] == HEADER_HST)
			{
				Copy(&MySensorsArr[senIndex].HstrData[0], ((uint8_t *) &msg.HstrPayload), 10);//&NewMsgStack[gReadStack][INDEX_DATA], 10);
				MySensorsArr[senIndex].IsHstr = true;
			}
			MySensorsArr[senIndex].DailyCnct = true;
//			MySensorsArr[senIndex].slot.status = SLOT_STATUS_BUSY;
			uint16_t tmp = msg.DataPayload.m_battery;//Bytes2Int(&NewMsgStack[gReadStack][FIRST_FIELD_LEN+12]);
			nSlot = 0;
			if  ((g_wCurMode == MODE_INSTALLATION) || ((tmp & 0x3000) >= 0x3000))	// its 3rd sending and up- replace slot
			{
				logd("sensor type is: %d", msg.DataPayload.m_type);//NewMsgStack[gReadStack][INDEX_SEN_TYPE]);
				nSlot = GetNextFreeSlot(msg.DataPayload.m_type/*NewMsgStack[gReadStack][INDEX_SEN_TYPE]*/);
				// if no space
				if (nSlot < 30)
				{
					MySensorsArr[senIndex].slot.index = nSlot;
					MySensorsArr[senIndex].slot.status = SLOT_STATUS_BUSY;
					//MySensorsArr[senIndex].slot.status = SLOT_STATUS_STANDBY; ??
					MySensorsArr[senIndex].DailyCnct = true;
				}
				else
					break;
			}
			//Build response
			logd("response for header: %d",msg.Header.m_Header );
			msg.Header.m_Header++;
			logd("new header: %d",msg.Header.m_Header );
			msg.Header.m_size = sizeof(msg.Header) + sizeof(msg.AckPayload) + 1;
			msg.Header.m_addressee = msg.Header.m_ID;
			msg.Header.m_ID = myGnrlInfo.m_ID;
			msg.AckPayload.m_indexEcho = msg.DataPayload.m_index;
			if (nSlot == 0)
				msg.AckPayload.m_slot = 0;
			else
				msg.AckPayload.m_slot = GetSecToConnect(nSlot);
			/*
 			Uint32toBytes l2b;
			Uint16toBytes i2b;
			radioTxPkt[INDEX_HEADER] = NewMsgStack[gReadStack][INDEX_HEADER]+1;
			radioTxPkt[INDEX_SIZE] = 14;
			Copy(&radioTxPkt[INDEX_TO_ADDRESS], &NewMsgStack[gReadStack][INDEX_FROM_ADDRESS], 4);
			l2b.iVal = myGnrlInfo.m_ID;
			Copy(&radioTxPkt[INDEX_FROM_ADDRESS], l2b.bVal, 4);
			if (nSlot == 0)
				i2b.iVal = 0;
			else
				i2b.iVal = GetSecToConnect(nSlot);
			Copy(&radioTxPkt[INDEX_SLOT], i2b.bVal, 2);
			radioTxPkt[INDEX_TX_COUNTER] = NewMsgStack[gReadStack][INDEX_RX_COUNTER];*/
			res = true;
		}
		else
		{
			printMsg("not data msg");
			break;
		}
		break;
	}
	/****************************
	case MODE_INSTALLATION:
	{
		// if its data or history msg from sensor
		if ((NewMsgStack[gReadStack][INDEX_HEADER] == HEADER_MSR) || (NewMsgStack[gReadStack][INDEX_HEADER] == HEADER_HST))
		{
			if (Bytes2Long(&NewMsgStack[gReadStack][INDEX_TO_ADDRESS]) == DEFAULT_ID)
			{
				l = Bytes2Long(&NewMsgStack[gReadStack][INDEX_FROM_ADDRESS]);
				senIndex = InsertNewSensor(l);
				if (senIndex == MAX_DATA)
				{
					printMsg("not enough soace for more sensor");
				// cant insert this sensor
					return false;
				}
				logd("sensor type is: ", NewMsgStack[gReadStack][INDEX_SEN_TYPE]);
				nSlot = GetNextFreeSlot(NewMsgStack[gReadStack][INDEX_SEN_TYPE]);
				logd("this sensor connect slot is: ", nSlot);
				// if no more empty slot
				if (nSlot >= 30)
					return false;
				MySensorsArr[senIndex].slot.index = nSlot;
				MySensorsArr[senIndex].slot.status = SLOT_STATUS_STANDBY;
				MySensorsArr[senIndex].DailyCnct = true;
				//Build response
				Uint32toBytes l2b;
				Uint16toBytes i2b;
				radioTxPkt[INDEX_HEADER] = NewMsgStack[gReadStack][INDEX_HEADER]+1;
				radioTxPkt[INDEX_SIZE] = 14;
				Copy(&radioTxPkt[INDEX_TO_ADDRESS], &NewMsgStack[gReadStack][INDEX_FROM_ADDRESS], 4);
				l2b.iVal = myGnrlInfo.m_ID;
				Copy(&radioTxPkt[INDEX_FROM_ADDRESS], l2b.bVal, 4);
				i2b.iVal = GetSecToConnect(nSlot);
				Copy(&radioTxPkt[INDEX_SLOT], i2b.bVal, 2);
				radioTxPkt[INDEX_TX_COUNTER] = NewMsgStack[gReadStack][INDEX_RX_COUNTER];
			}
		}
		break;
	}
	************************************/
	case  MODE_SENDING:
	{
		printMsg("Ack for data sending");
		//checif its the rifht message type
		if (msg.Header.m_Header != HEADER_SND_DATA_ACK)
			break;
		if (msg.Header.m_addressee != myGnrlInfo.m_ID)
			break;
		/*
		if (NewMsgStack[gReadStack][INDEX_HEADER] != HEADER_SND_DATA_ACK)
			break;
		// check if the message send to this hub
		if (Bytes2Long(&NewMsgStack[gReadStack][INDEX_TO_ADDRESS]) != myGnrlInfo.m_ID)
			// msg not for me
			break;
			*/
		//check if ACK from my logger?
//		if (Bytes2Long(&NewMsgStack[gReadStack][INDEX_FROM_ADDRESS]) != g_LoggerID)
		// if it was broadcast - save the logger id and  HUBSLOT number
		g_nMin = msg.RecAckPayload.m_min;//NewMsgStack[gReadStack][INDEX_MIN];
		g_nSec = msg.RecAckPayload.m_sec;//NewMsgStack[gReadStack][INDEX_SEC];
		logd("minute is: %d", g_nMin);
		//PrintNum(g_nMin);
		logd("Second is: %d", g_nSec);
		//PrintNum(g_nSec);
		if (g_LoggerID == DEFAULT_ID)
		{
			g_LoggerID = msg.Header.m_ID;//Bytes2Long(&NewMsgStack[gReadStack][INDEX_FROM_ADDRESS]);
			logd("logger id: %d", g_LoggerID);
			g_nHubSlot = msg.RecAckPayload.m_slot;//NewMsgStack[gReadStack][INDEX_HUB_SLOT];
			logd("HUBSLOT: %d", g_nHubSlot);
		}
		// sign all data that was send OK - can be removed
		senIndex = 0;
		do
		{
			senIndex = GetNextSensor(senIndex);
			if (senIndex < MAX_DATA)
				if (MySensorsArr[senIndex].Status == SEN_STATUS_SEND_DATA)
					MySensorsArr[senIndex].Status = SEN_STATUS_CELL_EMPTY;
		}
		while (senIndex < MAX_DATA);

		// if more data - do another sending task
		if (g_bIsMoreData)
			g_nCurTask = TASK_DO_JOB;
		res = true;
		break;
	}
	break;
	}
	return res;
}

uint8_t GetSensorData(uint8_t senIndex, uint8_t* tmp)
{
	uint8_t i;//, res = 0;

	if ((MySensorsArr[senIndex].ID == 0) || !(MySensorsArr[senIndex].IsData)) //&& (!MySensorsArr[senIndex].IsHstr)))
		return 0;
//	if (MySensorsArr[senIndex].IsData)
//	{
		tmp[0] = 12;
		tmp[1] = TYPE_DATA;
		Copy(&tmp[2], (uint8_t*)MySensorsArr[senIndex].ID, 4);
		Copy(&tmp[6], MySensorsArr[senIndex].data,6);
//		res = 12;
//	}
		//add history at the end of msg
	if (MySensorsArr[senIndex].IsHstr)
	{
		for (i = 5; i > 0; i--)
		{
			if (Bytes2Int(&MySensorsArr[senIndex].HstrData[(i-1)*2]) != -9999)
				break;
		}
		Copy(&tmp[12], MySensorsArr[senIndex].HstrData, i*2);
		tmp[0] += i*2;		// set new size
//		tmp[res] = res;
//		tmp[res+1] = TYPE_HSTR;
//		Copy(&tmp[res+2], (uint8_t*)MySensorsArr[senIndex].ID, 4);
//		Copy(&tmp[res+6], MySensorsArr[senIndex].HstrData,10);
//		res += 16;
	}
	return tmp[0];
}

uint8_t BuildDataMsg()
{
	uint8_t bufIndex = 0,/*INDEX_DATA,*/ senIndex = 0, i;
//	uint8_t x = 0, tmp[MAX_TX_LEN];
	g_bIsMoreData = false;
//	Uint32toBytes tmp32;
//	Uint16toBytes tmp16;

	msg.Header.m_Header = HEADER_SND_DATA;
	msg.Header.m_addressee = g_LoggerID;
	msg.Header.m_ID = myGnrlInfo.m_ID;
	msg.HubPayload.m_battery = g_iBtr;
	msg.HubPayload.m_index = 1;
	/*
	//set header
	radioTxPkt[INDEX_HEADER] = HEADER_SND_DATA;
	// set logger ID send to
	tmp32.iVal = g_LoggerID;
	Copy(&radioTxPkt[INDEX_TO_ADDRESS],  tmp32.bVal, 4);
	// set this hub ID
	tmp32.iVal = myGnrlInfo.m_ID;
	Copy(&radioTxPkt[INDEX_FROM_ADDRESS], tmp32.bVal, 4);
	// set battery of hub
	tmp16.iVal = g_iBtr;
	Copy(&radioTxPkt[INDEX_BTR], tmp16.bVal, 2);
	// set message index
	radioTxPkt[INDEX_NUM] = 1;
*/
	//set data
	printMsg("BuildDataMsg");
	senIndex = GetNextSensor(senIndex);
	i = GetSensorData(senIndex, &msg.HubPayload.m_data[0]);//&tmp[0]);
	while ((bufIndex + i < MAX_TX_LEN) && (senIndex != MAX_DATA))
	{
//		Copy(&radioTxPkt[bufIndex], &tmp[0], i);
		bufIndex += i;
		//x++;
		MySensorsArr[senIndex].Status = SEN_STATUS_SEND_DATA;
		senIndex = GetNextSensor(senIndex);
		if (senIndex != MAX_DATA)
			i = GetSensorData(senIndex, &msg.HubPayload.m_data[bufIndex]);//&tmp[0]);
	}
	// if no data at all - write 0 as payload size
	if ((g_bOnReset) || (i == 0))
	{
		msg.HubPayload.m_data[0] = 0;
		bufIndex = 1;
	}
//		radioTxPkt[bufIndex++] = 0;
	// if there is more datato send but not enough space - mark flag
	if ((i > 0) && (senIndex != MAX_DATA))
		g_bIsMoreData = true;
	// calc cs
//	radioTxPkt[bufIndex] = GetCheckSum(radioTxPkt, bufIndex);
	//bufIndex++;
	// write size
	/*radioTxPkt[INDEX_SIZE]*/msg.Header.m_size = sizeof(msg.Header) + bufIndex + 4; //4=2 bytes for m_battery + 1 byte for m_index + CS
	return bufIndex;
}

void BuildConfigMsg()
{
	printMsg("BuildConfigMsg");
	radioTxPkt[INDEX_HEADER] = HEADER_GETID;
	radioTxPkt[INDEX_SIZE] = 9;
	radioTxPkt[INDEX_SIZE+1] = g_iBtr;
	Copy(&radioTxPkt[INDEX_SIZE+3], (uint8_t*)Version[0], 4);
	//radioTxPkt[INDEX_SIZE+7] = GetCheckSum(radioTxPkt, INDEX_SIZE+7);
}

void BuildTx()
{
	if (g_wCurMode == MODE_CONFIGURE)
		BuildConfigMsg();
	  else
		  if (g_wCurMode == MODE_SENDING)
			  BuildDataMsg();
}

void BufferEnvelopeTransmit(EZRADIODRV_Handle_t handle)
{
	uint8_t i;
	uint8_t bufLen;

	printMsg("BufferEnvelopeTransmit");
	bufLen = msg.Header.m_size;//radioTxPkt[INDEX_SIZE];
	for ( i = 0; i < bufLen-1; i++)
		radioTxPkt[FIRST_FIELD_LEN+i] = (((const uint8_t *) &msg) [i]);

	radioTxPkt[FIRST_FIELD] = bufLen;

	radioTxPkt[bufLen] = GetCheckSum(&radioTxPkt[INDEX_HEADER], bufLen-1);
	///////////for test:
//	bufLen = 3;
//	radioTxPkt[0] = bufLen;
//	radioTxPkt[1] = 6;
//	radioTxPkt[2] = 6;
//	radioTxPkt[3] = 6;
//	radioTxPkt[4] = 6;
//	radioTxPkt[5] = 6;
//	radioTxPkt[6] = 6;
//	///////////end test:

	printMsg1(radioTxPkt, bufLen+1);
	EZRADIODRV_FieldLength_t fieldLen = {1, bufLen, 0, 0, 0};

	  EZRADIODRV_PacketLengthConfig_t pktLengthConf =
	  { ezradiodrvTransmitLenghtCustomFieldLen, bufLen+1, fieldLen };

	/* Transmit packet */
	//if(ECODE_EMDRV_EZRADIODRV_OK != ezradioStartTransmitConfigured(handle , radioTxPkt))
	  if(ECODE_EMDRV_EZRADIODRV_OK != ezradioStartTransmitCustom(handle, pktLengthConf, radioTxPkt))
	//if(ECODE_EMDRV_EZRADIODRV_OK != ezradioStartTransmitDefault(handle , radioTxPkt) )
	{
		printMsg("transmit Error");
		//while(1);
	}
	  g_nRetryCnt++;
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
  /////////////for card 102
  atexit(TurnOff);

  /* HFXO 48MHz, divided by 1 */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Setup GPIO for pushbuttons. */
  GpioSetup();

  /////////////for card 102
    // TCXO power ON
      GPIO_PinOutSet(GPIO_TCXO_PORT, GPIO_TCXO_PIN);

  SLEEP_Init(NULL,NULL);

  /* Set RTC to generate interrupt 100ms. */
  RTCDRV_Init();
  if (ECODE_EMDRV_RTCDRV_OK
      != RTCDRV_AllocateTimer(&rtcTickTimer) )
  {
    while (1) ;
  }
  if (ECODE_EMDRV_RTCDRV_OK
      != RTCDRV_StartTimer(rtcTickTimer, rtcdrvTimerTypePeriodic, APP_RTC_TIMEOUT_MS,
                           (RTCDRV_Callback_t)RTC_App_IRQHandler, NULL) )
  {
    while (1) ;
  }

  if (ECODE_EMDRV_RTCDRV_OK
      != RTCDRV_AllocateTimer(&rtc20SecTimer) )
  {
    while (1) ;
  }
//  if (ECODE_EMDRV_RTCDRV_OK
//      != RTCDRV_StartTimer(rtc20SecTimer, rtcdrvTimerTypePeriodic, APP_RTC_TIMEOUT_1S * 20,
//                           (RTCDRV_Callback_t)RTC_TimeSlot, NULL) ) {
//    while (1) ;
//  }


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

  // define changeable packet length
  appRadioHandle->packetTx.lenConfig.lenMode = ezradiodrvTransmitLenghtCustomFieldLen;
  appRadioHandle->packetTx.lenConfig.fieldLen.f1 = 1;


  /* Reset radio fifos and start reception. */
  ezradioResetTRxFifo();

  //GPIO_PinOutSet(GPIO_LED2_PORT, GPIO_LED2_PIN);
#if (defined EZRADIO_PLUGIN_RECEIVE)
  ezradioStartRx(appRadioHandle);
#endif


#ifdef DEBUG_MODE
  myGnrlInfo.m_ID = 590000;
	  logd("Hi! ID is: %d", myGnrlInfo.m_ID);
//  	  APP_StoreData(PAGE_ID_TYPE);
#endif
  //APP_RestoreData();
  if (myGnrlInfo.m_ID == 0)
  {
	  g_wCurMode = MODE_CONFIGURE;
  }
  else
  {
	  g_wCurMode = MODE_SENDING;
	  g_bOnReset = true;
	  ///for card 102
	  EnableBtnInt();
	  //g_iBtnPressed = 1;
  }
  g_nCurTask = TASK_DO_JOB;

  fDataIn = 0;
  /* Enter infinite loop that will take care of ezradio plugin manager and packet transmission. */
  while (1)
  {
	  /////////////for card 102
	  if (g_iBtnPressed > 0)
		{
			printMsg("btn PRESSED");
			//bool bIsTimer;
			while (GPIO_PinInGet(GPIO_BTN_PORT, GPIO_BTN_PIN) == 0);
			printMsg("btn up");
			logd("g_iBtnPressed = %d", g_iBtnPressed);
			//PrintNum(g_iBtnPressed);

			RTCDRV_Delay(100);

			// if less than 1.5 sec - do nothing
			//if more than 10 sec - delete configuration of sensor (ID+type) and exit (factory reset)
			//if (n >= 10)
			if (g_iBtnPressed > (APP_RTC_FREQ_HZ * 10))
			{
			}
			else
				//if (n >= 2)
				if (g_iBtnPressed > 25)
				{
					printMsg("btn prsd 1. listen");
					g_nCurTask = TASK_WAIT;
					g_wCurMode = MODE_LISTENING;
					rtcTickCnt = 6000;//10minute listen
				}
				else
					if (g_iBtnPressed < 25)
					{
						printMsg("btn prsd 2");
						g_nCurTask = TASK_DO_JOB;
						g_wCurMode = MODE_SENDING;// MODE_INSTALLATION;

//					if (!g_SensorAlive)
							//exit(0);
					}
			//GPIO_PinOutClear(GPIO_LED1_PORT, GPIO_LED1_PIN);
			EnableBtnInt();
		}
	  //GPIO_PinOutSet(GPIO_LED2_PORT, GPIO_LED2_PIN);

  /* Run radio plug-in manager */
    ezradioPluginManager(appRadioHandle);
    switch (g_nCurTask)
    {
    case  TASK_SYNC:
//    	printMsg("synchronize:");
//    	printMsg("sec is: ");
//    	PrintNum(g_nSec);
//    	if (((g_nMin * 60 + g_nSec) % 20) == 0)
    	if ((g_nSec % 20) == 0)
    	{
    		g_nCurTimeSlot = (g_nMin * 60 + g_nSec) / 20;
    		logd("synchronize! slot is: %d", g_nCurTimeSlot);
    		// start 20 sec timer
    		if (ECODE_EMDRV_RTCDRV_OK
    		      != RTCDRV_StartTimer(rtc20SecTimer, rtcdrvTimerTypePeriodic, APP_RTC_TIMEOUT_1S * 20,
    		                           (RTCDRV_Callback_t)RTC_TimeSlot, NULL) )
    		  {
    		    printMsg("failed to set 20 sec timer");
    		  }
    		//GPIO_PinOutClear(GPIO_LED2_PORT, GPIO_LED2_PIN);
printMsg("before sleep");
    		g_nCurTask = TASK_SLEEP;
    		//GoToSleep();
    	}
    	else
    		if (rtcTickCnt == 0)
    			rtcTickCnt = 10;
    	break;
    case TASK_WAIT:
//    	if (rtcTickCnt != rtcTickCntEcho)
//    	{
//    		rtcTickCntEcho = rtcTickCnt;
//    		if ((rtcTickCntEcho % 10) == 0)
//    			logd("tick= ", rtcTickCnt);
//    	}
		if (fDataIn)
		{
			fDataIn = false;
			gReadStack = GetFirstBusyCell();
			do
			{
				if (ParseMsg() == true)
					if (g_wCurMode != MODE_SENDING)
					{
						//ezradioStartTransmitConfigured( appRadioHandle, radioTxPkt );
						BufferEnvelopeTransmit(appRadioHandle);
					}
					else
						if (g_bOnReset)
						{
							g_bOnReset = false;
							g_nCurTask = TASK_SYNC;
							printMsg("start sync");
							break;
						}
				NewMsgStack[gReadStack][INDEX_STATUS] = CELL_EMPTY;
				gReadStack = GetFirstBusyCell();
			}
			while (gReadStack != MAX_MSG_IN_STACK);
		}
		else
			if (rtcTickCnt == 0)
			{
				printMsg("timeout");
				if (g_wCurMode == MODE_CONFIGURE)
					exit(0);
				if  ((g_wCurMode == MODE_SENDING) && (g_nRetryCnt < MAX_SEND_RETRY))
					g_nCurTask = TASK_DO_JOB;
				else
				{
				// to-do - if send data and hasnt got answer - make it history
				// to-do - if no answer during configuration mode - exit
					g_nCurTask = TASK_SLEEP;
				//GoToSleep();
				}
			}

    break;
    case TASK_DO_JOB:
    {
    	//PutRadio2Sleep();
//    	GoToSleep();
//    	RTCDRV_Delay(1000);
//    	WakeupRadio();

    	printMsg("do job. ");//cur mode is:");
    	//PrintIntNum((int32_t)g_wCurMode);
    	BuildTx();
    	// todo - insert size to TX
    	//ezradioStartTransmitConfigured( appRadioHandle, radioTxPkt );
    	logd("len mode is: %d", appRadioHandle->packetTx.lenConfig.lenMode);
    	//PrintNum(appRadioHandle->packetTx.lenConfig.lenMode);
    	BufferEnvelopeTransmit(appRadioHandle);
//    	printMsg("len of f2 is: ");
//    	PrintNum(appRadioHandle->packetTx.lenConfig.fieldLen.f2);

    		/* Transmit packet */
//    		if(ECODE_EMDRV_EZRADIODRV_OK != ezradioStartTransmitDefault(appRadioHandle , radioTxPkt) )
//    		{
//    			printMsg("transmit Error");
//    			//while(1);
//    		}
    	g_nCurTask = TASK_WAIT;
    	printMsg("wait");
    	rtcTickCnt = 30;
    }
    break;
    case TASK_SLEEP:
    	printMsg("sleep");
     	GoToSleep();
     	if (g_bflagWakeup == true)
     	{
     		bool bTimerRun;
     		WakeupRadio();
    		if (g_nCurTask != TASK_SLEEP)
    		{
    			logd("next task is: %d", g_nCurTask);

      			if (RTCDRV_IsRunning(rtcTickTimer, &bTimerRun) == ECODE_EMDRV_RTCDRV_OK)
     				if (!bTimerRun)
     					if (ECODE_EMDRV_RTCDRV_OK
     				      != RTCDRV_StartTimer(rtcTickTimer, rtcdrvTimerTypePeriodic, APP_RTC_TIMEOUT_MS,
     				                           (RTCDRV_Callback_t)RTC_App_IRQHandler, NULL) )
     				{
     				    printMsg("error start timer");
     				  }
    		}
     	}
     	break;
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

  printMsg("Packet Rec:");
  printMsg2(radioRxPkt,radioRxPkt[0]);
  printMsg("\r\n");

  if ( status == ECODE_EMDRV_EZRADIODRV_OK )
  {
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
		  fDataIn = true;
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
	printMsg("CRC Error");
  if ( status == ECODE_EMDRV_EZRADIODRV_OK ) {
    //printf("-->Pkt  RX: CRC Error\n");

#if (defined EZRADIO_PLUGIN_RECEIVE)
    /* Change to RX state */
    ezradioStartRx(handle);
#endif //#if ( defined EZRADIO_PLUGIN_RECEIVE )
  }
}
#endif //#if ( defined EZRADIO_PLUGIN_CRC_ERROR )


/////////////////////////////////////////////////////////////
/*
#define MAX_PACKET_LENGTH 128
#define FIFO_LENGTH 64

// Functions
bool isTransmited(EZRADIODRV_Handle_t radioHandle); // check tx
bool isRxComplete(EZRADIODRV_Handle_t radioHandle); // check rx

static uint8_t *txPositionInPayload; // pointer for Tx data buffer
static uint8_t *txFront; // pointer for Tx data head data
static uint32_t txBufferPositionInPayload = 0; // buffer position
static uint32_t txNumOfRestByte = 0; // Tx rest bytes
static bool txStatus = 0; // 0 fail or init, 1 success
static uint32_t rxPayloadLength = 0; // Rx payload length
static uint8_t *rxPositionInPayload; // pointer for Rx data buffer
static uint8_t rxPacketData[MAX_PACKET_LENGTH]; // Rx buffer
static uint32_t rxNumOfRestByte = 0; // Rx rest bytes


// ******************************************************************************
//* Tx
// ******************************************************************************
Ecode_t tjEzrTransmit(EZRADIODRV_Handle_t radioHandle, bool updateFields, EZRADIODRV_PacketLengthConfig_t pktLengthConf, uint8_t *pioRadioPacket)
{
	ezradio_cmd_reply_t ezradioReply;

	if ( radioHandle == NULL )
	{
		return ECODE_EMDRV_EZRADIODRV_ILLEGAL_HANDLE;
	}

	// Reset TX FIFO
	ezradio_fifo_info(EZRADIO_CMD_FIFO_INFO_ARG_FIFO_TX_BIT, NULL);

	// Request and check radio device state
	ezradio_request_device_state(&ezradioReply);

	if (ezradioReply.REQUEST_DEVICE_STATE.CURR_STATE == EZRADIO_CMD_REQUEST_DEVICE_STATE_REP_CURR_STATE_MAIN_STATE_ENUM_TX)
	{
		return ECODE_EMDRV_EZRADIODRV_TRANSMIT_FAILED;
	}

	// Update radio packet filed configurations if requested
	if (updateFields)
	{
		radioHandle->packetTx.lenConfig.lenMode = pktLengthConf.lenMode;
		radioHandle->packetTx.lenConfig.pktLen = 0;
		radioHandle->packetTx.lenConfig.pktLen += radioHandle->packetTx.lenConfig.fieldLen.f1 = pktLengthConf.fieldLen.f1;
		radioHandle->packetTx.lenConfig.pktLen += radioHandle->packetTx.lenConfig.fieldLen.f2 = pktLengthConf.fieldLen.f2;
		radioHandle->packetTx.lenConfig.pktLen += radioHandle->packetTx.lenConfig.fieldLen.f3 = pktLengthConf.fieldLen.f3;
		radioHandle->packetTx.lenConfig.pktLen += radioHandle->packetTx.lenConfig.fieldLen.f4 = pktLengthConf.fieldLen.f4;
		radioHandle->packetTx.lenConfig.pktLen += radioHandle->packetTx.lenConfig.fieldLen.f5 = pktLengthConf.fieldLen.f5;
	}

	txBufferPositionInPayload = 0;
	txPositionInPayload = pioRadioPacket;
	txFront = pioRadioPacket;
	txNumOfRestByte = 0;
	txStatus = 0;

	// packet length is larger than TX FIFO
	if (EZRADIO_FIFO_SIZE < radioHandle->packetTx.lenConfig.pktLen)
	{
		ezradio_write_tx_fifo(EZRADIO_FIFO_SIZE, txPositionInPayload);
		txBufferPositionInPayload += EZRADIO_FIFO_SIZE;
		txPositionInPayload += EZRADIO_FIFO_SIZE;
	}
	else
	{
		ezradio_write_tx_fifo(radioHandle->packetTx.lenConfig.pktLen, txPositionInPayload);
		txBufferPositionInPayload += radioHandle->packetTx.lenConfig.pktLen;
		txPositionInPayload += radioHandle->packetTx.lenConfig.pktLen;
	}

	ezradio_start_tx(radioHandle->packetTx.channel, 0x30, 0u);
	while(!isTransmited(radioHandle));

	return ECODE_EMDRV_EZRADIODRV_OK;
}

bool isTransmited(EZRADIODRV_Handle_t radioHandle)
{
	ezradio_cmd_reply_t ezradioReply;

	ezradio_get_int_status(EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_TX_FIFO_ALMOST_EMPTY_PEND_BIT, 0, 0, &ezradioReply);

	// ALMOST_EMPTY Interrupt
	if (ezradioReply.GET_INT_STATUS.PH_STATUS & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_TX_FIFO_ALMOST_EMPTY_PEND_BIT)
	{
		ezradio_get_int_status(EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_TX_FIFO_ALMOST_EMPTY_PEND_BIT, 1, 1, &ezradioReply);
		txNumOfRestByte = radioHandle->packetTx.lenConfig.pktLen - txBufferPositionInPayload;
		if (txNumOfRestByte > RADIO_CONFIGURATION_DATA_PKT_TX_THRESHOLD)
		{
			ezradio_write_tx_fifo(RADIO_CONFIGURATION_DATA_PKT_TX_THRESHOLD, txPositionInPayload);
			txBufferPositionInPayload += RADIO_CONFIGURATION_DATA_PKT_TX_THRESHOLD;
			txPositionInPayload += RADIO_CONFIGURATION_DATA_PKT_TX_THRESHOLD;
		}
		else
		{
			ezradio_write_tx_fifo(txNumOfRestByte, txPositionInPayload);
			txBufferPositionInPayload += txNumOfRestByte;
			txPositionInPayload += txNumOfRestByte;
		}
	}
	// PACKET_SET Interrupt
	if (ezradioReply.GET_INT_STATUS.PH_STATUS & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
	{
		txBufferPositionInPayload = 0;
		txPositionInPayload = NULL;
		txNumOfRestByte = 0;
		txStatus = 1;
		ezradio_get_int_status(EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_TX_FIFO_ALMOST_EMPTY_PEND_BIT, 1, 1, &ezradioReply);
		ezradio_change_state(EZRADIO_CMD_START_TX_ARG_CONDITION_TXCOMPLETE_STATE_ENUM_SLEEP);
		// Callback
		if ( radioHandle->packetTx.userCallback != NULL )
		{
			radioHandle->packetTx.userCallback( radioHandle, ECODE_EMDRV_EZRADIODRV_OK);
		}
		return TRUE;
	}
	return FALSE;
}
*/
