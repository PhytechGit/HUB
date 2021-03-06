/*
 * define.h
 *
 *  Created on: Feb 26, 2019
 *      Author: Yifat
 */

#ifndef SRC_MYSRC_DEFINE_H_
#define SRC_MYSRC_DEFINE_H_
#include "nvm.h"
#define DEBUG_MODE

typedef enum _SlotStatus
{
	SLOT_STATUS_EMPTY,
	SLOT_STATUS_STANDBY,
	SLOT_STATUS_BUSY,
}SlotStatus;

//typedef enum _ConfigStage
//{
//	CONFIG_STAGE_1,
//	CONFIG_STAGE_2,
//}ConfigStage;

typedef enum _Headers
{
	HEADER_MSR = 		0xB1,
	HEADER_MSR_ACK = 	0xB2,
	HEADER_HST = 		0xB3,
	HEADER_HST_ACK = 	0xB4,
//	HEADER_MSR_ACK_GET_VER = 	0xB5,
//	HEADER_VERSION = 	0xB6,
//	HEADER_VER_ACK = 	0xB7,
	HEADER_GETID = 		0xA6,
	HEADER_GETID_ACK = 	0xA7,
	HEADER_ID_OK = 		0xA8,
	HEADER_SND_DATA = 	0xC1,
	HEADER_SND_DATA_ACK = 0xC2,
}Headers;

typedef enum _WorkingMode
{
	MODE_CONFIGURE,
	MODE_INSTALLATION,
	MODE_LISTENING,
	MODE_SENDING,
//	MODE_SLEEPING,
//	MODE_SYNC,
}WorkingMode;

typedef enum _MsgType
{
	TYPE_DATA,
	TYPE_GPS,
//	TYPE_HSTR,
}MsgType;

typedef enum _SensorStatus
{
	SEN_STATUS_GOT_DATA,
	SEN_STATUS_SEND_DATA,
	SEN_STATUS_CELL_EMPTY,
	SEN_STATUS_GOT_ALERT_DATA,
}SensorStatus;

typedef enum _Task
{
	TASK_WAIT,
	TASK_DO_JOB,
	TASK_SLEEP,
	TASK_SYNC,
	TASK_SEND,
}Task;

typedef union _Uint16toBytes
{
	uint16_t iVal;
    uint8_t bVal[2];
}Uint16toBytes;

typedef union _Int16toBytes
{
	int16_t iVal;
    uint8_t bVal[2];
}Int16toBytes;

typedef union _Uint32toBytes
{
	uint32_t iVal;
    uint8_t bVal[4];
}Uint32toBytes;

typedef struct _Slot
{
	uint8_t 	index;
	SlotStatus	status;
}Slot;

typedef struct _sensor
{
	uint32_t 	ID;
	int16_t		msr;
	uint16_t	btr;
	uint8_t		rssi;
	uint8_t		type;
	//uint8_t data[6];
	int16_t HstrData[5];
//	bool	IsData;
	bool 	IsHstr;
	Slot		slot;
	bool	DailyCnct;
	SensorStatus Status;
} sensor;

//#define MAX_TX_LEN	50
//#define MAX_DATA	30
//todo - find the correct io
#define GPIO_TCXO_PORT			gpioPortC
#define GPIO_TCXO_PIN			10
/////////////for card 102
#define GPIO_MAIN_POWER_PORT	gpioPortF
#define GPIO_MAIN_POWER_PIN		2
#define GPIO_LED1_PORT			gpioPortF	//red light
#define GPIO_LED1_PIN			3
#define GPIO_LED2_PORT			gpioPortF	//yellow light
#define GPIO_LED2_PIN			4
#define GPIO_BTN_PORT 		gpioPortA
#define GPIO_BTN_PIN		1

#define IRG_SLOTS		0x38007
#define NOT_IRG_SLOTS	0x3FFC7FF8

#define DEFAULT_ID	0xFFFFFFFF

#define TYPE_NONE	0
#define TYPE_PIVOT	67
#define TYPE_PLANT	68
#define TYPE_WPS	69
#define TYPE_TENS	71
#define TYPE_SMS	72
#define TYPE_SD		73
#define TYPE_DER	74
#define TYPE_FI_3	83
#define TYPE_AIR_TMP_ALRTS	84
#define TYPE_IRRIGATION		85

#define FIRST_FIELD		0
#define FIRST_FIELD_LEN	1

#define INDEX_HEADER	FIRST_FIELD_LEN+0
/*
#define INDEX_SIZE		FIRST_FIELD_LEN+1
#define INDEX_TO_ADDRESS	FIRST_FIELD_LEN+2
#define INDEX_FROM_ADDRESS	FIRST_FIELD_LEN+6
#define INDEX_DATA		FIRST_FIELD_LEN+13
#define INDEX_BTR		FIRST_FIELD_LEN+10
#define INDEX_NUM		FIRST_FIELD_LEN+12
//#define INDEX_HISTORY		FIRST_FIELD_LEN+17
#define INDEX_TYPE		FIRST_FIELD_LEN+12
#define INDEX_SEN_TYPE		FIRST_FIELD_LEN+14
#define INDEX_SLOT		FIRST_FIELD_LEN+10
#define INDEX_TX_COUNTER	FIRST_FIELD_LEN+12
#define INDEX_RX_COUNTER	FIRST_FIELD_LEN+15
#define INDEX_TX_CS		FIRST_FIELD_LEN+13
#define INDEX_HUB_ID2	FIRST_FIELD_LEN+6
#define INDEX_HUB_SLOT	FIRST_FIELD_LEN+12
#define INDEX_MIN		FIRST_FIELD_LEN+10
#define INDEX_SEC		FIRST_FIELD_LEN+11
*/
#define MAX_AMIT_MSG_LEN	23//sizeof(PrtlHdr) + sizeof(PayloadSenData)>sizeof(PayloadSenHistory)?sizeof(PayloadSenData):sizeof(PayloadSenHistory) + 1
#define INDEX_RSSI		FIRST_FIELD_LEN+MAX_AMIT_MSG_LEN
#define INDEX_STATUS	INDEX_RSSI+1
#define MAX_MSG_LEN		INDEX_STATUS+1

#define CELL_EMPTY	0
#define CELL_BUSY	1
#define MAX_MSG_IN_STACK	5

#define MAX_SEND_RETRY	3
#endif /* SRC_MYSRC_DEFINE_H_ */
