/*
 * define.h
 *
 *  Created on: Feb 26, 2019
 *      Author: Yifat
 */

#ifndef SRC_MYSRC_DEFINE_H_
#define SRC_MYSRC_DEFINE_H_

#define DEBUG_MODE
#define VERSION = {'H',2,19,1}

typedef enum _SlotStatus
{
	STATUS_EMPTY,
	STATUS_STANDBY,
	STATUS_BUSY,
}SlotStatus;

typedef enum _Headers
{
	HEADER_MSR = 		0xBA,
	HEADER_MSR_ACK = 	0xBB,
	HEADER_HST = 		0xBC,
	HEADER_HST_ACK = 	0xBD,
	HEADER_GETID = 		0xA6,
	HEADER_GETID_ACK = 	0xA7,
	HEADER_ID_OK = 		0xA8,
	HEADER_SND_DATA = 	0xCA,
	HEADER_SND_DATA_ACK = 0xCB,
}Headers;

typedef enum _WorkingMode
{
	MODE_CONFIGURE,
	MODE_INSTALLATION,
	MODE_LISTENING,
	MODE_SENDING,
	MODE_SLEEPING,
}WorkingMode;

typedef enum _MsgType
{
	TYPE_DATA,
	TYPE_HSTR,
}MsgType;

typedef enum _SensorStatus
{
	STATUS_GOT_DATA,
	STATUS_SEND_DATA,
	STATUS_CELL_EMPTY,
}SensorStatus;

typedef union _Uint16toBytes
{
	uint16_t iVal;
    uint8_t bVal[2];
}Uint16toBytes;

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
	uint32_t ID;
	Slot	slot;
	uint8_t data[6];
	uint8_t HstrData[10];
	bool	IsData;
	bool 	IsHstr;
	SensorStatus Status;
} sensor;

#define MAX_TX_LEN	100

//todo - find the correct io
#define GPIO_TCXO_PORT			gpioPortC
#define GPIO_TCXO_PIN			10

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

#define INDEX_HEADER	0
#define INDEX_SIZE		1
#define INDEX_TO_ADDRESS	2
#define INDEX_FROM_ADDRESS	6
#define INDEX_DATA		10
#define INDEX_TYPE		12
#define INDEX_SLOT		10
#define INDEX_TX_COUNTER	11
#define INDEX_RX_COUNTER	17
#define INDEX_TX_CS		12
#define INDEX_RSSI		21
#define INDEX_STATUS	22
#define INDEX_HUB_ID2	6

#define CELL_EMPTY	0
#define CELL_BUSY	1
#define MAX_MSG_IN_STACK	5
#define MAX_MSG_LEN	21


#endif /* SRC_MYSRC_DEFINE_H_ */
