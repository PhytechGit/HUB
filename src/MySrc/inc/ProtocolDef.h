
#ifndef PROTOCOL_DEF_H
#define PROTOCOL_DEF_H

#define MAX_PAYLOAD_LEN	49
typedef struct _ProtocolHeader
{
	uint8_t m_Header;
	uint8_t	m_size;
	uint32_t m_addressee;
	uint32_t m_ID;
//	uint8_t	m_cs;
} PrtlHdr;

//0xB1
typedef struct _PayloadSenData
{
	uint16_t	m_data;
	uint16_t	m_battery;
	uint8_t		m_type;
	uint8_t		m_index;
} PayloadSenData;

//0xB3
typedef struct _PayloadSenHistory
{
	int16_t		m_history[5];
} PayloadSenHistory;

//
//typedef struct _PayloadSenVersion
//{
//	int8_t		m_version[4];
//} PayloadSenVersion;

//0xBB / 0xBD
typedef struct _PayloadHubAck
{
	uint16_t		m_slot;
	uint8_t 	m_indexEcho;
} PayloadHubAck;

//0xCA
typedef struct _PayloadHubData
{
	uint16_t	m_battery;
	uint8_t		m_index;
	uint8_t		m_data[MAX_PAYLOAD_LEN];
} PayloadHubData;


//0xCB
typedef struct _PayloadRecAck
{
	uint8_t		m_slot;
	uint8_t 	m_indexEcho;
	uint8_t		m_min;
	uint8_t		m_sec;
} PayloadRecAck;

typedef struct _Sen_Hub_Msg
{
	PrtlHdr Header;
	union
	{
		PayloadSenData 		DataPayload;
		PayloadSenHistory	HstrPayload;
//		PayloadSenVersion	VerPayload;
		PayloadHubAck		AckPayload;
		PayloadHubData		HubPayload;
		PayloadRecAck		RecAckPayload;
	};
//	uint8_t CheckSum;
}Sen_Hub_Rec_Msg;

#endif PROTOCOL_DEF_H
