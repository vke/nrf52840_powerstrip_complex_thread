#ifndef _TUYAMCU_H__
#define _TUYAMCU_H__

#define TUYAMCU_RELAY_0 0x01
#define TUYAMCU_RELAY_1 0x02
#define TUYAMCU_RELAY_2 0x03
#define TUYAMCU_RELAY_3 0x04
#define TUYAMCU_RELAY_4 0x07

#define TUYAMCU_CMD_PING       0x00
#define TUYAMCU_CMD_STATUS_LED 0x03
#define TUYAMCU_CMD_RESET      0x04
#define TUYAMCU_CMD_SET_DATA   0x06
#define TUYAMCU_CMD_DATA       0x07
#define TUYAMCU_CMD_QUERY_ALL  0x08
#define TUYAMCU_CMD_TEST       0x0e
#define TUYAMCU_CMD_GET_TIME   0x1c

#pragma pack(push, 1)

typedef struct
{
	uint16_t signature;
	uint8_t version;
	uint8_t command;
	uint16_t data_length;
	uint8_t data[];
} tuyamcu_packet_t;

#pragma pack(pop)

typedef bool (*tuyamcu_packet_handler_t)(tuyamcu_packet_t* p_packet);
typedef bool (*tuyamcu_relay_handler_t)(uint8_t relay, bool state);

int tuyamcu_init(tuyamcu_packet_handler_t packet_h, tuyamcu_relay_handler_t relay_h);

int tuyamcu_send_ping();
int tuyamcu_send_wifi_state(uint8_t state);
int tuyamcu_send_relay_state(uint8_t relay, bool state);
int tuyamcu_send_query_all_data();
int tuyamcu_send_time_zero();

#endif // _TUYAMCU_H__
