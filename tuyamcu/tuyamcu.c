#include "nrf.h"
#include "bsp.h"
#include "nrf_log.h"
#include "settings.h"
#include "tuyamcu.h"

#include <openthread/platform/alarm-milli.h>

#include "app_uart.h"
#include "app_error.h"
#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256

#define TUYAMCU_PACKET_HDR_SIZE 6
#define TUYAMCU_MIN_PACKET_SIZE 7
#define TUYAMCU_MAX_PACKET_SIZE 256
uint8_t tuyamcu_recv_buffer[TUYAMCU_MAX_PACKET_SIZE];
uint8_t tuyamcu_recv_pointer = 0;

tuyamcu_packet_handler_t packet_handler = NULL;
tuyamcu_relay_handler_t relay_handler = NULL;

uint8_t tuyamcu_crc(uint8_t *data, size_t data_size)
{
	uint8_t checksum = 0;
	
	for (size_t i = 0; i < data_size; i++) {
		checksum += data[i];
	}

	return checksum;
}

bool tuyamcu_parse_packet(tuyamcu_packet_t *p_packet)
{
	switch (p_packet->command) {
		case TUYAMCU_CMD_PING: {
			break;
		}
		case TUYAMCU_CMD_DATA: {
			// toooo lazy to parse this packet properly
			if (p_packet->data_length == 5 && p_packet->data[1] == 0x01) {
				switch (p_packet->data[0]) {
					case TUYAMCU_RELAY_0:
					case TUYAMCU_RELAY_1:
					case TUYAMCU_RELAY_2:
					case TUYAMCU_RELAY_3:
					case TUYAMCU_RELAY_4:
					{
						if (relay_handler) {
							relay_handler(p_packet->data[0], p_packet->data[4] ? true : false);
						}
						break;
					}
				}
			}
			break;
		}
		case TUYAMCU_CMD_GET_TIME: {
			tuyamcu_send_time_zero();
			break;
		}
	}
	return true;
}

void tuyamcu_byte_received(uint8_t byte)
{
	if (tuyamcu_recv_pointer == 0 && byte != 0x55) {
		// something really wrong here, skip byte
		NRF_LOG_INFO("Invalid start byte for TuyaMCU packet: 0x%02x", byte);
		return;
	}
	if (tuyamcu_recv_pointer == 1 && byte != 0xAA) {
		// something really wrong here, skip whole packet
		tuyamcu_recv_pointer = 0;
		NRF_LOG_INFO("Invalid second byte for TuyaMCU packet: 0x%02x", byte);
		return;
	}
	tuyamcu_recv_buffer[tuyamcu_recv_pointer] = byte;
//	NRF_LOG_INFO("TUYAMCUByteReceived %d: 0x%02x", tuyamcu_recv_pointer, byte);
	tuyamcu_recv_pointer++;

	if (tuyamcu_recv_pointer < TUYAMCU_MIN_PACKET_SIZE) {
		return;
	}

	if (tuyamcu_recv_pointer >= TUYAMCU_MAX_PACKET_SIZE) {
		NRF_LOG_INFO("Too big TuyaMCU packet recvd");
		tuyamcu_recv_pointer = 0;
		return;
	}

	tuyamcu_packet_t *p_packet = (tuyamcu_packet_t *)tuyamcu_recv_buffer;
	uint16_t data_length = ((p_packet->data_length & 0xFF) << 8) | ((p_packet->data_length >> 8) & 0xFF);
	if (data_length >= TUYAMCU_MAX_PACKET_SIZE - TUYAMCU_MIN_PACKET_SIZE) {
		// wrong packet size
		NRF_LOG_INFO("TuyaMCU wrong packet size recvd: %d", p_packet->data_length);
		tuyamcu_recv_pointer = 0;
		return;
	}
	// +1 for crc
	if (data_length + TUYAMCU_PACKET_HDR_SIZE + 1 > tuyamcu_recv_pointer) {
		// incomplete packet recvd
		return;
	}

	uint8_t crc = tuyamcu_crc(tuyamcu_recv_buffer, data_length + TUYAMCU_PACKET_HDR_SIZE);
	if (crc != tuyamcu_recv_buffer[data_length + TUYAMCU_PACKET_HDR_SIZE]) {
		// invalid checksum
		tuyamcu_recv_pointer = 0;
		NRF_LOG_INFO("TuyaMCU invalid packet checksum recvd: %d, %d", crc, tuyamcu_recv_buffer[data_length + 6]);
		return;
	}
	
	p_packet->data_length = data_length;

	bool continue_parse = true;
	if (packet_handler) {
		continue_parse = packet_handler(p_packet);
	}
	if (continue_parse) {
		tuyamcu_parse_packet(p_packet);
	}
	tuyamcu_recv_pointer = 0;
}

void uart_event_handle(app_uart_evt_t *p_event)
{
	switch (p_event->evt_type) {
		case APP_UART_DATA_READY: {
//			NRF_LOG_INFO("APP_UART_DATA_READY");
			uint8_t byte = 0;
			uint8_t cnt = 0;
			while (app_uart_get(&byte) == NRF_SUCCESS) {
				tuyamcu_byte_received(byte);
//				NRF_LOG_INFO("UART byte recvd: %d 0x%02x", cnt, byte);
				cnt++;
			}
			break;
		}
		case APP_UART_FIFO_ERROR: {
			NRF_LOG_ERROR("APP_UART_FIFO_ERROR: %u", p_event->data.error_code);
//			APP_ERROR_HANDLER(p_event->data.error_code);
			break;
		}
		case APP_UART_COMMUNICATION_ERROR: {
			NRF_LOG_ERROR("APP_UART_COMMUNICATION_ERROR: %u", p_event->data.error_communication);
//			APP_ERROR_HANDLER(p_event->data.error_communication);
			break;
		}
		case APP_UART_TX_EMPTY: {
			break;
		}
		case APP_UART_DATA: {
			break;
		}
	}
}

uint32_t app_uart_put_array(uint8_t *data, size_t data_size)
{
	uint32_t error = NRF_SUCCESS;
	for (size_t i = 0; i < data_size; i++) {
		error = app_uart_put(data[i]);
		if (error != NRF_SUCCESS)
			break;
	}
	return error;
}

int tuyamcu_init(tuyamcu_packet_handler_t packet_h, tuyamcu_relay_handler_t relay_h)
{
	NRF_LOG_INFO("TuyaMCU Init");

	packet_handler = packet_h;
	relay_handler = relay_h;

	const app_uart_comm_params_t comm_params =
	{
		UART_TUYAMCU_RX_PIN,
		UART_TUYAMCU_TX_PIN,
		UART_PIN_DISCONNECTED, // RTS
		UART_PIN_DISCONNECTED, // CTS
		APP_UART_FLOW_CONTROL_DISABLED,
		false,
#if defined(UART_PRESENT)
		NRF_UART_BAUDRATE_9600
//		(0x00290000UL)
#else
		NRF_UARTE_BAUDRATE_9600
#endif
	};

	uint32_t err_code = NRF_ERROR_INTERNAL;
	APP_UART_FIFO_INIT(&comm_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, uart_event_handle, APP_IRQ_PRIORITY_LOWEST, err_code);

	APP_ERROR_CHECK(err_code);

	return err_code;
}

int tuyamcu_send_ping()
{
	uint8_t cmd[] = {0x55, 0xAA, 0x00, TUYAMCU_CMD_PING, 0x00, 0x00, 0xFF};
	size_t cmd_size = sizeof(cmd) / sizeof(cmd[0]);
	cmd[cmd_size - 1] = tuyamcu_crc(cmd, cmd_size - 1);
	uint32_t err_code = app_uart_put_array(cmd, cmd_size);
	return err_code;
}

int tuyamcu_send_wifi_state(uint8_t state)
{
	uint8_t cmd[] = {0x55, 0xAA, 0x00, TUYAMCU_CMD_STATUS_LED, 0x00, 0x01, state, 0x00};
	size_t cmd_size = sizeof(cmd) / sizeof(cmd[0]);
	cmd[cmd_size - 1] = tuyamcu_crc(cmd, cmd_size - 1);
	uint32_t err_code = app_uart_put_array(cmd, cmd_size);
	return err_code;
}

int tuyamcu_send_relay_state(uint8_t relay, bool state)
{
	uint8_t cmd[] = {0x55, 0xAA, 0x00, TUYAMCU_CMD_SET_DATA, 0x00, 0x05, relay, 0x01, 0x00, 0x01, state ? 0x01 : 0x00, 0x00};
	size_t cmd_size = sizeof(cmd) / sizeof(cmd[0]);
	cmd[cmd_size - 1] = tuyamcu_crc(cmd, cmd_size - 1);
	uint32_t err_code = app_uart_put_array(cmd, cmd_size);
	return err_code;
}

int tuyamcu_send_query_all_data()
{
	uint8_t cmd[] = {0x55, 0xAA, 0x00, TUYAMCU_CMD_QUERY_ALL, 0x00, 0x00, 0x00};
	size_t cmd_size = sizeof(cmd) / sizeof(cmd[0]);
	cmd[cmd_size - 1] = tuyamcu_crc(cmd, cmd_size - 1);
	uint32_t err_code = app_uart_put_array(cmd, cmd_size);
	return err_code;
}

int tuyamcu_send_time_zero()
{
	uint8_t cmd[] = {0x55, 0xAA, 0x00, TUYAMCU_CMD_GET_TIME, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	size_t cmd_size = sizeof(cmd) / sizeof(cmd[0]);
	cmd[cmd_size - 1] = tuyamcu_crc(cmd, cmd_size - 1);
	uint32_t err_code = app_uart_put_array(cmd, cmd_size);
	return err_code;
}
