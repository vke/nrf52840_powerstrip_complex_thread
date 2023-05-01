/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>

#include "app_scheduler.h"
#include "app_timer.h"
#include "bsp_thread.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_saadc.h"
#include "nrf_delay.h"
#include "nrf_temp.h"

#include "settings.h"

#include "thread_coap_utils.h"
#include "thread_utils.h"

#include "tuyamcu/tuyamcu.h"

#include <openthread/thread.h>
#include <openthread/platform/alarm-milli.h>

#define ADC_CHANNELS                         2

#define SCHED_QUEUE_SIZE      32
#define SCHED_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE

APP_TIMER_DEF(m_voltage_timer_id);
APP_TIMER_DEF(m_internal_temperature_timer_id);
APP_TIMER_DEF(m_tuyamcu_state_check_timer_id);

uint32_t tuyamcu_last_packet_time = 0;
void relay_state_handler(char sensor_name, int64_t sensor_value);
static void tuyamcu_state_check_timeout_handler(void *p_context);

sensor_subscription sensor_subscriptions[] = {
	{ .sensor_name = 'v', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
	{ .sensor_name = 'V', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
	{ .sensor_name = 't', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },

	{ .sensor_name = '0', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = false, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = relay_state_handler, },
	{ .sensor_name = '1', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = false, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = relay_state_handler, },
	{ .sensor_name = '2', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = false, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = relay_state_handler, },
	{ .sensor_name = '3', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = false, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = relay_state_handler, },
	{ .sensor_name = '4', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = false, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = relay_state_handler, },

	{ .sensor_name = SENSOR_SUBSCRIPTION_NAME_LAST, .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
};

static nrf_saadc_value_t adc_buf[ADC_CHANNELS * ADC_SAMPLES_PER_CHANNEL];

int32_t internal_temp_prev = 0x7FFFFFFF;

int32_t dc_voltage_12_prev = 0x7FFFFFFF;
int32_t dc_voltage_12 = 0;
int32_t dc_voltage_3v3_prev = 0x7FFFFFFF;
int32_t dc_voltage_3v3 = 0;

void update_voltage_attributes_callback(void *p_event_data, uint16_t event_size)
{
	set_sensor_value('v', dc_voltage_3v3, false);
	set_sensor_value('V', dc_voltage_12, false);
}

void saadc_event_handler(nrf_drv_saadc_evt_t const *p_event)
{
	if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
	{
		uint32_t err_code;

		int32_t sums[ADC_CHANNELS];
		for (int i = 0; i < ADC_CHANNELS; i++) {
			sums[i] = 0;
			for (int j = 0; j < ADC_SAMPLES_PER_CHANNEL; j++) {
				sums[i] += p_event->data.done.p_buffer[j * ADC_CHANNELS + i];
			}
			sums[i] = sums[i] / ADC_SAMPLES_PER_CHANNEL;
		}

		if (dc_voltage_12_prev == 0x7FFFFFFF) {
			dc_voltage_12_prev = sums[1];
			dc_voltage_12 = sums[1];
		} else {
			dc_voltage_12 = (dc_voltage_12_prev + dc_voltage_12_prev + dc_voltage_12_prev + sums[1]) >> 2;
			dc_voltage_12_prev = dc_voltage_12;
		}

		if (sums[0] < 0)
			sums[0] = 0;

		if (dc_voltage_3v3_prev == 0x7FFFFFFF) {
			dc_voltage_3v3_prev = sums[0];
			dc_voltage_3v3 = sums[0];
		} else {
			dc_voltage_3v3 = (dc_voltage_3v3_prev + dc_voltage_3v3_prev + dc_voltage_3v3_prev + sums[0]) >> 2;
			dc_voltage_3v3_prev = dc_voltage_3v3;
		}

		err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, ADC_CHANNELS * ADC_SAMPLES_PER_CHANNEL);
		APP_ERROR_CHECK(err_code);

		app_sched_event_put(NULL, 0, update_voltage_attributes_callback);
	} else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE) {
	} else {
		NRF_LOG_INFO("saadc unhandled event: %d", p_event->type);
	}
}

static void adc_configure(void)
{
	ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_saadc_calibrate_offset();
	APP_ERROR_CHECK(err_code);

	while (nrfx_saadc_is_busy());

	nrf_saadc_channel_config_t config0 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
	config0.acq_time = NRF_SAADC_ACQTIME_40US;
	err_code = nrf_drv_saadc_channel_init(0, &config0);
	APP_ERROR_CHECK(err_code);

	nrf_saadc_channel_config_t config1 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
	config1.acq_time = NRF_SAADC_ACQTIME_40US;
	err_code = nrf_drv_saadc_channel_init(1, &config1);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_saadc_buffer_convert(adc_buf, ADC_CHANNELS * ADC_SAMPLES_PER_CHANNEL);
	APP_ERROR_CHECK(err_code);
}

static void voltage_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);

	ret_code_t err_code = nrf_drv_saadc_sample();
	APP_ERROR_CHECK(err_code);
}

static void internal_temperature_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);

	NRF_TEMP->TASKS_START = 1;
	/* Busy wait while temperature measurement is not finished. */
	while (NRF_TEMP->EVENTS_DATARDY == 0) {
	}
	NRF_TEMP->EVENTS_DATARDY = 0;

	int32_t temp = nrf_temp_read();

	NRF_TEMP->TASKS_STOP = 1;

	temp = temp << 2;

	if (internal_temp_prev == 0x7FFFFFFF) {
		internal_temp_prev = temp;
	} else {
		temp = (internal_temp_prev + internal_temp_prev + internal_temp_prev + temp) >> 2;
		internal_temp_prev = temp;

		set_sensor_value('t', temp >> 2, false);
	}
}

static void bsp_event_handler(bsp_event_t event)
{
}

static void thread_state_changed_callback(uint32_t flags, void * p_context)
{
	if (flags & OT_CHANGED_THREAD_ROLE) {
		otDeviceRole device_role = otThreadGetDeviceRole(p_context);
		const char *szRole = "UNKNOWN ROLE";
		switch (device_role) {
			case OT_DEVICE_ROLE_CHILD:
				szRole = "OT_DEVICE_ROLE_CHILD";
#ifndef DISABLE_OT_ROLE_LIGHTS
				bsp_board_led_on(LED_CHILD_ROLE);
				bsp_board_led_off(LED_ROUTER_ROLE);
#endif // DISABLE_OT_ROLE_LIGHTS
				break;
			case OT_DEVICE_ROLE_ROUTER:
				szRole = "OT_DEVICE_ROLE_ROUTER";
#ifndef DISABLE_OT_ROLE_LIGHTS
				bsp_board_led_on(LED_ROUTER_ROLE);
				bsp_board_led_off(LED_CHILD_ROLE);
#endif // DISABLE_OT_ROLE_LIGHTS
				break;
			case OT_DEVICE_ROLE_LEADER:
				szRole = "OT_DEVICE_ROLE_LEADER";
#ifndef DISABLE_OT_ROLE_LIGHTS
				bsp_board_led_on(LED_CHILD_ROLE);
				bsp_board_led_on(LED_ROUTER_ROLE);
#endif // DISABLE_OT_ROLE_LIGHTS
				break;

			case OT_DEVICE_ROLE_DISABLED:
				szRole = "OT_DEVICE_ROLE_DISABLED";
#ifndef DISABLE_OT_ROLE_LIGHTS
				bsp_board_led_off(LED_ROUTER_ROLE);
				bsp_board_led_off(LED_CHILD_ROLE);
#endif // DISABLE_OT_ROLE_LIGHTS
				break;

			case OT_DEVICE_ROLE_DETACHED:
				szRole = "OT_DEVICE_ROLE_DETACHED";
#ifndef DISABLE_OT_ROLE_LIGHTS
				bsp_board_led_off(LED_ROUTER_ROLE);
				bsp_board_led_off(LED_CHILD_ROLE);
#endif // DISABLE_OT_ROLE_LIGHTS
				break;
			default:
				break;
		}
		NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %s", flags, szRole);
	}
}

static void log_init(void)
{
	ret_code_t err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void timer_init(void)
{
	uint32_t error_code = app_timer_init();
	APP_ERROR_CHECK(error_code);

	// Voltage timer
	error_code = app_timer_create(&m_voltage_timer_id, APP_TIMER_MODE_REPEATED, voltage_timeout_handler);
	APP_ERROR_CHECK(error_code);

	// Internal temperature timer
	error_code = app_timer_create(&m_internal_temperature_timer_id, APP_TIMER_MODE_REPEATED, internal_temperature_timeout_handler);
	APP_ERROR_CHECK(error_code);

	// TuyaMCU state check timer
	error_code = app_timer_create(&m_tuyamcu_state_check_timer_id, APP_TIMER_MODE_REPEATED, tuyamcu_state_check_timeout_handler);
	APP_ERROR_CHECK(error_code);
}

static void thread_instance_init(void)
{
	thread_configuration_t thread_configuration =
	{
		.radio_mode            = THREAD_RADIO_MODE_RX_ON_WHEN_IDLE,
		.autocommissioning     = true,
		.default_child_timeout = 10,
		.wipe_settings         = false,
	};

	thread_init(&thread_configuration);
	thread_state_changed_callback_set(thread_state_changed_callback);
}

typedef struct
{
	uint8_t tuyamcu_id;
	uint32_t state_sent_time;
	bool state_set;
	bool state_received;
} tuyamcu_relay_state_t;

tuyamcu_relay_state_t tuyamcu_relay_states[] = {
	{TUYAMCU_RELAY_0, 0, false, false},
	{TUYAMCU_RELAY_1, 0, false, false},
	{TUYAMCU_RELAY_2, 0, false, false},
	{TUYAMCU_RELAY_3, 0, false, false},
	{TUYAMCU_RELAY_4, 0, false, false},
	{0xFF, 0, false, false},
};

uint8_t tuyamcu_get_state_index(uint8_t relay)
{
	for (uint8_t i = 0; tuyamcu_relay_states[i].tuyamcu_id != 0xFF; i++) {
		if (tuyamcu_relay_states[i].tuyamcu_id == relay) {
			return i;
		}
	}
	return 0xFF;
}

static void tuyamcu_state_check_timeout_handler(void *p_context)
{
	uint32_t time_now = otPlatAlarmMilliGetNow();

	if (time_now - tuyamcu_last_packet_time < 50) {
		return;
	}

	for (uint8_t i = 0; tuyamcu_relay_states[i].tuyamcu_id != 0xFF; i++) {
		if (tuyamcu_relay_states[i].state_set == tuyamcu_relay_states[i].state_received) {
			continue;
		}
		if (time_now - tuyamcu_relay_states[i].state_sent_time < 50) {
			continue;
		}
		tuyamcu_relay_states[i].state_sent_time = time_now;
		tuyamcu_send_relay_state(tuyamcu_relay_states[i].tuyamcu_id, tuyamcu_relay_states[i].state_set);
		NRF_LOG_INFO("tuyamcu_send_relay_state, %d, %d, %d", time_now, tuyamcu_relay_states[i].tuyamcu_id, tuyamcu_relay_states[i].state_set);
		tuyamcu_last_packet_time = time_now;
		// one relay per cycle
		return;
	}
}

bool tuyamcu_set_relay_state(uint8_t relay, bool state)
{
	uint8_t state_index = tuyamcu_get_state_index(relay);
	if (state_index == 0xFF) {
		return false;
	}
	tuyamcu_relay_states[state_index].state_set = state;
	NRF_LOG_INFO("tuyamcu_set_relay_state, %d, %d, %d", otPlatAlarmMilliGetNow(), relay, state);
	return true;
}

bool tuyamcu_relay_state_received(uint8_t relay, bool state)
{
	uint8_t state_index = tuyamcu_get_state_index(relay);
	if (state_index == 0xFF) {
		return false;
	}
	tuyamcu_relay_states[state_index].state_received = state;
	tuyamcu_relay_states[state_index].state_set = state;
	NRF_LOG_INFO("tuyamcu_relay_state_received, %d, %d, %d", otPlatAlarmMilliGetNow(), relay, state);
	return true;
}

void relay_state_handler(char sensor_name, int64_t sensor_value)
{
	uint8_t relay = -1;
	switch (sensor_name) {
		case '0': relay = TUYAMCU_RELAY_0; break;
		case '1': relay = TUYAMCU_RELAY_1; break;
		case '2': relay = TUYAMCU_RELAY_2; break;
		case '3': relay = TUYAMCU_RELAY_3; break;
		case '4': relay = TUYAMCU_RELAY_4; break;
		default: break;
	}
	if (relay == -1) {
		NRF_LOG_INFO("relay_state_handler unknown relay number, sensor: %02x", sensor_name);
		return;
	}
	uint8_t state_index = tuyamcu_get_state_index(relay);
	if (state_index != 0xFF) {
		// overwrite set state with last received state
		set_sensor_value(sensor_name, tuyamcu_relay_states[state_index].state_received ? 1 : 0, false);
	}
	NRF_LOG_INFO("relay_state_handler, sensor: %02x, relay: %d, value: %llu", sensor_name, relay, sensor_value);
	tuyamcu_set_relay_state(relay, sensor_value ? true : false);
}

bool tuyamcu_packet_handler(tuyamcu_packet_t* p_packet)
{
	tuyamcu_last_packet_time = otPlatAlarmMilliGetNow();
	if (p_packet->command == TUYAMCU_CMD_RESET) {
		NRF_POWER->GPREGRET = 0xB1;
		NVIC_SystemReset();
		return false;
	}
	return true;
}

bool tuyamcu_relay_handler(uint8_t relay, bool state)
{
	char sensor_name = 0x00;
	switch (relay) {
		case TUYAMCU_RELAY_0: sensor_name = '0'; break;
		case TUYAMCU_RELAY_1: sensor_name = '1'; break;
		case TUYAMCU_RELAY_2: sensor_name = '2'; break;
		case TUYAMCU_RELAY_3: sensor_name = '3'; break;
		case TUYAMCU_RELAY_4: sensor_name = '4'; break;
	}
	if (sensor_name == 0x00) {
		return true;
	}
	NRF_LOG_INFO("tuyamcu_packet_handler, relay: %d, state: %d", sensor_name, state ? 1 : 0);
	tuyamcu_relay_state_received(relay, state);
	set_sensor_value(sensor_name, state ? 1 : 0, false);
	return true;
}

int main(int argc, char * argv[])
{
	log_init();
	APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
	timer_init();
	adc_configure();
	nrf_temp_init();

	tuyamcu_init(tuyamcu_packet_handler, tuyamcu_relay_handler);
	// wait for tuyamcu startup
	nrf_delay_ms(500);

	uint32_t error_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
	APP_ERROR_CHECK(error_code);

	thread_instance_init();
	otPlatRadioSetTransmitPower(thread_ot_instance_get(), RADIO_TRANSMIT_POWER);
	thread_coap_utils_init();

	ret_code_t err_code = app_timer_start(m_voltage_timer_id, APP_TIMER_TICKS(VOLTAGE_TIMER_INTERVAL / ADC_SAMPLES_PER_CHANNEL), NULL);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(m_internal_temperature_timer_id, APP_TIMER_TICKS(INTERNAL_TEMPERATURE_TIMER_INTERVAL), NULL);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(m_tuyamcu_state_check_timer_id, APP_TIMER_TICKS(10), NULL);
	APP_ERROR_CHECK(err_code);

	tuyamcu_send_query_all_data();

	while (true) {
		thread_process();
		app_sched_execute();

		if (NRF_LOG_PROCESS() == false) {
			thread_sleep();
		}
	}
}
