#ifndef __SETTINGS__H__
#define __SETTINGS__H__

#define INFO_FIRMWARE_TYPE                   "psc"
#define INFO_FIRMWARE_VERSION                "1.0.1"

#define RADIO_TRANSMIT_POWER                 8

#define SUBSCRIPTION_TIMER_INTERVAL          500
#define INTERNAL_TEMPERATURE_TIMER_INTERVAL  1000
#define VOLTAGE_TIMER_INTERVAL               1000

#define DEFAULT_POLL_PERIOD                  100000
#define DEFAULT_POLL_PERIOD_FAST             50
#define DEFAULT_POLL_PERIOD_FAST_TIMEOUT     500
#define DEFAULT_CHILD_TIMEOUT                240

#define ADC_SAMPLES_PER_CHANNEL              32

#define UART_TUYAMCU_RX_PIN                  30 // RX1
#define UART_TUYAMCU_TX_PIN                  31 // TX1

#define DISABLE_OT_ROLE_LIGHTS               1
#define DISABLE_OT_TRAFFIC_LIGHTS            1

#endif // __SETTINGS__H__
