#ifndef BARE_MODULE_BOARD_H
#define BARE_MODULE_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LEDS_NUMBER    0

#define LEDS_ACTIVE_STATE 0

#define LEDS_LIST {}

#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 0

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST {}

#define BSP_SELF_PINRESET_PIN NRF_GPIO_PIN_MAP(0, 19)

#define HWFC           true

#ifdef __cplusplus
}
#endif

#endif // BARE_MODULE_BOARD_H
