#ifndef BUTTONINPUT_H_
#define BUTTONINPUT_H_

#include "nrf_gpio.h"
#define PIN_BUTTON 10
#define PIN_BUTTON2 9
#define isButtonPressed !nrf_gpio_pin_read(PIN_BUTTON)
#define isButton2Pressed !nrf_gpio_pin_read(PIN_BUTTON2)

#endif