#ifndef __WS2812_H
#define __WS2812_H

#include <ch32x035_rcc.h>
#include "gpio.h"

#define GREEN       0x070000
#define RED         0x000700
#define BLUE        0x000007
#define CYAN        0x070007
#define YELLOW      0x070700
#define MAGENTA     0x000707

#define WS2812_PORT     GPIOA
#define WS2812_PIN      GPIO_Pin_0

#define WS2812_GPIO_INIT() RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)

void WS2812_INIT();
void WS2812_SetColor(int32_t grb);

#endif