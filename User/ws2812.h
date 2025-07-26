#ifndef __WS2812_H
#define __WS2812_H

#include <ch32x035_rcc.h>
#include "gpio.h"

#define BRIGHTNESS  0x07

#define GREEN       (BRIGHTNESS << 16)
#define RED         (BRIGHTNESS << 8)
#define BLUE        (BRIGHTNESS)
#define CYAN        (BLUE | GREEN)
#define YELLOW      (GREEN | RED)
#define MAGENTA     (RED | BLUE)

#define WS2812_PORT     GPIOA
#define WS2812_PIN      GPIO_Pin_0

#define WS2812_GPIO_INIT() RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)

void WS2812_INIT();
void WS2812_SetColor(int32_t grb);

#endif