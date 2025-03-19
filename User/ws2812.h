#ifndef __WS2812_H
#define __WS2812_H

#include <ch32x035_rcc.h>
#include "gpio.h"

#define GREEN   0x070000
#define RED     0x000700
#define BLUE    0x000007
#define CYAN    0x070007
#define YELLOW  0x070700
#define MAGENTA 0x000707

static inline void WS2812_INIT() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_OUT_INIT(GPIOA, GPIO_Pin_0, Bit_RESET); //MCU_STATUS
    return;
}

static inline void WS2812_0() {
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    return;
}

static inline void WS2812_1() {
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BCR = GPIO_Pin_0;
    return;
}

static void WS2812_SetColor(int32_t grb) {
    __disable_irq();
    grb <<= 8;
    for(int i = 0; i < 24; i++) {
        if(grb < 0) WS2812_1();
        else WS2812_0();
        grb <<= 1;
    }
    GPIOA->BCR = GPIO_Pin_0;
    __enable_irq();
    TIM_Delay_Us(300);
    return;
}

#endif