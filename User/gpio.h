#ifndef __GPIO_H
#define __GPIO_H

#include <ch32x035_gpio.h>

void GPIO_IN_INIT(GPIO_TypeDef* group, uint32_t pin);
void GPIO_IPU_INIT(GPIO_TypeDef* group, uint32_t pin);
void GPIO_OUT_INIT(GPIO_TypeDef* group, uint32_t pin, uint32_t state);

#endif