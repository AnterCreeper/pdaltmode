#ifndef __GPIO_H
#define __GPIO_H

#include <ch32x035_gpio.h>

void __GPIO_INIT(GPIO_TypeDef* group, GPIOMode_TypeDef type, uint32_t pin);
void GPIO_OUT_INIT(GPIO_TypeDef* group, uint32_t pin, BitAction state);

#define GPIO_IN_INIT(group,pin)     __GPIO_INIT(group,GPIO_Mode_IN_FLOATING,pin)
#define GPIO_IPU_INIT(group,pin)    __GPIO_INIT(group,GPIO_Mode_IPU,pin)
#define GPIO_IPD_INIT(group,pin)    __GPIO_INIT(group,GPIO_Mode_IPD,pin)
#define GPIO_AF_INIT(group,pin)     __GPIO_INIT(group,GPIO_Mode_AF_PP,pin)
#define GPIO_Analog_INIT(group,pin) __GPIO_INIT(group,GPIO_Mode_AIN,pin)

#endif