#include "gpio.h"

void __GPIO_INIT(GPIO_TypeDef* group, GPIOMode_TypeDef type, uint32_t pin) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = type;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(group, &GPIO_InitStructure);
    return;
}

void GPIO_OUT_INIT(GPIO_TypeDef* group, uint32_t pin, BitAction state) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_WriteBit(group, pin, state);
    GPIO_Init(group, &GPIO_InitStructure);
    return;
}