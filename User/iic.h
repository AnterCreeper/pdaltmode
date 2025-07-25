#ifndef __IIC_H
#define __IIC_H

#include <ch32x035_rcc.h>

#define IIC_PORT        GPIOA
#define IIC_PIN_SDA     5
#define IIC_PIN_SCL     6
#define IIC_INVERVAL    1

#define IIC_GPIO_INIT() RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)


/* Internal Macro */

void IIC_Init();
void IIC_Start();
void IIC_Stop();
void IIC_SendByte(uint8_t data);
uint8_t IIC_ReadByte();
int  IIC_WaitAck();
void IIC_SendACK();
void IIC_SendNACK();

#endif
