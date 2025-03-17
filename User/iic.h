#ifndef __IIC_H
#define __IIC_H

#include "ch32x035_conf.h"

#ifndef IIC_PORT
#define IIC_PORT        GPIOA
#endif
#ifndef IIC_PIN_SDA
#define IIC_PIN_SDA     5
#endif
#ifndef IIC_PIN_SCL
#define IIC_PIN_SCL     6
#endif

#define IIC_INVERVAL    18

#define IIC_SDA_H()     GPIO_SetBits(IIC_PORT, 1 << IIC_PIN_SDA)
#define IIC_SDA_L()     GPIO_ResetBits(IIC_PORT, 1 << IIC_PIN_SDA)
#define IIC_SDA_X(x)    GPIO_WriteBit(IIC_PORT, 1 << IIC_PIN_SDA, x)
#define I2C_SDA_READ()  GPIO_ReadInputDataBit(IIC_PORT, 1 << IIC_PIN_SDA)

#define IIC_SCL_H()     GPIO_SetBits(IIC_PORT, 1 << IIC_PIN_SCL)
#define IIC_SCL_L()     GPIO_ResetBits(IIC_PORT, 1 << IIC_PIN_SCL)

void IIC_Init();
void IIC_Start();
void IIC_Stop();
void IIC_SendByte(uint8_t data);
uint8_t IIC_ReadByte();
int  IIC_WaitAck();
void IIC_SendACK();
void IIC_SendNACK();

#endif
