#ifndef __IIC_DEV_H
#define __IIC_DEV_H

#include "iic.h"

//#define IIC_DEV_DBG

int __IIC_WriteReg(const uint8_t dev, const uint16_t address, uint32_t data, const int len, const int imm, const int b8);
int __IIC_ReadReg(const uint8_t dev, const uint16_t address, uint32_t data, const int len, const int imm, const int b8);

#define IIC_WriteReg16(dev,address,data,len) __IIC_WriteReg(dev, address, (uint32_t)(data), len, 0, 0)
#define IIC_WriteReg16I(dev,address,data,len) __IIC_WriteReg(dev, address, data, len, 1, 0)
#define IIC_ReadReg16(dev,address,data,len) __IIC_ReadReg(dev, address, (uint32_t)(data), len, 0, 0)
#define IIC_ReadReg16I(dev,address,len) __IIC_ReadReg(dev, address, 0, len, 1, 0)

#define IIC_WriteReg8(dev,address,data,len) __IIC_WriteReg(dev, address, (uint32_t)(data), len, 0, 1)
#define IIC_WriteReg8I(dev,address,data,len) __IIC_WriteReg(dev, address, data, len, 1, 1)
#define IIC_ReadReg8(dev,address,data,len) __IIC_ReadReg(dev, address, (uint32_t)(data), len, 0, 1)
#define IIC_ReadReg8I(dev,address,len) __IIC_ReadReg(dev, address, 0, len, 1, 1)

#endif