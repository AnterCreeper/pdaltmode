#ifndef __IIC_DEV_H
#define __IIC_DEV_H

#include "iic.h"

//#define IIC_DEV_DBG

/** @brief Helper to issue IIC write command
 *
 *  @param dev 7-bit device address
 *  @param address register address
 *  @param data register data
 *  @param len data length in bytes
 *  @param imm data arg type, zero represent address of data and one represent immediate number
 *  @param b8 address bit width, zero represent 16-bit and one represent 8-bit
 *
 *  @return negative if failed, otherwise return zero
 */
int __IIC_WriteReg(const uint8_t dev, const uint16_t address, uint32_t data, const int len, const int imm, const int b8);

/** @brief Helper to issue IIC read command
 *
 *  @param dev 7-bit device address
 *  @param address register address
 *  @param data register data
 *  @param len data length in bytes
 *  @param imm data arg type, zero represent address of data and one represent immediate number
 *  @param b8 address bit width, zero represent 16-bit and one represent 8-bit
 *
 *  @return negative if failed, otherwise return data if imm arg doesn't equal zero
 */
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