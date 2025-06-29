#include "iic_dev.h"
#include "debug.h"

/*
    dev: 7bit device address
    address: register address
    data: register data
    len: data length in bytes
    imm: data is address or immediate number
    b8: address length, 1: 8b; 0: 16b
*/

int __IIC_WriteReg(const uint8_t dev, const uint16_t address, uint32_t data, const int len, const int imm, const int b8) {
#ifdef IIC_DEV_DBG
    uint32_t dbg = data;
#endif
    if(len < 0) return -1;
    IIC_Start();
    int i = b8 ? -2 : -3;
    int j = 0;
    while(i < len) {
        switch(i) {
        case -3:
            IIC_SendByte(dev << 1);
            break;
        case -2:
            if(b8) IIC_SendByte(dev << 1);
            else IIC_SendByte(address >> 8);
            break;
        case -1:
            IIC_SendByte(address & 0xFF);
            break;
        default:
            IIC_SendByte(imm ? data & 0xFF : ((uint8_t*)data)[i]);
        }
        if(!IIC_WaitAck()) {
            i++;
            if(imm && i > 0) data >>= 8;
        }
        else if(++j == 3) return -1;
    }
    IIC_Stop();
#ifdef IIC_DEV_DBG
    printf("debug IIC: write ");
    if(imm) printf("%08x", dbg);
    else {
        for(int i = len - 1; i >= 0; i--) printf("%02x", ((uint8_t*)data)[i]);
    }
    printf(" at 0x%04x\r\n", address);
#endif
    return 0;
}

int __IIC_ReadReg(const uint8_t dev, const uint16_t address, uint32_t data, const int len, const int imm, const int b8) {
    if(!imm && len <= 0) return -1;
    IIC_Start();
    int i = b8 ? -3 : -4;
    int j = 0;
    while(i < len) {
        switch(i) {
        case -4:
            IIC_SendByte(dev << 1);
            break;
        case -3:
            if(b8) IIC_SendByte(dev << 1);
            else IIC_SendByte(address >> 8);
            break;
        case -2:
            IIC_SendByte(address & 0xFF);
            break;
        case -1:
            IIC_Start();
            IIC_SendByte((dev << 1) | 0x1);
            break;
        default:
            if(imm) data = data | (IIC_ReadByte() << (i << 3));
            else ((uint8_t*)data)[i] = IIC_ReadByte();
            IIC_SendACK();
        }
        if(i >= 0 || !IIC_WaitAck()) {
            i++;
            continue;
        }
        if(++j == 3) return -1;
    }
    IIC_Stop();
#ifdef IIC_DEV_DBG
    printf("debug IIC: read ");
    if(imm) printf("%08x", data);
    else {
        for(int i = len - 1; i >= 0; i--) printf("%02x", ((uint8_t*)data)[i]);
    }
    printf(" at 0x%04x\r\n", address);
#endif
    return imm ? data : 0;
}