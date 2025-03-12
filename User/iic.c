#include "iic.h"

#define IIC_SDA_POS         ((IIC_PIN_SDA & 0x7) * 4)
#define IIC_SDA_CFGMASK     ((-1) ^ (0xF << IIC_SDA_POS))

static inline void Delay_Cyc(size_t cnt) {
    for(size_t i = 0; i < cnt; i++) {
        __asm("nop");
    }
    return;
}

static inline void IIC_SDA_IN() {
#if (IIC_PIN_SDA > 7)
    uint32_t reg = IIC_PORT->CFGHR & IIC_SDA_CFGMASK;
    IIC_PORT->CFGHR = reg | (8 << IIC_SDA_POS);
#else
    uint32_t reg = IIC_PORT->CFGLR & IIC_SDA_CFGMASK;
    IIC_PORT->CFGLR = reg | (8 << IIC_SDA_POS);
#endif
    return;
}

static inline void IIC_SDA_OUT() {
#if (IIC_PIN_SDA > 7)
    uint32_t reg = IIC_PORT->CFGHR & IIC_SDA_CFGMASK;
    IIC_PORT->CFGHR = reg | (3 << IIC_SDA_POS);
#else
    uint32_t reg = IIC_PORT->CFGLR & IIC_SDA_CFGMASK;
    IIC_PORT->CFGLR = reg | (3 << IIC_SDA_POS);
#endif
    return;
}

void IIC_Init() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = (1 << IIC_PIN_SDA) | (1 << IIC_PIN_SCL);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    IIC_SDA_H();
    IIC_SCL_H();
    GPIO_Init(IIC_PORT, &GPIO_InitStructure);
    printf("Enable IIC\r\n");
    return;
}

void IIC_Start() {
    IIC_SDA_H();
    IIC_SCL_H();
    IIC_SDA_OUT();
    Delay_Cyc(IIC_INVERVAL);
    IIC_SDA_L();
    Delay_Cyc(IIC_INVERVAL);
    IIC_SCL_L();
    Delay_Cyc(IIC_INVERVAL);
    return;
}

void IIC_Stop() {
    IIC_SDA_OUT();
    IIC_SDA_L();
    IIC_SCL_H();
    Delay_Cyc(IIC_INVERVAL);
    IIC_SDA_H();
    return;
}

void IIC_SendByte(uint8_t data) {
    IIC_SDA_OUT();
    for(int i = 0; i < 8; i++) {
        IIC_SDA_X((data & 0x80) > 0);
        Delay_Cyc(IIC_INVERVAL);
        IIC_SCL_H();
        Delay_Cyc(IIC_INVERVAL);
        IIC_SCL_L();
        data <<= 1;
    }
    return;
}

uint8_t IIC_ReadByte() {
    int value = 0;
    IIC_SDA_IN();
    for(int i = 0; i < 8; i++) {
        IIC_SCL_H();
        Delay_Cyc(IIC_INVERVAL);
        value <<= 1;
        value |= I2C_SDA_READ();
        IIC_SCL_L();
        Delay_Cyc(IIC_INVERVAL);
    }
    return value;
}

int IIC_WaitAck() {
    IIC_SDA_IN();
    Delay_Cyc(IIC_INVERVAL);
    IIC_SCL_H();
    Delay_Cyc(IIC_INVERVAL);
    int value = I2C_SDA_READ(); //H: NACK; L: ACK;
    IIC_SCL_L();
    Delay_Cyc(IIC_INVERVAL);
    IIC_SDA_H();
    return -value;
}

void IIC_SendACK() {
    IIC_SDA_L();
    IIC_SDA_OUT();
    Delay_Cyc(IIC_INVERVAL);
    IIC_SCL_H();
    Delay_Cyc(IIC_INVERVAL);
    IIC_SCL_L();
    Delay_Cyc(IIC_INVERVAL);
    IIC_SDA_H();
    return;
}

void IIC_SendNACK() {
    IIC_SDA_H();
    IIC_SDA_OUT();
    Delay_Cyc(IIC_INVERVAL);
    IIC_SCL_H();
    Delay_Cyc(IIC_INVERVAL);
    IIC_SCL_L();
    Delay_Cyc(IIC_INVERVAL);
    return;
}
