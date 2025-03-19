#include "timer.h"
#include "gpio.h"
#include "iic.h"
#include "mpd.h"
#include "debug.h"

#define GETBITS(x, n, m) ((x >> n) & ((1 << (m -n + 1)) - 1))

int __IIC_WriteReg(uint16_t address, uint32_t data, int len, const int imm) {
    if(len < 0) return -1;
    IIC_Start();
    int i = -3;
    int j = 0;
    while(i < len) {
        switch(i) {
        case -3:
            IIC_SendByte(MPD_DEV_ADR << 1);
            break;
        case -2:
            IIC_SendByte(address >> 8);
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
    return 0;
}

int __IIC_ReadReg(uint16_t address, uint32_t data, int len, const int imm) {
    if(len < 0) return -1;
    IIC_Start();
    int i = -4;
    int j = 0;
    while(i < len) {
        switch(i) {
        case -4:
            IIC_SendByte(MPD_DEV_ADR << 1);
            break;
        case -3:
            IIC_SendByte(address >> 8);
            break;
        case -2:
            IIC_SendByte(address & 0xFF);
            break;
        case -1:
            IIC_Start();
            IIC_SendByte((MPD_DEV_ADR << 1) | 0x1);
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
    return imm ? data : 0;
}

static inline int IIC_WriteReg(uint16_t address, void* data, int len) {
    return __IIC_WriteReg(address, (uint32_t)data, len, 0);
}

static inline int IIC_WriteRegI(uint16_t address, uint32_t data) {
    return __IIC_WriteReg(address, data, 4, 1);
}

static inline int IIC_ReadReg(uint16_t address, void* data, int len) {
    return __IIC_ReadReg(address, (uint32_t)data, len, 0);
}

static inline int IIC_ReadRegI(uint16_t address, int len) {
    return __IIC_ReadReg(address, 0, len, 1);
}

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler() {
    if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
        uint32_t sys, irq;
        IIC_ReadReg(0x0564, &irq, 4);
        IIC_ReadReg(0x0508, &sys, 4);
        printf("Interrupt from MPD\r\n");
        printf("mpd_irq: 0x%08x\r\n", irq);
        printf("mpd_sys: 0x%08x\r\n", sys);
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
    return;
}

void MPD_IRQInit() {
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    /* GPIOB.0 ----> EXTI_Line0 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    return;
}

void MPD_Init(){ //Mobile Peripheral Devices
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    printf("Reset MPD\r\n");
    GPIO_IN_INIT(GPIOB, GPIO_Pin_0);    //MCU_DP_INT
    GPIO_OUT_INIT(GPIOB, GPIO_Pin_1, Bit_RESET);    //MCU_DP_RST#
    TIM_Delay_Us(20);    //tRSTON
    //GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
    GPIO_IN_INIT(GPIOB, GPIO_Pin_1);
    uint32_t id = 0;
    IIC_ReadReg(0x0500, &id, 2);
    printf("mpd_id: 0x%04x\r\n", id);
    uint32_t pin = 0;
    IIC_ReadReg(0x0508, &pin, 1);
    printf("mpd_pin: 0x%02x\r\n", pin);

    IIC_WriteRegI(0x06a0, 0x123087);    //Set DP mode
    IIC_WriteRegI(0x07a0, 0x123002);    //Set DP mode
    IIC_WriteRegI(0x0918, 0x0201);      //Set SYSPLL
    IIC_WriteRegI(0x0800, 0x03000007);  //Init DP PHY
    printf("Enable MPD PLL\r\n");
    IIC_WriteRegI(0x0900, 0x05);    //Enable DP0 PLL
    TIM_Delay_Us(500);
    IIC_WriteRegI(0x0904, 0x05);    //Enable DP1 PLL
    TIM_Delay_Us(500);
    IIC_WriteRegI(0x0914, 0x320244);    //Set PXLPLL
    IIC_WriteRegI(0x0908, 0x05);    //Enable SYSPLL
    TIM_Delay_Us(500);
    printf("Enable MPD Func\r\n");
    IIC_WriteRegI(0x0800, 0x13001107);
    IIC_WriteRegI(0x0800, 0x03000007);  //Reset DP PHY
    uint32_t result;
    do {
        TIM_Delay_Us(500);
        IIC_ReadReg(0x0800, &result, 4);
    } while((result & (1 << 16)) == 0); //Query DP PHY Ready
    //printf("mpd_phy: %08x\r\n", result);
    printf("MPD Ready\r\n");

    //printf("Enable MPD IRQ\r\n");
    //MPD_IRQInit();
    //IIC_WriteRegI(0x0560, (1 << 16));   //Enable MPD SYS Interrupt
    return;
}

int MPD_WriteAUXI(uint32_t bsize, uint32_t address, uint32_t data) {
    int j = 0;
    uint32_t status;
    do {
        if(j++ == 3) return -1;
        IIC_WriteRegI(0x0668, address);
        IIC_WriteRegI(0x066C, data);
        IIC_WriteRegI(0x0660, 0x0008 | ((bsize - 1) << 8));
        TIM_Delay_Us(1000);
        IIC_ReadReg(0x068C, &status, 4);
    } while(GETBITS(status, 16, 18));
    //printf("mpd_aux_status: %08x\r\n", status);
    return 0;
}

uint32_t MPD_ReadAuxI(uint32_t bsize, uint32_t address) {
    int j = 0;
    uint32_t status, result = 0;
    do {
        if(j++ == 3) return -1;
        IIC_WriteRegI(0x0668, address);
        IIC_WriteRegI(0x0660, 0x0009 | ((bsize - 1) << 8));
        TIM_Delay_Us(1000);
        IIC_ReadReg(0x068C, &status, 4);
    } while(GETBITS(status, 16, 18));
    IIC_ReadReg(0x067C, &result, bsize);
    //printf("mpd_aux_status: %08x\r\n", status);
    return result;
}

int MPD_ReadAux(uint32_t bsize, uint32_t address, uint32_t* data) {
    int j = 0;
    uint32_t status;
    do {
        if(j++ == 3) return -1;
        IIC_WriteRegI(0x0668, address);
        IIC_WriteRegI(0x0660, 0x0009 | ((bsize - 1) << 8));
        TIM_Delay_Us(1000);
        IIC_ReadReg(0x068C, &status, 4);
    } while(GETBITS(status, 16, 18));
    IIC_ReadReg(0x067C, data, bsize);
    //printf("mpd_aux_status: %08x\r\n", status);
    return 0;
}

int MPD_CfgLink(){
    printf("Configure DP Link\r\n");
    //IIC_WriteRegI(0x0664, 0x00010732);  //Set DP AUX Feature
    uint32_t cap = MPD_ReadAuxI(4, 0x00000);
    printf("dp_sink_cap: 0x%08x\r\n", cap);

    MPD_WriteAUXI(2, 0x00100, 0x820A);  //DP Link Rate and Link Count
    MPD_WriteAUXI(1, 0x00108, 0x01);    //DP Link Coding

    printf("Start Link Training\r\n");
    uint32_t code, status, result;
    MPD_WriteAUXI(2, 0x00103, 0x0202);  //DP PHY Signal
    IIC_WriteRegI(0x06E4, 0x21);        //DP0_SnkLTCtrl, AUX 00102h
    IIC_WriteRegI(0x06D8, 0xF500002A);  //DP0_LTLoopCtrl, Set Training properties
    IIC_WriteRegI(0x06A0, 0x00023086);
    IIC_WriteRegI(0x06A0, 0x00023187);  //DP0_SrcCtrl, Enable Training
    IIC_WriteRegI(0x0600, 0x21);        //DP0Ctl, Enable Output
    do {
        TIM_Delay_Us(1000);
        IIC_ReadReg(0x06D0, &result, 4);
    } while((result & (1 << 13)) == 0); //Query DP0_LTStat Round Finish
    code = GETBITS(result, 8, 12);
    status = GETBITS(result, 0, 6);
    printf("Pattern 1:\r\n");
    printf("code: 0x%02x\r\n", code);
    printf("status: 0x%02x\r\n", status);

    IIC_WriteRegI(0x06E4, 0x22);
    MPD_WriteAUXI(2, 0x00103, 0x0A0A);
    IIC_WriteRegI(0x06A0, 0x00123086);
    IIC_WriteRegI(0x06A0, 0x00123287);
    do {
        TIM_Delay_Us(1000);
        IIC_ReadReg(0x06D0, &result, 4);
    } while((result & (1 << 13)) == 0); //Query DP0_LTStat Link Training Status
    code = GETBITS(result, 8, 12);
    status = GETBITS(result, 0, 6);
    printf("Pattern 2:\r\n");
    printf("code: 0x%02x\r\n", code);
    printf("status: 0x%02x\r\n", status);

    uint32_t result2[2] = {0, 0};
    MPD_ReadAux(7, 0x00200, result2);
    printf("Sink LT Status:\r\n");
    printf("code0: 0x%08x\r\n", result2[0]);
    printf("code1: 0x%08x\r\n", result2[1]);
    uint32_t symbol;
    MPD_ReadAux(4, 0x00210, &symbol);
    printf("symbol_err: 0x%08x\r\n", symbol);
    
    IIC_WriteRegI(0x06A0, 0x00121087);  //Clean LT Pattern
    MPD_WriteAUXI(1, 0x00102, 0x00);    //Clean DPCD LT Pattern
    
    MPD_WriteAUXI(1, 0x0010A, 0x02);    //Set No-ASSR and Enhanced Framing
    MPD_WriteAUXI(1, 0x00107, 0x00);    //Set SSCG
    return 0;
}

#define EDID_DEV_ADR        0x50    //7b1010000
#define EDID_BASE_ADDRESS   0x00

int MPD_ReadEDID(void* data, int bsize) {
    IIC_WriteRegI(0x0668, EDID_DEV_ADR);
    IIC_WriteRegI(0x0660, 0x0014); //Initate Write(Address only)
    TIM_Delay_Us(1000);
    uint32_t code = IIC_ReadRegI(0x068C, 3);
    if(GETBITS(code, 16, 18) || GETBITS(code, 4, 7)) {
        printf("DDC EDID NACK\r\n");
        return -1;
    }
    
    IIC_WriteRegI(0x066C, EDID_BASE_ADDRESS);
    IIC_WriteRegI(0x0660, 0x0004); //Send Register Address, 1 byte
    TIM_Delay_Us(1000);

    IIC_WriteRegI(0x0660, 0x0015); //Initate Read(Address only)
    TIM_Delay_Us(1000);

    while(bsize) {
        uint32_t send_byte = bsize > 16 ? 16 : bsize;
        IIC_WriteRegI(0x0660, 0x0005 | ((send_byte - 1) << 8)); //IIC Receive Data
        TIM_Delay_Us(1000);
        uint32_t code = IIC_ReadRegI(0x068C, 3);
        //FIXME: TC358867XBG aux_status can produce invalid data
        uint32_t recv_byte = /*GETBITS(code, 4, 7) == 0x4 ? 0 : */GETBITS(code, 8, 15);
        IIC_ReadReg(0x067C, data, recv_byte);
        data = data + recv_byte;
        bsize = bsize - recv_byte;
    }

    IIC_WriteRegI(0x0660, 0x0011); //STOP
    return 0;
}

void MPD_Test_EDID(uint32_t data[64]){
    uint8_t* code = (uint8_t*)data;
    printf("EDID:");
    for(int i = 0; i < 8; i++) {
        for(int j = 0; j < 32; j++) printf("%s%02x", j ? " " : "\r\n", code[(i << 5) + j]);
    }
    printf("\r\n");
    return;
}

void MPD_CfgVideo(){
    uint32_t pm = MPD_ReadAuxI(1, 0x00600);
    printf("dp_sink_pm: 0x%02x\r\n", pm);
    uint32_t cap = MPD_ReadAuxI(2, 0x00005);
    printf("dp_sink_type: 0x%04x\r\n", cap);

    uint32_t edid[64]; //256bytes
    if(!MPD_ReadEDID(edid, 256)) {
        MPD_Test_EDID(edid);
    }
    //1920x1080p @30fps
    //Video Input Parameter
    IIC_WriteRegI(0x0450, 0x05800100);  //VPCTRL0
    //HTIM01, HTIM02, VTIM01, VTIM02
    const uint32_t timing[4] = {0x0094002C, 0x00580780, 0x00240005, 0x00040438};
    IIC_WriteReg(0x0454, (void*)timing, 16);
    IIC_WriteRegI(0x0464, 0x01);    //Commit Timing Variable
    IIC_WriteRegI(0x0440, 0x0600);  //DPIPXLFMT, Parrallel Input Electrical Parameter
    //DP Main Stream Parameter
    //DP0_VidSyncDly, DP0_TotalVal, DP0_StartVal, DP0_ActiveVal, DP0_SyncVal
    const uint32_t timing2[5] = {0x003E0840, 0x04650898, 0x002900C0, 0x04380780, 0x8005802C};
    IIC_WriteReg(0x0644, (void*)timing2, 20);
    IIC_WriteRegI(0x0658, 0x033F0020);  //DP0_Misc
    return;
}

void MPD_CfgTest(){
    printf("Enable Test Color Checker Output\r\n");
    IIC_WriteRegI(0x0A00, 0xFFFFFF13);  //TSTCTL
    IIC_WriteRegI(0x0610, 0x000045ED);  //DP0_VidMNGen0, Auto
    IIC_WriteRegI(0x0614, 0x00008025);  //DP0_VidMNGen1, Auto
    IIC_WriteRegI(0x0510, 0x0103);  //SYSCTRL, Set DP Video Source
    TIM_Delay_Us(1000);
    IIC_WriteRegI(0x0600, 0x61);
    IIC_WriteRegI(0x0600, 0x63);    //DP0Ctl Output Enable
    
    TIM_Delay_Ms(500);
    uint32_t result[2] = {0, 0};
    MPD_ReadAux(7, 0x00200, result);
    printf("Sink Normal Status:\r\n");
    printf("code0: 0x%08x\r\n", result[0]);
    printf("code1: 0x%08x\r\n", result[1]);
    uint32_t symbol;
    MPD_ReadAux(4, 0x00210, &symbol);
    printf("symbol_err: 0x%08x\r\n", symbol);
    return;
}