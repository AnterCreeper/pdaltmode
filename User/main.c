#define TARGET_DEBUG
//#define IGNORE_HPD
//#define NO_MPD
//#define FORCE_SOFTMUL

#include "debug.h"
#include <ch32x035_usbpd.h>
#include <ch32x035_opa.h>

void __assert(const char* str, uint32_t line) {
    printf("assert fault at line %d, reason %s\r\n", line, str);
    while(1) __WFI();
    return;
}
#define assert(x) __assert(x, __LINE__)

#define GETBITS(x, n, m) ((x >> n) & ((1 << (m -n + 1)) - 1))

#include "iic.h"

#define IIC_DEV_ADR     (0x68 << 1)

int IIC_WriteReg(uint16_t address, uint32_t* data, size_t len) {
    IIC_Start();
    IIC_SendByte(IIC_DEV_ADR | 0x0);
    if(IIC_WaitAck()) return -1;
    IIC_SendByte(address >> 8);
    if(IIC_WaitAck()) return -1;
    IIC_SendByte(address & 0xFF);
    if(IIC_WaitAck()) return -1;
    for(size_t i = 0; i < len; i++) {
        IIC_SendByte(((uint8_t*)data)[i]);
        if(IIC_WaitAck()) return -1;
    }
    IIC_Stop();
    return 0;
}

int IIC_WriteRegI(uint16_t address, uint32_t data) {
    const int len = 4; //TC358867XBG must transfer at 4 bytes one time
    IIC_Start();
    IIC_SendByte(IIC_DEV_ADR | 0x0);
    if(IIC_WaitAck()) return -1;
    IIC_SendByte(address >> 8);
    if(IIC_WaitAck()) return -1;
    IIC_SendByte(address & 0xFF);
    if(IIC_WaitAck()) return -1;
    for(size_t i = 0; i < len; i++) {
        IIC_SendByte(data & 0xFF);
        if(IIC_WaitAck()) return -1;
        data >>= 8;
    }
    IIC_Stop();
    return 0;
}

int IIC_ReadReg(uint16_t address, void* data, size_t len) {
    IIC_Start();
    IIC_SendByte(IIC_DEV_ADR | 0x0);
    if(IIC_WaitAck()) return -1;
    IIC_SendByte(address >> 8);
    if(IIC_WaitAck()) return -1;
    IIC_SendByte(address & 0xFF);
    if(IIC_WaitAck()) return -1;
    
    IIC_Start();
    IIC_SendByte(IIC_DEV_ADR | 0x1);
    if(IIC_WaitAck()) return -1;
    for(size_t i = 0; i < len; i++) {
        ((uint8_t*)data)[i] = IIC_ReadByte();
        IIC_SendACK();
    }
    IIC_Stop();
    return 0;
}

void SYS_INIT() {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
#if SDI_PRINT == SDI_PR_CLOSE
    USART_Printf_Init(115200);
#else
    SDI_Printf_Enable();
#endif
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    printf("sys_clk: %d\r\n", SystemCoreClock);
    printf("chip_id: 0x%08x\r\n", DBGMCU_GetCHIPID());
    return;
}

void SYS_SLP() {
    EXTI_ClearITPendingBit(EXTI_Line14);
    EXTI_ClearITPendingBit(EXTI_Line15);
    NVIC_EnableIRQ(EXTI15_8_IRQn);
    printf("Enter SLP Mode\r\n");
#ifndef TARGET_DEBUG
    PWR_EnterSTANDBYMode();
#else
    __WFI();
#endif
    return;
}

void VSNS_INIT() {
    printf("Enable ADC\r\n");
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure); //MCU_VSENSE
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure); //MCU_PGA_OUT

    OPA_Unlock();
    OPA_InitTypeDef OPA_InitStructure = {0};
    OPA_InitStructure.OPA_NUM = OPA2;
    OPA_InitStructure.PSEL = CHP0;
    OPA_InitStructure.NSEL = CHN_PGA_4xIN;
    OPA_InitStructure.Mode = OUT_IO_OUT0;
    OPA_InitStructure.FB = FB_ON;
    OPA_Init(&OPA_InitStructure);
    OPA_Cmd(OPA2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_DeInit(ADC1);
    ADC_CLKConfig(ADC1, ADC_CLK_Div6);
    ADC_InitTypeDef ADC_InitStructure = {0};
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);

    return;
}

uint32_t VSNS_Read() {
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_11Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    uint32_t vin = ADC_GetConversionValue(ADC1);
    //vbus = (vin/2^12)*3300mV/4PGA/30kohm*(180+30)kohm
#if(!defined __riscv_mul || defined FORCE_SOFTMUL)
    uint32_t result = vin + (vin << 2);
    result = result + (result << 3);
    result = vin + (result << 3);
    result = result >> 8;
#else
    uint32_t result = (vin * 5775) >> 12;
#endif
    return result;
}

volatile uint32_t TIM1_CNT;

void TIM1_INIT() {
    printf("Enable USBPD Timer\r\n");
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = 499;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 47;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM1_CNT = 0;
    TIM_Cmd(TIM1, ENABLE);
    return;
}

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler() {
    if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM1_CNT++;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
    return;
}

void EXTI_INIT() {
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    /* GPIOC.14 ----> EXTI_Line14 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* GPIOC.15 ----> EXTI_Line15 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource15);
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    return;
}

void GPIO_PD_INIT() {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);               /* Open PD I/O clock, AFIO clock and PD clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    return;
}

void GPIO_IN_INIT(GPIO_TypeDef* group, uint32_t pin) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(group, &GPIO_InitStructure);
    return;
}

void GPIO_OUT_INIT(GPIO_TypeDef* group, uint32_t pin, uint32_t state) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_WriteBit(group, pin, state);
    GPIO_Init(group, &GPIO_InitStructure);
    return;
}

void GPIO_Toggle_INIT() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_OUT_INIT(GPIOA, GPIO_Pin_0, Bit_RESET); //MCU_STATUS
    GPIO_OUT_INIT(GPIOA, GPIO_Pin_1, Bit_SET); //TYPEC_VEN#
    GPIO_OUT_INIT(GPIOA, GPIO_Pin_3, Bit_SET); //TYPEC_SEL#

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_OUT_INIT(GPIOB, GPIO_Pin_3, Bit_SET); //TYPEC_EN#
    GPIO_OUT_INIT(GPIOB, GPIO_Pin_11, Bit_SET); //TYPEC_VCONN1
    GPIO_OUT_INIT(GPIOB, GPIO_Pin_12, Bit_SET); //TYPEC_VCONN2

    printf("Enable GPIO\r\n");
    return;
}

static inline void WS2812_0() {
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    GPIOA->BCR = GPIO_Pin_0;
    return;
}

static inline void WS2812_1() {
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BSHR = (GPIO_Pin_0 & (uint32_t)0x0000FFFF);
    GPIOA->BCR = GPIO_Pin_0;
    return;
}

static inline void WS2812_Reset() {
    GPIOA->BCR = GPIO_Pin_0;
    Delay_Us(300);
    return;
}

#define GREEN   0x070000
#define RED     0x000700
#define BLUE    0x000007
#define CYAN    0x070007
#define YELLOW  0x070700
#define MAGENTA 0x000707

void WS2812_SetColor(int32_t grb) {
    grb <<= 8;
    for(int i = 0; i < 24; i++) {
        if(grb < 0) WS2812_1();
        else WS2812_0();
        grb <<= 1;
    }
    WS2812_Reset();
    return;
}

void MPD_Init(){ //Mobile Peripheral Devices
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    printf("Reset MPD\r\n");
    GPIO_IN_INIT(GPIOB, GPIO_Pin_0);    //MCU_DP_INT
    GPIO_OUT_INIT(GPIOB, GPIO_Pin_1, Bit_RESET);    //MCU_DP_RST#
    Delay_Us(2);    //tRSTON
    GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
    uint32_t id = 0;
    IIC_ReadReg(0x0500, &id, 2);
    printf("mpd_id: 0x%04x\r\n", id);
    //0.8v SW, 3.5dB
    IIC_WriteRegI(0x06a0, 0x123087);    //Set DP mode
    IIC_WriteRegI(0x07a0, 0x123002);    //Set DP mode
    IIC_WriteRegI(0x0918, 0x0201);      //Set SYSPLL
    IIC_WriteRegI(0x0800, 0x03000007);  //Init DP PHY
    printf("Enable MPD PLL\r\n");
    IIC_WriteRegI(0x0900, 0x05);    //Enable DP0 PLL
    Delay_Us(500);
    IIC_WriteRegI(0x0904, 0x05);    //Enable DP1 PLL
    Delay_Us(500);
    IIC_WriteRegI(0x0914, 0x320245);    //Set PXLPLL
    IIC_WriteRegI(0x0908, 0x05);    //Enable SYSPLL
    Delay_Us(500);
    printf("Enable MPD Func\r\n");
    IIC_WriteRegI(0x0800, 0x13001107);
    IIC_WriteRegI(0x0800, 0x03000007);  //Reset DP PHY
    uint32_t result;
    do {
        Delay_Us(500);
        IIC_ReadReg(0x0800, &result, 4);
    } while((result & (1 << 16)) == 0); //Query DP PHY Ready
    printf("MPD Ready\r\n");
    return;
}

void MPD_WriteAUXI(uint32_t bsize, uint32_t address, uint32_t data) {
    IIC_WriteRegI(0x0668, address);
    IIC_WriteRegI(0x066C, data);
    IIC_WriteRegI(0x0660, 0x0008 | ((bsize - 1) << 8));
    Delay_Ms(2);
    return;
}

uint32_t MPD_ReadAuxI(uint32_t bsize, uint32_t address) {
    IIC_WriteRegI(0x0668, address);
    IIC_WriteRegI(0x0660, 0x0009 | ((bsize - 1) << 8));
    uint32_t result = 0;
    Delay_Ms(2);
    IIC_ReadReg(0x067C, &result, bsize);
    return result;
}

void MPD_ReadAux(uint32_t bsize, uint32_t address, uint32_t* data) {
    IIC_WriteRegI(0x0668, address);
    IIC_WriteRegI(0x0660, 0x0009 | ((bsize - 1) << 8));
    Delay_Ms(1);
    IIC_ReadReg(0x067C, data, bsize);
    return;
}

int MPD_CfgLink(){
    printf("Configure DP Link\r\n");
    IIC_WriteRegI(0x0664, 0x00010632);  //Set DP AUX Feature, Filter ON, 1MHz, 400us timeout(50cyc@125kHz)
    uint32_t cap = MPD_ReadAuxI(3, 0x00000);
    printf("dp_sink_cap: 0x%06x\r\n", cap);
    if(!cap) {
        printf("DP AUX Timeout\r\n");
        return -1;
    }
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
        Delay_Ms(1);
        IIC_ReadReg(0x06D0, &result, 4);
    } while((result & (1 << 13)) == 0); //Query DP0_LTStat Link Training Status
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
        Delay_Ms(1);
        IIC_ReadReg(0x06D0, &result, 4);
    } while((result & (1 << 13)) == 0); //Query DP0_LTStat Link Training Status
    code = GETBITS(result, 8, 12);
    status = GETBITS(result, 0, 6);
    printf("Pattern 2:\r\n");
    printf("code: 0x%02x\r\n", code);
    printf("status: 0x%02x\r\n", status);

    IIC_WriteRegI(0x06A0, 0x00121087);  //Clean LT Pattern
    MPD_WriteAUXI(1, 0x00102, 0x00);    //Clean DPCD LT Pattern

    uint32_t result2[2] = {0, 0};
    MPD_ReadAux(7, 0x00200, result2);
    printf("Sink LT Status:\r\n");
    printf("code0: 0x%08x\r\n", result2[0]);
    printf("code1: 0x%08x\r\n", result2[1]);
    
    MPD_WriteAUXI(1, 0x0010A, 0x02);    //Set No-ASSR and Enhanced Framing
    return 0;
}

void MPD_CfgVideo(){
    //1920x1080@60fps
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
    IIC_WriteRegI(0x0658, 0x1F3F0020);  //DP0_Misc
    return;
}

void MPD_CfgTest(){
    printf("Enable Test Color Checker Output\r\n");
    IIC_WriteRegI(0x0A00, 0x13);    //TSTCTL
    IIC_WriteRegI(0x0610, 0x0000473F);  //DP0_VidMNGen0, Auto
    IIC_WriteRegI(0x0614, 0x000080AC);  //DP0_VidMNGen1, Auto
    IIC_WriteRegI(0x0600, 0x61);
    IIC_WriteRegI(0x0600, 0x63);    //DP0Ctl Output Enable
    IIC_WriteRegI(0x0510, 0x0903);  //SYSCTRL, Set DP Video Source
    return;
}

void GPIO_Toggle_VBUS(int state){
    printf("Toggle VBUS %s\r\n", state ? "ON" : "OFF");
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, state ? Bit_RESET : Bit_SET);
    return;
}

void GPIO_Toggle_VCONN(int state){
    printf("Toggle VCONN %s\r\n", state > 0 ? "ON" : "OFF");
    if(state > 2) assert("Bad VCONN Toggle Pin");
    GPIO_WriteBit(GPIOB, GPIO_Pin_11, state & 0x2 ? Bit_RESET : Bit_SET); //TYPEC_VCONN1
    GPIO_WriteBit(GPIOB, GPIO_Pin_12, state & 0x1 ? Bit_RESET : Bit_SET); //TYPEC_VCONN2
    return;
}

void GPIO_Toggle_DPSEL(int state){
    int sel = state ^ ((USBPD->CONFIG & CC_SEL) ? Bit_RESET : Bit_SET);
    printf("Toggle DP MUX %s\r\n", sel ? "Normal" : "Flip");
    GPIO_WriteBit(GPIOA, GPIO_Pin_3, sel); //TYPEC_SEL#
    return;
}

void GPIO_Toggle_DPSIG(int state){
    printf("Toggle DP SIGNAL %s\r\n", state ? "ON" : "OFF");
    GPIO_WriteBit(GPIOB, GPIO_Pin_3, state ? Bit_RESET : Bit_SET); //TYPEC_EN#
    return;
}

typedef enum {
    STA_VDM_IDLE = 0,
    STA_DISC_IDENT,
    STA_DISC_SVID,
    STA_DISC_MODE,
    STA_ENTER_MODE,
    STA_DP_S_UPDATE,
    STA_DP_CONFIG,
    STA_MPD_CONFIG,
} VDM_STATUS;

typedef enum {
    DISCONNECTED = 0,
    CONNECTED,
} PHY_STATUS;

typedef enum {
    ROLE_UFP = 0,
    ROLE_DFP,
} PD_ROLE;

typedef enum {
    ROLE_SNK = 0,
    ROLE_SRC,
} PR_ROLE;

#define DEF_CMDTYPE_INIT            0x0
#define DEF_CMDTYPE_ACK             0x1
#define DEF_CMDTYPE_NAK             0x2
#define DEF_CMDTYPE_BUSY            0x3

#define DEF_SVID_EOF                0x0000
#define DEF_SVID_DEFAULT            0xff00
#define DEF_SVID_DISPLAYPORT        0xff01

/* Power Data Object of Source Capability
    BIT[31:30] - 00: Fixed Supply;
    BIT29 - Dual-Role Power
    BIT28 - USB Suspend Supported

    BIT27 - Unconstrained Power
    BIT26 - USB Communications Capable
    BIT25 - Dual-Role Data

    BIT[24:22] - 000: Reserved
    BIT[21:20] - Peak Current
    BIT[19:10] - Voltage in 50mV units
    BIT[9:0] - Maximum Current in 10mA units
*/
const uint32_t SRCCAP_5V1A5 = 0x2c019096;

//pre malloced buffer
__attribute__ ((aligned(4))) uint8_t PD_Rx_Buf[34]; /* PD receive buffer */
__attribute__ ((aligned(4))) uint8_t PD_Tx_Buf[34]; /* PD send buffer */

typedef struct DP_STATUS {
    uint8_t Pos;
    uint8_t Enabled;
    uint8_t DFP_Pin;
    uint8_t UFP_Pin;
    uint32_t Link;
    uint8_t LT; //If Link Trained
    uint8_t Receptable;
} dp_status_t;

typedef struct PD_FSM {
    PD_ROLE PD_Role;
    PR_ROLE PR_Role;
    uint8_t Msg_ID;
    uint8_t Det_Cnt;
    uint8_t Cap_Cnt;
    CC_STATUS Status;
    PHY_STATUS Connect;
    VDM_STATUS VDM_Status;
    void* Rx_Buf;
    void* Tx_Buf;
    int IRQ_ret;
    void* IRQ_func;
    dp_status_t DP_Status;
} pd_state_t;

pd_state_t* USBPD_FSM;

#define PD_PARSE_MSGID(x)           GETBITS(((uint8_t*)x)[1], 1, 3)
#define PD_PARSE_TYPE(x)            GETBITS(((uint8_t*)x)[0], 0, 4)
#define PD_PARSE_SIZE(x)            GETBITS(((uint8_t*)x)[1], 4, 6)

/* Vender Defined Message
    BIT[31:16] - Standard or Vendor ID, 0xFF00 for default and 0xFF01 for DisplayPort
    BIT15 - VDM Type, 1 = Structured VDM
    BIT[14:13] - Structured VDM Version, 00 = Version 1.0
    BIT[12:11] - 00: Reserved

    BIT[10:8] - Object Position
        000 = Reserved
        001 ¨C 110 = Index into the list of Vendor Defined Objects (VDOs) to identify the desired
        111 = Exit all Modes (equivalent of a power-on reset). Shall not be used with the Enter
            Mode command.

    BIT[7:6] - Command Type
        00 = Initiator
        01 = Responder ACK
        10 = Responder NAK
        11 = Responder BUSY

    BIT5 - Reserved
    BIT[4:0] - Command
*/

#define PD_PARSE_CMD(x)             GETBITS(((uint8_t*)x)[2], 0, 4)
#define PD_PARSE_CMDTYPE(x)         GETBITS(((uint8_t*)x)[2], 6, 7)
#define PD_PARSE_SVID(x)            ((uint16_t*)x)[2]

/* DisplayPort Capability
    BIT[31:24] - Reserved
    BIT[23:16] - UFP_D Pin Assignments supported
    BIT[15:8] - DFP_D Pin Assignments supported
    BIT7 - USB 2.0 signaling not required when set
    BIT6 - Receptacle indication, 0: plug; 1: receptacle;
    BIT[5:2] - Physical Signaling
    BIT[1:0] - Port capability
        00 = Reserved
        01 = UFP_D capable (including Branch Device)
        10 = DFP_D capable (including Branch Device)
        11 = Both DFP_D and UFP_D capable
*/

#define PD_PARSE_DP_CAP(x)          GETBITS(x, 0, 1)
#define PD_PARSE_DP_CAP_UFP_D(x)    GETBITS(x, 0, 0)
#define PD_PARSE_DP_CAP_DFP_D(x)    GETBITS(x, 1, 1)
#define PD_PARSE_DP_SIG(x)          GETBITS(x, 2, 2)
#define PD_PARSE_RECEPTACLE(x)      GETBITS(x, 6, 6)
#define PD_PARSE_DFP_PIN(x)         GETBITS(x, 8, 15)
#define PD_PARSE_UFP_PIN(x)         GETBITS(x, 16, 23)

typedef struct {
    uint32_t id_header;
    uint32_t cert_state;
    uint32_t product;
    uint32_t product_type;
} pd_vdo_ident_t;

#define PD_PARSE_USB_VID(x)         GETBITS(x, 0, 15)
#define PD_PARSE_MODAL(x)           GETBITS(x, 26, 26)
#define PD_PARSE_PROD_TYPE(x)       GETBITS(x, 27, 29)
#define PD_PARSE_USB_DEVICE(x)      GETBITS(x, 30, 30)
#define PD_PARSE_USB_HOST(x)        GETBITS(x, 31, 31)
#define PD_PARSE_PROD_ID(x)         GETBITS(x, 16, 31)
#define PD_PARSE_PROD_BCD(x)        GETBITS(x, 0, 15)

#define PD_PARSE_USBSS_SIG(x)       GETBITS(x, 0, 2)
#define PD_PARSE_USB_VBUS(x)        GETBITS(x, 3, 3)
#define PD_PARSE_USB_VCONN(x)       GETBITS(x, 4, 4)
#define PD_PARSE_VCONN_WATT(x)      GETBITS(x, 5, 7)
#define PD_PARSE_USBSS_DIR(x)       GETBITS(x, 8, 11)
#define PD_PARSE_FW_VER(x)          GETBITS(x, 24, 27)
#define PD_PARSE_HW_VER(x)          GETBITS(x, 28, 31)

/* DisplayPort Status
    BIT[31:9] - Reserved

    BIT8 - IRQ_HPD, 0 for DFP_D
    BIT7 - HPD State, 0 for DFP_D

    BIT6 - Exit DisplayPort Mode request, 0 for DFP_U
    BIT5 - USB Configuration request, 0 for DFP_U
    BIT4 - Multi-Function preferred, 0 for DFP_U
    BIT3 - Enabled, 0 for DFP_U
    BIT2 - Power Low, 0 for DFP_U

    BIT[1:0] DFP_D/UFP_D Connected
        00 = neither DFP_D nor UFP_D connected or adapter is disabled
        01 = DFP_D Connected
        10 = UFP_D Connected
        11 = Both DFP_D and UFP_D Connected
*/

#define PD_PARSE_DP_STA(x)          GETBITS(x, 0, 1)
#define PD_PARSE_DP_STA_UFP_D(x)    GETBITS(x, 1, 1)
#define PD_PARSE_DP_STA_DFP_D(x)    GETBITS(x, 0, 0)
#define PD_PARSE_DP_LP(x)           GETBITS(x, 2, 2)
#define PD_PARSE_DP_ENABLED(x)      GETBITS(x, 3, 3)
#define PD_PARSE_DP_MUL(x)          GETBITS(x, 4, 4)
#define PD_PARSE_DP_USB(x)          GETBITS(x, 5, 5)
#define PD_PARSE_DP_EXIT(x)         GETBITS(x, 6, 6)
#define PD_PARSE_DP_HPD(x)          GETBITS(x, 7, 7)
#define PD_PARSE_DP_IRQ(x)          GETBITS(x, 8, 8)

void PD_FSM_Reset(pd_state_t* PD_Ctl) {
    TIM1_CNT = 0;
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    TIM_SetCounter(TIM1, 0);
    
    PD_Ctl->PD_Role = ROLE_DFP;
    PD_Ctl->PR_Role = ROLE_SRC;

    PD_Ctl->Msg_ID = 0; 
    PD_Ctl->Det_Cnt = 0;
    PD_Ctl->Cap_Cnt = 0;

    PD_Ctl->Connect = DISCONNECTED;

    PD_Ctl->Status = STA_DISCONNECT;
    PD_Ctl->VDM_Status = STA_VDM_IDLE;

    PD_Ctl->IRQ_func = NULL;
    PD_Ctl->IRQ_ret = 0;

    PD_Ctl->DP_Status.Pos = 0;
    PD_Ctl->DP_Status.Link = 0;
    PD_Ctl->DP_Status.Enabled = DISABLE;
    PD_Ctl->DP_Status.LT = 0;
    return;
}

void PD_PHY_Reset(pd_state_t* PD_Ctl) {
    GPIO_Toggle_VCONN(RESET);
    GPIO_Toggle_VBUS(RESET);
    PD_FSM_Reset(PD_Ctl);
    return;
}

void PD_INIT(pd_state_t* PD_Ctl) {
    printf("Enable USBPD\r\n");

    GPIO_PD_INIT();
    EXTI_INIT();
    TIM1_INIT();

    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;
    USBPD->PORT_CC1 = CC_CMP_45 | CC_PU_180; //180 for 1A5
    USBPD->PORT_CC2 = CC_CMP_45 | CC_PU_180; //180 for 1A5

    PD_Ctl->Rx_Buf = PD_Rx_Buf;
    PD_Ctl->Tx_Buf = PD_Tx_Buf;
    USBPD->CONFIG = PD_DMA_EN;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;

    USBPD_FSM = PD_Ctl;
    PD_FSM_Reset(PD_Ctl);

    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->DMA = (uint32_t)PD_Ctl->Rx_Buf;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_24M;
    USBPD->CONTROL |= BMC_START;

    return;
}

void PD_Load_Header(uint8_t type, pd_state_t* PD_Ctl) {
    /* Message Header
       BIT15 - Extended;
       BIT[14:12] - Number of Data Objects
       BIT[11:9] - Message ID
       BIT8 - Port Power Role/Cable Plug, 0: SINK; 1: SOURCE;
       BIT[7:6] - Revision, 00: V1.0; 01: V2.0; 10: V3.0;
       BIT5 - Port Data Role, 0: UFP; 1: DFP;
       BIT[4:0] - Message Type
    */
    uint16_t data = type & 0x1f;
    data |= 0x40; //PD 2.0
    if(PD_Ctl->PD_Role) {
        data |= 0x20;
    }
    if(PD_Ctl->PR_Role) {
        data |= 0x100;
    }
    data |= PD_Ctl->Msg_ID << 9;
    ((uint16_t*)PD_Ctl->Tx_Buf)[0] = data;
    return;
}

void PD_Load_Header_ID(uint8_t type, uint8_t id, pd_state_t* PD_Ctl) {
    uint16_t data = type & 0x1f;
    data |= 0x40; //PD 2.0
    if(PD_Ctl->PD_Role) {
        data |= 0x20;
    }
    if(PD_Ctl->PR_Role) {
        data |= 0x100;
    }
    data |= id << 9;
    ((uint16_t*)PD_Ctl->Tx_Buf)[0] = data;
    return;
}

void PD_Load_DataObj(void* data, size_t len, pd_state_t* PD_Ctl) {
    if((len & 3) != 0) {
        char str[64];
        __builtin_sprintf(str, "Misalign of PDO with length %d", len);
        assert(str);
    }
    ((uint8_t*)(PD_Ctl->Tx_Buf))[1] |= len << 2; //Load data size into header
    __builtin_memcpy(PD_Ctl->Tx_Buf + 2, data, len);
    return;
}

void PD_Load_VDM(uint16_t svid, int cmd, void* buf, struct PD_FSM* PD_Ctl) {
    //see description above.
    uint32_t data = cmd & 0x1F;
    data |= DEF_CMDTYPE_INIT << 6;
    data |= 0x8000; //Structured VDM
    data |= svid << 16;
    ((uint32_t*)buf)[0] = data;
    return;
}

void PD_Load_VDM_POS(uint8_t pos, void* buf, struct PD_FSM* PD_Ctl) {
    if(!pos) assert("Empty VDO Pos");
    ((uint32_t*)buf)[0] |= pos << 8;
    return;
}

void PD_Load_VDM_CMDTYPE(uint8_t cmdtype, void* buf, struct PD_FSM* PD_Ctl) {
    ((uint32_t*)buf)[0] |= cmdtype << 6;
    return;
}

void PD_Load_VDM_DP_S(void* buf, struct PD_FSM* PD_Ctl) {
    //see description above.
    uint32_t data = 0x1;
    ((uint32_t*)buf)[1] = data;
    return;
}

void PD_Load_VDM_DP_CFG(void* buf, struct PD_FSM* PD_Ctl) {
    /* DisplayPort Capability
        BIT[31:16] - Reserved
        BIT[15:8] - Configure UFP_U Pin Assignment
            00000000 = De-select pin assignment.
            00000001 = Select Pin Assignment A
            00000010 = Select Pin Assignment B
            00000100 = Select Pin Assignment C
            00001000 = Select Pin Assignment D
            00010000 = Select Pin Assignment E
            00100000 = Select Pin Assignment F
            Pin assignments A, B, and F have been deprecated.
        BIT[7:6] - Reserved
        BIT[5:2] - Signaling for Transport of DisplayPort Protocol
            0: USB; 1: DP1.3;
        BIT[1:0] - Select Configuration
            00 = Set configuration for USB
            01 = Set configuration for UFP_U as DFP_D
            10 = Set configuration for UFP_U as UFP_D
            11 = Reserved
    */
    uint32_t data = 0x2 | (0x1 << 2); //Set UFP_U as UFP_D and Enable DP1.3 Signaling
    //int pin = PD_Ctl->DP_Status.Receptable ? PD_Ctl->DP_Status.UFP_Pin : PD_Ctl->DP_Status.DFP_Pin;
    //TODO
    GPIO_Toggle_DPSEL(0);
    data |= 0x4 << 8;
    ((uint32_t*)buf)[1] = data;
    return;
}

void PD_Send(void* buf, size_t len, int sop) {
    USBPD->PORT_CC1 |= CC_LVE;
    USBPD->PORT_CC2 |= CC_LVE;

    USBPD->TX_SEL = sop;
    USBPD->BMC_TX_SZ = len;
    uint32_t rx = USBPD->DMA;
    USBPD->DMA = (uint32_t)buf;

    USBPD->STATUS &= BMC_AUX_INVALID;
    USBPD->BMC_CLK_CNT = UPD_TMR_TX_24M;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->CONTROL |= BMC_START;

    while((USBPD->STATUS & IF_TX_END) == 0);
    USBPD->STATUS |= IF_TX_END;

    USBPD->PORT_CC1 &= ~CC_LVE;
    USBPD->PORT_CC2 &= ~CC_LVE;

    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->DMA = rx;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_24M;
    USBPD->CONTROL |= BMC_START;
    return;
}

void PD_Send_Rst() {
    PD_Send(NULL, 0, UPD_HARD_RESET);
    return;
}

size_t PD_Recv() {
    for(int i = 0; i < 250; i++) {
        if((USBPD->STATUS & IF_RX_ACT) == IF_RX_ACT) {
            size_t len = USBPD->BMC_BYTE_CNT;
            USBPD->STATUS |= IF_RX_ACT;
            return len;
        }
        Delay_Us(4);
    } //1ms tReceive for GoodCRC
    return 0;
}

int PD_Send_Retry(uint8_t type, void* data, size_t len, pd_state_t* PD_Ctl) {
    //Load Data Structure
    PD_Load_Header(type, PD_Ctl);
    PD_Load_DataObj(data, len, PD_Ctl);
    //Retry for 3 times
    for(int retry = 0; retry < 3; retry++) {
        PD_Send(PD_Ctl->Tx_Buf, len + 2, UPD_SOP0);
        int size = PD_Recv();
        if(size == 0) continue;
        if((size == 6) && 
            (PD_PARSE_TYPE(PD_Ctl->Rx_Buf) == DEF_TYPE_GOODCRC) && 
            (PD_PARSE_MSGID(PD_Ctl->Rx_Buf) == PD_Ctl->Msg_ID)) {
            PD_Ctl->Msg_ID = ++(PD_Ctl->Msg_ID) & 0x07;
            return 0;
        }
        Delay_Us(20);
    }
    return -1;
}

int PD_Send_Retry_Rst(uint8_t type, void* data, size_t len, pd_state_t* PD_Ctl) {
    if(PD_Send_Retry(type, data, len, PD_Ctl)) {
        if(PD_Send_Retry(DEF_TYPE_SOFT_RESET, NULL, 0, PD_Ctl)) {
            PD_Send_Rst();
            PD_PHY_Reset(PD_Ctl);
            return -1;
        }
        PD_FSM_Reset(PD_Ctl);
        return -1;
    }
    return 0;
}

int PD_Recv_Retry(uint8_t type, int timeout, pd_state_t* PD_Ctl) {
    for(int i = 0; i < timeout; i++) {
        int size = PD_Recv(); //1ms
        if(size && (PD_PARSE_TYPE(PD_Ctl->Rx_Buf) == type)) {
            Delay_Us(20);
            PD_Load_Header_ID(DEF_TYPE_GOODCRC, PD_PARSE_MSGID(PD_Ctl->Rx_Buf), PD_Ctl);
            PD_Send(PD_Ctl->Tx_Buf, 2, UPD_SOP0);
            return 0;
        }
    }
    return -1;
}

void PD_Enter_ListenMode(pd_state_t* PD_Ctl) {
    PD_Ctl->IRQ_ret = 0;
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET;
    NVIC_EnableIRQ(USBPD_IRQn);
    return;
}

void PD_Exit_ListenMode(pd_state_t* PD_Ctl) {
    USBPD->CONFIG &= ~(IE_RX_ACT | IE_RX_RESET);
    NVIC_DisableIRQ(USBPD_IRQn);
    PD_Ctl->IRQ_ret = 0;
    return;
}

void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBPD_IRQHandler() {
    //RESET Process
    if(USBPD_FSM == NULL) {
        assert("USBPD IRQ Handler NULL Pointer Exception");
        return;
    }
    if(USBPD->STATUS & IF_RX_RESET) {
        USBPD->STATUS |= IF_RX_RESET;
        PD_PHY_Reset(USBPD_FSM);
        USBPD_FSM->IRQ_ret = 1;
        printf("A unexpected Hard Reset was occured from UFP\r\n");
        return;
    }
    if(USBPD->STATUS & IF_RX_ACT) {
        size_t len = USBPD->BMC_BYTE_CNT;
        USBPD->STATUS |= IF_RX_ACT;
        PD_Load_Header(DEF_TYPE_GOODCRC, USBPD_FSM);
        PD_Send(USBPD_FSM->Tx_Buf, 2, UPD_SOP0);
        if(USBPD_FSM->IRQ_func) {
            int (*Handler)(size_t, pd_state_t*) = USBPD_FSM->IRQ_func;
            NVIC_DisableIRQ(USBPD_IRQn);
            USBPD_FSM->IRQ_ret = Handler(len, USBPD_FSM);
            NVIC_EnableIRQ(USBPD_IRQn);
        }
    }
    return;
}

void PD_Delay_Reset() {
    TIM1_CNT = 0;
    TIM_SetCounter(TIM1, 0);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    return;
}

void PD_Delay_Off() {
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    return;
}

void PD_Connect(int passive, int ccpin, pd_state_t* PD_Ctl) {
    PD_Ctl->Connect = CONNECTED;
    PD_Ctl->Status = STA_SINK_CONNECT;
    if(ccpin == 1) {
        USBPD->CONFIG &= ~CC_SEL;   //CC1
    } else { //attach == 2
        USBPD->CONFIG |= CC_SEL;    //CC2
    }
    printf("PD Connect at CC%d\r\n", ccpin);
    GPIO_Toggle_VBUS(SET);
    PD_Delay_Reset(); //Delay Start from VBUS ON
    if(!passive) {
        GPIO_Toggle_VCONN(ccpin);
    }
    return;
}

void PD_Disconnect(pd_state_t* PD_Ctl) {
    PD_Ctl->Connect = DISCONNECTED;
    PD_Ctl->Status = STA_DISCONNECT;
    PD_Delay_Off();
    PD_PHY_Reset(PD_Ctl);
    printf("PD Disconnect\r\n");
    return;
}

int PD_TestCC() {
    /* return:
        000: Nothing Attached
        001: Passive at CC1
        010: Passive at CC2
        011: Debug Accessory Mode
        100: Audio Adapter Accessory Mode / with Powered cable plugged but without Sink attached
        101: Active at CC1
        110: Active at CC2
        111: **Unreachable**
    */
    int Rd = (GPIOC->INDR & (PIN_CC1 | PIN_CC2)) >> 14;
    int Ra = ((USBPD->PORT_CC2 & PA_CC_AI) << 1) | (USBPD->PORT_CC1 & PA_CC_AI);
    return ((Ra < 3) << 2) | (Ra & (Rd ^ 3));
}

int PD_Detect(pd_state_t* PD_Ctl) {
    int state = PD_TestCC();
    int ccpin = state & 3;
    int passive = state < 4;
    if(PD_Ctl->Connect) {
        if (ccpin) {
            PD_Ctl->Det_Cnt = 0;
            return 0;
        } else {
            PD_Ctl->Det_Cnt++;
            if(PD_Ctl->Det_Cnt == 5) {
                PD_Ctl->Det_Cnt = 0;
                PD_Disconnect(PD_Ctl);
            }
            return -1;
        }
    } else {
        if (!ccpin) {
            PD_Ctl->Det_Cnt = 0;
            return 0;
        } else {
            PD_Ctl->Det_Cnt++;
            if(PD_Ctl->Det_Cnt == 5) {
                PD_Ctl->Det_Cnt = 0;
                PD_Connect(passive, ccpin, PD_Ctl);
                return 0;
            }
            return -1;
        }
    }
}

int PD_Delay(unsigned int ms, pd_state_t* PD_Ctl) {
    PD_Enter_ListenMode(PD_Ctl);
    while(TIM1_CNT < ms) {
        __WFI();
        if(PD_Ctl->IRQ_ret) {
            PD_Exit_ListenMode(PD_Ctl);
            return -1;
        }
    }
    PD_Exit_ListenMode(PD_Ctl);
    PD_Delay_Reset();

    while(PD_Detect(PD_Ctl)) Delay_Ms(1);
    if(PD_Ctl->Status == STA_DISCONNECT) return -1;

    return 0;
}

int PD_SendRecv_VDM(int cmd, int timeout, void* data, size_t len, pd_state_t* PD_Ctl) {
    if(PD_Send_Retry_Rst(DEF_TYPE_VENDOR_DEFINED, data, len, PD_Ctl)) return -1;
    PD_Delay_Reset();
    for(int i = 0; i < 5; i++) {
        do {
            if(PD_Recv_Retry(DEF_TYPE_VENDOR_DEFINED, timeout, PD_Ctl)) return -1;
        } while(PD_PARSE_CMD(PD_Ctl->Rx_Buf) != cmd);
        if(PD_PARSE_CMDTYPE(PD_Ctl->Rx_Buf) == DEF_CMDTYPE_ACK) return 0;
        if(PD_PARSE_CMDTYPE(PD_Ctl->Rx_Buf) == DEF_CMDTYPE_NAK) return -1;
        if(PD_Delay(50, PD_Ctl)) return -1;
    }
    //run out of nBusyCount
    return -1;
}

int PD_Test_IDENT(pd_state_t* PD_Ctl) {
    pd_vdo_ident_t vdo;
    
    int len = PD_PARSE_SIZE(PD_Ctl->Rx_Buf) - 1;
    __builtin_memcpy(&vdo, PD_Ctl->Rx_Buf + 6, 4 * len);
    //WCH can make SOP into buffer with header to form a 4-byte alignment which is well for struct operation.

    printf("VDM Info:\r\n");
    printf("VID: 0x%04x\r\n", PD_PARSE_USB_VID(vdo.id_header));
    printf("PID: 0x%04x\r\n", PD_PARSE_PROD_ID(vdo.product));
    printf("bcd: 0x%04x\r\n", PD_PARSE_PROD_BCD(vdo.product));
    printf("prod_type: %d\r\n", PD_PARSE_PROD_TYPE(vdo.id_header));
    printf("is_ama: %d\r\n", PD_PARSE_PROD_TYPE(vdo.id_header) == 0x05);
    printf("usb_host: %d\r\n", PD_PARSE_USB_HOST(vdo.id_header));
    printf("usb_device: %d\r\n", PD_PARSE_USB_DEVICE(vdo.id_header));
    printf("modal: %d\r\n", PD_PARSE_MODAL(vdo.id_header));
    printf("xid: 0x%08x\r\n", vdo.cert_state);

    if(!PD_PARSE_MODAL(vdo.id_header)) return -1;

    if(PD_PARSE_PROD_TYPE(vdo.id_header) != 0x05) return 0; //Alternate Mode Adapter
    if(len < 4) return 0;
    printf("AMA Info:\r\n");
    printf("hw_ver: 0x%04x\r\n", PD_PARSE_HW_VER(vdo.product_type));
    printf("fw_ver: 0x%04x\r\n", PD_PARSE_FW_VER(vdo.product_type));
    printf("vbus_req: %d\r\n", PD_PARSE_USB_VBUS(vdo.product_type));
    printf("vconn_req: %d\r\n", PD_PARSE_USB_VCONN(vdo.product_type));
    printf("vconn_watt: %d\r\n", PD_PARSE_VCONN_WATT(vdo.product_type));
    printf("usbss_sig: %d\r\n", PD_PARSE_USBSS_SIG(vdo.product_type));
    printf("usbss_dir: 0x%02x\r\n", PD_PARSE_USBSS_DIR(vdo.product_type));

    if(!PD_PARSE_USB_VCONN(vdo.product_type)) GPIO_Toggle_VCONN(RESET);
    return 0;
}

int PD_Test_SVID(pd_state_t* PD_Ctl) {
    uint16_t* svid = PD_Ctl->Rx_Buf + 2;
    for(int i = 0; i < 12; i++) {
        int pos = i ^ 1; //weirdo big endian whin whole PD's little endian stuffs...
        if(svid[pos] == DEF_SVID_EOF) return -1;
        if(svid[pos] == DEF_SVID_DISPLAYPORT) return 0;
    }
    return 1;
}

int PD_Test_MODE(pd_state_t* PD_Ctl) {
    uint32_t mode[6];

    int len = PD_PARSE_SIZE(PD_Ctl->Rx_Buf) - 1;
    __builtin_memcpy(mode, PD_Ctl->Rx_Buf + 6, 4*len);

    for(int i = 0; i < len; i++) {
        printf("MODE %d:\r\n", i + 1);
        /* PD_PARSE_DP_CAP
            00 = Reserved
            01 = UFP_D capable (including Branch Device)
            10 = DFP_D capable (including Branch Device)
            11 = Both DFP_D and UFP_D capable
        */
        printf("capability: %d\r\n", PD_PARSE_DP_CAP(mode[i]));
        printf("receptable: %d\r\n", PD_PARSE_RECEPTACLE(mode[i]));
        printf("supported_pin: 0x%02x\r\n", PD_PARSE_DFP_PIN(mode[i]));
    }
    
    for(int i = 0; i < len; i++) {
        if(!PD_PARSE_DP_SIG(mode[i])) continue;
        if(!PD_PARSE_DP_CAP_UFP_D(mode[i])) continue;
        printf("DP MODE %d is used\r\n", i + 1);
        PD_Ctl->DP_Status.Pos = i + 1;
        PD_Ctl->DP_Status.DFP_Pin = PD_PARSE_DFP_PIN(mode[i]);
        PD_Ctl->DP_Status.UFP_Pin = PD_PARSE_UFP_PIN(mode[i]);
        PD_Ctl->DP_Status.Receptable = PD_PARSE_RECEPTACLE(mode[i]);
        return 0;
    }
    return -1;
}

int PD_Test_DP_S(pd_state_t* PD_Ctl) {
    uint16_t* data = PD_Ctl->Rx_Buf + 6;
    uint32_t status = (data[1] << 16) | data[0];
    PD_Ctl->DP_Status.Link = status;
    //printf("DP Link Info:\r\n");
    /* PD_PARSE_DP_STA, inconsistency with previous PD_PARSE_DP_CAP one...
        00 = neither DFP_D nor UFP_D connected or adapter is disabled
        01 = DFP_D Connected
        10 = UFP_D Connected
        11 = Both DFP_D and UFP_D Connected
    */
    /*
    printf("enabled: %d\r\n", PD_PARSE_DP_ENABLED(status));
    printf("connected: %d\r\n", PD_PARSE_DP_STA(status));
    printf("low_power: %d\r\n", PD_PARSE_DP_LP(status));
    printf("multi_func: %d\r\n", PD_PARSE_DP_MUL(status));
    printf("hpd: %d\r\n", PD_PARSE_DP_HPD(status));
    printf("irq: %d\r\n", PD_PARSE_DP_IRQ(status));
    printf("usb_req: %d\r\n", PD_PARSE_DP_USB(status));
    printf("exit_req: %d\r\n", PD_PARSE_DP_EXIT(status));
    */
    if(!PD_PARSE_DP_ENABLED(status)) return -1;
    if(!PD_PARSE_DP_STA_UFP_D(status)) return -1;
    return 0;
}

int PD_Test_DP_HPD(pd_state_t* PD_Ctl) {
    uint16_t* data = PD_Ctl->Rx_Buf + 6;
    uint32_t status = (data[1] << 16) | data[0];
    return PD_PARSE_DP_HPD(status);
}

int PD_Listen_Ready(size_t len, pd_state_t* PD_Ctl) {
    int buf[6];
    switch(PD_PARSE_TYPE(PD_Ctl->Rx_Buf)) {
    case DEF_TYPE_VENDOR_DEFINED:
        PD_Load_VDM(PD_PARSE_SVID(PD_Ctl->Rx_Buf), PD_PARSE_CMD(PD_Ctl->Rx_Buf), buf, PD_Ctl);
        if(PD_PARSE_CMD(PD_Ctl->Rx_Buf) == DEF_VDM_ATTENTION) {
            PD_Load_VDM_CMDTYPE(DEF_CMDTYPE_ACK, buf, PD_Ctl);
            if(PD_Send_Retry_Rst(DEF_TYPE_VENDOR_DEFINED, buf, 4, PD_Ctl)) return -1;
            int new_enabled = PD_Test_DP_S(PD_Ctl) == 0;
            //transition
            if(PD_Ctl->DP_Status.Enabled != new_enabled) {
                if(new_enabled && !PD_Ctl->DP_Status.LT) PD_Ctl->VDM_Status = STA_DP_CONFIG;
                PD_Ctl->DP_Status.Enabled = new_enabled;
                return -1;
            }
            //hot-plug detect
            if(PD_PARSE_DP_HPD(PD_Ctl->DP_Status.Link)) PD_Ctl->VDM_Status = STA_MPD_CONFIG;
        } else {
            PD_Load_VDM_CMDTYPE(DEF_CMDTYPE_NAK, buf, PD_Ctl);
            if(PD_Send_Retry_Rst(DEF_TYPE_VENDOR_DEFINED, buf, 4, PD_Ctl)) return -1;
        }
        break;
    case DEF_TYPE_SOFT_RESET:
        PD_FSM_Reset(PD_Ctl);
        return -1;
    //TODO
    default:
        if(PD_Send_Retry_Rst(DEF_TYPE_REJECT, NULL, 0, PD_Ctl)) return -1;
        break;
    }
    return 0;
}

void PD_VDM_Proc(pd_state_t* PD_Ctl) {
    int buf[6];
    switch(PD_Ctl->VDM_Status){
    case STA_DISC_IDENT:
        PD_Load_VDM(DEF_SVID_DEFAULT, DEF_VDM_DISC_IDENT, buf, PD_Ctl);
        if(PD_SendRecv_VDM(DEF_VDM_DISC_IDENT, 30, buf, 4, PD_Ctl)) {
            printf("Device doesn't support VDM\r\n");
            PD_Ctl->VDM_Status = STA_VDM_IDLE;
            break;
        }
        if(PD_Test_IDENT(PD_Ctl)) {
            printf("Device doesn't support Modal Operation\r\n");
            PD_Ctl->VDM_Status = STA_VDM_IDLE;
            break;
        }
        PD_Ctl->VDM_Status = STA_DISC_SVID;
        break;
    case STA_DISC_SVID: //Get Supported SVID
        PD_Load_VDM(DEF_SVID_DEFAULT, DEF_VDM_DISC_SVID, buf, PD_Ctl);
        if(PD_SendRecv_VDM(DEF_VDM_DISC_SVID, 30, buf, 4, PD_Ctl)) {
            printf("Device doesn't contain any SVID\r\n");
            PD_Ctl->VDM_Status = STA_VDM_IDLE;
            break;
        }
        int result = PD_Test_SVID(PD_Ctl);
        if(result > 0) break; //Query More SVID
        if(result < 0) {
            printf("Device doesn't support DisplayPort\r\n");
            PD_Ctl->VDM_Status = STA_VDM_IDLE;
            break;
        }
        PD_Ctl->VDM_Status = STA_DISC_MODE;
        break;
    case STA_DISC_MODE: //Get DisplayPort Mode Capability
        PD_Load_VDM(DEF_SVID_DISPLAYPORT, DEF_VDM_DISC_MODE, buf, PD_Ctl);
        if(PD_SendRecv_VDM(DEF_VDM_DISC_MODE, 30, buf, 4, PD_Ctl) || PD_Test_MODE(PD_Ctl)) {
            printf("Device not capable of UFP_D\r\n");
            PD_Ctl->VDM_Status = STA_VDM_IDLE;
            break;
        }
        PD_Ctl->VDM_Status = STA_ENTER_MODE;
        break;
    case STA_ENTER_MODE:
        PD_Load_VDM(DEF_SVID_DISPLAYPORT, DEF_VDM_ENTER_MODE, buf, PD_Ctl);
        PD_Load_VDM_POS(PD_Ctl->DP_Status.Pos, buf, PD_Ctl);
        if(PD_SendRecv_VDM(DEF_VDM_ENTER_MODE, 50, buf, 4, PD_Ctl)) { //tVDMWaitModeEntry
            printf("Device refused to enter DP Alt-mode due to unknown reason\r\n");
            PD_Ctl->VDM_Status = STA_VDM_IDLE;
            break;
        }
        printf("DP Alt-mode established\r\n");
        PD_Ctl->VDM_Status = STA_DP_S_UPDATE;
        break;
    case STA_DP_S_UPDATE:
        PD_Load_VDM(DEF_SVID_DISPLAYPORT, DEF_VDM_DP_S_UPDATE, buf, PD_Ctl);
        PD_Load_VDM_DP_S(buf, PD_Ctl);
        if(PD_SendRecv_VDM(DEF_VDM_DP_S_UPDATE, 30, buf, 8, PD_Ctl)) break;
        if(PD_Test_DP_S(PD_Ctl)) {
            printf("DisplayPort not ready, waiting\r\n");
            PD_Ctl->VDM_Status = STA_VDM_IDLE;
        }
        PD_Ctl->VDM_Status = STA_DP_CONFIG;
        break;
    case STA_DP_CONFIG:
        PD_Ctl->DP_Status.Enabled = ENABLE;
        PD_Delay_Reset();
        printf("Perform DP Aux HPD cycle\r\n");
        GPIO_Toggle_DPSIG(RESET);
        if(PD_Delay(5, PD_Ctl)) return;
        PD_Load_VDM(DEF_SVID_DISPLAYPORT, DEF_VDM_DP_CONFIG, buf, PD_Ctl);
        PD_Load_VDM_DP_CFG(buf, PD_Ctl);
        if(PD_SendRecv_VDM(DEF_VDM_DP_CONFIG, 30, buf, 8, PD_Ctl)) break;
        GPIO_Toggle_DPSIG(SET); //Configure DP Signal and SBU Mux
#ifdef IGNORE_HPD
        PD_Ctl->VDM_Status = STA_MPD_CONFIG;
#else
        PD_Ctl->VDM_Status = PD_PARSE_DP_HPD(PD_Ctl->DP_Status.Link) ? STA_MPD_CONFIG : STA_VDM_IDLE;
#endif
        break;
    case STA_MPD_CONFIG:
#ifndef IGNORE_HPD
        if(PD_Ctl->DP_Status.LT) {
            PD_Ctl->VDM_Status = STA_VDM_IDLE;
            break;
        }
#endif
        PD_Enter_ListenMode(PD_Ctl);
#ifndef NO_MPD
        if(!MPD_CfgLink()) {
            MPD_CfgVideo();
            MPD_CfgTest();
            WS2812_SetColor(BLUE);
            PD_Ctl->DP_Status.LT = 1;
        } else {
            WS2812_SetColor(YELLOW);
            PD_Ctl->DP_Status.LT = 0;
        }
#endif
        PD_Exit_ListenMode(PD_Ctl);
        PD_Ctl->VDM_Status = STA_VDM_IDLE;
        break;
    case STA_VDM_IDLE:
        if(PD_Delay(200, PD_Ctl)) return;
        break;
    default:
        assert("Unexpected PD VDM FSM");
    }
    return;
}

int PD_Test_Cap(pd_state_t* PD_Ctl) {
    int pos = GETBITS(((uint8_t*)PD_Ctl->Rx_Buf)[5], 4, 6);
    if(pos != 1) return -1; //Invalid
    //int current = ((PD_Ctl->PD_Rx_Buf[4] & 0x0f) << 6) | (PD_Ctl->PD_Rx_Buf[3] >> 2); //BIT[19:10]
    //int mismatch = (PD_Ctl->PD_Rx_Buf[5] & 0x04) >> 2; //BIT26
    return 0;
}

int PD_Listen_Default(size_t len, pd_state_t* PD_Ctl) {
    //unexpected Messages received regarded as Protocol Errors
    if(PD_PARSE_TYPE(PD_Ctl->Rx_Buf) == DEF_TYPE_SOFT_RESET) {
        PD_FSM_Reset(PD_Ctl);
    } else {
        PD_Send_Rst();
        PD_PHY_Reset(PD_Ctl);
    }
    return -1;
}

void PD_Proc(pd_state_t* PD_Ctl) {
    switch(PD_Ctl->Status){
    case STA_DISCONNECT:
        WS2812_SetColor(RED);
        SYS_SLP();
        break;
    case STA_SINK_CONNECT:
        PD_Ctl->IRQ_func = PD_Listen_Default;
        if(PD_Delay(200, PD_Ctl)) return; //tFirstSourceCap
        PD_Ctl->Status = STA_TX_SRC_CAP;
        break;
    case STA_TX_SRC_CAP:
        while(PD_Send_Retry(DEF_TYPE_SRC_CAP, (void*)&SRCCAP_5V1A5, 4, PD_Ctl)) {
            PD_Ctl->Cap_Cnt++;
            if(PD_Ctl->Cap_Cnt == 16) {
                PD_Ctl->Status = STA_DISCONNECT;
                return;
            } //nCapsCount
            if(PD_Delay(150, PD_Ctl)) return; //tTypeCSendSourceCap
        }
        PD_Ctl->Status = STA_RX_REQ;
        break;
    case STA_RX_REQ:
        if(PD_Recv_Retry(DEF_TYPE_REQUEST, 30, PD_Ctl)) { //tSenderResponse
            PD_Ctl->Status = STA_PHY_RST;
            break;
        }
        if(PD_Test_Cap(PD_Ctl)) {
            PD_Ctl->Status = STA_PHY_RST;
        } else {
            PD_Ctl->Status = STA_TX_ACCEPT;
        }
        break;
    case STA_TX_ACCEPT:
        if(PD_Send_Retry(DEF_TYPE_ACCEPT, NULL, 0, PD_Ctl)) { //tReceiverResponse 15ms
            PD_Ctl->Status = STA_TX_SOFTRST;
        } else {
            PD_Ctl->Status = STA_TX_PS_RDY;
        }
        break;
    case STA_TX_PS_RDY:
        printf("vbus_mvolts: %04d\r\n", VSNS_Read());
        if(PD_Send_Retry(DEF_TYPE_PS_RDY, NULL, 0, PD_Ctl)) { //tPSTransition 500ms
            PD_Ctl->Status = STA_TX_SOFTRST;
        } else {
            WS2812_SetColor(GREEN);
            PD_Delay_Reset();
            PD_Ctl->IRQ_func = PD_Listen_Ready;
            PD_Ctl->Status = STA_IDLE;
            PD_Ctl->VDM_Status = STA_DISC_IDENT;
            printf("PD negotiation finished\r\n");
        }
        break;
    case STA_IDLE: //Now in PE_SRC_READY
        PD_VDM_Proc(PD_Ctl);
        break;
    case STA_TX_SOFTRST:
        if(!PD_Send_Retry(DEF_TYPE_SOFT_RESET, NULL, 0, PD_Ctl)) {
            PD_FSM_Reset(PD_Ctl);
            break;
        }
    case STA_PHY_RST:
        PD_Send_Rst();
        PD_PHY_Reset(PD_Ctl);
        break;
    default:
        assert("Unexpected PD FSM");
        break;
    }
    return;
}

int main() {
    SYS_INIT();
    GPIO_Toggle_INIT();
    IIC_Init();
#ifndef NO_MPD
    MPD_Init();
#endif
    VSNS_INIT();
    pd_state_t PD_Ctl;
    PD_INIT(&PD_Ctl);
    while(1) {
        if(!PD_Detect(&PD_Ctl)) PD_Proc(&PD_Ctl);
        Delay_Us(20);
    }
    return 0;
}