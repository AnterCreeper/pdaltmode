#include "timer.h"
#include "gpio.h"
#include "iic_dev.h"
#include "mpd.h"
#include "debug.h"

#define GETBITS(x, n, m) ((x >> n) & ((1 << (m - n + 1)) - 1))

//Chip always write data in 4bytes
#define IIC_WriteReg(address,data,len) __IIC_WriteReg(MPD_DEV_ADR, address, (uint32_t)(data), len, 0, 0)
#define IIC_WriteRegI(address,data) __IIC_WriteReg(MPD_DEV_ADR, address, data, 4, 1, 0)
#define IIC_ReadReg(address,data,len) __IIC_ReadReg(MPD_DEV_ADR, address, (uint32_t)(data), len, 0, 0)
#define IIC_ReadRegI(address,len) __IIC_ReadReg(MPD_DEV_ADR, address, 0, len, 1, 0)

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler() {
    if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
        uint32_t sys, irq;
        IIC_ReadReg(INTSTS_G, &irq, 4);
        IIC_ReadReg(SYSSTAT, &sys, 4);
        if(irq & INT_SYSERR) {
            printf("IRQ from MPD\r\n");
            printf("err: 0x%08x\r\n", sys);
        }
        IIC_WriteRegI(INTSTS_G, irq);
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
    GPIO_OUT_INIT(GPIOB, GPIO_Pin_1, Bit_SET);    //MCU_DP_RST#
    TIM_Delay_Us(1000);   //Tcorerdy
    GPIO_IPU_INIT(GPIOB, GPIO_Pin_0);    //MCU_DP_INT

    IIC_WriteRegI(SYSRSTENB, ENBI2C);
    IIC_WriteRegI(SYSRSTENB, ENBLCD0 | ENBBM | ENBDSIRX | ENBREG | ENBHDCP | ENBI2C);
    TIM_Delay_Ms(10);
    printf("Reset MPD\r\n");

    uint32_t id = 0, pin = 0;
    IIC_ReadReg(TC_IDREG, &id, 2);
    IIC_ReadReg(SYSSTAT, &pin, 1);
    printf("mpd_id: 0x%04x\r\n", id);
    printf("mpd_pin: 0x%02x\r\n", pin);

#ifdef MPD_2LANE
    IIC_WriteRegI(DP0_SRCCTRL, DP_SRCCTRL_SWG0(MPD_SWG) | DP_SRCCTRL_PRE0(MPD_PRE) |
                                DP_SRCCTRL_SWG1(MPD_SWG) | DP_SRCCTRL_PRE1(MPD_PRE) |
                                DP_SRCCTRL_SCRMBLDIS | DP_SRCCTRL_EN810B |
                                DP_SRCCTRL_NOTP | DP_SRCCTRL_LANESKEW |
                                DP_SRCCTRL_LANES_2 | MPD_BW | DP_SRCCTRL_AUTOCORRECT);
#else
    IIC_WriteRegMPDI(DP0_SRCCTRL, DP_SRCCTRL_SWG0(MPD_SWG) | DP_SRCCTRL_PRE0(MPD_PRE) |
                                DP_SRCCTRL_SCRMBLDIS | DP_SRCCTRL_EN810B |
                                DP_SRCCTRL_NOTP | DP_SRCCTRL_LANESKEW |
                                MPD_BW | DP_SRCCTRL_AUTOCORRECT);
#endif

    IIC_WriteRegI(DP1_SRCCTRL, DP_SRCCTRL_SWG0(MPD_SWG) | DP_SRCCTRL_PRE0(MPD_PRE) |
                                DP_SRCCTRL_SCRMBLDIS | DP_SRCCTRL_EN810B |
                                DP_SRCCTRL_NOTP | MPD_BW);
    IIC_WriteRegI(SYS_PLLPARAM, MPD_REFCLK | LSCLK_DIV_1);

    IIC_WriteRegI(DP_PHY_CTRL, DP_PHY_CTRL_EN);

    printf("Enable MPD PLL\r\n");
    IIC_WriteRegI(DP0_PLLCTRL, PLLUPDATE | PLLEN);   //Enable DP0 PLL
    TIM_Delay_Ms(10);
    IIC_WriteRegI(DP1_PLLCTRL, PLLUPDATE | PLLEN);   //Enable DP1 PLL
    TIM_Delay_Ms(10);

    printf("Reset MPD Func\r\n");
    IIC_WriteRegI(DP_PHY_CTRL, DP_PHY_CTRL_EN | DP_PHY_RST | PHY_M0_RST | PHY_M1_RST);
    TIM_Delay_Us(200);
    IIC_WriteRegI(DP_PHY_CTRL, DP_PHY_CTRL_EN);

    uint32_t result;
    do {
        TIM_Delay_Us(500);
        IIC_ReadReg(DP_PHY_CTRL, &result, 4);
    } while((result & PHY_RDY) == 0); //Query DP PHY Ready

    MPD_IRQInit();
    IIC_WriteRegI(INTCTL_G, INT_SYSERR);   //Enable MPD INT_SYSERR
    printf("MPD Ready\r\n");
    return;
}

int MPD_WriteAUXI(uint32_t bsize, uint32_t address, uint32_t data) {
    int j = 0;
    uint32_t status;
    do {
        if(j++ == 3) return -1;
        IIC_WriteRegI(DP0_AUXADDR, address);
        IIC_WriteRegI(DP0_AUXWDATA, data);
        IIC_WriteRegI(DP0_AUXCFG0, 0x0008 | ((bsize - 1) << 8));
        TIM_Delay_Us(1000);
        IIC_ReadReg(DP0_AUXSTATUS, &status, 4);
    } while(GETBITS(status, 16, 18));
    //printf("mpd_aux_status: %08x\r\n", status);
    return 0;
}

uint32_t MPD_ReadAuxI(uint32_t bsize, uint32_t address) {
    int j = 0;
    uint32_t status, result = 0;
    do {
        if(j++ == 3) return -1;
        IIC_WriteRegI(DP0_AUXADDR, address);
        IIC_WriteRegI(DP0_AUXCFG0, 0x0009 | ((bsize - 1) << 8));
        TIM_Delay_Us(1000);
        IIC_ReadReg(DP0_AUXSTATUS, &status, 4);
    } while(GETBITS(status, 16, 18));
    IIC_ReadReg(DP0_AUXRDATA, &result, bsize);
    //printf("mpd_aux_status: %08x\r\n", status);
    return result;
}

int MPD_ReadAux(uint32_t bsize, uint32_t address, uint32_t* data) {
    int j = 0;
    uint32_t status;
    do {
        if(j++ == 3) return -1;
        IIC_WriteRegI(DP0_AUXADDR, address);
        IIC_WriteRegI(DP0_AUXCFG0, 0x0009 | ((bsize - 1) << 8));
        TIM_Delay_Us(1000);
        IIC_ReadReg(DP0_AUXSTATUS, &status, 4);
    } while(GETBITS(status, 16, 18));
    IIC_ReadReg(DP0_AUXRDATA, data, bsize);
    //printf("mpd_aux_status: %08x\r\n", status);
    return 0;
}

int MPD_CfgLink(){
    printf("Configure DP Link\r\n");
    //IIC_WriteRegI(DP0_AUXCFG1, 0x00010732);  //Set DP AUX Parameter

    uint32_t cap = MPD_ReadAuxI(4, DPCD_REV);
    printf("dp_sink_cap: 0x%08x\r\n", cap);

    IIC_WriteRegI(DP_PHY_CTRL, DP_PHY_CTRL_EN);
    TIM_Delay_Us(100);

    if(IIC_ReadRegI(DP0CTL, 1) & DP_EN) {
        printf("Warning: Unexpected MPD DP Enabled before LT\r\n");
        IIC_WriteRegI(DP0CTL, 0x0); //Disable DP Output
    }
    MPD_WriteAUXI(2, DPCD_LINK_BW_SET, DPCD_ENHANCED_FRAME_EN | DPCD_BW | DPCD_LANES(MPD_LANES));
    MPD_WriteAUXI(1, DPCD_ML_CODING_SET, DPCD_SET_ANSI_8B10B);

    printf("Start Link Training\r\n");
    uint32_t code, status, result;
    /* DP0_LTLOOPCTRL
        BIT[31:28] - DeferIter
        BIT[27:24] - LoopIter
        BIT[23:16] - Reserved
        BIT[15:0] - LoopTimer
    */
    IIC_WriteRegI(DP0_LTLOOPCTRL, 0xFF00002A);

    //Pattern 1
    IIC_WriteRegI(DP0_SNKLTCTRL, DPCD_SCRAMBLING_DISABLE | DPCD_TRAINING_PATTERN_1);
#ifdef MPD_2LANE
    MPD_WriteAUXI(2, DPCD_TRAINING_SET, (MPD_SWG << 8) | MPD_SWG);  //DP PHY Signal
    IIC_WriteRegI(DP0_SRCCTRL, DP_SRCCTRL_SWG0(MPD_SWG) | DP_SRCCTRL_SWG1(MPD_SWG) |
                                DP_SRCCTRL_SCRMBLDIS | DP_SRCCTRL_EN810B |
                                DP_SRCCTRL_TP1 | DP_SRCCTRL_LANESKEW |
                                DP_SRCCTRL_LANES_2 | MPD_BW | DP_SRCCTRL_AUTOCORRECT);

                                     
#else
    MPD_WriteAUXI(1, DPCD_TRAINING_SET, MPD_SWG);  //DP PHY Signal
    IIC_WriteRegI(DP0_SRCCTRL, DP_SRCCTRL_SWG0(MPD_SWG) |
                                DP_SRCCTRL_SCRMBLDIS | DP_SRCCTRL_EN810B |
                                DP_SRCCTRL_TP1 | DP_SRCCTRL_LANESKEW |
                                MPD_BW | DP_SRCCTRL_AUTOCORRECT);
#endif

    //Start Link Training
    IIC_WriteRegI(DP0CTL, EF_EN | DP_EN);   //DP0Ctl, Enable Output
    do {
        TIM_Delay_Us(1000);
        IIC_ReadReg(DP0_LTSTAT, &result, 4);
    } while((result & LT_LOOPDONE) == 0);   //Query DP0_LTStat Round Finish
    code = GETBITS(result, 8, 12);
    status = GETBITS(result, 0, 6);
    printf("Pattern 1:\r\n");
    printf("code: 0x%02x\r\n", code);
    printf("status: 0x%02x\r\n", status);

    //Pattern 2
    IIC_WriteRegI(DP0_SNKLTCTRL, DPCD_SCRAMBLING_DISABLE | DPCD_TRAINING_PATTERN_2);
#ifdef MPD_2LANE
    MPD_WriteAUXI(2, DPCD_TRAINING_SET, (MPD_PRE << 11) | (MPD_SWG << 8) |
                             (MPD_PRE << 3) | MPD_SWG);  //DP PHY Signal
    IIC_WriteRegI(DP0_SRCCTRL, DP_SRCCTRL_SWG0(MPD_SWG) | DP_SRCCTRL_PRE0(MPD_PRE) |
                                DP_SRCCTRL_SWG1(MPD_SWG) | DP_SRCCTRL_PRE1(MPD_PRE) |
                                DP_SRCCTRL_SCRMBLDIS | DP_SRCCTRL_EN810B |
                                DP_SRCCTRL_TP2 | DP_SRCCTRL_LANESKEW |
                                DP_SRCCTRL_LANES_2 | MPD_BW | DP_SRCCTRL_AUTOCORRECT);
#else
    MPD_WriteAUXI(1, DPCD_TRAINING_SET, (MPD_PRE << 3) | MPD_SWG);  //DP PHY Signal
    IIC_WriteRegI(DP0_SRCCTRL, DP_SRCCTRL_SWG0(MPD_SWG) | DP_SRCCTRL_PRE0(MPD_PRE) |
                                DP_SRCCTRL_SCRMBLDIS | DP_SRCCTRL_EN810B |
                                DP_SRCCTRL_TP2 | DP_SRCCTRL_LANESKEW |
                                MPD_BW | DP_SRCCTRL_AUTOCORRECT);
#endif

    //Start Link Training
    do {
        TIM_Delay_Us(1000);
        IIC_ReadReg(DP0_LTSTAT, &result, 4);
    } while((result & LT_LOOPDONE) == 0);
    code = GETBITS(result, 8, 12);
    status = GETBITS(result, 0, 6);
    printf("Pattern 2:\r\n");
    printf("code: 0x%02x\r\n", code);
    printf("status: 0x%02x\r\n", status);
    
    //Clean LT
#ifdef MPD_2LANE
    IIC_WriteRegI(DP0_SRCCTRL, DP_SRCCTRL_SWG0(MPD_SWG) | DP_SRCCTRL_PRE0(MPD_PRE) |
                                DP_SRCCTRL_SWG1(MPD_SWG) | DP_SRCCTRL_PRE1(MPD_PRE) |
                                DP_SRCCTRL_EN810B |
                                DP_SRCCTRL_NOTP | DP_SRCCTRL_LANESKEW |
                                DP_SRCCTRL_LANES_2 | MPD_BW | DP_SRCCTRL_AUTOCORRECT);
#else
    IIC_WriteRegI(DP0_SRCCTRL, DP_SRCCTRL_SWG0(MPD_SWG) | DP_SRCCTRL_PRE0(MPD_PRE) |
                                DP_SRCCTRL_EN810B |
                                DP_SRCCTRL_NOTP | DP_SRCCTRL_LANESKEW |
                                MPD_BW | DP_SRCCTRL_AUTOCORRECT);
#endif
    MPD_WriteAUXI(1, DPCD_TRAINING_PATTERN_SET, 0x00);  //Clean DPCD LT Pattern

    //LT Status
    printf("Sink LT Status:\r\n");
    printf("code: 0x%02x\r\n", MPD_ReadAuxI(1, DPCD_LANE_STATUS));
    printf("align: 0x%02x\r\n", MPD_ReadAuxI(1, DPCD_LANE_ALIGN_STATUS));
    printf("sink: 0x%02x\r\n", MPD_ReadAuxI(1, DPCD_SINK_STATUS));
    printf("request: 0x%02x\r\n", MPD_ReadAuxI(1, DPCD_ADJUST_REQUEST));
    printf("errsym: 0x%08x\r\n", MPD_ReadAuxI(4, DPCD_SYMBOL_ERR_CNT));

    MPD_WriteAUXI(1, DPCD_EDP_CONFIGURATION_SET, 0x02); //Set No-ASSR and Enhanced Framing
    MPD_WriteAUXI(1, DPCD_DOWNSPREAD_CTRL, 0x00);    //Set SSCG

    return 0;
}

#define EDID_DEV_ADR        0x50    //7b1010000
#define EDID_BASE_ADDRESS   0x00

int MPD_ReadEDID(void* data, int bsize) {
    IIC_WriteRegI(DP0_AUXADDR, EDID_DEV_ADR);
    IIC_WriteRegI(DP0_AUXCFG0, 0x0014); //Initate Write(Address only)
    TIM_Delay_Us(1000);
    uint32_t code = IIC_ReadRegI(DP0_AUXSTATUS, 3);
    if(GETBITS(code, 16, 18) || GETBITS(code, 4, 7)) {
        printf("DDC EDID NACK, reason: %02x\r\n", GETBITS(code, 4, 7));
        return -1;
    }
    
    IIC_WriteRegI(DP0_AUXWDATA, EDID_BASE_ADDRESS);
    IIC_WriteRegI(DP0_AUXCFG0, 0x0004); //Send Register Address, 1 byte
    TIM_Delay_Us(1000);

    IIC_WriteRegI(DP0_AUXCFG0, 0x0015); //Initate Read(Address only)
    TIM_Delay_Us(1000);

    while(bsize) {
        uint32_t send_byte = bsize > 16 ? 16 : bsize;
        IIC_WriteRegI(DP0_AUXCFG0, 0x0005 | ((send_byte - 1) << 8)); //IIC Receive Data
        TIM_Delay_Us(1000);
        uint32_t code = IIC_ReadRegI(DP0_AUXSTATUS, 3);
        //FIXME: TC358867XBG aux_status can produce invalid data
        uint32_t recv_byte = /*GETBITS(code, 4, 7) == 0x4 ? 0 : */GETBITS(code, 8, 15);
        IIC_ReadReg(DP0_AUXRDATA, data, recv_byte);
        data = data + recv_byte;
        bsize = bsize - recv_byte;
    }

    IIC_WriteRegI(DP0_AUXCFG0, 0x0011); //STOP
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

void MPD_CfgStream(){
    uint32_t edid[64]; //256bytes
    if(!MPD_ReadEDID(edid, 256)) {
        MPD_Test_EDID(edid);
    }

#ifdef MPD_TEST
    IIC_WriteRegI(PXL_PLLCTRL, PLLBYP | PLLEN); //FIXME
    IIC_WriteRegI(PXL_PLLPARAM, IN_SEL_REFCLK | MPD_PXLPARAM);  //Set PXLPLL
    IIC_WriteRegI(PXL_PLLCTRL, PLLUPDATE | PLLEN);   //Enable PXL PLL
    TIM_Delay_Ms(10);
    IIC_WriteRegI(TSTCTL, ENI2CFILTER | MPD_TEST_MODE | (MPD_TEST_COLOR << 8));
#else
    IIC_WriteRegI(TSTCTL, ENI2CFILTER | (MPD_TEST_COLOR << 8));
#endif

    IIC_WriteRegI(VPCTRL0, MPD_DP_BPP | FRMSYNC_DISABLED | MSF_DISABLED | VSDELAY(MPD_VSDELAY));

    IIC_WriteReg(VP_TIM, (void*)rgb_timing, 16);
    IIC_WriteRegI(VFUEN0, VFUEN);    //Commit Timing Variable

    IIC_WriteReg(DP0_TIM, (void*)dp_timing, 20);
    IIC_WriteRegI(DPIPXLFMT, MPD_DPI_POL | MPD_DPI_FMT | MPD_DPI_BPP);

    IIC_WriteRegI(DP0_MISC, MAX_TU_SYMBOL(MPD_MAX_TU_SYMBOL) | TU_SIZE(MPD_TU_SIZE_RECOMMENDED) | BPC | FMT_RGB);  //DP0_Misc
    IIC_WriteRegI(DP0_VIDMNGEN1, MPD_DP_VIDGEN_N);

    IIC_WriteRegI(DP0CTL, VID_MN_GEN | EF_EN | DP_EN);
    TIM_Delay_Us(500);
    IIC_WriteRegI(DP0CTL, VID_MN_GEN | EF_EN | VID_EN | DP_EN);

#ifdef MPD_TEST
    IIC_WriteRegI(SYSCTRL, DP0_AUDSRC_NO_INPUT | DP0_VIDSRC_COLOR_BAR);
#else
    IIC_WriteRegI(SYSCTRL, DP0_AUDSRC_NO_INPUT | DP0_VIDSRC_DPI_RX);
#endif

    TIM_Delay_Ms(100);
    printf("vid_M: %d\r\n", IIC_ReadRegI(DP0_VMNGENSTATUS, 4));
    printf("vid_N: %d\r\n", IIC_ReadRegI(DP0_VIDMNGEN1, 4));
    printf("vfuen: %d\r\n", IIC_ReadRegI(VFUEN0, 4));
    return;
}

void MPD_Disable(){
	IIC_WriteRegI(DP0_SRCCTRL, 0);
    IIC_WriteRegI(DP0CTL, 0);
    IIC_WriteRegI(DP_PHY_CTRL, DP_PHY_CTRL_EN | PHY_M0_RST | PHY_M1_RST);
    return;
}