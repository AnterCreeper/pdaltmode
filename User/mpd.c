#include "timer.h"
#include "gpio.h"
#include "iic.h"
#include "mpd.h"
#include "debug.h"

#define GETBITS(x, n, m) ((x >> n) & ((1 << (m -n + 1)) - 1))

int __IIC_WriteReg(uint16_t address, uint32_t data, int len, const int imm) {
#ifdef MPD_IIC_DBG
    uint32_t dbg = data;
#endif
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
#ifdef MPD_IIC_DBG
    printf("debug IIC: write ");
    if(imm) printf("%08x", dbg);
    else {
        for(int i = len - 1; i >= 0; i--) printf("%02x", ((uint8_t*)data)[i]);
    }
    printf(" at 0x%04x\r\n", address);
#endif
    return 0;
}

int __IIC_ReadReg(uint16_t address, uint32_t data, int len, const int imm) {
    if(!imm && len <= 0) return -1;
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
#ifdef MPD_IIC_DBG
    printf("debug IIC: read ");
    if(imm) printf("%08x", data);
    else {
        for(int i = len - 1; i >= 0; i--) printf("%02x", ((uint8_t*)data)[i]);
    }
    printf(" at 0x%04x\r\n", address);
#endif
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
    printf("Reset MPD\r\n");
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_OUT_INIT(GPIOB, GPIO_Pin_1, Bit_SET);    //MCU_DP_RST#
    TIM_Delay_Us(1000);   //Tcorerdy
    GPIO_IPU_INIT(GPIOB, GPIO_Pin_0);    //MCU_DP_INT

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
    IIC_WriteRegI(DP0_SRCCTRL, DP_SRCCTRL_SWG0(MPD_SWG) | DP_SRCCTRL_PRE0(MPD_PRE) |
                                DP_SRCCTRL_SCRMBLDIS | DP_SRCCTRL_EN810B |
                                DP_SRCCTRL_NOTP | DP_SRCCTRL_LANESKEW |
                                MPD_BW | DP_SRCCTRL_AUTOCORRECT);
#endif

    IIC_WriteRegI(DP1_SRCCTRL, DP_SRCCTRL_SWG0(MPD_SWG) | DP_SRCCTRL_PRE0(MPD_PRE) |
                                DP_SRCCTRL_SCRMBLDIS | DP_SRCCTRL_EN810B |
                                DP_SRCCTRL_NOTP | MPD_BW);
    IIC_WriteRegI(SYS_PLLPARAM, MPD_REFCLK | LSCLK_DIV_1);

#ifdef MPD_2LANE
    uint32_t dp_phy_ctrl = BGREN | PWR_SW_EN | PHY_2LANE | PHY_A0_EN | PHY_M0_EN;
#else
    uint32_t dp_phy_ctrl = BGREN | PWR_SW_EN | PHY_A0_EN | PHY_M0_EN;
#endif
    IIC_WriteRegI(DP_PHY_CTRL, dp_phy_ctrl);

    printf("Enable MPD PLL\r\n");
    IIC_WriteRegI(DP0_PLLCTRL, PLLUPDATE | PLLEN);   //Enable DP0 PLL
    TIM_Delay_Ms(8);
    IIC_WriteRegI(DP1_PLLCTRL, PLLUPDATE | PLLEN);   //Enable DP1 PLL
    TIM_Delay_Ms(8);
#ifdef MPD_TEST
    IIC_WriteRegI(PXL_PLLPARAM, IN_SEL_REFCLK | MPD_PXLPARAM);  //Set PXLPLL
    IIC_WriteRegI(PXL_PLLCTRL, PLLUPDATE | PLLEN);   //Enable PXL PLL
    TIM_Delay_Ms(8);
#endif

    printf("Reset MPD Func\r\n");
    IIC_WriteRegI(DP_PHY_CTRL, dp_phy_ctrl | DP_PHY_RST | PHY_M0_RST | PHY_M1_RST);
    IIC_WriteRegI(DP_PHY_CTRL, dp_phy_ctrl);

    uint32_t result;
    do {
        TIM_Delay_Us(500);
        IIC_ReadReg(DP_PHY_CTRL, &result, 4);
    } while((result & PHY_RDY) == 0); //Query DP PHY Ready
    /*
    MPD_IRQInit();
    IIC_WriteRegI(0x0560, (1 << 16));   //Enable MPD SYS Interrupt
    */
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

    if((IIC_ReadRegI(DP0CTL, 1) & DP_EN) != 0) {
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
    do {
        TIM_Delay_Us(1000);
        IIC_ReadReg(DP0_LTSTAT, &result, 4);
    } while((result & LT_LOOPDONE) == 0);
    code = GETBITS(result, 8, 12);
    status = GETBITS(result, 0, 6);
    printf("Pattern 2:\r\n");
    printf("code: 0x%02x\r\n", code);
    printf("status: 0x%02x\r\n", status);

    //LT Status
    uint32_t symbol, request = 0;
    MPD_ReadAux(4, DPCD_LANE_STATUS, &result);
    MPD_ReadAux(1, DPCD_ADJUST_REQUEST, &request);
    MPD_ReadAux(4, DPCD_SYMBOL_ERR_CNT, &symbol);
    printf("Sink LT Status:\r\n");
    printf("code: 0x%08x\r\n", result);
    printf("request: 0x%02x\r\n", request);
    printf("symbol_err: 0x%08x\r\n", symbol);
    
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

void MPD_CfgVideo(){
    uint32_t pm = MPD_ReadAuxI(1, 0x00600);
    printf("dp_sink_pm: 0x%02x\r\n", pm);
    uint32_t cap = MPD_ReadAuxI(1, 0x00005);
    printf("dp_sink_type: 0x%02x\r\n", cap);

    uint32_t edid[64]; //256bytes
    if(!MPD_ReadEDID(edid, 256)) {
        MPD_Test_EDID(edid);
    }

    IIC_WriteRegI(VPCTRL0, MPD_DP_BPP | FRMSYNC_ENABLED | MSF_DISABLED | VSDELAY(MPD_VSDELAY));

    IIC_WriteReg(VP_TIM, (void*)rgb_timing, 16);
    IIC_WriteRegI(VFUEN0, 0x01);    //Commit Timing Variable
    IIC_WriteRegI(DPIPXLFMT, MPD_DPI_POL | MPD_DPI_FMT | MPD_DPI_BPP);

    IIC_WriteReg(DP0_TIM, (void*)dp_timing, 20);
    IIC_WriteRegI(DP0_MISC, MAX_TU_SYMBOL(42) | TU_SIZE(63) | BPC | FMT_RGB);  //DP0_Misc
    return;
}

void MPD_CfgTest(){
#ifdef MPD_TEST
    printf("Enable Test Transmition\r\n");
#else
    printf("Enable Video Transmition\r\n");
#endif
    IIC_WriteRegI(TSTCTL, ENI2CFILTER | MPD_TEST_MODE | (MPD_TEST_COLOR << 8));
    IIC_WriteRegI(DP0_VIDMNGEN1, MPD_DP_VIDGEN_N);

    IIC_WriteRegI(DP0CTL, VID_MN_GEN | EF_EN | DP_EN);
    IIC_WriteRegI(DP0CTL, VID_MN_GEN | EF_EN | VID_EN | DP_EN);
#ifdef MPD_TEST
    IIC_WriteRegI(SYSCTRL, DP0_AUDSRC_NO_INPUT | DP0_VIDSRC_COLOR_BAR); //SYSCTRL, Set DP Video Source
#else
    IIC_WriteRegI(SYSCTRL, DP0_AUDSRC_NO_INPUT | DP0_VIDSRC_DPI_RX); //SYSCTRL, Set DP Video Source
#endif
    TIM_Delay_Ms(100);

    printf("vid_M: %d\r\n", IIC_ReadRegI(DP0_VMNGENSTATUS, 4));
    printf("vid_N: %d\r\n", IIC_ReadRegI(DP0_VIDMNGEN1, 4));
    return;
}

void MPD_Disable(){
    IIC_WriteRegI(DP0CTL, 0);
#ifdef MPD_2LANE
    uint32_t dp_phy_ctrl = BGREN | PWR_SW_EN | PHY_2LANE | PHY_A0_EN | PHY_M0_EN;
#else
    uint32_t dp_phy_ctrl = BGREN | PWR_SW_EN | PHY_A0_EN | PHY_M0_EN;
#endif
    IIC_WriteRegI(DP_PHY_CTRL, dp_phy_ctrl | DP_PHY_RST | PHY_M0_RST | PHY_M1_RST);
    IIC_WriteRegI(DP_PHY_CTRL, dp_phy_ctrl);
    return;
}