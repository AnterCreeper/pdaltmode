//#define IGNORE_HPD
//#define NO_MPD
//#define FORCE_SOFTMUL
#define FLIP_SEL 0    //reverse type-c mux select signal, debug only and set 0 for normal

#include "timer.h"
#include "sys.h"
#include "gpio.h"
#include "ws2812.h"
#include "iic.h"
#include "mpd.h"
#include "usbpd.h"
#include "debug.h"
#include <ch32x035_opa.h>

void VSNS_INIT() {
    GPIO_IN_INIT(GPIOA, GPIO_Pin_7);
    GPIO_Analog_INIT(GPIOA, GPIO_Pin_4);

    OPA_Unlock();
    OPA_InitTypeDef OPA_InitStructure = {0};
    OPA_InitStructure.OPA_NUM = OPA2;
    OPA_InitStructure.PSEL = CHP0;
    OPA_InitStructure.NSEL = CHN_PGA_4xIN;
    OPA_InitStructure.Mode = OUT_IO_OUT0;
    OPA_InitStructure.FB = FB_ON;
    OPA_Init(&OPA_InitStructure);
    OPA_Cmd(OPA2, ENABLE);
    printf("Enable OPA\r\n");

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_DeInit(ADC1);
    ADC_CLKConfig(ADC1, ADC_CLK_Div4);
    ADC_InitTypeDef ADC_InitStructure = {0};
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);
    printf("Enable ADC\r\n");

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

//pre malloced buffer
__attribute__ ((aligned(4))) uint8_t PD_Rx_Buf[34]; /* PD receive buffer */
__attribute__ ((aligned(4))) uint8_t PD_Tx_Buf[34]; /* PD send buffer */

pd_state_t* USBPD_FSM;

void GPIO_Toggle_INIT() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_OUT_INIT(GPIOA, GPIO_Pin_1, Bit_SET); //TYPEC_VEN#
    GPIO_OUT_INIT(GPIOA, GPIO_Pin_3, Bit_SET); //TYPEC_SEL#

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_OUT_INIT(GPIOB, GPIO_Pin_3, Bit_SET); //TYPEC_EN#
    GPIO_OUT_INIT(GPIOB, GPIO_Pin_11, Bit_SET); //TYPEC_VCONN1
    GPIO_OUT_INIT(GPIOB, GPIO_Pin_12, Bit_SET); //TYPEC_VCONN2

    printf("Enable GPIO\r\n");
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
    int sel = state ^ ((USBPD->CONFIG & CC_SEL) > 0);
    printf("Toggle DP MUX %s\r\n", sel ? "Flip" : "Normal");
    GPIO_WriteBit(GPIOA, GPIO_Pin_3, sel); //TYPEC_SEL#
    return;
}

void GPIO_Toggle_DPSIG(int state){
    printf("Toggle DP SIGNAL %s\r\n", state ? "ON" : "OFF");
    GPIO_WriteBit(GPIOB, GPIO_Pin_3, state ? Bit_RESET : Bit_SET); //TYPEC_EN#
    return;
}

void GPIO_PD_INIT() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);               /* Open PD I/O clock, AFIO clock and PD clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);
    GPIO_IN_INIT(GPIOC, GPIO_Pin_14 | GPIO_Pin_15);
    return;
}

void PD_FSM_Reset(pd_state_t* PD_Ctl) {   
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
    PD_Ctl->DP_Status.Trained = 0;
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
    GPIO_Toggle_INIT();

    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;
    USBPD->PORT_CC1 = CC_CMP_45 | CC_PU_180; //180 for 1A5
    USBPD->PORT_CC2 = CC_CMP_45 | CC_PU_180; //180 for 1A5

    PD_Ctl->Rx_Buf = PD_Rx_Buf;
    PD_Ctl->Tx_Buf = PD_Tx_Buf;
    USBPD->CONFIG = PD_DMA_EN;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;

    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = USBPD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
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

static void PD_Load_Header(uint8_t type, pd_state_t* PD_Ctl) {
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

static void PD_Load_Header_ID(uint8_t type, uint8_t id, pd_state_t* PD_Ctl) {
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

static void PD_Load_DataObj(void* data, size_t len, pd_state_t* PD_Ctl) {
    if((len & 3) != 0) {
        char str[64];
        __builtin_sprintf(str, "Misalign of PDO with length %d", len);
        assert(str);
    }
    ((uint8_t*)(PD_Ctl->Tx_Buf))[1] |= len << 2; //Load data size into header
    __builtin_memcpy(PD_Ctl->Tx_Buf + 2, data, len);
    return;
}

static void PD_Load_VDM(uint16_t svid, int cmd, void* buf, pd_state_t* PD_Ctl) {
    //see description above.
    uint32_t data = cmd & 0x1F;
    data |= DEF_CMDTYPE_INIT << 6;
    data |= 0x8000; //Structured VDM
    data |= svid << 16;
    ((uint32_t*)buf)[0] = data;
    return;
}

static void PD_Load_VDM_POS(uint8_t pos, void* buf, pd_state_t* PD_Ctl) {
    if(!pos) assert("Empty VDO Pos");
    ((uint32_t*)buf)[0] |= pos << 8;
    return;
}

static void PD_Load_VDM_CMDTYPE(uint8_t cmdtype, void* buf, pd_state_t* PD_Ctl) {
    ((uint32_t*)buf)[0] |= cmdtype << 6;
    return;
}

static void PD_Load_VDM_DP_S(void* buf, pd_state_t* PD_Ctl) {
    //see description above.
    uint32_t data = 0x1;
    ((uint32_t*)buf)[1] = data;
    return;
}

int PD_DP_Pin_Decider(dp_status_t info) {
    int mode = info.Receptable ? info.UFP_Pin : info.DFP_Pin;
    printf("assign: 0x%02x\r\n", mode);
    if(mode & BIT(2)) return BIT(2); //Assignment C
    if(mode & BIT(4)) return BIT(4); //Assignment E
    return BIT(2);
}

static void PD_Load_VDM_DP_CFG(void* buf, pd_state_t* PD_Ctl) {
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
    GPIO_Toggle_DPSEL(FLIP_SEL);
    uint32_t data = 0x2 | (0x1 << 2) | (PD_DP_Pin_Decider(PD_Ctl->DP_Status) << 8); //Set UFP_U as UFP_D and Enable DP1.3 Signaling
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
        TIM_Delay(4);
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
            PD_Ctl->Msg_ID = (PD_Ctl->Msg_ID + 1) & 0x07;
            return 0;
        }
        TIM_Delay_Us(20);
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
            TIM_Delay_Us(20);
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
        USBPD->STATUS |= IF_RX_ACT;
        size_t len = USBPD->BMC_BYTE_CNT;
        TIM_Delay_Us(20);
        PD_Load_Header_ID(DEF_TYPE_GOODCRC, PD_PARSE_MSGID(USBPD_FSM->Rx_Buf), USBPD_FSM);
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
    PD_Ctl->Timer = TIM_GetTimer(); //Delay Start from VBUS ON
    if(!passive) {
        GPIO_Toggle_VCONN(ccpin);
    }
    return;
}

void PD_Disconnect(pd_state_t* PD_Ctl) {
    PD_Ctl->Connect = DISCONNECTED;
    PD_Ctl->Status = STA_DISCONNECT;
    PD_PHY_Reset(PD_Ctl);
    printf("PD Disconnect\r\n");
    return;
}

static int PD_TestCC() {
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
                BC_Disconnect(&PD_Ctl->BC_Ctl);
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
                BC_Connect(&PD_Ctl->BC_Ctl);
                return 0;
            }
            return -1;
        }
    }
}

int PD_Delay(unsigned int ms, pd_state_t* PD_Ctl) {
    timer_t next_timer = PD_Ctl->Timer + ms*(SystemCoreClock/8000);
    PD_Enter_ListenMode(PD_Ctl);
    while(!TIM_Timeout(next_timer)) {
        __WFI();
        if(PD_Ctl->IRQ_ret) {
            PD_Exit_ListenMode(PD_Ctl);
            return -1;
        }
    }
    PD_Exit_ListenMode(PD_Ctl);
    PD_Ctl->Timer = TIM_GetTimer(); 

    while(PD_Detect(PD_Ctl)) TIM_Delay_Us(1000);
    if(PD_Ctl->Status == STA_DISCONNECT) return -1;

    return 0;
}

int PD_SendRecv_VDM(int cmd, int timeout, void* data, size_t len, pd_state_t* PD_Ctl) {
    if(PD_Send_Retry_Rst(DEF_TYPE_VENDOR_DEFINED, data, len, PD_Ctl)) return -1;
    PD_Ctl->Timer = TIM_GetTimer();
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

static int PD_Test_IDENT(pd_state_t* PD_Ctl) {
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

static int PD_Test_SVID(pd_state_t* PD_Ctl) {
    uint16_t* svid = PD_Ctl->Rx_Buf + 2;
    for(int i = 0; i < 12; i++) {
        int pos = i ^ 1; //weirdo big endian whin whole PD's little endian stuffs...
        if(svid[pos] == DEF_SVID_EOF) return -1;
        if(svid[pos] == DEF_SVID_DISPLAYPORT) return 0;
    }
    return 1;
}

static int PD_Test_MODE(pd_state_t* PD_Ctl) {
    uint32_t mode[6];

    int len = PD_PARSE_SIZE(PD_Ctl->Rx_Buf) - 1;
    __builtin_memcpy(mode, PD_Ctl->Rx_Buf + 6, 4*len);

    for(int i = 0; i < len; i++) {
        printf("MODE %d:\r\n", i + 1);
        /* PD_PARSE_DP_CAP
            00 = Reserved
            01 = UFP_D(DP Sink Device) capable, including Branch Device
            10 = DFP_D(DP Source Device) capable, including Branch Device
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

static int PD_Test_DP_S(pd_state_t* PD_Ctl) {
    uint16_t* data = PD_Ctl->Rx_Buf + 6;
    uint32_t status = (data[1] << 16) | data[0];
    PD_Ctl->DP_Status.Link = status;

    printf("DP Link Info:\r\n");
    /* PD_PARSE_DP_STA, inconsistency with previous PD_PARSE_DP_CAP one...
        00 = neither DFP_D nor UFP_D connected or adapter is disabled
        01 = DFP_D Connected
        10 = UFP_D Connected
        11 = Both DFP_D and UFP_D Connected
    */
    printf("enabled: %d\r\n", PD_PARSE_DP_ENABLED(status));
    printf("connected: %d\r\n", PD_PARSE_DP_STA(status));
    printf("low_power: %d\r\n", PD_PARSE_DP_LP(status));
    printf("multi_func: %d\r\n", PD_PARSE_DP_MUL(status));
    printf("hpd: %d\r\n", PD_PARSE_DP_HPD(status));
    printf("irq: %d\r\n", PD_PARSE_DP_IRQ(status));
    printf("usb_req: %d\r\n", PD_PARSE_DP_USB(status));
    printf("exit_req: %d\r\n", PD_PARSE_DP_EXIT(status));

    if(!PD_PARSE_DP_ENABLED(status)) return -1;
    if(!PD_PARSE_DP_STA_UFP_D(status)) return -1;
    return 0;
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
                if(new_enabled && !PD_Ctl->DP_Status.Trained) PD_Ctl->VDM_Status = STA_DP_CONFIG;
                PD_Ctl->DP_Status.Enabled = new_enabled;
                return -1;
            }
            //hot-plug detect
            if(PD_PARSE_DP_HPD(PD_Ctl->DP_Status.Link)) PD_Ctl->VDM_Status = STA_MPD_HPD;
            else {
                MPD_Disable();
                PD_Ctl->DP_Status.Trained = 0;
            }
            //irq detect
            if(PD_PARSE_DP_IRQ(PD_Ctl->DP_Status.Link)) PD_Ctl->VDM_Status = STA_MPD_IRQ;
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
        PD_Ctl->Timer = TIM_GetTimer(); //Set Timer start
        printf("Perform DP Aux HPD cycle\r\n");
        GPIO_Toggle_DPSIG(RESET);
        if(PD_Delay(5, PD_Ctl)) return;
        PD_Load_VDM(DEF_SVID_DISPLAYPORT, DEF_VDM_DP_CONFIG, buf, PD_Ctl);
        PD_Load_VDM_DP_CFG(buf, PD_Ctl);
        if(PD_SendRecv_VDM(DEF_VDM_DP_CONFIG, 30, buf, 8, PD_Ctl)) break;
        GPIO_Toggle_DPSIG(SET); //Configure DP Signal and SBU Mux
#ifdef IGNORE_HPD
        PD_Ctl->VDM_Status = STA_MPD_HPD;
#else
        PD_Ctl->VDM_Status = PD_PARSE_DP_HPD(PD_Ctl->DP_Status.Link) ? STA_MPD_HPD : STA_VDM_IDLE;
#endif
        break;
    case STA_MPD_HPD:
#ifndef IGNORE_HPD
        if(PD_Ctl->DP_Status.Trained) {
            printf("ignore hpd event, link is already established.\r\n");
            PD_Ctl->VDM_Status = STA_VDM_IDLE;
            break;
        }
#endif
        PD_Enter_ListenMode(PD_Ctl);
#ifndef NO_MPD
        if(!MPD_CfgLink()) {
            MPD_CfgStream();
            WS2812_SetColor(BLUE);
            PD_Ctl->DP_Status.Trained = 1;
        } else {
            WS2812_SetColor(YELLOW);
            PD_Ctl->DP_Status.Trained = 0;
        }
#endif
        PD_Exit_ListenMode(PD_Ctl);
        PD_Ctl->VDM_Status = STA_VDM_IDLE;
        break;
    case STA_MPD_IRQ:
        MPD_HPD_IRQ();
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

static int PD_Test_Cap(pd_state_t* PD_Ctl) {
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
        MPD_Disable();
        WS2812_SetColor(RED);
        BC_Wait(&PD_Ctl->BC_Ctl);
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
                printf("USBPD not detected\r\n");
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
            PD_Ctl->Timer = TIM_GetTimer(); 
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
    VSNS_INIT();
    WS2812_INIT();
#ifndef NO_MPD
    IIC_Init();
    MPD_Init();
#endif
    pd_state_t PD_Ctl;
    PD_INIT(&PD_Ctl);
    BC_INIT(&PD_Ctl.BC_Ctl);
    EXTI_INIT();
    while(1) {
        if(!PD_Detect(&PD_Ctl)) PD_Proc(&PD_Ctl);
        else TIM_Delay_Us(20);
    }
    __builtin_unreachable();
    return 0;
}