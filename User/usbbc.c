#include "usbbc.h"
#include "debug.h"

static inline int BC_Timeout(bc_state_t* BC_Ctl, uint32_t ms) {
    timer_t next_timer = BC_Ctl->timer + ms*(SystemCoreClock/8000);
    return TIM_Timeout(next_timer);
}

void BC_Proc(bc_state_t* BC_Ctl) {
    switch(BC_Ctl->status) {
    case STA_WAIT:
        if(BC_DP_Read_VSRC() && !BC_DP_Read_VCC()) {
            BC_DM_Toggle_0V6();
            BC_Ctl->timer = TIM_GetTimer();
            BC_Ctl->status = STA_RESET;
            break;
        }
        if(BC_Timeout(BC_Ctl, 1000)) BC_Ctl->status = STA_BCIDLE;
        break;
    case STA_RESET:
        if(!BC_DP_Read_VSRC() || BC_Timeout(BC_Ctl, 40)) {
            BC_DM_Toggle_0V0();
            printf("USBBC detected\r\n");
            BC_Ctl->status = STA_BCIDLE;
        }
        break;
    case STA_BCIDLE:
    default:
        break;
    }
    return;
}

void BC_Wait(bc_state_t* BC_Ctl) {
    while(BC_Ctl->status != STA_BCIDLE) __WFI();
    TIM_Unregister(BC_Proc);
    return;
}

void BC_INIT(bc_state_t* BC_Ctl) {
    printf("Enable USBBC\r\n");
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_16 | GPIO_Pin_17;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    BC_Ctl->status = STA_BCIDLE;
    return;
}

void BC_Connect(bc_state_t* BC_Ctl) {
    BC_Ctl->status = STA_WAIT;
    BC_Ctl->timer = TIM_GetTimer();
    TIM_Register(BC_Proc, BC_Ctl);
    return;
}

void BC_Disconnect() {
    BC_DM_Toggle_0V0();
    return;
}
