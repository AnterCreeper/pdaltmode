#ifndef __BC_H
#define __BC_H

#include "timer.h"
#include "gpio.h"

#define UDM_BC_CMPO     (1<<19)
#define UDP_BC_CMPO     (1<<18)

#define UDM_BC_VSRC     (1<<17)
#define UDP_BC_VSRC     (1<<16)

#define BC_DP_Read_VCC()    GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_17)
#define BC_DP_Read_VSRC()   (AFIO->CTLR & UDP_BC_CMPO)
#define BC_DM_Toggle_0V0()  AFIO->CTLR &= ~UDM_BC_VSRC
#define BC_DM_Toggle_0V6()  AFIO->CTLR |= UDM_BC_VSRC

typedef enum {
    STA_BCIDLE = 0,
    STA_WAIT,
    STA_RESET,
} BC_STATUS;

typedef struct BC_FSM {
    timer_t timer;
    BC_STATUS status;
} bc_state_t;

void BC_INIT(bc_state_t* BC_Ctl);
void BC_Connect(bc_state_t* BC_Ctl);
void BC_Disconnect();
void BC_Wait(bc_state_t* BC_Ctl);

#endif