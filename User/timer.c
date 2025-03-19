#include "timer.h"
#include <ch32x035_tim.h>

volatile timer_t TIM1_CNT;
timer_irq_t TIM_IRQ[TIM_IRQ_SIZE];

void STK_INIT() {
    SysTick->CNT = 0;
    SysTick->CTLR = 1;
    TIM_Delay(16);
    SysTick->SR = 0;
    return;
}

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler() {
    if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM1_CNT = TIM_GetTimer();
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
    for(int i = 0; i < TIM_IRQ_SIZE; i++) {
        if(TIM_IRQ[i].func) {
            void (*Handler)(void*) = TIM_IRQ[i].func;
            Handler(TIM_IRQ[i].args);
        }
    }
    return;
}

void TIM1_INIT() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    TIM_TimeBaseInitStructure.TIM_Period = 499;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 47; //1ms interval
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    __builtin_memset(TIM_IRQ, 0, sizeof(TIM_IRQ));
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM_Cmd(TIM1, ENABLE);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    return;
}

void Timer_INIT() {
    STK_INIT();
    TIM1_INIT();
    return;
}

void TIM_Delay(unsigned long us) {
    volatile unsigned long t = us*(SystemCoreClock/8000000);
    while(t--) __asm__("");
    return;
}

void TIM_Wait(timer_t next_timer) {
    SysTick->CMP = next_timer;
    while(!(SysTick->SR & 0x1));
    SysTick->SR = 0;
    return;
}

void TIM_WaitL(timer_t next_timer) {
    while(TIM1_CNT < next_timer - SystemCoreClock/8000) __WFI();
    TIM_Wait(next_timer);
    return;
}

int TIM_Register(void* func, void* args) {
    if(!func) return -1;
    for(int i = 0; i < TIM_IRQ_SIZE; i++) {
        if(!TIM_IRQ[i].func) {
            TIM_IRQ[i].func = func;
            TIM_IRQ[i].args = args;
            return 0;
        }
    }
    return -1;
}

int TIM_Unregister(void* func) {
    for(int i = 0; i < TIM_IRQ_SIZE; i++) {
        if(TIM_IRQ[i].func == func) {
            TIM_IRQ[i].func = 0;
            TIM_IRQ[i].args = 0;
            return 0;
        }
    }
    return -1;
}