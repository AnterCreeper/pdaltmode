#ifndef __TIMER_H
#define __TIMER_H

#define __timer_t_defined
#include "system_ch32x035.h"
#include "debug.h"

#define TIM_IRQ_SIZE    8

typedef	uint64_t timer_t;

typedef struct {
    void* func;
    void* args;
} timer_irq_t;

void Timer_INIT();
#define TIM_PAUSE() TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE)
#define TIM_RUN()   TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE)

#define STK_STIE    (1 << 1)

inline static timer_t TIM_GetTimer() {
    return SysTick->CNT;
}

inline static int TIM_Timeout(timer_t timer) {
    return -(SysTick->CNT > timer);
}

void TIM_Delay(unsigned long us);
void TIM_Wait(timer_t next_timer);
void TIM_WaitL(timer_t next_timer);
#define TIM_Delay_Us(x) TIM_Wait(TIM_GetTimer()+x*(SystemCoreClock/8000000))
#define TIM_Delay_Ms(x) TIM_WaitL(TIM_GetTimer()+x*(SystemCoreClock/8000))

int TIM_Register(void* func, void* args);
int TIM_Unregister(void* func);

#endif