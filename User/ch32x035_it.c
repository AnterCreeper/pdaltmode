/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32x035_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/10/28
 * Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32x035_it.h"

void NMI_Handler() {
    NVIC_SystemReset();
    return;
}

const char reason[13][32] = {
    "Instruction address misaligned",
    "Instruction access fault",
    "Illegal instruction",
    "Breakpoint",
    "Load address misaligned",
    "Load access fault",
    "Store/AMO address misaligned",
    "Store/AMO access fault",
    "Environment call from U-mode",
    "Environment call from H-mode",
    "Environment call from S-mode",
    "Environment call from M-mode",
    "Unknown"
};

//void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler() {
    uint32_t v_ra, v_sp;
    asm volatile ("mv\t%0, ra"
        :"=r"(v_ra));
    asm volatile ("mv\t%0, sp"
        :"=r"(v_sp));
    uint32_t v_mepc,v_mcause,v_mtval;
    v_mepc=__get_MEPC();
    v_mcause=__get_MCAUSE();
    v_mtval=__get_MTVAL();

    printf("An unexpected exception was occured at 0x%08x\r\n", v_mepc);
    printf("mtval: 0x%08x\r\n", v_mtval);
    printf("errno %d: %s\r\n", v_mcause, reason[v_mcause > 11 ? 12 : v_mcause]);

    printf("register:\r\n");
    printf("ra: 0x%08x\r\n", v_ra);
    printf("sp: 0x%08x\r\n", v_sp);

    while(1) __WFI();
    return;
}

void EXTI15_8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI15_8_IRQHandler() {
    if(EXTI_GetITStatus(EXTI_Line14) != RESET) {
        SystemInit();
        printf("Exit SLP Mode\r\n");
        EXTI_ClearITPendingBit(EXTI_Line14);
        NVIC_DisableIRQ(EXTI15_8_IRQn);
    }
    if(EXTI_GetITStatus(EXTI_Line15) != RESET) {
        SystemInit();
        printf("Exit SLP Mode\r\n");
        EXTI_ClearITPendingBit(EXTI_Line15);
        NVIC_DisableIRQ(EXTI15_8_IRQn);
    }
    return;
}
