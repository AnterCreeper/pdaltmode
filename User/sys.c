#include "timer.h"
#include "sys.h"
#include "system_ch32x035.h"

#define GETBITS(x, n, m) ((x >> n) & ((1 << (m - n + 1)) - 1))

void __assert(const char* str, uint32_t line) {
    printf("assert fault at line %d, reason %s\r\n", line, str);
    while(1) __WFI();
    __builtin_unreachable();
    return;
}

#define ALPHABET_OFFSET ('A' - 1)
#define NUMBER_OFFSET ('0')

void SYS_Test_Arch(char str[8]) {
    uint32_t arch = __get_MARCHID();
    str[0] = GETBITS(arch, 26, 30) + ALPHABET_OFFSET;
    str[1] = GETBITS(arch, 21, 25) + ALPHABET_OFFSET;
    str[2] = GETBITS(arch, 16, 20) + ALPHABET_OFFSET;
    str[3] = '-';
    str[4] = GETBITS(arch, 10, 14) + ALPHABET_OFFSET;
    str[5] = GETBITS(arch, 5, 9) + NUMBER_OFFSET;
    str[6] = GETBITS(arch, 0, 4) + ALPHABET_OFFSET;
    str[7] = '\0';
    return;
}

void SYS_Test_ISA(char str[32]) {
    uint32_t isa = __get_MISA();
    __builtin_memset(str, 0, 32);
    for(int i = 0; i < 26; i++) {
        if(isa & (1 << i)) {
            *str = i + ALPHABET_OFFSET + 1;
            str++;
        }
    }
    return;
}

void SYS_INIT() {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemInit();
    Timer_INIT();
#if SDI_PRINT == SDI_PR_CLOSE
    USART_Printf_Init(115200);
#else
    SDI_Printf_Enable();
#endif
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    printf("%s version %s by %s\r\n", TARGET_PROJECT, TARGET_VERSION, TARGET_AUTHOR);
    printf("%s %s %s %s\r\n", TARGET_COMPILER, __VERSION__, __DATE__, __TIME__);
    printf("sys_clk: %d\r\n", SystemCoreClock);
    printf("chip_id: 0x%08x\r\n", DBGMCU_GetCHIPID());
    printf("rom_size_kib: %d\r\n", ESIG->FLACAP);

    /*
    char str0[8], str1[32];
    SYS_Test_Arch(str0);
    SYS_Test_ISA(str1);
    printf("chip_arch: %s RV%d%s\r\n", str0, 1 << (GETBITS(__get_MISA(), 30, 31) + 4), str1);
    printf("chip_impid: 0x%08x\r\n", __get_MIMPID());
    printf("chip_vendor: 0x%08x\r\n", __get_MVENDORID());
    */
    return;
}

void SYS_SLP() {
#ifdef SLP_ENABLE
    printf("Enter SLP Mode\r\n");
    TIM_PAUSE();
#endif
    EXTI_ClearITPendingBit(EXTI_Line14);
    EXTI_ClearITPendingBit(EXTI_Line15);
    //NVIC_DisableIRQ(EXTI7_0_IRQn);
    NVIC_EnableIRQ(EXTI15_8_IRQn);
#ifdef LP_ENABLE
    PWR_EnterSTOPMode(PWR_STOPEntry_WFI);
    //PWR_EnterSTANDBYMode();
#else
    __WFI();
#endif
    //EXTI_ClearITPendingBit(EXTI_Line0);
    //NVIC_EnableIRQ(EXTI7_0_IRQn);
#ifdef SLP_ENABLE
    TIM_RUN();
#endif
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

    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_8_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    return;
}

void EXTI15_8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI15_8_IRQHandler() {
    if(EXTI_GetITStatus(EXTI_Line14) != RESET) {
        printf("Exit SLP Mode\r\n");
        SystemInit();
        EXTI_ClearITPendingBit(EXTI_Line14);
        NVIC_DisableIRQ(EXTI15_8_IRQn);
    }
    if(EXTI_GetITStatus(EXTI_Line15) != RESET) {
        printf("Exit SLP Mode\r\n");
        SystemInit();
        EXTI_ClearITPendingBit(EXTI_Line15);
        NVIC_DisableIRQ(EXTI15_8_IRQn);
    }
    return;
}