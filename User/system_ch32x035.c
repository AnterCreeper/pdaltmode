/********************************** (C) COPYRIGHT *******************************
 * File Name          : system_ch32x035.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : CH32X035 Device Peripheral Access Layer System Source File.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include <ch32x035.h>
#include "system_ch32x035.h"

/* system_private_function_proto_types */
static void SetSysClock(void);

#ifdef SYSCLK_FREQ_8MHz_HSI
static void SetSysClockTo8_HSI(void);
#elif defined SYSCLK_FREQ_12MHz_HSI
static void SetSysClockTo12_HSI(void);
#elif defined SYSCLK_FREQ_16MHz_HSI
static void SetSysClockTo16_HSI(void);
#elif defined SYSCLK_FREQ_24MHz_HSI
static void SetSysClockTo24_HSI(void);
#elif defined SYSCLK_FREQ_48MHz_HSI
static void SetSysClockTo48_HSI(void);
#endif

/*********************************************************************
 * @fn      SystemInit
 *
 * @brief   Setup the microcontroller system Initialize the Embedded Flash Interface,
 *        update the SystemCoreClock variable.
 *
 * @return  none
 */
void SystemInit(void)
{
    RCC->CTLR |= (uint32_t)0x00000001;
    RCC->CFGR0 |= (uint32_t)0x00000050;
    RCC->CFGR0 &= (uint32_t)0xF8FFFF5F;
    SetSysClock();
}

/*********************************************************************
 * @fn      SetSysClock
 *
 * @brief   Configures the System clock frequency, HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClock(void)
{
//    GPIO_IPD_Unused();
#ifdef SYSCLK_FREQ_8MHz_HSI
    SetSysClockTo8_HSI();
#elif defined SYSCLK_FREQ_12MHz_HSI
    SetSysClockTo12_HSI();
#elif defined SYSCLK_FREQ_16MHz_HSI
    SetSysClockTo16_HSI();
#elif defined SYSCLK_FREQ_24MHz_HSI
    SetSysClockTo24_HSI();
#elif defined SYSCLK_FREQ_48MHz_HSI
    SetSysClockTo48_HSI();
#endif
}

#ifdef SYSCLK_FREQ_8MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo8_HSI
 *
 * @brief   Sets HSE as System clock source and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo8_HSI(void)
{
    /* Flash 2 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_2;

    /* HCLK = SYSCLK = APB1 */
    RCC->CFGR0 &= (uint32_t)0xFFFFFF0F;
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV6;

    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_0;
}

#elif defined SYSCLK_FREQ_12MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo12_HSI
 *
 * @brief   Sets System clock frequency to 12MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo12_HSI(void)
{
    /* Flash 2 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_2;

    /* HCLK = SYSCLK = APB1 */
    RCC->CFGR0 &= (uint32_t)0xFFFFFF0F;
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV4;

    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_0;
}

#elif defined SYSCLK_FREQ_16MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo16_HSI
 *
 * @brief   Sets System clock frequency to 16MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo16_HSI(void)
{
    /* Flash 2 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_2;

    /* HCLK = SYSCLK = APB1 */
    RCC->CFGR0 &= (uint32_t)0xFFFFFF0F;
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV3;

    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_1;
}

#elif defined SYSCLK_FREQ_24MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo24_HSI
 *
 * @brief   Sets System clock frequency to 24MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo24_HSI(void)
{
    /* Flash 2 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_2;

    /* HCLK = SYSCLK = APB1 */
    RCC->CFGR0 &= (uint32_t)0xFFFFFF0F;
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;

    /* Flash 1 wait state */
    FLASH->ACTLR = (uint32_t)FLASH_ACTLR_LATENCY_1;
}

#elif defined SYSCLK_FREQ_48MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo48_HSI
 *
 * @brief   Sets System clock frequency to 48MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo48_HSI(void)
{
    /* Flash 2 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_2;

    /* HCLK = SYSCLK = APB1 */
    RCC->CFGR0 &= (uint32_t)0xFFFFFF0F;
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
}

#endif