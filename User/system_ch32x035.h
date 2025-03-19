/********************************** (C) COPYRIGHT *******************************
 * File Name          : system_ch32x035.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : CH32X035 Device Peripheral Access Layer System Header File.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __SYSTEM_CH32X035_H
#define __SYSTEM_CH32X035_H

#ifdef __cplusplus
extern "C" {
#endif 

/* 
* Uncomment the line corresponding to the desired System clock (SYSCLK) frequency (after 
* reset the HSI is used as SYSCLK source).
*/
//#define SYSCLK_FREQ_8MHz_HSI   8000000
//#define SYSCLK_FREQ_12MHz_HSI  12000000
//#define SYSCLK_FREQ_16MHz_HSI  16000000
#define SYSCLK_FREQ_24MHz_HSI  24000000
//#define SYSCLK_FREQ_48MHz_HSI  HSI_VALUE

#ifdef SYSCLK_FREQ_8MHz_HSI
#define SystemCoreClock SYSCLK_FREQ_8MHz_HSI
#elif defined SYSCLK_FREQ_12MHz_HSI
#define SystemCoreClock SYSCLK_FREQ_12MHz_HSI
#elif defined SYSCLK_FREQ_16MHz_HSI
#define SystemCoreClock SYSCLK_FREQ_16MHz_HSI
#elif defined SYSCLK_FREQ_24MHz_HSI
#define SystemCoreClock SYSCLK_FREQ_24MHz_HSI
#elif defined SYSCLK_FREQ_48MHz_HSI
#define SystemCoreClock SYSCLK_FREQ_48MHz_HSI
#endif

/* System_Exported_Functions */  
extern void SystemInit(void);

#ifdef __cplusplus
}
#endif

#endif