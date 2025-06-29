#ifndef __SYS_H
#define __SYS_H

#include "debug.h"

#define TARGET_PROJECT  "pdaltmode"
#define TARGET_VERSION  "0.2a"
#define TARGET_AUTHOR   "Anter"

#ifdef __clang
#define TARGET_COMPILER "clang"
#else
#ifdef __GNUC__
#define TARGET_COMPILER "gcc"
#else
#define TARGET_COMPILER "Unknown"
#endif
#endif

#define SLP_ENABLE  //System Sleep Mode Enable
#ifdef SLP_ENABLE
//#define LP_ENABLE   //System Low Power Mode Enable, need comment for debug
#endif

#define ESIG ((ESIG_Typedef *)(uint32_t)0x1FFFF7E0)

typedef struct {
    uint16_t FLACAP;
    uint16_t Reserved0;
    uint32_t Reserved1;
    uint32_t UNIID1;
    uint32_t UNIID2;
    uint32_t UNIID3;
} ESIG_Typedef;

void __assert(const char* str, uint32_t line);
#define assert(x) __assert(x, __LINE__)

void SYS_INIT();
void SYS_SLP();
void EXTI_INIT();

#endif