/********************************** (C) COPYRIGHT  *******************************
 * File Name          : debug.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : This file contains all the functions prototypes for UART
 *                      Printf , Delay functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <ch32v00x.h>
#include <stdio.h>

 //////////////////////////////////////////////////////////////////////
 void SysTick_Init(void);
 uint32_t GetTick();
 void Delay_Us(uint32_t time_us);   // §Þ§Ú§Ü§â§à§ã§Ö§Ü§å§ß§Õ§í 1 000 000
 void Delay_Ms(uint32_t time_ms);   // §Þ§Ú§Ý§Ý§Ú§ã§Ö§Ü§å§ß§Õ§í 1 000
 //////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H */
