/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 Multiprocessor communication mode routine:
 Master:USART1_Tx(PD5)\USART1_Rx(PD6).
 This routine demonstrates that USART1 receives the data sent by CH341 and inverts
 it and sends it (baud rate 115200).

 Hardware connection:PD5 -- Rx
                     PD6 -- Tx

*/

#include "debug.h"

// UID серийный номер МК
// расположен в трех регистрах по 32 бита ( всего 96 бит )

// реферанс мануал Table 15-1 ESIG-related registers list
// R32_ESIG_UNIID1 0x1FFFF7E8 UID register 1 0xXXXXXXXX
// R32_ESIG_UNIID2 0x1FFFF7EC UID register 2 0xXXXXXXXX
// R32_ESIG_UNIID3 0x1FFFF7F0 UID register 3 0xXXXXXXXX

// реферанс мануал Table 15-1 ESIG-related registers list
// также в МК храниться регистр в котором зашит размер флеш памяти 1 регистр 16 бит
// R16_ESIG_FLACAP 0x1FFFF7E0 Flash capacity register 0xXXXX


/* Global define */


/* Global Variable */

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);

    uint32_t UID_1 = *((volatile uint32_t*)0x1FFFF7E8);
    uint32_t UID_2 = *((volatile uint32_t*)0x1FFFF7EC);
    uint32_t UID_3 = *((volatile uint32_t*)0x1FFFF7F0);

    uint16_t FLASH_SIZE = *((volatile uint32_t*)0x1FFFF7E0);

    printf("UID = %04X-%04X-%04X \r\n", UID_1, UID_2, UID_3);

    printf("FLASH SIZE = %d KB \r\n", FLASH_SIZE);

    while(1)
    {


    }
}
