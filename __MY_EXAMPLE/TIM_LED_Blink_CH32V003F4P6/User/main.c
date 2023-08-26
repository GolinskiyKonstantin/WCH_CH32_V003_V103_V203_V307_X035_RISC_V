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


/* Global define */


/* Global Variable */

// используем таймер 1 ( TIM1 )
void TIM_Setup_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
    NVIC_InitTypeDef        NVIC_InitStructure = {0};

    // включаем тактирование таймера TIM1
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );

    // настройки для таймера ( частота шины APB2 48МГц ) при 1000 периуд и 48000 предделитель
    // Тайчер тикает 1 раз в 1 секунду ( 48 000 000 / 48 000 = 1 000 раз в секунду ) периуд 1000 получаем 1 секунда
    TIM_TimeBaseInitStructure.TIM_Period = 1000-1;                                      // счетчик - периуд ( ставим на один меньше -1 )
    TIM_TimeBaseInitStructure.TIM_Prescaler = 48000-1;                                  // прескаллер таймера ( ставим на один меньше -1 )
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;                         // дополнительный сдвиг ( мертвое время )
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;                     // счет идет вверх
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    // устанавливаем счетчик таймера ( до которого считать чтоб он переполнился )
    // счетчик ( таймер ) 16 битный
    //TIM_SetAutoreload(TIM1, 1000-1);            // если TIM_TimeBaseInitStructure.TIM_Period нужно поменять гдето в коде

    // прерывание для таймера
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // включаем режим прерывание по переполнения таймера
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    TIM_Cmd( TIM1, ENABLE );
}

//********************************************************************************************
// прерывание для переполнения таймера если больше 65 секунд
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM1, TIM_IT_Update ) != RESET )
    {
        printf("TIM1 Update !!!  \r\n");
    }

    TIM_ClearITPendingBit( TIM1, TIM_IT_Update );
}
//********************************************************************************************

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


    TIM_Setup_Init();


    while(1)
    {


    }
}
