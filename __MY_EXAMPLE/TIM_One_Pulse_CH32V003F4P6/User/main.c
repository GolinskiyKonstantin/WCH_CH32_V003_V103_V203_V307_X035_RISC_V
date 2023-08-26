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

//*****************************************************************
//    Процедура одиночного импульсного выхода:
//    TIM2_CH1(PD4) TIM2_CH2(PD3)
//    Эта процедура демонстрирует, что в одноимпульсном режиме вывод TIM2_CH2(PD3) обнаруживает спадающий фронт, затем
//    TIM2_CH1(PD4) выдает положительный импульс.

void One_Pulse_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_ICInitTypeDef TIM_ICInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE );     // port D
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

    // TIM2_CH1(PD4) порт ETR по даташиту ( пин выдает однократный ШИМ сигнал ( импульс )
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;                   // pin 4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );                    // port D

    // TIM2_CH2(PD3) по даташиту ( принимает сигнал кнопка подключенная к земле для примера)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                   // pin 3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init( GPIOD, &GPIO_InitStructure);                     // port D

    // нвстройки таймера предделитель счетчик и т.д.
    TIM_TimeBaseInitStructure.TIM_Period = 200-1;               // счетчик ( разрядность ШИМ )
    TIM_TimeBaseInitStructure.TIM_Prescaler = 48000-1;          // 48 000 000 / 48 000  = 1 000 pulse sec
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 50;                         // длина импульса ( значение разрядности ШИМ 200 - 50 получим длину импульса 150 ( импульсов ) миллисекунд )
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init( TIM2, &TIM_OCInitStructure );

    TIM_ICStructInit( &TIM_ICInitStructure );
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;

    TIM_ICInit( TIM2, &TIM_ICInitStructure );
    TIM_SelectOnePulseMode( TIM2,TIM_OPMode_Single );
    TIM_SelectInputTrigger( TIM2, TIM_TS_TI2FP2 );
    TIM_SelectSlaveMode( TIM2, TIM_SlaveMode_Trigger );
}
//**********************************************************************************

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

    One_Pulse_Init();

    while(1)
    {


    }
}
