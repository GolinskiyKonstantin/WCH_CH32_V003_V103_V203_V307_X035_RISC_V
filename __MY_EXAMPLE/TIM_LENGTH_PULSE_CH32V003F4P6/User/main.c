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
volatile uint32_t timer_up = 0;
volatile uint32_t timer_up_flag = 1;

/* Global Variable */
//*****************************************************************************************************
// сигнал на пине TIM1_CH1(PD2) ( по даташиту ) считает время импульса и времі между импульсами
// у меня начинает счет при сигнале спадающем ( от питания тянет к земле ) если нужно наоборот то в настройках поменять
// TIM_ICPolarity_Falling и поменять подтяжку GPIO_Mode_IPU
// результат замера и момент переходов импульсов обрабатіваються в прерывании
// так как настроили таймер что он считает (прескаллер 48000-1 от частоті 48 000 000 ( получаем 1 тик таймера = 1 миллисекунде ))
// и счетчик максимальный 65535 то максимум можно замерить длину импульса 65 секунд
// в прерывании по переполнению считаеться сколько раз таймер переполнился что позволяет считать любое время
void Input_Capture_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    TIM_ICInitTypeDef       TIM_ICInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    NVIC_InitTypeDef        NVIC_InitStructure = {0};

    // включаем тактирования таймера и пина PD2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                   // включаем внутренюю подтяжку к питанию
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;                  // ставим максимальное значение 65535
    TIM_TimeBaseInitStructure.TIM_Prescaler = 48000 - 1;            // прескаллер 48000-1 от частоті 48 000 000 ( получаем 1 тик таймера = 1 миллисекунде )
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;

    TIM_PWMIConfig(TIM1, &TIM_ICInitStructure);

    // прерывание на режим захвата
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // прерывание для переполнения таймера если больше 65 секунд
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // включаем режим прерывание по захвату и на переполнения таймера
    TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update, ENABLE);

    TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
    TIM_Cmd(TIM1, ENABLE);
}
//********************************************************************************************

//********************************************************************************************
// прерывание на режим захвата
void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_CC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM1, TIM_IT_CC1 ) != RESET )
    {
        // печать времени между конца прошлого сигнала и начала нового
        printf( "CH1_Val:%u\r\n", TIM_GetCapture1( TIM1 ) + ( timer_up * 0xFFFFU ) );

        TIM_SetCounter( TIM1, 0 );

        timer_up_flag = 1;
    }

    if( TIM_GetITStatus( TIM1, TIM_IT_CC2 ) != RESET )
    {
        // печать времени между началом сигнала и конца сигнала
        printf( "CH2_Val:%u\r\n", TIM_GetCapture2( TIM1 ) + ( timer_up * 0xFFFFU ) );

        timer_up_flag = 1;
    }

    TIM_ClearITPendingBit( TIM1, TIM_IT_CC1 | TIM_IT_CC2 );
}
//********************************************************************************************

//********************************************************************************************
// прерывание для переполнения таймера если больше 65 секунд
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM1, TIM_IT_Update ) != RESET )
    {
        if( timer_up_flag == 1){
            timer_up_flag = 0;
            timer_up = 0;
        }
        else{
           timer_up++;
        }
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

    Input_Capture_Init(0xFFFF, 48000 - 1);

    while(1)
    {

    }
}
