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

// PWM output routine:
// TIM1_CH1(PD2)
// This example demonstrates that the TIM_CH1(PD2) pin outputs PWM in PWM mode 1
// and PWM mode 2.

// PWM Output Mode Definition
#define PWM_MODE1   0
#define PWM_MODE2   1

// PWM Output Mode Selection
#define PWM_MODE PWM_MODE1          // прямой ШИМ ( сколько импульс столько и вначале положительный фронт )
//#define PWM_MODE PWM_MODE2          // инверсный ШИМ ( сколько импульс столько будет в конце низкий уровень )


/* Global Variable */

//*************************************************************************************
// генерируем ШИМ сигнал на пине PD2 TIM1 таймером 1 ( по даташиту TI1CH1 таймер 1 канал 1 )
// генерируем ШИМ сигнал на пине PA1 TIM1 таймером 1 ( по даташиту TI1CH2 таймер 1 канал 2 )
void TIM1_PWMOut_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    // включаем тактирование таймера и порта D
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD |RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE );       // port D A

    // канал 1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                                           // pin 2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                                     // альтернативная функция
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );                                            // port D

    // канал 2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;                                           // pin 1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                                     // альтернативная функция
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure );                                            // port A

    // настройки для режима ШИМ
    TIM_TimeBaseInitStructure.TIM_Period = 100-1;                                       // счетчик - периуд ( разрядность ШИМ ( ставим на один меньше -1 )
    TIM_TimeBaseInitStructure.TIM_Prescaler = 48000-1;                                  // прескаллер таймера ( ставим на один меньше -1 )
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;                         // дополнительный сдвиг ( мертвое время )
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;                     // счет идет вверх
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

#if (PWM_MODE == PWM_MODE1)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                                     // прямой ШИМ ( сколько импульс столько и вначале положительный фронт )

#elif (PWM_MODE == PWM_MODE2)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                                   // инверсный ШИМ ( сколько импульс столько будет в конце низкий уровень )

#endif

    // канал 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 10;                                                 // пульс по умолчанию при старте ( скважность ШИМ )
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;                           // начальный фронт высокий или низкий
    TIM_OC1Init( TIM1, &TIM_OCInitStructure );

    // канал 2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 10;                                                 // пульс по умолчанию при старте ( скважность ШИМ )
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;                           // начальный фронт высокий или низкий
    TIM_OC2Init( TIM1, &TIM_OCInitStructure );

    // запускаем все что настроили
    TIM_CtrlPWMOutputs(TIM1, ENABLE );

    TIM_OC1PreloadConfig( TIM1, TIM_OCPreload_Disable );                                // канал 1
    TIM_OC2PreloadConfig( TIM1, TIM_OCPreload_Disable );                                // канал 2

    TIM_ARRPreloadConfig( TIM1, ENABLE );
    TIM_Cmd( TIM1, ENABLE );

    // если нужно прерывание то дописываем отдельно настройки и обработчик прерывания
}
//*****************************************************************************************


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


    TIM1_PWMOut_Init();

    // можем в любой момент отключить или включить ШИМ
    //TIM_CtrlPWMOutputs(TIM1, DISABLE);

    // установка нового значения импульса ( скважность ШИМ ) напрямую в регистр или через функцию (канал 1)
    //TIM1->CH1CVR = 20;  // TIM_SetCompare1(TIM1, 20);

    // установка нового значения импульса ( скважность ШИМ ) напрямую в регистр или через функцию (канал 2)
    //TIM1->CH2CVR = 20;  // TIM_SetCompare2(TIM1, 20);

    while(1)
    {

    }
}
