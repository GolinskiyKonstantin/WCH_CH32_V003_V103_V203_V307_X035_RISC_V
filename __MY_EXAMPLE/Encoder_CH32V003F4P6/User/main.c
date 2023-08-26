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

volatile int32_t encoder_count = 0;

//***************************************************************************
// энкодер подключен к таймеру 1 TIM1 к каналу 1 и 2  T1CH2 (pin PA1)   T1CH1 (pin PD2)
void TIM1_EncoderMode_Init(void)
{
    // включаем тактирование таймера и порта GPIOD2 через который и будем натикивать таймер
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA, ENABLE);     // port D port A

    // делаем внутренюю подтяжку к питанию -----------------------------------------------
    // кнопка подключена к земле ( если делать внешнюю подтяжку то настройка пина не нужна
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;               // pin 2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           // pull up
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);                  // port D
    //-----------------------------------------------------------

    // делаем внутренюю подтяжку к питанию -----------------------------------------------
    // кнопка подключена к земле ( если делать внешнюю подтяжку то настройка пина не нужна
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;               // pin 1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           // pull up
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  // port A
    //-----------------------------------------------------------

    //-----------------------------------------------------------
    // фильтр таймера
    uint16_t TIM_Filter = 0x0F; // значение от 0x00 до 0x0F
    TIM1->CHCTLR1 |= (uint16_t)(TIM_Filter << (uint16_t)4);
    //-----------------------------------------------------------

    // устанавливаем счетчик таймера ( до которого считать чтоб он переполнился )
    // счетчик ( таймер ) 16 битный
    TIM_SetAutoreload(TIM1, 4 - 1);     // 4 ( так как 4 счелчка на один шаг ставим 4 не забываем отнять 1 ( 4 - 1 ) )

    // настраиваем режим энкодера ( указываем направление счета TI1  TI2 TI12 и уровни срабатывания энкодера )
    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);

    //-----------------------------------------------------------
    // прерывание для переполнения таймера ( считаем в прерывании  )
    NVIC_InitTypeDef        NVIC_InitStructure = {0};

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    //-----------------------------------------------------------

    // запускаем таймер
    TIM_Cmd(TIM1, ENABLE);
}
//*************************************************************************************************

//********************************************************************************************
// прерывание для переполнения таймера
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void)
{
    //--------------------------------------------------------------
    // для механического энкодера ( чтобы небыло дребезга )
    // останавливаем таймер
    TIM_Cmd(TIM1, DISABLE);
    //--------------------------------------------------------------

    if( TIM_GetITStatus( TIM1, TIM_IT_Update ) != RESET )
    {
        // данный регистр TIM1->CTLR1 & TIM_DIR содержит направление таймера
        // если там 0 то таймер счелкает вперед
        // если не 0 ( 16 ) то таймер счелкает назад
        if( TIM1->CTLR1 & TIM_DIR ){
            //printf("count: %d %s\r\n ", TIM_GetCounter(TIM1), "-------" );
            encoder_count--;
        }
        else{
            //printf("count: %d %s\r\n ", TIM_GetCounter(TIM1), "+++++++" );
            encoder_count++;
        }
        printf("count: %d -> %d\r\n ", encoder_count, (TIM1->CTLR1 & TIM_DIR) );
    }

    TIM_ClearITPendingBit( TIM1, TIM_IT_Update );

    //--------------------------------------------------------------
    // для механического энкодера ( чтобы небыло дребезга )
    // запускаем таймер
    TIM_Cmd(TIM1, ENABLE);
    //--------------------------------------------------------------
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

    TIM1_EncoderMode_Init();

    while(1)
    {

        //-------------------------------------------------------------
//        // данный регистр TIM1->CTLR1 & TIM_DIR содержит направление таймера
//        // если там 0 то таймер счелкает вперед
//        // если не 0 ( 16 ) то таймер счелкает назад
//        if( TIM1->CTLR1 & TIM_DIR ){
//            printf("count: %d %s\r\n ", TIM_GetCounter(TIM1), "-------" );
//            encoder_count--;
//        }
//        else{
//            printf("count: %d %s\r\n ", TIM_GetCounter(TIM1), "+++++++" );
//            encoder_count++;
//        }
        //-------------------------------------------------------------

    }
}
