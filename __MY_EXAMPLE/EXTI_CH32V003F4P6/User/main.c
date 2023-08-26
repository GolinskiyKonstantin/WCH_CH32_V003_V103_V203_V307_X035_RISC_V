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

////////////////////////////////////////////////////////////////////
//*********************************************************************
// настраиваем прерывание на 5 пине порта C ( C5 )
void EXTI5_INT_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE); // port C

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;       // pin 5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // pull-up
    GPIO_Init(GPIOC, &GPIO_InitStructure);          // port C

    // GPIODC ----> EXTI_Line5
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5); // port C  pin 5
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;      // pin 5
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // Falling
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;               // запуск прерывания при старте
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //Priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         // запуск прерывания при старте ( можно запустить отдельно NVIC_EnableIRQ(EXTI7_0_IRQn); )
    NVIC_Init(&NVIC_InitStructure);
}
//*********************************************************************

//*********************************************************************
// обработчик прерывания:
// есть прерывания внешнее от 0 до 7 ( EXTI0-EXTI7 ) EXTI8->PVD(питание) EXTI9->auto-wakeup events
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))); // указываем вектор прерывания
void EXTI7_0_IRQHandler(void)
{
  // определяем на каком именно пине прерывание сработало
  if(EXTI_GetITStatus(EXTI_Line5) != RESET){ // pin 5
    printf("Run at EXTI\r\n");
    EXTI_ClearITPendingBit(EXTI_Line5);     // Clear Flag // pin 5
  }
}
//*********************************************************************

////////////////////////////////////////////////////////////////////


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
    SysTick_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);

    //инициализируем прерывание
    EXTI5_INT_INIT();

    while(1)
    {


    }
}
