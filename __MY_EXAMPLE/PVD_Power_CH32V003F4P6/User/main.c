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

//*********************************************************************
void PVD_Power_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;               // pin 4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);                  // port C

    //Configure the PVD Level to 2.9V
    PWR_PVDLevelConfig(PWR_PVDLevel_3V1);
    //2.85V rising edge/2.7V falling edge.
    //3.05V rising edge/2.9V falling edge.
    //3.3V rising edge/3.15V falling edge.
    //3.5V rising edge/3.3V falling edge.
    //3.7V rising edge/3.5V falling edge.
    //3.9V rising edge/3.7V falling edge.
    //4.1V rising edge/3.9V falling edge.
    //4.4V rising edge/4.2V falling edge.

    PWR_PVDCmd(ENABLE);

    EXTI_ClearITPendingBit(EXTI_Line8);                         // External interrupt line 8 Connected to the PVD Output

    EXTI_InitStructure.EXTI_Line = EXTI_Line8;                  // External interrupt line 8 Connected to the PVD Output
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable the PVD Interrupt
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
//*********************************************************************

//*********************************************************************
// 抉忌把忘忌抉找折我抗 扭把快把抑志忘扶我攸:
// 快扼找抆 扭把快把抑志忘扶我攸 志扶快扮扶快快 抉找 0 忱抉 7 ( EXTI0-EXTI7 ) EXTI8->PVD(扭我找忘扶我快) EXTI9->auto-wakeup events
void PVD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))); // 批抗忘戒抑志忘快技 志快抗找抉把 扭把快把抑志忘扶我攸
void PVD_IRQHandler(void)
{
  // 抉扭把快忱快抖攸快技 扶忘 抗忘抗抉技 我技快扶扶抉 扭我扶快 扭把快把抑志忘扶我快 扼把忘忌抉找忘抖抉
  if(EXTI_GetITStatus(EXTI_Line8) != RESET){ // External interrupt line 8 Connected to the PVD Output

    GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_SET);

    printf("PVD at EXTI\r\n");

    EXTI_ClearITPendingBit(EXTI_Line8);     // Clear Flag
  }
}
//*********************************************************************


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);


    //我扶我扯我忘抖我戒我把批快技 扭把快把抑志忘扶我快
    PVD_Power_Init();


    GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_SET);
    Delay_Ms(1000);
    GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_RESET);


    while(1)
    {

    }
}
