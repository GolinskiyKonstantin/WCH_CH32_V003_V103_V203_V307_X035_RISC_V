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

#define INT_EVENT           0
#define INT_INTERRUPT       1

/*
        Система поддерживает 2 режима низкого энергопотребления, которые можно выбрать для низкого энергопотребления, короткого запуска.
        время и несколько событий пробуждения для достижения наилучшего баланса.

        1) Спящий режим Sleep mode ( WFI - Any interrupt ) ( WFE - Wake-up event )
        В спящем режиме останавливаются только часы ЦП, но все периферийные часы работают нормально, и
        периферия в рабочем состоянии. Этот режим является самым неглубоким режимом с низким энергопотреблением, но это самый быстрый режим для
        разбудить систему.
        Условие выхода: любое прерывание или событие пробуждения.
        ( потребление зависит от питания 3.3 или 5 вольт а также от частоты и ( внутрений или внешний ) генератор
        Например:
        питание 3.3В 48МГц внутрений HSI потребление ( с включенной перефирией 4,2 мА ) ( с выключенной перефирией 1,8 мА )
        питание 3.3В 750КГц внутрений HSI потребление ( с включенной перефирией 0,4 мА ) ( с выключенной перефирией 0,4 мА )
        питание 5В 48МГц внутрений HSI потребление ( с включенной перефирией 4,2 мА ) ( с выключенной перефирией 1,8 мА )
        питание 5В 750КГц внутрений HSI потребление ( с включенной перефирией 0,4 мА ) ( с выключенной перефирией 0,4 мА )
        с внешним кварцевый генератор HSE потребление выше

        2) Режим ожидания Standby mode ( AWU auto-wakeup )
        Биты PDDS и SLEEPDEEP установлены, и для входа выполняется инструкция WFI/WFE. Сила
        питание основной части отключено, а RC-генератор HSI и кварцевый генератор HSE также отключены.
        выключен, и в этом режиме можно достичь наименьшего энергопотребления.
        Условия выхода: любое внешнее прерывание/событие (сигнал EXTI), внешний сигнал сброса на NRST, сброс IWDG,
        где сигнал EXTI включает в себя один из 18 внешних портов ввода/вывода, выход PVD, автопробуждение AWU auto-wakeup.
        потребление зависит от питания 3,3 или 5 вольт и от включен ли внутрений нискочастотный часовой резонатор или нет:
        LSI включен 3,3В - 10,5 мкА     RCC_LSICmd(ENABLE);     while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
        LSI включен 5В - 11,1 мкА       RCC_LSICmd(ENABLE);     while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
        LSI выключен 3,3В - 9 мкА       RCC_LSICmd(DISABLE);    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == SET);
        LSI выключен 5В - 9,6 мкА       RCC_LSICmd(DISABLE);    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == SET);

*/

/* Global define */


/* Global Variable */

//    В данном примере Спящий режим Sleep mode ( WFI - Any interrupt ) ( WFE - Wake-up event )
//
//    прерывание EXTI_Line0(PD0), вход PD0
//    низкий уровень запускает внешнее прерывание EXTI_Line0 для выхода из спящего режима,
//    Выполнение программы продолжается после пробуждения.


//********************************************************************************************
void EXTI0_INT_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};

#if INT_INTERRUPT
    NVIC_InitTypeDef NVIC_InitStructure = {0};
#endif

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;               // pin 0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           // input pull up
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);                  // port D

    /* GPIOD.0 ----> EXTI_Line0 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
#if INT_EVENT
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
#elif INT_INTERRUPT
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
#endif
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

#if INT_INTERRUPT
    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}
//********************************************************************************************

//********************************************************************************************
#if INT_INTERRUPT
    void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
    void EXTI7_0_IRQHandler(void)
    {
      // проснемся в любом случае даже если не указанный пин отработал ( а любой EXTI7_0 )
      if(EXTI_GetITStatus(EXTI_Line0)!=RESET)   // проверяем что сработал именно наш пин
      {
        printf("EXTI0 Wake_up\r\n");

        // при пробуждении обратно включаем системный таймер так как его выключили перед сном
        NVIC_EnableIRQ(SysTicK_IRQn);
        EXTI_ClearITPendingBit(EXTI_Line0);     // Clear Flag
      }
    }
#endif
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

    SysTick_Init();

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);

    EXTI0_INT_INIT();

    printf("Sleep Mode Test\r\n");
    printf("Sleep..........\r\n");

    // выключаем все ненужные прерывания

    // обязательно отключаем ( если включено ) прерывание системного таймера
    // так как это тоже прерывание и оно пробудит МК
    NVIC_DisableIRQ(SysTicK_IRQn);

#if INT_INTERRUPT
    __WFI();  // ( WFI - Any interrupt - for Interrupt)
#elif INT_EVENT
    __WFE(); // ( WFE - AWU auto-wakeup event - for Events)
#endif

    printf("\r\n RUN < Sleep mode > \r\n");

    while(1)
    {
        Delay_Ms(1000);
        printf("Run in main\r\n");
    }
}
