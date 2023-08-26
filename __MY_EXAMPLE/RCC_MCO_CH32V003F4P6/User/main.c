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


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{

    //------------------------------------------------------------------------
    // понижаем частоту общей шины от которой тактируеться переферия ( к МСО не относиться так как она тактируеться до предделителя )
    // этот пределлитель на шине AHB основная шина для почти всей переферии  HCLK  PCLK1  PCLK2
    // при этом тактирование системной шины остается указаной дефайнами    SYSCLK = 48МГц
    // ( делим частоту системной шины например 48МГц на указонный делитель )
    //    *            RCC_SYSCLK_Div1 - AHB clock = SYSCLK.
    //    *            RCC_SYSCLK_Div2 - AHB clock = SYSCLK/2.
    //    *            RCC_SYSCLK_Div3 - AHB clock = SYSCLK/3.
    //    *            RCC_SYSCLK_Div4 - AHB clock = SYSCLK/4.
    //    *            RCC_SYSCLK_Div5 - AHB clock = SYSCLK/5.
    //    *            RCC_SYSCLK_Div6 - AHB clock = SYSCLK/6.
    //    *            RCC_SYSCLK_Div7 - AHB clock = SYSCLK/7.
    //    *            RCC_SYSCLK_Div8 - AHB clock = SYSCLK/8.
    //    *            RCC_SYSCLK_Div16 - AHB clock = SYSCLK/16.
    //    *            RCC_SYSCLK_Div32 - AHB clock = SYSCLK/32.
    //    *            RCC_SYSCLK_Div64 - AHB clock = SYSCLK/64.
    //    *            RCC_SYSCLK_Div128 - AHB clock = SYSCLK/128.
    //    *            RCC_SYSCLK_Div256 - AHB clock = SYSCLK/256.

    //RCC_HCLKConfig(RCC_SYSCLK_Div2);    // поделили на 2 получаем 48 000 000 Гц / 2 =  24 000 000 Гц
    //RCC_HCLKConfig(RCC_SYSCLK_Div16);    // поделили на 256 получаем 48 000 000 Гц / 16 =  3 000 000 Гц

    // на сильно малых частотах на плохом питании может зависнуть или в юарт будут отправляться битые данные
    // также после зависания не даст перепрошиться для етого в настройках среды где флеш ставим выключение питания и потом прошиваем
    //------------------------------------------------------------------------


    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    SysTick_Init();

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);

    GPIO_InitTypeDef  GPIO_InitStructure={0};
    RCC_ClocksTypeDef RCC_ClocksStatus={0};

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    USART_Printf_Init(115200);
    SystemCoreClockUpdate();
    printf("SystemClk:%ld\r\n",SystemCoreClock);

    // просто смотрим в консоли частоты шинн ( в Герцах )
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    printf("SYSCLK_Frequency-%ld\r\n",  RCC_ClocksStatus.SYSCLK_Frequency);
    printf("HCLK_Frequency-%ld\r\n",    RCC_ClocksStatus.HCLK_Frequency);
    printf("PCLK1_Frequency-%ld\r\n",   RCC_ClocksStatus.PCLK1_Frequency);
    printf("PCLK2_Frequency-%ld\r\n",   RCC_ClocksStatus.PCLK2_Frequency);

    // так как вывод МСО по даташиту находиться на пине MCO Output GPIOC 4
    // то включаем тактирование порта С пина 4
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;               // pin 4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);                  // port C

    // включаем генерацию на внешний пин нужную нам частоту ------------
    // MCO Output GPIOC 4

//         RCC_MCO_NoClock - No clock selected.
//         RCC_MCO_SYSCLK - System clock selected.
//         RCC_MCO_HSI - HSI oscillator clock selected.
//         RCC_MCO_HSE - HSE oscillator clock selected.
//         RCC_MCO_PLLCLK - PLL clock selected.

    // 48МГц частота шины тактирования
    RCC_MCOConfig( RCC_MCO_SYSCLK );

    // частота внутренего резонатора 24МГц смотри по даташиту
    // выдает стабильно и работает независимо от внутренего или внешнего резонатора тактируемся
    //RCC_MCOConfig( RCC_MCO_HSI );


    // выдает частоту (24МГц) которая на кварцевом резонаторе ( внешнем например 24МГц стоит на плате при 48МГц общей шины)
    // по даташиту кварц можно ставить от 4 до 25 МГц
    // ( при использовании внутренего тактирования тут нечего не выдает )
    // частоту выдает стабильно
    //RCC_MCOConfig( RCC_MCO_HSE );

    // 48МГц ( частота что при внешнем что при внутренем резонаторе не стабильна )
    //RCC_MCOConfig( RCC_MCO_PLLCLK );

    //------------------------------------------------------------------

    while(1)
    {


    }
}
