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
//**************************************************************
      // в данном примере мы подключаем кнопку к пину GPIOD2 ( input of the compare capture channel (TIMx_CHx) ) к земле и включаем на данном пине внутренюю подтяжку к питанию ( или внешнюю ставим )
      // при нажатии на кнопку с параметром TIM_ICPolarity_Falling счетчик увеличиваеться на 1
      // как только достигает значения установленного счетчика TIM_SetAutoreload то отрабатывает флаг if(TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) != RESET)
      // в данной отработке мы етот флаг сбрасываем и начинаем заново считать
//**************************************************************

//*************************************************************************************************
// GPIOD2 увеличивает счетчик таймера ( input of the compare capture channel (TIMx_CHx) )
// таймер TIM1 и TIM2  тактируеться от шины HCLK ( в данном примере это 48МГц )
void TIM1_ETRClockMode1_Init(void)
{


    // включаем тактирование таймера и порта GPIOD2 через который и будем натикивать таймер
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOD, ENABLE);     // port D

    // делаем внутренюю подтяжку к питанию -----------------------------------------------
    // кнопка подключена к земле ( если делать внешнюю подтяжку то настройка пина не нужна
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;               // pin 2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);                  // port D
    //-----------------------------------------------------------

    // устанавливаем режим счет таймера вверх
    TIM_CounterModeConfig(TIM1, TIM_CounterMode_Up);

    // устанавливаем счетчик таймера ( до которого считать чтоб он переполнился )
    // счетчик ( таймер ) 16 битный
    TIM_SetAutoreload(TIM1, 0x3EB);     // 0x3EB = 1003

    // если нужно ставим предделитель для срабатывания if(TIM_GetFlagStatus(TIM1, TIM_FLAG_CC1) != RESET)
    // при TIM_ICPSC_DIV4 на каждом 4 нажатии кнопки будет взводиться флаг и будем попадать в обработчик
    //TIM_SetIC1Prescaler(TIM1, TIM_ICPSC_DIV4);

    // настройка внешнего тактирования ( от пина GPIOD2 )
    // первый параметр номер таймера
    // второй параметр предделитель сигнала которій генерируем GPIOD2
    // третий параметр полярность сигнала ( высокий или низкий )
    // последнее занчение фильтр ( параметр от 0x0 до 0xF )
    TIM_ETRClockMode1Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_Inverted, 0x0);

    // GPIOD2 Input as TIM Clock Source
    // первый параметр номер таймера
    // второй параметр внешнее тактирование 1 таймера
    // третий параметр сигнал срабатывания
    // последнее занчение фильтр ( параметр от 0x0 до 0xF ) ( для борьбы с дресбергом ставим максимум )
    TIM_TIxExternalClockConfig(TIM1, TIM_TIxExternalCLK1Source_TI1, TIM_ICPolarity_Falling, 0xf);

    // запускаем таймер
    TIM_Cmd(TIM1, ENABLE);
}
//*************************************************************************************************

//*************************************************************************************************
// пин GPIOC5 по даташиту T1ETR
void TIM1_ETRClockMode2_Init(void)
{
    // включаем тактирование таймера и порта GPIOC5 через который и будем натикивать таймер
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOC, ENABLE); // port C

    // делаем внутренюю подтяжку к питанию -----------------------------------------------
    // кнопка подключена к земле ( если делать внешнюю подтяжку то настройка пина не нужна
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;               // pin 5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);                  // port C
    //-----------------------------------------------------------

    // устанавливаем режим счет таймера вверх
    TIM_CounterModeConfig(TIM1, TIM_CounterMode_Up);


    // устанавливаем счетчик таймера ( до которого считать чтоб он переполнился )
    // счетчик ( таймер ) 16 битный
    TIM_SetAutoreload(TIM1, 0x3EB);     // 0x3EB = 1003


    // GPIOC5 Input as TIM Clock Source
    TIM_ETRClockMode2Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_Inverted, 0x0);

    TIM_SelectInputTrigger(TIM1, TIM_TS_ETRF); // TIM_TS_ETRF - External Trigger input.
    TIM_Cmd(TIM1, ENABLE);
}
//*************************************************************************************************


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

    // TIM1_ETRClockMode1_Init();
    TIM1_ETRClockMode2_Init();

    while(1)
    {
        // печатаем текущее значение таймера
        uint32_t count = TIM_GetCounter(TIM1);
        printf("Count = %d\r\n", count);

        // сюда попадаем в момент нажатия на кнопку ( в конце сбрасываем флаг )
        // ( input of the compare capture channel (TIMx_CHx) )
        if(TIM_GetFlagStatus(TIM1, TIM_FLAG_CC1) != RESET)  //  TIM_FLAG_CC1 - TIM Capture Compare 1 Flag
        {
            printf("TIM Capture Compare \r\n");

            TIM_ClearFlag(TIM1, TIM_FLAG_CC1);  // сбрасываем флаг
            Delay_Ms(500);
        }

        // данный флаг отработает при переполнении счетчика ( потом его нужно руками сбросить )
        if(TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) != RESET)
        {
            uint32_t count_update = TIM_GetCounter(TIM1);
            printf("Count Update ---> %d\r\n", count_update);

            // сбрасываем флаг переполнения счетчика
            TIM_ClearFlag(TIM1, TIM_FLAG_Update);
        }
    }
}
