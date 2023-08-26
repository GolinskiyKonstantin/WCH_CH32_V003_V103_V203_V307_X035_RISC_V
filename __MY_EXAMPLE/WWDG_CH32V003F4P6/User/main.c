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

//**********************************************************************

#define WWDG_CNT    0X7F

//**********************************************************************



/* Global Variable */

//**********************************************************************
// настраиваем прерывание на сторожевой таймер
// ( если чтото нужно делать в обработчике прерывания перед тем как произойдет сброс МК )
static void WWDG_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
//**********************************************************************

//*********************************************************************

// конфигурируем сторожевой таймер
// тактирование WWDG идет от шины HCLK / 4096 ( деление будет всегда на 4096 по даташиту )
// в данном примере системная шина 48МГц предделитель шины AHB равен 1 ( так как не включали другое )
// то получаеться шина HCLK равна 48МГц

// *     WWDG_Prescaler_1
// *     WWDG_Prescaler_2
// *     WWDG_Prescaler_4
// *     WWDG_Prescaler_8

// в данном примере прерывание перегрузит Мк если дерним функцию раньше чем WWDG_SetCounter(0x7f); или поже чем WWDG_SetWindowValue(0x5f); дергать нужно между ними
void WWDG_Config(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

    WWDG_SetCounter(0x7f);                          // The value of the decrement counter диаппазон (0x40 ~ 0x7f) ( 64 - 127 )
    WWDG_SetPrescaler(WWDG_Prescaler_8);            // устанавливаем предделитель WWDG 1 2 4 8 ( для расчетов  результат делим на /4096 ) описано выше почему.
    WWDG_SetWindowValue(0x5f);                      // Window value значение должно быть ниже 0x80 ( 128 ) и меньше чем WWDG_SetCounter
    WWDG_Enable(WWDG_CNT);
    WWDG_ClearFlag();

    // инициализируем прерывание и включаем прерывание
    // ( если чтото нужно делать в обработчике прерывания перед тем как произойдет сброс МК )
    WWDG_NVIC_Config();
    WWDG_EnableIT();
}
//*********************************************************************

//*********************************************************************
// обработчик прерывания для WWDG перед сбросом МК он его дергает можно чтото на последок сделать
void WWDG_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))); // указываем вектор прерывания
void WWDG_IRQHandler(void)
{
    printf("WWDG RESET !!!!!!!! \r\n");
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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);


    WWDG_Config(); // частота счета 48M/8/4096 в данной конфинурации получаеться ( примерно 1465 тиков в секунду )
    // в настройках выше указан промежуток между 95 тиков и 127 тиков ( нужно дергать между ними например 110-64 тикоов )
    // от нужного значения отнимаем 64 так как счет начинаеться не с 1 а с 64 ( в данном примере получаеться 46
    // 1465 / 1000 = 1,46 тиков за 1 миллисекунду
    // 46 / 1,46 = 31,5 миллисекунд

    // либо считаем минимум и максимум в миллисекундах
    // для 95 тиков ( как установлено в данном примере ) 95 - 64 = 31 тик / 1,46 = 21,23 миллисекунд
    // для 127 тиков ( как установлено в данном примере ) 127 - 64 = 63 тик / 1,46 = 43,15 миллисекунд
    // нужно дергать между 21,23 миллисекунд  и 43,15 миллисекунд

    while(1)
    {
        Delay_Ms(31);
        WWDG_SetCounter(WWDG_CNT);
        printf("WWDG feed\r\n");
    }
}
