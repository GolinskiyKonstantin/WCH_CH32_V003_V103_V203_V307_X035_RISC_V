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

//------------------------------------------------------------------
// Этот пример демонстрирует использование DMA для вывода ШИМ через вывод TIM1_CH1 (PD2).
//------------------------------------------------------------------

/* Global define */

// PWM Output Mode Definition
#define PWM_MODE1   0
#define PWM_MODE2   1

// PWM Output Mode Selection
#define PWM_MODE PWM_MODE1          // прямой ШИМ ( сколько импульс столько и вначале положительный фронт )
//#define PWM_MODE PWM_MODE2          // инверсный ШИМ ( сколько импульс столько будет в конце низкий уровень )



//------------------------------------------------------------------
// CH1CVR register Definition
// адресс берем из реферанс мануал таблица Table 10-3 TIM1-related registers list
// R16_TIM1_CH1CVR 0x40012C34 Compare/capture register 1 0x0000
#define TIM1_CH1CVR_ADDRESS    0x40012C34
//------------------------------------------------------------------

/* Global Variable */

//------------------------------------------------------------------
// данные которые будем отправлять
uint16_t pbuf[3] = {10, 50, 80};
//------------------------------------------------------------------

//**************************************************************************
/*********************************************************************
 * @fn      TIM1_PWMOut_Init
 *
 * @brief   Initializes TIM1 PWM output.
 *
 * @param   arr - the period value.
 *          psc - the prescaler value.
 *          ccp - the pulse value.
 *
 * @return  none
 */
void TIM1_PWMOut_Init(uint16_t arr, uint16_t psc, uint16_t ccp)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    TIM_OCInitTypeDef       TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    // включаем тактирования таймера и порта D TIM1_CH1 (PD2)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1, ENABLE);         // port D

    // настраиваем пин
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                                           // pin 2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);                                              // port D

    // настраиваем таймер
    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    // настраиваем режим ШИМ
    #if (PWM_MODE == PWM_MODE1)
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                                     // прямой ШИМ ( сколько импульс столько и вначале положительный фронт )

    #elif (PWM_MODE == PWM_MODE2)
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                                   // инверсный ШИМ ( сколько импульс столько будет в конце низкий уровень )

    #endif

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);												// TIM_OC1.2.3.4 ..... номер канала

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);										// TIM_OC1.2.3.4 ..... номер канала
    TIM_ARRPreloadConfig(TIM1, ENABLE);
}
//**************************************************************************

//**************************************************************************
/*********************************************************************
 * @fn      TIM1_DMA_Init
 *
 * @brief   Initializes the TIM DMAy Channelx configuration.
 *
 * @param   DMA_CHx -
 *            x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void TIM1_DMA_Init(DMA_Channel_TypeDef *DMA_CHx, uint32_t ppadr, uint32_t memadr, uint16_t bufsize)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    // включаем тактирование DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // настраиваем DMA
    // чтобы указать нужный канал DMA TIM1_UP( в данном примере DMA1_Channel5 )
    // смотри в референс мануал диаграмму Figure 8-1 DMA1 request image или таблицу Table 8-2 DMA1 peripheral mapping table for each channel
    // там к каждому каналу подключен свой набор переферии и т.д.
    // в режиме MEM2MEM bit можно любой канал
    DMA_DeInit(DMA_CHx);                                                        // деинециализируем DMA указав канал от 1 до 7
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;                           // адрес переферии  ( смотри реферанс мануал )
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;                              // адрес памяти которую будем отправлять ( в данном примере ето массив uint16_t pbuf[3] = {10, 50, 80}; )
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;                                 // размер отправляемых данных  ( не байт а штук ) зависит от DMA_PeripheralDataSize_HalfWord и DMA_MemoryDataSize_HalfWord
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // ( c PWM 8бит не работает только 16 ) выставляем размер 1 пакета ( 8 16 32 байта )
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         // ( можно 8 бит если массив данных 8 битный ) выставляем размер 1 пакета ( 8 16 32 байта )
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             // отправка циклическая или единоразовая
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                // режим память в память отключаем ( так как отправляем в переферию )
    DMA_Init(DMA_CHx, &DMA_InitStructure);

    // запускаем DMA указав канал
    DMA_Cmd(DMA_CHx, ENABLE);
}
//**************************************************************************

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

    // настраиваем таймер на частоту ( 48 000 000 / 48 000 = 1 000 тиков в 1 секунду ( так как счетчик 100 ) то получаем частота каждые 100 миллесикунд или 10 Гц )
    TIM1_PWMOut_Init(100-1, 48000 - 1, pbuf[0]);

    // указываем параметры для DMA
    // чтобы указать нужный канал DMA TIM1_UP( в данном примере DMA1_Channel5 )
    // смотри в референс мануал диаграмму Figure 8-1 DMA1 request image или таблицу Table 8-2 DMA1 peripheral mapping table for each channel
    // там к каждому каналу подключен свой набор переферии и т.д.
    // в режиме MEM2MEM bit можно любой канал
    TIM1_DMA_Init(DMA1_Channel5, (u32)TIM1_CH1CVR_ADDRESS, (u32)pbuf, 3);

    // запускаем перезапуск DMA после обновления таймера
    TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);

    // включаем таймер и ШИМ
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // в данном примере при указанных данных ( uint16_t pbuf[3] = {10, 50, 80}; ) и расчетах тамера 100 миллисекунд ( градация ШИМ 100 едениц )
    // получаем сперва будет 10 % верхний уровень потом ( 100 - 10 = 90% нижний ) далее  50% верхний и 50% нижний потом 80% верхний и 20% нижний
    // такое будет если в настройках ШИМ указали режим прямой TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1
    // если указали режим инверсный TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2 то будет наоборот сперва нижний фронт указанный в массиве
    // потом верхний с расчетом 100 -  указаный из массива

    while(1)
    {

    }
}
