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

#define DMA_INT_ENABLE   1   // пример с прерыванием или без прерывания ( 1 - 0 )

// Можно использовать как с прерыванием так и без прерывания
// с прерыванием обрабатывать данные в хедере прерывания
// без прерывания замер идет фоном ( в мейне можно работать с замером ) единственное мы не знаем был ли сделан замер или нет

/* Global define */

uint16_t TxBuf[10]; // буффер для замера данных DMA

/* Global Variable */
//====================================================================================================================================
// Если используем порт А1 то не забываем переключиться на внутрешнее тактирование так как по умолчанию на этом порту включен кварц
//====================================================================================================================================

//**********************************************************************************************
// ADC port C pin 4
void ADC_Function_Init(void)
{
    ADC_InitTypeDef  ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);   // port C
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;               // pin 4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);                  // port C

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_241Cycles); // PC4 по даташиту ето канал A2( ADC_Channel_2 )

    ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}
//**********************************************************************************************

//**********************************************************************************************
void DMA_Tx_Init(DMA_Channel_TypeDef *DMA_CHx, uint32_t ppadr, uint32_t memadr, uint16_t bufsize)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;   // DMA_Mode_Circular - циклический замер,  DMA_Mode_Normal - однократный замер
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);

#if DMA_INT_ENABLE
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;    // АЦП только на первом канале по реферанс мануал Figure 8-1 DMA1 request image
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     // запуск прерывания при старте ( можно запустить отдельно NVIC_EnableIRQ(DMA1_Channel1_IRQn); )
    NVIC_Init(&NVIC_InitStructure);

    // конфигурация для прерывания ( канал дма, флаг срабатывания прерывания и включение )
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); // DMA_IT_TC - Transfer complete interrupt mask
#endif

}
//**********************************************************************************************

//*********************************************************************
#if DMA_INT_ENABLE
    // обработчик прерывания
    void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
    void DMA1_Channel1_IRQHandler(void)
    {

        // проверяем по флагу готовы ли данные ( так как прерывание срабатывает по разным статусам - полу буфер, ошибка и т.д. смотри флаги )
        if( DMA_GetITStatus(DMA1_IT_TC1) ) // проверяем что данные готовы DMA1 Channel1 transfer complete flag.
        {
            printf("DMA1 IT Channel1 transfer complete...\r\n");
            // печатаем результат ( 10 замеров )
            for(uint8_t i = 0; i < 10; i++){
                    printf("%04d\r\n", TxBuf[i]);
            }
        }

        DMA_ClearITPendingBit(DMA1_IT_TC1); // сбрасываем флаг
    }
#endif
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

    SysTick_Init();

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);

    ADC_Function_Init();

    // инициализируем DMA
    DMA_Tx_Init(DMA1_Channel1, (uint32_t)&ADC1->RDATAR, (uint32_t)TxBuf, 10);

    // включаем DMA
    DMA_Cmd(DMA1_Channel1, ENABLE);
    // отключаем DMA ( если нужно )
    //DMA_Cmd(DMA1_Channel1, DISABLE);

#if DMA_INT_ENABLE
    // включаем прерывание для DMA1_Channel1 ( запускаем при конфигурации NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;)
    //NVIC_EnableIRQ(DMA1_Channel1_IRQn);
#endif

    // запускаем преобразование DMA
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);



    while(1)
    {
        //****************************************************
        #if !DMA_INT_ENABLE
            // печатаем результат ( 10 замеров ) если не используем прерывание
            for(uint8_t i = 0; i < 10; i++){
                    printf("%04d\r\n", TxBuf[i]);
                    Delay_Ms(10);
            }
        #endif
        //****************************************************
    }
}
