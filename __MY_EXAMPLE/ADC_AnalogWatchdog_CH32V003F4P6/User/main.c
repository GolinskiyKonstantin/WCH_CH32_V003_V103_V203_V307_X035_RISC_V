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

//********************************************************************
// отработка прерывания сторожевого таймера по аналоговому сигналу на пине 4 порта C ( PC4 )
void ADC_Function_Init(void)
{
    ADC_InitTypeDef  ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);       // port C
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;                   // pin 4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);                      // port C

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;                     // кол-во каналов с которыми работаем
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_241Cycles);     // C4 по даташиту на етом пине ( карта пинов ) находится канал 2 ( ADC A2 )

    // значения ( окно срабатывания ) Higher Threshold:900, Lower Threshold:500 Всего 10 бит 1023
    // прерывание будет срабатывать каждый раз если значение будет меньше 500 или больше 900
    ADC_AnalogWatchdogThresholdsConfig(ADC1, 900, 500);
    ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_2);                     // C4 по даташиту на етом пине ( карта пинов ) находится канал 2 ( ADC A2 )
    ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);

    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // запуск прерывания при старте ( можно запустить отдельно NVIC_EnableIRQ(ADC_IRQn); )
    NVIC_Init(&NVIC_InitStructure);

    ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);

    // конфигурация для прерывания ( ADC, флаг срабатывания прерывания и включение )
    ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE); // ADC_IT_AWD - Analog watchdog interrupt mask.

    ADC_Cmd(ADC1, ENABLE);

    // делаем калибровку
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}
//**************************************************************************

//*********************************************************************
void ADC1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void ADC1_IRQHandler(void)
{
    if(ADC_GetITStatus( ADC1, ADC_IT_AWD)){
        printf( "Enter AnalogWatchdog Interrupt\r\n" );
    }

    ADC_ClearITPendingBit( ADC1, ADC_IT_AWD);
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

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);

    SysTick_Init();

    ADC_Function_Init();

    while(1)
    {
        // запускаем преобразование в режиме RegularChannel ( только один канал ) -------------------------
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);  // ждем окончание замера
        uint16_t ADC_val = ADC_GetConversionValue(ADC1);        // читаем данные замера ( 10 бит  1023 максимум )
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

        Delay_Ms(500);
        printf("%04d\r\n", ADC_val);
        Delay_Ms(2);

    }
}
