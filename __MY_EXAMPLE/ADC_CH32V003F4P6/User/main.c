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

#define ADC_INT_ENABLE   0   // пример с прерыванием или без прерывания ( 1 - 0 )

#define ADC_REG          1
#define ADC_INJ          0

/* Global define */


/* Global Variable */

//====================================================================================================================================
// Если используем порт А1 то не забываем переключиться на внутрешнее тактирование так как по умолчанию на этом порту включен кварц
//====================================================================================================================================

//***************************************************************************
#if ADC_REG
    // работаем с одним каналом режим RegularChannel ( только один канал ) ( ADC 10 bit -> 1023 )
    // подключаем к пину C4 по даташиту на етом пине ( карта пинов ) находится канал 2 ( ADC A2 )
    void ADC_RegularChannel_Init(void)
    {
        ADC_InitTypeDef  ADC_InitStructure = {0};
        GPIO_InitTypeDef GPIO_InitStructure = {0};

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);    // port C
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
        RCC_ADCCLKConfig(RCC_PCLK2_Div8);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;       // pin 4
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_Init(GPIOC, &GPIO_InitStructure);          // port C

        ADC_DeInit(ADC1);
        ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_InitStructure.ADC_ScanConvMode = DISABLE;
        ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_None;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_NbrOfChannel = 1;         // кол-во каналов с которыми работаем
        ADC_Init(ADC1, &ADC_InitStructure);

        ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_241Cycles); // C4 по даташиту на етом пине ( карта пинов ) находится канал 2 ( ADC A2 )

        ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
        ADC_Cmd(ADC1, ENABLE);

    #if ADC_INT_ENABLE
        NVIC_InitTypeDef NVIC_InitStructure = {0};

        NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         // запуск прерывания при старте ( можно запустить отдельно NVIC_EnableIRQ(ADC_IRQn); )
        NVIC_Init(&NVIC_InitStructure);

        // конфигурация для прерывания ( ADC, флаг срабатывания прерывания и включение )
        ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // ADC_IT_EOC - End of conversion interrupt mask.
    #endif

        // делаем калибровку
        ADC_ResetCalibration(ADC1);
        while(ADC_GetResetCalibrationStatus(ADC1));
        ADC_StartCalibration(ADC1);
        while(ADC_GetCalibrationStatus(ADC1));
    }
#endif
//***************************************************************************

//***************************************************************************
#if ADC_INJ
    // работаем с двумя каналами режим InjectedChannel ( режим до 4 каналов ) ( ADC 10 bit -> 1023 )
    // подключаем к пину C4 по даташиту на етом пине ( карта пинов ) находится канал 2 ( ADC A2 )
    // подключаем к пину D2 по даташиту на етом пине ( карта пинов ) находится канал 3 ( ADC A3 )
    // подключаем к пину A1 по даташиту на етом пине ( карта пинов ) находится канал 1 ( ADC A1 )
    void ADC_InjectedChannel_Init(void)
    {
        ADC_InitTypeDef  ADC_InitStructure = {0};
        GPIO_InitTypeDef GPIO_InitStructure = {0};

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA, ENABLE);    // port A port C port D
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
        RCC_ADCCLKConfig(RCC_PCLK2_Div8);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;       // pin 4
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_Init(GPIOC, &GPIO_InitStructure);          // port C

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;       // pin 2
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_Init(GPIOD, &GPIO_InitStructure);          // port D

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;       // pin 1
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_Init(GPIOA, &GPIO_InitStructure);          // port A

        ADC_DeInit(ADC1);
        ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_InitStructure.ADC_ScanConvMode = ENABLE;
        ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_None;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_NbrOfChannel = 3;         // кол-во каналов с которыми работаем
        ADC_Init(ADC1, &ADC_InitStructure);

        ADC_InjectedSequencerLengthConfig(ADC1, 3);     // кол-во каналов с которыми работаем
        ADC_InjectedChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_241Cycles); // C4 по даташиту на етом пине ( карта пинов ) находится канал 2 ( ADC A2 )
        ADC_InjectedChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_241Cycles); // D2 по даташиту на етом пине ( карта пинов ) находится канал 3 ( ADC A3 )
        ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 3, ADC_SampleTime_241Cycles); // A1 по даташиту на етом пине ( карта пинов ) находится канал 1 ( ADC A1 )

        ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
        //ADC_AutoInjectedConvCmd(ADC1, ENABLE);
        ADC_Cmd(ADC1, ENABLE);

    #if ADC_INT_ENABLE
        NVIC_InitTypeDef NVIC_InitStructure = {0};

        NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // запуск прерывания при старте ( можно запустить отдельно NVIC_EnableIRQ(ADC_IRQn); )
        NVIC_Init(&NVIC_InitStructure);

        // конфигурация для прерывания ( ADC, флаг срабатывания прерывания и включение )
        ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE); // ADC_IT_JEOC - End of injected conversion interrupt mask.
    #endif

        // делаем калибровку
        ADC_ResetCalibration(ADC1);
        while(ADC_GetResetCalibrationStatus(ADC1));
        ADC_StartCalibration(ADC1);
        while(ADC_GetCalibrationStatus(ADC1));
    }
#endif
//*********************************************************************

//*********************************************************************
#if ADC_INT_ENABLE
    // обработчик прерывания
    void ADC1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
    void ADC1_IRQHandler(void)
    {
        //*****************************************************************
        #if ADC_REG
            //---------------------------
            if(ADC_GetITStatus(ADC1, ADC_IT_EOC))                       // ADC_IT_EOC - End of conversion interrupt mask.
            {
                printf("ADC IT complete...\r\n");
                uint16_t adc_val = ADC_GetConversionValue(ADC1);        // читаем данные замера ( 10 бит  1023 максимум )

                // печатаем результат
                printf("val:%04d\r\n", adc_val);                        // печатаем результат

                ADC_SoftwareStartConvCmd(ADC1, DISABLE);                // повторно запускаем после замера, срабатывает на DISABLE ( должно быть ENABLE );
            }
            ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);                    // сбрасываем флаг
            //---------------------------
        #elif ADC_INJ
            //---------------------------
            if(ADC_GetITStatus(ADC1, ADC_IT_JEOC))                      // ADC_IT_JEOC - End of injected conversion interrupt mask.
            {
                printf("ADC IT complete...\r\n");
                uint16_t adc_jval_1 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);  // читаем данные замера ( 10 бит  1023 максимум )
                uint16_t adc_jval_2 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);  // читаем данные замера ( 10 бит  1023 максимум )
                uint16_t adc_jval_3 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);  // читаем данные замера ( 10 бит  1023 максимум )

                printf("jval_1:%04d   jval_2:%04d   jval_3:%04d\r\n", adc_jval_1, adc_jval_2, adc_jval_3);  // печатаем результат

                ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);         // повторно запускаем после замера, запускаем замер ( тут срабатывает на ENABLE )
            }
            ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);                   // сбрасываем флаг
            //---------------------------
        #endif
        //*****************************************************************

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

#if ADC_REG
    ADC_RegularChannel_Init();          // режим RegularChannel ( только один канал )
#elif ADC_INJ
    ADC_InjectedChannel_Init();         // режим InjectedChannel ( режим до 4 каналов )
#endif

#if ADC_INT_ENABLE
    //включаем прерывание для ADC ( запускаем при конфигурации NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;)
    //NVIC_EnableIRQ(ADC_IRQn);

    #if ADC_REG
        ADC_SoftwareStartConvCmd(ADC1, DISABLE);                // однократно замускаем, срабатывает на DISABLE ( должно быть ENABLE );
    #elif ADC_INJ
        ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);         // однократно замускаем, запускаем замер ( тут срабатывает на ENABLE )
    #endif
#endif



    while(1)
    {
        #if !ADC_INT_ENABLE

            #if ADC_REG
                // запускаем преобразование в режиме RegularChannel ( только один канал ) -------------------------
                ADC_SoftwareStartConvCmd(ADC1, DISABLE);                // срабатывает на DISABLE ( должно быть ENABLE );
                while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);  // ждем окончание замера
                ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

                uint16_t adc_val = ADC_GetConversionValue(ADC1);        // читаем данные замера ( 10 бит  1023 максимум )

                printf("val:%04d\r\n", adc_val);                        // печатаем результат
                Delay_Ms(2);
                //-------------------------------------------------------------------------------------------
            #elif ADC_INJ
                //-- запускаем в режиме InjectedChannel ( режим до 4 каналов ) -------------------------------
                ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);             // запускаем замер ( тут срабатывает на ENABLE )
                while(ADC_GetFlagStatus( ADC1, ADC_FLAG_JEOC ) == RESET);   // ждем окончание замера
                ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);

                uint16_t adc_jval_1 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);  // читаем данные замера ( 10 бит  1023 максимум )
                uint16_t adc_jval_2 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);  // читаем данные замера ( 10 бит  1023 максимум )
                uint16_t adc_jval_3 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);  // читаем данные замера ( 10 бит  1023 максимум )

                printf("jval_1:%04d   jval_2:%04d   jval_3:%04d\r\n", adc_jval_1, adc_jval_2, adc_jval_3);  // печатаем результат
                Delay_Ms(2);
                //-------------------------------------------------------------------------------------------
            #endif

        #endif
    }
}
