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


//  выводится на OPA0, а АЦП производит выборку OPA0.
//  пины смотри в даташите
//  OPA0_CHP0--PA2
//  OPA0_CHN0--PA1
//  OPA0_OUT--PD4


//****************************************************************************
//  OPA0 используется как компаратор
//****************************************************************************
// принцип работы как у операционника подаем например на положительный CHP0--PA2 3 вольта
// а на отрицательный CHN0--PA1 2 вольта на выходе получаем 1 если наоборот то получим 0



//****************************************************************************
//  OPA0 используется как операционный уселитель
//****************************************************************************
// если отрицательный вход CHN0--PA1 подключить к выходу операционника OPA0_OUT--PD4
// а положительный вход CHP0--PA2 подключить к потенциометру
// то на выходе операционника OPA0_OUT--PD4 будет повторяться то напряжение
// которое приходит на положительный вход CHP0--PA2
// ( данная схема называеться повторитель напряжения ) в данной схеме увеличиваеться ТОК
//----------
// увеличиваем входящее напряжения ( максимум на выходе будет напряжение питания )
// для етого подключаем так:
// выход операционника OPA0_OUT--PD4 подключаем через два последовательно подключенных резисторах к GND
// получаеться как делитель напряжения , один край к GND другой край выход операционника OPA0_OUT--PD4
// а центральная ножка делителя ( между резисторами ) подключаем на отрицательный вход CHN0--PA1
// ( например сделали делитель на 2  - 10кОм и 10кОм резисторы в таком исполнении если
// на положительный вход CHP0--PA2 подать 1 вольт то на выходе операционника OPA0_OUT--PD4 будет 2 вольта
// можно подбирая разницу резисторов делать разный коэффициент умножения
// для етого увеличиваем резистор который стоит между
// выходом операционника OPA0_OUT--PD4 и отрицательным входом CHN0--PA1
// но не забываем что на выходе будет МАКСИМУМ напряжение питания МК




//******************************************************************
void OPA0_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    OPA_InitTypeDef  OPA_InitStructure = {0};

    //  пины смотри в даташите
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );     // port A

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;                   // pin 1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOA, &GPIO_InitStructure );                    // port A

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                   // pin 2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOA, &GPIO_InitStructure );                    // port A

    OPA_InitStructure.PSEL = CHP0;
    OPA_InitStructure.NSEL = CHN0;
    OPA_Init( &OPA_InitStructure );
    OPA_Cmd( ENABLE );

}
//******************************************************************

//******************************************************************
// OUT--PD4 данная ножка это выход операционника ( смотрим напряжение на ней )
// ( если в режиме компаратора ) можно сделать как цифровой порт и читать 0 или 1
void ADC_Channel7_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    ADC_InitTypeDef ADC_InitStructure = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );
    RCC_ADCCLKConfig( RCC_PCLK2_Div8 );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    ADC_RegularChannelConfig( ADC1, ADC_Channel_7, 1, ADC_SampleTime_241Cycles );
    ADC_Init( ADC1, &ADC_InitStructure );

    ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
    ADC_Cmd( ADC1, ENABLE );

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}
//******************************************************************

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

    OPA0_Init();
    ADC_Channel7_Init();

    while(1)
    {
        // запускаем замер АЦП
        ADC_SoftwareStartConvCmd( ADC1, ENABLE );
        while( !ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC ) );

        uint16_t ADC_val = ADC_GetConversionValue( ADC1 );
        printf( "OPA_OUT:%04d\r\n", ADC_val );
        Delay_Ms( 500 );

    }
}
