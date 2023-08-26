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
#include "string.h"

/* Global define */


/* Global Variable */

//*********************************************************************************************
void USART_config_1(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    /* USART1 TX-->D.5 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // TX
    GPIO_Init(GPIOD, &GPIO_InitStructure);              // D

    /* USART1 RX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   // RX
    GPIO_Init(GPIOD, &GPIO_InitStructure);                  // D

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}
//*********************************************************************************************

//*********************************************************************************************
void USART_config_2(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

     /*
     * @brief   Changes the mapping of the specified pin.
     *
     * @param   GPIO_Remap - selects the pin to remap.
     *            GPIO_Remap_SPI1 - SPI1 Alternate Function mapping
     *            GPIO_PartialRemap_I2C1 - I2C1 Partial Alternate Function mapping
     *            GPIO_PartialRemap_I2C1 - I2C1 Full Alternate Function mapping
     *            GPIO_PartialRemap1_USART1 - USART1 Partial1 Alternate Function mapping
     *            GPIO_PartialRemap2_USART1 - USART1 Partial2 Alternate Function mapping
     *            GPIO_FullRemap_USART1 - USART1 Full Alternate Function mapping
     *            GPIO_PartialRemap1_TIM1 - TIM1 Partial1 Alternate Function mapping
     *            GPIO_PartialRemap2_TIM1 - TIM1 Partial2 Alternate Function mapping
     *            GPIO_FullRemap_TIM1 - TIM1 Full Alternate Function mapping
     *            GPIO_PartialRemap1_TIM2 - TIM2 Partial1 Alternate Function mapping
     *            GPIO_PartialRemap2_TIM2 - TIM2 Partial2 Alternate Function mapping
     *            GPIO_FullRemap_TIM2 - TIM2 Full Alternate Function mapping
     *            GPIO_Remap_PA12 - PA12 Alternate Function mapping
     *            GPIO_Remap_ADC1_ETRGINJ - ADC1 External Trigger Injected Conversion remapping
     *            GPIO_Remap_ADC1_ETRGREG - ADC1 External Trigger Regular Conversion remapping
     *            GPIO_Remap_LSI_CAL - LSI calibration Alternate Function mapping
     *            GPIO_Remap_SDI_Disable - SDI Disabled
     *          NewState - ENABLE or DISABLE.
     */
    GPIO_PinRemapConfig(GPIO_PartialRemap2_USART1, ENABLE);

    /* USART1 TX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // TX
    GPIO_Init(GPIOD, &GPIO_InitStructure);              // D

    /* USART1 RX-->D.5 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   // RX
    GPIO_Init(GPIOD, &GPIO_InitStructure);                  // D

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}
//*********************************************************************************************

//*********************************************************************************************
void USART_config_3(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

     /*
     * @brief   Changes the mapping of the specified pin.
     *
     * @param   GPIO_Remap - selects the pin to remap.
     *            GPIO_Remap_SPI1 - SPI1 Alternate Function mapping
     *            GPIO_PartialRemap_I2C1 - I2C1 Partial Alternate Function mapping
     *            GPIO_PartialRemap_I2C1 - I2C1 Full Alternate Function mapping
     *            GPIO_PartialRemap1_USART1 - USART1 Partial1 Alternate Function mapping
     *            GPIO_PartialRemap2_USART1 - USART1 Partial2 Alternate Function mapping
     *            GPIO_FullRemap_USART1 - USART1 Full Alternate Function mapping
     *            GPIO_PartialRemap1_TIM1 - TIM1 Partial1 Alternate Function mapping
     *            GPIO_PartialRemap2_TIM1 - TIM1 Partial2 Alternate Function mapping
     *            GPIO_FullRemap_TIM1 - TIM1 Full Alternate Function mapping
     *            GPIO_PartialRemap1_TIM2 - TIM2 Partial1 Alternate Function mapping
     *            GPIO_PartialRemap2_TIM2 - TIM2 Partial2 Alternate Function mapping
     *            GPIO_FullRemap_TIM2 - TIM2 Full Alternate Function mapping
     *            GPIO_Remap_PA12 - PA12 Alternate Function mapping
     *            GPIO_Remap_ADC1_ETRGINJ - ADC1 External Trigger Injected Conversion remapping
     *            GPIO_Remap_ADC1_ETRGREG - ADC1 External Trigger Regular Conversion remapping
     *            GPIO_Remap_LSI_CAL - LSI calibration Alternate Function mapping
     *            GPIO_Remap_SDI_Disable - SDI Disabled
     *          NewState - ENABLE or DISABLE.
     */
    GPIO_PinRemapConfig(GPIO_FullRemap_USART1, ENABLE);

    /* USART1 TX-->C.0 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // TX
    GPIO_Init(GPIOC, &GPIO_InitStructure);              // C

    /* USART1 RX-->C.1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   // RX
    GPIO_Init(GPIOC, &GPIO_InitStructure);                  // C

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
}
//*********************************************************************************************

//*********************************************************************************************
void UART_Transmit(uint8_t* buff, uint16_t size){
    for(uint16_t i = 0; i < size; i++){
        //USART_FLAG_TXE - Transmit data register empty flag
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);    // тут крутимся пока буфер станет свободным
        USART_SendData(USART1, *buff++);
        // USART_FLAG_TC - Transmission Complete flag
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);     // тут ждем когда передача будет завершена
    }
}
//*********************************************************************************************

//*********************************************************************************************
void UART_Receive(uint8_t* buff, uint16_t size){
    for(uint16_t i = 0; i < size; i++){
        // USART_FLAG_RXNE - Receive data register not empty flag.
        while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);   // тут крутимся ( блокируем цикл )ждем когда байт попадет в буфер
        *buff++ = USART_ReceiveData(USART1);
    }
}
//*********************************************************************************************

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

    USART_config_1();
    //USART_config_2();
    //USART_config_3();

    while(1)
    {

//        //***************************************************************
//        // USART_FLAG_RXNE - Receive data register not empty flag.
//        //while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);   // тут крутимся ( блокируем цикл )ждем когда байт попадет в буфер
//        if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)        // тут не крутимся ( не блокируем цикл ) если чтото пришло тогда отсылаем обратно
//        {
//            uint8_t val = (uint8_t)USART_ReceiveData(USART1);
//
//            //USART_FLAG_TXE - Transmit data register empty flag
//            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);    // тут крутимся пока буфер станет свободным
//            USART_SendData(USART1, val);
//            // USART_FLAG_TC - Transmission Complete flag
//            while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);     // тут ждем когда передача будет завершена
//        }
//        //***************************************************************

        //***************************************************************
            char buff_v[3] = {0,};
            // юарт читает по 1 байту ( буфера нет ) если в момент когда данные приходят мы не читаем их то они теряються , сохранен будет только последний байт
            // поетому в функции UART_Receive() мы стопоримся и крутимся покругу пока не придет первый байт, далее есго считываем и считываем еще сколько указали
            // в функции ( в данном примере 3 байта всего, как прочитали покидаем функцию печатаем то что пришло и снова в нее попадаем и крутимся до очережных 3-х байт
            // поэтому данная реализация плохая принимать данные нужно через прерывания или DMA.
            // ну или по 1 байту тогда можно читать когда угодно
            UART_Receive(buff_v, sizeof(buff_v));

            UART_Transmit( buff_v, sizeof(buff_v));
        //***************************************************************

        //***************************************************************
//        char str []= "QWERTY!!!\r\n";
//        UART_Transmit( str, strlen(str));
        //***************************************************************

        //***************************************************************
        // USART_FLAG_IDLE - Idle Line detection flag.
//        if( USART_GetFlagStatus(USART1,USART_FLAG_IDLE) == SET){
//            printf("Idle Line detection !!! \r\n");
//            USART1->DATAR; //Сбросим флаг IDLE
//        }
        //***************************************************************

        //Delay_Ms(5);
    }
}
