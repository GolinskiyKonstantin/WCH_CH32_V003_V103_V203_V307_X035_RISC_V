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
// buffer uart Receive
volatile uint8_t uart_buff[20] = {0, };
volatile uint8_t index_buff = 0;
volatile uint8_t flag_RX_END = 0;
//*********************************************************************************************

//*********************************************************************************************
void USART_config_1(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure = {0};

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

    // перечень прерываний ( через побитовый или |, USART_IT_RXNE | USART_IT_IDLE не заработал пришлось отдельно )
//        USART_IT_LBD - LIN Break detection interrupt.
//        USART_IT_TXE - Transmit Data Register empty interrupt.
//        USART_IT_TC - Transmission complete interrupt.
//        USART_IT_RXNE - Receive Data register not empty interrupt.
//        USART_IT_IDLE - Idle line detection interrupt.
//        USART_IT_PE - Parity Error interrupt.
//        USART_IT_ERR - Error interrupt.
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);          // прерывание на прием USART_IT_RXNE
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);          // прерывание USART_IT_IDLE

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}
//*********************************************************************************************

//*********************************************************************************************
void USART_config_2(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure = {0};

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

    // перечень прерываний ( через побитовый или |, USART_IT_RXNE | USART_IT_IDLE не заработал пришлось отдельно )
//        USART_IT_LBD - LIN Break detection interrupt.
//        USART_IT_TXE - Transmit Data Register empty interrupt.
//        USART_IT_TC - Transmission complete interrupt.
//        USART_IT_RXNE - Receive Data register not empty interrupt.
//        USART_IT_IDLE - Idle line detection interrupt.
//        USART_IT_PE - Parity Error interrupt.
//        USART_IT_ERR - Error interrupt.
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);          // прерывание на прием USART_IT_RXNE
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);          // прерывание USART_IT_IDLE

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}
//*********************************************************************************************

//*********************************************************************************************
void USART_config_3(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure = {0};

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

    // перечень прерываний ( через побитовый или |, USART_IT_RXNE | USART_IT_IDLE не заработал пришлось отдельно )
//        USART_IT_LBD - LIN Break detection interrupt.
//        USART_IT_TXE - Transmit Data Register empty interrupt.
//        USART_IT_TC - Transmission complete interrupt.
//        USART_IT_RXNE - Receive Data register not empty interrupt.
//        USART_IT_IDLE - Idle line detection interrupt.
//        USART_IT_PE - Parity Error interrupt.
//        USART_IT_ERR - Error interrupt.
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);          // прерывание на прием USART_IT_RXNE
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);          // прерывание USART_IT_IDLE

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

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
// обработчик прерывания
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)
{
    //Если пришли данные по USART
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uart_buff[index_buff++] = (uint8_t)USART_ReceiveData(USART1);
    }
    //USART_ClearITPendingBit(USART1, USART_IT_RXNE);

    //Если прилетел флаг IDLE
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
         flag_RX_END = 1;
         //printf("USART_IT_IDLE\r\n");
         USART1->DATAR; //Сбросим флаг IDLE
    }

    // если нужно отключаем прерівание
    // USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
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

    char start_msg[] = "START INTERRUPT IDLE UART !!!\r\n";
    UART_Transmit(start_msg, strlen(start_msg));

    while(1)
    {

        if( flag_RX_END == 1 )
        {
            flag_RX_END = 0;
            UART_Transmit((uint8_t*)uart_buff, index_buff);
            index_buff = 0;
        }

       printf("----------\r\n");
       Delay_Ms(1000);
       printf("----------\r\n");
       Delay_Ms(1000);
       printf("----------\r\n");
       Delay_Ms(1000);
       printf("----------\r\n");
       Delay_Ms(1000);
       printf("----------\r\n");
       Delay_Ms(1000);

    }
}
