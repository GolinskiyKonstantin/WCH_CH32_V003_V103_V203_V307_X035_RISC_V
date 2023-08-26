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
 USART DMA, master/slave mode transceiver routine:
 USART1_Tx(PD5)\USART1_Rx(PD6).

 This routine demonstrates that two boards use DMA to send and receive.
 After successful sending and receiving, PD0 is connected to LED,
 and the LED light flashes.

   Hardware connection:PD5 -- PD6
                       PD6 -- PD5

*/

#include "debug.h"

/* Global typedef */


/* Global define */
#define SIZE_BUFF_RX    3
#define SIZE_BUFF_TX    4

/* Global Variable */
uint8_t TxBuffer[SIZE_BUFF_TX] = "123";
uint8_t RxBuffer[SIZE_BUFF_RX] = {0};


/*********************************************************************
 * @fn      USARTx_CFG
 *
 * @brief   Initializes the USART1peripheral.
 *
 * @return  none
 */
void USARTx_INIT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    /* USART1 TX-->D.5   RX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    DMA_Cmd(DMA1_Channel4, ENABLE); /* USART1 Tx */
    DMA_Cmd(DMA1_Channel5, ENABLE); /* USART1 Rx */

    USART_Cmd(USART1, ENABLE);
}

/*********************************************************************
 * @fn      DMA_INIT
 *
 * @brief   Configures the DMA for USART1.
 *
 * @return  none
 */
void DMA_INIT(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // TX
    DMA_DeInit(DMA1_Channel4);                                              // номер канала смотри реферанс мануал
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBuffer;              // сам буфер
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = SIZE_BUFF_TX;                        // размер буфера
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // однокрантно
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    // RX
    DMA_DeInit(DMA1_Channel5);                                              // номер канала смотри реферанс мануал
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RxBuffer;              // сам буфер
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         // циклически
    DMA_InitStructure.DMA_BufferSize = SIZE_BUFF_RX;                        // размер буфера
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
}

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

    DMA_INIT();
    USARTx_INIT(); // USART1

    // запускаем DMA
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    // ждем когда данные будут отправлены
    while(DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET); // Wait until USART1 TX DMA1 Transfer Complete

    // ждем когда данные будут приняты
    //while(DMA_GetFlagStatus(DMA1_FLAG_TC5) == RESET); // Wait until USART1 RX DMA1 Transfer Complete

    while(1)
    {
        // если данные были приняты ( 3 байта - размер буффера )
        // если кол-во байт будет принято больше чем указаные 3 то в буфер запишуться ПОСЛЕДНИИ 3
        // если меньше то флаг не отработает будет ждать пока недостающие попадут

        // Если нужно поменять количество данных переданных или принятых то используй функцию DMA_SetCurrDataCounter()
        // естественно кол-во данных не может прывышать кол-во размер буфера ( массива )
        //Устанавливает количество единиц данных в текущей передаче DMAy Channelx.
        //DMA_SetCurrDataCounter(DMA1_Channel5, 2);     // ставим на прием 2 байта

        if(DMA_GetFlagStatus(DMA1_FLAG_TC5) == SET) // Wait until USART1 RX DMA1 Transfer Complete
        {
            //USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);
            printf("RxBuffer -> :%s\r\n",RxBuffer);
            DMA_ClearFlag(DMA1_FLAG_TC5);   // очищаем флаг от повторного попадания в if
            //USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
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

       //***************************************************************
       // USART_FLAG_IDLE - Idle Line detection flag.
       // флаг сбрасываеться при поступлении байта
       // (пока байт не поступил то условие отрабатывает постоянно после сработки данного флага)
        if( USART_GetFlagStatus(USART1,USART_FLAG_IDLE) == SET){
            printf("Idle Line detection !!! \r\n");
            USART1->DATAR; //Сбросим флаг IDLE
        }
       //***************************************************************

    }
}
