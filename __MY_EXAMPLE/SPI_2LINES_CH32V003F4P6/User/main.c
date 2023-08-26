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

// для примера воьмем микросхему MCP23S17 ( MCP23017 & MCP23S17 16_Bit_IO ) которая на SPI 16 битный разшеритель портов ввода вывода

// подключение согласно даташиту
// MISO - PC7
// MOSI - PC6
// SCK - PC5
// CS - любой контакт я выбрал PD2

// на самой микросхеме A0 A1 A2 это выбор адресса

// *** регистры для работы с микросхемой ******************
// MCP23S17 SPI Slave Device
#define MCP_SLAVE_ID    0x40
#define MCP_SLAVE_ADDR  0x00      // A2=0,A1=0,A0=0
#define MCP_SLAVE_WRITE 0x00
#define MCP_SLAVE_READ  0x01

// MCP23S17 Registers Definition for BANK=0 (default)
#define IODIRA 0x00
#define IODIRB 0x01
#define IOCONA 0x0A
#define GPPUA  0x0C
#define GPPUB  0x0D
#define GPIO_A  0x12
#define GPIO_B  0x13
// ********************************************************



/* Global define */


/* Global Variable */

//********************************************************************
void SPI_FullDuplex_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    SPI_InitTypeDef SPI_InitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_SPI1, ENABLE );

    // SCK - PC5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    // MISO - PC7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    // MOSI - PC6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    // CS - любой контакт я выбрал PD2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );

    // режим SPI 1 или 2 линии и т.д.
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;

    // мастер или слейв
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

    // настройки самой шины SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;       // 8 or 16 Bit
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;      // only MSB
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI1, &SPI_InitStructure );

    // активируем SPI
    SPI_Cmd( SPI1, ENABLE );
}
//********************************************************************

////** Если данніе для отправки в SPI SPI_FirstBit_LSB ( а данный модуль поддерживает только SPI_FirstBit_MSB то переставляем биты местами****
//uint8_t swapBytes(uint8_t byte) {
//    uint8_t swapped = 0;
//    uint8_t i;
//
//    for (i = 0; i < 8; i++) {
//        if (byte & (1 << i)) {
//            swapped |= 1 << (7 - i);
//        }
//    }
//
//    return swapped;
//}
////********************************************************************

//********************************************************************
void MCP_Write(uint8_t addr, uint8_t data)
{
    // Activate the CS pin
    GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);    // CS LOW

    while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_BSY ) != RESET );

    // Start MCP23S17 OpCode transmission
    // отправляем данные -----------------------------------------------
    //if( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) != RESET )
    while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) == RESET );
    {
        SPI_I2S_SendData( SPI1, MCP_SLAVE_ID | ((MCP_SLAVE_ADDR << 1) & 0x0E)| MCP_SLAVE_WRITE );   // MCP_SLAVE_WRITE
    }
    //------------------------------------------------------------------

    // Start MCP23S17 Address transmission
    // отправляем данные -----------------------------------------------
    //if( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) != RESET )
    while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) == RESET );
    {
        SPI_I2S_SendData( SPI1, addr );
    }
    //------------------------------------------------------------------

    // Start Data transmission
    // отправляем данные -----------------------------------------------
    //if( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) != RESET )
    while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) == RESET );
    {
        SPI_I2S_SendData( SPI1, data );
    }
    //------------------------------------------------------------------

    // делаем задержку чтобы данные ушли перед подьема CS
    Delay_Ms(20);

    // CS pin is not active
    GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);    // CS HIGH
}
//******************************************************************************

//******************************************************************************
uint8_t MCP_Read(uint8_t addr)
{
    uint8_t read_data = 0;

    // Activate the CS pin
    GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);    // CS LOW

    while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_BSY ) != RESET );

    // Start MCP23S17 OpCode transmission
    // отправляем данные -----------------------------------------------
    //if( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) != RESET )
    while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) == RESET );
    {
        SPI_I2S_SendData( SPI1, MCP_SLAVE_ID | ((MCP_SLAVE_ADDR << 1) & 0x0E)| MCP_SLAVE_READ );    // MCP_SLAVE_READ
    }
    //------------------------------------------------------------------

    // Start MCP23S17 Address transmission
    // отправляем данные -----------------------------------------------
    //if( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) != RESET )
    while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) == RESET );
    {
        SPI_I2S_SendData( SPI1, addr );
    }
    //------------------------------------------------------------------

    //Delay_Ms(20);

    // Send Dummy transmission for reading the data
    // принимаем данные -----------------------------------------------
    //if( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_RXNE ) != RESET )
    while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_RXNE ) == RESET );
    {
        read_data = SPI_I2S_ReceiveData( SPI1 );
    }
    //------------------------------------------------------------------

    // делаем задержку чтобы данные ушли перед подьема CS
    Delay_Ms(20);

    // CS pin is not active
    GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);    // CS HIGH

    return read_data;
}
//******************************************************************************
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


    GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);    // CS HIGH

    SPI_FullDuplex_Init();

    GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);    // CS LOW

    // Initial the MCP23S17 SPI I/O Expander
    MCP_Write(IOCONA, 0x28);    // I/O Control Register: BANK=0, SEQOP=1, HAEN=1 (Enable Addressing)
    MCP_Write(IODIRA, 0x00);    // GPIOA As Output
    MCP_Write(IODIRB, 0xFF);    // GPIOB As Input
    MCP_Write(GPPUB, 0xFF);     // Enable Pull-up Resistor on GPIOB
    MCP_Write(GPIO_A, 0x00);    // Reset Output on GPIOA

    while(1)
    {

        uint8_t read_port = 0;

        // на порту А включаем указаный бит ( номер пина  0х01 0х02 и т.д 0xFF включаю все пины)
        MCP_Write(GPIO_A, 0xff);

        // читаю все пины порта В ( если нужно конкретный например 1 пин -В0 то ставим маску (GPIO_B & 0x01)  )
        // при данной настройке все пины внутри микросхемы подтянуты к питанию и все порты вадают 1 , для кнопки комутируем с 0
        read_port = MCP_Read(GPIO_B);
        printf("read_port -> %d\r\n", read_port);

        Delay_Ms( 500 );

        // выключаю все пины на порту А
        MCP_Write(GPIO_A, 0x00);

        Delay_Ms( 500 );

    }
}
