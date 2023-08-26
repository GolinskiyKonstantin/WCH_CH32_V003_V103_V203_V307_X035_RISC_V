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

#define I2C_ADR_DS3231          (0x68<<1)        // указываем сдвинутый адресс на <<1


/* Global Variable */

//******************************************************************************
// I2C ( default ) -> PC1=SDA   PC2=SCL - I2C
// I2C ( remap ) -> PC6=SDA   PC5=SCL - I2C_2 I2C_3
// I2C ( remap ) -> PD0=SDA   PD1=SCL - I2C_1
void I2C_Config(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitTSturcture={0};

    NVIC_InitTypeDef  NVIC_InitStructure = {0};

    // включаем тактирование порта С  шины APB2 и I2C
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    // настраиваем пины в альтернативной функцией---------------
    // SDA
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;               // pin 2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;         // открытый коллектор
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );                // port C

    // SCL
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;               // pin 1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;         // открытый коллектор
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );                // port C
    //----------------------------------------------------------

    // настройка шины I2C
    I2C_InitTSturcture.I2C_ClockSpeed = bound;              // частота I2C ( 100 000 Гц или 400 000 Гц )
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;             // режим I2C
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;     // Рабочий цикл I2C в быстром режиме ( I2C_DutyCycle_16_9 или I2C_DutyCycle_2 )
    I2C_InitTSturcture.I2C_OwnAddress1 = 0;                 // адрес устройства с которым будем общаться ( будем указывать отдельно ) указываем сдвинутый адресс на <<1
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;            // включаем АСК
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;      // режим адреса 7 или 10 бит
    I2C_Init( I2C1, &I2C_InitTSturcture );

    // включаем прерывание на event
    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;         //I2C1 Event Interrupt
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // запускаем шину I2C
    I2C_Cmd( I2C1, ENABLE );
    I2C_AcknowledgeConfig( I2C1, ENABLE );                  // Включает или отключает указанную функцию подтверждения I2C.

}
//******************************************************************************

//******************************************************************************
void I2C1_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_EV_IRQHandler(void)
{
    // ОТПРАВКА ДАННЫХ
    if( I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) == SET ){
        if( I2C_GetITStatus(I2C1, I2C_IT_SB ) ){   //  I2C_IT_SB - Start bit flag (Master mode).
            I2C_Send7bitAddress( I2C1, I2C_ADR_DS3231, I2C_Direction_Transmitter );
            I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
        }
        while(I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) == RESET );

        // TXE установлен, BTF установлен (только в этот момент можно генерировать STOP)
        if( (I2C_GetITStatus(I2C1, I2C_IT_TXE) == SET) && (I2C_GetITStatus(I2C1, I2C_IT_BTF) == SET) ){
                I2C_GenerateSTOP(I2C1, ENABLE);
        }
        // TXE установлен, BTF нет
        if( I2C_GetITStatus(I2C1, I2C_IT_TXE) == SET && I2C_GetITStatus(I2C1, I2C_IT_BTF) == RESET )
        {
           uint8_t data_reg = 0x00;             // ds3231 reg second
           I2C_SendData( I2C1, data_reg );      // отправляем 1 байт информации
           I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
           // ждем окончания передачи
           while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
           I2C_GenerateSTOP( I2C1, ENABLE );

           printf("I2C SendData 1!!!\r\n");
        }

        // TXE не установлен, BTF установлен
        if( I2C_GetITStatus(I2C1, I2C_IT_TXE) == RESET && I2C_GetITStatus(I2C1, I2C_IT_BTF) == SET )
        {
            uint8_t data_reg = 0x00;             // ds3231 reg second
            I2C_SendData( I2C1, data_reg );      // отправляем 1 байт информации
            I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
            // ждем окончания передачи
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
            I2C_GenerateSTOP( I2C1, ENABLE );
            printf("I2C SendData 2!!!\r\n");
        }

            // ПРИЕМ ДАННЫХ
        //while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );                                         // ждем когда линия будет свободна

        I2C_GenerateSTART( I2C1, ENABLE );                                                                  // Генерирует состояние СТАРТ связи I2Cx ( стартовый бит )

        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );                                     // ждем когда включиться режим мастера

        I2C_Send7bitAddress( I2C1, I2C_ADR_DS3231, I2C_Direction_Receiver );                                // указываем адрес устройства с которым будем общаться по I2C ( и режим прием или передача )

        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );                          // ждем когда включиться режим мастера для приема


        // проверяем если буфер приема свободен
        while( I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE ) ==  RESET );

        uint8_t data_read = I2C_ReceiveData( I2C1 );                                                                // отправляем 1 байт информации

        I2C_GenerateSTOP( I2C1, ENABLE );
        printf("data - %d \r\n", data_read);
    }

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


    // указываем скорость работы 100 000 Гц или 400 000 Гц
    I2C_Config( 100000 );


    while(1)
    {
        while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );                                         // ждем когда линия будет свободна
        // включаем прерывание на BUF
        //I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
        I2C_GenerateSTART( I2C1, ENABLE );

        printf("MasterTransmit \r\n");
        Delay_Ms(1000);

    }
}
