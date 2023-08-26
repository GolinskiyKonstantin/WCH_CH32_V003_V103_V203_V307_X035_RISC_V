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

//*********************************************************************************
//  для работы функций указываем сдвинутый адресс на <<1
//*********************************************************************************



/* Global Variable */

//******************************************************************************
// I2C ( default ) -> PC1=SDA   PC2=SCL - I2C
// I2C ( remap ) -> PC6=SDA   PC5=SCL - I2C_2 I2C_3
// I2C ( remap ) -> PD0=SDA   PD1=SCL - I2C_1
void I2C_Config(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitTSturcture={0};

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
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;  // Рабочий цикл I2C в быстром режиме ( I2C_DutyCycle_16_9 или I2C_DutyCycle_2 )
    I2C_InitTSturcture.I2C_OwnAddress1 = 0;                 // адрес устройства с которым будем общаться ( будем указывать отдельно ) // указываем сдвинутый адресс на <<1
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;            // включаем АСК
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;      // режим адреса 7 или 10 бит
    I2C_Init( I2C1, &I2C_InitTSturcture );

    // запускаем шину I2C
    I2C_Cmd( I2C1, ENABLE );
    I2C_AcknowledgeConfig( I2C1, ENABLE );                  // Включает или отключает указанную функцию подтверждения I2C.

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

    //******************************************************************************
    //---- I2C SCANNER 7-bit adres -------------------------------------------------
    #define TIME_OUT_I2C    10000       // кол-во тиков ( если МК мощный то ставим больше )

    uint32_t time_out = TIME_OUT_I2C;   //переменная ( аналог таймаута ) считает тики

    // в цикле прокручиваем все адреса ( 7 бит без учета нулевого там всегда 0 )
    for( uint8_t adress_i = 0; adress_i < 127; adress_i++ ){
        while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET ){Delay_Ms(1);}        // ждем когда линия будет свободна
        I2C_GenerateSTART( I2C1, ENABLE );                                              // Генерирует состояние СТАРТ связи I2Cx ( стартовый бит )
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) ){Delay_Ms(1);}    // ждем когда включиться режим мастера
        I2C_Send7bitAddress( I2C1, adress_i<<1, I2C_Direction_Transmitter );            // указываем адрес устройства с которым будем общаться по I2C ( и режим прием или передача )// указываем сдвинутый адресс на <<1

        // далее в цикле ждем если эвент отработает то такой адрес есть
        // если в течении TIME_OUT_I2C тиков не отработает то покидаем цикл и заходим на новую итерацию for
         while(1){
            if(I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED )){
                printf("I2C real adress -> 0x%02X  | adress <<1 -> 0x%02X \r\n", adress_i, adress_i<<1);
            }
            else{
                time_out--;
                if(time_out == 0){
                   time_out = TIME_OUT_I2C;
                   break;
                }
            }
         }
         I2C_GenerateSTOP( I2C1, ENABLE );                                               // Генерирует состояние СТОП связи I2Cx ( стоповый бит )
    }
    printf("I2C SCANN END!!!\r\n");
    //------------------------------------------------------
    //******************************************************************************


    while(1)
    {

    }
}
