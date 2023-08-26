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

#define I2C_ADR_AT24CXX       (0x57<<1)       // указываем сдвинутый адресс на <<1

#define REMAP_PIN               0            // пеняем порты по умолчанию I2C ( default ) -> PC1=SDA   PC2=SCL - I2C ремапим на I2C ( remap ) -> PC6=SDA   PC5=SCL - I2C_2 I2C_3

//////////////////////////////////////////////////////////////////////////////////////////////

// размер адреса регистра 8 или 16 бит
#define I2C_ADR_8_BIT           0
#define I2C_ADR_16_BIT          1

//////////////////////////////////////////////////////////////////////////////////////////////


/* Global Variable */

//******************************************************************************
// I2C ( default ) -> PC1=SDA   PC2=SCL - I2C
// I2C ( remap ) -> PC6=SDA   PC5=SCL - I2C_2 I2C_3
// I2C ( remap ) -> PD0=SDA   PD1=SCL - I2C_1
void I2C_Config(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitTSturcture={0};

#if REMAP_PIN

    // I2C ( remap ) -> PC6=SDA   PC5=SCL - I2C_2 I2C_3

    // включаем тактирование порта С  шины APB2 и I2C
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );

     /*
     * @brief   Changes the mapping of the specified pin.
     *
     * @param   GPIO_Remap - selects the pin to remap.
     *            GPIO_Remap_SPI1 - SPI1 Alternate Function mapping
     *            GPIO_PartialRemap_I2C1 - I2C1 Partial Alternate Function mapping
     *            GPIO_FullRemap_I2C1 - I2C1 Full Alternate Function mapping
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
    GPIO_PinRemapConfig(GPIO_FullRemap_I2C1, ENABLE);

    // настраиваем пины в альтернативной функцией---------------
    // SDA
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;               // pin 6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;         // открытый коллектор
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );                // port C

    // SCL
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;               // pin 5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;         // открытый коллектор
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );                // port C
    //----------------------------------------------------------

    // включаем тактирование шины I2C
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

#else

    // I2C ( default ) -> PC1=SDA   PC2=SCL - I2C

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

#endif

    // настройка шины I2C
    I2C_InitTSturcture.I2C_ClockSpeed = bound;              // частота I2C ( 100 000 Гц или 400 000 Гц )
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;             // режим I2C
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;     // Рабочий цикл I2C в быстром режиме ( I2C_DutyCycle_16_9 или I2C_DutyCycle_2 )
    I2C_InitTSturcture.I2C_OwnAddress1 = 0;                 // адрес устройства с которым будем общаться ( будем указывать отдельно ) указываем сдвинутый адресс на <<1
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;            // включаем АСК
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;      // режим адреса 7 или 10 бит
    I2C_Init( I2C1, &I2C_InitTSturcture );

    // запускаем шину I2C
    I2C_Cmd( I2C1, ENABLE );
    I2C_AcknowledgeConfig( I2C1, ENABLE );                  // Включает или отключает указанную функцию подтверждения I2C.
}
//******************************************************************************

//******************************************************************************
uint8_t I2C_mem_read(I2C_TypeDef *I2Cx, uint16_t i2c_adress, uint16_t read_adress, uint8_t size_adress, uint8_t* data, uint16_t size_data, uint32_t time_out)
{
    uint32_t time = time_out;

    while( 1 ){
        if( I2C_GetFlagStatus( I2Cx, I2C_FLAG_BUSY ) == RESET ){
            break;
        }
        else{
            time--;
            if( !time ){ return 0; }
            else{ Delay_Ms(1); }
        }
    }
    I2C_GenerateSTART( I2Cx, ENABLE );

    while( 1 ){
        if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_MODE_SELECT ) ){
            break;
        }
        else{
            time--;
            if( !time ){ return 0; }
            else{ Delay_Ms(1); }
        }
    }
    I2C_Send7bitAddress( I2Cx, i2c_adress, I2C_Direction_Transmitter );

    while( 1 ){
        if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ){
            break;
        }
        else{
            time--;
            if( !time ){ return 0; }
            else{ Delay_Ms(1); }
        }
    }

    if( I2C_ADR_8_BIT == size_adress ){   //(Address_Lenth  == Address_8bit)
        I2C_SendData( I2Cx, (uint8_t)(read_adress & 0x00FF) );
        while( 1 ){
            if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) ){
                break;
            }
            else{
                time--;
                if( !time ){ return 0; }
                else{ Delay_Ms(1); }
            }
        }
    }
    else{  //(Address_Lenth  == Address_16bit)
        I2C_SendData( I2Cx, (uint8_t)(read_adress >> 8) );
        while( 1 ){
            if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) ){
                break;
            }
            else{
                time--;
                if( !time ){ return 0; }
                else{ Delay_Ms(1); }
            }
        }

        I2C_SendData( I2Cx, (uint8_t)(read_adress & 0x00FF) );
        while( 1 ){
            if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) ){
                break;
            }
            else{
                time--;
                if( !time ){ return 0; }
                else{ Delay_Ms(1); }
            }
        }
    }

    I2C_GenerateSTART( I2Cx, ENABLE );

    while( 1 ){
        if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_MODE_SELECT ) ){
            break;
        }
        else{
            time--;
            if( !time ){ return 0; }
            else{ Delay_Ms(1); }
        }
    }

    I2C_Send7bitAddress( I2Cx, i2c_adress, I2C_Direction_Receiver );

    while( 1 ){
        if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) ){
            break;
        }
        else{
            time--;
            if( !time ){ return 0; }
            else{ Delay_Ms(1); }
        }
    }

    //----------------------------------
    for(uint16_t index = 0; index < size_data; index++){
        while( 1 ){
            if( I2C_GetFlagStatus( I2Cx, I2C_FLAG_RXNE ) !=  RESET ){
                break;
            }
            else{
                //I2C_AcknowledgeConfig( I2Cx, DISABLE );
                time--;
                if( !time ){ return 0; }
                else{ Delay_Ms(1); }
            }
        }

        *data = I2C_ReceiveData( I2Cx );
        data++;
    }
    //--------------------------

    I2C_GenerateSTOP( I2Cx, ENABLE );

    return 1;
}
//******************************************************************************

//******************************************************************************
uint8_t I2C_mem_write(I2C_TypeDef *I2Cx, uint16_t i2c_adress, uint16_t write_adress, uint8_t size_adress, uint8_t* data, uint16_t size_data, uint32_t time_out)
{
    uint32_t time = time_out;

    while( 1 ){
       if( I2C_GetFlagStatus( I2Cx, I2C_FLAG_BUSY ) == RESET ){
            break;
       }
       else{
           time--;
           if( !time ){ return 0; }
           else{ Delay_Ms(1); }
       }
    }

    I2C_GenerateSTART( I2C1, ENABLE );

    while( 1 ){
       if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_MODE_SELECT ) ){
            break;
       }
       else{
           time--;
           if( !time ){ return 0; }
           else{ Delay_Ms(1); }
       }
    }

    I2C_Send7bitAddress( I2Cx, i2c_adress, I2C_Direction_Transmitter );

    while( 1 ){
      if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ){
          break;
      }
      else{
          time--;
          if( !time ){ return 0; }
          else{ Delay_Ms(1); }
      }
    }

    if( I2C_ADR_8_BIT == size_adress ){   //(Address_Lenth  == Address_8bit)
        I2C_SendData( I2Cx, (uint8_t)(write_adress & 0x00FF) );
        while( 1 ){
            if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) ){
                break;
            }
            else{
                time--;
                if( !time ){ return 0; }
                else{ Delay_Ms(1); }
            }
        }
    }
    else{   //(Address_Lenth  == Address_16bit)

        I2C_SendData( I2Cx, (uint8_t)(write_adress >> 8) );

        while( 1 ){
            if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) ){
                break;
            }
            else{
                time--;
                if( !time ){ return 0; }
                else{ Delay_Ms(1); }
            }
        }

        I2C_SendData( I2Cx, (uint8_t)(write_adress & 0x00FF) );

        while( 1 ){
            if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) ){
                break;
            }
            else{
                time--;
                if( !time ){ return 0; }
                else{ Delay_Ms(1); }
            }
        }
    }

   //-----------------------------------------------------
   for(uint16_t index = 0; index < size_data; index++){

       while( 1 ){
           if( I2C_GetFlagStatus( I2Cx, I2C_FLAG_TXE ) !=  RESET ){
               break;
           }
           else{
               time--;
               if( !time ){ return 0; }
               else{ Delay_Ms(1); }
           }
        }

        I2C_SendData( I2Cx, *data );
        data++;

        while( 1 ){
            if( I2C_CheckEvent( I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) ){
                break;
            }
            else{
                time--;
                if( !time ){ return 0; }
                else{ Delay_Ms(1); }
            }
        }
    }
    //------------------------------------------------

    I2C_GenerateSTOP( I2Cx, ENABLE );

    return 1;
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


#if 0
    //******************************************************************************
    //---- I2C SCANNER 7-bit adres -------------------------------------------------
    #define TIME_OUT_I2C    10000       // кол-во тиков ( если МК мощный то ставим больше )

    uint32_t time_out = TIME_OUT_I2C;   //переменная ( аналог таймаута ) считает тики

    // в цикле прокручиваем все адреса ( 7 бит без учета нулевого там всегда 0 )
    for( uint8_t adress_i = 0; adress_i < 127; adress_i++ ){
        while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET ){Delay_Ms(1);}        // ждем когда линия будет свободна
        I2C_GenerateSTART( I2C1, ENABLE );                                              // Генерирует состояние СТАРТ связи I2Cx ( стартовый бит )
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) ){Delay_Ms(1);}    // ждем когда включиться режим мастера
        I2C_Send7bitAddress( I2C1, adress_i<<1, I2C_Direction_Transmitter );            // указываем адрес устройства с которым будем общаться по I2C ( и режим прием или передача ) указываем сдвинутый адресс на <<1

        // далее в цикле ждем если эвент отработает то такой адрес есть
        // если в течении TIME_OUT_I2C тиков не отработает то покидаем цикл и заходим на новую итерацию for
         while(1){
            if(I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED )){     // ждем когда включиться режим мастера для передачи
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
#endif

    uint8_t status = 0;

    uint8_t write_data[5] = {8, 1, 2, 3, 5 };
    uint8_t read_data[5] = {0, };

    printf("Start Write 24Cxx....\r\n");
    status = I2C_mem_write( I2C1, I2C_ADR_AT24CXX, 1, I2C_ADR_16_BIT, write_data, sizeof(write_data), 1500);
    printf("Status write: %s\r\n", status ? "OK" : "ERROR");

    status = 0;
    Delay_Ms(500);

    printf("Start Read 24Cxx....\r\n");
    status = I2C_mem_read( I2C1, I2C_ADR_AT24CXX, 1, I2C_ADR_16_BIT, read_data, sizeof(read_data), 1500);
    printf("Status read: %s\r\n", status ? "OK" : "ERROR");
    printf("DATA READ -> %d %d %d %d %d \r\n", read_data[0], read_data[1], read_data[2], read_data[3], read_data[4]);


    Delay_Ms(2000);

    while(1)
    {



    }
}
