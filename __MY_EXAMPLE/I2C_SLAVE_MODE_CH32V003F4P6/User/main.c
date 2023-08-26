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

#define I2C_ADR_SLAVE          (0x50<<1)        // указываем сдвинутый адресс на <<1

//******************************************************************************
// массив с данными которыми будем работать
// для примера у нас например устройство которое содержит 10 регистров ( 10 байт )
// которые можно читать и перезаписывать
#define MASS_SIZE       10
volatile uint8_t mass_value[MASS_SIZE] = {5, 1, 2, 3, 4, 5, 6, 7, 8, 9 };

// для чтения регистров отправляем сперва номер регистра который хотим прочитать
// ( или с которого хотим читать - читать можно указав начало )
// например хочу прочитать регистр по адресу 2 ( третий по счету )
// uint8_t data = 2;
// отправляю I2C_TX( I2C_ADR_SLAVE, &data, sizeof(data) );
// потом читаю данные регистра или регистров ( указав сколько нужно от указанного адресса )
//uint8_t data[5];
// I2C_RX( I2C_ADR_SLAVE, data, sizeof(data) );
// прочитаю регистр 2 3 4 5 6

// для записи в регистр создаем массив первое значение в нем будет номер регистра в который хотим записать
// ( или начало регистра с которого будем записывать если несколько данных )
// например хочу записать числа 5 8 3 в регистры 2 3 4
// создаю массив uint8_t data[4] = { 2, 5, 8, 3 };
// отправляю I2C_TX( I2C_ADR_SLAVE, data, sizeof(data) );

// читать и писать можно как по 1 байту так и все сразу
// ( адресация идет друг за другом и сама смещаеться на 1 если передавать или читать больше 1 байта

//******************************************************************************

/* Global Variable */

//******************************************************************************
// I2C ( default ) -> PC1=SDA   PC2=SCL - I2C
// I2C ( remap ) -> PC6=SDA   PC5=SCL - I2C_2 I2C_3
// I2C ( remap ) -> PD0=SDA   PD1=SCL - I2C_1
void I2C_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitTSturcture={0};

    NVIC_InitTypeDef  NVIC_InitStructure_EVT = {0};
    NVIC_InitTypeDef  NVIC_InitStructure_ERR = {0};

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

    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
    I2C_DeInit(I2C1 );

    // настройка шины I2C
    I2C_InitTSturcture.I2C_ClockSpeed = 0;                  // частота I2C ( 100 000 Гц или 400 000 Гц ) скорость указывать не нужно для режима SLAVE
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;             // режим I2C
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;     // Рабочий цикл I2C в быстром режиме ( I2C_DutyCycle_16_9 или I2C_DutyCycle_2 )
    I2C_InitTSturcture.I2C_OwnAddress1 = I2C_ADR_SLAVE;     // адрес устройства указываем сдвинутый адресс на <<1
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;            // включаем АСК
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;      // режим адреса 7 или 10 бит
    I2C_Init( I2C1, &I2C_InitTSturcture );

    NVIC_InitStructure_EVT.NVIC_IRQChannel = I2C1_EV_IRQn;         //I2C1 Event Interrupt
    NVIC_InitStructure_EVT.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure_EVT.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure_EVT.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure_EVT);

    NVIC_InitStructure_ERR.NVIC_IRQChannel = I2C1_ER_IRQn;         //I2C1 Error Interrupt
    NVIC_InitStructure_ERR.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure_ERR.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure_ERR.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure_ERR);

    // включаем прерывание на event
    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);

    // запускаем шину I2C
    I2C_Cmd( I2C1, ENABLE );
    I2C_AcknowledgeConfig( I2C1, ENABLE );                  // Включает или отключает указанную функцию подтверждения I2C.
    I2C_StretchClockCmd(I2C1, ENABLE);
}
//******************************************************************************


//******************************************************************************
// отчиска флагов для корректной работы
void I2C1_ClearFlag(void) {
  // ADDR-Flag clear
  while((I2C1->STAR1 & I2C_FLAG_ADDR) == I2C_FLAG_ADDR) {
    I2C1->STAR1;
    I2C1->STAR2;
  }

  // STOPF Flag clear
  while((I2C1->STAR1&I2C_FLAG_STOPF) == I2C_FLAG_STOPF) {
    I2C1->STAR1;
    I2C1->CTLR1 |= 0x1;
  }
}
//******************************************************************************

//******************************************************************************
void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void){

    uint32_t event = 0;
    uint8_t read_ferst_adress = 0;

    static uint8_t index_rx_byte = 255;
    static uint8_t index_tx_byte = 0;
    static uint8_t check_rx_adress_ok = 0;
    static uint8_t check_tx_adress_ok = 0;

    // читаем текущий эвент
    event = I2C_GetLastEvent(I2C1);

    if(event == I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED){  // эвент если адресс шины I2C на прием нашего устройства совпал
        //printf("I2C I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED!!!\r\n");

        check_rx_adress_ok = 1;    // если адресс совпал, отмечаем
    }
    else if(event == I2C_EVENT_SLAVE_BYTE_RECEIVED){   // эвент если пришел 1 байт
        //printf("I2C I2C_EVENT_SLAVE_BYTE_RECEIVED!!!\r\n");

        // если наш адресс то читаем
        if( check_rx_adress_ok == 1 ){
            // первый байт будет ето адресс регистра ( 8 байт ) ( если нужно 16 то переделать нужно )
            // поэтому индекс равен -1 чтоб определять был первый или нет
            if( index_rx_byte == 255 ){
                if( I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE ) !=  RESET ){
                        read_ferst_adress = I2C_ReceiveData(I2C1);
                        index_rx_byte = read_ferst_adress;
                        index_tx_byte = read_ferst_adress;
                 }
            }
            else{
                if( I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE ) !=  RESET ){
                    if( index_rx_byte < MASS_SIZE ){
                        mass_value[index_rx_byte] = I2C_ReceiveData(I2C1);
                        index_rx_byte++;
                    }
                    else{
                        // если указали адресс больше чем массив то просто читает чтобы очистить буфер
                        I2C_ReceiveData(I2C1);
                    }
                }
            }
        }
    }
    else if(event == I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED){ // эвент если адресс шины I2C на передачу нашего устройства совпал
        //printf("I2C I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED!!!\r\n");

        check_tx_adress_ok = 1;    // если адресс совпал, отмечаем
        // так как адрес наш отправляем 1 байт
        //( после отправки сработает эвент I2C_EVENT_SLAVE_BYTE_TRANSMITTED в нем и продолжаем отсылать если нужно отправить больше 1 байта
        if( I2C_GetFlagStatus( I2C1, I2C_FLAG_TXE ) !=  RESET && I2C_GetFlagStatus( I2C1, I2C_FLAG_BTF) == RESET){

            I2C_SendData(I2C1, mass_value[index_tx_byte]);  // отправляем тот байт который пришел в виде адресса регистра

            I2C_ClearFlag(I2C1, I2C_FLAG_BTF);
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);

            I2C1_ClearFlag();
        }
    }
    // эвент ( после отправки сработает эвент I2C_EVENT_SLAVE_BYTE_TRANSMITTED в нем и продолжаем отсылать если нужно отправить больше 1 байта
    // после каждой отправки уже с данного євента генерируеться новый евент до тех пор пока не подымим сигнал СТОП
    else if(event == I2C_EVENT_SLAVE_BYTE_TRANSMITTED){
        //printf("I2C I2C_EVENT_SLAVE_BYTE_TRANSMITTED!!!\r\n");

        if( check_tx_adress_ok == 1 ){  // если адресс совпал и есть еще байты то отправляем

            // отправляем оставшиеся биты ( от указаного до MASS_SIZE ) так как запрашиваемое устройство
            // когда запрашивает и читает отправленные биты в конце не подымает бит СТОП и мы не знаем когда прекратить отправку
            // поетому отправляем все а там уже сколько прочитают
            // в конце отправки мы должны сгенерировать сигнал СТОП чтоб принимаемое устройство знало что конец
            if( I2C_GetFlagStatus( I2C1, I2C_FLAG_TXE ) !=  RESET && I2C_GetFlagStatus( I2C1, I2C_FLAG_BTF) == SET){
                index_tx_byte++;
                I2C_SendData(I2C1, mass_value[index_tx_byte]);
                I2C_ClearFlag(I2C1, I2C_FLAG_BTF);

                I2C1_ClearFlag();

                if( !(index_tx_byte < MASS_SIZE) ){
                    check_tx_adress_ok = 0;
                    index_tx_byte = 0;

                    I2C_GenerateSTOP(I2C1, ENABLE);
                    I2C1_ClearFlag();
                    I2C_Cmd(I2C1, ENABLE);
                }
            }
        }
    }
    else if(event == I2C_EVENT_SLAVE_STOP_DETECTED){    // эвент генерации сигнала стоп
        //printf("I2C I2C_EVENT_SLAVE_STOP_DETECTED!!!\r\n");

        check_rx_adress_ok = 0;
        index_rx_byte = 255;

        I2C1_ClearFlag();
        I2C_Cmd(I2C1, ENABLE);
    }
}
//******************************************************************************

//******************************************************************************
void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void){

    //printf("I2C ERROR!!!\r\n");

    if (I2C_GetITStatus(I2C1, I2C_IT_AF)) {
       I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
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

    printf("I2C SLAVE MODE\r\n");

    // скорость указывать не нужно для режима SLAVE
    I2C_Config();


    while(1)
    {


    }
}
