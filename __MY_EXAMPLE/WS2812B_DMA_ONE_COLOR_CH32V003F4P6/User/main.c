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


#define LED_COUNT       10

const uint8_t led_color[LED_COUNT][4] ={
        {255, 0, 0, 255},   // R G B brightness
        {255, 0, 0, 10},   // R G B brightness
        {0, 255, 0, 255},   // R G B brightness
        {0, 255, 0, 10},   // R G B brightness
        {0, 0, 255, 255},   // R G B brightness
        {0, 0, 255, 10},   // R G B brightness
        {255, 0, 0, 255},   // R G B brightness
        {255, 0, 0, 255},   // R G B brightness
        {255, 0, 0, 10},   // R G B brightness
        {255, 0, 0, 255}   // R G B brightness
};

// ИСПОЛЬЗОВАТЬ с ВНЕШНИМ КВАРЦЕМ так как на внутреней RC частота плавает

// ШИМ нужно генерировать шим на частоте 800 КГц
// частота 48 000 000 / 800 000 = 60 тиков на одну итерацию ШИМ ( разрядность ШИМ )

//для передачи данных:
//
//Под каждый бит отводится определённое время — 1,25 микросекунды.
//
//Если мы передаём ноль, то сначала мы устанавливаем высокий уровень сигнала,
//держим его в таком состоянии 0,35 микросекунды
//(допускается отклонение 150 наносекунд или 0,15 микросекунды).
//По прошествии данного времени мы устанавливаем на ножке низкий
//уровень и держим его до передачи следующего бита 0,9 микросекунды (
//допускается такое же отклонение — 150 наносекунд)
//
//Единица передаётся наоборот. Мы также устанавливаем высокий уровень,
//ждём 0,9 микросекунды (отклонение то же), затем опускаем уровень,
//ждём 0,35 микросекунды (отклонение такое же — 150 наносекунд)

// расчитываем сколько тиков нужно на длинный импульс и на короткий
// Общий период бита у нас
// составляет 1,25 микросекунды. Поэтому мы получим пропорцию: при
// периоде 1,25 — 0,35 мкс, а при 60 тиков — X. Отсюда X = 60 * 0.35 / 1.25,
// что составит примерно 16,8 = 17 тиков на короткий импульс
// на длинный 60 - 17 = 43 тика

// ВНАЧАЛЕ КАЖДОГО НОВОГО ПАКЕТА данных делаем задержку ( отправляем шим 0 ) в течении ( более ) 50 микросекунд

// Зададим данные значения через #define
// данные значения можно немного менять для стабильности
#define HIGH    29  // % заполнение шим на 1 (43)
#define LOW     15  // % заполнение шим на 0 (17)


//------------------------------------------------------------------
// Этот пример демонстрирует использование DMA для вывода ШИМ через вывод TIM1_CH1 (PD2).
// ОДИНОЧНЫЙ РЕЖИМ ( через указаное время пуляем данные в ШИМ )
//------------------------------------------------------------------

//------------------------------------------------------------------
// CH1CVR register Definition
// адресс берем из реферанс мануал таблица Table 10-3 TIM1-related registers list
// R16_TIM1_CH1CVR 0x40012C34 Compare/capture register 1 0x0000
#define TIM1_CH1CVR_ADDRESS    0x40012C34
//------------------------------------------------------------------

//------------------------------------------------------------------
// данные которые будем отправлять
uint8_t pbuf[24+1] = {0, };
//------------------------------------------------------------------

//***************************************************************************
#define BitIsSet(reg, bit)  ((reg&(1<<bit))!=0) //  макрос установки бита в каком-либо числе

// функция заполнения нашего массива данными
void ws2812_pixel_rgb_to_buf_dma(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint8_t brightness){

        if(brightness) { // See notes in setBrightness()
              Rpixel = (Rpixel * brightness) >> 8;
              Gpixel = (Gpixel * brightness) >> 8;
              Bpixel = (Bpixel * brightness) >> 8;
        }

      for(int8_t i = 0; i < 8; i++){
        if (BitIsSet(Rpixel,(7-i)) == 1){
            pbuf[i+8] = HIGH;
        }
        else{
            pbuf[i+8] = LOW;
        }
        if (BitIsSet(Gpixel,(7-i)) == 1){
            pbuf[i+0] = HIGH;
        }
        else{
            pbuf[i+0] = LOW;
        }
        if (BitIsSet(Bpixel,(7-i)) == 1){
            pbuf[i+16] = HIGH;
        }
        else{
            pbuf[i+16] = LOW;
        }
      }
}
//***************************************************************************

/**************************************************************************
/*********************************************************************
 * @fn      TIM1_PWMOut_Init
 *
 * @brief   Initializes TIM1 PWM output.
 *
 * @param   arr - the period value.
 *          psc - the prescaler value.
 *          ccp - the pulse value.
 *
 * @return  none
 */
void TIM1_PWMOut_Init(uint16_t arr, uint16_t psc, uint16_t ccp)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    TIM_OCInitTypeDef       TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    // включаем тактирования таймера и порта D TIM1_CH1 (PD2)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1, ENABLE);         // port D

    // настраиваем пин
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                                           // pin 2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);                                              // port D

    // настраиваем таймер
    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    // настраиваем режим ШИМ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                                     // прямой ШИМ ( сколько импульс столько и вначале положительный фронт )

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);                                                // TIM_OC1.2.3.4 ..... номер канала

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);                                      // TIM_OC1.2.3.4 ..... номер канала
    TIM_ARRPreloadConfig(TIM1, ENABLE);
}
//**************************************************************************

//**************************************************************************
/*********************************************************************
 * @fn      TIM1_DMA_Init
 *
 * @brief   Initializes the TIM DMAy Channelx configuration.
 *
 * @param   DMA_CHx -
 *            x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void TIM1_DMA_Init(DMA_Channel_TypeDef *DMA_CHx, uint32_t ppadr, uint32_t memadr, uint16_t bufsize)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    // включаем тактирование DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // настраиваем DMA
    // чтобы указать нужный канал DMA TIM1_UP( в данном примере DMA1_Channel5 )
    // смотри в референс мануал диаграмму Figure 8-1 DMA1 request image или таблицу Table 8-2 DMA1 peripheral mapping table for each channel
    // там к каждому каналу подключен свой набор переферии и т.д.
    // в режиме MEM2MEM bit можно любой канал
    DMA_DeInit(DMA_CHx);                                                        // деинециализируем DMA указав канал от 1 до 7
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;                           // адрес переферии  ( смотри реферанс мануал )
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;                              // адрес памяти которую будем отправлять ( в данном примере ето массив uint16_t pbuf[4] = {10, 50, 80, 0}; )
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;                                 // размер отправляемых данных  ( не байт а штук ) зависит от DMA_PeripheralDataSize_HalfWord и DMA_MemoryDataSize_HalfWord
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // ( c PWM 8бит не работает только 16 ) выставляем размер 1 пакета ( 8 16 32 байта )
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;         // ( можно 8 бит если массив данных 8 битный ) выставляем размер 1 пакета ( 8 16 32 байта )
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               // отправка циклическая или единоразовая
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                // режим память в память отключаем ( так как отправляем в переферию )
    DMA_Init(DMA_CHx, &DMA_InitStructure);

}
//**************************************************************************

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

    // ШИМ нужно генерировать шим на частоте 800 КГц ( счетчик 60  предделитель 1  )
    TIM1_PWMOut_Init(60-1, 1-1, 0);

    // указываем параметры для DMA
    // чтобы указать нужный канал DMA TIM1_UP( в данном примере DMA1_Channel5 )
    // смотри в референс мануал диаграмму Figure 8-1 DMA1 request image или таблицу Table 8-2 DMA1 peripheral mapping table for each channel
    // там к каждому каналу подключен свой набор переферии и т.д.
    // в режиме MEM2MEM bit можно любой канал
    TIM1_DMA_Init(DMA1_Channel5, (u32)TIM1_CH1CVR_ADDRESS, (u32)pbuf, sizeof(pbuf)/sizeof(pbuf[0]));        // sizeof(pbuf)/sizeof(pbuf[0]) - кол-во пакетов ( кол-во массива )

    // запускаем перезапуск DMA после обновления таймера
    TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);

    // включаем таймер
    TIM_Cmd(TIM1, ENABLE);


    while(1)
    {

        // включаем PWM
        TIM_CtrlPWMOutputs(TIM1, ENABLE);

        // так как выше включили ШИМ и там сигнал 0 ( так как последний эллемент массива 0 )
        // по даташиту ждем не мение  50 микросекунд ( что означает что новый пакет данных )
        Delay_Us(60);


        for( uint16_t i = 0; i < LED_COUNT; i++){

            //ws2812_pixel_rgb_to_buf_dma(255, 0, 0, 255);
            ws2812_pixel_rgb_to_buf_dma(led_color[i][0], led_color[i][1], led_color[i][2], led_color[i][3] );

            // указываем сколько пакетов нужно передать ( указывать нужно каждый раз перед отправкой )
            DMA_SetCurrDataCounter(DMA1_Channel5, sizeof(pbuf)/sizeof(pbuf[0]) );  // sizeof(pbuf)/sizeof(pbuf[0]) - кол-во пакетов ( кол-во массива )

            // включаем ( запускаем DMA )
            DMA_Cmd(DMA1_Channel5, ENABLE);

            // ждем когда все пакеты будут отправлены ( флаг отрабатывает когда данные были переданы в но ШИМ ( последний ) еще не отработал
            // поэтому в массиве в конце поставил 0 ( передаст все кроме нуля ( последнего значения )
            // если в конце стоит 0 то можно и не отключать PWM в коне передачи ( и включить его всего 1 раз в настройках )
            while(DMA_GetFlagStatus(DMA1_FLAG_TC5) != SET); // DMA1_FLAG_TC5 - DMA1 Channel5 transfer complete flag.

            // выключаем DMA
            DMA_Cmd(DMA1_Channel5, DISABLE);

            // сбрасываем флаг ( завершения отправки )
            DMA_ClearFlag(DMA1_FLAG_TC5);
        }

        // выключаем PWM
        TIM_CtrlPWMOutputs(TIM1, DISABLE);

        Delay_Ms(2000);
    }
}
