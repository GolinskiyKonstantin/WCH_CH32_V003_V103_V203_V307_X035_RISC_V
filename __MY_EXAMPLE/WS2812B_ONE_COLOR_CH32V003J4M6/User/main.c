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

// Под каждый бит отводится определённое время — 1,25 микросекунды.
// 1 светодиод 24 бита

// пля передачи нуля подымаем пин на 0,4 микросекунды потом опускаем на 0,85 микросекунд ( погрешность +- 0,125 )
// для передачи еденицы подымаем пин на 0,8 микросекунды потом опускаем на 0,45 микросекунд ( погрешность +- 0,125 )
// частота 800КГц получаеться

// обязательно перед началом оправки данных прижать пин к земле на 50 и более микросекунд

// в данном примере подключаем сигнальный пин светодиода к (PC4)


//*******************************************************************************************
void LED_SendBit(uint8_t bit)
{
    if (bit) {
    //// Send a 1 bit
        GPIOC->BSHR = 1 << 4; // put pin C4 high and wait for 800ns
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");
        GPIOC->BCR = 1 << 4; // put pin C4 low and exit, the rest of the 1.2us is taken up by other functions
        return;
        }
    else {
        // Send a 0 bit
        GPIOC->BSHR = 1 << 4; // put pin C4 high and wait for 400ns
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");
        GPIOC->BCR = 1 << 4; // put pin C4 low and wait intil 1.2us
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
            __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    }
}
//*******************************************************************************************

//*******************************************************************************************
void LED_SendColour(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
{
    if(brightness) { // See notes in setBrightness()
        red = (red * brightness) >> 8;
        green = (green * brightness) >> 8;
        blue = (blue * brightness) >> 8;
    }

    // Send the green component first (MSB)
    for (int i = 7; i >= 0; i--) {
        LED_SendBit((green >> i) & 1);
    }
    // Send the red component next
    for (int i = 7; i >= 0; i--) {
        LED_SendBit((red >> i) & 1);
    }
    // Send the blue component last (LSB)
    for (int i = 7; i >= 0; i--) {
        LED_SendBit((blue >> i) & 1);
    }
}
//*******************************************************************************************

//*******************************************************************************************
void WS2812B_GPIO_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};

    // включаем тактирования порта (PC4)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);                               // port C

    // настраиваем пин
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;                                           // pin 4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);                                              // port C

}
//**************************************************************************


int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);


    WS2812B_GPIO_Init();

//    //----------------------------------
//    // отчищаем 24 светодиода
//    Delay_Us(50);
//    for(uint16_t i = 0; i< 24; i++){
//        LED_SendColour(0, 0, 0, 0);
//    }
//    //----------------------------------

//    //----------------------------------
//    // пример зажигаем 24 светодиода зеленым светом
//    Delay_Us(50);
//    for(uint16_t i = 0; i< 24; i++){
//        LED_SendColour(0, 255, 0, 255);
//    }
//    //----------------------------------


    while(1)
    {
        Delay_Us(50);
        for(uint16_t i = 0; i< 24; i++){
            LED_SendColour(0, 255, 0, 255);
        }

    }
}
