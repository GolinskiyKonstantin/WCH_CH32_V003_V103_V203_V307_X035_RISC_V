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


/* Global Variable */

//*********************************************************************
// иницеализируем кнопку PC5 ( будем по нажатию на нее сбрасывать сторожевой таймер )
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE ); // port C

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;       // pin 5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // input pull up
    GPIO_Init( GPIOC, &GPIO_InitStructure);         // port C
}
//*********************************************************************

//*********************************************************************
// настройка сторожевого таймера
// по даташиту он тактируеться от LSI RC 128kHz
/*********************************************************************
 * @fn      IWDG_Init
 *
 * @brief   Initializes IWDG.
 *
 * @param   IWDG_Prescaler: specifies the IWDG Prescaler value.
 *            IWDG_Prescaler_4: IWDG prescaler set to 4.
 *            IWDG_Prescaler_8: IWDG prescaler set to 8.
 *            IWDG_Prescaler_16: IWDG prescaler set to 16.
 *            IWDG_Prescaler_32: IWDG prescaler set to 32.
 *            IWDG_Prescaler_64: IWDG prescaler set to 64.
 *            IWDG_Prescaler_128: IWDG prescaler set to 128.
 *            IWDG_Prescaler_256: IWDG prescaler set to 256.
 *          Reload: specifies the IWDG Reload value.
 *            This parameter must be a number between 0 and 0x0FFF.
 *
 * @return  none
 */
void IWDG_Feed_Init(uint16_t prer, uint16_t rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(prer);                        // предделитель
    IWDG_SetReload(rlr);                            // счетчик сторажевого таймера ( ограницение значений от number between 0 and 0x0FFF (4095) )
    IWDG_ReloadCounter();                           // сброс сторожевого таймера
    IWDG_Enable();                                  // запуск сторожевого таймера
}
//*********************************************************************

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

    SysTick_Init();

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);

    KEY_Init();

    // по даташиту он тактируеться от LSI RC 128kHz
    // поставив предделитель на 128 получаем  128 000 / 128 = 1000 ( 1 миллисекунда )
    // второе значение ставим счетчик сторажевого таймера ( ограницение значений от number between 0 and 0x0FFF (4095) )
    IWDG_Feed_Init( IWDG_Prescaler_128, 4000 );   // ставим счетчик на 4000 получаем 4 секунды IWDG reset

    printf("---- START ------\r\n");

    while(1)
    {
        // если нажата кнопка то сбрасываем сторжевой таймер
        if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5) == Bit_RESET )          //PC5
        {
            printf("Feed dog\r\n");
            IWDG_ReloadCounter();       // сброс сторожевого таймера
        }
        else
        {
            printf("Don't feed dog\r\n");
        }
        Delay_Ms(200);
    }
}
