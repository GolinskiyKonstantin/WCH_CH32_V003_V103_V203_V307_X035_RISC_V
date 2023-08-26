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

#include "debug.h"


/* Global define */


/* Global Variable */

//*********************************************************************
void GPIO_Pin_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // INPUT INIT -----------------------------------------------
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    //-----------------------------------------------------------

    // OUTPUT INIT ----------------------------------------------
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    //-----------------------------------------------------------
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


    GPIO_Pin_Init();

    while(1)
    {
      //-------------------------------------------------------------------------------------
      // все функции по работе GPIO смотри в файле Peripheral -> inc -> ch32v00x_gpio.h
      //-------------------------------------------------------------------------------------

      //-- переключаем по Delay ---------------------------------
//            //GPIO_SetBits(GPIOD, GPIO_Pin_2);
//            GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);
//            Delay_Ms(500);
//            //GPIO_ResetBits(GPIOD, GPIO_Pin_2);
//            GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);
//            Delay_Ms(500);
      //---------------------------------------------------------

      //-- переключаем через GetTick() --------------------------
            static uint32_t count = 0;
            static uint8_t st = 0;

            if( GetTick() - count > 1000 ){
                count = GetTick();
                st = !st;
                if( st ){
                    GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_SET);
                }
                else{
                    GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_RESET);
                }
            }
       //---------------------------------------------------------

       //-- работаем с кнопкой -----------------------------------
            if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5) == Bit_SET ){
                GPIO_ResetBits(GPIOD, GPIO_Pin_2);
            }
            else{
                GPIO_SetBits(GPIOD, GPIO_Pin_2);
            }
       //---------------------------------------------------------

       //-- Toggle -----------------------------------------------
       //GPIO_WriteBit(GPIOD, GPIO_Pin_2, !GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2));
       //---------------------------------------------------------
    }
}
