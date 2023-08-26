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
volatile uint32_t coun_value = 0;


// биты регистров SYSTICK
#define SYSTICK_CTLR_STE        (1<<0)      // System counter enable control bit.(1: Start the system counter STK.) (0: Turn off the system counter STK and thecounter stops counting.)
#define SYSTICK_CTLR_STIE       (1<<1)      // Counter interrupt enable control bit. (1: Enabling counter interrupts.) (0: Turn off the counter interrupt.)
#define SYSTICK_CTLR_STCLK      (1<<2)      // Counter clock source selection bit. (1: HCLK for time base.) (0: HCLK/8 for time base.) ( можно поделить на 8 таймер )
#define SYSTICK_CTLR_STRE       (1<<3)      // Auto-reload Count enable bit. (1: Count up to the comparison value and startcounting from 0 again)  (0: Continue counting up.)
#define SYSTICK_CTLR_SWIE       (1<<31)     // Software interrupt trigger enable (SWI). (1: Triggering software interrupts.) (0: Turn off the trigger.)



//*********************************************************************
// обработчик прерывания просто печатаем надпись,
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    coun_value++;
    printf("Systick %u\r\n", coun_value);
    SysTick->SR = 0;            // Флаг сравнения значения счетчика, очистить запись 0
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

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);

    //------------------------------------------------------------
    NVIC_EnableIRQ(SysTicK_IRQn);           // включаем прерывания

    // Флаг сравнения значения счетчика, очистить запись 0,
    // 1: счет вверх для достижения значения сравнения.
    // 0: значение сравнения не достигнуто.
    SysTick->SR &= ~(1 << 0);

    // System count comparison value register ( незабываем ставить -1 от расчетного значения )
    //SysTick->CMP = SystemCoreClock - 1;       // значение счетчика ( для переполнения ) ( расчитан на 1 секунду )
    SysTick->CMP = (SystemCoreClock/1000) - 1;  // значение счетчика ( для переполнения ) ( расчитан на 1 миллисекунду )

    // System counter register
    SysTick->CNT = 0;                       // 32-bit счетчик ( обнуляем )

    // System count control register
    SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE | SYSTICK_CTLR_STCLK | SYSTICK_CTLR_STRE;      //Enable SysTick counter, IRQ, HCLK/1, Auto-reload Count
    //------------------------------------------------------------


    while(1)
    {

    }
}
