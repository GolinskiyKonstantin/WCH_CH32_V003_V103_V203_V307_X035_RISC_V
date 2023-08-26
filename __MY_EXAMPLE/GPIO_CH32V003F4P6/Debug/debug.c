/********************************** (C) COPYRIGHT  *******************************
 * File Name          : debug.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : This file contains all the functions prototypes for UART
 *                      Printf , Delay functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include <debug.h>

//*********************************************************************

/////////////////////////////////////////////////////////////////////////////////

// some bit definitions for systick regs
#define SYSTICK_SR_CNTIF        (1<<0)
#define SYSTICK_CTLR_STE        (1<<0)
#define SYSTICK_CTLR_STIE       (1<<1)
#define SYSTICK_CTLR_STCLK      (1<<2)
#define SYSTICK_CTLR_STRE       (1<<3)
#define SYSTICK_CTLR_SWIE       (1<<31)

//***************************************************************************

// §£§í §Þ§à§Ø§Ö§ä§Ö §Ú§ã§á§à§Ý§î§Ù§à§Ó§Ñ§ä§î SYSTICK_USE_HCLK, §Ö§ã§Ý§Ú §Ó§í §ï§ä§à §ã§Õ§Ö§Ý§Ñ§Ö§ä§Ö, §å §Ó§Ñ§ã §Ò§å§Õ§Ö§ä §Ó§í§ã§à§Ü§à§Ö §â§Ñ§Ù§â§Ö§ê§Ö§ß§Ú§Ö
// §à§Õ§ß§Ñ§Ü§à §ï§ä§à §à§Ô§â§Ñ§ß§Ú§é§Ú§ä §Ó§Ñ§ê§å §Þ§Ñ§Ü§ã§Ú§Þ§Ñ§Ý§î§ß§å§ð §Ù§Ñ§Õ§Ö§â§Ø§Ü§å §Õ§à 44 §ã§Ö§Ü§å§ß§Õ, §á§â§Ö§Ø§Õ§Ö §é§Ö§Þ §à§ß§Ñ §Ù§Ñ§Ó§Ö§â§ê§Ú§ä§ã§ñ
// §Ó§à§Ü§â§å§Ô. §£§í §ä§Ñ§Ü§Ø§Ö §Õ§à§Ý§Ø§ß§í §Ó§í§Ù§Ó§Ñ§ä§î SETUP_SYSTICK_HCLK.


#define DELAY_US_TIME ((SystemCoreClock)/1000000)
#define DELAY_MS_TIME ((SystemCoreClock)/1000)

//// Use systick = hclk/8
//#define DELAY_US_TIME ((SystemCoreClock)/8000000)
//#define DELAY_MS_TIME ((SystemCoreClock)/8000)
//***************************************************************************

volatile uint32_t systick_cnt;

static void DelaySysTick( uint32_t n );

//***************************************************************************
//Start up the SysTick IRQ
void SysTick_Init(void)
{
    SysTick->CTLR = 0;                              //disable default SysTick behavior
    NVIC_EnableIRQ(SysTicK_IRQn);                   //enable the SysTick IRQ
    SysTick->CMP = (SystemCoreClock/1000)-1;        //Set the tick interval to 1ms for normal op
    SysTick->CNT = 0;                               //Start at zero
    systick_cnt = 0;
    SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE | SYSTICK_CTLR_STCLK;      //Enable SysTick counter, IRQ, HCLK/1
}
//***************************************************************************

//***************************************************************************
// SysTick ISR just counts ticks
// note - the __attribute__((interrupt)) syntax is crucial!
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
    // move the compare further ahead in time.
    // as a warning, if more than this length of time
    // passes before triggering, you may miss your
    // interrupt.
    SysTick->CMP += (SystemCoreClock/1000);
    SysTick->SR = 0;        //clear IRQ
    systick_cnt++;          //update counter
}
//***************************************************************************

//***************************************************************************
static void DelaySysTick( uint32_t n )
{
    uint32_t targend = SysTick->CNT + n;
    while( ((int32_t)( SysTick->CNT - targend )) < 0 );
}
//***************************************************************************

//***************************************************************************
uint32_t GetTick()
{
    return systick_cnt;
}
//***************************************************************************

//*********************************************************************
void Delay_Us(uint32_t time_us)
{
    DelaySysTick( time_us * DELAY_US_TIME );
}
//*********************************************************************

//*********************************************************************
void Delay_Ms(uint32_t time_ms)
{
    DelaySysTick( time_ms * DELAY_MS_TIME );
}
//*********************************************************************
/////////////////////////////////////////////////////////////////////////////////

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
    return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}



