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

#define Buf_Size    32      // размер массивов

/* Global Variable */

// массив который будем копировать
uint32_t SRC_BUF[Buf_Size] = {0x01020304, 0x05060708, 0x090A0B0C, 0x0D0E0F10,
                         0x11121314, 0x15161718, 0x191A1B1C, 0x1D1E1F20,
                         0x21222324, 0x25262728, 0x292A2B2C, 0x2D2E2F30,
                         0x31323334, 0x35363738, 0x393A3B3C, 0x3D3E3F40,
                         0x41424344, 0x45464748, 0x494A4B4C, 0x4D4E4F50,
                         0x51525354, 0x55565758, 0x595A5B5C, 0x5D5E5F60,
                         0x61626364, 0x65666768, 0x696A6B6C, 0x6D6E6F70,
                         0x71727374, 0x75767778, 0x797A7B7C, 0x7D7E7F80};

// массив куда будем копировать
uint32_t DST_BUF[Buf_Size] = {0};

uint8_t  Flag = 0;

//***************************************************************************************

/*********************************************************************
 * @fn      BufCmp
 *
 * @brief   Compare the  buf
 *
 * @param   buf1 - pointer of buf1
 *          buf2 - pointer of buf1
 *          buflength - length of buf
 *
 * @return  1 - Two arrays are identical
 *          0 - Two arrays are inconsistent
 */
u8 BufCmp(u32 *buf1, u32 *buf2, u16 buflength)
{
    while(buflength--)
    {
        if(*buf1 != *buf2)
        {
            return 0;
        }
        buf1++;
        buf2++;
    }
    return 1;
}
//***************************************************************************************

//***************************************************************************************
void DMA1_CH3_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SRC_BUF); // указываем адрес (массив) с которого будем копировать
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)DST_BUF;       // указываем адрес (массив) куда будем копировать
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = Buf_Size * 4;                // размер массива ( умножаем на 4 так как данные массива 4 байта а DMA_PeripheralDataSize_Byte выставили 1 байт ( побайтово ) )
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // указываем длину 1 эллемента ( байт, полу слово или слово )
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;     // указываем длину 1 эллемента ( байт, полу слово или слово )
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // указываем тип работы ( однократное или циклическое )
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;

    // чтобы указать нужный канал DMA ( в данном примере DMA1_Channel3 )
    // смотри в референс мануал диаграмму Figure 8-1 DMA1 request image или таблицу Table 8-2 DMA1 peripheral mapping table for each channel
    // там к каждому каналу подключен свой набор переферии и т.д.
    // в режиме MEM2MEM bit можно любой канал
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);                    // DMA1_Channel3
    DMA_ClearFlag(DMA1_FLAG_TC3);                                   // DMA1_FLAG_TC3 - DMA1 Channel3 transfer complete flag.

    DMA_Cmd(DMA1_Channel3, ENABLE);
}
//***************************************************************************************

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

    // инициализируем и запускаем копирование
    DMA1_CH3_Init();

    // ждем конца копирования
    while(DMA_GetFlagStatus(DMA1_FLAG_TC3) == RESET);   // DMA1_FLAG_TC3 - DMA1 Channel3 transfer complete flag.

    // проверяем что все скоприровалось ( два массива )
    Flag = BufCmp(SRC_BUF, DST_BUF, Buf_Size);
    if(Flag == 0)
    {
        printf("DMA Transfer Fail\r\n");
    }
    else
    {
        printf("DMA Transfer Success\r\n");
    }

    // распечатываем два массива
    printf("SRC_BUF:\r\n");
    for(uint16_t i = 0; i < Buf_Size; i++) {
        printf("0x%08lx\r\n", SRC_BUF[i]);
    }

    printf("DST_BUF:\r\n");
    for(uint16_t i = 0; i < Buf_Size; i++){
        printf("0x%08lx\r\n", DST_BUF[i]);
    }

    while(1)
    {


    }
}
