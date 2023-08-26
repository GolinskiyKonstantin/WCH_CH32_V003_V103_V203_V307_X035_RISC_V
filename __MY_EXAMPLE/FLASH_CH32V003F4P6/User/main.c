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

// для работы нам нужно знать как распологаеться память ( сколько страниц и их размер )
// эти данные можно глянуть в документе CH32V003RM.pdf в таблице Table 16-1 Flash Memory Organization
// у мк ch32v003 256 страниц ( от 0 до 255 ) и все страницы по 64 байта ( начало адресации 0x0800 0000 )

/* Global define */

// энум для статусов
typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

#define PAGE_WRITE_START_ADDR          ((uint32_t)0x08003FC0) // Start from 255 page
#define PAGE_WRITE_END_ADDR            ((uint32_t)0x08004000) // End at 255 page ( 0x08003FFF пишем конечный адрес + 1 )
#define FLASH_PAGE_SIZE                64                     // размер 1 страницы ( в байтах )

// память блокируеться несколькоми страницами сразу выбираем нужную ( смотри define )
#define FLASH_PAGES_TO_BE_PROTECTED    FLASH_WRProt_Pages240to255   // CH32 Medium-density devices: Write protection of page 240 to 255

/* Global Variable */
volatile uint32_t              EraseCounter = 0x0, Address = 0x0;
volatile uint16_t              Data = 0xA1A4;       // данные которыми будем записывать всю страницу
volatile uint32_t              WRPR_Value = 0xFFFFFFFF, ProtectedPages = 0x0;
volatile uint32_t              NbrOfPage;

volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
volatile TestStatus MemoryProgramStatus = PASSED;
volatile TestStatus MemoryEraseStatus = PASSED;

//*********************************************************************
//void Option_Byte_CFG(void)
//{
//    FLASH_Unlock();
//    FLASH_EraseOptionBytes();
//    FLASH_UserOptionByteConfig(OB_IWDG_SW, OB_STOP_NoRST, OB_STDBY_NoRST, OB_RST_EN_DT12ms);
//    FLASH_Lock();
//}
//*********************************************************************

//*********************************************************************
// отчиска памяти и запись данных с кучами проверок
void Flash_Test(void)
{
    FLASH_Unlock();     // разблокируем память
    WRPR_Value = FLASH_GetWriteProtectionOptionByte();  // получаем данные о защите памяти

    // расчитываем кол-во страниц
    NbrOfPage = (PAGE_WRITE_END_ADDR - PAGE_WRITE_START_ADDR) / FLASH_PAGE_SIZE;

    // если память не заблокирована
    if((WRPR_Value & FLASH_PAGES_TO_BE_PROTECTED) != 0x00)
    {
        FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);

        // очищаем память и печатаем сколько страниц было очищено или ошибка отчистки
        for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
        {
            FLASHStatus = FLASH_ErasePage(PAGE_WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
            if(FLASHStatus != FLASH_COMPLETE)
            {
                printf("FLASH Erase ERR at Page %ld\r\n", (PAGE_WRITE_START_ADDR + (EraseCounter * FLASH_PAGE_SIZE) - 0x08000000)/FLASH_PAGE_SIZE );
            }
            printf("FLASH Erase Page %ld ...\r\n", (PAGE_WRITE_START_ADDR + (EraseCounter * FLASH_PAGE_SIZE) - 0x08000000)/FLASH_PAGE_SIZE );
        }

        // печатаем что проверяем была ли очищена память
        printf("Erase Checking...\r\n");

        // берем начальный адрес для записи
        Address = PAGE_WRITE_START_ADDR;

        // бежим по памяти проверяем была ли она очищена ( при очистке везде записываеться 1 )
        while((Address < PAGE_WRITE_END_ADDR) && (MemoryEraseStatus != FAILED))
        {
            if((*(__IO uint16_t *)Address) != 0xFFFF)
            {
                MemoryEraseStatus = FAILED;
            }
            Address += 2;   // так как пишем по 16 бит то смещаем адрес + 2
        }

        // печатаем результат проверки отчистки памяти ( была очищена или нет )
        if(MemoryEraseStatus == FAILED)
        {
            printf("Erase Flash FAIL!\r\n");
            printf("\r\n");
        }
        else
        {
            printf("Erase Flash PASS!\r\n");
            printf("\r\n");
        }

        // начинаем в память писать свои данные
        Address = PAGE_WRITE_START_ADDR; // берем начальный адрес для записи
        printf("Programing...\r\n");
        while((Address < PAGE_WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
        {
            FLASHStatus = FLASH_ProgramHalfWord(Address, Data); // печатаем в формате HalfWord значение Data
            Address = Address + 2; // так как пишем по 16 бит то смещаем адрес + 2
        }

        // печатаем что делаем проверку было ли коректно записаны данные
        printf("Program Checking...\r\n");
        Address = PAGE_WRITE_START_ADDR; // берем начальный адрес для записи

        // в цикле на всю указаную память записываем одно значение подрят
        while((Address < PAGE_WRITE_END_ADDR) && (MemoryProgramStatus != FAILED))
        {
            if((*(__IO uint16_t *)Address) != Data)
            {
                MemoryProgramStatus = FAILED;
            }
            Address += 2;
        }
        if(MemoryProgramStatus == FAILED)
        {
            printf("Memory Program FAIL!\r\n");
            printf("\r\n");
        }
        else
        {
            printf("Memory Program PASS! Data 0x%X \r\n", Data);
            printf("\r\n");
        }
    }
    else
    {
        // печатаем если память оказалась заблокирована
        MemoryProgramStatus = FAILED;
        printf("Error to program the flash : The desired pages are write protected\r\n");
    }

    FLASH_Lock();   // блокируем флеш память обратно
}
//*************************************************************************

//*************************************************************************
// быстрая запись работаем с 1 страницей
void Flash_Test_Fast(void)
{
    uint8_t Verify_Flag = 0;
    uint16_t buf[16];

    // набиваем данные в массив от 0 до 15
    for(uint8_t i = 0; i < 16; i++){
        buf[i] = i;
    }

    FLASH_Unlock_Fast();    // разблокирываем память

    FLASH_ErasePage_Fast(PAGE_WRITE_START_ADDR);    // очищаем страницу

    printf("64Byte Page Erase PASS\r\n");

    FLASH_BufReset();   // сброс буфера памяти

    // записываем в буфер памяти данные
    for(uint8_t i=0; i<16; i++){
        FLASH_BufLoad(PAGE_WRITE_START_ADDR + 4 * i, buf[i]);
    }

    // записываем из буфера в память данные
    FLASH_ProgramPage_Fast(PAGE_WRITE_START_ADDR);

    printf("64Byte Page Program PASS\r\n");

    FLASH_Lock_Fast();  // блокируем память обратно

    // читаем из памяти и проверяем с теми данными которые записывали
    for(uint8_t i = 0; i < 16; i++){
        if(buf[i] == *(uint32_t*)(PAGE_WRITE_START_ADDR + 4 * i))
        {
            Verify_Flag = 0;
        }
        else
        {
            Verify_Flag = 1;
            break;
        }
    }

    // печатаем результат чтения
    if(Verify_Flag)
        printf("64Byte Page Verify FAIL\r\n");
    else
        printf("64Byte Page Verify PASS\r\n");
}
//****************************************************************

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

    Flash_Test();
    //Flash_Test_Fast();

//    //*****************************************************************************************************
//    // защита чтения памяти при ENABLE блокирует ( DISABLE наоборот разблокирует но стирает память )
//    // если память заблокирована на чтение то снимаем блокировку и память будеть ОЧИЩЕННА
//    if ( FLASH_GetReadOutProtectionStatus() == SET ){
//        printf("FLASH READ Protected:\r\n");
//
//        //FLASH_Unlock();     // разблокируем память
//        //FLASH_ReadOutProtection(DISABLE);
//        //FLASH_Lock();   // блокируем флеш память обратно
//
//        //printf("FLASH READ >> NOT Protected:\r\n");
//    }
//    else{   // если память не заблокирована то блокируем ее ( делаем проверку чтобы блокировать только 1 раз )
//        printf("FLASH READ NOT Protected:\r\n");
//
//        FLASH_Unlock();     // разблокируем память
//        FLASH_ReadOutProtection(ENABLE);
//        FLASH_Lock();   // блокируем флеш память обратно
//
//        printf("FLASH READ >> Protected:\r\n");
//    }
//    //*****************************************************************************************************

    while(1)
    {


    }
}
