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

#define WAKE_UP_EVENT       0
#define EXTI_BUTTON_EVENT   1

/*
        Система поддерживает 2 режима низкого энергопотребления, которые можно выбрать для низкого энергопотребления, короткого запуска.
        время и несколько событий пробуждения для достижения наилучшего баланса.

        1) Спящий режим Sleep mode ( WFI - Any interrupt ) ( WFE - Wake-up event )
        В спящем режиме останавливаются только часы ЦП, но все периферийные часы работают нормально, и
        периферия в рабочем состоянии. Этот режим является самым неглубоким режимом с низким энергопотреблением, но это самый быстрый режим для
        разбудить систему.
        Условие выхода: любое прерывание или событие пробуждения.
        ( потребление зависит от питания 3.3 или 5 вольт а также от частоты и ( внутрений или внешний ) генератор
        Например:
        питание 3.3В 48МГц внутрений HSI потребление ( с включенной перефирией 4,2 мА ) ( с выключенной перефирией 1,8 мА )
        питание 3.3В 750КГц внутрений HSI потребление ( с включенной перефирией 0,4 мА ) ( с выключенной перефирией 0,4 мА )
        питание 5В 48МГц внутрений HSI потребление ( с включенной перефирией 4,2 мА ) ( с выключенной перефирией 1,8 мА )
        питание 5В 750КГц внутрений HSI потребление ( с включенной перефирией 0,4 мА ) ( с выключенной перефирией 0,4 мА )
        с внешним кварцевый генератор HSE потребление выше

        2) Режим ожидания Standby mode ( AWU auto-wakeup )
        Биты PDDS и SLEEPDEEP установлены, и для входа выполняется инструкция WFI/WFE. Сила
        питание основной части отключено, а RC-генератор HSI и кварцевый генератор HSE также отключены.
        выключен, и в этом режиме можно достичь наименьшего энергопотребления.
        Условия выхода: любое внешнее прерывание/событие (сигнал EXTI), внешний сигнал сброса на NRST, сброс IWDG,
        где сигнал EXTI включает в себя один из 18 внешних портов ввода/вывода, выход PVD, автопробуждение AWU auto-wakeup.
        потребление зависит от питания 3,3 или 5 вольт и от включен ли внутрений нискочастотный часовой резонатор или нет:
        LSI включен 3,3В - 10,5 мкА     RCC_LSICmd(ENABLE);     while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
        LSI включен 5В - 11,1 мкА       RCC_LSICmd(ENABLE);     while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
        LSI выключен 3,3В - 9 мкА       RCC_LSICmd(DISABLE);    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
        LSI выключен 5В - 9,6 мкА       RCC_LSICmd(DISABLE);    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

*/

// В данном примере мы засыпаем и автоматически просыпаемся при авто пробуждении AWU automatically wakes up

// Примечание. Чтобы максимально снизить энергопотребление, рекомендуется
// установить неиспользуемый GPIO to pull-down mode


/* Global define */


/* Global Variable */

//*********************************************************************
void EXTI_INT_INIT(void)
{
    EXTI_InitTypeDef EXTI_InitStructure = {0};

#if WAKE_UP_EVENT

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    EXTI_InitStructure.EXTI_Line = EXTI_Line9;                  // External interrupt line 9 Connected to the PWR Auto Wake-up event
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;             // event mode
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

#elif EXTI_BUTTON_EVENT

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE); // port D

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;       // pin 0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // pull-up
    GPIO_Init(GPIOD, &GPIO_InitStructure);          // port D

    // GPIOD ----> EXTI_Line0
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource0); // port D  pin 0
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;                  // pin 0
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;             // event
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;     // Falling
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                   // запуск прерывания при старте
    EXTI_Init(&EXTI_InitStructure);

    // можно также добавить NVIC и в обработчике прерывания чтото делать когда просыпаемся
    // или повторно нажимаем кнопку когда уже не спим

#endif

}
//********************************************************************

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

    EXTI_INT_INIT();

    //---------------------------
    // Примечание. Чтобы максимально снизить энергопотребление, рекомендуется
    // установить неиспользуемый GPIO to pull-down mode
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);

#if !EXTI_BUTTON_EVENT
    // при просыпании от кнопки PD0 переконфигурировать нельзя ) ( можно другие пины порта D кроме 0 )
    GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif

    // после переключения всех портов переферия ( UART SPI I2C и т.д ) не работает
    //---------------------------

    //--------------------------------------------------
    // выключаем все ненужные прерывания

    // обязательно отключаем ( если включено ) прерывание системного таймера
    // так как это тоже прерывание и оно пробудит МК
    NVIC_DisableIRQ(SysTicK_IRQn);


#if WAKE_UP_EVENT

    // включаем нискочастотный внутрешний резонатор LSI 128 KHz
    RCC_LSICmd(ENABLE);     // LSI 128 KHz ( от данной шины тактируеться PWR для временной задержки AutoWakeUp )
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET); // ждем клогда включиться

    // устанавливаем предделитель ( время через которое будет вызван AutoWakeUp )
    PWR_AWU_SetPrescaler(PWR_AWU_Prescaler_10240);  // 128 000 / 10240 = 12,5 тиков в секунду
    // указанное ниже значение ( максимум 0x3F ( 63 ) ) делим на полученное выше значение 12,5 тиков в секунду и получаем
    // время через которое проснется ( например 25 / 12,5 = 2 секунды )
    PWR_AWU_SetWindowValue(25); // устанавливаем оконный сторожевой таймер WWDG значение не должно превышать 0x3F ( 63 )

    // запускаем AutoWakeUp начинает считать время для вызова события
    PWR_AutoWakeUpCmd(ENABLE);

    // пробуждаться можем в обработчике прерывания AWU_IRQn ( настроить нужно NVIC ) там можно считать отдельно время и снова засыпать ( для больших интервалов )

#elif EXTI_BUTTON_EVENT

    // выключаем нискочастотный внутрешний резонатор LSI 128 KHz
    RCC_LSICmd(DISABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == SET); // ждем клогда выключиться

#endif

    // засыпаем в режиме PWR_STANDBYEntry_WFE ( режим event )
    PWR_EnterSTANDBYMode(PWR_STANDBYEntry_WFE);


    //-----------------------------------------
    // после пробуждения продолжаем код
    // так как после засыпания вся перефирия отключаеться то нужно ее заново проинициализировать

    // при пробуждении обратно включаем системный таймер так как его выключили перед сном
    NVIC_EnableIRQ(SysTicK_IRQn);

    // заново инициализируем юарт
    USART_Printf_Init(115200);
    printf("\r\n Auto wake up < Standby mode > \r\n");

    while(1)
    {
        Delay_Ms(1000);
        printf("Run in main\r\n");
    }
}
