/********************************** (C) COPYRIGHT *******************************
 * File Name          : app_generic_onoff_model.h
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2021/11/18
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef app_generic_onoff_model_H
#define app_generic_onoff_model_H

#ifdef __cplusplus
extern "C" {
#endif

#include "MESH_LIB.h"

#define MSG_PIN    GPIO_Pin_14
#define MSG_PCENR  RCC_APB2Periph_GPIOB
#define MSG_GPIO   GPIOB

extern const struct bt_mesh_model_op gen_onoff_op[];

/**
 * @brief   ��ȡ��ǰ��״̬.
 *
 * @param   led_pin - ����.
 *
 * @return  ��״̬
 */
BOOL read_led_state(uint32_t led_pin);

/**
 * @brief   ���õ�ǰ��״̬.
 *
 * @param   led_pin - ����.
 * @param   on      - ״̬.
 */
void set_led_state(uint32_t led_pin, BOOL on);

/**
 * @brief   ��ת��ǰ��״̬
 *
 * @param   led_pin - ����.
 */
void toggle_led_state(uint32_t led_pin);

#ifdef __cplusplus
}
#endif

#endif
