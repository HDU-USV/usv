/*************************************************************
*�ļ����ƣ�md_led_driver.h
*
*ժ    Ҫ��
*
*��ǰ�汾��
*��    �ߣ�LonelyMing
*���ڣ�2019/10/29
*���뻷����MDK5.27
*
*��ʷ��Ϣ��
*************************************************************/

#ifndef __MD_LED_DRIVER_H
#define __MD_LED_DRIVER_H

#include "md_include.h"

#define Status_Safety   HAL_GPIO_ReadPin(SAFETY_GPIO_Port,SAFETY_Pin)

void MD_LED_BLUE_Control(uint8_t status);
void MD_LED_AMBER_Control(uint8_t status);
void MD_LED_SAFETY_Control(uint8_t status);
	
#endif
