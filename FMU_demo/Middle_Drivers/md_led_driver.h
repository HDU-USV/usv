/*******************************************************************************
* 文件名称：md_led_driver.h
*
* 摘    要：STM32F1的LED中间驱动层（底层硬件驱动和软件应用层之间的层）
*
* 当前版本：
* 作    者：
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#ifndef __MD_LED_DRIVER_H
#define __MD_LED_DRIVER_H

#include "md_struct.h"


void MD_LED_AMBER_Control(uint8_t status);
void Loop_Led_Flick(void);
#endif
