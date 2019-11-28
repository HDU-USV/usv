/*******************************************************************************
* 文件名称：md_led_driver.c
*
* 摘    要：STM32F4的LED中间驱动层实现（底层硬件驱动和软件应用层之间的层）
*
* 当前版本：
* 作    者：
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#include "md_led_driver.h"
#include "main.h"

#define CYCLE_2HZ_FROM_500HZ   250
__IO uint32_t Led_RunCount=0;


/*******************************函数声明****************************************
* 函数名称: void MD_LED_AMBER_Control(uint8_t status)
* 输入参数: status：控制灯的亮灭状态
* 返回参数:  
* 功    能:
* 作    者: 
* 日    期: 2017/12/18
*******************************************************************************/ 
void MD_LED_AMBER_Control(uint8_t status)
{
	if(status==1)
	{
		//点亮
	  HAL_GPIO_WritePin(FMC_LED_AMBER_GPIO_Port,FMC_LED_AMBER_Pin,GPIO_PIN_RESET);
	}
	else
	{
		//熄灭
	   HAL_GPIO_WritePin(FMC_LED_AMBER_GPIO_Port,FMC_LED_AMBER_Pin,GPIO_PIN_SET);
	}
}

/*******************************函数声明****************************************
* 函数名称: void Loop_Led_Flick(void)
* 输入参数: 
* 返回参数:  
* 功    能: 50HZ循环LED闪烁
* 作    者: by Ming
* 日    期: 2019/11/20
*******************************************************************************/ 
void Loop_Led_Flick(void)
{
	//函数执行一次该变量+1
	Led_RunCount++;

	//Led_RunCount每变化250次，if语句执行一次，即该if语句2hz执行
	if(Led_RunCount % CYCLE_2HZ_FROM_500HZ ==0)
	{
		HAL_GPIO_TogglePin(FMC_LED_AMBER_GPIO_Port,FMC_LED_AMBER_Pin);
	}
}
