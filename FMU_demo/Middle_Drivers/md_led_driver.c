/*******************************************************************************
* �ļ����ƣ�md_led_driver.c
*
* ժ    Ҫ��STM32F1��LED�м�������ʵ�֣��ײ�Ӳ�����������Ӧ�ò�֮��Ĳ㣩
*
* ��ǰ�汾��
* ��    �ߣ�
* ��    �ڣ�2017/12/18
* ���뻷����keil5
*
* ��ʷ��Ϣ��
*******************************************************************************/

#include "md_led_driver.h"
#include "main.h"

#define CYCLE_2HZ_FROM_500HZ   250
__IO uint32_t Led_RunCount=0;


/*******************************��������****************************************
* ��������: void MD_LED_AMBER_Control(uint8_t status)
* �������: status�����ƵƵ�����״̬
* ���ز���:  
* ��    ��:
* ��    ��: 
* ��    ��: 2017/12/18
*******************************************************************************/ 
void MD_LED_AMBER_Control(uint8_t status)
{
	if(status==1)
	{
		//����
	  HAL_GPIO_WritePin(FMC_LED_AMBER_GPIO_Port,FMC_LED_AMBER_Pin,GPIO_PIN_RESET);
	}
	else
	{
		//Ϩ��
	   HAL_GPIO_WritePin(FMC_LED_AMBER_GPIO_Port,FMC_LED_AMBER_Pin,GPIO_PIN_SET);
	}
}

/*******************************��������****************************************
* ��������: void Loop_Led_Flick(void)
* �������: 
* ���ز���:  
* ��    ��: 50HZѭ��LED��˸
* ��    ��: by Ming
* ��    ��: 2019/11/20
*******************************************************************************/ 
void Loop_Led_Flick(void)
{
	//����ִ��һ�θñ���+1
	Led_RunCount++;

	//Led_RunCountÿ�仯250�Σ�if���ִ��һ�Σ�����if���2hzִ��
	if(Led_RunCount % CYCLE_2HZ_FROM_500HZ ==0)
	{
		HAL_GPIO_TogglePin(FMC_LED_AMBER_GPIO_Port,FMC_LED_AMBER_Pin);
	}
}
