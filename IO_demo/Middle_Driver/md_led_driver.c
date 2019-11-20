/*************************************************************
*�ļ����ƣ�md_led_driver.c
*
*ժ    Ҫ��LED�м�������ʵ��(��������Ӧ���㷨��֮��)
*
*��ǰ�汾��
*��    �ߣ�LonelyMing
*���ڣ�2019/10/29
*���뻷����MDK5.27
*
*��ʷ��Ϣ��
*************************************************************/

#include "md_led_driver.h"


/**************************��������***********************
*�������ƣ�void MD_LED_BLUE_Control(uint8_t status)
*���������status:���ƵƵ�����״̬��  1��������0��Ϩ��
*���ز�����LonelyMing
*��    �ܣ�����blue�Ƶ�����
*��    �ߣ�
*��    �ڣ�2019/10/29
*************************************************************/
void MD_LED_BLUE_Control(uint8_t status)
{
	if(status==1)
	{
		//����
		HAL_GPIO_WritePin(IO_LED_BLUE_GPIO_Port,IO_LED_BLUE_Pin,GPIO_PIN_RESET);
	}
	else
	{
		//Ϩ��
		HAL_GPIO_WritePin(IO_LED_BLUE_GPIO_Port,IO_LED_BLUE_Pin,GPIO_PIN_SET);
	}
}


/**************************��������***********************
*�������ƣ�void MD_LED_AMBER_Control(uint8_t status)
*���������status:���ƵƵ�����״̬��  1��������0��Ϩ��
*���ز�����LonelyMing
*��    �ܣ�����amber�Ƶ�����
*��    �ߣ�
*��    �ڣ�2019/10/29
*************************************************************/
void MD_LED_AMBER_Control(uint8_t status)
{
	if(status==1)
	{
		//����
		HAL_GPIO_WritePin(IO_LED_AMBER_GPIO_Port,IO_LED_AMBER_Pin,GPIO_PIN_RESET);
	}
	else
	{
		//Ϩ��
		HAL_GPIO_WritePin(IO_LED_AMBER_GPIO_Port,IO_LED_AMBER_Pin,GPIO_PIN_SET);
	}
}

/**************************��������***********************
*�������ƣ�void MD_LED_SAFETY_Control(uint8_t status)
*���������status:���ƵƵ�����״̬��  1��������0��Ϩ��
*���ز�����LonelyMing
*��    �ܣ�����safety�Ƶ�����
*��    �ߣ�
*��    �ڣ�2019/10/29
*************************************************************/
void MD_LED_SAFETY_Control(uint8_t status)
{
	if(status==1)
	{
		//����
		HAL_GPIO_WritePin(IO_LED_SAFETY_GPIO_Port,IO_LED_SAFETY_Pin,GPIO_PIN_RESET);
	}
	else
	{
		//Ϩ��
		HAL_GPIO_WritePin(IO_LED_SAFETY_GPIO_Port,IO_LED_SAFETY_Pin,GPIO_PIN_SET);
	}
}


