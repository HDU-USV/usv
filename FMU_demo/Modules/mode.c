/*******************************************************************************
* �ļ����ƣ�mode.c
*
* ժ    Ҫ������ģʽ�л�
*
* ��ǰ�汾��
* ��    �ߣ�
* ��    �ڣ�2019/11/27
* ���뻷����keil5
*
* ��ʷ��Ϣ��
*******************************************************************************/
#include "mode.h"

Mode current_mode;
void Mode_Update(void)
{
	switch(current_mode){
		case Manual:break;
		case Auto_WP:break;
		case Auto_RTL:break;
		default: break;
	}
}