/*******************************************************************************
* �ļ����ƣ�example.c
*
* ժ    Ҫ�����Գ���
*
* ��ǰ�汾��
* ��    �ߣ�
* ��    �ڣ�2018/05/24
* ���뻷����keil5
*
* ��ʷ��Ϣ��
*******************************************************************************/
#include "example.h"
#include "md_include.h"
#include "md_struct.h"
extern __IO ITStatus f_NewRCDataReady;
extern RC_Channel m_RC_input_from_F1;
uint16_t motor[6];
void Example_Control(RC_Channel*rc_input)
{
  motor[0] = rc_input->channel3 +750 - rc_input->channel1/2;
	motor[1] = rc_input->channel3 -750 + rc_input->channel1/2;
	motor[2] = rc_input->channel3;
	motor[3] = rc_input->channel3;
	motor[4] = rc_input->channel3;
	motor[5] = rc_input->channel3;
}

void Example_Exchage_COM(void)
{	
  /*1.��ȡң������ͨ��ֵ*/
	  Com_TwoBoard_RB_Update();
	/*2.���п�������*/
	if(f_NewRCDataReady==SET)
	{
	  f_NewRCDataReady = RESET;
		Example_Control(&m_RC_input_from_F1);
	  /*3.�������Ŀ���ֵ��F1*/
	 Com_TwoBoard_Msg_MotorValue_Send(motor);
	}
	 
}
