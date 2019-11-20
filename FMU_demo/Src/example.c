/*******************************************************************************
* 文件名称：example.c
*
* 摘    要：测试程序
*
* 当前版本：
* 作    者：
* 日    期：2018/05/24
* 编译环境：keil5
*
* 历史信息：
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
  /*1.获取遥控器的通道值*/
	  Com_TwoBoard_RB_Update();
	/*2.进行控制运算*/
	if(f_NewRCDataReady==SET)
	{
	  f_NewRCDataReady = RESET;
		Example_Control(&m_RC_input_from_F1);
	  /*3.输出电机的控制值给F1*/
	 Com_TwoBoard_Msg_MotorValue_Send(motor);
	}
	 
}
