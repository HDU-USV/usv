/*******************************************************************************
* �ļ����ƣ�md_rc.h
*
* ժ    Ҫ��ң�������C�ļ�
*
* ��ǰ�汾��
* ��    �ߣ�
* ��    �ڣ�2018/05/15
* ���뻷����keil5
*
* ��ʷ��Ϣ��
*******************************************************************************/
#include "md_rc.h"
#include "md_rc_ppm.h"
#include "md_rc_sbus.h"
#include "md_config.h"

#define RC_INPUT_PPM

uint16_t rc_pwm_in[RC_INPUT_CHANNELS];
__IO ITStatus rc_newdata_flag   = RESET;



/**************************��������***********************
*�������ƣ�void RC_Input_Init(void)
*���������
*���ز�����
*��    �ܣ�����ң�����Ĳ�ͬ���г�ʼ��
*��    �ߣ�
*��    �ڣ�2019/11/20
*************************************************************/
void RC_Input_Init(void)
{
#ifdef RC_INPUT_PPM
	PPM_Init();
#else
	SBUS_Init();
#endif
}

/**************************��������***********************
*�������ƣ�void RC_Input_Loop(void)
*���������
*���ز�����
*��    �ܣ�����ң�����Ĳ�ͬ��ȡ����
*��    �ߣ�
*��    �ڣ�2019/11/20
*************************************************************/
void RC_Input_Loop(void)
{
#ifdef RC_INPUT_PPM
	PPM_GetData(rc_pwm_in,RC_INPUT_CHANNELS);
#else
	SBUS_GetData(rc_pwm_in,RC_INPUT_CHANNELS);
#endif
}
