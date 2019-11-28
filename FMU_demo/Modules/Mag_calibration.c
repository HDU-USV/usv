/*
******************************************************************************
* @file    Mag_calibration.c 
* @author  
* @version V1.0.0
* @date    2018-10-22
* @brief   �شž�̬У׼
* @freq    50hz
******************************************************************************/

#include "Mag_calibration.h"


void Mag_Calibration_Instrction(void);
void Mag_Calibration_Operation(void);

uint8_t Mag_Calibration_Instrction_Step=0;

void Mag_Calibration_update(void)
{
	Mag_Calibration_Instrction();
	Mag_Calibration_Operation();
}

/*------------------------------------------------------------------------------ 
�ش�У׼����ָ�����
STEP0��У׼��ʼ�������л�ң�������ؽ���ش�У׼ģʽ��STEP=1��
STEP1��
------------------------------------------------------------------------------*/
void Mag_Calibration_Instrction()
{
	static float circlingcngle_roll=0.0f, circlingangle_pitch=0.0f, circlingangle_yaw=0.0f;
	static uint8_t direectionZ_magcali_cnt;
	static uint8_t delay_1sec = 0;
	
	switch(Mag_Calibration_Instrction_Step)
	{
		case 0://δ����У׼�ش�״̬���Ƕȼ����ʼ��
			circlingangle_yaw   = 0.0f;
			circlingcngle_roll  = 0.0f;
			circlingangle_pitch = 0.0f;
			delay_1sec = 0;    
			direectionZ_magcali_cnt = 0;
			if(get_System_State() == MAG_STATIC_CALIBRATION_STATE)
			  Mag_Calibration_Instrction_Step = 1;
		break;
		
		case 1://����ش�У׼״̬��У׼XY��ش�
			circlingangle_yaw += pIMU_Data->YawRate_ori*0.02f;
			if( fabs(circlingangle_yaw) > 3.0f*PI)
			  Mag_Calibration_Instrction_Step = 2;
		break;
		
		case 2://XYƽ��ش�У׼��ɣ����ɻ����𣬳�ʼ��Roll��Pitch
			circlingcngle_roll  = 0.0f;
			circlingangle_pitch = 0.0f;
			if(( pIMU_Data->Zacc > -2.0f )&&( pIMU_Data->Zacc < 2.0f ))
			  direectionZ_magcali_cnt++;
			else
			  direectionZ_magcali_cnt = 0;
			if(direectionZ_magcali_cnt >= 50)
			  Mag_Calibration_Instrction_Step = 3;
		break;
		
		case 3://�ɻ�����λ��У׼Z��ش�
			circlingcngle_roll  += pIMU_Data->RollRate_ori*0.02f;
			circlingangle_pitch += pIMU_Data->PitchRate_ori*0.02f;
			if( fabs(circlingcngle_roll) > 3.0f*PI || fabs(circlingangle_pitch) > 3.0f*PI)
			  Mag_Calibration_Instrction_Step = 4;
			break;
			
		case 4://Z��ش�У׼��ɣ�ֹͣ�ش�У׼,�ȴ��������
			delay_1sec++;
			if(delay_1sec >=50 && isCali_XYFactor_Check_Succeed())
			{
			  Mag_Calibration_Instrction_Step = 5;
			  set_System_State(PREPARING_STATE);
			}
			else if(delay_1sec >=50 && !isCali_XYFactor_Check_Succeed())
			  Mag_Calibration_Instrction_Step = 6;
		break;
		case 5://�˳��ش�У׼�׶Σ����ز������أ���LED��ָʾУ׼����Ƿ�����
			delay_1sec = 0;
			if(isToTakeOffHealthPrepared())
			  Mag_Calibration_Instrction_Step = 7;
		break;
		
		case 6://�ȴ��ɻ���λ��ƽ���ȴ����׼�����
			Mag_Calibration_Instrction_Step = 0;
		break;
		
		case 7://�ɻ���λ��ƽ�����׼����ɣ�״̬���㣬IMU��ʼ��
			if(delay_1sec++ >= 50)
			{
			  Mag_Calibration_Instrction_Step = 0;
			}
		break;
		default:
			Mag_Calibration_Instrction_Step = 0;
	};
}

uint8_t get_Mag_Calibration_Instrction_Step(void)
{
  return Mag_Calibration_Instrction_Step;
}

//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t Mag_Calibration_State = 0;
float HMC5983_maxx,HMC5983_maxy,HMC5983_maxz;
float HMC5983_minx,HMC5983_miny,HMC5983_minz;

void Mag_calibration_init_XY(void)
{
  HMC5983_maxx = -200;
  HMC5983_maxy = -200; 
  HMC5983_minx = 200;
  HMC5983_miny = 200;
}

void Mag_calibration_init_Z(void)
{
  HMC5983_maxz = -200; 
  HMC5983_minz = 200;
}

void Mag_calibration_init(void)
{
  Mag_calibration_init_XY();
  Mag_calibration_init_Z();
}


void Traverse_magvalue_XY(void)
{
  if(HMC5983_minx > pIMU_Data->magX_raw)               HMC5983_minx = pIMU_Data->magX_raw;
  if(HMC5983_miny > pIMU_Data->magY_raw)               HMC5983_miny = pIMU_Data->magY_raw;
  if(HMC5983_maxx < pIMU_Data->magX_raw)               HMC5983_maxx = pIMU_Data->magX_raw;
  if(HMC5983_maxy < pIMU_Data->magY_raw)               HMC5983_maxy = pIMU_Data->magY_raw;
}

void Cali_parameter_caculation_XY(void)
{
  pCali_Data->dMx_offset = (HMC5983_maxx+HMC5983_minx)/2;
  pCali_Data->dMy_offset = (HMC5983_maxy+HMC5983_miny)/2;
  pCali_Data->dMx_min = HMC5983_minx;
  pCali_Data->dMy_min = HMC5983_miny;
  pCali_Data->dMx_max = HMC5983_maxx;
  pCali_Data->dMy_max = HMC5983_maxy;
}

uint8_t isCali_XYFactor_Check_Succeed(void)
{      
  float magx_range,magy_range;
  magx_range = HMC5983_maxx-HMC5983_minx;
  magy_range = HMC5983_maxy-HMC5983_miny; 
  if(magx_range==0) return 0;
  if(magy_range==0) return 0;
  pCali_Data->MAGXY_factor = magx_range/magy_range;
  if(fabs(pCali_Data->MAGXY_factor - 1.0f) <= 0.2f)  
    return 1;
  else
    return 0;
}

void Traverse_magvalue_Z(void)
{
  if(HMC5983_minz > pIMU_Data->magZ_raw)                 HMC5983_minz = pIMU_Data->magZ_raw;
  if(HMC5983_maxz < pIMU_Data->magZ_raw)                 HMC5983_maxz = pIMU_Data->magZ_raw;    
}

void Cali_parameter_caculation_Z(void)
{
  pCali_Data->dMz_offset = (HMC5983_maxz+HMC5983_minz)/2;
  pCali_Data->dMz_min = HMC5983_minz;
  pCali_Data->dMz_max = HMC5983_maxz;
}

uint8_t isCali_XZFactor_Check_Succeed(void)
{      
  float magx_range,magz_range;
  magx_range = HMC5983_maxx-HMC5983_minx;
  magz_range = HMC5983_maxz-HMC5983_minz; 
  if(magx_range==0) return 0;
  if(magz_range==0) return 0;
  pCali_Data->MAGXZ_factor = magx_range/magz_range;
  if(fabs(pCali_Data->MAGXZ_factor - 1.0f) <= 0.2f)  
    return 1;
  else
    return 0;
}


void Save_Cali_parameter(void)
{
	//����У׼�Ĳ���
}

void Reset_Mag_calibration(void)
{ 
  pCali_Data->dMx_offset = 0.0f;	
  pCali_Data->dMy_offset = 0.0f;
  pCali_Data->dMz_offset = 0.0f;
  pCali_Data->dMx_min = 0.0f;	
  pCali_Data->dMy_min = 0.0f;
  pCali_Data->dMz_min = 0.0f;
  pCali_Data->dMx_max = 0.0f;	
  pCali_Data->dMy_max = 0.0f;
  pCali_Data->dMz_max = 0.0f;
  pCali_Data->MAGXY_factor = 1.0f;
  pCali_Data->MAGXZ_factor = 1.0f;
}
void Get_calibration_offset(void)
{
	//�ϵ��ʼ���󣬻�ȡ֮ǰУ׼������Ĳ���
}

void Mag_Calibration_Operation(void)
{
  switch(Mag_Calibration_Instrction_Step)
  {
  case 0:
    Mag_calibration_init();
    break;
  case 1:
    Traverse_magvalue_XY();  
    break;
  case 2:
    Cali_parameter_caculation_XY();
    break;
  case 3:
    Traverse_magvalue_Z();
    break;
  case 4:
    Cali_parameter_caculation_Z();
    if(isCali_XYFactor_Check_Succeed())
    {
      Save_Cali_parameter();
      Mag_Calibration_State = 0;
    }
    else
    {
      Reset_Mag_calibration();
      Mag_Calibration_State = 1;
    }
    break;
  default:
    break;
  };
}


