/*
  ******************************************************************************
  * @file    sensor.c
  * @author  across
  * @version 
  * @date    2019-06-14
  * @brief   传感器数据进行单位转换，坐标转换，滤波，校准
 ******************************************************************************
*/

#include "sensor.h"
#include "Filter.h"
#include <stdlib.h>
#include <math.h>

extern MPU6000_Data 		m_Mpu6000;
extern hmc5883MagData 	m_Hmc5883;

static uint8_t flag_calib_falied = 1;
static int16_t gyro_last[3] = {0};
static uint16_t gyro_int[3] = {0};
static int16_t gyro_offset_x = 0,gyro_offset_y = 0,gyro_offset_z = 0;
static float acc_offset_x = -0.05f,acc_offset_y = -0.16f,acc_offset_z = -0.7f;

//控制反馈数据
Control_state  Ctrl_state;

//IIR滤波器参数变量定义
IIR_coeff_Typedef Filter_Param_Gx;
IIR_coeff_Typedef *pFilter_Param_Gx = &Filter_Param_Gx;
IIR_coeff_Typedef Filter_Param_Gy;
IIR_coeff_Typedef *pFilter_Param_Gy = &Filter_Param_Gy;
IIR_coeff_Typedef Filter_Param_Gz;
IIR_coeff_Typedef *pFilter_Param_Gz = &Filter_Param_Gz;
IIR_coeff_Typedef Filter_Param_Ax;
IIR_coeff_Typedef *pFilter_Param_Ax = &Filter_Param_Ax;
IIR_coeff_Typedef Filter_Param_Ay;
IIR_coeff_Typedef *pFilter_Param_Ay = &Filter_Param_Ay;
IIR_coeff_Typedef Filter_Param_Az;
IIR_coeff_Typedef *pFilter_Param_Az = &Filter_Param_Az;


/*
* @brief  IIR滤波器初始化  计算滤波器参数
* @param  void
* @note		
* @retval void
* @author across
*/
void Filter_init(void)
{
	cal_iir_coeff(pFilter_Param_Gx, 500, GYRO_CutoffFreq);
  cal_iir_coeff(pFilter_Param_Gy, 500, GYRO_CutoffFreq);
  cal_iir_coeff(pFilter_Param_Gz, 500, GYRO_CutoffFreq);
	
	cal_iir_coeff(pFilter_Param_Ax, 500, ACC_CutoffFreq);
  cal_iir_coeff(pFilter_Param_Ay, 500, ACC_CutoffFreq);
  cal_iir_coeff(pFilter_Param_Az, 500, ACC_CutoffFreq);
}

/*
* @brief  根据设定的坐标系，转换数据的正负号
* @param  
* @note		
* @retval void
* @author across
*/
void Transform(float Aframex,float Aframey,float Aframez,float *Bframex,float *Bframey,float *Bframez)
{
    *Bframex = Aframex;
    *Bframey = Aframey;
    *Bframez = Aframez;
}

/*
* @brief  处理IMU原始数据，将其转换单位和对应的坐标系
* @param  void
* @note		
* @retval void
* @author across
*/
void IMU_Data_Combine(void)
{
	float gyro_range_scale = (0.0174532f / 16.4f); //正负2000度/秒  将原始数据转换成弧度/秒
	float accel_range_scale = MPU6000_ONE_G * (1.0f / 8192.0f); //正负4g
	
	//将mpu原始数据转换成标准单位  
	m_Mpu6000.gyrof.x = (m_Mpu6000.gyro_raw.x - gyro_offset_x) * gyro_range_scale;
	m_Mpu6000.gyrof.y = (m_Mpu6000.gyro_raw.y - gyro_offset_y) * gyro_range_scale;
	m_Mpu6000.gyrof.z = (m_Mpu6000.gyro_raw.z - gyro_offset_z) * gyro_range_scale;
	
	m_Mpu6000.accf.x = m_Mpu6000.acc_raw.x * accel_range_scale - acc_offset_x;
	m_Mpu6000.accf.y = m_Mpu6000.acc_raw.y * accel_range_scale - acc_offset_y;
	m_Mpu6000.accf.z = m_Mpu6000.acc_raw.z * accel_range_scale - acc_offset_z;
	
	//转换数据的坐标系
	//机体系  右前下  xyz
	Transform(m_Mpu6000.gyrof.y,-m_Mpu6000.gyrof.x,m_Mpu6000.gyrof.z,&m_Mpu6000.gyrof.x,&m_Mpu6000.gyrof.y,&m_Mpu6000.gyrof.z);
	Transform(m_Mpu6000.accf.y,-m_Mpu6000.accf.x,m_Mpu6000.accf.z,&m_Mpu6000.accf.x,&m_Mpu6000.accf.y,&m_Mpu6000.accf.z);
	
	//二阶IIR低通滤波器
	Ctrl_state.gyrof.x = get_iir_output(pFilter_Param_Gx, m_Mpu6000.gyrof.x);
  Ctrl_state.gyrof.y = get_iir_output(pFilter_Param_Gy, m_Mpu6000.gyrof.y);
  Ctrl_state.gyrof.z = get_iir_output(pFilter_Param_Gz, m_Mpu6000.gyrof.z);
	
	Ctrl_state.accf.x = get_iir_output(pFilter_Param_Ax, m_Mpu6000.accf.x);
  Ctrl_state.accf.y = get_iir_output(pFilter_Param_Ay, m_Mpu6000.accf.y);
  Ctrl_state.accf.z = get_iir_output(pFilter_Param_Az, m_Mpu6000.accf.z);
	
	//地磁数据赋值  单位 高斯
	Ctrl_state.magf.x = -m_Hmc5883.y;  
	Ctrl_state.magf.y =  m_Hmc5883.x;
	Ctrl_state.magf.z =  m_Hmc5883.z;
}


/*
* @brief  陀螺仪校准零点函数
* @param  void
* @note		上电初始化调用，一直到函数完成才接着运行
* @retval void
* @author across
*/
void Get_Gyro_Offset(void)
{
  static int32_t offset_sumx = 0, offset_sumy = 0, offset_sumz = 0;
  uint8_t k = 0;
  
  while(flag_calib_falied)
  {
    if(k < 200)
    {
			MPU6000_Get_Data(&m_Mpu6000);
			HAL_Delay(20);
      
      if(k == 0)
      {
        gyro_last[0] = m_Mpu6000.gyro_raw.x;
        gyro_last[1] = m_Mpu6000.gyro_raw.y;
        gyro_last[2] = m_Mpu6000.gyro_raw.z;  
      }
      
      gyro_int[0] += abs(m_Mpu6000.gyro_raw.x - gyro_last[0]);
      gyro_int[1] += abs(m_Mpu6000.gyro_raw.y - gyro_last[1]);
      gyro_int[2] += abs(m_Mpu6000.gyro_raw.z - gyro_last[2]);
      
      offset_sumx += m_Mpu6000.gyro_raw.x;
      offset_sumy += m_Mpu6000.gyro_raw.y;
      offset_sumz += m_Mpu6000.gyro_raw.z;
      
      k++;
    }
    else
    {
			//若船体运动，重新进行校准
      if((gyro_int[0] > 500) || (gyro_int[1] > 500) || (gyro_int[2] > 500))
      {
        flag_calib_falied = 1;
        k = 0;
        offset_sumx = 0;
        offset_sumy = 0;
        offset_sumz = 0;
        for(uint8_t j = 0; j < 3; j++)
        {
          gyro_int[j] = 0;
        }
      }
      else
      {
        flag_calib_falied = 0;
      }
    }
  }
  
  gyro_offset_x  = (offset_sumx/200);
  gyro_offset_y  = (offset_sumy/200);
  gyro_offset_z  = (offset_sumz/200);
}

