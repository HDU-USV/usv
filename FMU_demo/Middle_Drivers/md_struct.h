/*******************************************************************************
* 文件名称：md_struct.h
*
* 摘    要：自定义结构体
*
* 当前版本：
* 作    者：
* 日    期：2019/11/10
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#ifndef __MD_STRUCT_H
#define __MD_STRUCT_H

#include "stm32f4xx_hal.h"

#define PI 3.14159265f

/** 
  * @brief  LED Status enumeration 
  */
typedef enum
{
  LED_OFF = 0U,
  LED_ON
}LED_Status;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
} Vector3_Int16;

typedef struct{
	float x;
	float y;
	float z;
}Vector3_Float;

/* MPU6000六轴姿态传感器定义的结构体*/
typedef struct{
	Vector3_Int16 acc_raw;
	Vector3_Int16 gyro_raw;
	Vector3_Float accf;
	Vector3_Float gyrof;
	int16_t temp;   //temperature
}MPU6000_Data;


/* MS56XX气压计时定义的结构体*/
typedef struct {
  int32_t temp;
	int32_t pressure;
} MS56XX_Data;

/* HMC5883地磁计定义的结构体*/
typedef struct {
				float x;
				float y;
				float z;
				float orientation;
} hmc5883MagData;

/* 遥控器通道定义的结构体*/
typedef struct  
{
  uint16_t channel1;
	uint16_t channel2;
	uint16_t channel3;
	uint16_t channel4;
	uint16_t channel5;
	uint16_t channel6;
	uint16_t channel7;
	uint16_t channel8;
}RC_Channel;
extern RC_Channel m_RC_input_from_F1;

/* 电机定义的结构体*/
typedef struct
{
	uint16_t motor1;
	uint16_t motor2;
	uint16_t motor3;
	uint16_t motor4;
	uint16_t motor5;
	uint16_t motor6;
}Motor_Value;
extern Motor_Value Motor;

//四元数计算用到的结构体   四元数
typedef struct 
{
	float w;
	float x;
	float y;
	float z;
}Quaternion;
extern Quaternion Q;

// 欧拉角 
typedef struct 
{
  double roll;
	double pitch;
	double yaw;
}Euler_Angle;
extern Euler_Angle Angle;

/* 传感器数据处理和姿态解算结构体 */
typedef struct{
	Vector3_Float accf;
	Vector3_Float gyrof;
	Vector3_Float magf;
  Euler_Angle angle;
}Control_state;
extern Control_state jy901_state;

/* 传感器数据处理和姿态解算结构体 */
typedef struct{
		double lat;
		double lon;
}Way_Point;
extern Way_Point point_onetest;

/* 16位共同体*/
typedef union
{
  uint16_t value_16;
	uint8_t  value_8[2];
}UINT16_8BIT;

#define CONSTRAINT(in, min, max)  (in > max ? max : (in < min ? min : in))
#define ABS(a) ((a) > 0 ? (a) : -(a))
#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

#endif
