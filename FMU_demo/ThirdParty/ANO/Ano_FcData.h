/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ANO_FCDATA_H
#define __ANO_FCDATA_H
/* Includes ------------------------------------------------------------------*/
#include "config.h"
/* Exported types ------------------------------------------------------------*/
#define TRUE 1
#define FALSE 0 

enum
{
	AUTO_TAKE_OFF_NULL = 0,
	AUTO_TAKE_OFF = 1,
	AUTO_TAKE_OFF_FINISH,
	AUTO_LAND,
};

enum pwminmode_e
{
	PWM = 0,
	PPM,
	SBUS,
};

enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_X ,
 G_Y ,
 G_Z ,
 TEM ,
 MPU_ITEMS ,
};

	
enum
{
 CH_ROL = 0,
 CH_PIT ,
 CH_THR ,
 CH_YAW ,
 AUX1 ,
 AUX2 ,
 AUX3 ,
 AUX4 ,
 CH_NUM,
};

enum
{
	m1=0,
	m2,
	m3,
	m4,
	m5,
	m6,
	m7,
	m8,

};

enum
{
	MPU_6050_0 = 0,
	MPU_6050_1,
	
};

enum
{
	X = 0,
	Y = 1,
	Z = 2,
	VEC_XYZ,
};

enum
{
	ROL = 0,
	PIT = 1,
	YAW = 2,
	VEC_RPY,
};

enum
{
	KP = 0,
	KI = 1,
	KD = 2,
	PID,
};

enum _power_alarm
{

	HIGH_POWER = 0,
	HALF_POWER,
	LOW_POWER ,
	LOWEST_POWER, 
	

};


enum _flight_mode
{
	ATT_STAB = 0,//Attitude stabilization
	LOC_HOLD,
	RETURN_HOME,
	
};

//thr_mode
enum
{
  THR_MANUAL = 0,
	THR_AUTO,
	
};

typedef struct
{
	unsigned char first_f;
	float acc_offset[VEC_XYZ];
	float gyro_offset[VEC_XYZ];
	
	float surface_vec[VEC_XYZ];
	
	float mag_offset[VEC_XYZ];
	float mag_gain[VEC_XYZ];

} _save_st ;
extern _save_st save;

typedef struct
{
	//基本状态/传感器
	unsigned char start_ok;
	unsigned char sensor_ok;
	unsigned char motionless;
	unsigned char power_state;
	unsigned char wifi_ch_en;
	unsigned char rc_loss;	
	unsigned char gps_ok;	
	unsigned char gps_signal_bad;

	
	//控制状态
	unsigned char manual_locked;
	unsigned char unlock_en;
	unsigned char fly_ready;//unlocked
	unsigned char thr_low;
	unsigned char locking;
	unsigned char taking_off; //起飞
	unsigned char set_yaw;
	unsigned char ct_loc_hold;
	unsigned char ct_alt_hold;

	
	//飞行状态
	unsigned char flying;
	unsigned char auto_take_off_land;
	unsigned char home_location_ok;	
	unsigned char speed_mode;
	unsigned char thr_mode;	
	unsigned char flight_mode;
	unsigned char gps_mode_en;
	unsigned char motor_preparation;
	unsigned char locked_rotor;
	
	
}_flag;
extern _flag flag;

typedef struct
{
	unsigned char sonar_on;
	unsigned char tof_on;
	unsigned char of_flow_on;
	unsigned char of_tof_on;
	unsigned char baro_on;
	unsigned char gps_on;
	
}_switch_st;
extern _switch_st switchs;

typedef struct
{
	unsigned char gyro_ok;
	unsigned char acc_ok;
	unsigned char mag_ok;
	unsigned char baro_ok;
	unsigned char gps_ok;
	unsigned char sonar_ok;
	unsigned char tof_ok;
	unsigned char of_ok;
	
} _sensor_hd_check_st; //Hardware
extern _sensor_hd_check_st sens_hd_check;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void data_save(void);
void Para_Data_Init(void);


#endif

