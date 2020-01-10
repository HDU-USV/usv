#include "control.h"
#include "md_ubx.h"
#include "arm_math.h"


#define PID_PARAM_KP 1.5
#define PID_PARAM_KI 0.05
#define PID_PARAM_KD 0.2
#define LIMITING 200

__IO uint32_t Control_RunCount=0;
#define CYCLE_20HZ_FROM_500HZ   25
arm_pid_instance_f32 heading_pid;
float32_t des_heading,cur_heading,diff_heading = 0;

char control_flag,limit_flag,stop_flag=0;
int16_t turn_duty;

/**************************函数声明***********************
*函数名称：void turn_pid_init(void)
*输入参数：
*返回参数：
*功    能：DSP_PID初始化
*作    者：
*日    期：2019/12/18
*************************************************************/
void turn_pid_init(void)
{
	heading_pid.Kp = PID_PARAM_KP;		
	heading_pid.Ki = PID_PARAM_KI;		
	heading_pid.Kd = PID_PARAM_KD;	
	arm_pid_init_f32(&heading_pid, 1);
}


				
/**************************函数声明***********************
*函数名称：float32_t cal_heading(int32_t cur_lat, int32_t cur_lon, int32_t des_lat, int32_t des_lon)
*输入参数：cur_lat, cur_lon 当前经纬度 des_lat, des_lon 目的经纬度
*返回参数：heading 航向角
*功    能：根据目标、当前经纬度计算航向角
*作    者：
*日    期：2019/12/18
*************************************************************/
float32_t cal_heading(int32_t cur_lat, int32_t cur_lon, int32_t des_lat, int32_t des_lon)
{
	int16_t distance_North,distance_East=0;
	float64_t du_lat = 0;
	float32_t heading = 0;
	
	distance_North = des_lat-cur_lat;
	du_lat=cos((float64_t)(cur_lat/1800000000.f*PI));
	distance_East = (des_lon-cur_lon)*du_lat;
	heading = atan2(distance_East,distance_North)/PI*180;
	return heading;
}

/**************************函数声明***********************
*函数名称：float32_t heading_ENU2NED(float32_t heading)
*输入参数：heading 东北天航向角
*返回参数：heading 北东地航向角
*功    能：根据目标、当前经纬度计算航向角
*作    者：
*日    期：2019/12/18
*************************************************************/
float32_t heading_ENU2NED(float32_t heading)
{
	if (heading < 0) heading = 360 + heading; 		
	
	heading = 360 - heading;
	if(heading >= 360)
	{
		heading = heading-360;
	}
	return heading;
}


/**************************函数声明***********************
*函数名称：float32_t heading_diff_cal(float32_t heading)
*输入参数：float32_t cur_heading,float32_t des_heading 当前航向角 期望航向角
*返回参数：heading_diff 航向角误差 -180度~180度之间
*功    能：
*作    者：
*日    期：2019/12/18
*************************************************************/
float32_t heading_diff_cal(float32_t cur_heading,float32_t des_heading)
{
	float32_t heading_diff=0;
	heading_diff = cur_heading - des_heading;
	if (heading_diff > 180)
	{
		heading_diff -=360; 
	}
	else if (heading_diff < -180)
	{
		heading_diff +=360; 
	}
	return heading_diff;
}

void heading_control(void)
{
	Control_RunCount++;
	if(Control_RunCount % CYCLE_20HZ_FROM_500HZ ==0)
	{
    cur_heading = heading_ENU2NED(jy901_state.angle.yaw);		
		if(point_onetest.lat !=0 && point_onetest.lon !=0)
		{
			des_heading = cal_heading(_gps_position->lat,_gps_position->lon,point_onetest.lat*10000000,point_onetest.lon*10000000);
		  diff_heading = heading_diff_cal(cur_heading,des_heading);
		  turn_duty = arm_pid_f32(&heading_pid, diff_heading);
			if (turn_duty > LIMITING) turn_duty = LIMITING;
			else if (turn_duty < -LIMITING) turn_duty = -LIMITING;
			control_flag = 1;
		}
	}
}
