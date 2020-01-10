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

/**************************��������***********************
*�������ƣ�void turn_pid_init(void)
*���������
*���ز�����
*��    �ܣ�DSP_PID��ʼ��
*��    �ߣ�
*��    �ڣ�2019/12/18
*************************************************************/
void turn_pid_init(void)
{
	heading_pid.Kp = PID_PARAM_KP;		
	heading_pid.Ki = PID_PARAM_KI;		
	heading_pid.Kd = PID_PARAM_KD;	
	arm_pid_init_f32(&heading_pid, 1);
}


				
/**************************��������***********************
*�������ƣ�float32_t cal_heading(int32_t cur_lat, int32_t cur_lon, int32_t des_lat, int32_t des_lon)
*���������cur_lat, cur_lon ��ǰ��γ�� des_lat, des_lon Ŀ�ľ�γ��
*���ز�����heading �����
*��    �ܣ�����Ŀ�ꡢ��ǰ��γ�ȼ��㺽���
*��    �ߣ�
*��    �ڣ�2019/12/18
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

/**************************��������***********************
*�������ƣ�float32_t heading_ENU2NED(float32_t heading)
*���������heading �����캽���
*���ز�����heading �����غ����
*��    �ܣ�����Ŀ�ꡢ��ǰ��γ�ȼ��㺽���
*��    �ߣ�
*��    �ڣ�2019/12/18
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


/**************************��������***********************
*�������ƣ�float32_t heading_diff_cal(float32_t heading)
*���������float32_t cur_heading,float32_t des_heading ��ǰ����� ���������
*���ز�����heading_diff �������� -180��~180��֮��
*��    �ܣ�
*��    �ߣ�
*��    �ڣ�2019/12/18
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
