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
#include "control.h"
#include "math.h"
#include "md_ubx.h"


#define THROTTLE_MAX 2000
#define THROTTLE_HALF 1500
#define THROTTLE_MIN 1000
#define COMPENSATE_HALF 750
#define SET_SPEED 300

#define REDIUS_LIMIT 4.0
#define REDIUS_STOP 1.5

extern __IO ITStatus f_NewRCDataReady;

double cur_distance;
decode_move move_state=RUN;
#define EARTH_RADIUS  6371393

 double ConvertDegreesToRadians(double degrees)
{
		return degrees * PI / 180.0;
}

 double ConvertRadiansToDegrees(double radian)
{
		return radian * 180.0 / PI;
}

 double HaverSin(double theta)
{
		double v = sin(theta / 2);
		return v * v;
}

double Cal_Distance(double lat1,double lon1, double lat2,double lon2)		
{
	double vLon,vLat,h,distance;
	//用haversine公式计算球面两点间的距离。
	//经纬度转换成弧度
	lat1 = ConvertDegreesToRadians(lat1);
	lon1 = ConvertDegreesToRadians(lon1);
	lat2 = ConvertDegreesToRadians(lat2);
	lon2 = ConvertDegreesToRadians(lon2);

	//差值
	vLon = fabs(lon1 - lon2);
	vLat = fabs(lat1 - lat2);

	h = HaverSin(vLat) + cos(lat1) * cos(lat2) * HaverSin(vLon);

	distance = 2 * EARTH_RADIUS * asin(sqrt(h));

	return distance;
}	


void Motor_Stop(void)
{
		//STOP
	Motor.motor1 = THROTTLE_HALF;
	Motor.motor2 = THROTTLE_HALF;
	Motor.motor3 = THROTTLE_HALF;
	Motor.motor4 = THROTTLE_HALF;
	Motor.motor5 = THROTTLE_HALF;
	Motor.motor6 = THROTTLE_HALF;
}

void parse_move(double distance)
{
	switch(move_state)
	{
		case RUN:
		{
			if(distance < REDIUS_LIMIT)
				move_state = LIMIT;
			else 
			{
				
				Motor.motor2 = THROTTLE_HALF + SET_SPEED - turn_duty;
				Motor.motor1 = THROTTLE_HALF + SET_SPEED + turn_duty;	
			}break;		
		}
		case LIMIT:
		{
			if(distance < REDIUS_STOP)
				move_state = STOP;
			else 
			{
				Motor.motor2 = THROTTLE_HALF + SET_SPEED - turn_duty;
				Motor.motor1 = THROTTLE_HALF + SET_SPEED + turn_duty;	
			}break;	
		}
		case STOP:
		{
			Motor_Stop();
			point_onetest.lat=0;
			point_onetest.lon=0;		
			move_state = RUN;
		}break;
		default:Motor_Stop();
	}
}
void Example_Control(RC_Channel*rc_input)
{
	if(rc_input->channel8 < THROTTLE_HALF)
	{
		if(rc_input->channel3 < THROTTLE_HALF - 2)
		{
			Motor.motor1 = rc_input->channel3 - COMPENSATE_HALF + rc_input->channel1/2;				
			Motor.motor2 = rc_input->channel3 + COMPENSATE_HALF - rc_input->channel1/2;
		}
		else
		{
			Motor.motor1 = rc_input->channel3 + COMPENSATE_HALF - rc_input->channel1/2;
			Motor.motor2 = rc_input->channel3 - COMPENSATE_HALF + rc_input->channel1/2;
		}
//		Motor.motor3 = rc_input->channel3;
//		Motor.motor4 = rc_input->channel3;
//		Motor.motor5 = rc_input->channel3;
//		Motor.motor6 = rc_input->channel3;
		if (Motor.motor1 > THROTTLE_MAX)  
		{
			Motor.motor2 = Motor.motor2  + THROTTLE_MAX - Motor.motor1; 
			Motor.motor1 = THROTTLE_MAX;
		}
		else if (Motor.motor2 > THROTTLE_MAX)
		{
			Motor.motor1 = Motor.motor1  + THROTTLE_MAX - Motor.motor2; 
			Motor.motor2 = THROTTLE_MAX;		
		}
		else if (Motor.motor1 < THROTTLE_MIN)
		{
			Motor.motor2 = Motor.motor2  + THROTTLE_MIN - Motor.motor1; 
			Motor.motor1 = THROTTLE_MIN;
		}
		else if (Motor.motor2 < THROTTLE_MIN)
		{
			Motor.motor1 = Motor.motor1  + THROTTLE_MIN - Motor.motor2; 
			Motor.motor2 = THROTTLE_MIN;
		}
	}
	else 
	{	
		if(control_flag)
	 {

			if(point_onetest.lat !=0 && point_onetest.lon !=0)	
			{
				cur_distance=Cal_Distance(point_onetest.lat,point_onetest.lon,(double)(_gps_position->lat/10000000.0),(double)(_gps_position->lon/10000000.0));			

				parse_move(cur_distance);
//				if (cur_distance > REDIUS_STOP)
//				{
//					Motor.motor2 = THROTTLE_HALF + SET_SPEED - turn_duty;
//					Motor.motor1 = THROTTLE_HALF + SET_SPEED + turn_duty;					
//				}
//		    else
//			  {
//					Motor_Stop();
//				}				

			}
			else
			{
				Motor_Stop();
			}
		control_flag = 0;
	 }
 }
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
	 Com_TwoBoard_Msg_MotorValue_Send(&Motor);

	}
	 
}
