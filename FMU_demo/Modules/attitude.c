/*******************************************************************************
* 文件名称：attitude.c
*
* 摘    要：姿态库
*
* 当前版本：
* 作    者：
* 日    期：2019/12/10
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#include "attitude.h"
#include "md_struct.h"
#include "JY901.h"
#include "math.h"



/* 四元数姿态算法 */

//四元数,使用全局变量来保持上一次的四元数值  
double x = 0.0;	
double y = 0.0;
double z = 0.0;
double w = 1.0;	 		  

//核心算法1：欧拉角转四元数	
Quaternion from_euler_angle(Euler_Angle ea)
{
	static Quaternion out;

    double cos_roll  = cos(ea.roll * 0.5);
    double sin_roll  = sin(ea.roll * 0.5);
    double cos_pitch = cos(ea.pitch * 0.5);
    double sin_pitch = sin(ea.pitch * 0.5);
    double cos_yaw   = cos(ea.yaw * 0.5);
    double sin_yaw   = sin(ea.yaw * 0.5);

    out.w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
    out.x = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
    out.y = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
    out.z = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;

	return out;
}		 

//核心算法2：四元数转欧拉角	
double clamp(double a, double min, double max)
{
	if(a > max) return max;
	if(a < min)	return min;
	return a;
}

Euler_Angle to_euler_angle(void) 
{
    Euler_Angle ea;

    ea.roll  = atan2(2.0 * (w * z + x * y) , 1.0 - 2.0  * (z * z + x * x));
    ea.pitch = asin (clamp(2.0  * (w * x - y * z) , -1.0 , 1.0));	  
    ea.yaw   = atan2(2.0 * (w * y + z * x) , 1.0 - 2.0  * (x * x + y * y));

    return ea;
}

//核心算法3，四元数乘法
void normalize(Quaternion *c)
{
    double s = sqrt(c->w*c->w + c->x*c->x + c->y*c->y + c->z*c->z);

   	c->w /= s;
    c->x /= s;
    c->y /= s;
    c->z /= s;
}
//四元数乘法
 Quaternion multiply(Quaternion b)
{
    Quaternion c;

    c.w = w*b.w - x*b.x - y*b.y - z*b.z;
    c.x = w*b.x + x*b.w + y*b.z - z*b.y;
    c.y = w*b.y - x*b.z + y*b.w + z*b.x;
    c.z = w*b.z + x*b.y - y*b.x + z*b.w;

    normalize(&c);

    return c;
}

/* 更新四元数 */
void update_quaternion(Euler_Angle angle_in)
{
	Quaternion now;
	
	now = from_euler_angle(angle_in);			//转换欧拉角为四元数
	now = multiply(now);						//四元数乘法

	x = now.x;									//保存本次四元数结果
	y =	now.y;
	z = now.z;
	w = now.w;	
}

/* posture_computer 姿态计算函数 */
//输入 最近的角速度
//输出 当前姿态
Euler_Angle posture_computer(Euler_Angle angle_in)
{
	Quaternion now;

	now = from_euler_angle(angle_in);				//转换欧拉角为四元数
	now = multiply(now);						//四元数乘法

	x = now.x;									//保存本次四元数结果
	y =	now.y;
	z = now.z;
	w = now.w;	

	return to_euler_angle();						//直接用已经保存的四元数来计算输出欧拉角
}

void posture_update(void)
{
	Euler_Angle in;
	Euler_Angle out;
	
	in.roll = jy901_state.angle.roll /360 *PI;
	in.pitch = jy901_state.angle.pitch /360 *PI;
	in.yaw = jy901_state.angle.yaw /360 *PI;
	
}
