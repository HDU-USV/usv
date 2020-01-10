#include "md_struct.h"

/* 传感器数据处理和姿态解算结构体 */
Control_state jy901_state;

/* 遥控器通道定义的结构体*/
RC_Channel m_RC_input_from_F1;

/* 电机定义的结构体*/
Motor_Value Motor; 

/*四元数计算用到的结构体*/
Quaternion Q;

/*欧拉角*/
Euler_Angle Angle;

Way_Point point_onetest;
