#ifndef __SENSOR_H
#define __SENSOR_H

#include "md_include.h"

#define MPU6000_ONE_G					9.80665f
#define GYRO_CutoffFreq       30
#define ACC_CutoffFreq        30

void Filter_init(void);
void Transform(float Aframex,float Aframey,float Aframez,float *Bframex,float *Bframey,float *Bframez);
void IMU_Data_Combine(void); 
void Get_Gyro_Offset(void);


#endif

