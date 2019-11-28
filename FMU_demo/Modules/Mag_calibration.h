#ifndef __MAG_CALIBRATION_H
#define __MAG_CALIBRATION_H

#include "stdio.h"
#include "math.h"
#include "stm32f4xx_hal.h"

void Mag_Calibration_update(void);
uint8_t get_Mag_Calibration_Instrction_Step(void);
void is_DireectionZ_Mag_Cal_Ready_offset(void);  

extern d_Cali_Data *pCali_Data;
unsigned char get_Mag_ajust_step(void);
void Get_calibration_offset(void);
unsigned char get_flight_direction(void);
void ACC_Calibration_update(void);	
void Gyroad_Initial_Offset(void);
	
#endif 

