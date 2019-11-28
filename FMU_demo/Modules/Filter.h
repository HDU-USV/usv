#ifndef __FILTER_H
#define __FILTER_H

#include "md_include.h"

#define LPF_1_(hz,t,in,out) ((out) += ( 1.0f / ( 1.0f + 1.0f / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))

typedef struct
{
	 float           fc;
	 float           a1;
	 float           a2;
	 float           b0; 
	 float           b1;
	 float           b2;
	 float           y_1;        
	 float           y_2;       
}IIR_coeff_Typedef;

void  cal_iir_coeff(IIR_coeff_Typedef *coeff,float fs, float fc);
float get_iir_output(IIR_coeff_Typedef* coeff,float sample);
float LowPassFilter(float oldData, float newData, float cutoff_freq, float dt);

#endif
