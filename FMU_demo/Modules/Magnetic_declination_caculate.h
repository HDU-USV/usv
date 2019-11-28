
#ifndef __MAGNETIC_DECLICATION_CACULATE_H
#define __MAGNETIC_DECLICATION_CACULATE_H

#ifdef __cplusplus
extern "C" {
#endif 
#include "stm32f4xx_hal.h"  
  
float _get_mag_declination(double lat, double lon);  
float  get_mag_declination(double lat, double lon);

#ifdef __cplusplus
}
#endif

#endif 
