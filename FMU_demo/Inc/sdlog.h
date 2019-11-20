#ifndef __SDLOG_H
#define __SDLOG_H
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "ff.h"

#define NUM_MAX_LOG          10

typedef enum{
	LOG_SENSOR_RAW = 0,
	LOG_FILTERED_EULER,
	LOG_RCIN_PWMOUT,
	LOG_CONTROL,
	LOG_NONE
}Log_Type;

typedef enum{
	Logger_Open = 0,
	Logger_Idle,
	Logger_Close,
	Logger_Record
}Logger_Status;

uint8_t Open_Log_File(void);
void Close_Log_File(void);
uint8_t Log_To_File(Log_Type log_type);

void Logger_Update(void);
void Logger_Enable(void);
void Logger_Disable(void);
#endif 
