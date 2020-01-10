#ifndef __CONTROL_H
#define __CONTROL_H

#include "md_struct.h"

extern char control_flag,limit_flag;
extern int16_t turn_duty;

void turn_pid_init(void);
void heading_control(void);

#endif

