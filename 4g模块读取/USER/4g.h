#include "stm32f4xx.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
u8* lte4g_check_cmd(u8* str);
u8 lte4g_send_cmd(u8 *cmd,u8 *ack,u16 waittime);
void init_4g();