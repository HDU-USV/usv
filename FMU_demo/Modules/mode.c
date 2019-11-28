/*******************************************************************************
* 文件名称：mode.c
*
* 摘    要：航行模式切换
*
* 当前版本：
* 作    者：
* 日    期：2019/11/27
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#include "mode.h"

Mode current_mode;
void Mode_Update(void)
{
	switch(current_mode){
		case Manual:break;
		case Auto_WP:break;
		case Auto_RTL:break;
		default: break;
	}
}