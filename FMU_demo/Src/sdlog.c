/*******************************************************************************
* 文件名称：sdlog.c
*
* 摘    要：记录飞行器状态日志
*
* 当前版本：
* 作    者：Acorss工作室
* 日    期：2018/07/20
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#include "sdlog.h"
#include "stdlib.h"
#include "stdio.h"
#include "md_struct.h"


FIL log_file;
FIL index_file;

extern MPU6000_Data 		m_Mpu6000;
extern hmc5883MagData 	m_Hmc5883;
extern MS56XX_Data 	    m_Ms56xx;


UINT log_num;
char log_buffer[512];
#define MAX_NUM_RETRY      3

//记录文件个数
uint8_t index_log;

/*******************************函数声明****************************************
* 函数名称: uint8_t Open_Log_File(void)
* 输入参数:
* 返回参数:
* 功    能: 打开或者创建日志文件。
* 作    者: by Across工作室
* 日    期: 2018/07/21
*******************************************************************************/
uint8_t Open_Log_File(void)
{
	uint8_t res;
	uint8_t len_filename;
	char file_name[10];	
  //利用sprintf函数将XX.TXT字符写入filename；
	len_filename = sprintf(file_name, "%02d.txt", index_log);
	
	//index 创建目录TXT，并将“XX.TXT”写入到index.txt中
	f_open(&index_file, "index", FA_CREATE_ALWAYS | FA_WRITE);
	res = f_write(&index_file, file_name, len_filename, &log_num);
	//printf("index write res:%d\r\n", res);
	f_close(&index_file);
  //
	res = f_open(&log_file, file_name, FA_CREATE_ALWAYS | FA_WRITE);

	if(res) {

		//printf("create log file failed");
		
	} else {
		
		//printf("create file success: %02d.txt\n", index_log);
		index_log++;
		
	}		
	return res;
}

/*******************************函数声明****************************************
* 函数名称: uint8_t Close_Log_File(void)
* 输入参数:
* 返回参数:
* 功    能: 关闭日志文件。每次操作完一定要关闭。
* 作    者: by Across工作室
* 日    期: 2018/07/21
*******************************************************************************/
void Close_Log_File(void)
{
	f_close(&log_file);
}

/*******************************函数声明****************************************
* 函数名称: uint8_t Log_To_File(log_type_e log_type)
* 输入参数:
* 返回参数:
* 功    能: 将数据填入到日志中
* 作    者: by Across工作室
* 日    期: 2018/07/21
*******************************************************************************/
uint8_t Log_To_File(Log_Type log_type)
{
	uint16_t len;
	
	switch(log_type){
		case LOG_SENSOR_RAW:
			len = sprintf(log_buffer, "%4d %4d %4d %4d %4d %4d %8.3f %8.3f %8.3f\r\n", m_Mpu6000.acc_raw.x, m_Mpu6000.acc_raw.y, m_Mpu6000.acc_raw.z,
										m_Mpu6000.gyro_raw.x, m_Mpu6000.gyro_raw.y, m_Mpu6000.gyro_raw.z, m_Hmc5883.x, m_Hmc5883.y, m_Hmc5883.z);
			if(f_write(&log_file, log_buffer, len, &log_num)) return 1;
			break;
		
		case LOG_FILTERED_EULER:  
		
			break;
		
		case LOG_RCIN_PWMOUT:
			len = sprintf(log_buffer, "%4d %4d %4d %4d %4d %4d %4d %4d\r\n", m_RC_input_from_F1.channel1, m_RC_input_from_F1.channel2, m_RC_input_from_F1.channel3, m_RC_input_from_F1.channel4,
										m_RC_input_from_F1.channel5, m_RC_input_from_F1.channel6, m_RC_input_from_F1.channel7, m_RC_input_from_F1.channel8);
			if(f_write(&log_file, log_buffer, len, &log_num)) return 1;
			break;
		
		case LOG_CONTROL:
			break;
		
		default:
			break;
	}
	
	return 0;
}



/*******************************函数声明****************************************
* 函数名称: void Update_Logger(void)
* 输入参数:
* 返回参数:
* 功    能: 将数据填入到日志中
* 作    者: by Across工作室
* 日    期: 2018/07/21
*******************************************************************************/
//初始状态下
Log_Type m_Log_Type=LOG_SENSOR_RAW;
Logger_Status m_Logger_Status = Logger_Close; 
void Logger_Update(void)
{
	 uint8_t res=0;
  //1.根据日志管理器的状态实现响应的功能
	 switch (m_Logger_Status)
   {
		case(Logger_Idle):
			//do nothing;
   		break;
		 
   	case(Logger_Close):
			Close_Log_File();
		  m_Logger_Status = Logger_Idle;
   		break;
		//2.日志功能打开则创建日志文件	
   	case(Logger_Open):
			res = Open_Log_File();
		  
		  if(res)
			{
				//打开或者创建未能成功，则改变logger状态到Logger_Idle
				m_Logger_Status = Logger_Idle;
			}
			else
			{ 
				//打开或者创建成功，则改变logger状态到Logger_Record
		    m_Logger_Status = Logger_Record;
			}
   		break;
		//3.记录数据到日志文件
		case(Logger_Record):
			Log_To_File(m_Log_Type);
   		break;	
		
   	default:
			m_Logger_Status = Logger_Idle;
   		break;
   }	
}

/*******************************函数声明****************************************
* 函数名称: void Logger_Enable(void)
* 输入参数:
* 返回参数:
* 功    能: 使logger管理器状态切换到open
* 作    者: by Across工作室
* 日    期: 2018/07/21
*******************************************************************************/
void Logger_Enable(void)
{
  m_Logger_Status=Logger_Open;
}

/*******************************函数声明****************************************
* 函数名称: void Logger_Disable(void)
* 输入参数:
* 返回参数:
* 功    能: 使logger管理器状态切换到关闭
* 作    者: Across工作室
* 日    期: 2018/07/21
*******************************************************************************/
void Logger_Disable(void)
{
	if((m_Logger_Status == Logger_Record)||(m_Logger_Status == Logger_Open))
  {
	  m_Logger_Status=Logger_Close;
	} 
}
