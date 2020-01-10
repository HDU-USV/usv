/*******************************************************************************
* �ļ����ƣ�sdlog.c
*
* ժ    Ҫ����¼������״̬��־
*
* ��ǰ�汾��
* ��    �ߣ�Acorss������
* ��    �ڣ�2018/07/20
* ���뻷����keil5
*
* ��ʷ��Ϣ��
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

//��¼�ļ�����
uint8_t index_log;

/*******************************��������****************************************
* ��������: uint8_t Open_Log_File(void)
* �������:
* ���ز���:
* ��    ��: �򿪻��ߴ�����־�ļ���
* ��    ��: by Across������
* ��    ��: 2018/07/21
*******************************************************************************/
uint8_t Open_Log_File(void)
{
	uint8_t res;
	uint8_t len_filename;
	char file_name[10];	
  //����sprintf������XX.TXT�ַ�д��filename��
	len_filename = sprintf(file_name, "%02d.txt", index_log);
	
	//index ����Ŀ¼TXT��������XX.TXT��д�뵽index.txt��
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

/*******************************��������****************************************
* ��������: uint8_t Close_Log_File(void)
* �������:
* ���ز���:
* ��    ��: �ر���־�ļ���ÿ�β�����һ��Ҫ�رա�
* ��    ��: by Across������
* ��    ��: 2018/07/21
*******************************************************************************/
void Close_Log_File(void)
{
	f_close(&log_file);
}

/*******************************��������****************************************
* ��������: uint8_t Log_To_File(log_type_e log_type)
* �������:
* ���ز���:
* ��    ��: ���������뵽��־��
* ��    ��: by Across������
* ��    ��: 2018/07/21
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



/*******************************��������****************************************
* ��������: void Update_Logger(void)
* �������:
* ���ز���:
* ��    ��: ���������뵽��־��
* ��    ��: by Across������
* ��    ��: 2018/07/21
*******************************************************************************/
//��ʼ״̬��
Log_Type m_Log_Type=LOG_SENSOR_RAW;
Logger_Status m_Logger_Status = Logger_Close; 
void Logger_Update(void)
{
	 uint8_t res=0;
  //1.������־��������״̬ʵ����Ӧ�Ĺ���
	 switch (m_Logger_Status)
   {
		case(Logger_Idle):
			//do nothing;
   		break;
		 
   	case(Logger_Close):
			Close_Log_File();
		  m_Logger_Status = Logger_Idle;
   		break;
		//2.��־���ܴ��򴴽���־�ļ�	
   	case(Logger_Open):
			res = Open_Log_File();
		  
		  if(res)
			{
				//�򿪻��ߴ���δ�ܳɹ�����ı�logger״̬��Logger_Idle
				m_Logger_Status = Logger_Idle;
			}
			else
			{ 
				//�򿪻��ߴ����ɹ�����ı�logger״̬��Logger_Record
		    m_Logger_Status = Logger_Record;
			}
   		break;
		//3.��¼���ݵ���־�ļ�
		case(Logger_Record):
			Log_To_File(m_Log_Type);
   		break;	
		
   	default:
			m_Logger_Status = Logger_Idle;
   		break;
   }	
}

/*******************************��������****************************************
* ��������: void Logger_Enable(void)
* �������:
* ���ز���:
* ��    ��: ʹlogger������״̬�л���open
* ��    ��: by Across������
* ��    ��: 2018/07/21
*******************************************************************************/
void Logger_Enable(void)
{
  m_Logger_Status=Logger_Open;
}

/*******************************��������****************************************
* ��������: void Logger_Disable(void)
* �������:
* ���ز���:
* ��    ��: ʹlogger������״̬�л����ر�
* ��    ��: Across������
* ��    ��: 2018/07/21
*******************************************************************************/
void Logger_Disable(void)
{
	if((m_Logger_Status == Logger_Record)||(m_Logger_Status == Logger_Open))
  {
	  m_Logger_Status=Logger_Close;
	} 
}
