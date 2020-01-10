/*******************************************************************************
* 文件名称：mavlink_main.c
*
* 摘    要：mavlink自定义文件
*
* 当前版本：
* 作    者：
* 日    期：2018/05/30
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#include "mavlink.h"
#include "mavlink_types.h"

#include "usart.h"
#include "md_struct.h"
#include "ringbuffer.h"
#include "stdio.h"

//#include "md_gps.h"
#include "md_ubx.h"

#include "md_led_driver.h"

#define  MAVLINK_READ_BUFFER_SIZE 128

/*缓冲区管理器*/
//ringbuffer管理变量
RingBuffer  m_Mavlink_RX_RingBuffMgr;
/*MAVLINK接收数据缓冲区数组*/
uint8_t   m_Mavlink_RX_Buff[MAVLINK_READ_BUFFER_SIZE*10];
uint8_t mavlink_byte;
extern float des_heading;
/*******************************函数声明****************************************
* 函数名称: void Mavlink_RB_Init(void)
* 输入参数:
* 返回参数:
* 功    能: 初始化一个循环队列，用来管理接收到的串口数据。其实就是一个数据缓冲区
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Init(void)
{
	//将m_Mavlink_RX_Buffm_Mavlink_RX_RingBuffMgr环队列进行关联管理。
	rbInit(&m_Mavlink_RX_RingBuffMgr, m_Mavlink_RX_Buff, sizeof(m_Mavlink_RX_Buff));
}

/*******************************函数声明****************************************
* 函数名称: void Mavlink_RB_Clear(void)
* 输入参数:
* 返回参数:
* 功    能: 归零ringbuffer里面的设置。
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Clear(void)
{
	rbClear(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: uint8_t  Mavlink_RB_IsOverFlow(void)
* 输入参数:
* 返回参数:  溢出为1，反之为0
* 功    能: 判断缓冲器是否溢出
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
uint8_t  Mavlink_RB_IsOverFlow(void)
{
	return m_Mavlink_RX_RingBuffMgr.flagOverflow;
}

/*******************************函数声明****************************************
* 函数名称: void Mavlink_RB_Push(uint8_t data)
* 输入参数: data：待压入的数据
* 返回参数:
* 功    能: 将接收的数据压入缓冲区
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Push(uint8_t data)
{
	rbPush(&m_Mavlink_RX_RingBuffMgr, (uint8_t)(data & (uint8_t)0xFFU));
}

/*******************************函数声明****************************************
* 函数名称: uint8_t Mavlink_RB_Pop(void)
* 输入参数:
* 返回参数: uint8_t 读出的数据
* 功    能: 从缓冲器读出数据
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
uint8_t Mavlink_RB_Pop(void)
{
	return rbPop(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: uint8_t Mavlink_RB_HasNew(void)
* 输入参数:
* 返回参数:
* 功    能: 判断是否有新的数据
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
uint8_t Mavlink_RB_HasNew(void)
{
	return !rbIsEmpty(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: uint16_t Mavlink_RB_Count(void)
* 输入参数:
* 返回参数:
* 功    能: 判断有多少个新数据
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
uint16_t Mavlink_RB_Count(void)
{
	return rbGetCount(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: void Mavlink_Rece_Enable(void)
* 输入参数:
* 返回参数:
* 功    能: 使能DMA接收
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlink_Rece_Enable(void)
{
	HAL_UART_Receive_DMA(&huart8, &mavlink_byte, 1);
}

/*******************************函数声明****************************************
* 函数名称: void Mavlink_Init(void)
* 输入参数:
* 返回参数:
* 功    能: 初始化MAVLINK：使能接收，ringbuffer关联
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlink_Init(void)
{
	Mavlink_Rece_Enable();
	Mavlink_RB_Init();
}

/*******************************函数声明****************************************
* 函数名称: void Mavlin_RX_InterruptHandler(void)
* 输入参数:
* 返回参数:
* 功    能: 串口中断的处理函数，主要是讲数据压入ringbuffer管理器
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlin_RX_InterruptHandler(void)
{
	//数据压入
	rbPush(&m_Mavlink_RX_RingBuffMgr,mavlink_byte);
}
/*在“mavlink_helpers.h中需要使用”*/
mavlink_system_t mavlink_system =
{
	1,
	1
}; // System ID, 1-255, Component/Subsystem ID, 1-255

/*******************************函数声明****************************************
* 函数名称: void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
* 输入参数:
* 返回参数:
* 功    能: 重新定义mavlink的发送函数，与底层硬件接口关联起来
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
	HAL_UART_Transmit(&huart8, (uint8_t *)ch, length, 2000);
}

mavlink_heartbeat_t my_heartbeat;
mavlink_command_long_t my_command_long;
mavlink_mission_count_t my_mission_count;
mavlink_mission_item_int_t my_mission_item_int;
mavlink_mission_item_t my_mission_item;
uint16_t _wpCurrent=0;
uint16_t _wpAttempt=0;
uint16_t _count=0;
/*******************************函数声明****************************************
* 函数名称: void Mavlink_Msg_Handle(void)
* 输入参数:
* 返回参数:
* 功    能: 处理从QGC上位机传过来的数据信息
* 作    者: by 
* 日    期: 2019/11/28
*******************************************************************************/
void Mavlink_Msg_Handle(mavlink_message_t msg)
{
	switch(msg.msgid)
	{
//		case MAVLINK_MSG_ID_HEARTBEAT:
////			printf("this is heartbeat from QGC\r\n");
//			mavlink_msg_heartbeat_decode(&msg,&my_heartbeat);
//			break;
		case MAVLINK_MSG_ID_SYS_STATUS:
//			printf("this is sys status from QGC\r\n");
	//		  osd_vbat = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
	//			osd_curr = mavlink_msg_sys_status_get_current_battery(&msg); //Battery current, in 10*milliamperes (1 = 10 milliampere)
	//			osd_battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg); //Remaining battery energy: (0%: 0, 100%: 100)
			break;
		case MAVLINK_MSG_ID_MISSION_REQUEST:
			break;
		case MAVLINK_MSG_ID_COMMAND_INT:
			printf("this is nav waypoint command from QGC\r\n");
			break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
			mavlink_msg_command_long_decode(&msg,&my_command_long);
			mavlink_msg_command_ack_send(MAVLINK_COMM_0, my_command_long.command, MAV_MISSION_ACCEPTED);
			break;
		case MAVLINK_MSG_ID_MISSION_COUNT:
			mavlink_msg_mission_count_decode(&msg,&my_mission_count);
		  _count=my_mission_count.count;
		  _wpCurrent = 0;
//		  mavlink_msg_mission_request_send(MAVLINK_COMM_0,my_mission_count.target_system,my_mission_count.target_component, _wpCurrent);
			mavlink_msg_mission_request_int_send(MAVLINK_COMM_0,my_mission_count.target_system,my_mission_count.target_component, _wpCurrent);
			break;
		case MAVLINK_MSG_ID_MISSION_ITEM:
			mavlink_msg_mission_item_decode(&msg,&my_mission_item);
			
			_wpCurrent = my_mission_item.seq + 1;
			if (my_mission_item.seq == _count-1)
			{
				point_onetest.lat = my_mission_item.x;
				point_onetest.lon = my_mission_item.y;
				
				mavlink_msg_mission_ack_send(MAVLINK_COMM_0,my_mission_item.target_system,my_mission_item.target_component,MAV_MISSION_ACCEPTED);
			}
			else if(my_mission_item.seq < _count-1)
			{
				mavlink_msg_mission_request_send(MAVLINK_COMM_0,my_mission_count.target_system,my_mission_count.target_component, _wpCurrent);
			}	
			break;
		
		case MAVLINK_MSG_ID_MISSION_ITEM_INT:
			mavlink_msg_mission_item_int_decode(&msg,&my_mission_item_int);
			
			_wpCurrent = my_mission_item_int.seq + 1;
			if (my_mission_item_int.seq == _count-1)
			{
				point_onetest.lat = (my_mission_item_int.x/(double)10000000);
				point_onetest.lon = (my_mission_item_int.y/(double)10000000);
				mavlink_msg_mission_ack_send(MAVLINK_COMM_0,my_mission_item_int.target_system,my_mission_item_int.target_component,MAV_MISSION_ACCEPTED);
			}
			else if(my_mission_item_int.seq < _count-1)
			{
				mavlink_msg_mission_request_int_send(MAVLINK_COMM_0,my_mission_item_int.target_system,my_mission_item_int.target_component, _wpCurrent);
			}	
			break;
		default:
			break;
	}
}


/*******************************函数声明****************************************
* 函数名称: Loop_Mavlink_Parse(void)
* 输入参数:
* 返回参数:
* 功    能: 在main函数中不间断调用此函数
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
mavlink_message_t msg;
mavlink_status_t status;

void Loop_Mavlink_Parse(void)
{
	if(Mavlink_RB_IsOverFlow())
	{
		Mavlink_RB_Clear();
	}

	while(Mavlink_RB_HasNew())
	{
		uint8_t read = Mavlink_RB_Pop();
		if(mavlink_parse_char(MAVLINK_COMM_0, read, &msg, &status))
		{
			//信号处理函数
			Mavlink_Msg_Handle(msg);
//			printf("Received message with ID %d, sequence: %d from component %d of system %d\r\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
		}
	}
}


/*测试函数代码区******************************************************************************/
extern MPU6000_Data 		m_Mpu6000;
extern MS56XX_Data 	  m_Ms56xx;
extern hmc5883MagData 	m_Hmc5883;
extern Control_state  Ctrl_state;


static void mavlink_test_heartbeat2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
	if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HEARTBEAT >= 256)
	{
		return;
	}
#endif
//	mavlink_message_t msg;
//	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
//	uint16_t i;
	mavlink_heartbeat_t packet_in =
	{
		963497464,17,84,151,218,3
	};
	mavlink_heartbeat_t packet1, packet2;
	memset(&packet1, 0, sizeof(packet1));
	packet1.custom_mode = packet_in.custom_mode;
	packet1.type = packet_in.type;
	packet1.autopilot = packet_in.autopilot;
	packet1.base_mode = packet_in.base_mode;
	packet1.system_status = packet_in.system_status;
	packet1.mavlink_version = packet_in.mavlink_version;


#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
	{
		// cope with extensions
		memset(MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN);
	}
#endif
	memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0 , packet1.type , packet1.autopilot , packet1.base_mode , packet1.custom_mode , packet1.system_status );
	MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}


static void mavlink_test_raw_imu2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
	if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RAW_IMU >= 256)
	{
		return;
	}
#endif
//	mavlink_message_t msg;
//	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
//	uint16_t i;
	mavlink_raw_imu_t packet1;
	memset(&packet1, 0, sizeof(packet1));
	packet1.time_usec =0123456;
	packet1.xacc = m_Mpu6000.acc_raw.x;
	packet1.yacc = m_Mpu6000.acc_raw.y;
	packet1.zacc = m_Mpu6000.acc_raw.z;
	packet1.xgyro = m_Mpu6000.gyro_raw.x;
	packet1.ygyro = m_Mpu6000.gyro_raw.y;
	packet1.zgyro = m_Mpu6000.gyro_raw.z;
	packet1.xmag = (int16_t)m_Hmc5883.x;
	packet1.ymag = (int16_t)m_Hmc5883.y;
	packet1.zmag = (int16_t)m_Hmc5883.z;

#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
	{
		// cope with extensions
		memset(MAVLINK_MSG_ID_RAW_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RAW_IMU_MIN_LEN);
	}
#endif

	mavlink_msg_raw_imu_send(MAVLINK_COMM_0 , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag );
}


static void mavlink_test_highres_imu2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
	if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RAW_IMU >= 256)
	{
		return;
	}
#endif
//	mavlink_message_t msg;
//	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
//	uint16_t i;
	mavlink_highres_imu_t packet1;
	memset(&packet1, 0, sizeof(packet1));
	packet1.time_usec =0123456;
	packet1.xacc = m_Mpu6000.accf.x;
	packet1.yacc = m_Mpu6000.accf.y;
	packet1.zacc = m_Mpu6000.accf.z;
	packet1.xgyro = m_Mpu6000.gyrof.x;
	packet1.ygyro = m_Mpu6000.gyrof.y;
	packet1.zgyro = m_Mpu6000.gyrof.z;
	packet1.xmag = m_Hmc5883.x;
	packet1.ymag = m_Hmc5883.y;
	packet1.zmag = m_Hmc5883.z;

#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
	{
		// cope with extensions
		memset(MAVLINK_MSG_ID_RAW_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RAW_IMU_MIN_LEN);
	}
#endif

	mavlink_msg_highres_imu_send(MAVLINK_COMM_0 , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag,0,0,0,0,0 );
}

static void mavlink_test_hil_sensor2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
	if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RAW_IMU >= 256)
	{
		return;
	}
#endif
//	mavlink_message_t msg;
//	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
//	uint16_t i;
	mavlink_hil_sensor_t packet1;
	memset(&packet1, 0, sizeof(packet1));
	packet1.time_usec =0123456;
	packet1.xacc = Ctrl_state.accf.x;
	packet1.yacc = Ctrl_state.accf.y;
	packet1.zacc = Ctrl_state.accf.z;
	packet1.xgyro = Ctrl_state.gyrof.x;
	packet1.ygyro = Ctrl_state.gyrof.y;
	packet1.zgyro = Ctrl_state.gyrof.z;
	packet1.xmag = Ctrl_state.magf.x;
	packet1.ymag = Ctrl_state.magf.y;
	packet1.zmag = Ctrl_state.magf.z;

#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
	{
		// cope with extensions
		memset(MAVLINK_MSG_ID_RAW_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RAW_IMU_MIN_LEN);
	}
#endif

	mavlink_msg_hil_sensor_send(MAVLINK_COMM_0 , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag,0,0,0,0,0 );
}

static void mavlink_test_attitude2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
	if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RAW_IMU >= 256)
	{
		return;
	}
#endif
//	mavlink_message_t msg;
//	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
//	uint16_t i;
	mavlink_attitude_t packet1;
	memset(&packet1, 0, sizeof(packet1));
	packet1.time_boot_ms =0123456;
	packet1.roll = (float)jy901_state.angle.roll/180.0f*PI;
	packet1.pitch = (float)jy901_state.angle.pitch/180.0f*PI;
	packet1.yaw = (float)des_heading/180.0f*PI;


#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
	{
		// cope with extensions
		memset(MAVLINK_MSG_ID_RAW_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RAW_IMU_MIN_LEN);
	}
#endif

	mavlink_msg_attitude_send(MAVLINK_COMM_0 , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , 0 , 0 , 0 );
}

static void mavlink_test_raw_gps2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
	if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_RAW_INT >= 256)
	{
		return;
	}
#endif
//	mavlink_message_t msg;
//	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
//	uint16_t i;
	mavlink_gps_raw_int_t packet1;
	memset(&packet1, 0, sizeof(packet1));
	
	packet1.time_usec =(uint64_t)123456;
	packet1.lat = _gps_position->lat;
	packet1.lon = _gps_position->lon;
	packet1.alt = _gps_position->alt;
	packet1.eph = _gps_position->eph;
	packet1.epv = _gps_position->epv;
	packet1.vel = _gps_position->vel_m_s*100;
	packet1.cog = (uint16_t)(_gps_position->cog_rad*343774);
	packet1.fix_type = _gps_position->fix_type;
	packet1.satellites_visible = _gps_position->satellites_used;

#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
	{
		// cope with extensions
		memset(MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN);
	}
#endif

	mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0 , packet1.time_usec , packet1.fix_type ,packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible);
}

void mavlink_test(void)
{
	static uint16_t test_count=0;
	mavlink_message_t lastmsg;
	test_count++;
	//5hz
	if((test_count%100)==0)
	{
		mavlink_test_heartbeat2(1,1,&lastmsg);
		mavlink_test_raw_gps2(1,1,&lastmsg);
	}
	if((test_count%10)==0)
	{
		mavlink_test_attitude2(1,1,&lastmsg);
//		mavlink_test_raw_imu2(1,1,&lastmsg);
//		mavlink_test_highres_imu2(1,1,&lastmsg);
//		mavlink_test_hil_sensor2(1,1,&lastmsg);
	}
//	if((test_count%10)==0)
//	{

//	}

	
}
