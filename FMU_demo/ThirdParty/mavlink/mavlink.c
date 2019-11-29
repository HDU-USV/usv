/*******************************************************************************
* �ļ����ƣ�mavlink_main.c
*
* ժ    Ҫ��mavlink�Զ����ļ�
*
* ��ǰ�汾��
* ��    �ߣ�
* ��    �ڣ�2018/05/30
* ���뻷����keil5
*
* ��ʷ��Ϣ��
*******************************************************************************/

#include "mavlink.h"
#include "mavlink_types.h"

#include "usart.h"
#include "md_struct.h"
#include "ringbuffer.h"
#include "stdio.h"

#include "md_led_driver.h"

#define  MAVLINK_READ_BUFFER_SIZE 128

/*������������*/
//ringbuffer�������
RingBuffer  m_Mavlink_RX_RingBuffMgr;
/*MAVLINK�������ݻ���������*/
uint8_t   m_Mavlink_RX_Buff[MAVLINK_READ_BUFFER_SIZE*10];
uint8_t mavlink_byte;
/*******************************��������****************************************
* ��������: void Mavlink_RB_Init(void)
* �������:
* ���ز���:
* ��    ��: ��ʼ��һ��ѭ�����У�����������յ��Ĵ������ݡ���ʵ����һ�����ݻ�����
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Init(void)
{
	//��m_Mavlink_RX_Buffm_Mavlink_RX_RingBuffMgr�����н��й�������
	rbInit(&m_Mavlink_RX_RingBuffMgr, m_Mavlink_RX_Buff, sizeof(m_Mavlink_RX_Buff));
}

/*******************************��������****************************************
* ��������: void Mavlink_RB_Clear(void)
* �������:
* ���ز���:
* ��    ��: ����ringbuffer��������á�
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Clear(void)
{
	rbClear(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************��������****************************************
* ��������: uint8_t  Mavlink_RB_IsOverFlow(void)
* �������:
* ���ز���:  ���Ϊ1����֮Ϊ0
* ��    ��: �жϻ������Ƿ����
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
uint8_t  Mavlink_RB_IsOverFlow(void)
{
	return m_Mavlink_RX_RingBuffMgr.flagOverflow;
}

/*******************************��������****************************************
* ��������: void Mavlink_RB_Push(uint8_t data)
* �������: data����ѹ�������
* ���ز���:
* ��    ��: �����յ�����ѹ�뻺����
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Push(uint8_t data)
{
	rbPush(&m_Mavlink_RX_RingBuffMgr, (uint8_t)(data & (uint8_t)0xFFU));
}

/*******************************��������****************************************
* ��������: uint8_t Mavlink_RB_Pop(void)
* �������:
* ���ز���: uint8_t ����������
* ��    ��: �ӻ�������������
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
uint8_t Mavlink_RB_Pop(void)
{
	return rbPop(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************��������****************************************
* ��������: uint8_t Mavlink_RB_HasNew(void)
* �������:
* ���ز���:
* ��    ��: �ж��Ƿ����µ�����
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
uint8_t Mavlink_RB_HasNew(void)
{
	return !rbIsEmpty(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************��������****************************************
* ��������: uint16_t Mavlink_RB_Count(void)
* �������:
* ���ز���:
* ��    ��: �ж��ж��ٸ�������
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
uint16_t Mavlink_RB_Count(void)
{
	return rbGetCount(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************��������****************************************
* ��������: void Mavlink_Rece_Enable(void)
* �������:
* ���ز���:
* ��    ��: ʹ��DMA����
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
void Mavlink_Rece_Enable(void)
{
	HAL_UART_Receive_DMA(&huart8, &mavlink_byte, 1);
}

/*******************************��������****************************************
* ��������: void Mavlink_Init(void)
* �������:
* ���ز���:
* ��    ��: ��ʼ��MAVLINK��ʹ�ܽ��գ�ringbuffer����
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
void Mavlink_Init(void)
{
	Mavlink_Rece_Enable();
	Mavlink_RB_Init();
}

/*******************************��������****************************************
* ��������: void Mavlin_RX_InterruptHandler(void)
* �������:
* ���ز���:
* ��    ��: �����жϵĴ���������Ҫ�ǽ�����ѹ��ringbuffer������
* ��    ��: by Across
* ��    ��: 2018/06/02
*******************************************************************************/
void Mavlin_RX_InterruptHandler(void)
{
	//����ѹ��
	rbPush(&m_Mavlink_RX_RingBuffMgr,mavlink_byte);
}
/*�ڡ�mavlink_helpers.h����Ҫʹ�á�*/
mavlink_system_t mavlink_system =
{
	1,
	1
}; // System ID, 1-255, Component/Subsystem ID, 1-255

/*******************************��������****************************************
* ��������: void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
* �������:
* ���ز���:
* ��    ��: ���¶���mavlink�ķ��ͺ�������ײ�Ӳ���ӿڹ�������
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
	HAL_UART_Transmit(&huart8, (uint8_t *)ch, length, 2000);
}

mavlink_heartbeat_t my_heartbeat;
mavlink_command_long_t my_command_long;
mavlink_mission_count_t my_mission_count;
mavlink_mission_item_int_t my_mission_item_int;
uint16_t _transfer_seq=0;
/*******************************��������****************************************
* ��������: void Mavlink_Msg_Handle(void)
* �������:
* ���ز���:
* ��    ��: �����QGC��λ����������������Ϣ
* ��    ��: by 
* ��    ��: 2019/11/28
*******************************************************************************/
void Mavlink_Msg_Handle(mavlink_message_t msg)
{
	switch(msg.msgid)
	{
		case MAVLINK_MSG_ID_HEARTBEAT:
			printf("this is heartbeat from QGC\r\n");
			mavlink_msg_heartbeat_decode(&msg,&my_heartbeat);
			break;
		case MAVLINK_MSG_ID_SYS_STATUS:
			printf("this is sys status from QGC\r\n");
	//		  osd_vbat = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
	//			osd_curr = mavlink_msg_sys_status_get_current_battery(&msg); //Battery current, in 10*milliamperes (1 = 10 milliampere)
	//			osd_battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg); //Remaining battery energy: (0%: 0, 100%: 100)
			break;
		case MAVLINK_MSG_ID_COMMAND_INT:
			printf("this is nav waypoint command from QGC\r\n");
			break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
			mavlink_msg_command_long_decode(&msg,&my_command_long);
			mavlink_msg_command_ack_send(MAVLINK_COMM_0, my_command_long.command, MAV_MISSION_ACCEPTED,0,0,0,0);
			break;
		case MAVLINK_MSG_ID_MISSION_COUNT:
			mavlink_msg_mission_count_decode(&msg,&my_mission_count);;
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0,my_mission_count.target_system,my_mission_count.target_component,MAV_MISSION_ACCEPTED,0);
//			mavlink_msg_mission_request_list_send(MAVLINK_COMM_0,my_mission_count.target_system,my_mission_count.target_component, 0);
		if(_transfer_seq < my_mission_count.count) 
		{
			mavlink_msg_mission_request_send(MAVLINK_COMM_0,my_mission_count.target_system,my_mission_count.target_component, _transfer_seq,0);
			_transfer_seq++;
		}
		else 
		_transfer_seq=0;
//		  mavlink_msg_mission_request_int_send(MAVLINK_COMM_0,my_mission_count.target_system,my_mission_count.target_component, 0,0);
			break;
		case MAVLINK_MSG_ID_MISSION_ITEM_INT:
			mavlink_msg_mission_item_int_decode(&msg,&my_mission_item_int);
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0,my_mission_item_int.target_system,my_mission_item_int.target_component,MAV_MISSION_ACCEPTED,0);
			break;
		default:
			break;
	}
}


/*******************************��������****************************************
* ��������: Loop_Mavlink_Parse(void)
* �������:
* ���ز���:
* ��    ��: ��main�����в���ϵ��ô˺���
* ��    ��: by 
* ��    ��: 2018/06/02
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
			//�źŴ�����
			Mavlink_Msg_Handle(msg);
			printf("Received message with ID %d, sequence: %d from component %d of system %d\r\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
		}
	}
}


/*���Ժ���������******************************************************************************/
extern MPU6000_Data 		m_Mpu6000;
extern MS56XX_Data 	  m_Ms56xx;
extern hmc5883MagData 	m_Hmc5883;

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


static void mavlink_test_raw_gps(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
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
	packet1.lat = 302800000;
	packet1.lon = 1201500000;
	packet1.alt = 1;
	packet1.eph = 250;
	packet1.epv = 3440;
	packet1.vel = 11520;
	packet1.cog = 0;
	packet1.fix_type = GPS_FIX_TYPE_2D_FIX;
	packet1.satellites_visible = 8;

#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
	{
		// cope with extensions
		memset(MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN);
	}
#endif

	mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0 , packet1.time_usec , packet1.fix_type ,packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible ,packet1.alt_ellipsoid ,packet1.h_acc, packet1.v_acc, packet1.vel_acc, packet1.hdg_acc);
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
	}
	if((test_count%50)==0)
	{
		mavlink_test_raw_imu2(1,1,&lastmsg);
	}
	if((test_count%500)==0)
	{
		mavlink_test_raw_gps(1,1,&lastmsg);
	}
	
}
