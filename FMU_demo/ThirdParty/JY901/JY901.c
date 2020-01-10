#include "JY901.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "ringbuffer.h"
#include "md_struct.h"
#include "attitude.h"
#include "math.h"

#define CYCLE_50HZ_FROM_500HZ   10

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;



/*������������*/
//ringbuffer�������
RingBuffer  m_JY901_RX_RingBuffMgr;
/*GPS�������ݻ���������*/
uint8_t   m_JY901_RX_Buff[JY901_READ_BUFFER_SIZE*10];

//�������ݴ�ű���
uint8_t jy901_rx_temp;

__IO uint32_t JY901_RunCount=0;

void JY901_RB_Clear(void);

void JY901_InterruptHandler()
{
	//����ѹ��
	rbPush(&m_JY901_RX_RingBuffMgr,jy901_rx_temp);
}

 void JY901_Process(unsigned char ucData)
{
	static uint8_t ucRxBuffer[250];
	static uint8_t ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);
									jy901_state.accf.x=(float)stcAcc.a[0]/2048*9.8f; 
									jy901_state.accf.y=(float)stcAcc.a[1]/2048*9.8f; 
									jy901_state.accf.z=(float)stcAcc.a[2]/2048*9.8f; 
									break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);
									jy901_state.gyrof.x=(float)stcGyro.w[0]/16.384f; 
									jy901_state.gyrof.y=(float)stcGyro.w[1]/16.384f; 
									jy901_state.gyrof.z=(float)stcGyro.w[2]/16.384f; 
									break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);
									jy901_state.angle.roll=(double)stcAngle.Angle[0]/32768*180; 
									jy901_state.angle.pitch=(double)stcAngle.Angle[1]/32768*180; 
									jy901_state.angle.yaw=(double)stcAngle.Angle[2]/32768*180;			
									break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);
									jy901_state.magf.x=(float)stcMag.h[0]; 
									jy901_state.magf.y=(float)stcMag.h[1]; 
									jy901_state.magf.z=(float)stcMag.h[2]; 
									break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//��ջ�����
	}
}

/*******************************��������****************************************
* ��������: void JY901_RB_Init(void)
* �������:
* ���ز���:
* ��    ��: ��ʼ��һ��ѭ�����У�����������յ��Ĵ������ݡ���ʵ����һ�����ݻ�����
* ��    ��: 
* ��    ��: 2015/05/01
*******************************************************************************/
void JY901_RB_Init(void)
{
	//��m_GPS_RX_Buffm_GPS_RX_RingBuffMgr�����н��й�������
	rbInit(&m_JY901_RX_RingBuffMgr, m_JY901_RX_Buff, sizeof(m_JY901_RX_Buff));
}

/*******************************��������****************************************
* ��������: void JY901_Rece_Enable(void)
* �������:
* ���ز���:
* ��    ��: ʹ��DMA����
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
void JY901_Rece_Enable(void)
{
	HAL_UART_Receive_DMA(&huart3, &jy901_rx_temp, 1);
}

/*******************************��������****************************************
* ��������: void JY901_Init(void)
* �������:
* ���ز���:
* ��    ��: ��ʼ��MAVLINK��ʹ�ܽ��գ�ringbuffer����
* ��    ��: by 
* ��    ��: 2018/06/02
*******************************************************************************/
void JY901_Init(void)
{
	JY901_Rece_Enable();
	JY901_RB_Init();
}

/*******************************��������****************************************
* ��������: void JY901_RB_Clear(void)
* �������:
* ���ز���:
* ��    ��: ����ringbuffer��������á�
* ��    ��: 
* ��    ��: 2019/12/07
*******************************************************************************/
void JY901_RB_Clear(void)
{
	rbClear(&m_JY901_RX_RingBuffMgr);
}

/*******************************��������****************************************
* ��������: uint8_t  GPS_RB_IsOverFlow(void)
* �������:
* ���ز���:  ���Ϊ1����֮Ϊ0
* ��    ��: �жϻ������Ƿ����
* ��    ��: 
* ��    ��: 2018/05/07
*******************************************************************************/
uint8_t  JY901_RB_IsOverFlow(void)
{
	return m_JY901_RX_RingBuffMgr.flagOverflow;
}

/*******************************��������****************************************
* ��������: void GPS_RB_Push(uint8_t data)
* �������: data����ѹ�������
* ���ز���:
* ��    ��: �����յ�����ѹ�뻺����
* ��    ��: 
* ��    ��: 2018/05/07
*******************************************************************************/
void JY901_RB_Push(uint8_t data)
{
	rbPush(&m_JY901_RX_RingBuffMgr, (uint8_t)(data & (uint8_t)0xFFU));
}

/*******************************��������****************************************
* ��������: uint8_t GPS_RB_Pop(void)
* �������:
* ���ز���: uint8_t ����������
* ��    ��: �ӻ�������������
* ��    ��: 
* ��    ��: 2018/05/07
*******************************************************************************/
uint8_t JY901_RB_Pop(void)
{
	return rbPop(&m_JY901_RX_RingBuffMgr);
}

/*******************************��������****************************************
* ��������: uint8_t GPS_RB_HasNew(void)
* �������:
* ���ز���:
* ��    ��: �ж��Ƿ����µ�����
* ��    ��: 
* ��    ��: 2018/05/07
*******************************************************************************/
uint8_t JY901_RB_HasNew(void)
{
	return !rbIsEmpty(&m_JY901_RX_RingBuffMgr);
}

/*******************************��������****************************************
* ��������: uint16_t JY901_RB_Count(void)
* �������:
* ���ز���:
* ��    ��: �ж��ж��ٸ�������
* ��    ��: 
* ��    ��: 2018/05/07
*******************************************************************************/
uint16_t JY901_RB_Count(void)
{
	return rbGetCount(&m_JY901_RX_RingBuffMgr);
}






void JY901_Receive(void)
{
	uint8_t read=0;
	//����ִ��һ�θñ���+1
	JY901_RunCount++;

	//Bar_RunCountÿ�仯5�Σ�if���ִ��һ�Σ�����if���100hzִ��
	if(JY901_RunCount % CYCLE_50HZ_FROM_500HZ ==0)
	{	

		if(JY901_RB_IsOverFlow())
		{
			JY901_RB_Clear();
		}

		while(JY901_RB_HasNew())
		{
			read = JY901_RB_Pop();
			JY901_Process(read);						
//			Angle = posture_computer(jy901_state.angle);
		}

	}
	else{}
}
