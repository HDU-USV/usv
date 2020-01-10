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



/*缓冲区管理器*/
//ringbuffer管理变量
RingBuffer  m_JY901_RX_RingBuffMgr;
/*GPS接收数据缓冲区数组*/
uint8_t   m_JY901_RX_Buff[JY901_READ_BUFFER_SIZE*10];

//接收数据存放变量
uint8_t jy901_rx_temp;

__IO uint32_t JY901_RunCount=0;

void JY901_RB_Clear(void);

void JY901_InterruptHandler()
{
	//数据压入
	rbPush(&m_JY901_RX_RingBuffMgr,jy901_rx_temp);
}

 void JY901_Process(unsigned char ucData)
{
	static uint8_t ucRxBuffer[250];
	static uint8_t ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
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
		ucRxCnt=0;//清空缓存区
	}
}

/*******************************函数声明****************************************
* 函数名称: void JY901_RB_Init(void)
* 输入参数:
* 返回参数:
* 功    能: 初始化一个循环队列，用来管理接收到的串口数据。其实就是一个数据缓冲区
* 作    者: 
* 日    期: 2015/05/01
*******************************************************************************/
void JY901_RB_Init(void)
{
	//将m_GPS_RX_Buffm_GPS_RX_RingBuffMgr环队列进行关联管理。
	rbInit(&m_JY901_RX_RingBuffMgr, m_JY901_RX_Buff, sizeof(m_JY901_RX_Buff));
}

/*******************************函数声明****************************************
* 函数名称: void JY901_Rece_Enable(void)
* 输入参数:
* 返回参数:
* 功    能: 使能DMA接收
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
void JY901_Rece_Enable(void)
{
	HAL_UART_Receive_DMA(&huart3, &jy901_rx_temp, 1);
}

/*******************************函数声明****************************************
* 函数名称: void JY901_Init(void)
* 输入参数:
* 返回参数:
* 功    能: 初始化MAVLINK：使能接收，ringbuffer关联
* 作    者: by 
* 日    期: 2018/06/02
*******************************************************************************/
void JY901_Init(void)
{
	JY901_Rece_Enable();
	JY901_RB_Init();
}

/*******************************函数声明****************************************
* 函数名称: void JY901_RB_Clear(void)
* 输入参数:
* 返回参数:
* 功    能: 归零ringbuffer里面的设置。
* 作    者: 
* 日    期: 2019/12/07
*******************************************************************************/
void JY901_RB_Clear(void)
{
	rbClear(&m_JY901_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: uint8_t  GPS_RB_IsOverFlow(void)
* 输入参数:
* 返回参数:  溢出为1，反之为0
* 功    能: 判断缓冲器是否溢出
* 作    者: 
* 日    期: 2018/05/07
*******************************************************************************/
uint8_t  JY901_RB_IsOverFlow(void)
{
	return m_JY901_RX_RingBuffMgr.flagOverflow;
}

/*******************************函数声明****************************************
* 函数名称: void GPS_RB_Push(uint8_t data)
* 输入参数: data：待压入的数据
* 返回参数:
* 功    能: 将接收的数据压入缓冲区
* 作    者: 
* 日    期: 2018/05/07
*******************************************************************************/
void JY901_RB_Push(uint8_t data)
{
	rbPush(&m_JY901_RX_RingBuffMgr, (uint8_t)(data & (uint8_t)0xFFU));
}

/*******************************函数声明****************************************
* 函数名称: uint8_t GPS_RB_Pop(void)
* 输入参数:
* 返回参数: uint8_t 读出的数据
* 功    能: 从缓冲器读出数据
* 作    者: 
* 日    期: 2018/05/07
*******************************************************************************/
uint8_t JY901_RB_Pop(void)
{
	return rbPop(&m_JY901_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: uint8_t GPS_RB_HasNew(void)
* 输入参数:
* 返回参数:
* 功    能: 判断是否有新的数据
* 作    者: 
* 日    期: 2018/05/07
*******************************************************************************/
uint8_t JY901_RB_HasNew(void)
{
	return !rbIsEmpty(&m_JY901_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: uint16_t JY901_RB_Count(void)
* 输入参数:
* 返回参数:
* 功    能: 判断有多少个新数据
* 作    者: 
* 日    期: 2018/05/07
*******************************************************************************/
uint16_t JY901_RB_Count(void)
{
	return rbGetCount(&m_JY901_RX_RingBuffMgr);
}






void JY901_Receive(void)
{
	uint8_t read=0;
	//函数执行一次该变量+1
	JY901_RunCount++;

	//Bar_RunCount每变化5次，if语句执行一次，即该if语句100hz执行
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
