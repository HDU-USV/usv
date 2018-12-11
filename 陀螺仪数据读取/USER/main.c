#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "UART2.h"
#include <string.h>
#include <stdio.h>
#include "JY901.h"


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

//ALIENTEK 探索者STM32F407开发板 实验4
//串口通信实验 -库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK
//
void acc_calibration()
{
	int cnt = 0;

	int acc_x=0, acc_y=0, acc_z=0;
	
	char send_data[5];
	int i = 0;
	//1. send 3 order
	send_data[0] = 0xFF;
	send_data[1] = 0xAA;
	send_data[2] = 0x05;
	send_data[3] = 0x00;
	send_data[4] = 0x00;
	for(i=0;i<5;i++)
	{
		UART2_Put_Char(send_data[i]);
	}
	delay_ms(100);
	
	send_data[0] = 0xFF;
	send_data[1] = 0xAA;
	send_data[2] = 0x06;
	send_data[3] = 0x00;
	send_data[4] = 0x00;
	for(i=0;i<5;i++)
	{
		UART2_Put_Char(send_data[i]);
	}
	delay_ms(100);
	
	send_data[0] = 0xFF;
	send_data[1] = 0xAA;
	send_data[2] = 0x07;
	send_data[3] = 0x00;
	send_data[4] = 0x00;
	for(i=0;i<5;i++)
	{
		UART2_Put_Char(send_data[i]);
	}
	delay_ms(100);
	
	//2. read data 
	for(cnt = 0; cnt < 100; cnt++)
	{
		delay_ms(30);
		acc_x += stcAcc.a[0];
		acc_y += stcAcc.a[1];
		acc_z += stcAcc.a[2];
		
	}
	acc_x = acc_x/100;
	acc_y = acc_y/100;
	acc_z = acc_z/100;
	acc_z = acc_z-2048;
	

	//3. write e
	send_data[0] = 0xFF;
	send_data[1] = 0xAA;
	send_data[2] = 0x05;
	send_data[3] = acc_x&0xFF;
	send_data[4] = (acc_x&0xFF00) >> 8;
	for(i=0;i<5;i++)
	{
		UART2_Put_Char(send_data[i]);
	}
	delay_ms(100);
	
	       //calibration y
	send_data[0] = 0xFF;
	send_data[1] = 0xAA;
	send_data[2] = 0x06;
	send_data[3] = acc_y & 0xFF;
	send_data[4] = (acc_y & 0xFF00) >> 8;
	for(i=0;i<5;i++)
	{
		UART2_Put_Char(send_data[i]);
	}
	delay_ms(100);
	
	send_data[0] = 0xFF;
	send_data[1] = 0xAA;
	send_data[2] = 0x07;
	send_data[3] = acc_z & 0xFF;
	send_data[4] = (acc_z & 0xFF00) >> 8;
	for(i=0;i<5;i++)
	{
		UART2_Put_Char(send_data[i]);
	}
	delay_ms(100);
	
	//4. save configuration!
	
	send_data[0] = 0xFF;
	send_data[1] = 0xAA;
	send_data[2] = 0x00;
	send_data[3] = 0x00;
	send_data[4] = 0x00;
	for(i=0;i<5;i++)
	{
		UART2_Put_Char(send_data[i]);
	}
}

void UART1_Put_String(unsigned char *Str)
{
	while(*Str)
	{
		if(*Str=='\r')USART_SendData(USART1, 0x0d);
			else if(*Str=='\n')USART_SendData(USART1, 0x0a);
				else USART_SendData(USART1, *Str);
		Str++;
	}
}

void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
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
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//清空缓存区
	}
}

int main(void)
{ 
 
	char str[100];

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	uart_init(115200);	//串口初始化波特率为115200
	uart2_init(9600);
	delay_ms(1000);//等等JY-91初始化完成
	acc_calibration();

	while(1)
	{
		
		delay_ms(500);
		//输出时间
		sprintf(str,"Time:20%d-%d-%d %d:%d:%.3f\r\n",stcTime.ucYear,stcTime.ucMonth,stcTime.ucDay,stcTime.ucHour,stcTime.ucMinute,(float)stcTime.ucSecond+(float)stcTime.usMiliSecond/1000);
		//UART1_Put_String(str);
		printf("%s", str);
		delay_ms(10);
		//输出加速度
		sprintf(str,"Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
		printf("%s", str);
		delay_ms(10);
		//输出角速度
		sprintf(str,"Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
		printf("%s", str);
		delay_ms(10);
		//输出角度
		sprintf(str,"Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
		printf("%s", str);
		delay_ms(10);
		//输出磁场
		sprintf(str,"Mag:%d %d %d\r\n",stcMag.h[0],stcMag.h[1],stcMag.h[2]);
		printf("%s", str);
		
		sprintf(str,"Acc_Original:%d %d %d\r\n",stcAcc.a[0],stcAcc.a[1],stcAcc.a[2]);
		printf("%s", str);
		delay_ms(10);
		//输出气压、高度
//		sprintf(str,"Pressure:%ld Height%.2f\r\n",stcPress.lPressure,(float)stcPress.lAltitude/100);
//		printf_char(str); 
//		delay_ms(10);
//		//输出端口状态
//		sprintf(str,"DStatus:%d %d %d %d\r\n",stcDStatus.sDStatus[0],stcDStatus.sDStatus[1],stcDStatus.sDStatus[2],stcDStatus.sDStatus[3]);
//		printf_char(str);
//		delay_ms(10);
//		//输出经纬度
//		sprintf(str,"Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",stcLonLat.lLon/10000000,(double)(stcLonLat.lLon % 10000000)/1e5,stcLonLat.lLat/10000000,(double)(stcLonLat.lLat % 10000000)/1e5);
//		printf_char(str);
//		delay_ms(10);
//		//输出地速
//		sprintf(str,"GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n",(float)stcGPSV.sGPSHeight/10,(float)stcGPSV.sGPSYaw/10,(float)stcGPSV.lGPSVelocity/1000);
//		printf_char(str);
//		delay_ms(10);
//		//输出四元素
//		sprintf(str,"Four elements:%.5f %.5f %.5f %.5f\r\n\r\n",(float)stcQ.q[0]/32768,(float)stcQ.q[1]/32768,(float)stcQ.q[2]/32768,(float)stcQ.q[3]/32768);
//		printf_char(str);
//		delay_ms(10);//等待传输完成

}
}

