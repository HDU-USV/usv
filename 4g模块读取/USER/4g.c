#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stm32f4xx.h" 
#include "usart3.h"
#include "delay.h"
u8* lte4g_check_cmd(u8* str)
{
	char *strx=0;
	if(USART3_RX_STA&0X8000)		//接收到一次数据了
	{ 
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//添加结束符
		//printf("%s", USART3_RX_BUF);
		strx=strstr((const char*)USART3_RX_BUF,(const char*)str);
	} 
	return (u8*)strx;
}
u8 lte4g_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART3_RX_STA=0;
	if((u32)cmd<=0XFF)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//
		USART3->DR=(u32)cmd;
	}else u3_printf("%s\r\n",cmd);//发送命令
	if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			delay_ms(10);
			if(USART3_RX_STA&0X8000)//接收到期待的应答结果
			{
				if(lte4g_check_cmd(ack))break;//得到有效数据 
				USART3_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
}

void init_4g()
{
	while(lte4g_send_cmd("AT+WKMOD=NET\r\n", "AT+WKMOD=NET", 20))
	{
		printf("please wait ....");
	}
	printf("success set transimission!!!");
	
	while(lte4g_send_cmd("AT+SOCKAEN=ON\r\n", "AT+SOCKAEN=ON", 20))
	{
		printf("please wait ....");
	}
	printf("Socket A Enable ....");
	
	while(lte4g_send_cmd("AT+SOCKA=TCP,test.usr.cn,2317\r\n", "AT+SOCKA=TCP,test.usr.cn,2317", 20))
	{
		printf("please wait ....");
	}    
	printf("sever set success ....");
	
	while(lte4g_send_cmd("AT+SOCKASL=LONG\r\n", "AT+SOCKASL=LONG", 20))
	{
		printf("please wait ....");
	}
	printf("sever Long set success ....");
	
	while(lte4g_send_cmd("AT+Z", "AT+Z", 1000))
	{
	  delay_ms(3000);
		printf("please wait ....");
	}
	printf("reboot success ....");
	
	while(lte4g_send_cmd("AT+ENTM", "AT+ENTM", 1000))
	{
	  delay_ms(3000);
		printf("please wait ....");
	}
	printf("enter commiunication ....");

}