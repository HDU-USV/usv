#include "sys.h"
#include "delay.h"  
#include "usart.h"  
#include "led.h"
#include "lcd.h"
#include "usmart.h"	
#include "usart3.h" 	
#include "key.h" 	 
#include "string.h"	 	 
#include "gps.h"	
#include "4g.h"

//ALIENTEK 探索者STM32F407开发板 扩展实验2
//ATK-NEO-6M GPS模块测试实验 -库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com  
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK

				  	 
u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//串口1,发送缓存区
nmea_msg gpsx; 											//GPS信息
__align(4) u8 dtbuf[50];   								//打印缓存器
const u8*fixmode_tbl[4]={"Fail","Fail"," 2D "," 3D "};	//fix mode字符串 
 
//显示GPS定位信息 
void Gps_Msg_Show(void)
{
 	float tp;		   
	POINT_COLOR=BLUE;  	 
	tp=gpsx.longitude;	   
	sprintf((char *)dtbuf,"Longitude:%.5f %1c   ",tp/=100000,gpsx.ewhemi);	//得到经度字符串
 	LCD_ShowString(30,130,200,16,16,dtbuf);	 	   
	tp=gpsx.latitude;	   
	sprintf((char *)dtbuf,"Latitude:%.5f %1c   ",tp/=100000,gpsx.nshemi);	//得到纬度字符串
 	LCD_ShowString(30,150,200,16,16,dtbuf);	 	 
	tp=gpsx.altitude;	   
 	sprintf((char *)dtbuf,"Altitude:%.1fm     ",tp/=10);	    			//得到高度字符串
 	LCD_ShowString(30,170,200,16,16,dtbuf);	 			   
	tp=gpsx.speed;	   
 	sprintf((char *)dtbuf,"Speed:%.3fkm/h     ",tp/=1000);		    		//得到速度字符串	 
 	LCD_ShowString(30,190,200,16,16,dtbuf);	 				    
	if(gpsx.fixmode<=3)														//定位状态
	{  
		sprintf((char *)dtbuf,"Fix Mode:%s",fixmode_tbl[gpsx.fixmode]);	
	  	LCD_ShowString(30,210,200,16,16,dtbuf);			   
	}	 	   
	sprintf((char *)dtbuf,"Valid satellite:%02d",gpsx.posslnum);	 		//用于定位的卫星数
 	LCD_ShowString(30,230,200,16,16,dtbuf);	    
	sprintf((char *)dtbuf,"Visible satellite:%02d",gpsx.svnum%100);	 		//可见卫星数
 	LCD_ShowString(30,250,200,16,16,dtbuf);		 
	sprintf((char *)dtbuf,"UTC Date:%04d/%02d/%02d   ",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date);	//显示UTC日期
	//printf("year2:%d\r\n",gpsx.utc.year);
	LCD_ShowString(30,270,200,16,16,dtbuf);		    
	sprintf((char *)dtbuf,"UTC Time:%02d:%02d:%02d   ",gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);	//显示UTC时间
  	LCD_ShowString(30,290,200,16,16,dtbuf);		  
}	 


int main(void)
{ 
	u16 i,rxlen;
	u16 lenx;
	u8 key=0XFF;
	u8 upload=0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);      	//初始化延时函数
	uart_init(115200);			//初始化串口波特率为115200 
	usart3_init(115200);			//初始化串口3波特率为38400
	delay_ms(2000);
	init_4g();
	 
	while(1) 
	{		
//		delay_ms(1);
//		if(USART3_RX_STA&0X8000)		//接收到一次数据了
//		{
//			rxlen=USART3_RX_STA&0X7FFF;	//得到数据长度
//			for(i=0;i<rxlen;i++)USART1_TX_BUF[i]=USART3_RX_BUF[i];	   
// 			USART3_RX_STA=0;		   	//启动下一次接收
//			USART1_TX_BUF[i]=0;			//自动添加结束符
//			GPS_Analysis(&gpsx,(u8*)USART1_TX_BUF);//分析字符串
//			//Gps_Msg_Show();				//显示信息	
//			if(upload)printf("\r\n%s\r\n",USART1_TX_BUF);//发送接收到的数据到串口1
// 		}
//		key=KEY_Scan(0);
//		if(key==KEY0_PRES)
//		{
//			upload=!upload;
//			POINT_COLOR=RED;
//			if(upload)LCD_ShowString(30,100,200,16,16,"NMEA Data Upload:ON ");
//			else LCD_ShowString(30,100,200,16,16,"NMEA Data Upload:OFF");
// 		}
//		if((lenx%500)==0)LED0=!LED0; 	    				 
//		lenx++;	
	}									    
}

