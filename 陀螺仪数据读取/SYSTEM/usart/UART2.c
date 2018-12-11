#include <stdio.h>
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "misc.h"
#include "UART2.h"

static unsigned char TxBuffer[256];
static unsigned char TxCounter=0;
static unsigned char count=0; 
extern void CopeSerial2Data(unsigned char ucData);

void uart2_init(unsigned long baudrate)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //??GPIOA??
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//??USART2??
	 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2???USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3???USART2  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2?GPIOA3???
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//????
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //??50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //??????
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //??
	GPIO_Init(GPIOA,&GPIO_InitStructure); //???GPIOA2,?GPIOA3
	//4.???????:?????,??,???????
	USART_InitStructure.USART_BaudRate = baudrate;//????????9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//???8?????
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//?????
	USART_InitStructure.USART_Parity = USART_Parity_No;//??????
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//????????
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //????
	USART_Init(USART2, &USART_InitStructure); //?????2??
	//5.???NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//?????0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //????3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ????
	NVIC_Init(&NVIC_InitStructure); //??????????VIC???
	//6.????
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//?
	USART_Cmd(USART2, ENABLE);
}
void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  {   
    USART_SendData(USART2, TxBuffer[TxCounter++]); 
    USART_ClearITPendingBit(USART2, USART_IT_TXE);
    if(TxCounter == count) USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
  }
	else if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
		CopeSerial2Data((unsigned char)USART2->DR);//´¦ÀíÊý¾Ý
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
	
	USART_ClearITPendingBit(USART2,USART_IT_ORE);
}


void UART2_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;  
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  
}

void UART2_Put_String(unsigned char *Str)
{
	while(*Str)
	{
		if(*Str=='\r')UART2_Put_Char(0x0d);
			else if(*Str=='\n')UART2_Put_Char(0x0a);
				else UART2_Put_Char(*Str);
		Str++;
	}
}