#ifndef __UART2_H
#define __UART2_H

void uart2_init(unsigned long baudrate);
void UART2_Put_Char(unsigned char DataToSend);
void UART2_Put_String(unsigned char *Str);
#endif

//------------------End of File----------------------------

