/*******************************************************************************
* �ļ����ƣ�example.h
*
* ժ    Ҫ�����Գ���
*
* ��ǰ�汾��
* ��    �ߣ�ACROSS
* ��    �ڣ�2018/05/24
* ���뻷����keil5
*
* ��ʷ��Ϣ��
*******************************************************************************/
#ifndef __EXAMPLE_H
#define __EXAMPLE_H

#include "stm32f4xx_hal.h"
#include "main.h"

typedef enum {
	RUN = 0,
	LIMIT,
	STOP
} decode_move;

void Example_Exchage_COM(void);
#endif
