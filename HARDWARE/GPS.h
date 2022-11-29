#ifndef __GPS_H
#define __GPS_H

#include "sys.h"
#include <delay.h>
//#include <usart.h>

void GPS_DMA_Init(void);

//RMC_DATA�ṹ��

struct GGA_DATA
{
	u8 time[3]; //h,m,s
	char ng[13],lat[13];  //��γ��
	float lon_f, lat_f;
	u8 num;	//��������
	short height;
	short speed;
};



#endif
