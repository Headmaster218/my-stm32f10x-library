#include <091OLED.h>
#include <OLEDFONT.h>


void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
	if(cmd) I2C1_Soft_Single_Write(0x3c, 0x40, dat);
	else I2C1_Soft_Single_Write(0x3c, 0x00, dat);
}


/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
	unsigned char m;
	for(m=0;m<8;m++)
	{
		OLED_WR_Byte(0xb0+m,0);	//page0-page1
		OLED_WR_Byte(0x00,0);		//low column start address
		OLED_WR_Byte(0x10,0);		//high column start address
		for(m=0;m<128;m++) OLED_WR_Byte(fill_Data,1);
	}
}

//��������

void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f),OLED_CMD); 
}   	  
//����OLED��ʾ    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
void OLED_Clear(void)  
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //������ʾ
}
void OLED_On(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(1,OLED_DATA); 
	} //������ʾ
}
//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 16/8 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size)
{      	
	unsigned char c=0,i=0;	
		c=chr-' ';//�õ�ƫ�ƺ��ֵ			
		if(x>Max_Column-1){x=0;y=y+2;}
		if(Char_Size ==16)
			{
			OLED_Set_Pos(x,y);	
			for(i=0;i<8;i++) OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++) OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
			}
			else 
			{	
				OLED_Set_Pos(x,y);
				for(i=0;i<6;i++)
				OLED_WR_Byte(F6x8[c][i],OLED_DATA);
			}
}
//m^n����
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  


//��ʾС��
// x,y :�������
// ilen :������λ��
// flen :С����λ��
// size:�����С16/12
void OLED_ShowFloat(u8 x, u8 y, float num, u8 ilen, u8 flen, u8 size2)
{
	OLED_ShowChar(x + size2 / 2 * ilen, y, '.', size2);
	OLED_ShowNum(x, y, num, ilen, size2);
	ilen = flen;
	while (ilen--)
		num *= 10;
	OLED_ShowNum(x + size2 / 2 * flen + 6, y, num, flen, size2);
}

//��ʾ����
// x,y :�������
// len :���ֵ�λ��
// size:�����С16/12
// mode:ģʽ	0,���ģʽ;1,����ģʽ
// num:��ֵ(-32767~32766);
void OLED_ShowNum(u8 x, u8 y, int num, u8 len, u8 size2)
{
	u8 t, temp;
	u8 enshow = 0;
	if (num < 0)
	{
		num = -num;
		OLED_ShowChar(x, y, '-', size2);
		x += size2 / 2;
	}
	for (t = 0; t < len; t++)
	{
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				OLED_ShowChar(x + (size2 / 2) * t, y, ' ', size2);
				continue;
			}
			else
				enshow = 1;
		}
		OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0', size2);
	}
}
//��ʾһ���ַ��Ŵ�
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 Char_Size)
{
	unsigned char j = 0;
	while (chr[j] != '\0')
	{
		OLED_ShowChar(x, y, chr[j], Char_Size);
		x += Char_Size == 16 ? 8 : 6;
		if (x > 120)
		{
			x = 0;
			y += Char_Size == 16 ? 2 : 1;
		}
		j++;
	}
}

/***********������������ʼ������(x,y),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7*****************/
void OLED_DrawBMP(unsigned char pos_x, unsigned char pos_y, unsigned char pic_x, unsigned char pic_y, unsigned char BMP[])
{
	unsigned int j = 0;
	unsigned char x, y;
	pic_y/=8;
	for (y = pos_y; y < pos_y+pic_y; y++)
	{
		OLED_Set_Pos(pos_x, y);
		for (x = pos_x; x < pos_x+pic_x; x++)
		{
			OLED_WR_Byte(BMP[j++], OLED_DATA);
		}
	}
}

//dir:L,R
//start:0-7
//len:0-7
//end:0-7
//interval:0-7
void Start_Horizontal_Scroll(u8 dir, u8 start, u8 end, u8 interval)
{
	Scroll_Cmd(0);
	if (dir == 'R')
		OLED_WR_Byte(0x26, OLED_CMD);
	else
		OLED_WR_Byte(0x27, OLED_CMD);

	OLED_WR_Byte(0x00, OLED_CMD);
	OLED_WR_Byte(start, OLED_CMD);
	OLED_WR_Byte(interval, OLED_CMD);
	OLED_WR_Byte(end, OLED_CMD);
	OLED_WR_Byte(0x00, OLED_CMD);
	OLED_WR_Byte(0xFF, OLED_CMD);
	Scroll_Cmd(1);
}

//��ʼ��SSD1306					    
void OLED_Init(void)
{ 	
	OLED_WR_Byte(0xAE | 0, OLED_CMD); // Set Display OFF

	OLED_WR_Byte(0x81, OLED_CMD); //���öԱȶ�
	OLED_WR_Byte(0x2, OLED_CMD);
	
	OLED_WR_Byte(0xa6 | 0, OLED_CMD); // Set Normal/Inverse Display
	
	OLED_WR_Byte(0xB0, OLED_CMD); //Set Page Start Address for Page Addressing Mode
	OLED_WR_Byte(0x40, OLED_CMD); //Set Display Start Line
	OLED_WR_Byte(0xa0|0, OLED_CMD); //��Ļ����
	
	OLED_WR_Byte(0xC0, OLED_CMD); //C0/C8Set COM Output Scan Direction
	
	OLED_WR_Byte(0xd3, OLED_CMD);//Set Display Offset
	OLED_WR_Byte(0x00, OLED_CMD);
	
	OLED_WR_Byte(0xda, OLED_CMD);//Set COM Pins Hardware Configuration 
	OLED_WR_Byte(0x12, OLED_CMD);//128*64
	OLED_WR_Byte(0x02, OLED_CMD);//128*32

	OLED_WR_Byte(0xa8, OLED_CMD); //��������·��
	OLED_WR_Byte(0x1f, OLED_CMD);

	//Set Display Clock Divide Ratio/Oscillator Frequency 
	OLED_WR_Byte(0xd5, OLED_CMD);
	OLED_WR_Byte(0xf0, OLED_CMD);

	OLED_WR_Byte(0xd9, OLED_CMD);//Set Pre-charge Period
	OLED_WR_Byte(0x22, OLED_CMD);


	OLED_WR_Byte(0xdb, OLED_CMD);//Set VCOMH Deselect Level 
	OLED_WR_Byte(0x49, OLED_CMD);

	OLED_WR_Byte(0x8d, OLED_CMD);//8Dh ; Charge Pump Setting
	OLED_WR_Byte(0x14, OLED_CMD);//14h ; Enable Charge Pump
	OLED_WR_Byte(0xaE | 1, OLED_CMD); //AFh; Display ON
	OLED_Clear();
} 
