#include "stm32f10x.h"
#include "motor.h"
#include "delay.h"

struct _motor motor_A ={0}, motor_B={0}, motor_C={0}, motor_D={0};

double Kp = 0.15,Ki,Kd = 1;
void Adder_PID()
{
	motor_A.output_spd += LIM(motor_A.target_spd - motor_A.now_spd,-1000,1000) * Kp;
	motor_A.output_spd -= LIM(ABS((motor_A.now_spd - motor_A.last_spd) * Kd) , -500, 500);
	motor_A.output_spd =  LIM(motor_A.output_spd,-1000,1000);

	motor_B.output_spd += LIM(motor_B.target_spd - motor_B.now_spd,-1000,1000) * Kp;
	motor_B.output_spd -= LIM(ABS((motor_B.now_spd - motor_B.last_spd) * Kd) , -500, 500);
	motor_B.output_spd =  LIM(motor_B.output_spd,-1000,1000);
	
	Motor_Set_Output(motor_A.output_spd, 'A');
	Motor_Set_Output(motor_B.output_spd, 'B');

	motor_B.last_spd = motor_B.now_spd;
	motor_A.last_spd = motor_A.now_spd;
}
// �Գ���ִ����ת������
//����angleΪ��ת�ĽǶȣ���λ�ȣ�����ʱ��Ϊ����˳ʱ��Ϊ��
void Car_Rotate(float angle)
{
	angle = angle * ANGLE_TO_CAP;
	Car_Set_Move(angle, -angle, angle, -angle);
}
//�Գ���ִ��ƽ�Ʋ���������distanceΪƽ�Ƶľ��루��λ���ף���
//����dirΪƽ�Ƶķ����Ϊǰ���������ַ���
void Car_Move_Dir(short distance, u8 dir)
{
	switch (dir)
	{
	case 'F':
		Car_Set_Move(distance, distance, distance, distance);
		break;
	case 'B':
		Car_Set_Move(-distance, -distance, -distance, -distance);
		break;
	case 'R':
		distance*=1.414;
		Car_Set_Move(distance, -distance, -distance, distance);
		break;
	case 'L':
		distance*=1.414;
		Car_Set_Move(-distance, distance, distance, -distance);
		break;
	}
}

//Ϊ�ĸ���������һ������Ŀ��ֵ������ABCD�ֱ��Ӧ�ĸ����ӵ��ƶ����롣��mm��
void Car_Set_Move(short A, short B, short C, short D)
{       
	motor_A.start_distance	= motor_A.distance;                                                                                  
	motor_A.distance_target	+= A * MM_TO_CAP;
	motor_A.target_movement	= ABS(motor_A.distance_target - motor_A.start_distance);
	
	motor_B.start_distance	= motor_B.distance;
	motor_B.distance_target	+= B * MM_TO_CAP;
	motor_B.target_movement	= ABS(motor_B.distance_target - motor_B.start_distance);

	motor_C.start_distance	= motor_C.distance;
	motor_C.distance_target	+= C * MM_TO_CAP;
	motor_C.target_movement	= ABS(motor_C.distance_target - motor_C.start_distance);

	motor_D.start_distance	= motor_D.distance;
	motor_D.distance_target	+= D * MM_TO_CAP;
	motor_D.target_movement	= ABS(motor_D.distance_target - motor_D.start_distance);
}

//�ú�����Ҫ���ڵ��ã�������������������ƶ���
double ave_complete_percent = 0;
void Car_Move()
{
	ave_complete_percent = (motor_A.complete_percent + motor_B.complete_percent + motor_C.complete_percent + motor_D.complete_percent) / 4;
	Wheel_Move(&motor_A);
	Wheel_Move(&motor_B);
	Wheel_Move(&motor_C);
	Wheel_Move(&motor_D);

}
//����ͣ�汾
//������Ƶ������ӵ����ֵ������*motorxΪĳ�����ӵĽṹ�塣
void Wheel_Move(struct _motor *motorx)
{
	if(motorx->distance_target - motorx->distance > 0)
		motorx->dir = 1;
	else
		motorx->dir = -1;
	
	if(motorx->target_movement)
	motorx->complete_percent = 1-(double)(motorx->dir * (motorx->distance_target - motorx->distance)) / (double)motorx->target_movement;

	if(ABS(motorx->distance_target - motorx->distance) < 3)//С���ͣ,2mm
	{
		motorx->output_spd = motorx->dir * -50;
	}
	else if(ABS(motorx->start_distance - motorx->distance) < SOFT_START_DIS)//��������
	{
		motorx->output_spd += motorx->dir*20;//1s����
		motorx->output_spd = LIM(motorx->output_spd+motorx->dir*250, -1000, 1000);
	}
	else if(ABS(motorx->distance_target - motorx->distance) < 3)//С���ͣ,2mm
	{
		motorx->output_spd = motorx->dir * -50;
	}
	else if(ABS(motorx->distance_target - motorx->distance) < 100)//����������
	{
		motorx->output_spd = motorx->dir*400;
	}
	else if(ABS(motorx->distance_target - motorx->distance) < 300)//���ٹ���
	{
		motorx->output_spd = motorx->dir*(300+(ABS(motorx->distance_target - motorx->distance)/10)*(ABS(motorx->distance_target - motorx->distance)/10));
	}
	else//������ʻ����
	{
		motorx->output_spd = motorx->dir*1000;
	}

	motorx->output_spd *= 1 + ave_complete_percent - motorx->complete_percent;
	motorx->output_spd = LIM(motorx->output_spd, -1000, 1000);
	Motor_Set_Output(motorx->output_spd, motorx->num);
}

union _u16_bit cap_State, gpio_State;
unsigned long cap_time = 0;
// Motor_Cap_IRQHandler
void TIM2_IRQHandler(void)
{	
	cap_State.data = TIM2->SR;
	gpio_State.data = GPIOB->IDR;

	if(cap_State.bit.ch0)//���������
	{
		TIM2->SR &= !TIM_IT_Update;
		cap_time+=50000;
	}

	if (cap_State.bit.ch1)
	{					   // ch1�ж�
		//motor_A.last_time = motor_A.now_time;
		motor_A.now_time = TIM2->CCR1+cap_time;
		if(!gpio_State.bit.ch12)
		{
			motor_A.distance ++;
		}
		else
		{
			motor_A.distance --;
		}
	}
	if (cap_State.bit.ch2)
	{					   // ch2�ж�
		//motor_B.last_time = motor_B.now_time;
		motor_B.now_time = TIM2->CCR2+cap_time;
		if(!gpio_State.bit.ch13)
		{
			motor_B.distance --;
		}
		else
		{
			motor_B.distance ++;
		}
	}
	if(cap_State.bit.ch3)
	{					   // ch3�ж�
		//motor_C.last_time = motor_C.now_time;
		motor_C.now_time = TIM2->CCR3+cap_time;
		if(gpio_State.bit.ch14)
		{
			motor_C.distance --;
		}
		else
		{
			motor_C.distance ++;
		}
	}
	if(cap_State.bit.ch4)
	{					   // ch4�ж�
		//motor_D.last_time = motor_D.now_time;
		motor_D.now_time = TIM2->CCR4+cap_time;
		if(!gpio_State.bit.ch15)
		{
			motor_D.distance --;
		}
		else
		{
			motor_D.distance ++;
		}
	}
}

//-1000 < spd < 1000
// wheel == A/B/C/D
void Motor_Set_Output(short spd, char wheel)
{
	if (wheel == 'A')
	{
		if (spd == 0)
		{
			TIM4->CCR1 = 0;
		}
		else if (spd > 0)
		{
			GPIOB->BSRR = GPIO_Pin_11;
			GPIOB->BRR = GPIO_Pin_10;
			TIM4->CCR1 = spd;
		}
		else
		{
			GPIOB->BRR = GPIO_Pin_11;
			GPIOB->BSRR = GPIO_Pin_10;
			TIM4->CCR1 = -spd;
		}
	}
	else if (wheel == 'B')
    {
		if (spd == 0)
        {
			TIM4->CCR2 = 0;
		}
		else if (spd > 0)
        {
			GPIOA->BSRR = GPIO_Pin_4;
			GPIOA->BRR = GPIO_Pin_5;
			TIM4->CCR2 = spd;
		}
		else
        {
			GPIOA->BRR = GPIO_Pin_4;
			GPIOA->BSRR = GPIO_Pin_5;
			TIM4->CCR2 = -spd;
		}
	}
	else if (wheel == 'C')
	{
		if (spd == 0)
		{
			TIM4->CCR3 = 0;
		}
		else if (spd > 0)
		{
			GPIOC->BSRR = GPIO_Pin_15;
			GPIOC->BRR = GPIO_Pin_14;
			TIM4->CCR3 = spd;
		}
		else
		{
			GPIOC->BRR = GPIO_Pin_15;
			GPIOC->BSRR = GPIO_Pin_14;
			TIM4->CCR3 = -spd;
		}
	}
	else if (wheel == 'D')
	{
		if (spd == 0)
		{
			TIM4->CCR4 = 0;
		}
		else if (spd > 0)
		{
			GPIOA->BSRR = GPIO_Pin_6;
			GPIOA->BRR = GPIO_Pin_7;
			TIM4->CCR4 = spd;
		}
		else
		{
			GPIOA->BRR = GPIO_Pin_6;
			GPIOA->BSRR = GPIO_Pin_7;
			TIM4->CCR4 = -spd;
		}
	}
}

void Motor_Init()
{
	Motor_Direction_Init();
	Motor_PWM_Init(1000, 1);		// 1000�ֶ�
	Motor_Cap_Init(50000, 1440 - 1); // 72,000,000 / 1,440 = 50000Hz����Ƶ��	һ���1s == 50,000
	motor_A.num = 'A';
	motor_B.num = 'B';
	motor_C.num = 'C';
	motor_D.num = 'D';
}
/******************************����ΪӲ����ʼ��*********************************/
void Motor_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM2_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  //ʹ��TIM2ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��GPIOAʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; // PA0 ���֮ǰ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;									 // PA0 ����
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIOA->BRR = 0x000F;

	//��ʼ��TIM2
	TIM_TimeBaseStructure.TIM_Period = arr;						//�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);				//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	//��ʼ��TIM2���벶�����
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1;				 // CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //�����ز���
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 //���������Ƶ,����Ƶ
	TIM2_ICInitStructure.TIM_ICFilter = 0x00;						 // IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);

	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2;				 // CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //�����ز���
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 //���������Ƶ,����Ƶ
	TIM2_ICInitStructure.TIM_ICFilter = 0x00;						 // IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);

	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3;				 // CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //�����ز���
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 //���������Ƶ,����Ƶ
	TIM2_ICInitStructure.TIM_ICFilter = 0x00;						 // IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);

	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4;				 // CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //�����ز���
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 //���������Ƶ,����Ƶ
	TIM2_ICInitStructure.TIM_ICFilter = 0x00;						 // IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);

	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;			  // TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);							  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1 |TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE); //��������ж� ,����CC1IE�����ж�

	TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��5
}

//��ʼ��PWM
//ʹ��TIM4 CH1 CH2 CH3 CH4
void Motor_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);						//ʹ�ܶ�ʱ��3ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); //ʹ��GPIO�����AFIO���ù���ģ��ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		   //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��GPIO

	TIM_TimeBaseStructure.TIM_Period = arr;						//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;				//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);				//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;			  //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	  //�������:TIM����Ƚϼ��Ը�

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);	
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_Cmd(TIM4, ENABLE);
}

void Motor_Direction_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC , ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIOA->BRR = 0xF0;
	GPIOB->BRR = 0xC00;
	GPIOC->BRR = 0xC000;
}
