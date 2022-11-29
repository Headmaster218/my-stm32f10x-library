#include "timer.h"

void TIM1_Int_Init(u16 arr, u16 psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʱ��ʹ��

  TIM_TimeBaseStructure.TIM_Period = arr;                     //�����Զ���װ�ؼĴ�������ֵ
  TIM_TimeBaseStructure.TIM_Prescaler = (psc - 1);            //����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;                //����ʱ�ӷָ�
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;            //�ظ���������
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);             //������ʼ��
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);                       //���жϱ�־λ

  TIM_ITConfig(           //ʹ�ܻ���ʧ��ָ����TIM�ж�
      TIM1,               // TIM1
      TIM_IT_Update |     // TIM �����ж�Դ
          TIM_IT_Trigger, // TIM �����ж�Դ
      ENABLE              //ʹ��
  );

  //�������ȼ�
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�0��
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //�����ȼ�0��
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_Cmd(TIM1, ENABLE); //ʹ��TIMx����
}

void PWM_Init(u16 arr, u16 psc)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);                        //ʹ�ܶ�ʱ��3ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); //ʹ��GPIO�����AFIO���ù���ģ��ʱ��

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��GPIO

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��GPIO

  TIM_TimeBaseStructure.TIM_Period = arr;                     //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
  TIM_TimeBaseStructure.TIM_Prescaler = psc;                  //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;                //����ʱ�ӷָ�:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM���ϼ���ģʽ
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);             //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;             //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;      //�������:TIM����Ƚϼ��Ը�

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_Cmd(TIM4, ENABLE);
}