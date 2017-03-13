#include "timer.h"
#include "led.h"

//��ʱ��3�жϷ������	 

//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3ʱ��ʹ��    
 	TIM3->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM3->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
	TIM3->DIER|=1<<0;   //��������ж�	  
	TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3
  	MY_NVIC_Init(1,3,TIM3_IRQChannel,2);//��ռ1�������ȼ�3����2									 
}
//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM2_PWM_Init(u16 arr,u16 psc)
{		 					 
	//�˲������ֶ��޸�IO������
	RCC->APB1ENR|=1; 		//TIM2ʱ��ʹ��    
	RCC->APB2ENR|=1<<2;    	//ʹ��PORTAʱ��	
	GPIOA->CRL&=0XFFFF0000;	//PA���
	GPIOA->CRL|=0X0000BBBB;	//���ù������ 	  	 
	   
	RCC->APB2ENR|=1<<0;     //��������ʱ��	   
	AFIO->MAPR&=0XFFFFFCFF; //���MAPR��[11:10]

	TIM2->ARR=arr;			//�趨�������Զ���װֵ 
	TIM2->PSC=psc;			//Ԥ��Ƶ������Ƶ
	
	TIM2->CCMR1|=6<<4;  	//CH2 PWM2ģʽ
	TIM2->CCMR1|=1<<3;      //CH2Ԥװ��ʹ��	 
		
	TIM2->CCMR1|=6<<12;  	//CH2 PWM2ģʽ		 
	TIM2->CCMR1|=1<<11; 	//CH2Ԥװ��ʹ��

	TIM2->CCMR2|=6<<4;  	//CH2 PWM2ģʽ		 
	TIM2->CCMR2|=1<<3; 	    //CH2Ԥװ��ʹ��
	TIM2->CCMR2|=6<<12;  	//CH2 PWM2ģʽ		 
	TIM2->CCMR2|=1<<11; 	//CH2Ԥװ��ʹ��
		   
	TIM2->CCER|=1<<0;   	//OC2 ���ʹ��
	TIM2->CCER|=1<<4;   	//OC2 ���ʹ��
	TIM2->CCER|=1<<8;   	//OC2 ���ʹ��
	TIM2->CCER|=1<<12;   	//OC2 ���ʹ��
		   
	TIM2->CR1=0x8000;   	//ARPEʹ�� 
	TIM2->CR1|=0x01;    	//ʹ�ܶ�ʱ��5 

	PWM_CH1=0;
	PWM_CH2=0;
	PWM_CH3=0;
	PWM_CH4=0;	
									  
} 
void TIM5_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<3;	//TIM5ʱ��ʹ��    
 	TIM5->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM5->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
	TIM5->DIER|=1<<0;   //��������ж�	  
	TIM5->CR1|=0x01;    //ʹ�ܶ�ʱ��3
  	MY_NVIC_Init(1,3,TIM5_IRQChannel,2);//��ռ1�������ȼ�3����2									 
}
 	 













