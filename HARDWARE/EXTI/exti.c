#include "exti.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "24l01.h"
extern u8 tmp_buf[16];

//�ⲿ�ж�0�������
void EXTI0_IRQHandler(void)
{	 
	EXTI->PR=1<<0;  //���LINE0�ϵ��жϱ�־λ  
}
//�ⲿ�ж�2�������
void EXTI2_IRQHandler(void)
{
	EXTI->PR=1<<2;  //���LINE2�ϵ��жϱ�־λ  
}
//�ⲿ�ж�3�������
void EXTI3_IRQHandler(void)
{	

	EXTI->PR=1<<3;  //���LINE3�ϵ��жϱ�־λ  
}
//�ⲿ�ж�4�������
void EXTI4_IRQHandler(void)
{	 
//	if(NRF24L01_RxPacket(tmp_buf)==0)//һ�����յ���Ϣ,����ʾ����.
//	{
//	} 	 
	EXTI->PR=1<<4;  //���LINE4�ϵ��жϱ�־λ  
}		   
void EXTIX_Init(void)
{
	Ex_NVIC_Config(GPIO_G,4,FTIR); 			//�½��ش���

	MY_NVIC_Init(2,2,EXTI4_IRQChannel,2);	//��ռ2�������ȼ�2����2	   
}












