#include "led.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
void LED_Init(void)
{	   	 

	RCC->APB2ENR|=1<<3;    //ʹ��PORTBʱ��	
	   	 
	GPIOB->CRH&=0XF0FFFFFF;
	GPIOB->CRH|=0X03000000; 	//PB14�������
	GPIOB->CRH&=0X0FFFFFFF;
	GPIOB->CRH|=0X30000000; 	//PA15�������

	LED_RED_OFF;
	LED_GRE_OFF;
}

void Show_str_leds()
{	  
	int ax=0;
	for(ax=0;ax<3;ax++)
	{
		LED_RED_ON;delay_ms(100);
		LED_GRE_ON;delay_ms(100);
		LED_RED_OFF;LED_GRE_OFF;
		delay_ms(100);
	}
	LED_RED_ON;	delay_ms(100);LED_RED_OFF;delay_ms(40);
	LED_RED_ON;	delay_ms(100);LED_RED_OFF;delay_ms(40);	
}

void ALL_LEDS_ON()
{
	LED_RED_ON;
	LED_GRE_ON;
}
void ALL_LEDS_OFF()
{
	LED_RED_OFF;
	LED_GRE_OFF;
}

void LED_twinkle()
{
	LED_RED_ON;	delay_ms(100);LED_RED_OFF;delay_ms(40);
	LED_RED_ON;	delay_ms(100);LED_RED_OFF;delay_ms(600);	
}






