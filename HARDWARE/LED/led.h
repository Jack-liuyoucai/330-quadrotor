#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 							  

#define LED_RED_OFF  (GPIOB->ODR |=  (1<<14))
#define LED_RED_ON   (GPIOB->ODR &= ~(1<<14))
#define LED_RED_TST  (GPIOB->ODR ^=  (1<<14))

#define LED_GRE_OFF  (GPIOB->ODR |=  (1<<15))
#define LED_GRE_ON   (GPIOB->ODR &= ~(1<<15))
#define LED_GRE_TST  (GPIOB->ODR ^=  (1<<15))

#define LED_0_OFF  (GPIOB->ODR |=  (1<<12))
#define LED_0_ON   (GPIOB->ODR &= ~(1<<12))
#define LED_0_TST  (GPIOB->ODR ^=  (1<<12))

#define LED_1_OFF  (GPIOB->ODR |=  (1<<13))
#define LED_1_ON   (GPIOB->ODR &= ~(1<<13))
#define LED_1_TST  (GPIOB->ODR ^=  (1<<13))

void LED_Init(void);//³õÊ¼»¯	
void Show_str_leds(void);
void ALL_LEDS_ON(void);
void ALL_LEDS_OFF(void);
void LED_twinkle(void);
	 				    
#endif

















