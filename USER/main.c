#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"	
#include "spi.h"
#include "exti.h"
#include "timer.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "STM32_I2C.h"
#include "math.h"
#include "24L01.h"	
#include "define.h"
#include "imu.h"
#include "control.h"

extern struct DATA_XYZ_F GYR_F;
extern struct DATA_XYZ ACC_AVG;  
extern struct DATA_XYZ ACC_OFFSET;
extern struct DATA_XYZ GYR_OFFSET;
extern struct DATA_XYZ HMC;
extern float rool,pitch;
extern int Motor1,Motor2,Motor3,Motor4; 
extern struct DATA_XYZ ACC;
extern float Pitch,Rool,Yaw;

u8 RX_buf[32]={0}; 
u8 TX_buf[32]={0};
u8 delays=10;	  ////  IIC delayspeed
u8 wireless=0;
u32 loss_wireless=0;
extern u8 FLY_EN; 
 float  tempa1,tempa2,tempa3,tempa4;

void  read_control_cmd(void)
{
      //float  t1,t2,t12;
      if(TIM4CH1_CAPTURE_STA&0x80)
      {
      tempa1=TIM4CH1_CAPTURE_STA&0x3f; 
      tempa1*=65536;  
      tempa1+=TIM4CH1_CAPTURE_VAL;
      tempa1-=TIM4CH1_CAPTURE_VAL_OLD;
      TIM4CH1_CAPTURE_STA=0;
      }
       if(TIM4CH2_CAPTURE_STA&0x80)
      {
      tempa2=TIM4CH2_CAPTURE_STA&0x3f; 
      tempa2*=65536;  
      tempa2+=TIM4CH2_CAPTURE_VAL; 
      tempa2-=TIM4CH2_CAPTURE_VAL_OLD;
      TIM4CH2_CAPTURE_STA=0;
      }
       if(TIM4CH3_CAPTURE_STA&0x80)
      {
      tempa3=TIM4CH3_CAPTURE_STA&0x3f; 
      tempa3*=65536;  
      tempa3+=TIM4CH3_CAPTURE_VAL;
      tempa3-=TIM4CH3_CAPTURE_VAL_OLD;
      TIM4CH3_CAPTURE_STA=0;
      }
       if(TIM4CH4_CAPTURE_STA&0x80)
      {
      tempa4=TIM4CH4_CAPTURE_STA&0x3f; 
      tempa4*=65536;  
      tempa4+=TIM4CH4_CAPTURE_VAL; 
      tempa4-=TIM4CH4_CAPTURE_VAL_OLD;
      TIM4CH4_CAPTURE_STA=0;
      }
} 
int main(void)			
{			  
	Stm32_Clock_Init(9);	//系统时钟设置	 
	delay_init(72);	   		//延时初始化
	delay_ms(1000);
	Stm32_Clock_Init(9);	//系统时钟设置
	delay_init(72);	 		
	uart_init(72,115200);	 	//串口初始化为9600
	RCC->APB2ENR|=1<<0;	  	  // 开辅助时钟	//////////////////////////////////////////
	AFIO->MAPR = 0x02000000;  // 禁用JTAG	 /////////////////////////////////////////////
	LED_Init();	
	Show_str_leds();

 /*	NRF24L01_Init();			//  无线初始化///////////////////////////////////////////////////////////////
  	while( NRF24L01_Check());
 	NRF24L01_RX_Mode();     	//拿出数据包
*/
  //  buhuo_init();
  //  read_control_cmd0();

	LED_RED_ON;
	delays=20;	     	    	//  init with low_speed iic	
    MPU6050_INIT();delay_ms(50);
	MPU6050_INIT();delay_ms(50);
	Get_OFFSET(500); 
    delays=10;			  	//  cap  with high_speed iic
	if(ACC_OFFSET.X==0&&ACC_OFFSET.Y==0){ALL_LEDS_ON();while(1);}	 // 传感器失败,开灯阻塞 
	PID_INIT();	  	

	TIM2_PWM_Init(19999,71); //999   2           	  
	TIM3_Int_Init(20,7199);  // 定时2MS 
	delay_ms(50000); 	ALL_LEDS_OFF();
	while(1) 
	{	  
	 read_control_cmd();
	 UART1_ReportIMU(Yaw,Pitch,Rool,0,0,0,0);
	} 		  		 
}
////////////////////////////////////////////////// TIMER3 _interrupt in 2ms----->>>>  Control 

float 	angle=0;
long acc_hold[3]={0};
u16 tms=0;
float p_long=0.0,r_long=0.0;
float adj_r=0.0,adj_p=0.0;
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//溢出中断
	{					       				   				     	    	 
    	MPU6050_READ();
		MPU6050_CONVENT();
		ACC_SMOOTH(10);  // ACC 平滑滤波
	//	RX_buf[31]= 0;   ///// 
		MahonyIMUupdate(GYR_F.X,GYR_F.Y,GYR_F.Z,(float)ACC_AVG.X,(float)ACC_AVG.Y,(float)ACC_AVG.Z);	//函数内转成弧度
	//	Tx_buf();		
	/*	NRF24L01_RxPacket(RX_buf,TX_buf);	// CHECK WIRLE LESS		//				   		
		if(RX_buf[31]==0)		  //  Check in every 2ms , and send in every 10ms 
		{
			loss_wireless++;
			if(loss_wireless>10)  //FLY_EN==1&& loss 20ms 
			{
				wireless=0;
				CRASH_LANDING();	
			}
		}
		else if(RX_buf[31]==0XAA)  // 正常接收  
		{
		
			LED_GRE_OFF;	
			loss_wireless=0;
			wireless=1;	
		} 
			*/
	    READ_CONTROL_COMMAND();	
		STABLE_WITH_PID();	  ////  PID CONTROL	      			   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    			    		  			   
}
 
