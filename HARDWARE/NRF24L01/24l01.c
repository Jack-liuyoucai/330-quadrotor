#include "delay.h"
#include "spi.h"	
#include "24L01.h"					  
//////////////////////////////////////////////////////////////////////////////////
extern u8 tmp_buf[32];
extern u8 txbuf[32];
u8 tmp_buf2[32]={0X22};
u8 TX_ADDRESS[5]  = {0x12,0x34,0x56,0x78,0xA1};//路由节点地址，更改2-5通道时改第一个地址即可
u8 RX_ADDRESS[5]  = {0x12,0x34,0x56,0x78,0xA1};//路由节点地址，更改2-5通道时改第一个地址即可
//初始化24L01的IO口
void NRF24L01_Init(void)
{
  	RCC->APB2ENR|=1<<4;     //使能PORTB时钟 	    
 	RCC->APB2ENR|=1<<2;		//使能PORTA时钟

	GPIOC->CRL&=0XFF0FFFFF; 
	GPIOC->CRL|=0X00800000;	//PB0 IRQ 输入 	
	GPIOC->ODR|=1<<5;    	//PB0 上拉	 
	GPIOC->CRL&=0XFFF0FFFF; 
	GPIOC->CRL|=0X00030000;	//PB11 CE  输出   	    				   
	GPIOA->CRL&=0XFFF0FFFF; 
	GPIOA->CRL|=0X00030000;	//PA4 CSN 推挽输出 	       
	
	SPI1_Init();    		//初始化SPI	  
                        	//针对NRF的特点修改SPI的设置
 	SPI1->CR1&=~(1<<6); 	//SPI设备失能
	SPI1->CR1&=~(1<<1); 	//空闲模式下SCK为0 CPOL=0
	SPI1->CR1&=~(1<<0); 	//数据采样从第1个时间边沿开始,CPHA=0  
	SPI1->CR1|=1<<6; 		//SPI设备使能

	SPI1_SetSpeed(SPI_SPEED_4); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）    // 不用每次都设置SPI速度
	NRF24L01_CE=0; 			//使能24L01
	NRF24L01_CSN=1;			//SPI片选取消	 		 	 
}
 	 	 
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN=0;                 //使能SPI传输
  	status =SPI1_ReadWriteByte(reg);//发送寄存器号 
  	SPI1_ReadWriteByte(value);      //写入寄存器的值
  	NRF24L01_CSN=1;                 //禁止SPI传输	   
  	return(status);       			//返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN = 0;          //使能SPI传输		
  	SPI1_ReadWriteByte(reg);   //发送寄存器号
  	reg_val=SPI1_ReadWriteByte(0XFF);//读取寄存器内容
  	NRF24L01_CSN = 1;          //禁止SPI传输		    
  	return(reg_val);           //返回状态值
}	
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN = 0;           //使能SPI传输
  	status=SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI1_ReadWriteByte(0XFF);//读出数据
  	NRF24L01_CSN=1;       //关闭SPI传输
  	return status;        //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //使能SPI传输
  	status = SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI1_ReadWriteByte(*pBuf++); //写入数据	 
  	NRF24L01_CSN = 1;       //关闭SPI传输
  	return status;          //返回读到的状态值
}				   
 
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
u8 VAL=0;
u8 sss=0;
u8 NRF24L01_RxPacket(u8 *rxbuf,u8*txbuf)
{
	u8 sta;	 							   
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值   		
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//发送完成
	{	 	 
		VAL=NRF24L01_Read_Reg(0X60);
	//	NRF24L01_Write_Buf(W_ACK_PYLOD,txbuf,VAL);			// 发送ACK 数据包
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,VAL);			// 读取已接收数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);					//清除RX FIFO寄存器 
		return 	128;
	}   
	return 1;//没收到任何数据
}				    
//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了
u8 FF1=0,DYN=0;; 		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE=0;	  
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
  	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
			  
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0X1A);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);	     //设置RF通信频率		
	
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_FEATURE , NRD_EN_DPL | NRF_EN_ACK_PAYLOAD);
    if(NRF24L01_Read_Reg(NRF_FEATURE)== 0x00 && NRF24L01_Read_Reg(NRF_DYNPD) == 0x00)
	{
        NRF24L01_Write_Reg(NRF_ACTIVATE, 0x73);
    }
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_FEATURE , NRD_EN_DPL | NRF_EN_ACK_PAYLOAD);
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_DYNPD, NRF_DPL_P0 | NRF_DPL_P1| NRF_DPL_P2| NRF_DPL_P3| NRF_DPL_P4| NRF_DPL_P5);
    	    	  
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0F);//设置TX发射参数,0db增益,1Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	NRF24L01_CE = 1; //CE为高,进入接收模式 
}						 

//检测24L01是否存在
//返回值:0，成功;1，失败	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0X55,0X55,0X55,0X55,0X55};
	u8 i;
	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0X55)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}	 



void  buhuo_init(void)    //PB6，7，8，9  TIM4
{
    RCC->APB1ENR|=1<<2;       //使能TIMER4时钟
    RCC->APB2ENR|=1<<3;       //使能PB时钟
    
    GPIOB->CRL&=0x00FFFFFF;   //PB6，7，8，9清除之前设置
    GPIOB->CRL|=0x88000000;   //PB6，7，8，9输入
    GPIOB->CRH&=0xFFFFFF00;   //PB6，7，8，9清除之前设置
    GPIOB->CRH|=0x00000088;   //PB6，7，8，9输入
    
    GPIOB->ODR|=0<<0;         //PB6，7，8，9下拉
    TIM4->ARR=0xffff;         //设定定时器自动重装值
    TIM4->PSC=71;             //预分频器
   
   
    TIM4->CCMR1|=1<<0|1<<8;   //CC1S=01  选择输入端IC1映射到TI1上			 
    TIM4->CCMR1|=0<<4|0<<12;  //IC1F=0000  配置输入过滤波器，不滤波
    TIM4->CCMR1|=0<<10;       //IC2PS=00  配置输入分频，不分频
    
    TIM4->CCMR2|=1<<0|1<<8;   //CC1S=01  选择输入端IC1映射到TI1上			 
    TIM4->CCMR2|=0<<4|0<<12;  //IC1F=0000  配置输入过滤波器，不滤波
    TIM4->CCMR2|=0<<10;       //IC2PS=00  配置输入分频，不分频
    
    
    TIM4->CCER|=0<<1|0<<5|0<<9|0<<13;         //CC1P=0  上升沿捕获
    TIM4->CCER|=1<<0|1<<4|1<<8|1<<12;         //CC1E=1  允许捕获计数器的值到捕获寄存器中
    
    
    TIM4->DIER|=1<<1|1<<2|1<<3|1<<4;        //允许捕获中断
    TIM2->DIER|=1<<0;        //允许更新中断
    
    TIM4->CR1|=0x01;         //使能定时器2
    
    //MY_NVIC_Init2(1,2,TIM4_IRQn,2); //抢占2，子优先级0，组2
}
//捕获状态
//【7】：0，没有成功的捕获；1，成功捕获到一次
//【6】：0，还没捕获到高电平；1，已经捕获到高电平了
//【5：0】：捕获高电平后溢出的次数
u16  TIM4CH1_CAPTURE_VAL,TIM4CH2_CAPTURE_VAL,TIM4CH3_CAPTURE_VAL,TIM4CH4_CAPTURE_VAL;          //输入捕获值
u8   TIM4CH1_CAPTURE_STA=0,TIM4CH2_CAPTURE_STA=0,TIM4CH3_CAPTURE_STA=0,TIM4CH4_CAPTURE_STA=0;  //输入捕获状态
u16  TIM4CH1_CAPTURE_VAL_OLD,TIM4CH2_CAPTURE_VAL_OLD,TIM4CH3_CAPTURE_VAL_OLD,TIM4CH4_CAPTURE_VAL_OLD;          //输入捕获值
//定时器2中断服务程序
void TIM4_IRQHandler(void)
{
        u16  tsr;
//        led3_on;
        tsr=TIM4->SR;
//1*********************************************************************************************//        
        if((TIM4CH1_CAPTURE_STA&0x80)==0) //还未成功捕获
        {
            if(tsr&0x01)//溢出
            {
               if((TIM4CH1_CAPTURE_STA&0x40)==0x40)//已经捕获到高电平了
               {
                    if((TIM4CH1_CAPTURE_STA&0x3F)==0x3F)//高电平太长了
                     {
                          TIM4CH1_CAPTURE_STA|=0x80;   //标记成功捕获了一次
                          TIM4CH1_CAPTURE_VAL=0xffff;
                      }
                    else  TIM4CH1_CAPTURE_STA++;
               }
             }
             if(tsr&(1<<1))                             //捕获1发生捕获事件
             {
                 if(TIM4CH1_CAPTURE_STA&0x40)         //捕获到一个下降沿
                 {
                   TIM4CH1_CAPTURE_STA|=0x80;         //标记成功捕获到一次高电平脉宽
                   TIM4CH1_CAPTURE_VAL=TIM4->CCR1;    //获取当前的捕获值
                   TIM4->CCER&=~(1<<1);               //CC1P=0设置为上升沿捕获
                 }
                 else                                 //还未开始，第一次捕获上升沿
                 {
                   TIM4CH1_CAPTURE_STA=0;              //清空
                   TIM4CH1_CAPTURE_VAL=0;
                   TIM4CH1_CAPTURE_STA|=0x40;          //标记捕获到了上升沿
                  // TIM4->CNT=0;                        //计数器清空
                   TIM4CH1_CAPTURE_VAL_OLD=TIM4->CNT;
                   TIM4->CCER|=1<<1;                   //CC1P设置为下降沿捕获
                 }
             }     
        }
//2*********************************************************************************************// 
                if((TIM4CH2_CAPTURE_STA&0x80)==0) //还未成功捕获
        {
            if(tsr&0x01)//溢出
            {
               if((TIM4CH2_CAPTURE_STA&0x40)==0x40)//已经捕获到高电平了
               {
                    if((TIM4CH2_CAPTURE_STA&0x3F)==0x3F)//高电平太长了
                     {
                          TIM4CH2_CAPTURE_STA|=0x80;   //标记成功捕获了一次
                          TIM4CH2_CAPTURE_VAL=0xffff;
                      }
                    else  TIM4CH2_CAPTURE_STA++;
               }
             }
             if(tsr&(1<<2))                             //捕获2发生捕获事件
             {
                 if(TIM4CH2_CAPTURE_STA&0x40)         //捕获到一个下降沿
                 {
                   TIM4CH2_CAPTURE_STA|=0x80;         //标记成功捕获到一次高电平脉宽
                   TIM4CH2_CAPTURE_VAL=TIM4->CCR2;    //获取当前的捕获值
                   TIM4->CCER&=~(1<<5);               //CC1P=0设置为上升沿捕获
                 }
                 else                                 //还未开始，第一次捕获上升沿
                 {
                   TIM4CH2_CAPTURE_STA=0;              //清空
                   TIM4CH2_CAPTURE_VAL=0;
                   TIM4CH2_CAPTURE_STA|=0x40;          //标记捕获到了上升沿
                //   TIM4->CNT=0;                        //计数器清空
                   TIM4CH2_CAPTURE_VAL_OLD=TIM4->CNT;
                   TIM4->CCER|=1<<5;                   //CC1P设置为下降沿捕获
                 }
             }     
        }
//3*********************************************************************************************// 
                if((TIM4CH3_CAPTURE_STA&0x80)==0) //还未成功捕获
        {
            if(tsr&0x01)//溢出
            {
               if((TIM4CH3_CAPTURE_STA&0x40)==0x40)//已经捕获到高电平了
               {
                    if((TIM4CH3_CAPTURE_STA&0x3F)==0x3F)//高电平太长了
                     {
                          TIM4CH3_CAPTURE_STA|=0x80;   //标记成功捕获了一次
                          TIM4CH3_CAPTURE_VAL=0xffff;
                      }
                    else  TIM4CH3_CAPTURE_STA++;
               }
             }
             if(tsr&(1<<3))                             //捕获3发生捕获事件
             {
                 if(TIM4CH3_CAPTURE_STA&0x40)         //捕获到一个下降沿
                 {
                   TIM4CH3_CAPTURE_STA|=0x80;         //标记成功捕获到一次高电平脉宽
                   TIM4CH3_CAPTURE_VAL=TIM4->CCR3;    //获取当前的捕获值
                   TIM4->CCER&=~(1<<9);               //CC1P=0设置为上升沿捕获
                 }
                 else                                 //还未开始，第一次捕获上升沿
                 {
                   TIM4CH3_CAPTURE_STA=0;              //清空
                   TIM4CH3_CAPTURE_VAL=0;
                   TIM4CH3_CAPTURE_STA|=0x40;          //标记捕获到了上升沿
                 //  TIM4->CNT=0;                        //计数器清空
                   TIM4CH3_CAPTURE_VAL_OLD=TIM4->CNT;
                   TIM4->CCER|=1<<9;                   //CC1P设置为下降沿捕获
                 }
             }     
        }
//4*********************************************************************************************// 
                   if((TIM4CH4_CAPTURE_STA&0x80)==0) //还未成功捕获
        {
            if(tsr&0x01)//溢出
            {
               if((TIM4CH4_CAPTURE_STA&0x40)==0x40)//已经捕获到高电平了
               {
                    if((TIM4CH4_CAPTURE_STA&0x3F)==0x3F)//高电平太长了
                     {
                          TIM4CH4_CAPTURE_STA|=0x80;   //标记成功捕获了一次
                          TIM4CH4_CAPTURE_VAL=0xffff;
                      }
                    else  TIM4CH4_CAPTURE_STA++;
               }
             }
             if(tsr&(1<<4))                             //捕获4发生捕获事件
             {
                 if(TIM4CH4_CAPTURE_STA&0x40)         //捕获到一个下降沿
                 {
                   TIM4CH4_CAPTURE_STA|=0x80;         //标记成功捕获到一次高电平脉宽
                   TIM4CH4_CAPTURE_VAL=TIM4->CCR4;    //获取当前的捕获值
                   TIM4->CCER&=~(1<<13);               //CC1P=0设置为上升沿捕获
                 }
                 else                                 //还未开始，第一次捕获上升沿
                 {
                   TIM4CH4_CAPTURE_STA=0;              //清空
                   TIM4CH4_CAPTURE_VAL=0;
                   TIM4CH4_CAPTURE_STA|=0x40;          //标记捕获到了上升沿
                 //  TIM4->CNT=0;                        //计数器清空
                   TIM4CH4_CAPTURE_VAL_OLD=TIM4->CNT;
                   TIM4->CCER|=1<<13;                   //CC1P设置为下降沿捕获
                 }
             }     
        }
//******************************************************************************************//        
        
        TIM4->SR=0;    //清除中断标志
} 

void  read_control_cmd0(void)
{
  float  temp3;
  do{
  if(TIM4CH3_CAPTURE_STA&0x80)
      {
      temp3=TIM4CH3_CAPTURE_STA&0x3f; 
      temp3*=65536;  
      temp3+=TIM4CH3_CAPTURE_VAL;
      temp3-=TIM4CH3_CAPTURE_VAL_OLD;
      TIM4CH3_CAPTURE_STA=0;
      }
  }
  while(temp3>(4140+25));
}





 






