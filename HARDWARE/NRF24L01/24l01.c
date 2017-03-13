#include "delay.h"
#include "spi.h"	
#include "24L01.h"					  
//////////////////////////////////////////////////////////////////////////////////
extern u8 tmp_buf[32];
extern u8 txbuf[32];
u8 tmp_buf2[32]={0X22};
u8 TX_ADDRESS[5]  = {0x12,0x34,0x56,0x78,0xA1};//·�ɽڵ��ַ������2-5ͨ��ʱ�ĵ�һ����ַ����
u8 RX_ADDRESS[5]  = {0x12,0x34,0x56,0x78,0xA1};//·�ɽڵ��ַ������2-5ͨ��ʱ�ĵ�һ����ַ����
//��ʼ��24L01��IO��
void NRF24L01_Init(void)
{
  	RCC->APB2ENR|=1<<4;     //ʹ��PORTBʱ�� 	    
 	RCC->APB2ENR|=1<<2;		//ʹ��PORTAʱ��

	GPIOC->CRL&=0XFF0FFFFF; 
	GPIOC->CRL|=0X00800000;	//PB0 IRQ ���� 	
	GPIOC->ODR|=1<<5;    	//PB0 ����	 
	GPIOC->CRL&=0XFFF0FFFF; 
	GPIOC->CRL|=0X00030000;	//PB11 CE  ���   	    				   
	GPIOA->CRL&=0XFFF0FFFF; 
	GPIOA->CRL|=0X00030000;	//PA4 CSN ������� 	       
	
	SPI1_Init();    		//��ʼ��SPI	  
                        	//���NRF���ص��޸�SPI������
 	SPI1->CR1&=~(1<<6); 	//SPI�豸ʧ��
	SPI1->CR1&=~(1<<1); 	//����ģʽ��SCKΪ0 CPOL=0
	SPI1->CR1&=~(1<<0); 	//���ݲ����ӵ�1��ʱ����ؿ�ʼ,CPHA=0  
	SPI1->CR1|=1<<6; 		//SPI�豸ʹ��

	SPI1_SetSpeed(SPI_SPEED_4); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��    // ����ÿ�ζ�����SPI�ٶ�
	NRF24L01_CE=0; 			//ʹ��24L01
	NRF24L01_CSN=1;			//SPIƬѡȡ��	 		 	 
}
 	 	 
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN=0;                 //ʹ��SPI����
  	status =SPI1_ReadWriteByte(reg);//���ͼĴ����� 
  	SPI1_ReadWriteByte(value);      //д��Ĵ�����ֵ
  	NRF24L01_CSN=1;                 //��ֹSPI����	   
  	return(status);       			//����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN = 0;          //ʹ��SPI����		
  	SPI1_ReadWriteByte(reg);   //���ͼĴ�����
  	reg_val=SPI1_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
  	NRF24L01_CSN = 1;          //��ֹSPI����		    
  	return(reg_val);           //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN = 0;           //ʹ��SPI����
  	status=SPI1_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI1_ReadWriteByte(0XFF);//��������
  	NRF24L01_CSN=1;       //�ر�SPI����
  	return status;        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //ʹ��SPI����
  	status = SPI1_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI1_ReadWriteByte(*pBuf++); //д������	 
  	NRF24L01_CSN = 1;       //�ر�SPI����
  	return status;          //���ض�����״ֵ̬
}				   
 
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
u8 VAL=0;
u8 sss=0;
u8 NRF24L01_RxPacket(u8 *rxbuf,u8*txbuf)
{
	u8 sta;	 							   
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ   		
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//�������
	{	 	 
		VAL=NRF24L01_Read_Reg(0X60);
	//	NRF24L01_Write_Buf(W_ACK_PYLOD,txbuf,VAL);			// ����ACK ���ݰ�
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,VAL);			// ��ȡ�ѽ�������
		NRF24L01_Write_Reg(FLUSH_RX,0xff);					//���RX FIFO�Ĵ��� 
		return 	128;
	}   
	return 1;//û�յ��κ�����
}				    
//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������
u8 FF1=0,DYN=0;; 		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE=0;	  
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
  	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
			  
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0X1A);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);	     //����RFͨ��Ƶ��		
	
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_FEATURE , NRD_EN_DPL | NRF_EN_ACK_PAYLOAD);
    if(NRF24L01_Read_Reg(NRF_FEATURE)== 0x00 && NRF24L01_Read_Reg(NRF_DYNPD) == 0x00)
	{
        NRF24L01_Write_Reg(NRF_ACTIVATE, 0x73);
    }
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_FEATURE , NRD_EN_DPL | NRF_EN_ACK_PAYLOAD);
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_DYNPD, NRF_DPL_P0 | NRF_DPL_P1| NRF_DPL_P2| NRF_DPL_P3| NRF_DPL_P4| NRF_DPL_P5);
    	    	  
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0F);//����TX�������,0db����,1Mbps,���������濪��   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  	NRF24L01_CE = 1; //CEΪ��,�������ģʽ 
}						 

//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0X55,0X55,0X55,0X55,0X55};
	u8 i;
	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)if(buf[i]!=0X55)break;	 							   
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}	 



void  buhuo_init(void)    //PB6��7��8��9  TIM4
{
    RCC->APB1ENR|=1<<2;       //ʹ��TIMER4ʱ��
    RCC->APB2ENR|=1<<3;       //ʹ��PBʱ��
    
    GPIOB->CRL&=0x00FFFFFF;   //PB6��7��8��9���֮ǰ����
    GPIOB->CRL|=0x88000000;   //PB6��7��8��9����
    GPIOB->CRH&=0xFFFFFF00;   //PB6��7��8��9���֮ǰ����
    GPIOB->CRH|=0x00000088;   //PB6��7��8��9����
    
    GPIOB->ODR|=0<<0;         //PB6��7��8��9����
    TIM4->ARR=0xffff;         //�趨��ʱ���Զ���װֵ
    TIM4->PSC=71;             //Ԥ��Ƶ��
   
   
    TIM4->CCMR1|=1<<0|1<<8;   //CC1S=01  ѡ�������IC1ӳ�䵽TI1��			 
    TIM4->CCMR1|=0<<4|0<<12;  //IC1F=0000  ����������˲��������˲�
    TIM4->CCMR1|=0<<10;       //IC2PS=00  ���������Ƶ������Ƶ
    
    TIM4->CCMR2|=1<<0|1<<8;   //CC1S=01  ѡ�������IC1ӳ�䵽TI1��			 
    TIM4->CCMR2|=0<<4|0<<12;  //IC1F=0000  ����������˲��������˲�
    TIM4->CCMR2|=0<<10;       //IC2PS=00  ���������Ƶ������Ƶ
    
    
    TIM4->CCER|=0<<1|0<<5|0<<9|0<<13;         //CC1P=0  �����ز���
    TIM4->CCER|=1<<0|1<<4|1<<8|1<<12;         //CC1E=1  �������������ֵ������Ĵ�����
    
    
    TIM4->DIER|=1<<1|1<<2|1<<3|1<<4;        //�������ж�
    TIM2->DIER|=1<<0;        //��������ж�
    
    TIM4->CR1|=0x01;         //ʹ�ܶ�ʱ��2
    
    //MY_NVIC_Init2(1,2,TIM4_IRQn,2); //��ռ2�������ȼ�0����2
}
//����״̬
//��7����0��û�гɹ��Ĳ���1���ɹ�����һ��
//��6����0����û���񵽸ߵ�ƽ��1���Ѿ����񵽸ߵ�ƽ��
//��5��0��������ߵ�ƽ������Ĵ���
u16  TIM4CH1_CAPTURE_VAL,TIM4CH2_CAPTURE_VAL,TIM4CH3_CAPTURE_VAL,TIM4CH4_CAPTURE_VAL;          //���벶��ֵ
u8   TIM4CH1_CAPTURE_STA=0,TIM4CH2_CAPTURE_STA=0,TIM4CH3_CAPTURE_STA=0,TIM4CH4_CAPTURE_STA=0;  //���벶��״̬
u16  TIM4CH1_CAPTURE_VAL_OLD,TIM4CH2_CAPTURE_VAL_OLD,TIM4CH3_CAPTURE_VAL_OLD,TIM4CH4_CAPTURE_VAL_OLD;          //���벶��ֵ
//��ʱ��2�жϷ������
void TIM4_IRQHandler(void)
{
        u16  tsr;
//        led3_on;
        tsr=TIM4->SR;
//1*********************************************************************************************//        
        if((TIM4CH1_CAPTURE_STA&0x80)==0) //��δ�ɹ�����
        {
            if(tsr&0x01)//���
            {
               if((TIM4CH1_CAPTURE_STA&0x40)==0x40)//�Ѿ����񵽸ߵ�ƽ��
               {
                    if((TIM4CH1_CAPTURE_STA&0x3F)==0x3F)//�ߵ�ƽ̫����
                     {
                          TIM4CH1_CAPTURE_STA|=0x80;   //��ǳɹ�������һ��
                          TIM4CH1_CAPTURE_VAL=0xffff;
                      }
                    else  TIM4CH1_CAPTURE_STA++;
               }
             }
             if(tsr&(1<<1))                             //����1���������¼�
             {
                 if(TIM4CH1_CAPTURE_STA&0x40)         //����һ���½���
                 {
                   TIM4CH1_CAPTURE_STA|=0x80;         //��ǳɹ�����һ�θߵ�ƽ����
                   TIM4CH1_CAPTURE_VAL=TIM4->CCR1;    //��ȡ��ǰ�Ĳ���ֵ
                   TIM4->CCER&=~(1<<1);               //CC1P=0����Ϊ�����ز���
                 }
                 else                                 //��δ��ʼ����һ�β���������
                 {
                   TIM4CH1_CAPTURE_STA=0;              //���
                   TIM4CH1_CAPTURE_VAL=0;
                   TIM4CH1_CAPTURE_STA|=0x40;          //��ǲ�����������
                  // TIM4->CNT=0;                        //���������
                   TIM4CH1_CAPTURE_VAL_OLD=TIM4->CNT;
                   TIM4->CCER|=1<<1;                   //CC1P����Ϊ�½��ز���
                 }
             }     
        }
//2*********************************************************************************************// 
                if((TIM4CH2_CAPTURE_STA&0x80)==0) //��δ�ɹ�����
        {
            if(tsr&0x01)//���
            {
               if((TIM4CH2_CAPTURE_STA&0x40)==0x40)//�Ѿ����񵽸ߵ�ƽ��
               {
                    if((TIM4CH2_CAPTURE_STA&0x3F)==0x3F)//�ߵ�ƽ̫����
                     {
                          TIM4CH2_CAPTURE_STA|=0x80;   //��ǳɹ�������һ��
                          TIM4CH2_CAPTURE_VAL=0xffff;
                      }
                    else  TIM4CH2_CAPTURE_STA++;
               }
             }
             if(tsr&(1<<2))                             //����2���������¼�
             {
                 if(TIM4CH2_CAPTURE_STA&0x40)         //����һ���½���
                 {
                   TIM4CH2_CAPTURE_STA|=0x80;         //��ǳɹ�����һ�θߵ�ƽ����
                   TIM4CH2_CAPTURE_VAL=TIM4->CCR2;    //��ȡ��ǰ�Ĳ���ֵ
                   TIM4->CCER&=~(1<<5);               //CC1P=0����Ϊ�����ز���
                 }
                 else                                 //��δ��ʼ����һ�β���������
                 {
                   TIM4CH2_CAPTURE_STA=0;              //���
                   TIM4CH2_CAPTURE_VAL=0;
                   TIM4CH2_CAPTURE_STA|=0x40;          //��ǲ�����������
                //   TIM4->CNT=0;                        //���������
                   TIM4CH2_CAPTURE_VAL_OLD=TIM4->CNT;
                   TIM4->CCER|=1<<5;                   //CC1P����Ϊ�½��ز���
                 }
             }     
        }
//3*********************************************************************************************// 
                if((TIM4CH3_CAPTURE_STA&0x80)==0) //��δ�ɹ�����
        {
            if(tsr&0x01)//���
            {
               if((TIM4CH3_CAPTURE_STA&0x40)==0x40)//�Ѿ����񵽸ߵ�ƽ��
               {
                    if((TIM4CH3_CAPTURE_STA&0x3F)==0x3F)//�ߵ�ƽ̫����
                     {
                          TIM4CH3_CAPTURE_STA|=0x80;   //��ǳɹ�������һ��
                          TIM4CH3_CAPTURE_VAL=0xffff;
                      }
                    else  TIM4CH3_CAPTURE_STA++;
               }
             }
             if(tsr&(1<<3))                             //����3���������¼�
             {
                 if(TIM4CH3_CAPTURE_STA&0x40)         //����һ���½���
                 {
                   TIM4CH3_CAPTURE_STA|=0x80;         //��ǳɹ�����һ�θߵ�ƽ����
                   TIM4CH3_CAPTURE_VAL=TIM4->CCR3;    //��ȡ��ǰ�Ĳ���ֵ
                   TIM4->CCER&=~(1<<9);               //CC1P=0����Ϊ�����ز���
                 }
                 else                                 //��δ��ʼ����һ�β���������
                 {
                   TIM4CH3_CAPTURE_STA=0;              //���
                   TIM4CH3_CAPTURE_VAL=0;
                   TIM4CH3_CAPTURE_STA|=0x40;          //��ǲ�����������
                 //  TIM4->CNT=0;                        //���������
                   TIM4CH3_CAPTURE_VAL_OLD=TIM4->CNT;
                   TIM4->CCER|=1<<9;                   //CC1P����Ϊ�½��ز���
                 }
             }     
        }
//4*********************************************************************************************// 
                   if((TIM4CH4_CAPTURE_STA&0x80)==0) //��δ�ɹ�����
        {
            if(tsr&0x01)//���
            {
               if((TIM4CH4_CAPTURE_STA&0x40)==0x40)//�Ѿ����񵽸ߵ�ƽ��
               {
                    if((TIM4CH4_CAPTURE_STA&0x3F)==0x3F)//�ߵ�ƽ̫����
                     {
                          TIM4CH4_CAPTURE_STA|=0x80;   //��ǳɹ�������һ��
                          TIM4CH4_CAPTURE_VAL=0xffff;
                      }
                    else  TIM4CH4_CAPTURE_STA++;
               }
             }
             if(tsr&(1<<4))                             //����4���������¼�
             {
                 if(TIM4CH4_CAPTURE_STA&0x40)         //����һ���½���
                 {
                   TIM4CH4_CAPTURE_STA|=0x80;         //��ǳɹ�����һ�θߵ�ƽ����
                   TIM4CH4_CAPTURE_VAL=TIM4->CCR4;    //��ȡ��ǰ�Ĳ���ֵ
                   TIM4->CCER&=~(1<<13);               //CC1P=0����Ϊ�����ز���
                 }
                 else                                 //��δ��ʼ����һ�β���������
                 {
                   TIM4CH4_CAPTURE_STA=0;              //���
                   TIM4CH4_CAPTURE_VAL=0;
                   TIM4CH4_CAPTURE_STA|=0x40;          //��ǲ�����������
                 //  TIM4->CNT=0;                        //���������
                   TIM4CH4_CAPTURE_VAL_OLD=TIM4->CNT;
                   TIM4->CCER|=1<<13;                   //CC1P����Ϊ�½��ز���
                 }
             }     
        }
//******************************************************************************************//        
        
        TIM4->SR=0;    //����жϱ�־
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





 






