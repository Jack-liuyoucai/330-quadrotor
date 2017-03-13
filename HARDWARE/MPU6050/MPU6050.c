#include "MPU6050.h"
#include "STM32_I2C.h"
#include "delay.h"
#include "math.h"
#include "led.h"
#include "define.h"

u8 mpu6050_buffer[14]={0};
#define Gyr_Gain 0.06097;
extern struct DATA_XYZ ACC_OFFSET;
extern struct DATA_XYZ GYR_OFFSET;
extern struct DATA_XYZ ACC;
extern struct DATA_XYZ GYR;
extern struct DATA_XYZ GYR_RATE;
extern struct DATA_XYZ GYR_CTR;

struct DATA_XYZ_F gyr_offset_f={0.0,0.0,0.0};

/**************************ʵ�ֺ���********************************************
//��iic��ȡ�������ݷֲ�,������Ӧ�Ĵ���
*******************************************************************************/
void MPU6050_CONVENT(void)
{
	ACC.X=((u16)(mpu6050_buffer[0] << 8) | mpu6050_buffer[1]);
	ACC.Y=((u16)(mpu6050_buffer[2] << 8) | mpu6050_buffer[3]);
	ACC.Z=((u16)(mpu6050_buffer[4] << 8) | mpu6050_buffer[5]);
	
	GYR_RATE.X=((u16)(mpu6050_buffer[8] << 8) | mpu6050_buffer[9]);
	GYR_RATE.Y=((u16)(mpu6050_buffer[10] << 8) | mpu6050_buffer[11]);
	GYR_RATE.Z=((u16)(mpu6050_buffer[12] << 8) | mpu6050_buffer[13]);

	GYR.X= GYR_RATE.X-GYR_OFFSET.X;
	GYR.Y= GYR_RATE.Y-GYR_OFFSET.Y;
	GYR.Z= GYR_RATE.Z-GYR_OFFSET.Z;
	
}
/**************************ʵ�ֺ���********************************************
//��iic��ȡ�������ݷֲ�,������Ӧ�Ĵ���,����MPU6050_Last
*******************************************************************************/
void MPU6050_READ(void)
{
	i2cRead(devAddr,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
ʧ��Ϊ0
*******************************************************************************/ 
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
	u8 b;
	i2cRead(dev, reg, 1, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	i2cWrite(dev, reg, b);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitStart  Ŀ���ֽڵ���ʼλ
length   λ����
data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
ʧ��Ϊ0
*******************************************************************************/ 
void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{	
	u8 b;
	u8 mask;
	i2cRead(dev, reg, 1, &b);
	mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	b &= mask;
	b |= data;
	i2cWrite(dev, reg, b);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setClockSource(uint8_t source)
*��������:	    ����  MPU6050 ��ʱ��Դ
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(u8 source){
	IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
	
}
/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see MPU6050_GYRO_FS_250
* @see MPU6050_RA_GYRO_CONFIG
* @see MPU6050_GCONFIG_FS_SEL_BIT
* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_setFullScaleGyroRange(u8 range) {
	IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(u8 range) {
	IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
*��������:	    ����  MPU6050 �Ƿ����˯��ģʽ
enabled =1   ˯��
enabled =0   ����
*******************************************************************************/
void MPU6050_setSleepEnabled(u8 enabled) {
	IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(u8 enabled) {
	IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(u8 enabled) {
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_setDLPF(u8 mode)
{
	IICwriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}
void MPU6050_setRATE(u8 rate)
{
	IICwriteBits(devAddr, MPU6050_RA_SMPLRT_DIV, 0, 8, rate);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_initialize(void)
*��������:	    ��ʼ�� 	MPU6050 �Խ������״̬��
*******************************************************************************/
void MPU6050_INIT(void)
{
	i2cInit();
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //����ʱ��  0x6b   0x01
	delay_ms(2);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//������������� +-2000��ÿ��
	delay_ms(2);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_4);	//���ٶȶ�������� +-4G
	delay_ms(2);
 //	MPU6050_setRATE(15);								   /////////
	delay_ms(2);
	MPU6050_setDLPF(MPU6050_DLPF_BW_42);
	delay_ms(2);
	MPU6050_setSleepEnabled(0); //���빤��״̬
	delay_ms(2);
	MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C
	delay_ms(2);
	MPU6050_setI2CBypassEnabled(1);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
	delay_ms(2);

}
void Get_OFFSET(u16 average_times)
{
	u16 ct=0;
	long ACCSUM_X=0;
	long ACCSUM_Y=0;
	long ACCSUM_Z=0;
	long GYRSUM_X=0;
	long GYRSUM_Y=0;
	long GYRSUM_Z=0;
	for(ct=0;ct<average_times;ct++)
	{
		if(ct%20==0)  LED_GRE_TST;		
		MPU6050_READ();
		MPU6050_CONVENT();	
	 	ACCSUM_X+=ACC.X;
		ACCSUM_Y+=ACC.Y;
		ACCSUM_Z+=ACC.Z-8192;
		GYRSUM_X+=GYR.X;
		GYRSUM_Y+=GYR.Y;
		GYRSUM_Z+=GYR.Z;
		delay_ms(3)	;
	}
	ACC_OFFSET.X=ACCSUM_X/average_times; 
	ACC_OFFSET.Y=ACCSUM_Y/average_times;
	ACC_OFFSET.Z=ACCSUM_Z/average_times;
	GYR_OFFSET.X=GYRSUM_X/average_times;
	GYR_OFFSET.Y=GYRSUM_Y/average_times;
	GYR_OFFSET.Z=GYRSUM_Z/average_times;
	gyr_offset_f.X=GYR_OFFSET.X*Gyr_Gain;
	gyr_offset_f.Y=GYR_OFFSET.Y*Gyr_Gain;
	gyr_offset_f.Z=GYR_OFFSET.Z*Gyr_Gain;
	GYR_CTR.X=GYR_OFFSET.X;
	GYR_CTR.Y=GYR_OFFSET.Y;
}


