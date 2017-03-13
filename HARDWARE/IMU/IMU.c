#include "imu.h"
#include "define.h"
#include "mpu6050.h"
#include "math.h"
#include "fast_math.h"

#define Gyr_Gain 0.06097
//0.015267   
extern float Pitch,Rool,Yaw;
extern struct DATA_XYZ_F GYR_F;
extern struct DATA_XYZ_F GYR_F_RATE;
extern struct DATA_XYZ ACC;
extern struct DATA_XYZ GYR;
extern struct DATA_XYZ GYR_RATE;
extern struct DATA_XYZ ACC_AVG; 

short ACC_X_BUF[10]={0};
short ACC_Y_BUF[10]={0};
short ACC_Z_BUF[10]={0};

void ACC_SMOOTH(u8 smooth_tms)
{
	static u8 filter_cnt=0;
	int temp1=0,temp2=0,temp3=0;
	u8 i;

	GYR_F.X = GYR.X * Gyr_Gain;
	GYR_F.Y = GYR.Y * Gyr_Gain;
	GYR_F.Z = GYR.Z * Gyr_Gain;
	GYR_F_RATE.X = GYR_RATE.X * Gyr_Gain;
	GYR_F_RATE.Y = GYR_RATE.Y * Gyr_Gain;
	GYR_F_RATE.Z = GYR_RATE.Z * Gyr_Gain;

	ACC_X_BUF[filter_cnt] = ACC.X;//更新滑动窗口数组
	ACC_Y_BUF[filter_cnt] = ACC.Y;
	ACC_Z_BUF[filter_cnt] = ACC.Z;
	for(i=0;i<smooth_tms;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	ACC_AVG.X = temp1 / (float)smooth_tms;
	ACC_AVG.Y = temp2 / (float)smooth_tms;
	ACC_AVG.Z = temp3 / (float)smooth_tms;
	filter_cnt++;
	if(filter_cnt==smooth_tms)	filter_cnt=0;	 		  
}
 

float twoKp = 2;    // 2 * proportional gain (Kp)
float twoKi = 0.005;    // 2 * integral gain (Ki)
float dt =0.0025;
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;  // integral error terms scaled by Ki
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  

void MahonyIMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
	gx*=0.0174;
    gy*=0.0174;
    gz*=0.0174;
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    // Compute and apply integral feedback if enabled
    integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
    integralFBy += twoKi * halfey * dt;
    integralFBz += twoKi * halfez * dt;
    gx += integralFBx;  // apply integral feedback
    gy += integralFBy;
    gz += integralFBz;
    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
    // Integrate rate of change of quaternion
    gx *= 0.00125f;//(0.5f * dt);   // pre-multiply common factors
  	gy *= 0.00125f;//(0.5f * dt);
 	gz *= 0.00125f;//(0.5f * dt);
  	qa = q0;
  	qb = q1;
  	qc = q2;
  	q0 += (-qb * gx - qc * gy - q3 * gz);
  	q1 += (qa * gx + qc * gz - q3 * gy);
  	q2 += (qa * gy - qb * gz + q3 * gx);
  	q3 += (qa * gz + qb * gy - qc * gx);

  	// Normalise quaternion
  	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  	q0 *= recipNorm;
  	q1 *= recipNorm;
  	q2 *= recipNorm;
  	q3 *= recipNorm;

    Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
    Rool = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
	  if(Rool>90||Rool<-90)
	  {
	  	if(Pitch>0)
		Pitch=180-Pitch;
		if(Pitch<0)
		Pitch=-(180+Pitch);
	  }	  
	Yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 57.295;	
	if(Yaw<0) Yaw+=360;
}

/*
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float  a=0.0,b=0.0,c=0.0; 
    gx*=0.0174;
    gy*=0.0174;
    gz*=0.0174;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f)) {
		MahonyIMUupdate(gx, gy, gz, ax, ay, az);
		return;
	}
	// Normalise accelerometer measurement
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;     

	// Normalise magnetometer measurement
	recipNorm = invSqrt(mx * mx + my * my + mz * mz);
	mx *= recipNorm;
	my *= recipNorm;
	mz *= recipNorm;   

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;   

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

	// Estimated direction of gravity and magnetic field
	halfvx = q1q3 - q0q2;
	halfvy = q0q1 + q2q3;
	halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
	// Error is sum of cross product between estimated direction and measured direction of field vectors
	halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
	halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
	halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

	// Compute and apply integral feedback if enabled
	integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
	integralFBy += twoKi * halfey * dt;
	integralFBz += twoKi * halfez * dt;
	gx += integralFBx;	// apply integral feedback
	gy += integralFBy;
	gz += integralFBz;

	// Apply proportional feedback
	gx += twoKp * halfex;
	gy += twoKp * halfey;
	gz += twoKp * halfez;
	
	// Integrate rate of change of quaternion
	gx *= 0.001f;//(0.5f * dt);		// pre-multiply common factors
	gy *= 0.001f;//(0.5f * dt);
	gz *= 0.001f;//(0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

  	a = 2 * (q1*q3 - q0*q2);
  	b = 2 * (q0*q1 + q2*q3);
  	c = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  	Pitch =- atan(a / sqrt(b*b + c*c)) * 57.295;
  	Rool = atan(b / sqrt(a*a + c*c)) * 57.295;	
		
	//Yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 57.295;
} 
	  
float Kp=1.6 ;		//  2.5
float Ki=0.002;	  // 应该大一点
#define halfT 0.001f                 
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   
float exInt = 0, eyInt = 0, ezInt = 0;  
//float Pitch=0.0,Rool=0.0;	
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
 // float hx, hy, hz, bx, bz;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  float q0q0 = q0*q0;					  
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
//  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
//  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;

  gx*=0.0174;
  gy*=0.0174;
  gz*=0.0174;
  norm = sqrt(ax*ax + ay*ay + az*az);      
  ax = ax /norm;										
  ay = ay / norm;
  az = az / norm;
           
  vx = 2*(q1q3 - q0q2);												
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  ex = (ay*vz - az*vy) ;                           					
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								 
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;
															 
  gx = gx + Kp*ex + exInt;					   							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							
					   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  Rool = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll

}			   */

#define FILTER_NUM 19

float Accbuffer_x[FILTER_NUM]={0};
float Accbuffer_y[FILTER_NUM]={0};
float Accbuffer_z[FILTER_NUM]={0};

void ACC_Filter_with_FIR()
{
		  unsigned char i=0;
/*	 	  float hn[FILTER_NUM]={																		//FIR滤波常数（凯赛窗），带通频率100Hz,带阻频率200hz
		  					 -0.0048,0.0000,0.0155,0.0186,-0.0152,
 							 -0.0593,-0.0345,0.1045,0.2881,0.3739,
 							  0.2881,0.1045,-0.0345,-0.0593,
 							 -0.0152,0.0186,0.0155,0.0000,-0.0048
		  };       */
/*	float hn[FILTER_NUM]={
		 0.0293,0.0312,0.0331,0.0347,0.0361,0.0374,0.0384,0.0392,0.0397,0.0400,0.0400,0.0397,0.0392,0.0384,
		 0.0374,0.0361,0.0347,0.0331,0.0312,0.0293
		 } ;*/
	/*6hz FC=400HZ*//*	 float hn[FILTER_NUM]={
		  0.0249,0.0259,0.0269,0.0277,0.0284,0.0289,0.0294,0.0297,0.0299,0.0300,0.0299,0.0297,0.0294,0.0289,
		0.0284,0.0277,0.0269,0.0259,0.0249} ;	 */	
   float hn[FILTER_NUM]={0.0137,0.0140,0.0142,0.0144,0.0146,0.0147,0.0148,0.0149,0.0150,0.0150,0.0150,0.0149,0.0148,0.0147,
   0.0146,0.0144,0.0142,0.0140,0.0137};
		  float Acctmp_x=0;
		  float Acctmp_y=0;
		  float Acctmp_z=0;
/*------------------------------------------FIR滤波--------------------------------------------------------*/

		Accbuffer_x[0]=(float)ACC.X;
		Accbuffer_y[0]=(float)ACC.Y;
		Accbuffer_z[0]=(float)ACC.Z;

		for(i=0;i<FILTER_NUM;i++)
		{
			Acctmp_x+=hn[i]*Accbuffer_x[i];
			Acctmp_y+=hn[i]*Accbuffer_y[i];
			Acctmp_z+=hn[i]*Accbuffer_z[i];			
		}

		 ACC_AVG.X = (short)Acctmp_x ;
		 ACC_AVG.Y = (short)Acctmp_y ;
		 ACC_AVG.Z = (short)Acctmp_z ;	

		for(i=0;i<FILTER_NUM-1;i++)
		{
			Accbuffer_x[FILTER_NUM-1-i]=Accbuffer_x[FILTER_NUM-2-i];
			Accbuffer_y[FILTER_NUM-1-i]=Accbuffer_y[FILTER_NUM-2-i];
			Accbuffer_z[FILTER_NUM-1-i]=Accbuffer_z[FILTER_NUM-2-i];
		}

	GYR_F.X = GYR.X * Gyr_Gain;
	GYR_F.Y = GYR.Y * Gyr_Gain;
	GYR_F.Z = GYR.Z * Gyr_Gain;
	GYR_F_RATE.X = GYR_RATE.X * Gyr_Gain;
	GYR_F_RATE.Y = GYR_RATE.Y * Gyr_Gain;
	GYR_F_RATE.Z = GYR_RATE.Z * Gyr_Gain;		  
}

float ADXL345_Get_Angle(float x,float y,float z,u8 dir)
{
	float temp;
 	float res=0;
	switch(dir)
	{
		case 0://与自然Z轴的角度
 			temp=sqrt((x*x+y*y))/z;
 			res=atan(temp);
 			break;
		case 1://与自然X轴的角度
 			temp=x/sqrt((y*y+z*z));
 			res=atan(temp);
 			break;
 		case 2://与自然Y轴的角度
 			temp=y/sqrt((x*x+z*z));
 			res=atan(temp);
 			break;
 	}
	return res*180/3.14;
}


 



