#include "control.h"
#include "sys.h"
#include "define.h"
#include "timer.h"
#include "led.h"
struct DATA_XYZ_F GYR_F={0.0,0.0,0.0};
struct DATA_XYZ_F GYR_F_RATE={0.0,0.0,0.0};
struct DATA_XYZ_F GYR_F_AVG={0.0,0.0,0.0};
struct DATA_XYZ_F Current_ERROR={0.0,0.0,0.0};

struct DATA_XYZ ACC={0,0,0};
struct DATA_XYZ GYR_CTR={0,0,0};
//struct DATA_XYZ DATA[100]={0,0,0};
struct DATA_XYZ GYR={0,0,0};
struct DATA_XYZ GYR_RATE={0,0,0};
struct DATA_XYZ HMC={0,0,0};
struct DATA_XYZ ACC_OFFSET={0,0,0};
struct DATA_XYZ GYR_OFFSET={0,0,0};
struct DATA_XYZ ACC_AVG={0,0,0}; 
extern struct DATA_XYZ_F gyr_offset_f;
extern u8 wireless;
struct PID PID_DATA;
struct PID YAW_PID;
struct PID EXP_PID;
struct DATA_16 COMD={0,0,0,0};
float Pitch=0.0,Rool=0.0,Yaw=0.0;
float rool=0,pitch=0;
float EXP_YAW=0;
u8 ct=0;
u8 FLY_EN=0;
u8 ThreeD_EN=0;
u16  ThreeD_TMS=0;
u8 Motion=0,Last_Motion=0; 
float Dynamic_PID_rate=0.0;
float D2=0.0;
struct DATA_XYZ_F EXP_THTA={0.0,0.0,0.0};

float Exp_rate=0.005;   
#define MAX_ANGLE 15
float t1=0.0,t2=0.0,t3=0.0;
float NOW_GO_BACK_X=0.0,NOW_GO_BACK_Y=0.0;
float LAST_GO_BACK_X=0.0,LAST_GO_BACK_Y=0.0;
float GO_BACK_X_VAL=0.0,GO_BACK_Y_VAL=0.0;
u16 GO_BACK_X_CT=0,GO_BACK_Y_CT=0;
u8  GO_BACK_X_STR=0,GO_BACK_Y_STR=0;
u8 flagtwo=0;

void PID_INIT(void)	   
{
	PID_DATA.P = 2.7;	//2.682	   
	PID_DATA.I = 1;   //0.2
	PID_DATA.D = 0.7;	//0.521	 
	PID_DATA.SUM_ERROR_R=0.0;
	PID_DATA.SUM_ERROR_P=0.0;
	PID_DATA.IMAX = 300;

	YAW_PID.P = 0;	//7.5   	 7.5
	YAW_PID.I = 0;   //0.2	     0
	YAW_PID.D = 0;	//1.8	 	 1.8
	YAW_PID.SUM_ERROR_R=0.0;
	YAW_PID.SUM_ERROR_P=0.0;
	YAW_PID.IMAX = 100;

	EXP_PID.P = 1;	    
	EXP_PID.I = 0;    
	EXP_PID.D = 1.8; 	 
	EXP_PID.SUM_ERROR_R=0.0;
	EXP_PID.SUM_ERROR_P=0.0;
	EXP_PID.IMAX = 100;
}

float EXP_BUF_x[50]={0};
float EXP_BUF_y[50]={0};
float avg_exp_x=0,avg_exp_y=0;
void EXP_SMOOTH(u8 tms)
{
	static u8 cnt=0; 
	u8 i=0;
	float temp1=0,temp2=0;
	EXP_BUF_x[cnt] = EXP_THTA.X;//���»�����������
	EXP_BUF_y[cnt] = EXP_THTA.Y;//���»�����������
	for(i=0;i<tms;i++)
	{
		temp1 += EXP_BUF_x[i]; 
		temp2 += EXP_BUF_y[i]; 
	}
	avg_exp_x = temp1 /(float)tms; 
	avg_exp_y = temp2 /(float)tms;
	cnt++;
	if(cnt==tms)	cnt=0;	 		  
}


float last_exp_x=0.0,last_exp_y=0.0;
float now_exp_x=0.0,now_exp_y=0.0;
float turn=0;
int tt1=0,tt2=0;
int maxx=30;
int goback=0;
float yaw_error=0,last_yaw_error=0.0;
float yaw_p=1,yaw_d=0.5;	  //5,  3.065
float exp_yaw=0.0;
float yaw_tmp=0.0;
float turn_rate=0.0008;
 extern  float  tempa1,tempa2,tempa3,tempa4;
void READ_CONTROL_COMMAND(void)			//��ȡҡ�������źŷ���
{
		COMD.THR=tempa3-1040;
   	if(COMD.THR>650) COMD.THR=650;
 	
	if(COMD.THR>30)
	{
		FLY_EN=1;
	}
	else
	{	
		FLY_EN=0;
	
	}
	COMD.LR=-(tempa1-1390); // 
	COMD.FB=tempa2-1440; //
	COMD.TURN= (tempa4)-1403; //	1936
	if(COMD.TURN<5&&COMD.TURN>-5)  COMD.TURN=0;
     turn=COMD.TURN*turn_rate;
	exp_yaw +=turn;	 /////////////////////////////////////
	if(exp_yaw>360) exp_yaw=0;
	if(exp_yaw<0) exp_yaw=360;


 	if(COMD.FB<5&&COMD.FB>-5)	
		COMD.FB=0 ;
	else
		Motion=1;
	if(COMD.LR<5&&COMD.LR>-5) 
		COMD.LR=0;
	else
		Motion=1;
	if(COMD.FB==0&&COMD.LR==0)
		Motion=0;	 
	now_exp_x=COMD.FB*Exp_rate;
	now_exp_y=-COMD.LR*Exp_rate;
    
 	EXP_THTA.X= now_exp_x;
	EXP_THTA.Y= now_exp_y; 
	EXP_SMOOTH(6);
	if(avg_exp_x> MAX_ANGLE)	avg_exp_x= MAX_ANGLE;
	if(avg_exp_x<-MAX_ANGLE)	avg_exp_x=-MAX_ANGLE;	
	if(avg_exp_y> MAX_ANGLE)	avg_exp_y= MAX_ANGLE;
	if(avg_exp_y<-MAX_ANGLE)	avg_exp_y=-MAX_ANGLE;		   // �����Ƕ� �޷�	
/*******************************-**************************************/
	Current_ERROR.X =   avg_exp_x- Rool;	
	Current_ERROR.Y =   avg_exp_y- Pitch;

	last_exp_x=now_exp_x;
	last_exp_y=now_exp_y;

}


float last_gyr_x=0.0,last_gyr_y=0.0;
float sum_gyr_x=0.0,sum_gyr_y=0.0;
int Motor1=0,Motor2=0,Motor3=0,Motor4=0; 	
int yaw=0; 	
float yaw_dt=0;

float RI_MAX=0,PI_MAX=0;
 
void STABLE_WITH_PID(void)
{
	Last_Motion=Motion;
////////////////////////////////////////////////////////////////////////////////PID����
   	RI_MAX=PID_DATA.IMAX  ;
 	rool = PID_DATA.P * Current_ERROR.X;

	if(Current_ERROR.X<60&&Current_ERROR.X>-60)
		PID_DATA.SUM_ERROR_R+= PID_DATA.I*Current_ERROR.X/400.0 ;
	else
		PID_DATA.SUM_ERROR_R=0;

	if(PID_DATA.SUM_ERROR_R> RI_MAX)	PID_DATA.SUM_ERROR_R= RI_MAX;
	if(PID_DATA.SUM_ERROR_R<-RI_MAX)	PID_DATA.SUM_ERROR_R=-RI_MAX;
 
    rool+=	PID_DATA.SUM_ERROR_R;	  // ���ϻ���
 
   rool  +=	 - (PID_DATA.D * (GYR_F_RATE.X-gyr_offset_f.X))  ;
 
////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////// 

	PI_MAX=PID_DATA.IMAX;
	pitch = PID_DATA.P * Current_ERROR.Y ;

	if(Current_ERROR.Y<60&&Current_ERROR.Y>-60)
 		PID_DATA.SUM_ERROR_P+= PID_DATA.I*Current_ERROR.Y/400.0 ;
 	else
 		PID_DATA.SUM_ERROR_P=0;
	if(PID_DATA.SUM_ERROR_P> PI_MAX)	PID_DATA.SUM_ERROR_P= PI_MAX;
	if(PID_DATA.SUM_ERROR_P<-PI_MAX)	PID_DATA.SUM_ERROR_P=-PI_MAX;
   
 	pitch +=  PID_DATA.SUM_ERROR_P;	  // ���ϻ���  
 
	pitch +=	 - (PID_DATA.D * (GYR_F_RATE.Y-gyr_offset_f.Y)) ; 
/////////////
	yaw_tmp= exp_yaw-Yaw;
	if(yaw_tmp>-180)
	{
		if(yaw_tmp>180)
			yaw_error=( exp_yaw-Yaw-360);
		else
			yaw_error= exp_yaw-Yaw;
	}
	else
		yaw_error= (360+exp_yaw-Yaw);
	yaw =-yaw_p*yaw_error- yaw_d*GYR_F.Z;

// 	yaw = -5 * GYR_F.Z;
//	yaw-=COMD.TURN*0.47;
//	yaw=0;	
////////////////////////////////////////////////////////////////////////////////�����ֵ�ںϵ��ĸ����
	Motor1=(int)(COMD.THR + rool- pitch - yaw )+1000; //1///////////rool��////pitch����///////yaw��//////////////////////////////////////////////////////
	Motor2=(int)  (COMD.THR + rool + pitch + yaw)+1000;	//1//
	Motor3=(int)(COMD.THR - rool + pitch - yaw)+1000; //1//
	Motor4=(int)(COMD.THR - rool- pitch + yaw)+1000;///	 
	Limit_amplitude(&Motor1,&Motor2,&Motor3,&Motor4);
  //FLY_EN=0;
	if(FLY_EN==1&&COMD.THR>25)  
	{
	    Refresh_Or_STOP_MOTOR(1);
		ThreeD_EN=0;			 
	 	ThreeD_TMS=0;
	}
		else
	  { Refresh_Or_STOP_MOTOR(2); } 
 
   if((Pitch>50||Pitch<(-50))||(Rool>50||Rool<(-50)))flagtwo=1;
	if(flagtwo==1||(wireless==0))
	 {
	 Refresh_Or_STOP_MOTOR(2); 	LED_GRE_ON;
      }
	
}

void CRASH_LANDING()
{
	COMD.THR=  330;
	FLY_EN=1;
	COMD.LR=	0;
	COMD.FB=	0;
	COMD.TURN=  0;  
	Current_ERROR.X=0;
	Current_ERROR.Y=0;
	PID_DATA.SUM_ERROR_R=0;
	PID_DATA.SUM_ERROR_P=0; 	
}

void Limit_amplitude(int*m1,int* m2,int* m3,int* m4) 
{
	if(*m1>1600) *m1=1600;
	else if(*m1<0) *m1=0;
	if(*m2>1600) *m2=1600;
	else if(*m2<0) *m2=0;
	if(*m3>1600) *m3=1600;
	else if(*m3<0) *m3=0;
	if(*m4>1600) *m4=1600;
	else if(*m4<0) *m4=0;
}
void Refresh_Or_STOP_MOTOR(u8 sel)
{
	if(sel==1)
	{  
	
     	PWM_CH1=Motor4;	  //2
		PWM_CH2=Motor1;	  //3
		PWM_CH3=Motor2;	  //4
		PWM_CH4=Motor3;	  //1
	}
	else if(sel==2)	
	{  rool=0; pitch=0; yaw=0;
	  // Rool=0;Pitch=0;Yaw=0;
	   	
		PWM_CH1=1000;
		PWM_CH2=1000;
		PWM_CH3=1000;
		PWM_CH4=1000;		
	}
}


