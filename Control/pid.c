#include "myfile.h"
/************************全变量***************************/
uint8_t Place_Enable,PWM_Enable=1;//转向环和速度环总使能开关
int Speed_Out_L,Speed_Out_R,Place_Out=0;//
int sensor_err,final_err=0;
float straight_err;

/************************PID参数***************************/
int Basic_Speed=0;    //基础速度
float Turn_factor=0.5; 
int Left_Speed,Right_Speed=0;
int Speed_PID[3] = {250,0,30}; 


float Place_PD[2] = {5.3, 4.3};

void Control()
{
			// 获取传感器加权误差
			sensor_err = Error_Calcaulate(); 
	
			// 融合陀螺仪（目前仅透传）
			final_err = get_fused_error(sensor_err,GZ);
	
			// 转向PD计算
			if(Place_Enable)
			{
				Place_Out=(int)Place_Control(final_err,0,Place_PD);
			}
			
			// 元素处理逻辑
			Element_Process();
			// 读取编码器
			Encoder_Read();
			
			// 计算左右轮目标速度 (叠加转向差速)
			Different_Speed();
			
			// 速度环PID计算
			Speed_Out_L=PID_Control_L(Speed_L,Left_Speed,Speed_PID); 
			Speed_Out_R=PID_Control_R(Speed_R,Right_Speed,Speed_PID);
	
			// 限幅
			Speed_Out_L=Min_Max( Speed_Out_L ,-Max_PWM, Max_PWM );
			Speed_Out_R=Min_Max( Speed_Out_R ,-Max_PWM, Max_PWM );
			
			// 输出PWM
			if(PWM_Enable)
  		{
				Motor_SetPWM_L(Speed_Out_L);
				Motor_SetPWM_R(Speed_Out_R);
			}
			 
}


/************************位置式转向PD***************************/
float Place_Control(float NowPoint, float SetPoint, float *TURN_PID) 
{
	static float LastError = 0; 
	float KP, KD; 
	float NowError, Out; 
	NowError = SetPoint - NowPoint; 
	KP = *TURN_PID; 
	KD = *(TURN_PID+1); 
	Out = KP * NowError + KD *(NowError-LastError);
	LastError = NowError; 
	return Out; 
}

/************************位置式速度环L (增量式更常用，但这里是位置式)***************************/
int PID_Control_L(int NowPoint, int SetPoint, int *TURN_PID) 
{
	  static int Integral = 0, LastError = 0; 
		int KP,KI,KD,Out,NowError; 
		KP = *TURN_PID; 
		KI = *(TURN_PID+1); 
		KD = *(TURN_PID+2); 
	
    NowError = SetPoint - NowPoint;
    
    Out = KP * NowError + KI * Integral+KD *(NowError-LastError);
    LastError = NowError; 
    return Out;
}

/************************位置式速度环R***************************/
int PID_Control_R(int NowPoint, int SetPoint, int *TURN_PID) 
{
	  static int Integral = 0, LastError = 0; 
		int KP,KI,KD,Out,NowError; 
		KP = *TURN_PID; 
		KI = *(TURN_PID+1); 
		KD = *(TURN_PID+2); 
	
    NowError = SetPoint - NowPoint;
    
    Out = KP * NowError + KI * Integral + KD *(NowError-LastError);
    LastError = NowError;
    return Out;
}

/************************差速分配***************************/
void Different_Speed() 
{ 
 float k;  
 // 根据PID计算出的转向输出 Place_Out，分配给左右轮
 // 系数 0.01 是为了将 Place_Out 归一化到 k 比例
 if(Place_Out >= 0) 
 {
     // 需要左转：减左轮，加右轮
     k = Place_Out * 0.01f; 
     Left_Speed = Basic_Speed * (1 - k); 
     Right_Speed = Basic_Speed * (1 + k*Turn_factor); 
 } 
 else 
 { 
     // 需要右转：减右轮，加左轮
     k = -Place_Out * 0.01f; 
     Left_Speed = Basic_Speed * (1 + k*Turn_factor); 
     Right_Speed = Basic_Speed * (1 - k); 
  } 
}
