#include "myfile.h"
/************************全局变量***************************/
uint8_t Place_Enable,PWM_Enable=1;//（转向环和速度环的使能开关）
int Speed_Out_L,Speed_Out_R,Place_Out=0;//两个环的输出
int sensor_err,final_err=0;
float straight_err;

/************************PID调节区***************************/
int Basic_Speed=0;    //基础速度，在这里修改速度，但是元素要先注释掉
float Turn_factor=0.5;
int Left_Speed,Right_Speed=0;
int Speed_PID[3] = {250,0,30}; 
float Place_PD[2] = {5.3,4.3};

void Control()
{
			// 获取传感器加权误差（原有算法）
			sensor_err = Error_Calcaulate(); 
	
			// 计算融合误差	
			final_err = get_fused_error(sensor_err,GZ);
	
			//转向环
			if(Place_Enable)
			{
				Place_Out=(int)Place_Control(final_err,0,Place_PD);
			}
			
			//元素处理，一开始调车的时候先关掉
			Element_Process();
			//获取编码器值
			Encoder_Read();
			
			//转向处理
			Different_Speed();
			
			//速度环
			Speed_Out_L=PID_Control_L(Speed_L,Left_Speed,Speed_PID); 
			Speed_Out_R=PID_Control_R(Speed_R,Right_Speed,Speed_PID);
	
			//输出限幅
			Speed_Out_L=Min_Max( Speed_Out_L ,-Max_PWM, Max_PWM );
			Speed_Out_R=Min_Max( Speed_Out_R ,-Max_PWM, Max_PWM );
			
			//将pwm输出到电机上
			if(PWM_Enable)
  		{
				Motor_SetPWM_L(Speed_Out_L);
				Motor_SetPWM_R(Speed_Out_R);
//			Motor_SetPWM_L(2000);
//			Motor_SetPWM_R(2000);
			}
			 
}


/************************位置式转向环pd***************************/
float Place_Control(float NowPoint, float SetPoint, float *TURN_PID) //PD控制位置环
{
	static float LastError = 0; // 静态变量保持历史值
	float KP, KD; 
	float NowError, Out; 
	NowError = SetPoint - NowPoint; // 当前误差 
	KP = *TURN_PID; 
	KD = *(TURN_PID+1); 
	Out = KP * NowError + KD *(NowError-LastError); // PID 输出值 
	LastError = NowError; //更新误差 
	return Out; 
}

/************************位置式速度环pd (左轮)***************************/
int PID_Control_L(int NowPoint, int SetPoint, int *TURN_PID) 
{
	  static int Integral = 0, LastError = 0; 
		int KP,KI,KD,Out,NowError; 
		KP = *TURN_PID; 
		KI = *(TURN_PID+1); 
		KD = *(TURN_PID+2); 
	
    NowError = SetPoint - NowPoint;
    
		//这里pi、pd我都试过了pd效果更好一点
    Out = KP * NowError + KI * Integral+KD *(NowError-LastError);
    LastError = NowError; // 更新误差 (原来代码缺了这行？不，之前的代码里好像也没有更新LastError？)
    return Out;
}

/************************位置式速度环pd (右轮)***************************/
int PID_Control_R(int NowPoint, int SetPoint, int *TURN_PID) 
{
	  static int Integral = 0, LastError = 0; 
		int KP,KI,KD,Out,NowError; 
		KP = *TURN_PID; 
		KI = *(TURN_PID+1); 
		KD = *(TURN_PID+2); 
	
    NowError = SetPoint - NowPoint;
    
		//这里pi、pd我都试过了pd效果更好一点
    Out = KP * NowError + KI * Integral+KD *(NowError-LastError);
    LastError = NowError;
    return Out;
}

/************************差速计算***************************/
void Different_Speed() 
{ 
 float k;  
 //转向环的输出
 //输出的速度取决于基础速度的大小，所以速度快了之后转向力度也会跟着变大
 if(Place_Out >= 0) 
	 {
		 k = Place_Out * 0.01; 
		 Left_Speed = Basic_Speed * (1 - k); 
		 Right_Speed = Basic_Speed * (1 + k*Turn_factor); 
	 } 
	 else 
	{ 
	 k = -Place_Out * 0.01; 
	 Left_Speed = Basic_Speed * (1 + k*Turn_factor); 
	 Right_Speed = Basic_Speed * (1 - k); 
  } 
}
