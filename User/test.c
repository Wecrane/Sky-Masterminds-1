#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"


/************************模块测试文件***************************/
///*IIC测试*/
//int main(void)
//{
//	OLED_Init();
//	MyI2C_Init();
//	MyI2C_Start();
//	MyI2C_SendByte(0xA0);
//	uint8_t Ack=MyI2C_ReceiveAck();
//	MyI2C_Stop();
//	OLED_ShowNum(0, 0, Ack, 3, OLED_8X16);
//	OLED_Update();
//	while (1)
//	{

//	}
//}



///*mpu6050测试*/

//uint8_t ID;
//int16_t AX, AY, AZ, GX, GY, GZ;

//int main(void)
//{
//	OLED_Init();
//	MPU6050_Init();
//	
//	OLED_ShowString(1, 1, "ID:",OLED_8X16);
//	ID = MPU6050_GetID();
//	OLED_ShowHexNum(1, 4, ID, 2,OLED_8X16);
//	
//	while (1)
//	{
//		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY,&GZ);
//		
//    		float Filter_pitch = Kalman_Cal_pitch(AX, AY, AZ, GX);
//		
//				OLED_ShowFloatNum(1, 1, pitch,2,2,OLED_8X16);
//  			OLED_ShowFloatNum(1, 17, roll,2,2,OLED_8X16);
//				OLED_ShowFloatNum(1, 32, Filter_pitch,3,3,OLED_8X16);
//		//角加速度
////			OLED_ShowSignedNum(1, 1, AX,5,OLED_8X16);
////  		OLED_ShowSignedNum(1, 17, AY, 5,OLED_8X16);
////			OLED_ShowSignedNum(1, 32, AZ, 5,OLED_8X16);
////		//角速度
////			OLED_ShowSignedNum(1, 48, GX, 5,OLED_8X16);
////			OLED_ShowSignedNum(60, 17,GY, 5,OLED_8X16);
////			OLED_ShowSignedNum(60, 1, GZ, 5,OLED_8X16);
//			OLED_Update();
//	}
//}



///*电机测试*/
//int main(void)
//{
//	/*OLED初始化*/
//	OLED_Init();
//	PWM_Init();
//	Motor_Init();
//	int PWM=1000;
//	while (1)
//	{
//		
//			Motor_SetPWM(PWM);
//			OLED_Update();
//			
//		
//	}
//}


/*定时器2测试*/
//int main(void)
//{
//	/*OLED初始化*/
//	OLED_Init();
//	Timer_Init();
//	while (1)
//	{
//	
//		
//		
//	}
//}
//void TIM2_IRQHandler(void)
//{
//	
//	static int Count = 0;  // 静态变量，保持上次中断时的值
//	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
//	{
//		Count++;
//		OLED_ShowSignedNum(1, 1, Count, 5,OLED_8X16);
//		OLED_Update();
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//	}
//}

/*编码器测试*/

//int main(void)
//{
//		OLED_Init();
//		Timer_Init();
//		Encoder_Init(); 
//		Speed_L=0;
//		Speed_R=0;
//	while (1)
//	
//	{
//		OLED_Printf(0,0,OLED_8X16,"Speed:%+05d",Speed_L);
//		OLED_Printf(0,17,OLED_8X16,"Speed:%+05d",Speed_R);
//		
//		OLED_Update();
//			
//	}
//}


//void TIM3_IRQHandler(void)
//{
//	static uint16_t count;
//	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
//	{	
//		count++;		
//		if(count>=50)			
//		{
//			Speed_L=Encoder_Get_L();
//			Speed_R=Encoder_Get_R();
//		}
//		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//	}
//}



///*灰度传感器测试*/
//int i1,i2,i3,i4,i5,err=0;
//int main(void)
//{
//	/*OLED初始化*/
//	OLED_Init();
//	SENSOR_GPIO_Config();
//	while (1)
//	{		
//		if(L2==0){i1=0;}
//		else{i1=50;}
//		if(L1==0){i2=0;}
//		else{i2=25;}
//		if(M==0) {i3=0;}
//		else{i3=0;}
//		if(R1==0){i4=0;}
//		else{i4=-25;}
//		if(R2==0){i5=0;}	
//		else{i5=-50;}
//		err=i1+i2+i3+i4+i5;		
//		OLED_Printf(0,0,OLED_8X16,"I1:%+02d",i1);
//		OLED_Printf(0,17,OLED_8X16,"I2:%+02d",i2);
//		OLED_Printf(0,34,OLED_8X16,"I3:%+02d",i3);
//		OLED_Printf(0,51,OLED_8X16,"I4:%+02d",i4);
//		OLED_Printf(66,0,OLED_8X16,"I5:%+02d",i5);
//		OLED_Printf(66,17,OLED_8X16,"err:%+03d",err);
//		OLED_Update();
//	}
//}

/*速度环测试*/
//double Bias_PID[2] = {0, 0}; 
//int main(void)
//{
//		OLED_Init();
//		Timer_Init();
//		PWM_Init();
//		Encoder_Init(); 
//		Speed_L=0;
//		Speed_R=0;
//	  Motor_Init();
//	  double PWM_L=0;
//		double PWM_R=0;
//	while (1)
//	
//	{
//		Motor_SetPWM(PWM);
//		OLED_Printf(0,0,OLED_8X16,"Speed:%+05d",Speed_L);
//		OLED_Printf(0,17,OLED_8X16,"Speed:%+05d",Speed_R);	
//		OLED_Update();
//			
//	}
//}

//void TIM2_IRQHandler(void)
//{
//	if (TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
//	{	
//		{
//			Speed_L=Encoder_Get_L();
//			Speed_R=Encoder_Get_R();
//			PWM_L=PID_Control(Speed_L,100,Bias_PID);
//			PWM_R=PID_Control(Speed_R,100,Bias_PID);
//			Motor_SetPWM_L(PWM_L);
//			Motor_SetPWM_R(PWM_R);
//		}
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//	}
//}

//double PID_Control(double NowPiont, double SetPoint, double *TURN_PID) //PD控制
//{
//	double KP, KD; 
//	double NowError, LastError,Out; 
//	NowError = SetPoint - NowPiont; // 当前误差 
//	KP = *TURN_PID; 
//	KD = *TURN_PID+1; 
//	Out = KP * NowError + KD *(NowError-LastError); // PID 输出值 
//	LastError = NowError; //更新误差 
//	return Out; 
//}



///*转向环误差测试*/
//double Dir_PID[2] = {0, 0}; 
//int main(void)
//{
//		OLED_Init();
//		Timer_Init();
//	while (1)
//	
//	{

//		OLED_Printf(0,0,OLED_8X16,"Speed:%+05d",Speed_L);
//		OLED_Printf(0,17,OLED_8X16,"Speed:%+05d",Speed_R);	
//		OLED_Update();
//			
//	}
//}

//double PID_Control(double NowPiont, double SetPoint, double *TURN_PID) //PD控制
//{
//	double KP, KD; 
//	double NowError, LastError,Out; 
//	NowError = SetPoint - NowPiont; // 当前误差 
//	KP = *TURN_PID; 
//	KD = *TURN_PID+1; 
//	Out = KP * NowError + KD *(NowError-LastError); // PID 输出值 
//	LastError = NowError; //更新误差 
//	return Out; 
//}


///*串口测试*/
//uint16_t RP1, RP2, RP3, RP4;

//int main(void)
//{
//	OLED_Init();
//	RP_Init();
//	Serial_Init();
//	
//	while (1)
//	{
//		RP1 = RP_GetValue(1);
//		RP2 = RP_GetValue(2);
//		RP3 = RP_GetValue(3);
//		RP4 = RP_GetValue(4);
//		
//		OLED_Printf(0, 0, OLED_8X16, "RP1:%04d", RP1);
//		OLED_Printf(0, 16, OLED_8X16, "RP2:%04d", RP2);
//		OLED_Printf(0, 32, OLED_8X16, "RP3:%04d", RP3);
//		OLED_Printf(0, 48, OLED_8X16, "RP4:%04d", RP4);
//		
//		OLED_Update();
//		
//		Serial_Printf("%d,%d,%d,%d\r\n", RP1, RP2, RP3, RP4);
//		
//		Delay_ms(10);
//	}
//}

///*串口测试*/
//int main(void)
//{
//	OLED_Init();
//	Serial_Init();
//	Speed_L=0;
//	Speed_R=0;
//	Encoder_Init(); 
//	while (1)
//	{
//			
//		OLED_Printf(0, 0, OLED_8X16, "Speed_L:%04d", Speed_L);
//		OLED_Printf(0, 16, OLED_8X16, "Speed_R:%04d", Speed_R);		
//		OLED_Update();
//		Speed_L=Encoder_Get_L();
//		Speed_R=-Encoder_Get_R();
//		
//		Serial_Printf("%d,%d\r\n", Speed_L, Speed_R);
//		
//		Delay_ms(10);
//	}
//}




