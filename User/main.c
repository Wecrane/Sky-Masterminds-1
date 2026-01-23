#include "myfile.h"
#include <string.h>
#include <stdio.h>

volatile uint8_t data_ready = 0;

// 系统模式变量
// 0: 待机/选择模式
// 1: 巡线模式
// 2: 蓝牙键控模式
uint8_t Sys_Mode = 0; 
float Remote_BaseSpeed = 40.0f; // 蓝牙模式基础速度

// 蓝牙处理函数
void Bluetooth_Handle(void)
{
    if (Serial_RxPacketReady)
    {
        // 调试回显：开启这个可以看到实际收到了什么，方便排查
        // 加上 [] 是为了看清有没有多余的空格
        Serial_Printf("Recv:[%s]\r\n", Serial_RxPacket);

        if (strncmp(Serial_RxPacket, "SPD:", 4) == 0)
        {
            float spd = 0;
            if (sscanf(Serial_RxPacket, "SPD:%f", &spd) == 1)
            {
                Remote_BaseSpeed = spd;
                Serial_Printf("Speed Set:%.1f\r\n", Remote_BaseSpeed);
            }
        }
        else if (strcmp(Serial_RxPacket, "FWD") == 0 || strcmp(Serial_RxPacket, "W") == 0)
        {
             Basic_Speed = (int)Remote_BaseSpeed;
             Place_Out = 0;
             Serial_Printf("FWD\r\n");
        }
        // 支持 FWD:50 格式
        else if (strncmp(Serial_RxPacket, "FWD:", 4) == 0)
        {
             int spd = 0;
            if (sscanf(Serial_RxPacket, "FWD:%d", &spd) == 1)
            {
                Basic_Speed = spd;
                Place_Out = 0;
                Serial_Printf("FWD Speed:%d\r\n", spd);
            }
        }
        else if (strcmp(Serial_RxPacket, "BWD") == 0 || strcmp(Serial_RxPacket, "S") == 0)
        {
             Basic_Speed = -(int)Remote_BaseSpeed;
             Place_Out = 0;
             Serial_Printf("BWD\r\n");
        }
        // 支持 BWD:50 格式
        else if (strncmp(Serial_RxPacket, "BWD:", 4) == 0)
        {
             int spd = 0;
            if (sscanf(Serial_RxPacket, "BWD:%d", &spd) == 1)
            {
                Basic_Speed = -spd; // 后退为负速度
                Place_Out = 0;
                Serial_Printf("BWD Speed:%d\r\n", spd);
            }
        }
        else if (strcmp(Serial_RxPacket, "TL") == 0 || strcmp(Serial_RxPacket, "A") == 0)
        {
             Basic_Speed = (int)Remote_BaseSpeed; 
             Place_Out = 150; // 左转 (根据PID差速逻辑，正值减左轮加速右轮 => 左转)
             Serial_Printf("Turn Left\r\n");
        }
        else if (strcmp(Serial_RxPacket, "TR") == 0 || strcmp(Serial_RxPacket, "D") == 0)
        {
             Basic_Speed = (int)Remote_BaseSpeed;
             Place_Out = -150; // 右转 (根据PID差速逻辑，负值加左轮减右轮 => 右转)
             Serial_Printf("Turn Right\r\n");
        }
        else if (strcmp(Serial_RxPacket, "STOP") == 0 || strcmp(Serial_RxPacket, "X") == 0)
        {
             Basic_Speed = 0;
             Place_Out = 0;
             Serial_Printf("STOP\r\n");
        }
        else if (strcmp(Serial_RxPacket, "?") == 0)
        {
             Serial_Printf("Mode:BT,Speed:%d,Turn:%d\r\n", Basic_Speed, Place_Out);
        }
        else if (strcmp(Serial_RxPacket, "HELP") == 0)
        {
             Serial_Printf("Cmds: SPD:xx, FWD, BWD, TL, TR, STOP\r\n");
        }
        
        Serial_RxPacketReady = 0;
    }
}


int main(void)
{
		Serial_Init();
		Key_Init();
		MPU6050_Init();
	
	
		OLED_Init();
		Timer_Init();
		Encoder_Init(); 
		SENSOR_GPIO_Config(); 			//巡线传感器初始化
		Motor_Init();					//电机初始化
		PWM_Init();						//占空比定时器1初始化
	
        // 初始显示
        OLED_Clear();
        OLED_ShowString(0, 0, "K1: Line Follow", OLED_8X16);
        OLED_ShowString(0, 16, "K2: Bluetooth", OLED_8X16);

	while (1)
	{		  
		if(data_ready) 	//定时器是否到了巡线周期
		{  	 
			//获取偏航角
			MPU6050_GetData();		
			data_ready = 0;
        }			
			
            Key_Num = Key_GetNum();  	    
			
            if (Key_Num == 1)
            {
                Sys_Mode = 1;
                // 进入巡线模式配置
                Start_Run_Flag = 1;
                Place_Enable = 1; // 开启PID转向环（巡线用）
                PWM_Enable = 1;
                Basic_Speed = 40; // 给个初速度尝试
                
                OLED_Clear();
                OLED_ShowString(0, 0, "Mode: LineFollow", OLED_8X16);
                OLED_ShowString(0, 16, "Running...", OLED_8X16);
            }
            else if (Key_Num == 2)
            {
                Sys_Mode = 2;
                // 进入蓝牙模式配置
                Start_Run_Flag = 0; // 关闭巡线逻辑
                Place_Enable = 0;   // 关闭PID转向环自动计算，改为手动设置 Place_Out
                PWM_Enable = 1;
                Basic_Speed = 0;    // 初始静止
                Place_Out = 0;
                
                OLED_Clear();
                OLED_ShowString(0, 0, "Mode: Bluetooth", OLED_8X16);
                OLED_ShowString(0, 16, "Waiting Cmd...", OLED_8X16);
                Serial_Printf("Bluetooth Mode Ready\r\n");
            }
            
            // 模式处理
            if (Sys_Mode == 2)
            {
                Bluetooth_Handle();
            }
            // Sys_Mode == 1: 巡线由 Control() -> Element_Process() 自动处理
            // Sys_Mode == 0: 待机
		  
			OLED_Update();	
	}
}

/************************中断***************************/
void TIM2_IRQHandler(void)
{

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{			
		Control();      //控制函数（包含 PID 计算）
		data_ready = 1; //标志位
		Key_Tick();		//按键扫描
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}







