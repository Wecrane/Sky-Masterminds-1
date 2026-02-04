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
        char cmd[100];
        char displayCmd[16] = {0}; // 用于OLED显示
        int val = 0;
        
        // 复制并保证字符串安全
        strncpy(cmd, (const char *)Serial_RxPacket, sizeof(cmd) - 1);
        cmd[sizeof(cmd) - 1] = '\0';
        
        // 简单转大写 (只转前几个关键字符)
        for(int i=0; cmd[i] && i<10; i++) {
            if(cmd[i] >= 'a' && cmd[i] <= 'z') cmd[i] -= 32;
        }

        Serial_Printf("Recv:[%s]\r\n", cmd);

        // --- 解析指令 ---

        // 1. 设置基础速度 SPD:xx
        if (strncmp(cmd, "SPD:", 4) == 0)
        {
            float spd = 0;
            if (sscanf(cmd + 4, "%f", &spd) == 1)
            {
                Remote_BaseSpeed = spd;
                Serial_Printf("BaseSpeed Set:%.1f\r\n", Remote_BaseSpeed);
                sprintf(displayCmd, "SPD:%.0f", spd);
            }
        }
        // 2. 前进 FWD 或 FWD:xx
        else if (strncmp(cmd, "FWD", 3) == 0 || strcmp(cmd, "W") == 0)
        {
            int target_spd = (int)Remote_BaseSpeed;
            if (cmd[3] == ':' && sscanf(cmd + 4, "%d", &val) == 1) {
                target_spd = val;
            }
            
            Basic_Speed = target_spd;
            Place_Out = 0;
            Serial_Printf("FWD Speed:%d\r\n", target_spd);
            sprintf(displayCmd, "FWD:%d", target_spd);
        }
        // 3. 后退 BWD 或 BWD:xx
        else if (strncmp(cmd, "BWD", 3) == 0 || strcmp(cmd, "S") == 0)
        {
            int target_spd = (int)Remote_BaseSpeed;
            if (cmd[3] == ':' && sscanf(cmd + 4, "%d", &val) == 1) {
                target_spd = val;
            }

            Basic_Speed = -target_spd;
            Place_Out = 0;
            Serial_Printf("BWD Speed:%d\r\n", target_spd);
            sprintf(displayCmd, "BWD:%d", target_spd);
        }
        // 4. 左转 TL 或 TL:xx (Turn Left)
        else if (strncmp(cmd, "TL", 2) == 0 || strcmp(cmd, "A") == 0)
        {
            int turn_val = 150; // 默认转向力度
            if (cmd[2] == ':' && sscanf(cmd + 3, "%d", &val) == 1) {
                turn_val = val; 
            }
            
            // 转向逻辑：保持基础速度，施加旋转分量
            Basic_Speed = (Basic_Speed == 0) ? (int)Remote_BaseSpeed : Basic_Speed;
            Place_Out = turn_val; 
            Serial_Printf("TurnL Val:%d\r\n", turn_val);
            sprintf(displayCmd, "TL:%d", turn_val);
        }
        // 5. 右转 TR 或 TR:xx (Turn Right)
        else if (strncmp(cmd, "TR", 2) == 0 || strcmp(cmd, "D") == 0)
        {
            int turn_val = 150; // 默认转向力度
            if (cmd[2] == ':' && sscanf(cmd + 3, "%d", &val) == 1) {
                turn_val = val;
            }
            
            Basic_Speed = (Basic_Speed == 0) ? (int)Remote_BaseSpeed : Basic_Speed; 
            Place_Out = -turn_val;
            Serial_Printf("TurnR Val:%d\r\n", turn_val);
            sprintf(displayCmd, "TR:%d", turn_val);
        }
        // 6. 停止 STOP
        else if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "X") == 0)
        {
             Basic_Speed = 0;
             Place_Out = 0;
             Serial_Printf("STOP\r\n");
             sprintf(displayCmd, "STOP");
        }
        else if (strcmp(cmd, "HELP") == 0)
        {
             Serial_Printf("CMD: FWD[:v], BWD[:v], TL[:v], TR[:v], SPD:v, STOP\r\n");
             sprintf(displayCmd, "HELP");
        }
        else
        {
            sprintf(displayCmd, "Unk:%.5s", cmd);
        }
        
        // --- 刷新OLED显示 ---
        // 清除第3行 (Y=32) 和 第4行 (Y=48)
        OLED_ClearArea(0, 32, 128, 32); 
        
        OLED_ShowString(0, 32, "Cmd:", OLED_8X16);
        OLED_ShowString(32, 32, displayCmd, OLED_8X16);
        
        // 显示当前状态：速度和转向值
        char status[20];
        sprintf(status, "V:%d T:%d", Basic_Speed, Place_Out);
        OLED_ShowString(0, 48, status, OLED_8X16);

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







