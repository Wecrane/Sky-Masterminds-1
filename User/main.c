/***************************************************************************************
  * 开源代码：B站号-江科大胡不才开源巡线车
  * 请自行查看后使用和修改，并应用到自己的项目之中
  * 修改时间：2025 11 17 v2.0
  ***************************************************************************************
  */
  
#include "myfile.h"
#include "mode_control.h"

/************************主函数***************************/
int main(void)
{
    // 先初始化OLED，以便显示校准界面
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(8, 16, "Calibrating...", OLED_8X16);
    OLED_ShowString(20, 36, "Please Wait", OLED_8X16);
    OLED_Update();
    
    // 然后初始化MPU6050（里面有2秒延时用于稳定）
    MPU6050_Init();
    
    // 其他硬件初始化（定时器最后初始化，避免中断干扰校准）
    Serial_Init();
    Key_Init();        
    Encoder_Init(); 
    SENSOR_GPIO_Config();  // 循线引脚初始化
    Motor_Init();          // 电机初始化
    PWM_Init();            // 占空比定时器1初始化
    
    // 初始化模式控制
    Mode_Init();
    System_Mode = MODE_CALIBRATING;
    
    // 执行陀螺仪校准（必须在Timer_Init之前，避免中断干扰I2C通信）
    calibrate_gyro();
    
    // 校准完成后再启动定时器
    Timer_Init();
    
    // 切换到菜单
    System_Mode = MODE_MENU;
    OLED_Clear();
    
    while (1)
    {        
        // 模式处理
        Mode_Process();
        
        // 调试输出（可选）
        // Serial_Printf("%d,%f,%d,%f\r\n", sensor_err, final_err, Place_Out, 0);
        
        OLED_Update();    
    }
}

/************************定时器中断***************************/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        // 按键扫描
        Key_Tick();
        
        // 仅在巡线模式下执行完整控制
        if (System_Mode == MODE_LINE_FOLLOW)
        {
            Control();
        }
        // 蓝牙模式执行速度环控制
        else if (System_Mode == MODE_BLUETOOTH)
        {
            Encoder_Read();
            Speed_Out_L = SpeedPID_Compute(&speedPID_L, Left_Speed, Speed_L); 
            Speed_Out_R = SpeedPID_Compute(&speedPID_R, Right_Speed, Speed_R);
            Speed_Out_L = Min_Max(Speed_Out_L, -Max_PWM, Max_PWM);
            Speed_Out_R = Min_Max(Speed_Out_R, -Max_PWM, Max_PWM);
            Motor_SetPWM_L(Speed_Out_L);
            Motor_SetPWM_R(Speed_Out_R);
        }
        
        // 更新陀螺仪（所有模式下都更新yaw）
        update_yaw();
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}







