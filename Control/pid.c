#include "myfile.h"

/*=============================================================================
 * 控制频率: 100Hz (10ms周期)
 * 定时器配置: TIM2, 72MHz / 7200 / 100 = 100Hz
 *===========================================================================*/

uint8_t Place_Enable = 1, PWM_Enable = 1;
uint8_t Angle_Enable = 0;
int Speed_Out_L, Speed_Out_R, Place_Out, Angle_out = 0;
float sensor_err = 0;
float final_err = 0;

int Basic_Speed = 0;
int Left_Speed, Right_Speed = 0;

/*=============================================================================
 * PID控制器实例 - 每个控制环独立
 * 速度环: 左右轮各一个独立PID，避免静态变量互相干扰
 * 位置环: 用于巡线转向控制
 * 角度环: 用于陀螺仪角度闭环
 *===========================================================================*/
SpeedPID_t speedPID_L = {200, 1.0, 100, 0, 0};  // 左轮速度环
SpeedPID_t speedPID_R = {200, 1.0, 100, 0, 0};  // 右轮速度环
PlacePID_t placePID   = {35.0f, 0.0f, 1.0f, 0.10f, 0};  // 位置环(巡线) Kp降低
PlacePID_t anglePID   = {4.0f, 0.0f, 2.0f, 0.0f, 0};    // 角度环(陀螺仪)

/*=============================================================================
 * 主控制函数 - 在TIM2中断中以100Hz调用
 *===========================================================================*/
void Control(void)
{	
    // update_yaw() 已在中断中单独调用，保证所有模式都能更新yaw
    
    sensor_err = Error_Calcaulate(); 
    final_err = get_fused_error(sensor_err, gyro_z);			
    
    // 位置环 - 巡线转向
    if (Place_Enable)
    {
        Place_Out = (int)PlacePID_Compute(&placePID, 0, final_err, gyro_z);
    }
            
    // 角度环 - 陀螺仪闭环
    if (Angle_Enable)
    {
        Angle_out = (int)PlacePID_Compute(&anglePID, 0, yaw, gyro_z);
        Angle_out = Min_Max(Angle_out, -50, 50);
    }
    
    Element_Process();
    Encoder_Read();
    Different_Speed();		

    // 速度环 - 左右轮独立PID
    Speed_Out_L = SpeedPID_Compute(&speedPID_L, Left_Speed, Speed_L); 
    Speed_Out_R = SpeedPID_Compute(&speedPID_R, Right_Speed, Speed_R);
    
    Speed_Out_L = Min_Max(Speed_Out_L, -Max_PWM, Max_PWM);
    Speed_Out_R = Min_Max(Speed_Out_R, -Max_PWM, Max_PWM);
    
    if (PWM_Enable)
    {
        Motor_SetPWM_L(Speed_Out_L);
        Motor_SetPWM_R(Speed_Out_R);
    }
    else
    {
        // PWM禁用时主动停止电机，解决丢线后不停车问题
        Motor_SetPWM_L(0);
        Motor_SetPWM_R(0);
    }
}

/*=============================================================================
 * PlacePID_Compute - 位置环/转向环计算
 * 采用非线性PD + 陀螺仪补偿结构
 * 
 * @param pid      - PID控制器实例
 * @param setpoint - 目标值(通常为0，表示走直线/回中)
 * @param measured - 当前测量值(误差或角度)
 * @param gyro     - 陀螺仪角速度(用于抑制振荡)
 * @return         - 控制输出
 *===========================================================================*/
float PlacePID_Compute(PlacePID_t *pid, float setpoint, float measured, float gyro) 
{
    float error = setpoint - measured;
    float output;
    
    // P项: 线性比例 + 非线性增强(大误差时更激进)
    // D项: 基于误差变化率 - 陀螺仪角速度阻尼(抑制转向)
    output = pid->Kp * error 
           + pid->Kpp * (error * fabs(error))
           + pid->Kd * (error - pid->last_error)
           - pid->Kdd * gyro;  // 负号：角速度越大，抑制转向越强
    
    // 死区处理，消除小误差时的抖动
    if (fabs(output) <= 5) output = 0;
    
    pid->last_error = error;
    return output; 
}

/*=============================================================================
 * SpeedPID_Compute - 速度环PID计算
 * 采用增量式PID，带积分限幅
 * 
 * @param pid      - PID控制器实例
 * @param setpoint - 目标速度
 * @param measured - 当前速度(编码器读数)
 * @return         - PWM输出值
 *===========================================================================*/
int SpeedPID_Compute(SpeedPID_t *pid, int setpoint, int measured) 
{
    int error = setpoint - measured;
    int output;
    
    // 积分累加并限幅
    pid->integral += error;
    pid->integral = Min_Max(pid->integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    
    // PID计算
    output = pid->Kp * error 
           + pid->Ki * pid->integral 
           + pid->Kd * (error - pid->last_error);
    
    pid->last_error = error;
    return output;
}

/*=============================================================================
 * PID初始化函数
 *===========================================================================*/
void SpeedPID_Init(SpeedPID_t *pid, int kp, int ki, int kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0;
    pid->last_error = 0;
}

void PlacePID_Init(PlacePID_t *pid, float kp, float kpp, float kd, float kdd)
{
    pid->Kp = kp;
    pid->Kpp = kpp;
    pid->Kd = kd;
    pid->Kdd = kdd;
    pid->last_error = 0;
}

/*=============================================================================
 * Different_Speed - 差速计算
 * 根据位置环和角度环输出计算左右轮目标速度
 * SPEED_GAIN: 控制转向输出对轮速的影响程度，越小修正越温和
 *===========================================================================*/
#define SPEED_GAIN 0.008f  // 差速增益

void Different_Speed() 
{ 
 float k;  
 float Turn_factor=0.5;
		
 if(Place_Out >= 0) 
	 {
		k = Place_Out * SPEED_GAIN + Angle_out * SPEED_GAIN; 
		 Left_Speed = Basic_Speed * (1 - k); 
		 Right_Speed = Basic_Speed * (1 + k*Turn_factor); 
	 } 
	 else 
	{ 
		k = -(Place_Out * SPEED_GAIN + Angle_out * SPEED_GAIN); 
		Left_Speed = Basic_Speed * (1 + k*Turn_factor); 
		Right_Speed = Basic_Speed * (1 - k); 
  } 
}
