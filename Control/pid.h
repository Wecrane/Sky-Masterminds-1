#ifndef __PID_H
#define __PID_H

/*=============================================================================
 * PID控制器 - 控制频率: 100Hz (10ms周期)
 * Timer配置: 72MHz / 7200 / 100 = 100Hz
 *===========================================================================*/

#define INTEGRAL_MAX  1000
#define Min_Max(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define Max_PWM       6800
#define CONTROL_FREQ  100    // 控制频率 Hz
#define CONTROL_DT    0.01f  // 控制周期 秒

/*=============================================================================
 * 速度环PID结构体 (增量式PID)
 * 参数说明:
 *   Kp - 比例系数: 决定响应速度，过大会振荡
 *   Ki - 积分系数: 消除稳态误差，过大会超调
 *   Kd - 微分系数: 抑制振荡，预测误差趋势
 *===========================================================================*/
typedef struct {
    int Kp;              // 比例系数 (典型值: 100-300)
    int Ki;              // 积分系数 (典型值: 0-50)
    int Kd;              // 微分系数 (典型值: 50-200)
    int integral;        // 积分累计值
    int last_error;      // 上次误差(用于微分)
} SpeedPID_t;

/*=============================================================================
 * 位置环/转向环PD结构体 (位置式PD+非线性项)
 * 参数说明:
 *   Kp  - 线性比例: 基础响应强度
 *   Kpp - 非线性比例: err*|err|，大误差时增强响应
 *   Kd  - 微分系数: 基于误差变化率的阻尼
 *   Kdd - 陀螺仪补偿: 利用角速度提前预判，抑制振荡
 *===========================================================================*/
typedef struct {
    float Kp;            // 线性比例系数 (典型值: 3-8)
    float Kpp;           // 非线性比例系数 (典型值: 0.05-0.3)
    float Kd;            // 误差微分系数 (典型值: 2-6)
    float Kdd;           // 陀螺仪补偿系数 (典型值: 0.05-0.2)
    float last_error;    // 上次误差
} PlacePID_t;

/* 函数声明 */
void Control(void);
void Different_Speed(void);
void SpeedPID_Init(SpeedPID_t *pid, int kp, int ki, int kd);
void PlacePID_Init(PlacePID_t *pid, float kp, float kpp, float kd, float kdd);
int  SpeedPID_Compute(SpeedPID_t *pid, int setpoint, int measured);
float PlacePID_Compute(PlacePID_t *pid, float setpoint, float measured, float gyro);

/* 全局变量 */
extern SpeedPID_t speedPID_L, speedPID_R;  // 左右轮独立速度环
extern PlacePID_t placePID;                 // 位置环(巡线转向)
extern PlacePID_t anglePID;                 // 角度环(陀螺仪闭环)

extern int Speed_Out_L, Speed_Out_R, Place_Out, Angle_out;
extern float sensor_err;
extern float final_err;
extern int Basic_Speed;
extern int Left_Speed, Right_Speed;
extern uint8_t Place_Enable, PWM_Enable, Angle_Enable;

#endif
