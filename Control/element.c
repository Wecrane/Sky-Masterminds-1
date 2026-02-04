#include "myfile.h"


// 状态变量
uint8_t Element_Flag = 0;          // 元素锁定标志：0-未锁定（允许丢线保护），1-锁定
uint8_t Patrol_State = 66;          // 巡线状态机：0-23
uint8_t Noline_Flag = 0;           // 丢线标志：0-正常，1-首次检测，2-持续丢线
uint8_t Stop_Flag = 0;             // 停车标志：0-未停车，1-触发停车
uint8_t second_line_count = 0;     // 二次发车区域横线计数
uint8_t second_last_line = 0;      // 二次发车区域横线状态
uint8_t Turn_State = 0;            // 转向状态：0-未转向，1-右转中，2-左转中

// 配置参数
#define YAW_DEADZONE 0.5f          // yaw角死区（±3°）
#define YAW_KP 60.0f                // 虚线补盲比例系数
#define NOLINE_THRESHOLD 80.0f     // 丢线保护阈值（持续丢线超过30cm停车）
// 转向确认计数阈值：连续检测到该条件达此次数才触发转向
#define TURN_CONFIRM_COUNT 3


extern float Real_Distance;          // 单计数对应的实际距离（来自编码器配置）
float straight_target_yaw = 0.0f;
float yaw_error = 0.0f;           // 当前yaw与基准的偏差

/************************元素总控制台***************************/
void Element_Process()
{
    if (Start_Run_Flag == 1 && Stop_Flag == 0)
    {
        Patrol_Process();
        Element_Noline();
    }
}


/************************巡线状态机（核心逻辑）***************************/
void Patrol_Process()
{
    // --------------------------
    // 转向过程
    // --------------------------


if (Turn_State == 1)  // 右转中（R2触发）
{
    Element_Flag = 1;
    Place_Enable = 0;
    Basic_Speed = 50;
    Place_Out = -100;

    static uint8_t has_left = 0;
    static uint8_t turn_started = 0;  // 标记转向是否开始
    static float left_wheel_total = 0.0f;  // 转向过程中左轮累计距离

    // 初始化：首次进入转向时，重置左轮累计距离
    if (turn_started == 0)
    {
        left_wheel_total = 0.0f;  // 从0开始计数
        turn_started = 1;
    }

    left_wheel_total += Speed_L * Real_Distance;

    // 正常结束条件：先离线（R2=0）→再回线（R2=1）
    if (R2 == 0)
    {
        has_left = 1;  // 标记已离线
    }
    else if (R2 == 1 && has_left == 1)
    {
        // 回线时重置所有状态
        Turn_State = 0;
        Place_Out = 0;
        has_left = 0;
        turn_started = 0;
        left_wheel_total = 0.0f;
        Place_Enable = 1;
        Element_Flag = 0;
        Location_per = 0.0f;
        return;
    }

    // 异常保护）
    if (left_wheel_total > 50.0f)
    {
        Turn_State = 0;
        Place_Out = 0;
        has_left = 0;
        turn_started = 0;
        left_wheel_total = 0.0f;
        Place_Enable = 0;
        Basic_Speed = 0;
        Motor_SetPWM_L(0);
        Motor_SetPWM_R(0);
        Stop_Flag = 1;
        Element_Flag = 0;
        Location_per = 0.0f;
		Noline_Flag = 2;
        return;
    }

    return;
}
else if (Turn_State == 2)  // 左转中（L2触发）
{
    Element_Flag = 1;
    Place_Enable = 0;
    Basic_Speed = 50;
    Place_Out = 100;  // 左转转向量

    static uint8_t has_left = 0;     // 标记是否经历过L2离线
    static uint8_t turn_started = 0; // 标记左转是否开始
    static float right_wheel_total = 0.0f;  // 左转过程中右轮累计距离

    // 初始化：首次进入左转时，重置右轮距离计数
    if (turn_started == 0)
    {
        right_wheel_total = 0.0f;  // 从0开始累计
        turn_started = 1;
    }

    // 实时累加右轮距离：右轮当前周期计数 × 单计数距离
    right_wheel_total += Speed_R * Real_Distance;

    // 正常结束条件：先离线（L2=0）→再回线（L2=1）
    if (L2 == 0)
    {
        has_left = 1;  // 标记已离线
    }
    else if (L2 == 1 && has_left == 1)
    {
        // 回线时重置所有状态
        Turn_State = 0;
        Place_Out = 0;
        has_left = 0;
        turn_started = 0;
        right_wheel_total = 0.0f;
        Place_Enable = 1;
        Element_Flag = 0;
        Location_per = 0.0f;
        return;
    }

    // 异常保护
    if (right_wheel_total > 50.0f)
    {
        Turn_State = 0;
        Place_Out = 0;
        has_left = 0;
        turn_started = 0;
        right_wheel_total = 0.0f;
        Place_Enable = 0;
        Basic_Speed = 0;
        Motor_SetPWM_L(0);
        Motor_SetPWM_R(0);
        Stop_Flag = 1;
        Element_Flag = 0;
        Location_per = 0.0f;
		Noline_Flag = 2;
        return;
    }

    return;
}

    // --------------------------
    // 状态0：大圆环前分段加减速
    // --------------------------
    if (Patrol_State == 0)
    {
        Place_Enable = 1;
        PWM_Enable = 1;
        if (Location_per <= 50) 
            Basic_Speed = 150;
        else if (Location_per <= 330) 
            Basic_Speed = 80;
        else 
        {
            Basic_Speed = 60;
            Patrol_State = 1;
            Location_per = 0.0f;  // 清零状态机距离
        }
    }

    // --------------------------
    // 状态1：大圆环十字检测
    // --------------------------
    else if (Patrol_State == 1)
    {
        if (L2 == 1 && R2 == 1)
        {
            Patrol_State = 2;
            Turn_State = 1;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态2：大圆环首次绕环
    // --------------------------
    else if (Patrol_State == 2)
    {
        Basic_Speed = 110;
        if (Location_per > 100)
        {
            Basic_Speed = 60;
            Patrol_State = 3;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态3：大圆环首次等待R2触发右转
    // --------------------------
    else if (Patrol_State == 3)
    {
        if (R2 == 1)
        {
            Patrol_State = 4;
            Turn_State = 1;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态4：大圆环首次右转后直行
    // --------------------------
    else if (Patrol_State == 4)
    {
        Basic_Speed = 110;
        if (Location_per > 40)
        {
            Basic_Speed = 60;
            Patrol_State = 5;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态5：小圆环十字检测
    // --------------------------
    else if (Patrol_State == 5)
    {
        if (L2 == 1 && R2 == 1)
        {
            Patrol_State = 6;
            Turn_State = 1;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态6：小圆环绕环
    // --------------------------
    else if (Patrol_State == 6)
    {
        Basic_Speed = 120;
        if (Location_per > 230)
        {
            Basic_Speed = 60;
            Patrol_State = 7;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态7：小圆环等待R2触发右转
    // --------------------------
    else if (Patrol_State == 7)
    {
        if (R2 == 1)
        {
            Patrol_State = 8;
            Turn_State = 1;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态8：小圆环直行返回大圆环
    // --------------------------
    else if (Patrol_State == 8)
    {
        Basic_Speed = 110;
        if (Location_per > 40)
        {
            Basic_Speed = 60;
            Patrol_State = 9;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态9：大圆环二次十字检测
    // --------------------------
    else if (Patrol_State == 9)
    {
        if (L2 == 1 && R2 == 1)
        {
            Patrol_State = 10;
            Turn_State = 1;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态10：大圆环二次绕环
    // --------------------------
    else if (Patrol_State == 10)
    {
        Basic_Speed = 110;
        if (Location_per > 100)
        {
            Basic_Speed = 60;
            Patrol_State = 11;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态11：大圆环二次等待R2触发右转
    // --------------------------
    else if (Patrol_State == 11)
    {
        if (R2 == 1)
        {
            Patrol_State = 12;
            Turn_State = 1;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态12：直行（准备左转）
    // --------------------------
    else if (Patrol_State == 12)
    {
        if (Location_per < 20)
            Basic_Speed = 60;
        else if (Location_per < 150)
            Basic_Speed = 150;
        else
        {
            Basic_Speed = 60;
            Patrol_State = 13;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态13：左转
    // --------------------------
    else if (Patrol_State == 13)
    {
        // 防误判：要求外侧和内侧传感器连续检测若干次才触发左转
        static uint8_t left_confirm_cnt = 0;
        if (L2 == 1 && L1 == 1)
        {
            if (left_confirm_cnt < TURN_CONFIRM_COUNT) left_confirm_cnt++;
        }
        else
        {
            left_confirm_cnt = 0;
        }

        if (left_confirm_cnt >= TURN_CONFIRM_COUNT)
        {
            left_confirm_cnt = 0;
            Patrol_State = 14;
            Turn_State = 2;
            Location_per = 0.0f;
        }
    }


// --------------------------
// 状态14：二次发车区域检测（共5根横线，检测4根后直行）
// --------------------------
else if (Patrol_State == 14)
{
    Basic_Speed = 90;
    Place_Enable = 1;
    PWM_Enable = 1;

    // 横线判定：L2和R2同时检测到黑线
    uint8_t second_current_line = (L2 == 1 && R2 == 1) ? 1 : 0;
    
    // 上升沿计数
    if (second_current_line == 1 && second_last_line == 0)
    {
        second_line_count++;
    }
    second_last_line = second_current_line;
    
    // 累计5根横线后，切换为yaw辅助直行（关闭速度环）
    if (second_line_count >= 4)
    {   
		yaw=0.0f;
        Element_Flag = 1;
        Place_Enable = 0;      // 关闭原有转向环
        PWM_Enable = 0;        // 关闭速度环：直接控制PWM
        Location_per = 0.0f;
        Patrol_State = 15;
        second_line_count = 0;
        second_last_line = 0;
    }
}

// --------------------------
// 状态15：固定PWM直行（50cm后执行左转）- yaw辅助修正
// --------------------------
else if (Patrol_State == 15)
{   
	Element_Flag = 1;
    Place_Enable = 0;
    PWM_Enable = 0;
	
static uint8_t init_flag = 0;
if (init_flag == 0)
{
    straight_target_yaw = 0.0f;
    init_flag = 1;
}

// 直接计算累积偏差
yaw_error = yaw - straight_target_yaw;

// 计算PWM;
int32_t left_pwm = 6000 + (int32_t)(yaw_error * YAW_KP);
int32_t right_pwm = 6000 - (int32_t)(yaw_error * YAW_KP);

// 限幅保护
left_pwm = (left_pwm < 0) ? 0 : (left_pwm > 8000 ? 8000 : left_pwm);
right_pwm = (right_pwm < 0) ? 0 : (right_pwm > 8000 ? 8000 : right_pwm);

// 输出PWM
Motor_SetPWM_L(left_pwm);
Motor_SetPWM_R(right_pwm);

    // 距离达标后切换为左转
    if (Location_per > 60)
    {
		Location_per = 0.0f;
        Patrol_State = 16;
    }
}

// --------------------------
// 状态16：二次左转
// --------------------------
else if (Patrol_State == 16)
{
    PWM_Enable = 1;       
    Place_Enable = 0;     
    Basic_Speed = 20;     
    Place_Out = 150;      



    // 转向完成判定
    if (yaw >= 58.0f)
    {
        Place_Enable = 0;
        PWM_Enable = 0;
        Location_per = 0.0f;
        Patrol_State = 17;
        return;
    }
}

// --------------------------
// 状态17：固定PWM直行
// --------------------------
else if (Patrol_State == 17)
{
    // 静态变量：初始化标志、yaw稳定计数器、基准yaw
    static uint8_t init_flag = 0;
    static uint8_t yaw_stable_cnt = 0;  // yaw稳定计数
    static float last_yaw = 0.0f;       // 上一周期yaw
    float yaw_error;

    // 1. 等待yaw稳定（连续5次波动≤1°视为稳定）
    if (init_flag == 0)
    {
        float yaw_fluctuation = fabs(yaw - last_yaw);
        last_yaw = yaw;

        if (yaw_fluctuation <= 1.0f)
        {
            yaw_stable_cnt++;
        }
        else
        {
            yaw_stable_cnt = 0;
        }

        if (yaw_stable_cnt >= 2)
        {
            straight_target_yaw = 91.0f;  
            init_flag = 1;
            yaw_stable_cnt = 0;
        }
        else
        {
            Motor_SetPWM_L(0);
            Motor_SetPWM_R(0);
            return;
        }
    }


// 直接计算累积偏差
yaw_error = yaw - straight_target_yaw;

// 计算PWM
int32_t left_pwm = 6000 + (int32_t)(yaw_error * YAW_KP);
int32_t right_pwm = 6000 - (int32_t)(yaw_error * YAW_KP);

// 限幅保护
left_pwm = (left_pwm < 0) ? 0 : (left_pwm > 8000 ? 8000 : left_pwm);
right_pwm = (right_pwm < 0) ? 0 : (right_pwm > 8000 ? 8000 : right_pwm);

// 输出PWM
Motor_SetPWM_L(left_pwm);
Motor_SetPWM_R(right_pwm);

    // 3. 距离达标后切换状态
    if (Location_per > 90)
    {
        PWM_Enable = 1;
        Place_Enable = 1;
        Basic_Speed = 90;
        Location_per = 0.0f;
        init_flag = 0;
        Patrol_State = 18;
    }
}

    // --------------------------
    // 状态18：恢复转向环后分段降速
    // 逻辑：基于全程距离分阶段降速，为后续十字检测做准备
    // --------------------------
    else if (Patrol_State == 18)
    {

        // 基于全程距离分段降速（避免速度过快导致检测失误）
		 if (Location_per <= 30)
        {
            Basic_Speed = 110;      
        }
        else if (Location_per <= 90)
        {
            Basic_Speed = 130;      
			Element_Flag = 0;         // 解锁元素：允许正常巡线逻辑生效			
        }
        else if (Location_per <= 230)
        {
            Basic_Speed = 120;        
        }
        else
        {
            Basic_Speed = 60;        // 后段：低速行驶，准备十字检测
            Location_per = 0.0f;        // 清零距离：为十字检测计数做准备
            Patrol_State = 19;        // 进入下一状态：十字检测（进入大圆环）
        }
    }

    // --------------------------
    // 状态19：检测十字后触发右转
    // --------------------------
    else if (Patrol_State == 19)
    {
        Basic_Speed = 60;
        if (L2 == 1 && R2 == 1)
        {
            Patrol_State = 20;
            Turn_State = 1;
            Location_per = 0.0f;
        }
    }

    // --------------------------
    // 状态20：大圆环绕环（计数R2触发）
    // --------------------------
    else if (Patrol_State == 20)
    {
        static uint8_t r2_count = 0, last_r2 = 0, after_4th_flag = 0;

        if (R2 == 1 && last_r2 == 0)
        {
            r2_count++;
            if (r2_count == 4)
            {
                after_4th_flag = 1;
                Location_per = 0.0f;
            }
            else if (r2_count == 5)
            {
                Turn_State = 1;
                r2_count = 0;
                last_r2 = 0;
                after_4th_flag = 0;
                Patrol_State = 21;
                return;
            }
        }
        last_r2 = R2;

        Basic_Speed = after_4th_flag ? ((Location_per <= 100) ? 150 : 60) : 150;
    }


    // --------------------------
    // 状态21：检测2根横线后停车
    // --------------------------
    else if (Patrol_State == 21)
    {
        Basic_Speed = 120;
        static uint8_t cross_count = 0, last_cross = 0;

        uint8_t current_cross = (L2 == 1 && R2 == 1) ? 1 : 0;
        if (current_cross == 1 && last_cross == 0)
            cross_count++;
        last_cross = current_cross;

        if (cross_count >= 2)
        {
            Patrol_State = 22;
        }
    }

    // --------------------------
    // 状态22：流程结束
    // --------------------------
    else if (Patrol_State == 22)
    {
        Element_Flag = 1;
        Place_Enable = 0;
        PWM_Enable = 0;
        Basic_Speed = 0;
        Motor_SetPWM_L(0);
        Motor_SetPWM_R(0);
    }
else if (Patrol_State == 66 && Element_Flag == 0)
{
        Place_Enable = 1;
        PWM_Enable = 1;
        Basic_Speed = 60;

}

}

/************************优化后的丢线保护逻辑***************************/
void Element_Noline()
{
    // 仅未锁定且未停车时生效
    if (Element_Flag == 0 && Stop_Flag == 0)
    {
        // 检测是否丢线（全传感器无信号）
        uint8_t is_noline = (L2 == 0 && L1 == 0 && M == 0 && R1 == 0 && R2 == 0);

        if (is_noline)
        {
            // 丢线状态：累计距离超过阈值则停车
            if (Location > NOLINE_THRESHOLD)
            {
                Stop_Flag = 1;
                Place_Enable = 0;
                Basic_Speed = 0;
                Motor_SetPWM_L(0);
                Motor_SetPWM_R(0);
				Noline_Flag = 1;
            }
        }
        else
        {
            // 找回线：重置丢线距离
            Location = 0.0f;
        }
    }
}
