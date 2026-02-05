#include "myfile.h"
#include "mode_control.h"
#include <string.h>
#include <stdlib.h>

/*=============================================================================
 * 全局变量
 *===========================================================================*/
SystemMode_t System_Mode = MODE_CALIBRATING;
int BT_Speed = 50;              // 蓝牙控制基础速度
static int8_t BT_Direction = 0; // 蓝牙控制方向：0-停止，1-前，-1-后
static int8_t BT_Turn = 0;      // 蓝牙转向：0-直行，1-右，-1-左
static int BT_TurnSpeed = 30;   // 转向速度

// 命令缓冲区
static char cmd_buffer[CMD_BUFFER_SIZE];
static uint8_t cmd_index = 0;

// 命令就绪标志和解析缓冲区（避免在中断中调用Serial_Printf导致卡死）
static volatile uint8_t cmd_ready = 0;
static char parse_buffer[CMD_BUFFER_SIZE];

/*=============================================================================
 * 模式初始化
 *===========================================================================*/
void Mode_Init(void)
{
    System_Mode = MODE_CALIBRATING;
    BT_Speed = 50;
    BT_Direction = 0;
    BT_Turn = 0;
    BT_TurnSpeed = 30;
    cmd_index = 0;
    cmd_ready = 0;
    memset(cmd_buffer, 0, CMD_BUFFER_SIZE);
    memset(parse_buffer, 0, CMD_BUFFER_SIZE);
}

/*=============================================================================
 * 模式处理主函数 - 在main循环中调用
 *===========================================================================*/
void Mode_Process(void)
{
    uint8_t key_event = Key_GetEvent();
    
    // 处理蓝牙命令（在主循环中解析，避免在中断中卡死）
    BT_ProcessCommand();
    
    switch (System_Mode)
    {
        case MODE_CALIBRATING:
            // 校准模式不响应按键，在main中完成后切换
            Mode_ShowCalibrating();
            break;
            
        case MODE_MENU:
            Mode_ShowMenu();
            if (key_event == KEY_EVENT_K1_SHORT)
            {
                // K1短按：进入巡线模式
                System_Mode = MODE_LINE_FOLLOW;
                OLED_Clear();
                PWM_Enable = 1;
                Place_Enable = 1;
                Basic_Speed = 65;  // 设置基础速度
            }
            else if (key_event == KEY_EVENT_K2_SHORT)
            {
                // K2短按：进入蓝牙模式
                System_Mode = MODE_BLUETOOTH;
                OLED_Clear();
                PWM_Enable = 1;
                Place_Enable = 0;
                BT_Direction = 0;
                BT_Turn = 0;
                BT_Speed = 0;
                BT_TurnSpeed = 0;
                Left_Speed = 0;
                Right_Speed = 0;
                // 重置速度环PID积分项
                speedPID_L.integral = 0;
                speedPID_L.last_error = 0;
                speedPID_R.integral = 0;
                speedPID_R.last_error = 0;
            }
            else if (key_event == KEY_EVENT_K1_LONG)
            {
                // K1长按：进入信息查看模式
                System_Mode = MODE_INFO_VIEW;
                OLED_Clear();
                PWM_Enable = 0;
                Place_Enable = 0;
            }
            break;
            
        case MODE_LINE_FOLLOW:
            Mode_ShowLineFollow();
            if (key_event == KEY_EVENT_K1_SHORT || key_event == KEY_EVENT_K2_SHORT)
            {
                // 退出巡线模式
                System_Mode = MODE_MENU;
                OLED_Clear();
                PWM_Enable = 0;
                Place_Enable = 0;
                Basic_Speed = 0;
                Motor_SetPWM_L(0);
                Motor_SetPWM_R(0);
            }
            break;
            
        case MODE_BLUETOOTH:
            Mode_ShowBluetooth();
            BT_Control_Update();
            if (key_event == KEY_EVENT_K1_SHORT || key_event == KEY_EVENT_K2_SHORT)
            {
                // 退出蓝牙模式
                System_Mode = MODE_MENU;
                OLED_Clear();
                PWM_Enable = 0;
                Place_Enable = 0;
                Left_Speed = 0;
                Right_Speed = 0;
                Motor_SetPWM_L(0);
                Motor_SetPWM_R(0);
                BT_Direction = 0;
                BT_Turn = 0;
            }
            break;
            
        case MODE_INFO_VIEW:
            Mode_ShowInfoView();
            if (key_event == KEY_EVENT_K1_SHORT || key_event == KEY_EVENT_K2_SHORT)
            {
                // 退出信息查看模式
                System_Mode = MODE_MENU;
                OLED_Clear();
            }
            break;
            
        default:
            System_Mode = MODE_MENU;
            break;
    }
}

/*=============================================================================
 * 显示函数
 *===========================================================================*/
void Mode_ShowCalibrating(void)
{
    // 128x64 OLED, 8x16字体居中
    OLED_ShowString(8, 16, "Calibrating...", OLED_8X16);
    OLED_ShowString(20, 36, "Please Wait", OLED_8X16);
}

void Mode_ShowMenu(void)
{
    OLED_ShowString(0, 0, "=== Main Menu ===", OLED_8X16);
    OLED_ShowString(0, 20, "K1: Line Follow", OLED_8X16);
    OLED_ShowString(0, 36, "K2: Bluetooth", OLED_8X16);
    OLED_ShowString(0, 52, "Hold K1: Info", OLED_6X8);
}

void Mode_ShowLineFollow(void)
{
    OLED_ShowString(0, 0, "[Line Follow]", OLED_8X16);
    OLED_Printf(0, 20, OLED_8X16, "Speed:%+03d", Basic_Speed);
    OLED_Printf(0, 36, OLED_8X16, "Err:%+04.1f", final_err);
    OLED_ShowString(0, 56, "K1/K2:Exit", OLED_6X8);
}

void Mode_ShowBluetooth(void)
{
    OLED_ShowString(0, 0, "[Bluetooth]", OLED_8X16);
    OLED_Printf(0, 20, OLED_8X16, "Speed:%+03d", BT_Speed);
    OLED_Printf(0, 36, OLED_8X16, "Dir:%+02d Turn:%+02d", BT_Direction, BT_Turn);
    OLED_ShowString(0, 56, "K1/K2:Exit", OLED_6X8);
}

void Mode_ShowInfoView(void)
{
    OLED_ShowString(0, 0, "[Info View]", OLED_8X16);
    OLED_Printf(0, 16, OLED_8X16, "Yaw:%+07.2f", yaw);
    OLED_Printf(0, 32, OLED_8X16, "Loc:%+07.2f", Location);
    OLED_Printf(0, 48, OLED_6X8, "L:%+04d R:%+04d", Speed_L, Speed_R);
    OLED_ShowString(0, 58, "K1/K2:Exit", OLED_6X8);
}

/*=============================================================================
 * 进入蓝牙模式 - 任何模式下都可调用
 *===========================================================================*/
static void Enter_Bluetooth_Mode(void)
{
    System_Mode = MODE_BLUETOOTH;
    OLED_Clear();
    PWM_Enable = 1;
    Place_Enable = 0;
    BT_Direction = 0;
    BT_Turn = 0;
    BT_Speed = 0;
    BT_TurnSpeed = 0;
    Left_Speed = 0;
    Right_Speed = 0;
    // 重置速度环PID积分项，避免残留影响
    speedPID_L.integral = 0;
    speedPID_L.last_error = 0;
    speedPID_R.integral = 0;
    speedPID_R.last_error = 0;
}

/*=============================================================================
 * 蓝牙命令解析
 * 命令格式：
 *   FWD:速度  - 前进
 *   BWD:速度  - 后退
 *   TL:速度   - 左转
 *   TR:速度   - 右转
 *   STOP      - 停止
 *   KEY:1     - 进入蓝牙模式
 *   ESTOP     - 急停，退出所有模式返回主菜单
 *===========================================================================*/

// 去除字符串前后空格
static char* str_trim(char *str)
{
    char *end;
    // 去除前导空格
    while (*str == ' ' || *str == '\t') str++;
    if (*str == '\0') return str;
    // 去除尾部空格
    end = str + strlen(str) - 1;
    while (end > str && (*end == ' ' || *end == '\t')) end--;
    *(end + 1) = '\0';
    return str;
}

void BT_Command_Parse(char *cmd)
{
    int speed_val = 0;
    char *colon_pos;
    char original_cmd[CMD_BUFFER_SIZE];  // 保存原始命令用于回显
    
    // 保存原始命令
    strncpy(original_cmd, cmd, CMD_BUFFER_SIZE - 1);
    original_cmd[CMD_BUFFER_SIZE - 1] = '\0';
    
    // 回显收到的原始命令
    Serial_Printf("[RX] %s\r\n", original_cmd);
    
    // 去除前后空格
    cmd = str_trim(cmd);
    
    // 转换为大写便于比较
    for (int i = 0; cmd[i]; i++) {
        if (cmd[i] >= 'a' && cmd[i] <= 'z') {
            cmd[i] -= 32;
        }
    }
    
    // KEY:1命令 - 任何模式下都响应，进入蓝牙模式
    if (strcmp(cmd, "KEY:1") == 0)
    {
        Enter_Bluetooth_Mode();
        Serial_Printf("[OK] Enter BT Mode\r\n");
        return;
    }
    
    // ESTOP命令 - 任何模式下都响应，急停并返回主菜单
    if (strcmp(cmd, "ESTOP") == 0)
    {
        // 立即停止电机
        Motor_SetPWM_L(0);
        Motor_SetPWM_R(0);
        // 清零所有控制变量
        BT_Direction = 0;
        BT_Turn = 0;
        BT_Speed = 0;
        BT_TurnSpeed = 0;
        Left_Speed = 0;
        Right_Speed = 0;
        Basic_Speed = 0;
        PWM_Enable = 0;
        Place_Enable = 0;
        Angle_Enable = 0;
        // 重置所有PID
        speedPID_L.integral = 0;
        speedPID_L.last_error = 0;
        speedPID_R.integral = 0;
        speedPID_R.last_error = 0;
        placePID.last_error = 0;
        anglePID.last_error = 0;
        // 返回主菜单
        System_Mode = MODE_MENU;
        OLED_Clear();
        Serial_Printf("[OK] ESTOP -> MENU\r\n");
        return;
    }
    
    // 其他命令只在蓝牙模式下响应
    if (System_Mode != MODE_BLUETOOTH)
    {
        Serial_Printf("[ERR] Not in BT Mode\r\n");
        return;
    }
    
    // STOP命令
    if (strcmp(cmd, "STOP") == 0)
    {
        BT_Direction = 0;
        BT_Turn = 0;
        BT_Speed = 0;
        BT_TurnSpeed = 0;
        Left_Speed = 0;
        Right_Speed = 0;
        // 重置速度环PID，消除积分残留
        speedPID_L.integral = 0;
        speedPID_L.last_error = 0;
        speedPID_R.integral = 0;
        speedPID_R.last_error = 0;
        // 立即停止电机
        Motor_SetPWM_L(0);
        Motor_SetPWM_R(0);
        Serial_Printf("[OK] STOP\r\n");
        return;
    }
    
    // 查找冒号分隔符
    colon_pos = strchr(cmd, ':');
    if (colon_pos != NULL)
    {
        speed_val = atoi(colon_pos + 1);
        if (speed_val < 0) speed_val = 0;
        if (speed_val > 100) speed_val = 100;
        
        // FWD:速度 - 前进
        if (strncmp(cmd, "FWD:", 4) == 0)
        {
            BT_Direction = 1;
            BT_Turn = 0;
            BT_Speed = speed_val;
            Serial_Printf("[OK] FWD Dir=1 Spd=%d\r\n", BT_Speed);
        }
        // BWD:速度 - 后退
        else if (strncmp(cmd, "BWD:", 4) == 0)
        {
            BT_Direction = -1;
            BT_Turn = 0;
            BT_Speed = speed_val;
            Serial_Printf("[OK] BWD Dir=-1 Spd=%d\r\n", BT_Speed);
        }
        // TL:速度 - 左转
        else if (strncmp(cmd, "TL:", 3) == 0)
        {
            BT_Turn = -1;
            BT_TurnSpeed = speed_val;
            Serial_Printf("[OK] TL Turn=-1 TSpd=%d\r\n", BT_TurnSpeed);
        }
        // TR:速度 - 右转
        else if (strncmp(cmd, "TR:", 3) == 0)
        {
            BT_Turn = 1;
            BT_TurnSpeed = speed_val;
            Serial_Printf("[OK] TR Turn=1 TSpd=%d\r\n", BT_TurnSpeed);
        }
        else
        {
            Serial_Printf("[ERR] Unknown CMD\r\n");
        }
    }
    else
    {
        Serial_Printf("[ERR] Invalid Format\r\n");
    }
}

/*=============================================================================
 * 蓝牙接收字符 - 在串口中断中调用
 * 注意：只接收数据，不做解析和回显，避免卡死
 *===========================================================================*/
void BT_ReceiveChar(uint8_t ch)
{
    // 收到换行或回车，设置命令就绪标志
    if (ch == '\n' || ch == '\r')
    {
        if (cmd_index > 0 && cmd_ready == 0)  // 上一条命令已处理完才接收新命令
        {
            cmd_buffer[cmd_index] = '\0';
            // 复制到解析缓冲区
            strncpy(parse_buffer, cmd_buffer, CMD_BUFFER_SIZE);
            cmd_ready = 1;  // 设置标志，由主循环处理
        }
        // 重置接收缓冲区
        cmd_index = 0;
        memset(cmd_buffer, 0, CMD_BUFFER_SIZE);
    }
    else
    {
        // 过滤掉无效字符（只保留可打印字符）
        if (ch >= 0x20 && ch <= 0x7E)
        {
            // 存入缓冲区
            if (cmd_index < CMD_BUFFER_SIZE - 1)
            {
                cmd_buffer[cmd_index++] = ch;
            }
            else
            {
                // 缓冲区溢出，重置
                cmd_index = 0;
                memset(cmd_buffer, 0, CMD_BUFFER_SIZE);
            }
        }
    }
}

/*=============================================================================
 * 蓝牙命令处理 - 在主循环中调用，检查并处理待解析的命令
 *===========================================================================*/
void BT_ProcessCommand(void)
{
    if (cmd_ready)
    {
        BT_Command_Parse(parse_buffer);
        cmd_ready = 0;
    }
}

/*=============================================================================
 * 蓝牙控制更新 - 设置目标速度，PWM由速度环在TIM2中断中计算
 *===========================================================================*/
void BT_Control_Update(void)
{
    int left_speed = 0;
    int right_speed = 0;
    int base_speed = BT_Speed;          // 目标速度 (编码器值)
    int turn_speed = BT_TurnSpeed;      // 转向速度
    
    if (BT_Direction == 1)  // 前进
    {
        left_speed = base_speed;
        right_speed = base_speed;
    }
    else if (BT_Direction == -1)  // 后退
    {
        left_speed = -base_speed;
        right_speed = -base_speed;
    }
    
    // 叠加转向
    if (BT_Turn == -1)  // 左转
    {
        left_speed -= turn_speed;
        right_speed += turn_speed;
    }
    else if (BT_Turn == 1)  // 右转
    {
        left_speed += turn_speed;
        right_speed -= turn_speed;
    }
    
    // 原地转向（无前进后退时）
    if (BT_Direction == 0 && BT_Turn != 0)
    {
        if (BT_Turn == -1)  // 原地左转
        {
            left_speed = -turn_speed;
            right_speed = turn_speed;
        }
        else  // 原地右转
        {
            left_speed = turn_speed;
            right_speed = -turn_speed;
        }
    }
    
    // 设置目标速度，PWM由速度环PID在TIM2中断中计算
    Left_Speed = left_speed;
    Right_Speed = right_speed;
}
