#ifndef __MODE_CONTROL_H
#define __MODE_CONTROL_H

#include "stm32f10x.h"

/*=============================================================================
 * 系统模式定义
 *===========================================================================*/
typedef enum {
    MODE_CALIBRATING = 0,   // 校准中
    MODE_MENU,              // 主菜单
    MODE_LINE_FOLLOW,       // 巡线模式
    MODE_BLUETOOTH,         // 蓝牙键控模式
    MODE_INFO_VIEW          // 信息查看模式
} SystemMode_t;

/*=============================================================================
 * 蓝牙命令格式
 * FWD:速度  - 前进 (如 FWD:50)
 * BWD:速度  - 后退 (如 BWD:50)
 * TL:速度   - 左转 (如 TL:30)
 * TR:速度   - 右转 (如 TR:30)
 * STOP      - 停止
 * KEY:1     - 进入蓝牙模式
 * ESTOP     - 急停，退出所有模式返回主菜单
 *===========================================================================*/
#define CMD_BUFFER_SIZE  20

/*=============================================================================
 * 外部变量声明
 *===========================================================================*/
extern SystemMode_t System_Mode;
extern int BT_Speed;            // 蓝牙控制速度

/*=============================================================================
 * 函数声明
 *===========================================================================*/
void Mode_Init(void);
void Mode_Process(void);
void Mode_ShowCalibrating(void);
void Mode_ShowMenu(void);
void Mode_ShowLineFollow(void);
void Mode_ShowBluetooth(void);
void Mode_ShowInfoView(void);

// 蓝牙命令处理
void BT_Command_Parse(char *cmd);
void BT_ReceiveChar(uint8_t ch);
void BT_ProcessCommand(void);  // 在主循环中调用
void BT_Control_Update(void);

#endif
