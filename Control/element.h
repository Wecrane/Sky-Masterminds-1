#ifndef _ELEMENT_H__
#define _ELEMENT_H__

void Element_Process(void);        // 总控制台：调度巡线、丢线保护逻辑
void Element_Normal(void);         // 正常巡线逻辑：未锁定时恢复默认配置
void Patrol_Process(void);         // 核心：巡线过程
void Element_Noline(void);         // 丢线保护逻辑：防误判与紧急停车

extern float kL,kR;
extern uint8_t Element_Flag,Noline_Flag,Stop_Flag,Patrol_State;
#endif
