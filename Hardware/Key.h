#ifndef __KEY_H
#define __KEY_H

// 按键事件定义
#define KEY_EVENT_NONE       0
#define KEY_EVENT_K1_SHORT   1   // K1短按
#define KEY_EVENT_K2_SHORT   2   // K2短按
#define KEY_EVENT_K1_LONG    3   // K1长按
#define KEY_EVENT_K2_LONG    4   // K2长按

void Key_Init(void);
uint8_t Key_GetNum(void);    // 保留兼容性
uint8_t Key_GetEvent(void);  // 获取按键事件
void Key_Tick(void);


#endif
