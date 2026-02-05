#ifndef _ELEMENT_H__
#define _ELEMENT_H__

// 元素处理函数
void Element_Process(void);
void Element_Normal(void);
void Element_Noline(void);
void Element_Stop(void);

// 外部变量
extern uint8_t Element_Flag, Noline_Flag, Stop_Flag;
extern int Speed_Choice[5];

#endif
