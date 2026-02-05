#include "myfile.h"

/************************元素控制***************************/

// 标志量
uint8_t Element_Flag = 0;
uint8_t Noline_Flag = 0;


/************************元素控制台***************************/
// 根据需要启用或关闭特定元素处理

void Element_Process(void)
{
    Element_Normal();       // 正常状态下，默认打开转向环和速度环
    Element_Noline();       // 丢线处理
}

// 正常巡线状态
void Element_Normal(void)
{
    if (Element_Flag == 0)
    {
        Place_Enable = 1;
        PWM_Enable = 1;
        Basic_Speed = 40; 
    }
}

// 丢线处理
void Element_Noline(void)
{
    if (Element_Flag == 0 && Noline_Flag == 0)
    {
        if (L2 == 0 && L1 == 0 && M == 0 && R1 == 0 && R2 == 0)  // 丢线判断
        {
            Noline_Flag = 1;
            Clear_Location();
        }
    }
    
    if (Noline_Flag == 1)
    {
        if (L2 == 0 && L1 == 0 && M == 0 && R1 == 0 && R2 == 0)
        {
            if (Location > 50)  // 累计路程大于50，正式进入丢线
            {
                Place_Enable = 0;
                PWM_Enable = 0;
                Basic_Speed = 0;
            }
        }
        else
        {
            Clear_Location();
            Noline_Flag = 0;
        }
    }
}

