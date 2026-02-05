#include "myfile.h"
/************************按键***************************/

// 按键事件定义
#define LONG_PRESS_TIME     100     // 长按阈值（100*10ms=1秒）
#define DEBOUNCE_COUNT      3       // 消抖计数

static uint8_t KeyEvent = 0;        // 按键事件
static uint8_t KeyHoldCount = 0;    // 按键按住计数
static uint8_t LastKeyState = 0;    // 上一次按键状态
static uint8_t LongPressFlag = 0;   // 长按标志（防止松手时再触发短按）

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

uint8_t Key_GetState(void)
{
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)
	{
		return 1;  // K1按下
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0)
	{
		return 2;  // K2按下
	}
	return 0;  // 无按键
}

// 长按检测的按键扫描（在定时器中断中调用，约10ms一次）
void Key_Tick(void)
{
	static uint8_t DebounceCount = 0;
	static uint8_t StableState = 0;
	uint8_t CurrState = Key_GetState();
	
	// 消抖处理
	if (CurrState == StableState)
	{
		DebounceCount = 0;
	}
	else
	{
		DebounceCount++;
		if (DebounceCount >= DEBOUNCE_COUNT)
		{
			DebounceCount = 0;
			StableState = CurrState;
		}
	}
	
	// 按键状态变化检测
	if (StableState != 0 && LastKeyState == 0)
	{
		// 按键按下
		KeyHoldCount = 0;
		LongPressFlag = 0;
	}
	else if (StableState != 0 && LastKeyState != 0)
	{
		// 按键持续按住
		KeyHoldCount++;
		if (KeyHoldCount >= LONG_PRESS_TIME && !LongPressFlag)
		{
			// 触发长按事件
			LongPressFlag = 1;
			if (StableState == 1)
			{
				KeyEvent = KEY_EVENT_K1_LONG;
			}
			else if (StableState == 2)
			{
				KeyEvent = KEY_EVENT_K2_LONG;
			}
		}
	}
	else if (StableState == 0 && LastKeyState != 0)
	{
		// 按键松开
		if (!LongPressFlag)
		{
			// 短按事件
			if (LastKeyState == 1)
			{
				KeyEvent = KEY_EVENT_K1_SHORT;
			}
			else if (LastKeyState == 2)
			{
				KeyEvent = KEY_EVENT_K2_SHORT;
			}
		}
		KeyHoldCount = 0;
		LongPressFlag = 0;
	}
	
	LastKeyState = StableState;
}

// 获取按键事件（获取后自动清零）
uint8_t Key_GetEvent(void)
{
	uint8_t event = KeyEvent;
	KeyEvent = 0;
	return event;
}

// 保留旧接口兼容性
uint8_t Key_GetNum(void)
{
	uint8_t event = Key_GetEvent();
	if (event == KEY_EVENT_K1_SHORT || event == KEY_EVENT_K1_LONG)
		return 1;
	if (event == KEY_EVENT_K2_SHORT || event == KEY_EVENT_K2_LONG)
		return 2;
	return 0;
}
