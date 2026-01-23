#include "myfile.h"
/************************°´¼ü***************************/
uint8_t KeyNum=0;
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
		return 1;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0)
	{
		return 2;
	}
	return 0;
}

void Key_Tick()
{
	static uint8_t Count;
	static uint8_t CurrState, PrevState;
	Count ++;
	
	if (Count >=3)
	{
		Count = 0;
		PrevState = CurrState;
		CurrState = Key_GetState();
		if (CurrState == 0 && PrevState != 0)
		{
			KeyNum = PrevState;
		}
	}
}

uint8_t Key_GetNum(void)
{
	uint8_t Temp;
	if (KeyNum)
	{
		Temp = KeyNum;
		KeyNum = 0;
		return Temp;
	}
	return 0;
}
