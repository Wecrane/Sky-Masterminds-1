#include "stm32f10x.h"                  // Device header

/************************编码器读取***************************/
int Speed_L,Speed_R,Speed_L_Temp,Speed_R_Temp;
float Speed_Smooth=0.2; 
float Location=0.0f;
float Location_per=0.0f;
float Real_Distance=0.0093940817285f;			//这里是自己实际推算出来的

void Encoder_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
	
	TIM_Cmd(TIM3, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
	
	TIM_Cmd(TIM4, ENABLE);
	
}

int16_t Encoder_Get_L(void)
{
		int16_t Temp1;
    Temp1 = TIM_GetCounter(TIM4); // 先获取计数值
    Speed_L_Temp = Temp1;         // 再赋值给临时变量
    TIM_SetCounter(TIM4, 0);      // 重置计数器
    return Temp1;
}

int16_t Encoder_Get_R(void)
{
    int16_t Temp2;
    Temp2 = -TIM_GetCounter(TIM3); // 先获取计数值
    Speed_R_Temp = Temp2;         // 再赋值给临时变量
    TIM_SetCounter(TIM3, 0);      // 重置计数器
    return Temp2;
}


void Encoder_Read()
{		
				Speed_L=Encoder_Get_L();
				Speed_R=Encoder_Get_R();   
			
				Speed_L=Speed_L*Speed_Smooth+Speed_L_Temp*(1-Speed_Smooth);
				Speed_R=Speed_R*Speed_Smooth+Speed_R_Temp*(1-Speed_Smooth);
	
				Location+=(Speed_L+Speed_R)*Real_Distance;
				Location_per+=(Speed_L+Speed_R)*Real_Distance;
}

void Clear_Location(void) 
{
    Location = 0.0f;  // 直接重置为0
}

