#include "sensor.h"


#define L2 digital(1)
/*************************************
*函数名称：SENSOR_GPIO_Config
*函数功能：GPIO管脚的配置
*参数：
*说明：
*			
**************************************/
void SENSOR_GPIO_Config(void)
{		
 GPIO_InitTypeDef GPIO_InitStructure;
    
    // ??GPIOB?GPIOA??(??????)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
    
    // ??GPIOB?Pin0?Pin1?????
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // ??GPIOA?Pin8?Pin11?Pin12?????
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*************************************
*函数名称：digtal
*函数功能：获取X通道数字值
*参数：
*说明：
*			
**************************************/
unsigned char digital(unsigned char channel)//1-ADC_N	  获取X通道数字值
{
	u8 value = 0;
	switch(channel) 
	{
		case 1:  
			if(PAin(4) == Color_Chice) value = 1;
			else value = 0;  
			break;  
		case 2: 
			if(PAin(5) == Color_Chice) value = 1;
			else value = 0;  
			break;  
		case 3: 
			if(PAin(12) == Color_Chice) value = 1;
			else value = 0;  
			break;   
		case 4:  
			if(PBin(0) == Color_Chice) value = 1;
			else value = 0;  
			break;   
		case 5:
			if(PBin(1) == Color_Chice) value = 1;
			else value = 0;  
			break;
	}
	return value; 
}




