#include "sensor.h"

/*************************************
*�������ƣ�SENSOR_GPIO_Config
*�������ܣ�GPIO�ܽŵ�����
*������
*˵����
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
    
    // 初始化 GPIOA Pin4, Pin5 (L2, L1) 和 Pin8, Pin11, Pin12
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*************************************
*�������ƣ�digtal
*�������ܣ���ȡXͨ������ֵ
*������
*˵����
*			
**************************************/
unsigned char digital(unsigned char channel)//1-ADC_N	  ��ȡXͨ������ֵ
{
	u8 value = 0;
	switch(channel) 
	{
		case 1:  
			if(PAin(4) == 1) value = 1;
			else value = 0;  
			break;  
		case 2: 
			if(PAin(5) == 1) value = 1;
			else value = 0;  
			break;  
		case 3: 
			if(PAin(12) == 1) value = 1;
			else value = 0;  
			break;   
		case 4:  
			if(PBin(0) == 1) value = 1;
			else value = 0;  
			break;   
		case 5:
			if(PBin(1) == 1) value = 1;
			else value = 0;  
			break;
	}
	return value; 
}




