#include "myfile.h"
uint8_t Serial_RxData;
uint8_t Serial_RxFlag;

// 新增变量
char Serial_RxPacket[100];
uint8_t Serial_RxPacketReady = 0;
static uint8_t Serial_RxPacketIndex = 0;


/************************串口初始化***************************/
void Serial_Init(void)
{
	 // 开启 GPIOB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 开启 USART3 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);    

    GPIO_InitTypeDef GPIO_InitStructure;
    // PB10 TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // PB11 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);

    // 配置 NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART3, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART3, Byte);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;
}

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		char RxData = USART_ReceiveData(USART3);
		
        // 现有的单字节处理
        Serial_RxData = RxData;
		Serial_RxFlag = 1;

        // 字符串包处理 (\r\n 结尾)
        if (RxData == '\n')
        {
            // 去除指令尾部的空格（防止指令为 "?   " 导致匹配失败）
            while (Serial_RxPacketIndex > 0 && Serial_RxPacket[Serial_RxPacketIndex - 1] == ' ')
            {
                Serial_RxPacketIndex--;
            }
            
            Serial_RxPacket[Serial_RxPacketIndex] = '\0'; // 添加字符串结束符
            Serial_RxPacketReady = 1;
            Serial_RxPacketIndex = 0;
        }
        else if (RxData == '\r')
        {
            // 收到 \r 忽略
        }
        else
        {
            // 忽略指令开头的空格（防止上一条指令后的空格污染下一条指令）
            if (Serial_RxPacketIndex == 0 && RxData == ' ')
            {
                // do nothing
            }
            else
            {
                Serial_RxPacket[Serial_RxPacketIndex] = RxData;
                Serial_RxPacketIndex++;
                if(Serial_RxPacketIndex >= 100) Serial_RxPacketIndex = 0; // 防止溢出
            }
        }

		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}
