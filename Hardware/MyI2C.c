#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void MyI2C_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_4, (BitAction)BitValue);
    Delay_us(5);  // 降低延时，提高通信速度
}

void MyI2C_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_3, (BitAction)BitValue);
    Delay_us(5);  // 降低延时，提高通信速度
}

uint8_t MyI2C_R_SDA(void)
{
    uint8_t BitValue;
    BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
    Delay_us(5);  // 降低延时，提高通信速度
    return BitValue;
}

void MyI2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  // 开漏输出，适合I2C
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//关闭JTAG功能(PB3/4)，只使用SWD(PA13/14)调试
    
    // 释放总线
    GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4);	
}

void MyI2C_Start(void)
{
    MyI2C_W_SDA(1);  // 确保SDA为高
    Delay_us(5);
    MyI2C_W_SCL(1);  // 确保SCL为高
    Delay_us(5);
    MyI2C_W_SDA(0);  // SDA拉低，产生起始信号
    Delay_us(5);
    MyI2C_W_SCL(0);  // SCL拉低，开始传输
    Delay_us(5);
}

void MyI2C_Stop(void)
{
    MyI2C_W_SDA(0);  // 确保SDA为低
    Delay_us(5);
    MyI2C_W_SCL(1);  // SCL拉高
    Delay_us(5);
    MyI2C_W_SDA(1);  // SDA拉高，产生停止信号
    Delay_us(5);
}

void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i ++)
    {
        MyI2C_W_SCL(0);  // 先拉低时钟
        Delay_us(2);
        // 使用掩码的方式取出Byte的指定一位数据并写入到SDA线
        MyI2C_W_SDA(!!(Byte & (0x80 >> i)));
        Delay_us(2);
        MyI2C_W_SCL(1);  // 释放时钟，从机在SCL高电平期间读取SDA
        Delay_us(5);
        MyI2C_W_SCL(0);  // 拉低SCL，主机开始发送下一位数据
        Delay_us(2);
    }
}

uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i, Byte = 0x00;
    MyI2C_W_SDA(1);  // 释放SDA线，准备接收
    for (i = 0; i < 8; i ++)
    {
        MyI2C_W_SCL(0);  // 拉低时钟
        Delay_us(2);
        MyI2C_W_SCL(1);  // 释放时钟
        Delay_us(2);
        if (MyI2C_R_SDA()){Byte |= (0x80 >> i);}  // 读取SDA数据
        Delay_us(3);
    }
    return Byte;
}

void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SCL(0);  // 拉低时钟
    Delay_us(2);
    MyI2C_W_SDA(AckBit);  // 发送应答位
    Delay_us(2);
    MyI2C_W_SCL(1);  // 释放时钟
    Delay_us(5);
    MyI2C_W_SCL(0);  // 拉低时钟
    Delay_us(2);
}

uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t AckBit;
    MyI2C_W_SDA(1);  // 释放SDA线
    Delay_us(2);
    MyI2C_W_SCL(1);  // 释放时钟，准备接收应答
    Delay_us(5);
    AckBit = MyI2C_R_SDA();  // 读取应答位
    MyI2C_W_SCL(0);  // 拉低时钟
    Delay_us(2);
    return AckBit;
}

