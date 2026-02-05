#include "myfile.h"
#include <math.h>
float pitch,roll,yaw;
int16_t AX, AY, AZ, GX, GY, GZ;
// ȫ�ֱ���
float gyro_zero_z = 0.0f;
float yaw_angle = 0.0f;
float last_gyro_z = 0.0f;
float dt = 0.010f;
float gyro_z=0.0f;
#define MPU6050_ADDRESS		0xD0
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(Data);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	Data = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();
	
	return Data;
}

void MPU6050_Init(void)
{
	MyI2C_Init();
	Delay_ms(100);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
	Delay_ms(2000);
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *GyroZ )
{
	// �� MPU6050 ��ȡԭʼ���ٶȺͽ��ٶ�����
uint8_t DataH, DataL;

//// ��ȡ���ٶ�ֵ
//DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
//DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
//*AccX = (DataH << 8) | DataL;


//DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
//DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
//*AccY = (DataH << 8) | DataL;


//DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
//DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
//*AccZ = (DataH << 8) | DataL;

//// ��ȡ���ٶ�ֵ
//DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
//DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
//*GyroX = (DataH << 8) | DataL;


//DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
//DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
//*GyroY = (DataH << 8) | DataL;


DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
*GyroZ= ((DataH << 8) | DataL);	//ʵ�ʽ��ٶ�
		
}

void Clear_yaw(void) 
{
    yaw = 0.0f;  // 直接清零
}

// 陀螺仪零点校准 - 采集多次取平均值
void calibrate_gyro(void)
{
    float sum = 0;
    int16_t gz_raw; 
    int valid_count = 0;
    
    // 采集100次数据取平均（约1秒）
    for(int i = 0; i < 100; i++) 
    {    
        MPU6050_GetData(&gz_raw);
        sum += gz_raw;
        valid_count++;
        Delay_ms(10);  // 等待10ms
    }
    
    // 计算零点偏移
    if(valid_count > 0)
    {
        gyro_zero_z = sum / valid_count;
    }
    
    // 清除yaw
    yaw = 0.0f;
    last_gyro_z = 0.0f;
}

// ��ȡ���ٶȲ�����
void update_yaw(void)
{
    static int16_t gz_raw;
    
    MPU6050_GetData(&gz_raw);
    
    // ������Ư
    gyro_z =(((float)gz_raw - gyro_zero_z) / 16.4f);
    
    // ����
    if(fabs(gyro_z) < 0.5f) gyro_z = 0;
      
    // ���λ���
    yaw += (last_gyro_z + gyro_z) / 2 * dt;
	
    last_gyro_z = gyro_z;
    
}


// ����1���򵥵�ѭ����ʱ������ȷ���ȶ���
void Simple_Delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms; i++) {
        for(uint32_t j = 0; j < 7200; j++) 
		{  // �������ֵ��У׼
            __asm__("nop");
        }
    }

}


