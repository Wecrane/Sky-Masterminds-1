#include "myfile.h"          // 项目主头文件
#include <math.h>

// 全局变量（供外部访问）
float yaw;                  // 累积偏航角（°）
int16_t AX, AY, AZ, GX, GY, GZ;  // 原始传感器数据（兼容其他模块）

// MPU6050配置参数
#define MPU6050_ADDRESS     0xD0    // I2C地址
#define GYRO_SCALE_FACTOR   16.4f   // ±2000°/s量程下的转换系数（LSB/(°/s)）
static const float dt = 0.05f;    // 采样周期（50ms，与实际控制周期一致）
uint8_t gyro_calibration_done = 0; // 校准完成标志

/**
 * MPU6050写寄存器
 */
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

/**
 * MPU6050读寄存器
 */
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
    MyI2C_SendAck(1);  // 发送非应答
    MyI2C_Stop();
    return Data;
}

/**
 * MPU6050初始化（仅配置Z轴相关参数）
 */
void MPU6050_Init(void)
{
    MyI2C_Init();
    Delay_ms(100);  // 等待传感器上电稳定
    
    // 复位传感器
    MPU6050_WriteReg(0x6B, 0x80);
    Delay_ms(100);
    // 唤醒传感器（使用内部8MHz时钟）
    MPU6050_WriteReg(0x6B, 0x00);
    Delay_ms(10);
    
    // 配置采样率和滤波（针对Z轴优化）
    MPU6050_WriteReg(0x19, 0x31);  // SMPLRT_DIV=49 → 采样率=20Hz（50ms/次）
    MPU6050_WriteReg(0x1A, 0x02);  // CONFIG=2 → DLPF截止94Hz
    MPU6050_WriteReg(0x1B, 0x18);  // GYRO_CONFIG=0x18 → 陀螺仪量程±2000°/s
    MPU6050_WriteReg(0x1C, 0x00);  // ACCEL_CONFIG=0 → 加速度计量程±2g
    
    // 初始化变量
    yaw = 0.0f;
    AX = AY = AZ = GX = GY = GZ = 0;
}

/**
 * 读取Z轴数据并更新yaw（核心函数）
 */
void MPU6050_GetData(void)
{
    uint8_t Data[2];  // 存储Z轴原始数据（高8位+低8位）
    
    // 读取陀螺仪Z轴数据（寄存器0x47=GYRO_ZOUT_H，0x48=GYRO_ZOUT_L）
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(0x47);  // 起始寄存器地址
    MyI2C_ReceiveAck();
    
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);  // 读操作
    MyI2C_ReceiveAck();
    
    // 连续读取2字节
    for (uint8_t i = 0; i < 2; i++) {
        Data[i] = MyI2C_ReceiveByte();
        MyI2C_SendAck((i == 1) ? 1 : 0);  // 最后一字节发非应答
    }
    MyI2C_Stop();
    
    // 解析Z轴原始数据（16位有符号整数）
    GZ = (int16_t)(Data[0] << 8) | Data[1];
    
    // 陀螺仪Z轴零偏校准（首次上电执行）
    static uint8_t gyro_calibrated = 0;
    static float gyro_offset_z = 0.0f;  // Z轴零偏（°/s）
    
    if (!gyro_calibrated) {
        static uint16_t count = 0;
        static float sum_gz = 0.0f;
        
        // 累积100次原始数据（约500ms）
        sum_gz += (float)GZ;
        count++;
        
        if (count >= 100) {
            // 计算零偏（转换为°/s）
            gyro_offset_z = (sum_gz / 100.0f) / GYRO_SCALE_FACTOR;
            gyro_calibrated = 1;
            gyro_calibration_done = 1;  // 校准完成标志置位
            yaw = 0.0f;  // 初始化yaw
        }
        return;  // 校准期间不更新yaw
    }
    
    // 计算Z轴实际角速度（°/s）=（原始数据/转换系数）- 零偏
    float raw_gz_deg = ((float)GZ / GYRO_SCALE_FACTOR) - gyro_offset_z;
    
    // 轻量低通滤波（减少噪声）
    static float filtered_gz = 0.0f;
    filtered_gz = 0.2f * filtered_gz + 0.8f * raw_gz_deg;  // 新数据权重80%
    
    // 积分计算yaw角（累积角度）
    yaw += filtered_gz * dt;
}

/**
 * 清除yaw角（重置累积值）
 */
void Clear_yaw(void)
{
    yaw = 0.0f;
}
