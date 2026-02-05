#ifndef __MPU6050_H
#define __MPU6050_H

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *GyroZ);
void update_yaw(void);
void calibrate_gyro(void);
void Clear_yaw(void);
void Simple_Delay_ms(uint32_t ms);
extern float pitch,roll,yaw;
extern int16_t GZ;
extern float gyro_z,dt;
#endif
