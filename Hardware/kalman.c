#include "stm32f10x.h"      
#include "MyI2C.h"
#include "MPU6050_Reg.h"
#include "MPU6050.h"
#include <math.h>
#include "kalman.h"



#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
static float Q_angle = 0.001;		//角度数据置信度，角度噪声的协方差
static float Q_gyro  = 0.003;		//角速度数据置信度，角速度噪声的协方差  
static float R_angle = 0.5;			//加速度计测量噪声的协方差
static float dt      = 0.01;			//采样周期即计算任务周期10ms                                                            
float pitch_kalman = 0.0f;    // 估计的俯仰角


double my_sqrt(double x) 
	{
    if (x < 0) {
        return -1;  // 返回 -1 表示无法计算负数的平方根
    }
    
    double guess = x / 2.0;  // 初始猜测
    double epsilon = 0.00001;  // 精度要求

    while (1) {
        double next_guess = (guess + x / guess) / 2.0;
        if (fabs(guess - next_guess) < epsilon) {
            break;  // 达到精度要求，退出循环
        }
        guess = next_guess;
    }

    return guess;
}

double my_atan2(double y, double x) 
	{
    if (x > 0) {
        return atan(y / x);  // 第一象限和第四象限
    } else if (x < 0) {
        if (y >= 0) {
            return atan(y / x) + M_PI;  // 第二象限
        } else {
            return atan(y / x) - M_PI;  // 第三象限
        }
    } else {
        if (y > 0) {
            return M_PI / 2;  // 正Y轴方向
        } else if (y < 0) {
            return -M_PI / 2;  // 负Y轴方向
        } else {
            return 0;  // 原点
        }
    }
		

}



//float  Kalman_Cal_pitch(float acc_x, float acc_y, float acc_z, float gyro) //卡尔曼滤波roll轴计算				
//{
//	
//	float pitch_hudu=pitch*(3.14159265/ 180.0);
//		
//	
//	// 预测步骤
//	pitch_kalman += (gyro - Q_bias) * dt;

//// 更新协方差矩阵PP = F*PP*F^T + Q
//	float dt2 = dt * dt;
//	PP[0][0] = PP[0][0] - dt*(PP[0][1] + PP[1][0]) + dt2*PP[1][1] + Q_angle*dt;
//	PP[0][1] = PP[0][1] - dt*PP[1][1];
//	PP[1][0] = PP[0][1]; // 保持对称
//	PP[1][1] = PP[1][1] + Q_gyro*dt;
//	
//	// 更新步骤
//  // 计算卡尔曼增益
//	K_0 = PP[0][0] / (PP[0][0] + R_angle);
//	K_1 = PP[1][0] / (PP[0][0] + R_angle);
//	
//	// 更新估计值
//	pitch_kalman = pitch_kalman + K_0 * (pitch_hudu - pitch_kalman);
//	Q_bias = Q_bias + K_1 * (pitch_hudu - pitch_kalman);
//	
//	// 更新协方差矩阵
//	PP[0][0] = PP[0][0] - K_0 * PP[0][0];
//	PP[0][1] = PP[0][1] - K_0 * PP[0][1];
//	PP[1][0] = PP[1][0] - K_1 * PP[0][0];
//	PP[1][1] = PP[1][1] - K_1 * PP[0][1];
//		
//		

//		
//		return pitch_kalman * (180.0 / 3.14159265); // 将弧度转换回角度并返回
////		return pitch_kalman; // 返回估计的俯仰角，单位为角度
//		
//		
//	}

void Kalman_Cal_Pitch(float acc,float gyro) //卡尔曼滤波roll轴计算				
{
	static float Q_bias;	//Q_bias:陀螺仪的偏差  Angle_err:角度偏量 
	static float K_0, K_1;	//卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };//过程协方差矩阵P，初始值为单位阵
	pitch_kalman += (gyro - Q_bias) * dt; //状态方程,角度值等于上次最优角度加角速度减零漂后积分
	PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0])*dt;
	PP[0][1] = PP[0][1] - PP[1][1]*dt;
	PP[1][0] = PP[1][0] - PP[1][1]*dt;
	PP[1][1] = PP[1][1] + Q_gyro;
	K_0 = PP[0][0] / (PP[0][0] + R_angle);
	K_1 = PP[1][0] / (PP[0][0] + R_angle);
	pitch_kalman = pitch_kalman + K_0 * (acc - pitch_kalman);
	Q_bias = Q_bias + K_1 * (acc - pitch_kalman);
	PP[0][0] = PP[0][0] - K_0 * PP[0][0];
	PP[0][1] = PP[0][1] - K_0 * PP[0][1];
	PP[1][0] = PP[1][0] - K_1 * PP[0][0];
	PP[1][1] = PP[1][1] - K_1 * PP[0][1];
			
}

