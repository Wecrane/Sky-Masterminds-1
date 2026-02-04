#include "myfile.h"
#include "sensor.h"

/************************参数配置***************************/
#define SENSOR_NUM 5									 //传感器数量
#define SENSOR_MAX_ERR      50    		// 最大限制误差
#define GYRO_SCALE       	  3.5f 			// 
#define FUSION_ALPHA        0.92f		  // 
#define DT                  0.005f 	 	// 
#define SCALE_FACTOR 10							  // 

//电路图的权重（旧代码保留）
#define L22  -2.8*L2
#define L11  -1.3*L1
#define MM   0*M
#define R11  1.3*R1
#define R22  2.8*R2

/* **************** 结构体 **************** */
typedef struct 
{
    float integrated_err; 
    int last_sensor_err;  
} FusionState;

typedef struct   
{
    float position;  
    float channel;  
} SensorMap;

/* **************** 全局变量 **************** */
static FusionState fstate = {0, 0};
int i1,i2,i3,i4,i5,err=0;

// 传感器通道与权重映射
// channel 对应 digital() 函数的参数: 1=L2, 2=L1, 3=M, 4=R1, 5=R2
static const SensorMap sensorMap[SENSOR_NUM] = 
{
    {-6.0, 1}, // L2 (已增加外侧权重，便于直角入弯)
    {-1.3, 2}, // L1	
    { 0, 3},   // M 
    { 1.3, 4}, // R1
    { 6.0, 5}  // R2 (已增加外侧权重)
};


/* **************** 误差计算核心算法 **************** */
int Error_Calcaulate()
{		
	int active_sum = 0;
	int active_count = 0;
    int outer_sensor_triggered = 0; // 标记是否有外侧传感器触发

	for (int i = 0; i < SENSOR_NUM; i++) 		
	{
        // digital返回1代表检测到黑线
        if (digital((unsigned char)sensorMap[i].channel)) 
        {
            active_sum += (int)sensorMap[i].position; 
            active_count++; 

            // 检查是否为最外侧传感器 (L2 index=0 或 R2 index=4)
            if (i == 0 || i == 4) {
                outer_sensor_triggered = 1;
            }
        }
    }
	
    // 基础加权平均
    err = active_count ? (active_sum * SCALE_FACTOR) / active_count : 0;
    
    // --- 直角/急弯策略 ---
    // 如果外侧传感器触发，说明处于弯道边缘，需要极大的转向力。
    // 非线性放大误差
    if (outer_sensor_triggered)
    {
        // 放大倍数，确保能够产生足够的差速
        err = (int)(err * 2.0f); 
    }
    
	return err;           
}


/* **************** 这个函数保留原样，虽然只返回 sensor_err **************** */
int get_fused_error(int sensor_err, float gyro_z) 
{
    // 目前此函数仅透传 sensor_err，如果需要陀螺仪融合可以在这里打开逻辑
	return sensor_err;
}

/* **************** 兼容性保留 **************** */
float Right_err()
{
    // 逻辑已经移入 Error_Calcaulate
	return 1.0f;
}












