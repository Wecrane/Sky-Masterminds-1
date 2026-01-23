#include "myfile.h"
#include "sensor.h"

/************************传感器误差计算***************************/
// 新增结构体和宏定义（放在文件开头）
#define SENSOR_NUM 5									 //传感器数量
#define SENSOR_MAX_ERR      50    		// 传感器最大误差
#define GYRO_SCALE       	  3.5f 			// 角速度→误差缩放系数 (示例: 1°/s → 0.08 error)
#define FUSION_ALPHA        0.92f		  // 互补滤波系数(0.9-0.98)
#define DT                  0.005f 	 	// 控制周期(5ms)
#define SCALE_FACTOR 10							  // 缩放因子
//五路光电管的权重
#define L22  -2.8*L2
#define L11  -1.3*L1
#define MM   0*M
#define R11  1.3*R1
#define R22  2.8*R2

/* **************** 结构体 **************** */
typedef struct 
{
    float integrated_err; // 陀螺仪积分误差
    int last_sensor_err;  // 上次传感器误差
} FusionState;
typedef struct   //用于定义结构体类型。结构体是一种用户自定义的数据类型，可以将多个数据项（成员）组合在一起。在这里，typedef 是将结构体定义的类型命名为 SensorMap，以便后续使用。
{
    float position;  // 传感器位置坐标（线性分布）
    float channel;  // 传感器对应的通道号
} SensorMap;

/* **************** 全局变量 **************** */
static FusionState fstate = {0, 0};
int i1,i2,i3,i4,i5,err=0;

// 根据传感器物理位置定义坐标（假设线性排列）
static const SensorMap sensorMap[SENSOR_NUM] =   //数组是一个常量数组，且只能在定义它的文件或代码段内访问。const 表明数组中的元素值不可修改。
{
    {-2.8, 1}, // L2
    {-1.3, 2}, // L1	
    { 0, 3}, // M 
    { 1.3, 4}, // R1
    { 2.8, 5}  // R2	
};


/* ****************计算传感器偏差 **************** */
int Error_Calcaulate()
{		
	int active_sum = 0;
	int active_count = 0;
	 for (int i = 0; i < SENSOR_NUM; i++) 		
	{
            if (digital(sensorMap[i].channel))  		//digital 的函数，传入传感器的通道号。
						{
                active_sum += sensorMap[i].position; // 累加激活传感器的位置
                active_count++; 										 //在循环中，active_count 会在每个激活的传感器时增加 1：
						}
   }
	
				err = active_count ? (active_sum * SCALE_FACTOR) / active_count : 0;	//这里是最终输出err后然后限幅，只是比较复杂的写法
				err = err*Right_err();				//直角偏差放大
	return err;           
}



/* **************** 陀螺仪辅助计算 **************** */
int get_fused_error(int sensor_err, float gyro_z) 
{
//    /**** 步骤1：陀螺仪积分处理 ****/
//	
//    float delta_err = gyro_z*GYRO_SCALE;// 将角速度放大
//    
//    // 积分计算（假设调用周期为dt秒）
//    float dt = 0.01f; // 1ms周期（我的实际周期是5ms这里我希望他积分快一点）
//    fstate.integrated_err += delta_err * dt;
//    
//    /**** 步骤2：互补滤波融合 ****/
//    int fused_err = 0;
//    if( sensor_err!=0||digital(3)==1) 
//		{			
//				// 情况A：有传感器信号时，重置积分，重置方式为0.4的传感器0.6的陀螺仪。
//        fstate.integrated_err = fstate.integrated_err * 0.6f + sensor_err * 0.4f;
////		fstate.last_sensor_err = sensor_err;		
//		fused_err=(1-FUSION_ALPHA)* fstate.integrated_err + FUSION_ALPHA*sensor_err;
//    } 
//		
//				// 情况B：无传感器信号时，信任陀螺仪
//		else
//		{
//				fused_err= fstate.integrated_err;			//没有传感器的时候，陀螺仪为不断积分，误差一直变大
//		}
//		 /**** 步骤4：限幅处理 ****/
//			//和if的算法是一样的只是这样写更简便
//			fused_err = (fused_err > SENSOR_MAX_ERR) ? SENSOR_MAX_ERR : ((fused_err < -SENSOR_MAX_ERR) ? -SENSOR_MAX_ERR : fused_err);

//    return (int)fused_err;
	return sensor_err;
}

/* **************** 直角放大偏差 **************** */
float Right_err()
{
	if(R2==0&&M==1&&L1==1&&L2==1)//左直角
		{return 27.0f;}
	else if(L2==0&&M==1&&R1==1&&R2==1)//右直角
		{return 27.0f;}
	else 
		{return 1.0f;}
}












