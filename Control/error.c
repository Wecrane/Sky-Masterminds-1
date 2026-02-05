#include "myfile.h"
#include "sensor.h"

/************************传感器误差计算***************************/
// 新增结构体和宏定义（放在文件开头）
#define SENSOR_NUM 5									 //传感器数量

/* **************** 结构体 **************** */
typedef struct   //用于定义结构体类型。结构体是一种用户自定义的数据类型，可以将多个数据项（成员）组合在一起。在这里，typedef 是将结构体定义的类型命名为 SensorMap，以便后续使用。
{
    float Angle;  // 传感器位置坐标（线性分布）
    float channel;  // 传感器对应的通道号
} SensorAngle;

typedef struct   //用于定义结构体类型。结构体是一种用户自定义的数据类型，可以将多个数据项（成员）组合在一起。在这里，typedef 是将结构体定义的类型命名为 SensorMap，以便后续使用。
{
    float Angle;  
    float error;  
	float ALPHA;
	uint16_t times;  //放大倍数，归一化到100 -100之间
} FusedAngle; 


static FusedAngle fusedSensor={0,0,0.7,10};  // 创建fusedSensor变量
// 根据传感器物理位置定义坐标（假设线性排列）
static const SensorAngle sensorAngle[SENSOR_NUM] =   //数组是一个常量数组，且只能在定义它的文件或代码段内访问。const 表明数组中的元素值不可修改。
{
    {-1.5, 1}, // L2
    {-0.3, 2}, // L1	
    { 0, 3}, // M 
    { 0.3, 4}, // R1
    { 1.5, 5}  // R2	
};


/* ****************计算传感器偏差 **************** */
float Error_Calcaulate()
{		
	float active_sum = 0;
	int active_count = 0;
	float err=0;
	 for (int i = 0; i < SENSOR_NUM; i++) 		  
	{
		if (digital(sensorAngle[i].channel))  		//digital 的函数，传入传感器的通道号。
			{
                active_sum += sensorAngle[i].Angle; // 累加激活传感器的位置
                active_count++; 				    //在循环中，active_count 会在每个激活的传感器时增加 1：
			}
    }	
		err = active_sum / active_count;	//加权平均		
		err = err*Right_err();	
		if (active_count == 0) return 0;  // 处理无传感器情况
		return err;           
}


/* **************** 陀螺仪辅助计算 **************** */
float get_fused_error(float sensor_err, float gyro_z) 
{ 
		static float last_gyro_z = 0;
		float dt1=0.003f;
	    fusedSensor.Angle += (last_gyro_z + gyro_z) / 2 * dt1;
	    fusedSensor.Angle=Min_Max( fusedSensor.Angle,-10,10);
		if(sensor_err!=0||digital(3)==1) 
		{			
		// 情况A：有传感器信号时，重置积分，重置方式为0.4的传感器0.6的陀螺仪。
			
			fusedSensor.Angle=fusedSensor.Angle * (1-fusedSensor.ALPHA) + (sensor_err*fusedSensor.ALPHA);	
			fusedSensor.error=fusedSensor.Angle * (1-fusedSensor.ALPHA) + (sensor_err*fusedSensor.ALPHA)*fusedSensor.times;		
	    }    		
		
		// 情况B：无传感器信号时，信任陀螺仪
		else
		{
			//fusedSensor.error=fusedSensor.Angle*fusedSensor.times;//没有传感器的时候，陀螺仪为不断积分，误差一直变大	 *fusedSensor.times		
		}				
		last_gyro_z  = gyro_z;		
		fusedSensor.error=Min_Max(fusedSensor.error,-30,30);
		fusedSensor.error=low_pass_filter(fusedSensor.error,0.5);
		return fusedSensor.error;
}


float Right_err()
{
	 if(L2==0&&L1==1&&M==1&&R1==1&&R2==1){return 10.0f;}
	 else if(L2==0&&L1==0&&M==1&&R1==1&&R2==1){return 4.0f;}
	 
     else if(L2==1&&L1==1&&M==1&&R1==1&&R2==0){return 10.0f;}
	 else if(L2==1&&L1==1&&M==1&&R1==0&&R2==0){return 4.0f;}		 
     else {return 1.0f;}
	
}
/**
 * @brief 带初始化的一阶低通滤波器
 * @param input 输入值
 * @param alpha 滤波系数 (0-1)
 * @return 滤波后的输出值
 */

float low_pass_filter(float input, float alpha)
{
    static float output = 0.0f;
    output = alpha * input + (1.0f - alpha) * output;
    return output;
}











