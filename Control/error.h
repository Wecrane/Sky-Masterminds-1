#ifndef __ERROR_H
#define	__ERROR_H

#include "sensor.h"	   
/*-------------���ֶ˿�----------------*/
float Error_Calcaulate(void);
extern int err;
float get_fused_error(float sensor_err, float gyro_z);
float Right_err(void);
float low_pass_filter(float input, float alpha);
#endif 
