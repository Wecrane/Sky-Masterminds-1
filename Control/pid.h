#ifndef __PID_H
#define __PID_H



#define INTEGRAL_MAX  1000
#define Min_Max(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define Max_PWM 			7200 
void Control(void);
void Different_Speed(void);


int PID_Control_L(int NowPiont, int SetPoint, int *TURN_PID); 
int PID_Control_R(int NowPoint, int SetPoint, int *TURN_PID);
float Place_Control(float NowPoint, float SetPoint, float *TURN_PID); //PD����
extern int Speed_Out_L,Speed_Out_R,Place_Out;
extern int sensor_err,final_err;
extern int Basic_Speed;
extern uint8_t Place_Enable,PWM_Enable;
#endif
