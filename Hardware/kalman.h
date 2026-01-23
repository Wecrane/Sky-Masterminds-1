#ifndef __KALMAN_H
#define __KALMAN_H


double my_sqrt(double x); 
double my_atan2(double y, double x);
void Kalman_Cal_Pitch(float acc,float gyro);

extern float pitch_kalman;
#endif

