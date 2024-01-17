#ifndef __KALMAN_H
#define __KALMAN_H

extern float Angle_X_Final,Angle_Y_Final;
void Angle_Calcu(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);
#endif
