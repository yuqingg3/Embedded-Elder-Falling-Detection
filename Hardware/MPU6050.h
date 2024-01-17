#ifndef __MPU6050_H
#define __MPU6050_H


void MPU6050WriteReg(u8 RegAddress,u8 Data);
u8 MPU6050_ReadReg(u8 RegAddress);
void MPU6050_Init(void);
u8 MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ);
#endif
