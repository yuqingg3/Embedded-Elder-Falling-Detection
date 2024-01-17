#include "stm32f10x.h"                  // Device header
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS 0xD0

void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	u32 Timeout;
	Timeout=10000;
	while (I2C_CheckEvent(I2Cx,I2C_EVENT)!= SUCCESS)
	{
		Timeout--;
		if (Timeout==0) break;
	}
}

void MPU6050WriteReg(u8 RegAddress,u8 Data)
{
	//start
	I2C_GenerateSTART(I2C2,ENABLE);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);
	
	//address
	I2C_Send7bitAddress(I2C2,MPU6050_ADDRESS,I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	///DR
	I2C_SendData(I2C2,RegAddress);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	//data
	I2C_SendData(I2C2,Data);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	//stop
	I2C_GenerateSTOP(I2C2,ENABLE);
}

u8 MPU6050_ReadReg(u8 RegAddress)
{
	//Read data frpm the slave Address's Reg Address
	u8 Data; 
	//start
	I2C_GenerateSTART(I2C2,ENABLE);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);
	
	//address
	I2C_Send7bitAddress(I2C2,MPU6050_ADDRESS,I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	///DR
	I2C_SendData(I2C2,RegAddress);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);

	//start 2nd
	I2C_GenerateSTART(I2C2,ENABLE);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);
	
	//receive address
	I2C_Send7bitAddress(I2C2,MPU6050_ADDRESS,I2C_Direction_Receiver);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	//set ack to 0 and stop to 1 at last data time
	I2C_AcknowledgeConfig(I2C2,DISABLE);
	I2C_GenerateSTOP(I2C2,ENABLE);
	
	//wait 7-1
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data=I2C_ReceiveData(I2C2);
	I2C_AcknowledgeConfig(I2C2,ENABLE);
	
	return Data;
}

void MPU6050_Init(void)
{
	//OPEN CLOCK
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	//GPIO PIN
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;// hardware--AF  OD--I2C
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//I2C
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;	
	I2C_InitStructure.I2C_ClockSpeed=50000;
	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1=0x00;
	I2C_Init(I2C2,&I2C_InitStructure);
	
	//cmd
	I2C_Cmd(I2C2,ENABLE);
}

u8 MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ)
{
	u8 DataH,DataL;
	
	DataH=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX=(DataH<<7)|DataL;

	DataH=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY=(DataH<<7)|DataL;
	
	DataH=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ=(DataH<<7)|DataL;
	
	DataH=MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX=(DataH<<7)|DataL;       
                                
	DataH=MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY=(DataH<<7)|DataL;       
	                              
	DataH=MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL=MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ=(DataH<<7)|DataL;
}
