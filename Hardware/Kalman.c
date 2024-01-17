#include "stm32f10x.h"                  // Device header
#include "Kalman.h"
#include "MPU6050.h"
#include "math.h"

float accx,accy,accz; //accelarator
float gyrox,gyroy,gyroz; //GYRO
float Accel_x,Accel_y,Accel_z; //data storation
float Gyro_x,Gyro_y,Gyro_z;
float Angle_x_temp,Angle_y_temp;
float Angle_X_Final,Angle_Y_Final;

void Angle_Calcu(void)
{
	//1.read the data
	int16_t AX,AY,AZ,GX,GY,GZ;
	float accx,accy,accz;
	MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);
	accx=(float)AX;
	accy=(float)AY;
	accz=(float)AZ;
	gyrox=(float)GX;
	gyroy=(float)GY;
	gyroz=(float)GZ;
	Accel_x=accx;
	Accel_y=accy;
	Accel_z=accz;
	Gyro_x =gyrox;
	Gyro_y =gyroy;
	Gyro_z =gyroz;
	//2.accl process
	if (Accel_x<32764) accx=Accel_x/16384; //calc x accx
	else							 accx=1-(Accel_x-49152)/16384;
	if (Accel_y<32764) accy=Accel_y/16384; //calc y accx
	else							 accy=1-(Accel_y-49152)/16384;
	if (Accel_z<32764) accz=Accel_z/16384; //calc z accx
	else							 accz=1-(Accel_z-49152)/16384;
	//calc angle with table
	Angle_x_temp=(atan(accy/accz))*180/3.14;
	Angle_y_temp=(atan(accx/accz))*180/3.14;
	//get +-
	if (Accel_x<32764) Angle_y_temp=+Angle_y_temp;
	if (Accel_x>32764) Angle_y_temp=-Angle_y_temp;
	if (Accel_y<32764) Angle_x_temp=+Angle_x_temp;
	if (Accel_y>32764) Angle_x_temp=-Angle_x_temp;
	//3.gyro process
	if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);
	if(Gyro_x>32764) Gyro_x=+(65535-Gyro_x)/16.4;
	if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);
	if(Gyro_y>32764) Gyro_y=+(65535-Gyro_y)/16.4;
	if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);
	if(Gyro_z>32764) Gyro_z=+(65535-Gyro_z)/16.4;
	//4.use Kalman_Filter
	Kalman_Filter_X(Angle_x_temp,Gyro_x);
	Kalman_Filter_Y(Angle_y_temp,Gyro_y);
}

//Kalman parameter
float Q_angle=0.001; //angle noise covariance
float Q_gyro=0.003;  //gyro noise covariance
float R_angle=0.5;
float dt=0.02;
char C_0=1;
float Q_bias,Angle_err;
float PCt_0,PCt_1,E;
float K_0,K_1,t_0,t_1;
float P[4]={0,0,0,0};
float PP[2][2]={{1,0},{0,1}};

void Kalman_Filter_X(float Accel,float Gyro)
{
	//step 1,X(k|k-1)=AX(k-1|k-1_+BU(k)
	//X=(Angle,Q_bias)
	//A(1,1)=1,A(1,2)=-dt,A(2,1)=0,A(2,2)=1
	Angle_X_Final=Angle_X_Final+(Gyro-Q_bias)*dt;
	//step2,P(k|k-1)=AP(k-1|k-1)A^T+Q
	//Q(1,1)=cov(Angle,Angle) Q(1,2)=cov(Q_bias,Angle)
	//Q(2,1)=cov(Angle,Q_bias)Q(2,2)=cov(Q_bias,Q_bias)
	P[0]=Q_angle-PP[0][1]-PP[1][0];
	P[1]=-PP[1][1];
	P[2]=-PP[1][1];
	P[3]=Q_gyro;
	PP[0][0] += P[0]*dt;
	PP[0][1] += P[1]*dt;
	PP[1][0] += P[2]*dt;
	PP[1][1] += P[3]*dt;
	//step3,Kg(k)=P(k|k-1)H^T/(HP(k|k-1)H^T+R)
	//Kg=(K_0,K_1) H=(1,0)
	PCt_0=C_0*PP[0][0];
	PCt_1=C_0*PP[1][0];
	E=R_angle+C_0*PCt_0;
	K_0=PCt_0/E;
	K_1=PCt_1/E;
	//step4,P(k|k)=(I-Kg(k)H)P(k|k-1_
	t_0=PCt_0;
	t_1=C_0*PP[0][1];
	PP[0][0]-=K_0*t_0;
	PP[0][1]-=K_0*t_1;
	PP[1][0]-=K_1*t_0;
	PP[1][1]-=K_1*t_1;
	//step5,X(k|k)=X(K|k-1_+Kg(k)(Z(k)-X(k|k-1__
	Angle_err=Accel-Angle_X_Final;
	Angle_X_Final+=K_0*Angle_err;
	Q_bias+=K_1*Angle_err;
	Gyro_x=Gyro-Q_bias;
}

void Kalman_Filter_Y(float Accel,float Gyro)
{
	//step 1,X(k|k-1)=AX(k-1|k-1_+BU(k)
	//X=(Angle,Q_bias)
	//A(1,1)=1,A(1,2)=-dt,A(2,1)=0,A(2,2)=1
	Angle_Y_Final+=(Gyro-Q_bias)*dt;
	//step2,P(k|k-1)=AP(k-1|k-1)A^T+Q
	//Q(1,1)=cov(Angle,Angle) Q(1,2)=cov(Q_bias,Angle)
	//Q(2,1)=cov(Angle,Q_bias)Q(2,2)=cov(Q_bias,Q_bias)
	P[0]=Q_angle-PP[0][1]-PP[1][0];
	P[1]=-PP[1][1];
	P[2]=-PP[1][1];
	P[3]=Q_gyro;
	PP[0][0] += P[0]*dt;
	PP[0][1] += P[1]*dt;
	PP[1][0] += P[2]*dt;
	PP[1][1] += P[3]*dt;
	//step3,Kg(k)=P(k|k-1)H^T/(HP(k|k-1)H^T+R)
	//Kg=(K_0,K_1) H=(1,0)
	Angle_err=Accel-Angle_Y_Final;
	PCt_0=C_0*PP[0][0];
	PCt_1=C_0*PP[1][0];
	E=R_angle+C_0*PCt_0;
	K_0=PCt_0/E;
	K_1=PCt_1/E;
	//step4,P(k|k)=(I-Kg(k)H)P(k|k-1_
	t_0=PCt_0;
	t_1=C_0*PP[0][1];
	PP[0][0]-=K_0*t_0;
	PP[0][1]-=K_0*t_1;
	PP[1][0]-=K_1*t_0;
	PP[1][1]-=K_1*t_1;
	//step5,X(k|k)=X(K|k-1_+Kg(k)(Z(k)-X(k|k-1)
	Angle_Y_Final+=K_0*Angle_err;
	Q_bias+=K_1*Angle_err;
	Gyro_y=Gyro-Q_bias;
}
