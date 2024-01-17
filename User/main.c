#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "Kalman.h"
#include "timer.h"
#include "PWM.h"

u16 i;
int main(void)
{
	
	OLED_Init();
	MPU6050_Init();
	PWM_Init();
	
	TIM4_Int_Init(199,7199);//get the data every 20ms
	OLED_ShowString(1,1,"pitch:");
	OLED_ShowString(2,1,"roll:");
	
	
	while(1)
	{
		OLED_ShowSignedNum(1,7,(int32_t)Angle_X_Final,32);
		OLED_ShowSignedNum(2,7,(int32_t)Angle_Y_Final,32);
		if (Angle_X_Final>30.0 | Angle_X_Final<-30.0|Angle_Y_Final>30.0|Angle_Y_Final<-30.0)
		{
			OLED_ShowString(3,1,"Warning:Fall");
			for(i=0;i<=100;i++){
				PWM_SetCompare1(i);
				Delay_ms(10);
			}
			for(i=0;i<=100;i++){
				PWM_SetCompare1(100-i);
				Delay_ms(10);
			}
		}
	}
} 
