#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
		//open the clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	//PA0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*if use another pin here
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);//to PA15
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//turn PA15 and etc to GPIO
	end if*/
	
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//USE PWM to control
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //if:15
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//select internal clock
	TIM_InternalClockConfig(TIM2);
	//timebase
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //1/2/4
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //up/down/centeralinged123
	TIM_TimeBaseInitStructure.TIM_Period=100-1;//(ARR)CK_CNT_OV=CK_CNT72000000/ARR+1/PSC+1
	TIM_TimeBaseInitStructure.TIM_Prescaler =720-1;//(PSC)
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	
	//Enable OC 
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//Active,Timing,PWM12...
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High ;//High/low
	TIM_OCInitStructure.TIM_OutputState=ENABLE;//ENABLE/DISABLE
	TIM_OCInitStructure.TIM_Pulse=0;//(CCR)0x0000to0xFFFF Duty=CCR/(ARR+1)
	TIM_OC1Init (TIM2,&TIM_OCInitStructure);
	
	
	//Start cmd
	TIM_Cmd(TIM2,ENABLE);
	
}

void PWM_SetCompare1(u16 Compare)
{
	TIM_SetCompare1(TIM2,Compare);
}
