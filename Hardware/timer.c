#include "stm32f10x.h"                  // Device header
#include "timer.h"
#include "Kalman.h"

void TIM4_Int_Init(u16 arr,u16 psc)
{
	//open the clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	//select internal clock
	TIM_InternalClockConfig(TIM4);
	//timebase
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //1/2/4
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //up/down/centeralinged123
	TIM_TimeBaseInitStructure.TIM_Period= arr;//CK_CNT_OV=CK_CNT72000000/ARR+1/PSC+1
	TIM_TimeBaseInitStructure.TIM_Prescaler =psc;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	//Enable Interrrupt
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	//NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_Init(&NVIC_InitStructure);
	
	//Start cmd
	TIM_Cmd(TIM4,ENABLE);
}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		Angle_Calcu();
	}
}
