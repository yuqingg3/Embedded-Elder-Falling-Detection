#ifndef __TIMER_H
#define __TIMER_H

void TIM4_Int_Init(u16 arr,u16 psc);
void TIM4_IRQHandler(void);
#endif
