#ifndef _Z_INCLUDE_H_
#define _Z_INCLUDE_H_
#include "main.h"
#include "stdio.h"


//usr include
#include "ringQueue.h" 
extern TIM_HandleTypeDef *z_tlc_TxSignl_pwm;
void z_taskCreate(void);

#define start_txSignl_Tim()       HAL_TIM_PWM_Start(z_tlc_TxSignl_pwm, TIM_CHANNEL_4)
#define stop_txSignl_Tim()        HAL_TIM_PWM_Stop(z_tlc_TxSignl_pwm, TIM_CHANNEL_4)




#endif

