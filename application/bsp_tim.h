/**
  ******************************************************************************
  * @file       
	* @author			sxx
  * @brief      
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-10-2021     1. 
  * @todo				1. 
	*
  ******************************************************************************
  */

#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "struct_typedef.h"

//system time cnt
extern volatile uint32_t TimerCnt;

typedef struct
{
	float moment;
	float duration;
} counter_t;

void delay_Init(void);
//float Get_SystemTimer(void);
uint32_t Get_SystemTimer(void);
uint32_t Get_SystemTimer_s(void);

#endif
