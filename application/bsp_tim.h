/**
  ******************************************************************************
  * @file       bsp_tim.h
	* @author			sxx
  * @brief      global timer
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-19-2021     1. done
	*
  ******************************************************************************
  */

#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "struct_typedef.h"

/* global system time cnter */
extern volatile uint32_t TimerCnt;


/* counter typedef */
// not enabled
typedef struct
{
	float moment;
	float duration;
} counter_t;


/* function prototypes */
void delay_Init(void);
uint32_t Get_SystemTimer(void);
uint32_t Get_SystemTimer_s(void);

#endif
