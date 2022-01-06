/**
  ******************************************************************************
  * @file       bsp_tim.c
	* @author			sxx
  * @brief      global timer
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-19-2021     1. done
	*
  ******************************************************************************
  */
	
#include "tim.h"
#include "bsp_tim.h"

/* global system time cnter */
volatile uint32_t TimerCnt;


/**
	* @brief          init the TIM
	* @param		      none
  * @retval         none
  */
void delay_Init()
{
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start(&htim5);
}


/**
	* @brief          get time in us
	* @param		      none
  * @retval         time in us
  */
uint32_t Get_SystemTimer_us()
{
	return htim5.Instance->CNT + TimerCnt * 0x4E20;
}

/**
	* @brief          get time in s
	* @param		      none
  * @retval         time in s
  */
uint32_t Get_SystemTimer_s()
{
	return Get_SystemTimer_us() / 1000000.0f;
}

/**
	* @brief          time in ms
	* @param		      none
  * @retval         time in ms
  */
uint32_t Get_SystemTimer()
{
	return Get_SystemTimer_us() / 1000.0f;
}

