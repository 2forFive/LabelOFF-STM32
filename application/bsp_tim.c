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
	
#include "tim.h"
#include "bsp_tim.h"

volatile uint32_t TimerCnt;


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void delay_Init()
{
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start(&htim5);
}


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
uint32_t Get_SystemTimer_us(void)
{
	return htim5.Instance->CNT + TimerCnt * 0x4E20;
}

/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
uint32_t Get_SystemTimer_s(void)
{
	return Get_SystemTimer_us() / 1000000.0f;
}

/**
	* @brief          ms
	* @param		      a: xxx
  * @retval         
  */
uint32_t Get_SystemTimer(void)
{
	return Get_SystemTimer_us() / 1000.0f;
}

