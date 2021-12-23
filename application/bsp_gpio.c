/**
  ******************************************************************************
  * @file       bsp_gpio.c
	* @author			sxx
  * @brief      
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-14-2021     1. 
  * @todo				1. 
	*
  ******************************************************************************
  */
	
#include "bsp_gpio.h"


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void switch_hotline_set_status(switch_hotline_t *switch_hotline, GPIO_PinState target_status)
{
	HAL_GPIO_WritePin(switch_hotline->port, switch_hotline->pin, target_status);
	switch_hotline->status = target_status;
	if(target_status == GPIO_PIN_SET) switch_hotline->on_times++;
}

