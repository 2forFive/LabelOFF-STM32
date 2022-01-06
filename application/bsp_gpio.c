/**
  ******************************************************************************
  * @file       bsp_gpio.c
	* @author			sxx
  * @brief      hotline status control function
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-14-2021     1. done
	*
  ******************************************************************************
  */
	
#include "bsp_gpio.h"

/**
	* @brief          set the status of the pin of relay/hotline
	* @param		      switch_hotline: pointer of hotline
	*									target_status: target pin status
  * @retval         none
  */
void switch_hotline_set_status(switch_hotline_t *switch_hotline, GPIO_PinState target_status)
{
	HAL_GPIO_WritePin(switch_hotline->port, switch_hotline->pin, target_status);
	switch_hotline->status = target_status;
	// add turn on times of hotline
	if(target_status == GPIO_PIN_SET) switch_hotline->on_times++;
}

