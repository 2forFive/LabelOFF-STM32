/**
  ******************************************************************************
  * @file       bsp_gpio.h
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
	
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "struct_typedef.h"
#include "gpio.h"

typedef struct
{
	GPIO_TypeDef	*port;
	uint16_t			pin;
	
	GPIO_PinState status;
	
	uint16_t			on_times;
	uint16_t 			mom_set;	//whether 
	float					moment;
	float					duration; // how long it has been turned on
	
} switch_hotline_t;

void switch_hotline_set_status(switch_hotline_t *switch_hotline, GPIO_PinState status);

#endif
