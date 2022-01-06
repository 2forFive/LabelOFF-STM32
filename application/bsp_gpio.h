/**
  ******************************************************************************
  * @file       bsp_gpio.h
	* @author			sxx
  * @brief      relay struct typedef
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-14-2021     1. done
	*	 V1.1.0			Dec-21-2021			1. more reasonable
	*
  ******************************************************************************
  */
	
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "struct_typedef.h"
#include "gpio.h"

typedef struct
{
	GPIO_TypeDef	*port;		// GPIO port
	uint16_t			pin;			// GPIO pin
	
	GPIO_PinState status;		// pin status, SET or RESET
	
	uint16_t			on_times; // how many times has the relay/hotline been turned on
	uint16_t 			mom_set;	// whether the turn on moment was set
	float					moment;		// the turn on moment
	float					duration; // how long the relay/hotline has been turned on
} switch_hotline_t;

/* function prototypes */
void switch_hotline_set_status(switch_hotline_t *switch_hotline, GPIO_PinState status);

#endif
