/**
  ******************************************************************************
  * @file       servo_bus.h
	* @author			sxx
  * @brief      
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-10-2021      1. 
  * @todo				1. 
	*
  ******************************************************************************
  */

#ifndef SERVO_BUS_H
#define SERVO_BUS_H
#include "struct_typedef.h"
#include "usart.h"

typedef struct
{
	uint16_t id;
	UART_HandleTypeDef* uart;
	
	uint16_t angle_default;
	uint16_t angle_work;
	uint16_t time;
	
	uint16_t angle_get;
} servo_bus_t;

#endif
