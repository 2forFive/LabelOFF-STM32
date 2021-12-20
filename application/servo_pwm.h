/**
  ******************************************************************************
  * @file       servo_pwm.h
	* @author			sxx
  * @brief      pwm servo typedef, including extern functions from .c
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-9-2021      1. typedef done
  * @todo				1. adding extern functions
	*
  ******************************************************************************
  */

#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include "struct_typedef.h"
#include "tim.h"

typedef struct
{
	uint16_t id;
	TIM_HandleTypeDef *tim;
	uint16_t ch;
	
	uint16_t angle_min;
	uint16_t angle_max;
	uint16_t angle_default;
	
	uint16_t time;
	
	uint16_t angle_get;
} servo_pwm_t;


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void servo_pwm_init(servo_pwm_t *servo_pwm, 
												uint16_t id, 
												TIM_HandleTypeDef *tim, 
												uint16_t ch, 
												uint16_t angle_default, 
												uint16_t angle_min, 
												uint16_t angle_max, 
												uint16_t time );
uint16_t servo_pwm_set_angle_direct(servo_pwm_t* servo_pwm, uint16_t set);
uint16_t servo_pwm_get_angle(servo_pwm_t *servo_pwm);

#endif
