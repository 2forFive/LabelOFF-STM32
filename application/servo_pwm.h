/**
  ******************************************************************************
  * @file       servo_pwm.h
	* @author			sxx
  * @brief      pwm servo typedef, function prototypes
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-9-2021      1. struct typedef
	*
  ******************************************************************************
  */

#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include "struct_typedef.h"
#include "tim.h"

typedef struct
{
	uint16_t id;						// identifier by user
	TIM_HandleTypeDef *tim; // TIM
	uint16_t ch;						// channel
	
	uint16_t angle_min;			// minimum angle
	uint16_t angle_max;			// maximum angle
	uint16_t angle_default; // default position
	
	uint16_t time;					// expectant time of one motion
	
	uint16_t angle_get;			// equal to the corresponding CCR of TIM
} servo_pwm_t;

/* function prototypes */
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
