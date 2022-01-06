/**
  ******************************************************************************
  * @file       servo_pwm.c
	* @author			sxx
  * @brief      pwm servo functions: getting angle data, initiating the servo, 
  *             and changing CCR to change angle.
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-9-2021      1. done
	*
  ******************************************************************************
  */

#include "servo_pwm.h"

/* function prototypes */
uint16_t servo_pwm_set_angle_direct(servo_pwm_t* servo_pwm, uint16_t set);

/**
  * @brief          return the pwm servo angle data
  * @param		      servo_pwm: controlled pwm servo point
  * @retval         pwm servo angle data
  */
uint16_t servo_pwm_get_angle(servo_pwm_t *servo_pwm)
{
	switch(servo_pwm->ch)
	{
		case TIM_CHANNEL_1:
			servo_pwm->angle_get = (servo_pwm->tim)->Instance->CCR1;
			break;
		case TIM_CHANNEL_2:
			servo_pwm->angle_get = (servo_pwm->tim)->Instance->CCR2;
			break;
		case TIM_CHANNEL_3:
			servo_pwm->angle_get = (servo_pwm->tim)->Instance->CCR3;
			break;
		case TIM_CHANNEL_4:
			servo_pwm->angle_get = (servo_pwm->tim)->Instance->CCR4;
			break;
	}
	return 1;
}


/**
  * @brief          initiate the pwm servo
  * @param		      servo_pwm: controlled pwm servo point
  * @retval         none
  */
void servo_pwm_init(servo_pwm_t *servo_pwm, 
												uint16_t id, 
												TIM_HandleTypeDef *tim, 
												uint16_t ch, 
												uint16_t angle_default, 
												uint16_t angle_min, 
												uint16_t angle_max, 
												uint16_t time )
{
	// value assignment
	servo_pwm->id  = id;
	servo_pwm->tim = tim;
	servo_pwm->ch  = ch;
	
	servo_pwm->angle_default = angle_default;
	servo_pwm->angle_min		 = angle_min;
	servo_pwm->angle_max 		 = angle_max;
	
	servo_pwm->time					 = time;
	
	// start TIM pwm generation
	HAL_TIM_PWM_Start(tim, ch);
	
	// set the servo to the default position
	servo_pwm_set_angle_direct(servo_pwm, angle_default);
	
	// init angle info
	servo_pwm_get_angle(servo_pwm);
}


/**
	* @brief          change the pwm servo ccr; go to target angle directly
  * @param		      servo_pwm: controlled pwm servo point
	* @param		      set: target angle
  * @retval         boolean, done or not
  */
uint16_t servo_pwm_set_angle_direct(servo_pwm_t* servo_pwm, uint16_t set)
{
	servo_pwm_get_angle(servo_pwm);
	__HAL_TIM_SET_COMPARE(servo_pwm->tim, servo_pwm->ch, set);
	HAL_Delay(2);
	
	return 0;
}


/**
  * @brief          change the pwm servo ccr; go to target angle step by step according to step value
  * @param		      servo_pwm: controlled pwm servo point
	* @param		      set: target angle
	* @param		      step: value of a unit step
  * @retval         done or not
  */
uint16_t servo_pwm_set_angle_bySteps(servo_pwm_t* servo_pwm, uint16_t set, uint16_t step)
{
	/* get current angle */
	servo_pwm_get_angle(servo_pwm);
	
	uint16_t i=0;
	
	// clockwise
	if(servo_pwm->angle_get < set)
	{
		for(i=servo_pwm->angle_get ; i<=(set-step) ; i+=step)
		{
			__HAL_TIM_SET_COMPARE(servo_pwm->tim, servo_pwm->ch, i);
			HAL_Delay(2);
		}
		if(i<set)
		{
			__HAL_TIM_SET_COMPARE(servo_pwm->tim, servo_pwm->ch, set);
			HAL_Delay(2);
		}
		return 1;
	}
	// counterclockwise
	else if(servo_pwm->angle_get > set)
	{
		for(i=servo_pwm->angle_get ; i>=(set+step) ; i-=step)
			{
				__HAL_TIM_SET_COMPARE(servo_pwm->tim, servo_pwm->ch, i);
				HAL_Delay(2);
			}
			if(i>set)
			{
				__HAL_TIM_SET_COMPARE(servo_pwm->tim, servo_pwm->ch, set);
				HAL_Delay(2);
			}
			return 2;
	}
	// get equals target
	else
	{
		return 0;
	}

}

