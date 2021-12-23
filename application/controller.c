/**
  ******************************************************************************
  * @file       controller.c
	* @author			sxx
  * @brief      All contol tasks: functions
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-11-2021     1. template, outline
	*  V1.1.0			Dec-13-2021			1. add motor functions
  * @todo				1. xx_task
	*							2. more inits
	*							3. situation controller, the one will be in while(1) in main.c
	*							4. static some functions
	*							5. #define, replace some data
	*
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>

#include "controller.h"

/** @todo Function prototypes except for the main init & task */
void motor_task(controller_t *controller, uint8_t front, uint8_t middle, uint8_t end);
void servo_cutter_task(servo_pwm_t *servo_cutter, servo_cutter_angle_e target);
void servo_pusher_task(servo_pwm_t *servo_pusher, servo_pusher_angle_e target);
uint16_t switch_hotline_task(servo_pwm_t *cutter, switch_hotline_t *hotline);

void task_transfer(controller_t *controller);
void task_cut(controller_t *controller);
void task_remove(controller_t *controller);
void task_release(controller_t *controller);
//void task_transmit(controller_t *controller, cmd_mode_e mode, signal_e signal);

void controller_reset(controller_t *controller_reset);

/**
	* @brief          initializer for all components
	* @param		      controller_init: 
  * @retval         
  */
void controller_init(controller_t *controller_init)
{
	/*** liaison ***/
	controller_init->liaison =  get_liaison_ptr(8);
	controller_init->liaison->uart = &huart8;
	
	
	/*** controller ***/
	controller_init->Signal		  = get_Signal_ptr();
	controller_init->Flag 			= get_Flag_ptr();
	controller_init->enteredCut	= 0;
	
	
	/*** uart ***/
	HAL_UART_Receive_IT(&huart8, &(controller_init->liaison->rxData), 1);
	
	
	/*** can, pid, motor ***/
	//can init
	can_filter_init(&hcan1);
	
	//motor speed PID
	/** @todo maybe a special one for 2006 */
	const static fp32 m3508_speed_pid[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
	const static fp32 m2006_speed_pid[3] = {M2006_MOTOR_SPEED_PID_KP, M2006_MOTOR_SPEED_PID_KI, M2006_MOTOR_SPEED_PID_KD};
	
	for(int i=0; i<5; i++)
  {	
		//get the motor pointer in bsp_can.c
		controller_init->motor[i] = get_motor_measure_ptr(i);
		//pid functions init
    pid_init( &(controller_init->motor_pid[i]) );
		//pid parameters init
		if(i == 4)
		{
			controller_init->motor_pid[i].f_param_init(&(controller_init->motor_pid[i]), 
																						 PID_Speed, M2006_MOTOR_SPEED_PID_MAX_OUT, 5000, 10, 8000, 0, m2006_speed_pid);
		}
		else 
    {
			controller_init->motor_pid[i].f_param_init(&(controller_init->motor_pid[i]), 
																							 PID_Speed, M3508_MOTOR_SPEED_PID_MAX_OUT, 5000, 10, 8000, 0, m3508_speed_pid);
		}
  }
	
	//motor status
	controller_init->motor_status[0] = 0;
	controller_init->motor_status[1] = 0;
	controller_init->motor_status[2] = 0;
	
	//speed init
	controller_init->set_spd[0] = MOTOR1_SPEED;
	controller_init->set_spd[1] = MOTOR2_SPEED;
	controller_init->set_spd[2] = MOTOR3_SPEED;
	controller_init->set_spd[3] = MOTOR4_SPEED;
	controller_init->set_spd[4] = MOTOR5_SPEED;
	
	//pid target init
	for(int j=0; j<5; j++)
	{	
		controller_init->motor_pid[j].target = 0;
	}
	
	
	/*** servos ***/
	servo_pwm_init(&controller_init->servo_cutter, 
										1, 
										&htim5, 
										TIM_CHANNEL_4, 
										SERVO_CUTTER_DEFAULT, 
										SERVO_CUTTER_MIN, 
										SERVO_CUTTER_MAX, 
										1 );
	servo_pwm_init(&controller_init->servo_pusher, 
										2, 
										&htim5, 
										TIM_CHANNEL_3, 
										SERVO_PUSHER_DEFAULT, 
										SERVO_PUSHER_MIN, 
										SERVO_PUSHER_MAX, 
										1 );
	
	
	/*** hot line ***/
	controller_init->switch_hotline.port			= HOTLINE_PORT;
	controller_init->switch_hotline.pin				= HOTLINE_PIN;
	controller_init->switch_hotline.status	 	= GPIO_PIN_RESET;
	controller_init->switch_hotline.on_times	= 0;
	controller_init->switch_hotline.mom_set		= 0;
	controller_init->switch_hotline.moment 		= 0.0f;
	controller_init->switch_hotline.duration 	= 0.0f;
	
}


/**
	* @brief          condition controller, involves all the components
	* @param		      a: xxx
  * @retval         
  */
void controller_task(controller_t *controller_task)
{
	motor_task(controller_task, controller_task->motor_status[0], 
															controller_task->motor_status[1], 
															controller_task->motor_status[2]);
	if(*controller_task->Flag == Flag_INIT)
	{
		controller_reset(controller_task);
	}
	if(*controller_task->Flag == Flag_TRANSFER)
	{
		task_transfer(controller_task);
	}
	else if(*controller_task->Flag == Flag_CUT)
	{
		task_cut(controller_task);
	}
	else if(*controller_task->Flag == Flag_REMOVE)
	{
		task_remove(controller_task);
	}
	else if(*controller_task->Flag == Flag_RELEASE)
	{
		task_release(controller_task);
		//task_transmit(controller_task, MODE_Work, Signal_END);
		uart_transmit(controller_task->liaison, MODE_Work, Signal_END);
	}
}


/**
	* @brief          reset the positions of motors and servos
	* @param		      a: xxx
  * @retval         
  */
void controller_reset(controller_t *controller_reset)
{
	controller_reset->motor_status[0] = 0;
	controller_reset->motor_status[1] = 0;
	controller_reset->motor_status[2] = 0;
	
	servo_cutter_task(&controller_reset->servo_cutter, Cutter_Relax);
	servo_pusher_task(&controller_reset->servo_pusher, Pusher_Relax);
	
	if(*controller_reset->Flag != Flag_INIT)
		*controller_reset->Flag = Flag_INIT;
}


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void controller_stop(controller_t *controller_stop)
{
	motor_task(controller_stop, 0, 0, 0);
	switch_hotline_set_status(&controller_stop->switch_hotline, GPIO_PIN_RESET);
	controller_reset(controller_stop);
}


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void task_transfer(controller_t *controller)
{
	// enable wheelset A & B
	//motor_task(controller, 1, 1, 0);
	controller->motor_status[0] = 1;
	controller->motor_status[1] = 1;
	controller->motor_status[2] = 0;
	
	// let the cutter servo in barrier position
	servo_cutter_task(&(controller->servo_cutter), Cutter_Barrier);
	// let the pusher servo in relax position
	servo_pusher_task(&(controller->servo_pusher), Pusher_Relax);
}


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void task_cut(controller_t *controller)
{
	//motor_task(controller, 0, 0, 0);
	controller->motor_status[0] = 0;
	controller->motor_status[1] = 0;
	controller->motor_status[2] = 0;
	
	if(switch_hotline_task(&(controller->servo_cutter), &(controller->switch_hotline)))
	{
		servo_cutter_task(&(controller->servo_cutter), Cutter_Barrier);
		
		servo_pusher_task(&(controller->servo_pusher), Pusher_Push);
		
		//motor_task(controller, 0, 0, 1);
		controller->motor_status[0] = 0;
		controller->motor_status[1] = 0;
		controller->motor_status[2] = 1;
		
		*controller->Flag = Flag_REMOVE;
		
		controller->switch_hotline.mom_set		= 0;
//		controller->switch_hotline.moment 		= 0.0f;
	}
	
}


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void task_remove(controller_t *controller)
{
	servo_cutter_task(&(controller->servo_cutter), Cutter_Barrier);
	
	servo_pusher_task(&(controller->servo_pusher), Pusher_Push);
	
	//motor_task(controller, 0, 0, 1);
	controller->motor_status[0] = 0;
	controller->motor_status[1] = 0;
	controller->motor_status[2] = 1;
	
}



/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void task_release(controller_t *controller)
{
	//motor_task(controller, 0, 1, 0);
	controller->motor_status[0] = 0;
	controller->motor_status[1] = 1;
	controller->motor_status[2] = 0;
	
	servo_pusher_task(&(controller->servo_pusher), Pusher_Relax);
	servo_cutter_task(&(controller->servo_cutter), Cutter_Relax);
}


///**
//	* @brief          
//	* @param		      a: xxx
//  * @retval         
//  */
//void task_transmit(controller_t *controller, cmd_mode_e mode, signal_e signal)
//{
//	uart_transmit(controller->liaison, mode, signal);
//}



/**
	* @brief          enable and drive certain motor(s) according to need
	* @param		      controller: 
	* @param					front : 1 - enable the front  wheelset , 0 - unable
	* @param					middle: 1 - enable the middle wheelset , 0 - unable
	* @param					end		: 1 - enable the end 		wheelset , 0 - unable
  * @retval         
  */
void motor_task(controller_t *controller, uint8_t front, uint8_t middle, uint8_t end)
{
	for(int j=0; j<5; j++) controller->motor_pid[j].target = 0;
	
	//enable certain motor(s) according to need
	if(front)
	{
		controller->motor_pid[0].target = controller->set_spd[0];
		controller->motor_pid[1].target = controller->set_spd[1];
	}
	if(middle)
	{
		controller->motor_pid[4].target = controller->set_spd[4];
	}
	if(end)
	{
		controller->motor_pid[2].target = controller->set_spd[2];
		controller->motor_pid[3].target = controller->set_spd[3];
	}
	
	//pid calculation according to target
	for(int j=0; j<5; j++)
		controller->motor_pid[j].f_cal_pid( &(controller->motor_pid[j]), 
																				controller->motor[j]->speed_rpm);
	
	//send calculation result to motors
	set_motor_current(&hcan1, CAN_Motor_ALL_ID1, controller->motor_pid[0].output,   					//CAN_Motor_ALL_ID1, motor 1-4
											controller->motor_pid[1].output,
											controller->motor_pid[2].output,
											controller->motor_pid[3].output);
	set_motor_current(&hcan1, CAN_Motor_ALL_ID2, controller->motor_pid[4].output, 0, 0, 0);   //CAN_Motor_ALL_ID2, motor 5
	
	//PID control rate = 100Hz
	HAL_Delay(10);
}
	

/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void servo_cutter_task(servo_pwm_t *servo_cutter, servo_cutter_angle_e target)
{
	servo_pwm_set_angle_direct(servo_cutter, target);
	//HAL_Delay(10);
}


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void servo_pusher_task(servo_pwm_t *servo_pusher, servo_pusher_angle_e target)
{
	servo_pwm_set_angle_direct(servo_pusher, target);
	//HAL_Delay(10);
}


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
uint16_t switch_hotline_task(servo_pwm_t *cutter, switch_hotline_t *hotline)
{
	if(hotline->status != GPIO_PIN_SET) //if the hotline isn't on
	{
		switch_hotline_set_status(hotline, GPIO_PIN_SET);
		hotline->moment = Get_SystemTimer();
		hotline->mom_set = 1;
		return 0;
	}
	if(!hotline->mom_set) //if the moment hasn't set yet
	{
		hotline->moment = Get_SystemTimer() - HOTLINE_HEAT_DELAY;
		hotline->mom_set = 1;
	}
	hotline->duration = Get_SystemTimer() - hotline->moment;
	if(hotline->duration < HOTLINE_HEAT_DELAY)
		return 0;
	servo_cutter_task(cutter, Cutter_Cut);
	if(hotline->duration < (HOTLINE_WORK_DELAY + HOTLINE_HEAT_DELAY))
		return 0;
	return 1;
}


