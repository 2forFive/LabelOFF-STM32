/**
  ******************************************************************************
  * @file       controller.h
	* @author			sxx
  * @brief      consts and enums for the actuators;
	*							struct of the controller
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-11-2021     1. template, online
	*  V1.1.0			Dec-13-2021			1. modified pid constants
	*	 V1.2.0			Dec-14-2021			1. add hot line
	*	 V2.0.0			Dec-23-2021			1. done
	*
  ******************************************************************************
  */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "struct_typedef.h"
#include "bsp_can.h"
#include "pid.h"
#include "servo_pwm.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_tim.h"

/*** define consts ***/
/* motor speed */
#define MOTOR1_SPEED -200  // front left , counterclockwise, negative
#define MOTOR2_SPEED  200  // front right, 			 clockwise, positive
#define MOTOR3_SPEED  600  // end   upper, 			 clockwise, positive
#define MOTOR4_SPEED -600  // end   lower, counterclockwise, negative
#define MOTOR5_SPEED  800  // middle			, 			 clockwise, positive

/* motor current */
#define M3508_MAX_CURRENT 16384
#define M2006_MAX_CURRENT 10000

/* motor speed PID */
/* M3508 */
#define M3508_MOTOR_SPEED_PID_KP 				1.5f
#define M3508_MOTOR_SPEED_PID_KI 				0.1f
#define M3508_MOTOR_SPEED_PID_KD 				0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT 	M3508_MAX_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 	2000.0f
/* M2006 */
#define M2006_MOTOR_SPEED_PID_KP 				2.0f
#define M2006_MOTOR_SPEED_PID_KI 				0.1f
#define M2006_MOTOR_SPEED_PID_KD 				0.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT 	M2006_MAX_CURRENT
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 	2000.0f

/* cutter servo */
#define SERVO_CUTTER_MIN			500
#define SERVO_CUTTER_MAX 			1100
#define SERVO_CUTTER_DEFAULT  SERVO_CUTTER_MIN

/* pusher servo */
#define SERVO_PUSHER_MIN			500
#define SERVO_PUSHER_MAX 			1200
#define SERVO_PUSHER_DEFAULT	SERVO_PUSHER_MIN
//#define SERVO_PUSHER_PRESSURE 1100

/* hotline */
#define HOTLINE_PORT			 GPIOI
#define HOTLINE_PIN				 GPIO_PIN_2
#define HOTLINE_HEAT_DELAY 23000
#define HOTLINE_WORK_DELAY 1500
// use LED5 to substitute hotline when debugging for the sake of safety
//#define HOTLINE_PORT			 GPIOG
//#define HOTLINE_PIN				 GPIO_PIN_5

/* enum of servo angles */
/* cutter */
typedef enum
{
	Cutter_Relax   = SERVO_CUTTER_DEFAULT,
	Cutter_Tangent = 625,
	Cutter_Barrier = 700,
	Cutter_Cut		 = 800
} servo_cutter_angle_e;
/* pusher */
typedef enum
{
	Pusher_Relax = SERVO_PUSHER_DEFAULT,
	Pusher_Push	= 1050
} servo_pusher_angle_e;


/* controller struct typedef */
typedef struct
{
	/* time period counter */
	counter_t counter;

	/* liaison, communicate with Pi */
	liaison_t *liaison;
	
	/* uart signal */
	//uint16_t *Signal;  // for debugging without upper computer
	signal_e *Signal;
	
	/* work condition flag, only make sense when Signal=Start */
	//uint16_t *Flag;    // for debugging without upper computer
	flag_e *Flag;
	
	/* whether has entered Cut step */
	// seems no use anymore but was kept in case of misunderstanding
	uint8_t enteredCut;
	
	/* controller of three motor modules */
	uint8_t motor_status[3];
	
	/* motor 1-5 */
	int32_t set_spd[5];								// target spd
	const motor_measure_t *motor[5];	// motor data
	pid_t motor_pid[5];								// pid
	
	/* servo cutter & pusher */
	servo_pwm_t servo_cutter;
	servo_pwm_t servo_pusher;
	
	/* relay of hotline*/
	switch_hotline_t switch_hotline;
	
} controller_t;


/* manually debugging controller */
typedef struct
{
	uint16_t turn_on;
	uint16_t signal;
	uint16_t flag;
} uart_stand_in_t;


/** function prototypes */
void controller_init(controller_t *controller_init);
void controller_task(controller_t *controller_task);
void controller_reset(controller_t *controller_reset);
void controller_stop(controller_t *controller_stop);
// in case of manually debugging motors, can be commented
void motor_task(controller_t *controller, uint8_t front, uint8_t middle, uint8_t end);

#endif
