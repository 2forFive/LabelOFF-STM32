/**
  ******************************************************************************
  * @file       pid.h
	* @author			sxx
  * @brief      PID control
  * @note       Modified from DJI M2006 Demo and lml's trials.
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-11-2021     1. done
	*
  ******************************************************************************
  */
	
#ifndef PID_H
#define PID_H

#include "struct_typedef.h"
#include "bsp_can.h"

/* pid id enum */
typedef enum
{
	PID_Position,
	PID_Speed
} pid_id_e;

/* pid struct typedef*/
typedef struct _pid_t
{
	pid_id_e id;
	
	float target;							// target value
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;
	
	float   measure;					// measured value
	float   err;							// error
	float   last_err;      		// error of last time
	
	float pout;
	float iout;
	float dout;
	
	float output;
	float last_output;			// output of last time
	
	float MaxOutput;				// maximum value of output
	float IntegralLimit;		// maximum value of iout
	float DeadBand;			  	// the absolute value of deadband
	float ControlPeriod;		// control period
	float  Max_Err;					// maximum value of error
	
	uint32_t thistime;
	uint32_t lasttime;
	uint8_t dtime;	
	
	//PID parameter initialization
	void (*f_param_init)(struct _pid_t *pid,  
										pid_id_e id,
										uint16_t maxOutput,
										uint16_t integralLimit,
										float deadband,
										int16_t max_err,     
										int16_t  target,
										const float PID[3]);
	//pid parameter modifying
	void (*f_pid_reset)(struct _pid_t *pid, float kp,float ki, float kd);
	//pid calculation
	float (*f_cal_pid)(struct _pid_t *pid, float measure);

} pid_t;


/* function prototypes */
void pid_init(pid_t* pid);

#endif
