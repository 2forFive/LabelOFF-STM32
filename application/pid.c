/**
  ******************************************************************************
  * @file       pid.c
	* @author			sxx
  * @brief      PID control
  * @note       Modified from DJI M2006 Demo and lml's trials.
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-11-2021     1. done
	*
  ******************************************************************************
  */
	
#include "pid.h"
#include "stm32f4xx.h"

#define ABS(x)		((x>0)? x: -x) 

static void pid_param_init(pid_t 	 *pid, 
													pid_id_e id,
													uint16_t maxout,
													uint16_t intergral_limit,
													float 	 deadband,
													int16_t  max_err,
													int16_t  target,
													const float PID[3])
{
	pid->id = id;		
	
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;
	
	pid->kp = PID[0];
	pid->ki = PID[1];
	pid->kd = PID[2];
	
	pid->output = 0;
}



/**
	* @brief          pid parameters assignment
	* @param		      pid: pointer of pid
	*									kp: value of parameter p
	*									ki: value of parameter i
	*									kd: value of parameter d
  * @return         none
  */
static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}


/**
	* @brief          pid calculation
	* @param		      pid: pointer of pid
	*									measure: measured value
  * @return         output of the calculation
  */
static float pid_calculate(pid_t* pid, float measure)
{
	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime-pid->lasttime;
	pid->measure = measure;
		
	pid->last_err  = pid->err;
	pid->last_output = pid->output;
	
	pid->err = pid->target - pid->measure;
	
	//whether in the deadband
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		
		pid->iout += (pid->ki * pid->err);
		
		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
		//iout limit
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		
		//pid output, the some of p,i,d outs
		pid->output = pid->pout + pid->iout + pid->dout;

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //filtering?
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	
	}

	return pid->output;
}


/**
	* @brief          initiate the pid struct
	* @param		      pid: pointer of pid
  * @return         none
  */
void pid_init(pid_t* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}



