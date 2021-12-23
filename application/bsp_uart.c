/**
  ******************************************************************************
  * @file       bsp_uart.c
	* @author			sxx
  * @brief      contact with pi, may be also control the servos
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-11-2021     1. HAL_UART_RxCpltCallback
  * @todo				1. 
	*
  ******************************************************************************
  */
	
#include <stdio.h>
#include <string.h>
#include "struct_typedef.h"
#include "bsp_uart.h"

liaison_t liaison_ch8;
//uint16_t Signal = 0;
//uint16_t Flag 	= 0;
signal_e Signal = Signal_INIT;
flag_e Flag 		= Flag_INIT;

//uint8_t i;
//uint8_t rxData;
//char temp[10];

//uint8_t temp;
//char temp[] = {0};


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void uart_transmit(liaison_t *liaison, cmd_mode_e mode, signal_e signal)
{
	switch(mode)
	{
		case MODE_INIT:
		{
			sprintf(liaison->txData, "#I%04d", signal);
			break;
		}
		case MODE_Work:
		{
			sprintf(liaison->txData, "#W%04d", signal);
			break;
		}
		case MODE_Error:
		{	
			sprintf(liaison->txData, "#E%04d", signal);
			break;
		}
		case MODE_Test:
		{
			sprintf(liaison->txData, "#T%04d", signal);
			break;
		}
		default:
		{
			break;
		}
	}
	
	HAL_UART_Transmit_IT(liaison->uart, (uint8_t *)liaison->txData, 25);
	HAL_Delay(1);
}


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART8)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED1_Pin);
		HAL_UART_Transmit(&huart8, &liaison_ch8.rxData, 1, 100);
		
		if(liaison_ch8.rxData=='#' || liaison_ch8.i>=sizeof(liaison_ch8.temp) )
		{
			memset(liaison_ch8.temp, 0, sizeof(liaison_ch8.temp)); 
			liaison_ch8.i = 0;
		}
		liaison_ch8.temp[liaison_ch8.i++] = liaison_ch8.rxData;
		
		HAL_UART_Receive_IT(&huart8, &liaison_ch8.rxData, 1);
	}
	
	if(strcmp(liaison_ch8.temp, "#W1000") == 0)
	{
		Signal = Signal_START;
		// LED
	}
	if(strcmp(liaison_ch8.temp, "#W2000") == 0)
	{
		Signal = Signal_END;
		// LED
	}
	else if(strcmp(liaison_ch8.temp, "#E9999") == 0)
	{
		Signal = Signal_ERROR;
		Flag	 = Flag_INIT;
		// LED
	}
	
	if(Signal==Signal_START)
	{
		if(strcmp(liaison_ch8.temp, "#F0000") == 0)
		{
			Flag = Flag_INIT;
		}
		else if(strcmp(liaison_ch8.temp, "#F1001") == 0)
		{
			Flag = Flag_TRANSFER;
		}
		else if(strcmp(liaison_ch8.temp, "#F1002") == 0)
		{
			Flag = Flag_CUT;
		}
		else if(strcmp(liaison_ch8.temp, "#F1003") == 0)
		{
			Flag = Flag_REMOVE;
		}
		else if(strcmp(liaison_ch8.temp, "#F1004") == 0)
		{
			Flag = Flag_RELEASE;
		}
	}
}


liaison_t* get_liaison_ptr(uint8_t ch)
{
	if(ch == 8)
		return &liaison_ch8;
	return 0;
}


//uint16_t* get_Signal_ptr()
//{
//	return &Signal;
//}

//uint16_t* get_Flag_ptr()
//{
//	return &Flag;
//}

signal_e* get_Signal_ptr()
{
	return &Signal;
}

flag_e* get_Flag_ptr()
{
	return &Flag;
}

