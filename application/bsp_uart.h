/**
  ******************************************************************************
  * @file       bsp_uart.h
	* @author			sxx
  * @brief      contact with pi, may be also control the servos
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-11-2021     1. 
  * @todo				1. 
	*
  ******************************************************************************
  */
	
#ifndef BSP_UART_H
#define BSP_UART_H
#include "struct_typedef.h"
#include "usart.h"

typedef enum
{
	Signal_INIT	 = 0,
	
	Signal_START = 1000,
	Signal_END   = 2000,
	
	Signal_ERROR = 9999,
	
	Signal_TEST  = 8888
} signal_e;

typedef enum
{
	Flag_INIT 		= 0,
	Flag_TRANSFER = 1001,
	Flag_CUT			= 1002,
	Flag_REMOVE		= 1003,
	Flag_RELEASE	= 1004
} flag_e;


typedef enum
{
	MODE_INIT,
	MODE_Work,
	MODE_Error,
	MODE_Test
} cmd_mode_e;

typedef enum
{
	WORK_Begin = 1000,
	WORK_End 	 = 1999,
	ERROR_All  = 9999,
	TEST_OK		 = 1
} command_e;


typedef struct
{
	UART_HandleTypeDef *uart;
	
	char txData[10];
	
	uint8_t rxData;
	
	uint8_t i;
	char temp[10];
	char compare[10];
	
} liaison_t;



//extern uint8_t rxData;
void uart_transmit(liaison_t *liaison, cmd_mode_e mode, signal_e signal);
liaison_t* get_liaison_ptr(uint8_t ch);
//uint16_t* get_Signal_ptr(void);
//uint16_t* get_Flag_ptr(void);
signal_e* get_Signal_ptr(void);
flag_e* 	get_Flag_ptr(void);


#endif
