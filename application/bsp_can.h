/**
  ******************************************************************************
  * @file       bsp_can.h
	* @author			sxx
  * @brief      CAN ID enum, CAN Tx and Rx structs, motor data struct
  * @note       
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-13-2021     1. done
	*
  ******************************************************************************
  */
	
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "can.h"

/* CAN ID enum */
typedef enum
{
	CAN_Motor_ALL_ID1 = 0x200,
	CAN_Motor1_ID 		= 0x201,
	CAN_Motor2_ID 		= 0x202,
	CAN_Motor3_ID 		= 0x203,
	CAN_Motor4_ID 		= 0x204,
	
	CAN_Motor5_ID		  = 0x205,
	CAN_Motor6_ID 		= 0x206,
	CAN_Motor7_ID 		= 0x207,
	CAN_Motor8_ID 		= 0x208,
	CAN_Motor_ALL_ID2 = 0x1FF
} can_msg_id_e;


/* CAN Tx */
typedef struct
{
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t txbuffer[8];
	uint32_t mailbox;
} CANTxMsg_t;

/* CAN Rx */
typedef struct
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rxbuffer[8];
} CANRxMsg_t;


#define FILTER_BUF_LEN		5
/*motor data typedef*/
typedef struct{
	int16_t	 	speed_rpm;
  float  		real_current;
  int16_t  	given_current;
  uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle; 	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	uint8_t		buf_idx;
	uint16_t	angle_buf[FILTER_BUF_LEN];
	uint16_t	fited_angle;
	uint32_t	msg_cnt;
} motor_measure_t;


/* function prototypes*/
void can_filter_init(CAN_HandleTypeDef *hcan);
void get_motor_measure(motor_measure_t *ptr);
void set_motor_current(CAN_HandleTypeDef *hcan, u32 stdId, s16 iq1, s16 iq2, s16 iq3, s16 iq4);

extern const motor_measure_t* get_motor_measure_ptr(uint8_t i);

#endif
