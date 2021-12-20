/**
  ******************************************************************************
  * @file       bsp_can.c
	* @author			sxx
  * @brief      
  * @note       Modified from DJI M2006 Demo and lml's trials.
	*							Original code uses old version HAL functions, 
	*							now they are the latest version.
  * @history
  *  Version    Date            Modification
  *  V1.0.0     Dec-10-2021     1. done
  * @todo				1. rename
	*							2. function prototypes
	*
  ******************************************************************************
  */

#include "can.h"
#include "bsp_can.h"

motor_measure_t motor[5] = {0};

CANTxMsg_t txMsg;
CANRxMsg_t rxMsg;


void get_total_angle(motor_measure_t *p);
void get_motor_offset(motor_measure_t *ptr, CAN_HandleTypeDef *hcan);


/**
  * @brief          return the pwm servo angle data
  * @param		      servo_pwm: controlled pwm servo point
  * @retval         pwm servo angle data
  */
void can_filter_init(CAN_HandleTypeDef *hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterTypeDef		CAN_FilterConfigStructure;  //oldVer: CAN_FilterConfTypeDef

	CAN_FilterConfigStructure.FilterBank = 0;   		//oldVer: FilterNumber
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
  //oldVer: BankNumber; can1(0-13) & can2(14-27) respectively half filters
	CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
	CAN_FilterConfigStructure.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(hcan, &CAN_FilterConfigStructure);
	
	HAL_CAN_Start(hcan);
	
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	
}


uint32_t FlashTimer;
/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_GetTick() - FlashTimer > 500){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED1_Pin+1);
		FlashTimer = HAL_GetTick();
	}
	
	if(hcan->Instance == CAN1)
	{
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &(rxMsg.RxHeader), (rxMsg.rxbuffer)) != HAL_OK)
			Error_Handler();
	}

	//ignore can1 or can2.
	switch(rxMsg.RxHeader.StdId){
		case CAN_Motor1_ID:
		case CAN_Motor2_ID:
		case CAN_Motor3_ID:
		case CAN_Motor4_ID:
		{
			static u8 i;
			i = rxMsg.RxHeader.StdId - CAN_Motor1_ID;
			get_motor_measure(&motor[i]);
			break;
		}
		
		case CAN_Motor5_ID:
		{
			get_motor_measure(&motor[4]);
			break;
		}
	
		default:
		{
			break;
		}
	}
	
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	//__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  //oldVer: CAN_IT_FMP0

}


/**
	* @brief          
	* @param		      a: xxx
  * @retval         
  */
void get_motor_measure(motor_measure_t *ptr)
{
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(rxMsg.rxbuffer[0]<<8 | rxMsg.rxbuffer[1]) ;
	ptr->speed_rpm  = (int16_t)(rxMsg.rxbuffer[2]<<8 | rxMsg.rxbuffer[3]);
	ptr->real_current = (rxMsg.rxbuffer[4]<<8 | rxMsg.rxbuffer[5])*5.f/16384.f;

	ptr->hall = rxMsg.rxbuffer[6];
	
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}


/* this function should be called after system+can init */
void get_motor_offset(motor_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->angle = (uint16_t)(rxMsg.rxbuffer[0]<<8 | rxMsg.rxbuffer[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@brief =0, ?????????3510????????(?0)??????
	*/
void get_total_angle(motor_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//possible situations
		res1 = p->angle + 8192 - p->last_angle;	//forward , delta=+
		res2 = p->angle - p->last_angle;				//backward, delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//backward, delta -
		res2 = p->angle - p->last_angle;				//forward , delta +
	}
	//no matter forward or backward, the smaller one is true
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}


/**
	* @brief          
	* @param		      a: xxx
  * @return         
  */
void set_motor_current(CAN_HandleTypeDef *hcan, u32 stdId, s16 iq1, s16 iq2, s16 iq3, s16 iq4){
	txMsg.TxHeader.StdId = stdId;
	txMsg.TxHeader.IDE = CAN_ID_STD;
	txMsg.TxHeader.RTR = CAN_RTR_DATA;
	txMsg.TxHeader.DLC = 0x08;
	txMsg.txbuffer[0] = iq1 >> 8;
	txMsg.txbuffer[1] = iq1;
	txMsg.txbuffer[2] = iq2 >> 8;
	txMsg.txbuffer[3] = iq2;
	txMsg.txbuffer[4] = iq3 >> 8;
	txMsg.txbuffer[5] = iq3;
	txMsg.txbuffer[6] = iq4 >> 8;
	txMsg.txbuffer[7] = iq4;
	
	HAL_CAN_AddTxMessage(hcan, &txMsg.TxHeader, txMsg.txbuffer, &txMsg.mailbox);
}	


/**
  * @brief          return the motor data pointer
  * @param[in]      i: motor number, range [0,4]
  * @retval         motor data point
  */
const motor_measure_t* get_motor_measure_ptr(uint8_t i)
{
    //return &motor[(i & 0x03)];
	return &motor[i];
}

