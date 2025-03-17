/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -15.0f
#define V_MAX 15.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -120.0f
#define T_MAX 120.0f

#define KP_SOFTSTOP 20.0f
#define KD_SOFTSTOP 0.4f;





typedef struct
{
	uint8_t id;
	uint8_t mode;
	uint16_t exdata;
	uint8_t res;

	float position;
	float speed;
	float kp;
	float kd;
	float torque;

	//位置限制
	float max_position;
	float min_position;

}Motor_CAN_Send_Struct;

typedef struct
{
	uint8_t master_id;
	uint8_t motor_id;
	uint8_t fault_message;
		/*未标�????
		bit5: HALL 编码故障,10000,16
		bit4: 磁编码故�????,01000,8
		bit3: 过温,00100,4
		bit2: 过流,00010,2
		bit1: 欠压故障,00001,1*/
	uint8_t motor_state;
	/*  0 : Reset 模式[复位]
		1 : Cali 模式[标定]
		2 : Motor 模式[运行]*/
	uint8_t mode;

	uint16_t current_position;//[0~65535]对应(-4π~4π)
	uint16_t current_speed;//[0~65535]对应(-15rad/s~15rad/s)
	uint16_t current_torque;//[0~65535]对应�????-120Nm~120Nm�????
	uint16_t current_temp;//当前温度：Temp(摄氏度）*10

	float current_position_f;//[0~65535]对应(-4π~4π)
	float current_speed_f;//[0~65535]对应(-15rad/s~15rad/s)
	float current_torque_f;//[0~65535]对应�????-120Nm~120Nm�????
	float current_temp_f;//当前温度：Temp(摄氏度）*10


}Motor_CAN_Recieve_Struct;




typedef struct
{
	Motor_CAN_Send_Struct ID_1_Motor_send,ID_2_Motor_send,ID_3_Motor_send,ID_4_Motor_send;
	Motor_CAN_Recieve_Struct ID_1_Motor_recieve,ID_2_Motor_recieve,ID_3_Motor_recieve,ID_4_Motor_recieve;


}Can_Bus_Data_Struct;


/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */


extern Motor_CAN_Recieve_Struct Motor_Recieve_Single_CAN1;
extern Motor_CAN_Recieve_Struct Motor_Recieve_Single_CAN2;
extern Can_Bus_Data_Struct CAN_1,CAN_2;

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);


void Motor_Enable(CAN_HandleTypeDef *hcan,Motor_CAN_Send_Struct *Motor_Data,uint32_t send_mail_box);
void Motor_Disable(CAN_HandleTypeDef *hcan,Motor_CAN_Send_Struct *Motor_Data,uint32_t send_mail_box);
void Motor_Zore(CAN_HandleTypeDef *hcan,Motor_CAN_Send_Struct *Motor_Data,uint32_t send_mail_box);
void CAN_Send_Control(CAN_HandleTypeDef *hcan,Motor_CAN_Send_Struct *Motor_Data,uint32_t send_mail_box);//运控�??,CAN1=CAN_TX_MAILBOX0,CAN2=CAN_TX_MAILBOX1
void Motor_Passive_SET(CAN_HandleTypeDef *hcan,Motor_CAN_Send_Struct *Motor_Data,uint32_t send_mail_box);

void ENABLE_ALL_MOTOR(void);
void DISABLE_ALL_MOTOR(void);
void ZERO_ALL_MOTOR(void);
void PASSIVE_ALL_MOTOR(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

