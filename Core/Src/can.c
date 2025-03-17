/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "usart.h"

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  //CAN filter init
    CAN_FilterTypeDef sFilterConfig1;

    sFilterConfig1.FilterActivation = ENABLE;//打开过滤�?????????????????????????????????????????????
    sFilterConfig1.FilterBank = 0;//过滤�?????????????????????????????????????????????0 这里可设0-13
    sFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;//采用掩码模式
    sFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;//采用32位掩码模�?????????????????????????????????????????????
    sFilterConfig1.FilterFIFOAssignment = CAN_FILTER_FIFO0;//采用FIFO0
    sFilterConfig1.FilterIdHigh = 0x0000; //设置过滤器ID�?????????????????????????????????????????????16�?????????????????????????????????????????????
    sFilterConfig1.FilterIdLow = 0x0000;//设置过滤器ID�?????????????????????????????????????????????16�?????????????????????????????????????????????
    sFilterConfig1.FilterMaskIdHigh = 0x0000;//设置过滤器掩码高16�?????????????????????????????????????????????
    sFilterConfig1.FilterMaskIdLow = 0x0000;//设置过滤器掩码低16�?????????????????????????????????????????????
    ///初始化过滤器
    if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig1) != HAL_OK)
    {
  	  Error_Handler();
    }
    ///�?????????????????????????????????????????????启CAN模块，可以写在CAN初始化函数里�?????????????????????????????????????????????
    if(HAL_CAN_Start(&hcan1) != HAL_OK)//打开can
    {
  	  Error_Handler();
    }
    ///�?????????????????????????????????????启CAN中断接收
    if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
  	  Error_Handler();
  	  printf("-----CAN_RX_IT ENABLE FAILED-----");
    }





  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  //CAN过滤器初始化
    CAN_FilterTypeDef sFilterConfig2;

    sFilterConfig2.FilterActivation = ENABLE;//打开过滤�?????????????????????????????????????????????

    sFilterConfig2.FilterBank = 14;
    sFilterConfig2.SlaveStartFilterBank = 14;

    sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;//采用掩码模式
    sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;//采用32位掩码模�?????????????????????????????????????????????
    sFilterConfig2.FilterFIFOAssignment = CAN_FILTER_FIFO1;//采用FIFO0
    sFilterConfig2.FilterIdHigh = 0x0000; //设置过滤器ID�?????????????????????????????????????????????16�?????????????????????????????????????????????
    sFilterConfig2.FilterIdLow = 0x0000;//设置过滤器ID�?????????????????????????????????????????????16�?????????????????????????????????????????????
    sFilterConfig2.FilterMaskIdHigh = 0x0000;//设置过滤器掩码高16�?????????????????????????????????????????????
    sFilterConfig2.FilterMaskIdLow = 0x0000;//设置过滤器掩码低16�?????????????????????????????????????????????
    ///初始化过滤器
    if(HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig2) != HAL_OK)
    {
  	  Error_Handler();
    }
    ///�?????????????????????????????????????????????启CAN模块，可以写在CAN初始化函数里�?????????????????????????????????????????????
    if(HAL_CAN_Start(&hcan2) != HAL_OK)//打开can
    {
  	  Error_Handler();
    }
    ///�?????????????????????????????????????启CAN中断接收
    if(HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
  	  Error_Handler();
  	  printf("-----CAN_RX_IT ENABLE FAILED-----");
    }


  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 9, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/********************************数据定义***********************************/
Can_Bus_Data_Struct CAN_1;
Can_Bus_Data_Struct CAN_2;

/********************************数据处理***********************************/

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if(x > x_max) x=x_max;
	else if(x < x_min) x= x_min;
	return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/********************************Send***********************************/
CAN_TxHeaderTypeDef txMsg_CAN={
		.StdId = 0,
		.ExtId = 0xff,
		.IDE = CAN_ID_EXT,
		.RTR = CAN_RTR_DATA,
		.DLC = 8,
		};
uint8_t Data_CAN[8];

Motor_CAN_Send_Struct Motor_Data_Single;
//使能,
void Motor_Enable(CAN_HandleTypeDef *hcan,Motor_CAN_Send_Struct *Motor_Data,uint32_t send_mail_box)
{
	Motor_Data_Single.id=Motor_Data->id;
	Motor_Data_Single.exdata=0xfd;//主机ID
	Motor_Data_Single.mode=3;
	Motor_Data_Single.res=0x04;

    txMsg_CAN.ExtId=((Motor_Data_Single.id&0xff)|((Motor_Data_Single.exdata&0xffff)<<8)|((Motor_Data_Single.mode&0x1f)<<24));


	for(int i=0;i<8;i++)
	{
		Data_CAN[i]=0;
	}

	HAL_CAN_AddTxMessage(hcan,&txMsg_CAN,Data_CAN,&send_mail_box);
}

//失能
void Motor_Disable(CAN_HandleTypeDef *hcan,Motor_CAN_Send_Struct *Motor_Data,uint32_t send_mail_box)
{
	Motor_Data_Single.id=Motor_Data->id;
	Motor_Data_Single.exdata=0xfd;//主机ID
	Motor_Data_Single.mode=4;
	Motor_Data_Single.res=0x04;

	 txMsg_CAN.ExtId=((Motor_Data_Single.id&0xff)|((Motor_Data_Single.exdata&0xffff)<<8)|((Motor_Data_Single.mode&0x1f)<<24));

	for(int i=0;i<8;i++)
	{
		Data_CAN[i]=0;
	}
	Data_CAN[0]=1;
	HAL_CAN_AddTxMessage(hcan,&txMsg_CAN,Data_CAN,&send_mail_box);
}

//设置电机机械零位会把当前电机位置设为机械零位（掉电丢�???
void Motor_Zore(CAN_HandleTypeDef *hcan,Motor_CAN_Send_Struct *Motor_Data,uint32_t send_mail_box)
{
	Motor_Data_Single.id=Motor_Data->id;
	Motor_Data_Single.exdata=0;//主机ID
	Motor_Data_Single.mode=6;
	Motor_Data_Single.res=0x04;
	txMsg_CAN.ExtId=((Motor_Data_Single.id&0xff)|((Motor_Data_Single.exdata&0xffff)<<8)|((Motor_Data_Single.mode&0x1f)<<24));

	for(int i=0;i<8;i++)
	{
		Data_CAN[i]=0;
	}
	Data_CAN[0]=1;
	HAL_CAN_AddTxMessage(hcan,&txMsg_CAN,Data_CAN,&send_mail_box);
}

//运控�???,CAN1=CAN_TX_MAILBOX0,CAN2=CAN_TX_MAILBOX1
void CAN_Send_Control(CAN_HandleTypeDef *hcan,Motor_CAN_Send_Struct *Motor_Data,uint32_t send_mail_box)
{
	//运控模式专用的局部变
	CAN_TxHeaderTypeDef txMsg_Control={
			.StdId = 0,
			.ExtId = 0,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = 8,
			};
	uint8_t Data_CAN_Control[8];

	Motor_Data->mode=1;
	Motor_Data->res=0x04;

	Motor_Data->exdata= float_to_uint(Motor_Data->torque,T_MIN,T_MAX,16);//用于补充Data[8]长度
	//轮电机和关节电机的�?�度、力矩范围不�???�???
	if(Motor_Data->id==4)
	{
	   Motor_Data->exdata= float_to_uint(Motor_Data->torque,-17,17,16);//用于补充Data[8]长度
	}


	txMsg_Control.ExtId=((Motor_Data->id&0xff)|((Motor_Data->exdata&0xffff)<<8)|((Motor_Data->mode&0x1f)<<24));

	Data_CAN_Control[0]=float_to_uint(Motor_Data->position,P_MIN,P_MAX,16)>>8;
	Data_CAN_Control[1]=float_to_uint(Motor_Data->position,P_MIN,P_MAX,16);

	Data_CAN_Control[2]=float_to_uint(Motor_Data->speed,V_MIN,V_MAX,16)>>8;
	Data_CAN_Control[3]=float_to_uint(Motor_Data->speed,V_MIN,V_MAX,16);
	//轮电机和关节电机的�?�度、力矩范围不�???�???
	if(Motor_Data->id==4)
	{
		Data_CAN_Control[2]=float_to_uint(Motor_Data->speed,-44,44,16)>>8;
		Data_CAN_Control[3]=float_to_uint(Motor_Data->speed,-44,44,16);
	}

	Data_CAN_Control[4]=float_to_uint(Motor_Data->kp,KP_MIN,KP_MAX,16)>>8;
	Data_CAN_Control[5]=float_to_uint(Motor_Data->kp,KP_MIN,KP_MAX,16);
	Data_CAN_Control[6]=float_to_uint(Motor_Data->kd,KD_MIN,KD_MAX,16)>>8;
	Data_CAN_Control[7]=float_to_uint(Motor_Data->kd,KD_MIN,KD_MAX,16);

	HAL_CAN_AddTxMessage(hcan,&txMsg_Control,Data_CAN_Control,&send_mail_box);
}

void Motor_Passive_SET(CAN_HandleTypeDef *hcan,Motor_CAN_Send_Struct *Motor_Data,uint32_t send_mail_box)
{
	Motor_Data->speed=0;
	Motor_Data->kp=0;
	Motor_Data->kd=3.0;
	Motor_Data->torque=0;

	CAN_Send_Control(hcan,Motor_Data,send_mail_box);

}






void ENABLE_ALL_MOTOR(void)
{


	Motor_Enable(&hcan1,&CAN_1.ID_1_Motor_send,CAN_TX_MAILBOX0);
	osDelay(2);//延时100us
	Motor_Enable(&hcan2,&CAN_2.ID_1_Motor_send,CAN_TX_MAILBOX1);
	osDelay(2);//延时100us

	Motor_Enable(&hcan1,&CAN_1.ID_2_Motor_send,CAN_TX_MAILBOX0);
	osDelay(2);//延时100us
	Motor_Enable(&hcan2,&CAN_2.ID_2_Motor_send,CAN_TX_MAILBOX1);
	osDelay(2);//延时100us

	Motor_Enable(&hcan1,&CAN_1.ID_3_Motor_send,CAN_TX_MAILBOX0);
	osDelay(2);//延时100us
	Motor_Enable(&hcan2,&CAN_2.ID_3_Motor_send,CAN_TX_MAILBOX1);
	osDelay(2);//延时100us

	Motor_Enable(&hcan1,&CAN_1.ID_4_Motor_send,CAN_TX_MAILBOX0);
	osDelay(2);//延时100us
	Motor_Enable(&hcan2,&CAN_2.ID_4_Motor_send,CAN_TX_MAILBOX1);
	osDelay(2);//延时100us

}

void DISABLE_ALL_MOTOR(void)
{
	Motor_Disable(&hcan1,&CAN_1.ID_1_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Disable(&hcan2,&CAN_2.ID_1_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us

	Motor_Disable(&hcan1,&CAN_1.ID_2_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Disable(&hcan2,&CAN_2.ID_2_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us

	Motor_Disable(&hcan1,&CAN_1.ID_3_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Disable(&hcan2,&CAN_2.ID_3_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us

	Motor_Disable(&hcan1,&CAN_1.ID_4_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Disable(&hcan2,&CAN_2.ID_4_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us





}

void ZERO_ALL_MOTOR(void)
{

	Motor_Zore(&hcan1,&CAN_1.ID_1_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Zore(&hcan2,&CAN_2.ID_1_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us

	Motor_Zore(&hcan1,&CAN_1.ID_2_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Zore(&hcan2,&CAN_2.ID_2_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us

	Motor_Zore(&hcan1,&CAN_1.ID_3_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Zore(&hcan2,&CAN_2.ID_3_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us

	Motor_Zore(&hcan1,&CAN_1.ID_4_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Zore(&hcan2,&CAN_2.ID_4_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us



}

void PASSIVE_ALL_MOTOR(void)
{
	Motor_Passive_SET(&hcan1,&CAN_1.ID_1_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Passive_SET(&hcan2,&CAN_2.ID_1_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us

	Motor_Passive_SET(&hcan1,&CAN_1.ID_2_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Passive_SET(&hcan2,&CAN_2.ID_2_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us

	Motor_Passive_SET(&hcan1,&CAN_1.ID_3_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Passive_SET(&hcan2,&CAN_2.ID_3_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us

	Motor_Passive_SET(&hcan1,&CAN_1.ID_4_Motor_send,CAN_TX_MAILBOX0);
	osDelay(1);//延时100us
	Motor_Passive_SET(&hcan2,&CAN_2.ID_4_Motor_send,CAN_TX_MAILBOX1);
	osDelay(1);//延时100us

}



/********************************接收***********************************/

Motor_CAN_Recieve_Struct Motor_Recieve_Single_CAN1;
Motor_CAN_Recieve_Struct Motor_Recieve_Single_CAN2;
/**
 * can1接收中断回调函数,在中断服务函数中被执
 *
**/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];
	BaseType_t pxHigherPriorityTaskWoken=pdFALSE;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);

	if(hcan->Instance == CAN1)
	{

		Motor_Recieve_Single_CAN1.master_id=(rxHeader.ExtId)&0xff;
		Motor_Recieve_Single_CAN1.motor_id=(rxHeader.ExtId>>8)&0xff;
		Motor_Recieve_Single_CAN1.fault_message=(rxHeader.ExtId>>16)&0x3f;
		Motor_Recieve_Single_CAN1.motor_state=(rxHeader.ExtId>>21)&0x03;
		Motor_Recieve_Single_CAN1.mode=(rxHeader.ExtId>>24)&0x1f;//通信模式

		Motor_Recieve_Single_CAN1.current_position=(rxData[0]<<8)|(rxData[1]);
		Motor_Recieve_Single_CAN1.current_speed=(rxData[2]<<8)|(rxData[3]);
		Motor_Recieve_Single_CAN1.current_torque=(rxData[4]<<8)|(rxData[5]);
		Motor_Recieve_Single_CAN1.current_temp=(rxData[6]<<8)|(rxData[7]);


		//发�?�信号量进行数据打包到spi
		vTaskNotifyGiveFromISR(CAN1_Decode_TASHandle,&pxHigherPriorityTaskWoken);
	}
	 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}
/**
 * can2接收中断回调函数
 * 在这里处理左腿接收到�????????4个电机的数据,它进行解�????????,分别放到左腿的每�????????个电机的结构体里�????????
**/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];
	BaseType_t pxHigherPriorityTaskWoken=pdFALSE;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData);

	if(hcan->Instance == CAN2)
	{
		Motor_Recieve_Single_CAN2.master_id=(rxHeader.ExtId)&0xff;
		Motor_Recieve_Single_CAN2.motor_id=(rxHeader.ExtId>>8)&0xff;
		Motor_Recieve_Single_CAN2.fault_message=(rxHeader.ExtId>>16)&0x3f;
		Motor_Recieve_Single_CAN2.motor_state=(rxHeader.ExtId>>21)&0x03;
		Motor_Recieve_Single_CAN2.mode=(rxHeader.ExtId>>24)&0x1f;

		Motor_Recieve_Single_CAN2.current_position=(rxData[0]<<8)|(rxData[1]);
		Motor_Recieve_Single_CAN2.current_speed=(rxData[2]<<8)|(rxData[3]);
		Motor_Recieve_Single_CAN2.current_torque=(rxData[4]<<8)|(rxData[5]);
		Motor_Recieve_Single_CAN2.current_temp=(rxData[6]<<8)|(rxData[7]);


		//发�?�信号量进行数据打包到spi
		vTaskNotifyGiveFromISR(CAN2_Decode_TASHandle,&pxHigherPriorityTaskWoken);//通知唤醒CAN2数据处理任务

	}
	 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);//任务切换
}




/* USER CODE END 1 */
