/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "can.h"
#include "tim.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//声明
#define PI (3.1415926f)

#define BEEP_ON    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,400);
#define BEEP_OFF   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);

void Can_Bus_Motor_Init(void);



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Motor_Contron_THandle;
uint32_t Motor_Contron_TaskBuffer[ 128 ];
osStaticThreadDef_t Motor_Contron_TaskControlBlock;
osThreadId CAN1_Decode_TASHandle;
uint32_t CAN1_Decode_TASBuffer[ 128 ];
osStaticThreadDef_t CAN1_Decode_TASControlBlock;
osThreadId CAN2_Decode_TASHandle;
uint32_t CAN2_Decode_TASBuffer[ 128 ];
osStaticThreadDef_t CAN2_Decode_TASControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Motor_Contron_Task_Task(void const * argument);
void CAN1_Decode_Task(void const * argument);
void CAN2_Decode_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Motor_Contron_T */
  osThreadStaticDef(Motor_Contron_T, Motor_Contron_Task_Task, osPriorityRealtime, 0, 128, Motor_Contron_TaskBuffer, &Motor_Contron_TaskControlBlock);
  Motor_Contron_THandle = osThreadCreate(osThread(Motor_Contron_T), NULL);

  /* definition and creation of CAN1_Decode_TAS */
  osThreadStaticDef(CAN1_Decode_TAS, CAN1_Decode_Task, osPriorityAboveNormal, 0, 128, CAN1_Decode_TASBuffer, &CAN1_Decode_TASControlBlock);
  CAN1_Decode_TASHandle = osThreadCreate(osThread(CAN1_Decode_TAS), NULL);

  /* definition and creation of CAN2_Decode_TAS */
  osThreadStaticDef(CAN2_Decode_TAS, CAN2_Decode_Task, osPriorityNormal, 0, 128, CAN2_Decode_TASBuffer, &CAN2_Decode_TASControlBlock);
  CAN2_Decode_TASHandle = osThreadCreate(osThread(CAN2_Decode_TAS), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	//配置滴答定时器的中断周期180000000 /10000U个机器周期（180Mhz），100us产生�????????次中�????????,即系统延时单位为0.1ms
	  HAL_SYSTICK_Config(180000000 /10000U);

  /* Infinite loop */
  for(;;)
  {
    LED0_ON
	osDelay(5000);//0.1ms
    LED0_OFF
	LED1_OFF
	LED2_OFF
    osDelay(5000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Motor_Contron_Task_Task */
/**
* @brief Function implementing the Motor_Contron_T thread.
* @param argument: Not used
* @retval None
*/


int run_count=0;
uint32_t err_spi;


//初始位置
float init_p_1=4;
float init_p_2=4;
//回零点的时间，ms
#define TO_ZERO_TIME 10000

float to_zero_speed=TO_ZERO_TIME;


int sin_time=12000;//正弦�?周期的时间，ms
float position_wide=0.5;//正弦角度范围rad


//初始化标�?
char init_flag=0;

//数据记录
float MAX_T=0;



/* USER CODE END Header_Motor_Contron_Task_Task */
void Motor_Contron_Task_Task(void const * argument)
{
  /* USER CODE BEGIN Motor_Contron_Task_Task */

	Can_Bus_Motor_Init();//电机初始�??
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	BEEP_ON
	osDelay(10000);
	BEEP_OFF

	//多发几次命令，读取初始角�?
	 ENABLE_ALL_MOTOR();
	 osDelay(500);
	 ENABLE_ALL_MOTOR();
	 osDelay(500);
	 ENABLE_ALL_MOTOR();
	 osDelay(500);

	//控制数据�?
	CAN_1.ID_1_Motor_send.position=0;
	CAN_1.ID_1_Motor_send.speed=0;
	CAN_1.ID_1_Motor_send.torque=0;
	CAN_1.ID_1_Motor_send.kp=60;
	CAN_1.ID_1_Motor_send.kd=2;

	CAN_2.ID_1_Motor_send.position=0;
	CAN_2.ID_1_Motor_send.speed=0;
	CAN_2.ID_1_Motor_send.torque=0;
	CAN_2.ID_1_Motor_send.kp=60;
	CAN_2.ID_1_Motor_send.kd=2;


	CAN_1.ID_4_Motor_send.position=0;
	CAN_1.ID_4_Motor_send.speed=1;
	CAN_1.ID_4_Motor_send.torque=0;
	CAN_1.ID_4_Motor_send.kp=0;
	CAN_1.ID_4_Motor_send.kd=0.5;

   //获取初始位置
	init_p_1=CAN_1.ID_1_Motor_recieve.current_position_f;
	init_p_2=CAN_2.ID_1_Motor_recieve.current_position_f;

	printf("init_p_1= %f rad",init_p_1);
	printf("init_p_2= %f rad",init_p_2);
  /* Infinite loop */
  for(;;)
  {
	run_count++;

	//回零
	if(init_flag==0)
	{
		if(to_zero_speed>0)
		{
			to_zero_speed--;
	       CAN_1.ID_1_Motor_send.position=init_p_1*(to_zero_speed/TO_ZERO_TIME);
	       CAN_2.ID_1_Motor_send.position=init_p_2*(to_zero_speed/TO_ZERO_TIME);
		}
		if(to_zero_speed<=0)
		{
			init_flag=1;
	        run_count=0;
		}
	}
	//正弦
	else if(init_flag==1)
	{
		CAN_1.ID_1_Motor_send.position=position_wide*sin(2.0*3.1415926*(float)run_count/sin_time);
		CAN_2.ID_1_Motor_send.position=position_wide*sin(2.0*3.1415926*(float)run_count/sin_time);
	}

 
	//RS04 CAN1
	CAN_Send_Control(&hcan1,&CAN_1.ID_1_Motor_send,CAN_TX_MAILBOX0);
	osDelay(2);//200US
	//RS04 CAN2
	CAN_Send_Control(&hcan2,&CAN_2.ID_1_Motor_send,CAN_TX_MAILBOX1);
	osDelay(2);//200US
	//RS02
	CAN_Send_Control(&hcan1,&CAN_1.ID_4_Motor_send,CAN_TX_MAILBOX0);
	osDelay(2);


	//
	if(CAN_1.ID_1_Motor_recieve.current_torque_f>MAX_T)MAX_T=CAN_1.ID_1_Motor_recieve.current_torque_f;

    //�?100ms打印�?�?
	if(run_count%100==0)
	{
	   //printf("Motor_Speed= %f rad/s = %f rpm \r\n\r\n",CAN_1.ID_4_Motor_recieve.current_speed_f,(CAN_1.ID_4_Motor_recieve.current_speed_f/2.0/3.141592*60));

		//printf("Motor_Position= %f rad\r\n",CAN_1.ID_1_Motor_recieve.current_position_f);


		printf("					Motor_Torque= %f NM 			MAX_T_back=  %f	NM\r\n\r\n",CAN_1.ID_1_Motor_recieve.current_torque_f,MAX_T);

		printf("Motor_Temp= %f C       fault_message=%d \r\n\r\n\r\n",CAN_1.ID_1_Motor_recieve.current_temp_f,CAN_1.ID_1_Motor_recieve.fault_message);
	}
	osDelay(4);//600us
  }
  /* USER CODE END Motor_Contron_Task_Task */
}

/* USER CODE BEGIN Header_CAN1_Decode_Task */
/**
* @brief Function implementing the CAN1_Decode_TAS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN1_Decode_Task */
void CAN1_Decode_Task(void const * argument)
{
  /* USER CODE BEGIN CAN1_Decode_Task */
	uint32_t err;
  /* Infinite loop */
  for(;;)
  {
	  err=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(err==1)
    {
    	LED1_ON


    	if(Motor_Recieve_Single_CAN1.master_id==0xfd)
    	{

    	Motor_Recieve_Single_CAN1.current_position_f=uint_to_float(Motor_Recieve_Single_CAN1.current_position,(-4.0*PI),(4.0*PI),16);
    	Motor_Recieve_Single_CAN1.current_speed_f=uint_to_float(Motor_Recieve_Single_CAN1.current_speed,(-15.0),(15.0),16);
    	Motor_Recieve_Single_CAN1.current_torque_f=uint_to_float(Motor_Recieve_Single_CAN1.current_torque,(-120.0),(120.0),16);
    	Motor_Recieve_Single_CAN1.current_temp_f=(float)Motor_Recieve_Single_CAN1.current_temp/10;

    	 switch(Motor_Recieve_Single_CAN1.motor_id)
			{
			  case 1://RS04电机
			  {
				  CAN_1.ID_1_Motor_recieve=Motor_Recieve_Single_CAN1;
				  break;
			  }
			  case 2://RS04电机
			  {
				  CAN_1.ID_2_Motor_recieve=Motor_Recieve_Single_CAN1;
				  break;
			  }
			  case 3://RS04电机
			  {
				  CAN_1.ID_3_Motor_recieve=Motor_Recieve_Single_CAN1;
				  break;
			  }
			  case 4://RS02电机
			  {
				  //RS02电机和RS04电机的角速度、力矩范围不�???�???
				  Motor_Recieve_Single_CAN1.current_speed_f=uint_to_float(Motor_Recieve_Single_CAN1.current_speed,(-44.0),(44.0),16);//
				  Motor_Recieve_Single_CAN1.current_torque_f=uint_to_float(Motor_Recieve_Single_CAN1.current_torque,(-17.0),(17.0),16);

				  CAN_1.ID_4_Motor_recieve=Motor_Recieve_Single_CAN1;
				  break;
			  }
			  default:
				  break;

			}
    	}
    }
  }
  /* USER CODE END CAN1_Decode_Task */
}

/* USER CODE BEGIN Header_CAN2_Decode_Task */
/**
* @brief Function implementing the CAN2_Decode_TAS thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_CAN2_Decode_Task */
void CAN2_Decode_Task(void const * argument)
{
  /* USER CODE BEGIN CAN2_Decode_Task */
	uint32_t err;
	//记录接收到的次数，四个电机都接受到之后向上位机返回数�???
	static int RX_count=0;
  /* Infinite loop */
  for(;;)
  {
	 err= ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	   if(err==1)
	    {
		   LED2_ON
		   RX_count++;
		   if(Motor_Recieve_Single_CAN2.master_id==0xfd)
		       {

			   Motor_Recieve_Single_CAN2.current_position_f=uint_to_float(Motor_Recieve_Single_CAN2.current_position,(-4.0*PI),(4.0*PI),16);
			   Motor_Recieve_Single_CAN2.current_speed_f=uint_to_float(Motor_Recieve_Single_CAN2.current_speed,(-15.0),(15.0),16);//
			   Motor_Recieve_Single_CAN2.current_torque_f=uint_to_float(Motor_Recieve_Single_CAN2.current_torque,(-120.0),(120.0),16);
			   Motor_Recieve_Single_CAN2.current_temp_f=(float)Motor_Recieve_Single_CAN2.current_temp/10;

		       	 switch(Motor_Recieve_Single_CAN2.motor_id)
		   			{
		   			  case 1://RS04电机
		   			  {
		   				 CAN_2.ID_1_Motor_recieve=Motor_Recieve_Single_CAN2;
		   				  break;
		   			  }
		   			  case 2://RS04电机
		   			  {
		   				CAN_2.ID_2_Motor_recieve=Motor_Recieve_Single_CAN2;
		   				  break;
		   			  }
		   			  case 3://RS04电机
		   			  {
		   				CAN_2.ID_3_Motor_recieve=Motor_Recieve_Single_CAN2;
		   				  break;
		   			  }
		   			  case 4://RS02电机
		   			  {
		   				
		   				 Motor_Recieve_Single_CAN2.current_speed_f=uint_to_float(Motor_Recieve_Single_CAN2.current_speed,(-44.0),(44.0),16);//
		   				 Motor_Recieve_Single_CAN2.current_torque_f=uint_to_float(Motor_Recieve_Single_CAN2.current_torque,(-17.0),(17.0),16);
		   				 CAN_2.ID_4_Motor_recieve=Motor_Recieve_Single_CAN2;
		   				  break;
		   			  }
		   			  default:
		   				  break;

		   			}
		       	}


	    }
  }
  /* USER CODE END CAN2_Decode_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */



/*
 * @brief 电机初始�???  设置腿结构体参数。电机信息，电机id，主机id,帧头的相关数�???,默认参数，限�???
 * */

void Can_Bus_Motor_Init(void)
{


	//其他参数设置
	CAN_1.ID_1_Motor_send.id=1;
	CAN_1.ID_1_Motor_send.res=0x04;
	CAN_1.ID_1_Motor_send.max_position=2;
	CAN_1.ID_1_Motor_send.min_position=-2;

	CAN_1.ID_2_Motor_send.id=2;
	CAN_1.ID_2_Motor_send.res=0x04;
	CAN_1.ID_2_Motor_send.max_position=2;
	CAN_1.ID_2_Motor_send.min_position=-2;

	CAN_1.ID_3_Motor_send.id=3;
	CAN_1.ID_3_Motor_send.res=0x04;
	CAN_1.ID_3_Motor_send.max_position=2;
	CAN_1.ID_3_Motor_send.min_position=-2;

	CAN_1.ID_4_Motor_send.id=4;
	CAN_1.ID_4_Motor_send.res=0x04;


	CAN_2.ID_1_Motor_send.id=1;
	CAN_2.ID_1_Motor_send.res=0x04;
	CAN_2.ID_1_Motor_send.max_position=2;
	CAN_2.ID_1_Motor_send.min_position=-2;

	CAN_2.ID_2_Motor_send.id=2;
	CAN_2.ID_2_Motor_send.res=0x04;
	CAN_2.ID_2_Motor_send.max_position=2;
	CAN_2.ID_2_Motor_send.min_position=-2;

	CAN_2.ID_3_Motor_send.id=3;
	CAN_2.ID_3_Motor_send.res=0x04;
	CAN_2.ID_3_Motor_send.max_position=2;
	CAN_2.ID_3_Motor_send.min_position=-2;

	CAN_2.ID_4_Motor_send.id=4;
	CAN_2.ID_4_Motor_send.res=0x04;




}




/* USER CODE END Application */
