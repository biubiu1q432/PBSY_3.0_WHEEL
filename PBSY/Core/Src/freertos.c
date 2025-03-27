/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "atk_ms901m.h"
#include "atk_ms901m_uart.h"
#include "math.h"
#include "vl6180x.h"
#include "tim.h"
#include "motor.h"
#include "dma.h"
#include "semphr.h"
#include "queue.h"
#include "pid.h"
#include "atk_mw579_uart.h"
#include "atk_mw579.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

void Task_Stat_Print(void);
void CarStatParmPrint(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


#if ISBLUE == 1
uint8_t blue_tmp;
#endif


bool mpu_isReady = false;
bool lidar_isReady = false;

//MPU
float sita;
extern uint8_t mpu_once_isr_data[MPU_ISR_TRIG];
extern atk_ms901m_gyro_data_t gyro_dat;
extern atk_ms901m_accelerometer_data_t accelerometer_dat;
extern atk_ms901m_quaternion_data_t quaternion_dat;
extern atk_ms901m_attitude_data_t attitude_dat;


//TARGET PARA
TARGET_PARA TargetPara;


//Lidar
Lidar lidar;
int Queue_lidar[5] = { 255,255,255,255,255 }; // 初始化判断数组
int q_size = sizeof(Queue_lidar) / sizeof(Queue_lidar[0]);

//MOTOR
extern Motor_Stat LEFT_MOTOR;
extern Motor_Stat RIG_MOTOR;

//CAR_STAT
CAR_STAT Car_stat;
MOVE_TASK_STAT move_task_stat;
TASK_STAT TaskStat;


//CARD
extern uint8_t CARD_DATA[CARD_DATA_SIZE];
extern DMA_HandleTypeDef hdma_usart1_rx;

//SERIAL
extern char ORDER_DATA[ODER_DATA_SIZE];
extern DMA_HandleTypeDef hdma_usart2_rx;


/* USER CODE END Variables */
/* Definitions for GET_TASK */
osThreadId_t GET_TASKHandle;
const osThreadAttr_t GET_TASK_attributes = {
  .name = "GET_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for MPU_TASK */
osThreadId_t MPU_TASKHandle;
const osThreadAttr_t MPU_TASK_attributes = {
  .name = "MPU_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LIDAR_TASK */
osThreadId_t LIDAR_TASKHandle;
const osThreadAttr_t LIDAR_TASK_attributes = {
  .name = "LIDAR_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PID_TASK */
osThreadId_t PID_TASKHandle;
const osThreadAttr_t PID_TASK_attributes = {
  .name = "PID_TASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CARD_TASK */
osThreadId_t CARD_TASKHandle;
const osThreadAttr_t CARD_TASK_attributes = {
  .name = "CARD_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for CARD_FLAG */
osSemaphoreId_t CARD_FLAGHandle;
const osSemaphoreAttr_t CARD_FLAG_attributes = {
  .name = "CARD_FLAG"
};
/* Definitions for TASK_FLAG */
osSemaphoreId_t TASK_FLAGHandle;
const osSemaphoreAttr_t TASK_FLAG_attributes = {
  .name = "TASK_FLAG"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Get_Task(void *argument);
void Read_MPU(void *argument);
void Read_Lidar(void *argument);
void Move_Control(void *argument);
void Read_ID(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the semaphores(s) */
  /* creation of CARD_FLAG */
  CARD_FLAGHandle = osSemaphoreNew(1, 0, &CARD_FLAG_attributes);

  /* creation of TASK_FLAG */
  TASK_FLAGHandle = osSemaphoreNew(1, 0, &TASK_FLAG_attributes);

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
  /* creation of GET_TASK */
  GET_TASKHandle = osThreadNew(Get_Task, NULL, &GET_TASK_attributes);

  /* creation of MPU_TASK */
  MPU_TASKHandle = osThreadNew(Read_MPU, NULL, &MPU_TASK_attributes);

  /* creation of LIDAR_TASK */
  LIDAR_TASKHandle = osThreadNew(Read_Lidar, NULL, &LIDAR_TASK_attributes);

  /* creation of PID_TASK */
  PID_TASKHandle = osThreadNew(Move_Control, NULL, &PID_TASK_attributes);

  /* creation of CARD_TASK */
  CARD_TASKHandle = osThreadNew(Read_ID, NULL, &CARD_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Get_Task */
/**
  * @brief  Function implementing the GET_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Get_Task */
void Get_Task(void *argument)
{
  /* USER CODE BEGIN Get_Task */
	char DONE = '#';
	char STOP = '!';
	char ID_ = '@';
 
	char test_[] = "##################################################################\r\n";
    /* Infinite loop */
    for(;;)
    {
        
		//SEND
		if(TaskStat.ISDONE){
			HAL_UART_Transmit_DMA(&huart2,test_,sizeof(test_));
			TaskStat.ISDONE = false;
		}
		
		if(xSemaphoreTake(CARD_FLAGHandle,100)==pdTRUE){
			//=========封装ID=======//
			
			//=========封装ID=======//
			TaskStat.ISID= false;
		}
		
		if(TaskStat.ISSTOP){
		
			HAL_UART_Transmit_DMA(&huart2,test_,sizeof(test_));
			TaskStat.ISSTOP = false;
			
		}
		
		
		//RECIEVE
		if(xSemaphoreTake(TASK_FLAGHandle,100) == pdTRUE){

			char task_seg[3];
			float value1, value2;

			sscanf(ORDER_DATA, "%[^|]|%f|%f", task_seg, &value1, &value2);
			
			// GoAhead
			if (strcmp(task_seg, "1") == 0) {
				TargetPara.ARG_VAL = value1;
				TargetPara.SITA = Car_stat.Car_Alpha;
				
				move_task_stat.AHEAD_FLAG = true;
			} 
			
			//	Turn
			else if (strcmp(task_seg, "2") == 0) {
				TargetPara.MAX_VAL = value2;
				TargetPara.SITA = value1 + Car_stat.Car_Alpha;
				
				//坐标系整合
				if(TargetPara.SITA > 180.f)TargetPara.SITA = (TargetPara.SITA - 180.f) - 180.f ;
				if(TargetPara.SITA < -180.f)TargetPara.SITA = (TargetPara.SITA + 180.f) + 180.f ;


				printf("GET ORFER !!!!!!!   TURN VAL : %f  TARGET SITA:  %f\r\n",value2,value1);
				move_task_stat.TURN_FLAG = true;
			} 
			
			// Dis
			else if (strcmp(task_seg, "3") == 0) {
				TargetPara.SITA = Car_stat.Car_Alpha;
				TargetPara.DIS  = value1;
				TargetPara.MAX_VAL = value2;
				
				move_task_stat.DIS_FLAG = true;
			}
			
			// STop
			else if (strcmp(task_seg, "4") == 0) {				
				move_task_stat.STOP_FLAG = true;
				printf("GET_ORDER_STOP\r\n");
			}


		}
        
		
		vTaskDelay(pdMS_TO_TICKS(100));


    }
  /* USER CODE END Get_Task */
}

/* USER CODE BEGIN Header_Read_MPU */
/**
* @brief Function implementing the MPU_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Read_MPU */
void Read_MPU(void *argument)
{
  
	/* USER CODE BEGIN Read_MPU */
	
	uint8_t cnt = 0;
	uint8_t ret = atk_ms901m_init();
    float sita_init = atk_ms901m_sita_init(10);
	float sita_fliter[FILTER_RANGE] = {sita_init,sita_init,sita_init,sita_init,sita_init};
	//==========DEBUG:给pid调试一个单独参数================//
	TargetPara.SITA = sita_init;
	//==========DEBUG:给pid调试一个单独参数================//
	
	/* Infinite loop */
    for(;;)
    {
		uint8_t	ret = atk_ms901m_get_attitude(&attitude_dat,MPU_MAX_WAIT);        		
		Car_stat.Car_Alpha = GildeAverageValueFilter_float(attitude_dat.yaw,sita_fliter,FILTER_RANGE);
		
		cnt +=1;
		if(cnt == 5)mpu_isReady = true;
		vTaskDelay(pdMS_TO_TICKS(30));
	}
  /* USER CODE END Read_MPU */
}

/* USER CODE BEGIN Header_Read_Lidar */
/**
* @brief Function implementing the LIDAR_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Read_Lidar */
void Read_Lidar(void *argument)
{
  /* USER CODE BEGIN Read_Lidar */
	
	uint8_t cnt = 0;
	
//	VL6180X_Init(1);	
//	vTaskDelay(pdMS_TO_TICKS(10));
//	VL6180X_Init(2);		
//	vTaskDelay(pdMS_TO_TICKS(10));
//	VL6180X_Range_Cailbration(&lidar,CAILBRATION_DIS,CAILBRATION_REPIT);

	/* Infinite loop */
    for(;;)
    {
		
//		lidar.LefLidar = VL6180X_Read_Range(1) - lidar.Lef_Cali;
//		lidar.RigLidar = VL6180X_Read_Range(2) - lidar.Rig_Cali;
//		Sliding_Window_Algorithm(Queue_lidar,q_size,&lidar);
		
		
		cnt+=1;
		if(cnt == 5)lidar_isReady = true;
		vTaskDelay(pdMS_TO_TICKS(25));


    }
  /* USER CODE END Read_Lidar */
}

/* USER CODE BEGIN Header_Move_Control */
/**
* @brief Function implementing the PID_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Move_Control */
void Move_Control(void *argument)
{
  /* USER CODE BEGIN Move_Control */
	
	uint8_t ret = 0;
	PID_init();
	Car_Move_Stat_Refresh();
	Car_Dis_ReFresh();

	
	while(lidar_isReady != true || mpu_isReady != true)vTaskDelay(pdMS_TO_TICKS(50));
	
	printf("\n ALL_IS_READEY !!!\r\n");
	/* Infinite loop */
    
	
	for(;;)
    {
		
		CarStat_Get();	
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

		
		CarStatParmPrint();
		
		
		//CarGoAhead_Lidar(45);
		
		//CarGoAhead(45,TargetPara.SITA);

        
		
		/*内部调用*/
		if(move_task_stat.STOP_FLAG) {
			printf("RUN_STOP_ORDER\r\n");
			Car_Move_Stat_Refresh();
			Car_Dis_ReFresh();
			ret  += CarStop();
			if(ret >= 3){
				Motor_Set(0,0);
				printf("STOP OVER!\r\n");
				TaskStat.ISDONE = true;
				move_task_stat.STOP_FLAG = false;
				Car_Dis_ReFresh();
				ret = 0;
			} 

        }
		
		
		
		else if(move_task_stat.AHEAD_FLAG) {
            ret += CarGoAhead(TargetPara.ARG_VAL,TargetPara.SITA);
            if(ret>=3) {
                move_task_stat.AHEAD_FLAG = false;
				Car_Dis_ReFresh();//距离重置
				TaskStat.ISDONE = true;
				ret = 0;
            }
        }

        else if(move_task_stat.TURN_FLAG) {
            ret += CarTurn(TargetPara.SITA,Car_stat.Car_Alpha,TargetPara.MAX_VAL,0.5);
			printf("RUN_TURN\r\n");

			if(ret>=3) {
				printf("TURN OVER \r\n");

				move_task_stat.TURN_FLAG = false;
                Car_Alpha_ReFresh(TargetPara.SITA);
				Car_Dis_ReFresh();
				TaskStat.ISDONE = true;
				ret = 0;
            }

        }

        else if(move_task_stat.DIS_FLAG) {
			ret +=CarSetDis(TargetPara.DIS,TargetPara.MAX_VAL,Car_stat.Car_Alpha);
            if(ret>=3) {
                move_task_stat.DIS_FLAG = false;
				Car_Dis_ReFresh();
				TaskStat.ISDONE = true;
				ret = 0;
            }

        }



        vTaskDelay(pdMS_TO_TICKS(40));


    }
  /* USER CODE END Move_Control */
}

/* USER CODE BEGIN Header_Read_ID */
/**
* @brief Function implementing the CARD_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Read_ID */
void Read_ID(void *argument)
{
  /* USER CODE BEGIN Read_ID */
    /* Infinite loop */
    for(;;)
    {

        if(xSemaphoreTake(CARD_FLAGHandle,portMAX_DELAY) == pdTRUE) {
            for(int i=0; i<=sizeof(CARD_DATA_SIZE); i++) {
                //printf("%d",CARD_DATA[i]);
                CARD_DATA[i] = 0;
            }
        }

    }
  /* USER CODE END Read_ID */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/*调试*/
void CarStatParmPrint(void) {

    printf("SITA:   %f   TARGET_SITA	%f   	DIS:  %f	 L_VAL: %f	R_VAL:  %f 	L_LIDAR:%d  RIG_LIDAR: %d	\r\n",
			   Car_stat.Car_Alpha,
			   TargetPara.SITA,
			   Car_stat.Car_Dis,
			   LEFT_MOTOR.Motorspeed,
			   RIG_MOTOR.Motorspeed,
			   lidar.LefLidar,
			   lidar.RigLidar);
		
	//printf("L:%f  R:%f\r\n",lidar.LefLidar,lidar.RigLidar);

	//printf("stop_flag : %d  turn_flag : %d  isDone : %d\r\n",move_task_stat.STOP_FLAG,move_task_stat.TURN_FLAG,TaskStat.ISDONE);
}



/*FIND_COLOR_STOP_ISR*/
BaseType_t xHigherPriorityTaskWoken_stop = pdFALSE;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == R_FC_Pin || GPIO_Pin==L_FC_Pin) move_task_stat.STOP_FLAG = true;     
}


/*MPU_RXNE_ISR*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if( (huart->Instance == UART5) && ((huart->Instance->SR&USART_SR_RXNE) == RESET) ) {
        atk_ms901m_uart_rx_fifo_write(mpu_once_isr_data, MPU_ISR_TRIG);
        HAL_UART_Receive_IT(&huart5,mpu_once_isr_data,MPU_ISR_TRIG);
    }
}


/*SERIAL_DMA_ISR*/
BaseType_t xHigherPriorityTaskWoken_card = pdFALSE;
BaseType_t xHigherPriorityTaskWoken_serial = pdFALSE;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

    //CARD
    if(huart == &huart1) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1,CARD_DATA,sizeof(CARD_DATA));
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
        if(CARD_FLAGHandle != NULL)	{
            BaseType_t err = xSemaphoreGiveFromISR(CARD_FLAGHandle,&xHigherPriorityTaskWoken_card);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken_card);
        }

    }

    //SERIAL
    if(huart == &huart2) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2,ORDER_DATA,sizeof(ORDER_DATA));//SERIAL
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
        if(TASK_FLAGHandle != NULL)	{
            BaseType_t err = xSemaphoreGiveFromISR(TASK_FLAGHandle,&xHigherPriorityTaskWoken_serial);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken_serial);
        }

    }
	
	
	//BLUE
#if ISBLUE == 1
	extern struct rx_frame g_uart_rx_frame;
	if(huart==&huart4){
			
		HAL_UART_Receive_IT(&huart4, &blue_tmp, 1);           /* 使能寄存器非空中断*/
		
		if (g_uart_rx_frame.sta.len < (ATK_MW579_UART_RX_BUF_SIZE - 1))     /* 判断UART接收缓冲是否溢出*/																																																																		
		{
				g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = blue_tmp;             /* 将接收到的数据写入缓冲 */
				g_uart_rx_frame.sta.len++;                                      /* 更新接收到的数据长度 */
		}
		else                                                                /* UART接收缓冲溢出 */
		{
				g_uart_rx_frame.sta.len = 0;                                    /* 覆盖之前收到的数据 */
				g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = blue_tmp;             /* 将接收到的数据写入缓冲 */
				g_uart_rx_frame.sta.len++;                                      /* 更新接收到的数据长度 */
		}
	
	}
#endif
}

/*
 task name		task 当前状态 	 													task 优先级   				最小剩余 task 栈空间

						X: running     正在运行 			 								越大优先级越高	    剩余！
						B: blocked    等待（自动，osdelay,二值等待）
						R: ready        就绪
						D: deleted     删除
						S: suspended挂起（手动）
*/
void Task_Stat_Print(void) {

//		TaskStatus_t taskInfo ;
//		BaseType_t getFreeSpace = pdTRUE;
//		eTaskState taskState = eInvalid;
//
//		TaskHandle_t taskhandle = GET_TASKHandle ;
//		vTaskGetInfo(taskhandle,&taskInfo,getFreeSpace,taskState);
//		printf("Task Name: %s  Task State: %d  Stack High water mark: %d\r\n",taskInfo.pcTaskName,taskInfo.eCurrentState,taskInfo.usStackHighWaterMark);
//
//		taskhandle = MPU_TASKHandle ;
//		vTaskGetInfo(taskhandle,&taskInfo,getFreeSpace,taskState);
//		printf("Task Name: %s  Task State: %d  Stack High water mark: %d\r\n",taskInfo.pcTaskName,taskInfo.eCurrentState,taskInfo.usStackHighWaterMark);
//
//		taskhandle = LIDAR_TASKHandle ;
//		vTaskGetInfo(taskhandle,&taskInfo,getFreeSpace,taskState);
//		printf("Task Name: %s  Task State: %d  Stack High water mark: %d\r\n",taskInfo.pcTaskName,taskInfo.eCurrentState,taskInfo.usStackHighWaterMark);
//
//		taskhandle = PID_TASKHandle ;
//		vTaskGetInfo(taskhandle,&taskInfo,getFreeSpace,taskState);
//		printf("Task Name: %s  Task State: %d  Stack High water mark: %d\r\n",taskInfo.pcTaskName,taskInfo.eCurrentState,taskInfo.usStackHighWaterMark);
//
//		taskhandle = CARD_TASKHandle ;
//		vTaskGetInfo(taskhandle,&taskInfo,getFreeSpace,taskState);
//		printf("Task Name: %s  Task State: %d  Stack High water mark: %d\r\n",taskInfo.pcTaskName,taskInfo.eCurrentState,taskInfo.usStackHighWaterMark);
//		printf("\r\n");

    char InfoBuffer[200];
    vTaskList(InfoBuffer);
    printf("taskName ttaskState ttaskPrio ttaskStack ttaskNum\r\n");
    printf("%s",InfoBuffer);
    printf("\r\n");
    printf("\r\n");
    printf("LF %d RIG %d \r\n",lidar.LefLidar,lidar.RigLidar);

}


/* USER CODE END Application */

