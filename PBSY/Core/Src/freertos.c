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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//MPU
float sita;
extern uint8_t mpu_once_isr_data;
extern atk_ms901m_gyro_data_t gyro_dat;
extern atk_ms901m_accelerometer_data_t accelerometer_dat;
extern atk_ms901m_quaternion_data_t quaternion_dat;
extern atk_ms901m_attitude_data_t attitude_dat;


//MOTOR
extern Motor_Stat LEFT_MOTOR;
extern Motor_Stat RIG_MOTOR;



/* USER CODE END Variables */
/* Definitions for GET_TASK */
osThreadId_t GET_TASKHandle;
const osThreadAttr_t GET_TASK_attributes = {
    .name = "GET_TASK",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MPU_TASK */
osThreadId_t MPU_TASKHandle;
const osThreadAttr_t MPU_TASK_attributes = {
    .name = "MPU_TASK",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LIDAR_TASK */
osThreadId_t LIDAR_TASKHandle;
const osThreadAttr_t LIDAR_TASK_attributes = {
    .name = "LIDAR_TASK",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PID_TASK */
osThreadId_t PID_TASKHandle;
const osThreadAttr_t PID_TASK_attributes = {
    .name = "PID_TASK",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CARD_TASK */
osThreadId_t CARD_TASKHandle;
const osThreadAttr_t CARD_TASK_attributes = {
    .name = "CARD_TASK",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityLow,
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
    TASK_FLAGHandle = osSemaphoreNew(1, 1, &TASK_FLAG_attributes);

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
    /* Infinite loop */
    for(;;)
    {

        vTaskDelay(pdMS_TO_TICKS(10));
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

//    float sita_fliter[5];
//    uint8_t ret;
//    ret = atk_ms901m_init();
//    if(ret == ATK_MS901M_ERROR) {
//        printf("ATK_MS901M_ERROR");
//    }
//    float sita_init =atk_ms901m_sita_init();

    /* Infinite loop */
    for(;;)
    {
//        ret = atk_ms901m_get_attitude(&attitude_dat,100);

//        float tmp_sita = attitude_dat.yaw;
//        sita = GildeAverageValueFilter_float(tmp_sita,sita_fliter,5) - sita_init;
//        printf("MPU: %f\r\n",sita);
        vTaskDelay(pdMS_TO_TICKS(10));

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

//    uint8_t L_D = 0;
//    uint8_t R_D = 0;
//    uint8_t ret1= VL6180X_Init(0);
//    uint8_t ret2 = VL6180X_Init(1);
//    printf("VL START!");
    /* Infinite loop */
    for(;;)
    {
        //L_D = VL6180X_Read_Range(0);
//        R_D = VL6180X_Read_Range(1);

//        printf("LD: %d",L_D);
//        printf("R_D: %d",R_D);

        vTaskDelay(pdMS_TO_TICKS(20));
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
    
	/* Infinite loop */
    for(;;)
    {
		MotorSpeed_Get();
		vTaskDelay(pdMS_TO_TICKS(50));
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
        HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    /* USER CODE END Read_ID */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/*FIND_COLOR_STOP_ISR*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

    if(GPIO_Pin == R_FC_Pin || GPIO_Pin==L_FC_Pin) {

    }

}

/*MPU_RXNE_ISR*/
extern uint8_t TEST;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if( (huart->Instance == UART5) && ((huart->Instance->SR&USART_SR_RXNE) == RESET) ) {
        atk_ms901m_uart_rx_fifo_write(&mpu_once_isr_data, 1);
        HAL_UART_Receive_IT(&huart5,&mpu_once_isr_data,1);
    }
}


/*SERIAL_DMA_ISR*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

    //CARD
    if(huart == &huart1) {

    }

    //SERIAL
    if(huart == &huart2) {

    }


}


/* USER CODE END Application */

