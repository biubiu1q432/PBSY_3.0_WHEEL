/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RESERVE_PIN1_Pin GPIO_PIN_1
#define RESERVE_PIN1_GPIO_Port GPIOC
#define RESERVE_PIN2_Pin GPIO_PIN_2
#define RESERVE_PIN2_GPIO_Port GPIOC
#define R_EC_B_Pin GPIO_PIN_0
#define R_EC_B_GPIO_Port GPIOA
#define R_EC_A_Pin GPIO_PIN_1
#define R_EC_A_GPIO_Port GPIOA
#define Serial_TX_Pin GPIO_PIN_2
#define Serial_TX_GPIO_Port GPIOA
#define Serial_RX_Pin GPIO_PIN_3
#define Serial_RX_GPIO_Port GPIOA
#define L_PWM_1_Pin GPIO_PIN_6
#define L_PWM_1_GPIO_Port GPIOA
#define L_PWM_2_Pin GPIO_PIN_7
#define L_PWM_2_GPIO_Port GPIOA
#define R_PWM_1_Pin GPIO_PIN_0
#define R_PWM_1_GPIO_Port GPIOB
#define R_PWM_2_Pin GPIO_PIN_1
#define R_PWM_2_GPIO_Port GPIOB
#define R_SCL_Pin GPIO_PIN_10
#define R_SCL_GPIO_Port GPIOB
#define R_SDA_Pin GPIO_PIN_11
#define R_SDA_GPIO_Port GPIOB
#define L_FC_Pin GPIO_PIN_14
#define L_FC_GPIO_Port GPIOB
#define L_FC_EXTI_IRQn EXTI15_10_IRQn
#define R_FC_Pin GPIO_PIN_15
#define R_FC_GPIO_Port GPIOB
#define R_FC_EXTI_IRQn EXTI15_10_IRQn
#define RESERVE_PWM1_Pin GPIO_PIN_6
#define RESERVE_PWM1_GPIO_Port GPIOC
#define RESERVE_PWM2_Pin GPIO_PIN_9
#define RESERVE_PWM2_GPIO_Port GPIOC
#define L_EC_B_Pin GPIO_PIN_8
#define L_EC_B_GPIO_Port GPIOA
#define L_EC_A_Pin GPIO_PIN_9
#define L_EC_A_GPIO_Port GPIOA
#define Debug_TX_Pin GPIO_PIN_10
#define Debug_TX_GPIO_Port GPIOC
#define Debug_RX_Pin GPIO_PIN_11
#define Debug_RX_GPIO_Port GPIOC
#define MPU_TX_Pin GPIO_PIN_12
#define MPU_TX_GPIO_Port GPIOC
#define MPU_RX_Pin GPIO_PIN_2
#define MPU_RX_GPIO_Port GPIOD
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOB
#define Card_TX_Pin GPIO_PIN_6
#define Card_TX_GPIO_Port GPIOB
#define Card_RX_Pin GPIO_PIN_7
#define Card_RX_GPIO_Port GPIOB
#define L_SCL_Pin GPIO_PIN_8
#define L_SCL_GPIO_Port GPIOB
#define L_SDA_Pin GPIO_PIN_9
#define L_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */



//UART
#define UART_PRINT 2

//PWM
#define MAX_PWM 1000
#define MOTOR_ARR 1000
#define MIN_PWM 100

//EC
#define EC_ARR	65535		
#define ENCODER_TIME 0.050	
#define LEFT_EC_1	1400			
#define RIGHT_EC_1	1400				
#define PI		3.1415926			
#define D		65						
#define C_1		PI*D		

//MPU
#define MPU_MAX_WAIT 15

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
