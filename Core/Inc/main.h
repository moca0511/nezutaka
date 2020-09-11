/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"uart.h"
#include<stdio.h>
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AD_LF_Pin GPIO_PIN_0
#define AD_LF_GPIO_Port GPIOC
#define AD_RF_Pin GPIO_PIN_1
#define AD_RF_GPIO_Port GPIOC
#define AD_LS_Pin GPIO_PIN_2
#define AD_LS_GPIO_Port GPIOC
#define AD_RS_Pin GPIO_PIN_3
#define AD_RS_GPIO_Port GPIOC
#define BOARD_SW_Pin GPIO_PIN_0
#define BOARD_SW_GPIO_Port GPIOA
#define BOARD_SW_EXTI_IRQn EXTI0_IRQn
#define BATTERY_Pin GPIO_PIN_1
#define BATTERY_GPIO_Port GPIOA
#define MO_R_Pin GPIO_PIN_2
#define MO_R_GPIO_Port GPIOA
#define MO_R_EXTI_IRQn EXTI2_IRQn
#define SLEEP_L_Pin GPIO_PIN_3
#define SLEEP_L_GPIO_Port GPIOA
#define SLEEP_R_Pin GPIO_PIN_4
#define SLEEP_R_GPIO_Port GPIOA
#define STEPPER_CLOCK_L_Pin GPIO_PIN_5
#define STEPPER_CLOCK_L_GPIO_Port GPIOA
#define CW_CCW_R_Pin GPIO_PIN_6
#define CW_CCW_R_GPIO_Port GPIOA
#define CW_CCW_L_Pin GPIO_PIN_7
#define CW_CCW_L_GPIO_Port GPIOA
#define LED_LS_Pin GPIO_PIN_0
#define LED_LS_GPIO_Port GPIOB
#define LED_RS_Pin GPIO_PIN_1
#define LED_RS_GPIO_Port GPIOB
#define UP_Pin GPIO_PIN_10
#define UP_GPIO_Port GPIOB
#define DOWN_Pin GPIO_PIN_11
#define DOWN_GPIO_Port GPIOB
#define OK_Pin GPIO_PIN_12
#define OK_GPIO_Port GPIOB
#define LED_LF_Pin GPIO_PIN_6
#define LED_LF_GPIO_Port GPIOC
#define LED_RF_Pin GPIO_PIN_7
#define LED_RF_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOC
#define STEPPER_CLOCK_R_Pin GPIO_PIN_8
#define STEPPER_CLOCK_R_GPIO_Port GPIOA
#define MO_L_Pin GPIO_PIN_13
#define MO_L_GPIO_Port GPIOA
#define MO_L_EXTI_IRQn EXTI15_10_IRQn
#define LED4_Pin GPIO_PIN_10
#define LED4_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOC
#define BOARD_LED_Pin GPIO_PIN_2
#define BOARD_LED_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_5
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_6
#define LED_B_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOB
#define SLEEP_LB9_Pin GPIO_PIN_9
#define SLEEP_LB9_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define TASK_START 0x00000001U
#define TASK_STOP 0x00000002U
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
