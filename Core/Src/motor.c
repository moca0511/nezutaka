/*
 * motor.c
 *
 *  Created on: Jul 16, 2020
 *      Author: junmo
 */
#include"motor.h"
#include "cmsis_os.h"
#include "buzzer.h"
#include "nezutaka.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
uint32_t MotorHz_R = 0;
uint32_t MotorHz_L = 0;
uint32_t MotorStepCount_R = 0;
uint32_t MotorStepCount_L = 0;
uint32_t MOTORSPEED_R = 0;
uint32_t MOTORSPEED_L = 0;

extern void MOTOR_R(void *argument) {
	/* USER CODE BEGIN MOTOR_R */
	osDelay(10);
	uint32_t hz = 0;
	uint32_t speed_prev = 0;

	__HAL_TIM_SET_COMPARE(&htim1, STEPPER_CLOCK_R_CHANNEL, 50);

	/* Infinite loop */
	for (;;) {
		if (MOTORSPEED_R != speed_prev) {
			speed_prev = MOTORSPEED_R;
			hz = SPEEDtoHz(MOTORSPEED_R);
			if (hz == 0) {
				HAL_TIM_PWM_Stop_IT(&htim1, STEPPER_CLOCK_R_CHANNEL);
			} else {
				htim1.Init.Prescaler = 1680000 / hz - 1;
				htim1.Init.Period = 100;
				if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
					Error_Handler();
				}
				HAL_TIM_PWM_Start_IT(&htim1, STEPPER_CLOCK_R_CHANNEL);
			}
		}
		osDelay(1);
	}

	/* USER CODE END MOTOR_R */
}

/* USER CODE BEGIN Header_MOTOR_L */
/**
 * @brief Function implementing the MOTOR_L_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MOTOR_L */
extern void MOTOR_L(void *argument) {
	/* USER CODE BEGIN MOTOR_L */

	osDelay(10);
	uint32_t hz = 0;
	uint32_t speed_prev = 0;

	__HAL_TIM_SET_COMPARE(&htim8, STEPPER_CLOCK_L_CHANNEL, 50);

	/* Infinite loop */
	for (;;) {
		if (MOTORSPEED_L != speed_prev) {
			speed_prev = MOTORSPEED_L;
			hz = SPEEDtoHz(MOTORSPEED_L);
			if (hz == 0) {
				HAL_TIMEx_PWMN_Stop_IT(&htim8, STEPPER_CLOCK_L_CHANNEL);
			} else {
				htim8.Init.Prescaler = 1680000 / hz - 1;
				htim8.Init.Period = 100;
				if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
					Error_Handler();
				}
				HAL_TIMEx_PWMN_Start_IT(&htim8, STEPPER_CLOCK_L_CHANNEL);
			}
		}
		osDelay(1);
	}
	/* USER CODE END MOTER_SLEEP_CHECK */
}

extern void MOTER_SLEEP_CHECK(void *argument) {
	osDelay(10);
	int sleepcount = 0;
	for (;;) {
		if (MOTORSPEED_R == 0 && MOTORSPEED_L == 0) {
			if (sleepcount == 1000) {
				HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_SET);
				sleepcount=0;
			} else {
				sleepcount++;
			}
		} else {
			HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_RESET);
		}
		osDelay(100);
	}
}

uint32_t SPEEDtoHz(uint32_t speed) {
	return speed / STEP_LENGTH;
}
uint32_t HztoSPEED(uint32_t Hz) {
	return Hz * STEP_LENGTH;
}

void mortor_direction(uint8_t motor,uint8_t direction){
	if(motor==MR){
		HAL_GPIO_WritePin(CW_CCW_R_GPIO_Port, CW_CCW_R_Pin,
				direction);
	}else{
		HAL_GPIO_WritePin(CW_CCW_L_GPIO_Port, CW_CCW_L_Pin,
						direction);
		HAL_GPIO_TogglePin(CW_CCW_L_GPIO_Port, CW_CCW_L_Pin);
	}
	return;
}
