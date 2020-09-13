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

extern osMutexId_t UART_MutexHandle;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
uint32_t MotorHz_R = 0;
uint32_t MotorHz_L = 0;
uint32_t MotorStepCount_R = 0;
uint32_t MotorStepCount_L = 0;
uint32_t MOTORSPEED_R = 0;
uint32_t MOTORSPEED_L = 0;
extern osThreadId_t MOTOR_R_TaskHandle;
extern osThreadId_t MOTOR_L_TaskHandle;

extern void MOTOR_R(void *argument) {
	/* USER CODE BEGIN MOTOR_R */
	Delay_ms(10);
	uint32_t hz = 0;
	uint32_t speed_prev = 0;
	osThreadFlagsWait(TASK_START, osFlagsWaitAny, osWaitForever);
	__HAL_TIM_SET_COMPARE(&htim1, STEPPER_CLOCK_R_CHANNEL, 50);

	/* Infinite loop */
	for (;;) {
		if (osThreadFlagsWait(TASK_STOP, osFlagsWaitAny, 5U) == TASK_STOP) {
			osThreadFlagsWait(TASK_START, osFlagsWaitAny, osWaitForever);
		}
		if (MOTORSPEED_R != speed_prev) {
//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("MOTORSPEED_R:%d \n", MOTORSPEED_R);
//				osMutexRelease(UART_MutexHandle);
//			}
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
		//Delay_ms(5);
		osThreadYield();
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

	Delay_ms(10);
	uint32_t hz = 0;
	uint32_t speed_prev = 0;
	osThreadFlagsWait(TASK_START, osFlagsWaitAny, osWaitForever);
	__HAL_TIM_SET_COMPARE(&htim8, STEPPER_CLOCK_L_CHANNEL, 50);

	/* Infinite loop */
	for (;;) {
		if (osThreadFlagsWait(TASK_STOP, osFlagsWaitAny, 5U) == TASK_STOP) {
			osThreadFlagsWait(TASK_START, osFlagsWaitAny, osWaitForever);
		}
		if (MOTORSPEED_L != speed_prev) {

//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("MOTORSPEED_L:%d \n", MOTORSPEED_L);
//				osMutexRelease(UART_MutexHandle);
//			}
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
		//Delay_ms(5);
		osThreadYield();
	}
	/* USER CODE END MOTER_SLEEP_CHECK */
}
/*
extern void MOTER_SLEEP_CHECK(void *argument) {
	Delay_ms(10);
	int16_t sleepcount = 10;
	for (;;) {
		if (MOTORSPEED_R == 0 && MOTORSPEED_L == 0) {
			if (sleepcount <= 0) {
				HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_SET);
				sleepcount = 10;
			} else {
				sleepcount--;
			}
		} else {
			HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_RESET);
		}
		Delay_ms(10);
//		osThreadYield();
	}
}*/

uint32_t SPEEDtoHz(uint32_t speed) {
	return speed / STEP_LENGTH;
}
uint32_t HztoSPEED(uint32_t Hz) {
	return Hz * STEP_LENGTH;
}

void mortor_direction(uint8_t motor, uint8_t direction) {
	if (motor == MR) {
		HAL_GPIO_WritePin(CW_CCW_R_GPIO_Port, CW_CCW_R_Pin, direction);
	} else {
		HAL_GPIO_WritePin(CW_CCW_L_GPIO_Port, CW_CCW_L_Pin, direction);
		HAL_GPIO_TogglePin(CW_CCW_L_GPIO_Port, CW_CCW_L_Pin);
	}
	return;
}

void mortor_stop(void) {
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_START);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_START);
	MOTORSPEED_R = 0;
	MOTORSPEED_L = 0;
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_STOP);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_STOP);
	Delay_ms(50);
	HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_SET);
}