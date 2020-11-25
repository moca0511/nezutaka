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
uint32_t MotorSPEED_R = 0;
uint32_t MotorSPEED_L = 0;
uint32_t MotorStepCount_R = 0;
uint32_t MotorStepCount_L = 0;

/*
 * 説明：右モータ制御タスク
 * 引数：無し
 * 戻り値:無し
 */
extern void MOTOR_R(void *argument) {
	/* USER CODE BEGIN MOTOR_R */
	Delay_ms(10);
	uint32_t hz = 0;
	uint32_t speed_prev = 0;
	__HAL_TIM_SET_COMPARE(&htim1, STEPPER_CLOCK_R_CHANNEL, 50);
	while (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
	osWaitForever) != TASK_START)
		;
	/* Infinite loop */
	for (;;) {
		if (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
				2U) == TASK_STOP) {
			while (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
			osWaitForever) != TASK_START)
				;
		}
		if (MotorSPEED_R != speed_prev) {

//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("MotorHz_R:%ld \n", MotorHz_R);
//				osMutexRelease(UART_MutexHandle);
//			}
			speed_prev = MotorSPEED_R;
			hz = SPEEDtoHz(MotorSPEED_R);
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
		osSemaphoreRelease(SchengeRSemHandle);
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
/*
 * 説明：左モータ制御タスク
 * 引数：無し
 * 戻り値:無し
 */
extern void MOTOR_L(void *argument) {
	/* USER CODE BEGIN MOTOR_L */

	Delay_ms(10);
	uint32_t hz = 0;
	uint32_t speed_prev = 0;
	__HAL_TIM_SET_COMPARE(&htim8, STEPPER_CLOCK_L_CHANNEL, 50);
	while (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
	osWaitForever) != TASK_START)
		;
	/* Infinite loop */
	for (;;) {
		if (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
				2U) == TASK_STOP) {
			while (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
			osWaitForever) != TASK_START)
				;
		}
		if (MotorSPEED_L != speed_prev) {

//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("MotorHz_L:%ld \n", MotorHz_L);
//				osMutexRelease(UART_MutexHandle);
//			}
			speed_prev = MotorSPEED_L;
			hz = SPEEDtoHz(MotorSPEED_L);
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
		osSemaphoreRelease(SchengeLSemHandle);
		osThreadYield();
	}
	/* USER CODE END MOTER_SLEEP_CHECK */
}

/*
 * 説明：モータスリープ制御タスク
 * 引数：無し
 * 戻り値:無し
 */
extern void MORTOR_SLEEP_CHECK(void *argument) {
	Delay_ms(10);
	int16_t sleepcount = 5;
	for (;;) {
		if (MotorSPEED_R == 0 && MotorSPEED_L == 0) {
			if (sleepcount <= 0) {
				HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_SET);
				sleepcount = 5;
			} else {
				sleepcount--;
			}
		} else {
			HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_RESET);
			sleepcount = 5;
		}
		Delay_ms(100);
		//		osThreadYield();
	}
}
/*
 * 説明：mm/s→Hz変換
 * 引数：無し
 * 戻り値:無し
 */
float32_t SPEEDtoHz(float32_t speed) {
	return speed / STEP_LENGTH;
}
/*
 * 説明：Hz→mm/s変換
 * 引数：無し
 * 戻り値:無し
 */
float32_t HztoSPEED(float32_t Hz) {
	return Hz * STEP_LENGTH;
}

/*
 * 説明：モータ回転方向設定
 * 引数：mortor 左右選択
 * 　　　direction 回転方向
 * 戻り値:無し
 */
void mortor_direction(uint8_t motor, uint8_t direction) {
	if (motor == MR) {
		HAL_GPIO_WritePin(CW_CCW_R_GPIO_Port, CW_CCW_R_Pin, direction);
	} else {
		HAL_GPIO_WritePin(CW_CCW_L_GPIO_Port, CW_CCW_L_Pin, direction);
		HAL_GPIO_TogglePin(CW_CCW_L_GPIO_Port, CW_CCW_L_Pin);
	}
	return;
}
/*
 * 説明：左モータ速度取得
 * 引数：無し
 * 戻り値:左モータ速度(mm/s)
 */
uint32_t get_MotorSpeed_L(void) {
	return MotorSPEED_L;
}
/*
 * 説明：右モータ速度取得
 * 引数：無し
 * 戻り値:右モータ速度(mm/s)
 */
uint32_t get_MotorSpeed_R(void) {
	return MotorSPEED_R;
}
/*
 * 説明：モータ速度取得(重心速度)
 * 引数：無し
 * 戻り値:重心速度(mm/s)
 */
uint32_t get_MotorSpeed(void) {
	return (MotorSPEED_R + MotorSPEED_R) / 2;
}
/*
 * 説明：左モータ速度設定
 * 引数：speed 速度(mm/s)
 * 戻り値：無し
 */
void set_MotorSpeed_L(uint32_t speed) {
	MotorSPEED_L = speed;
	return;
}
/*
 * 説明：右モータ速度設定
 * 引数：speed 速度(mm/s)
 * 戻り値：無し
 */
void set_MotorSpeed_R(uint32_t speed) {
	MotorSPEED_R = speed;
	return;
}
/*
 * 説明：両モータ速度設定
 * 引数：speed 速度(mm/s)
 * 戻り値：無し
 */
void set_MotorSpeed(uint32_t speed) {
	MotorSPEED_L = MotorSPEED_R = speed;
	return;
}
/*
 * 説明：左モータステップ数取得
 * 引数：左モータステップ数
 * 戻り値：無し
 */
uint32_t get_MotorStepCount_L(void) {
	return MotorStepCount_L;
}
/*
 * 説明：右モータステップ数取得
 * 引数：右モータステップ数
 * 戻り値：無し
 */
uint32_t get_MotorStepCount_R(void) {
	return MotorStepCount_R;
}
/*
 * 説明：モータステップ数取得
 * 引数：左右のモータステップ数平均
 * 戻り値：無し
 */
uint32_t get_MotorStepCount(void) {
	return (MotorStepCount_R + MotorStepCount_L) / 2;
}
/*
 * 説明：左モータステップ数リセット
 * 引数：無し
 * 戻り値：無し
 */
void reset_MotorStepCount_L(void) {
	MotorStepCount_L = 0;
	return;
}
/*
 * 説明：右モータステップ数リセット
 * 引数：無し
 * 戻り値：無し
 */
void reset_MotorStepCount_R(void) {
	MotorStepCount_R = 0;
	return;
}
/*
 * 説明：左右モータステップ数リセット
 * 引数：無し
 * 戻り値：無し
 */
void reset_MotorStepCount(void) {
	MotorStepCount_L = MotorStepCount_R = 0;
	return;
}

