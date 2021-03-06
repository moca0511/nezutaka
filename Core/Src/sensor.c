/*
 * sensor.c
 *
 *  Created on: Jun 22, 2020
 *      Author: junmo
 */

#include "sensor.h"
#include "timer.h"
#include "cmsis_os.h"
#include "adc.h"
#include "run.h"
#include "buzzer.h"
#include "UI.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
SensorData sensorData = { 0, 0, 0, 0 };
extern uint32_t wall_config[12];

/*
 * 説明：壁センサ読み取りタスク
 * 引数：無し
 * 戻り値：無し
 */
extern void Sensor(void *argument) {
	/* USER CODE BEGIN Sensor */
	Delay_ms(10);
	ADCSet ADC_config = { &hadc1, ADC_CHANNEL_RS };
	PWMconfig pwm_config = { &htim3, PWM_CHANNEL_RS, 5000, 50, 84000000 };
	int32_t data_buf = 0;
	while (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
	osWaitForever) != TASK_START)
		;
	/* Infinite loop */
	for (;;) {
		if (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
				5U) == TASK_STOP) {
//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("SSTOP\n");
//				osMutexRelease(UART_MutexHandle);
//			}
			HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_RS);
			HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_RF);
			HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_LS);
			HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_LF);
//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("Sn\n\n");
//				osMutexRelease(UART_MutexHandle);
//			}
			while (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
			osWaitForever) != TASK_START)
				;
//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("Sstart\n");
//				osMutexRelease(UART_MutexHandle);
//			}

		}
		/*} else {
		 if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		 printf("Sy\n\n");
		 osMutexRelease(UART_MutexHandle);
		 }
		 }*/

		//LED_RS on
		pwm_config.pin = PWM_CHANNEL_RS;
		PWM_Set(&pwm_config);
		HAL_TIM_PWM_Start_IT(pwm_config.htim, pwm_config.pin);

		//wait20us
		wait_us(20);

//Sensor RS read
		ADC_config.channel = ADC_CHANNEL_RS;
		data_buf = ADConv(&ADC_config);
		if (data_buf > 400) {
			sensorData.ADC_DATA_RS = data_buf;
		}
//LED_RS off
		HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_RS);
//		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("RS");
//			osMutexRelease(UART_MutexHandle);
//		}

//LED_RF on
		pwm_config.pin = PWM_CHANNEL_RF;
		PWM_Set(&pwm_config);
		HAL_TIM_PWM_Start_IT(pwm_config.htim, pwm_config.pin);

		//wait20us
		wait_us(20);

		//Sensor_RF read
		ADC_config.channel = ADC_CHANNEL_RF;
		data_buf = ADConv(&ADC_config);
		if (data_buf > 400) {
			sensorData.ADC_DATA_RF = data_buf;
		}

		//LED_RF off
		HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_RF);
//		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("RF");
//			osMutexRelease(UART_MutexHandle);
//		}
		//LED_LS on
		pwm_config.pin = PWM_CHANNEL_LS;
		PWM_Set(&pwm_config);
		HAL_TIM_PWM_Start_IT(pwm_config.htim, pwm_config.pin);

		//wait20us
		wait_us(20);

		//sensor_LS read
		ADC_config.channel = ADC_CHANNEL_LS;
		data_buf = ADConv(&ADC_config);
		if (data_buf > 400) {
			sensorData.ADC_DATA_LS = data_buf;
		}

		//LED_LS off
		HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_LS);
//		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("LS");
//			osMutexRelease(UART_MutexHandle);
//		}
		//LED_LF on
		pwm_config.pin = PWM_CHANNEL_LF;
		PWM_Set(&pwm_config);
		HAL_TIM_PWM_Start_IT(pwm_config.htim, pwm_config.pin);

		//wait20us
		wait_us(20);

		//Sensor_LF read
		ADC_config.channel = ADC_CHANNEL_LF;
		data_buf = ADConv(&ADC_config);
		if (data_buf > 400) {
			sensorData.ADC_DATA_LF = data_buf;
		}

		//LED_LF off
		HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_LF);
//		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("LF");
//			osMutexRelease(UART_MutexHandle);
//		}
		/*		Delay_ms(2);*/
		osThreadYield();

		//task sycle time

	}
	/* USER CODE END Sensor */
}
/*
 * 説明：指定壁センサの値を複数回読み平均をとる
 * 引数：select 壁センサ指定
 * 戻り値：無し
 */
uint32_t read_wall(uint8_t select) {
	uint32_t value = 0;
	uint32_t* pADCdata;
	switch (select) {
		case RS:
			pADCdata=&sensorData.ADC_DATA_RS;
			break;
		case RF:
			pADCdata=&sensorData.ADC_DATA_RF;
			break;
		case LS:
			pADCdata=&sensorData.ADC_DATA_LS;
			break;
		case LF:
			pADCdata=&sensorData.ADC_DATA_LF;
			break;
		default:
			return 0;
		}
	value = *pADCdata;
	for (int i = 0; i < 5; i++) {
		value += *pADCdata;
		value /= 2;
		Delay_ms(1);
	}
	return value;
}

/*
 * 説明：壁センサの各リファレンス値取得
 * 引数：無し
 * 戻り値：無し
 */
void wall_calibration(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 0, 100, 500, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) / 2 };
	RUNConfig turn_config = { TURN_R, 0, 0, 100, 500, 90 };

	tone(tone_hiC, 10);
	for (int i = 0; i < 12; i++) {
		wall_config[i] = 0;
	}

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(100);
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		printf("RS_threshould\n");
		osMutexRelease(UART_MutexHandle);
	}
	UILED_SET(1);
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 1) {
		Delay_ms(50);
	}
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
		Delay_ms(50);
	}
	tone(tone_hiC, 50);
	wall_config[RS_threshold] = read_wall(RS);
	tone(tone_C, 50);
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		printf("LS_threshould\n");
		osMutexRelease(UART_MutexHandle);
	}
	UILED_SET(8);
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 1) {
		Delay_ms(50);
	}
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
		Delay_ms(50);
	}
	tone(tone_hiC, 50);

	wall_config[LS_threshold] = read_wall(LS);
	tone(tone_C, 50);
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		printf("F_threshould\n");
		osMutexRelease(UART_MutexHandle);
	}
	UILED_SET(6);
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 1) {
		Delay_ms(50);
	}
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
		Delay_ms(50);
	}
	tone(tone_hiC, 50);
	wall_config[RF_threshold] = read_wall(RF);
	wall_config[LF_threshold] = read_wall(LF);
	tone(tone_C, 50);
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		printf("ALL_WALL&FREE\n");
		osMutexRelease(UART_MutexHandle);
	}
	UILED_SET(15);
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 1) {
		Delay_ms(50);
	}
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
		Delay_ms(50);
	}
	tone(tone_hiC, 50);
	osDelay(100);
	wall_config[RS_WALL] = read_wall(RS);
	wall_config[LS_WALL] = read_wall(LS);
	wall_config[RF_FREE] = read_wall(RF);
	wall_config[LF_FREE] = read_wall(LF);
	straight(RUN_config, 0, 0, 0);
	turn(turn_config);
	osDelay(100);
	wall_config[LS_FREE] = read_wall(LS);
	wall_config[RF_WALL] = read_wall(RF);
	wall_config[LF_WALL] = read_wall(LF);
	turn_config.value = 180;
	turn_config.direction = TURN_L;
	turn(turn_config);
	osDelay(100);
	wall_config[RS_FREE] = read_wall(RS);
	wall_config[RF_WALL] = (wall_config[RF_WALL]
			+ read_wall(RF)) / 2;
	wall_config[LF_WALL] = (wall_config[LF_WALL]
			+ read_wall(LF)) / 2;
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	turn_config.value = 90;
	turn_config.direction = TURN_R;
	turn(turn_config);
	sirituke();
//	mortor_sleep();

	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		for (int i = 0; i < 12; i++) {
			printf("wall[%d]=%ld,", i, wall_config[i]);
		}
		printf("\n");
		osMutexRelease(UART_MutexHandle);
	}
	tone(tone_hiC, 50);
}

/*
 * 説明：debug情報出力開始・停止
 * 引数：無し
 * 戻り値：無し
 */
void print_sensordata(void) {
	static uint8_t flag = 0;
	tone(tone_C, 50);
	if (flag == 1) {
//		osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
		osThreadFlagsSet(SENSOR_PRINT_TAHandle, TASK_STOP);
		flag = 0;
	} else {
		flag = 1;
//		osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
		osThreadFlagsSet(SENSOR_PRINT_TAHandle, TASK_START);
	}

	tone(tone_C, 50);
}
/*
 * 説明：debug情報出力
 * 引数：無し
 * 戻り値：無し
 */
extern void SENSOR_PRINT(void *argument) {
	/* USER CODE BEGIN WALL_READ */
	while (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
	osWaitForever) != TASK_START)
		;
	/* Infinite loop */
	for (;;) {
		if (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
				0U) == TASK_STOP) {
			while (osThreadFlagsWait(TASK_STOP | TASK_START, osFlagsWaitAny,
			osWaitForever) != TASK_START)
				;
		}
		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf(",%ld,%ld,%ld,%ld,%ld,%ld\n", sensorData.ADC_DATA_RS,
					sensorData.ADC_DATA_LS, sensorData.ADC_DATA_RF,
					sensorData.ADC_DATA_LF, get_MotorSpeed_R(),
					get_MotorSpeed_L());
			osMutexRelease(UART_MutexHandle);
		}
		Delay_ms(100);
//		osThreadYield();

	}
	/* USER CODE END WALL_READ */
}
/*
 * 説明：指定壁センサ値取得
 * 引数：select 壁センサ選択
 * 戻り値：センサ値
 */
uint32_t get_sensordata(uint8_t select) {
	switch (select) {
	case RS:
		return sensorData.ADC_DATA_RS;
	case RF:
		return sensorData.ADC_DATA_RF;
	case LS:
		return sensorData.ADC_DATA_LS;
	case LF:
		return sensorData.ADC_DATA_LF;
	default:
		return 0;
	}
}

