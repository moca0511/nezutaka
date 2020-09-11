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
uint8_t sensor_debug_f;
extern osThreadId_t Sensor_TaskHandle;
extern osThreadId_t WALL_READ_TASKHandle;
extern osThreadId_t SENSOR_PRINT_TAHandle;
extern uint32_t wall_config[12];

extern void Sensor(void *argument) {
	/* USER CODE BEGIN Sensor */
	osDelay(10);
	ADCSet ADC_config = { &hadc1, ADC_CHANNEL_RS };
	PWMconfig pwm_config = { &htim3, PWM_CHANNEL_RS, 5000, 50, 84000000 };
	SensorData sensorData_buf = { 0, 0, 0, 0 };
	uint8_t count = 5;
	osThreadFlagsWait(TASK_START, osFlagsWaitAny, osWaitForever);
	/* Infinite loop */
	for (;;) {
		for (int i = 0; i < count; i++) {
			if (osThreadFlagsWait(TASK_STOP, osFlagsWaitAny, 1U) == TASK_STOP) {
				osThreadFlagsWait(TASK_START, osFlagsWaitAny, osWaitForever);
				sensorData_buf.ADC_DATA_RS = 0;
				sensorData_buf.ADC_DATA_RF = 0;
				sensorData_buf.ADC_DATA_LS = 0;
				sensorData_buf.ADC_DATA_LF = 0;
			}
			//printf("sensor\n");

			pwm_config.hz = 5000;
			pwm_config.duty = 50;

			//LED_RS on
//		pwm_config.htim = &htim3;
			pwm_config.pin = PWM_CHANNEL_RS;
//		printf("set pwmset\n");
			PWM_Set(&pwm_config);
			//	printf("pwmstart\n");
			HAL_TIM_PWM_Start_IT(pwm_config.htim, pwm_config.pin);

			//wait20us
			wait_us(20);
			//printf("wait us endend\n");

//Sensor RS read
//		ADC_config.hadc = &hadc1;
			ADC_config.channel = ADC_CHANNEL_RS;
			//printf("set sensorread\n");
			sensorData_buf.ADC_DATA_RS += ADConv(&ADC_config);
//		if (sensor_debug_f)
//			printf("RS_ADC=%5ld\n", sensorData.ADC_DATA_RS);

//LED_RS off
//printf("pwmstop\n");
			HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_RS);

			//LED_RF on
//		pwm_config.htim = &htim3;
			pwm_config.pin = PWM_CHANNEL_RF;
			PWM_Set(&pwm_config);
			HAL_TIM_PWM_Start_IT(pwm_config.htim, pwm_config.pin);

			//wait20us
			wait_us(20);

			//Sensor_RF read
//		ADC_config.hadc = &hadc1;
			ADC_config.channel = ADC_CHANNEL_RF;
			sensorData_buf.ADC_DATA_RF += ADConv(&ADC_config);
			//if (sensor_debug_f)
			//printf("RF_ADC=%5ld\n", sensorData.ADC_DATA_RF);

			//LED_RF off
			HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_RF);

			//LED_LS on
//		pwm_config.htim = &htim3;
			pwm_config.pin = PWM_CHANNEL_LS;
			PWM_Set(&pwm_config);
			HAL_TIM_PWM_Start_IT(pwm_config.htim, pwm_config.pin);

			//wait20us
			wait_us(20);

			//sensor_LS read
//		ADC_config.hadc = &hadc1;
			ADC_config.channel = ADC_CHANNEL_LS;
			sensorData_buf.ADC_DATA_LS += ADConv(&ADC_config);
			//if (sensor_debug_f)
			//printf("LS_ADC=%5ld\n", sensorData.ADC_DATA_LS);

			//LED_LS off
			HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_LS);

			//LED_LF on
//		pwm_config.htim = &htim3;
			pwm_config.pin = PWM_CHANNEL_LF;
			PWM_Set(&pwm_config);
			HAL_TIM_PWM_Start_IT(pwm_config.htim, pwm_config.pin);

			//wait20us
			wait_us(20);

			//Sensor_LF read
//		ADC_config.hadc = &hadc1;
			ADC_config.channel = ADC_CHANNEL_LF;
			sensorData_buf.ADC_DATA_LF += ADConv(&ADC_config);
			//if (sensor_debug_f)
			//printf("LF_ADC=%5ld\n\n", sensorData.ADC_DATA_LF);

			//LED_LF off
			HAL_TIM_PWM_Stop_IT(&htim3, PWM_CHANNEL_LF);
		}
		sensorData.ADC_DATA_RS = sensorData_buf.ADC_DATA_RS / count;
		sensorData.ADC_DATA_RF = sensorData_buf.ADC_DATA_RF / count;
		sensorData.ADC_DATA_LS = sensorData_buf.ADC_DATA_LS / count;
		sensorData.ADC_DATA_LF = sensorData_buf.ADC_DATA_LF / count;

		sensorData_buf.ADC_DATA_RS = 0;
		sensorData_buf.ADC_DATA_RF = 0;
		sensorData_buf.ADC_DATA_LS = 0;
		sensorData_buf.ADC_DATA_LF = 0;
		//task sycle time
	}
	/* USER CODE END Sensor */
}

uint32_t read_wall(uint32_t *pADCdata) {
	uint32_t value = 0;
	value = *pADCdata;
	for (int i = 0; i < 10; i++) {
		value += *pADCdata;
		value /= 2;
		osDelay(10);
	}
	return value;
}

//init wall value
void wall_calibration(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 0, 100, 1000, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) / 3 };
	RUNConfig turn_config = { TURN_R, 0, 0, 200, 1000, 90 };
	tone(tone_hiC, 10);
	for (int i = 0; i < 12; i++) {
		wall_config[i] = 0;
	}

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	osDelay(50);
	printf("RS_threshould\n");
	UILED_SET(1);
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 1) {
		osDelay(50);
	}
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
		osDelay(50);
	}
	tone(tone_hiC, 100);
	wall_config[RS_threshold] = read_wall(&sensorData.ADC_DATA_RS);
	tone(tone_C, 100);

	printf("LS_threshould\n");
	UILED_SET(8);
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 1) {
		osDelay(50);
	}
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
		osDelay(50);
	}
	tone(tone_hiC, 100);
	wall_config[LS_threshold] = read_wall(&sensorData.ADC_DATA_LS);
	tone(tone_C, 100);

	printf("F_threshould\n");
	UILED_SET(6);
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 1) {
		osDelay(50);
	}
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
		osDelay(50);
	}
	tone(tone_hiC, 100);
	wall_config[RF_threshold] = read_wall(&sensorData.ADC_DATA_RF);
	wall_config[LF_threshold] = read_wall(&sensorData.ADC_DATA_LF);
	tone(tone_C, 100);

	printf("ALL_WALL&FREE\n");
	UILED_SET(15);
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 1) {
		osDelay(50);
	}
	while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
		osDelay(50);
	}
	tone(tone_hiC, 100);

	wall_config[RS_WALL] = read_wall(&sensorData.ADC_DATA_RS);
	wall_config[LS_WALL] = read_wall(&sensorData.ADC_DATA_LS);
	wall_config[RF_FREE] = read_wall(&sensorData.ADC_DATA_RF);
	wall_config[LF_FREE] = read_wall(&sensorData.ADC_DATA_LF);
	straight(RUN_config);
	turn(turn_config);
	wall_config[LS_FREE] = read_wall(&sensorData.ADC_DATA_LS);
	wall_config[RF_WALL] = read_wall(&sensorData.ADC_DATA_RF);
	wall_config[LF_WALL] = read_wall(&sensorData.ADC_DATA_LF);
	turn_config.value = 180;
	turn_config.direction = TURN_L;
	turn(turn_config);
	wall_config[RS_FREE] = read_wall(&sensorData.ADC_DATA_RS);
	wall_config[RF_WALL] = (wall_config[RF_WALL]
			+ read_wall(&sensorData.ADC_DATA_RF)) / 2;
	wall_config[LF_WALL] = (wall_config[LF_WALL]
			+ read_wall(&sensorData.ADC_DATA_LF)) / 2;
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	turn_config.value = 90;
	turn_config.direction = TURN_R;
	turn(turn_config);
	sirituke();

	for (int i = 0; i < 12; i++) {
		printf("wall[%d]=%ld,", i, wall_config[i]);
	}
	printf("\n");
	tone(tone_hiC, 100);
}

//sensordebug
void print_sensordata(void) {
	static uint8_t flag = 0;
	tone(tone_C, 100);
	if (flag == 1) {
		osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
		osThreadFlagsSet(SENSOR_PRINT_TAHandle, TASK_STOP);
		flag = 0;
	} else {
		flag = 1;
		osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
		osThreadFlagsSet(SENSOR_PRINT_TAHandle, TASK_START);
	}

	tone(tone_C, 100);
}

extern void SENSOR_PRINT(void *argument) {
	/* USER CODE BEGIN WALL_READ */
	osThreadFlagsWait(TASK_START, osFlagsWaitAny, osWaitForever);
	/* Infinite loop */
	for (;;) {
		if (osThreadFlagsWait(TASK_STOP, osFlagsWaitAny, 5000U) == TASK_STOP) {
			osThreadFlagsWait(TASK_START, osFlagsWaitAny, osWaitForever);
		}
		printf("\nRS=%ld,LS=%ld,RF=%ld,LF=%ld\n\n", sensorData.ADC_DATA_RS,
				sensorData.ADC_DATA_LS, sensorData.ADC_DATA_RF,
				sensorData.ADC_DATA_LF);
	}
	/* USER CODE END WALL_READ */
}

extern void WALL_READ(void *argument) {
	/* USER CODE BEGIN WALL_READ */
	/* Infinite loop */
	for (;;) {

		osDelay(10);
	}
	/* USER CODE END WALL_READ */
}

