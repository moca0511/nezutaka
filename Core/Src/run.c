/*
 * run.c
 *
 *  Created on: Jul 28, 2020
 *      Author: junmo
 */
#include"run.h"
#include "cmsis_os.h"
#include "sensor.h"
#include "maze.h"

extern osMutexId_t UART_MutexHandle;
extern uint32_t MotorStepCount_R;
extern uint32_t MotorStepCount_L;

extern osSemaphoreId_t SchengeRSemHandle;
extern osSemaphoreId_t SchengeLSemHandle;

extern uint32_t MotorSPEED_R;
extern uint32_t MotorSPEED_L;
extern SensorData sensorData;
extern uint32_t wall_config[12];
extern osThreadId_t MOTOR_R_TaskHandle;
extern osThreadId_t MOTOR_L_TaskHandle;
extern uint8_t wall_calibration_F;

extern int16_t posX, posY;	//　現在の位置
extern int8_t head;	//　現在向いている方向(北東南西(0,1,2,3))

uint16_t straight(RUNConfig config) {
	uint16_t move = 0;
	float32_t fspeed = config.initial_speed, fspeed_L = config.initial_speed,
			fspeed_R = config.initial_speed;

	float32_t plpl = (float32_t) config.acceleration
			/ (float32_t) configTICK_RATE_HZ;
	int32_t stopcount = config.value / STEP_LENGTH
			- (MotorStepCount_R + MotorStepCount_L) / 2;
	int32_t gensoku = -1;
	int32_t deviation_prevR = 0, deviation_prevL = 0;
	uint8_t stop = 0; //　移動距離補正フラグ
	int32_t deviation_sumR = 0, deviation_sumL = 0;
//	float32_t MAX_Hz = SPEEDtoHz(config.max_speed), FINISH_Hz = SPEEDtoHz(
//			config.finish_speed);
//	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("straight\n");
//		osMutexRelease(UART_MutexHandle);
//	}
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_START);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_START);
	if (config.max_speed > SPEED_MAX) {
		config.max_speed = SPEED_MAX;
	}

	//1進行方向設定
	mortor_direction(MR, config.direction);
	mortor_direction(ML, config.direction);
	//on
	HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_RESET);
	//1ループ１
	do {
		//1スピード変更処理
		if (((((MotorStepCount_R + MotorStepCount_L) / 2 >= gensoku)
				&& gensoku != -1)
				|| (((MotorStepCount_R + MotorStepCount_L) / 2
						>= (stopcount * 0.48)) && gensoku == -1)
				|| (config.max_speed == config.initial_speed
						&& config.finish_speed < config.initial_speed))
				&& plpl >= 0) {
			plpl *= -1;
		}

		fspeed += plpl * 5;
//		tick_prev = osKernelGetTickCount();

		if (fspeed >= MAX_SPEED) {
			fspeed = MAX_SPEED;
			if (gensoku == -1 && config.initial_speed == config.finish_speed) {
				gensoku = stopcount
						- ((MotorStepCount_R + MotorStepCount_L) / 2) * 1.02;
			}
		}
		if (fspeed < config.finish_speed && plpl < 0) {
			fspeed = config.finish_speed;
		}
		if (fspeed < 150
				&& SPEEDtoHz(5)
						< (stopcount - (MotorStepCount_R + MotorStepCount_L) / 2)) {
			fspeed = 150;
		}
		if (fspeed < SPEED_MIN
				&& SPEEDtoHz(5)
						> (stopcount - (MotorStepCount_R + MotorStepCount_L) / 2)) {
			fspeed = SPEED_MIN;
		}
		fspeed_L = fspeed_R = fspeed;
		//PID
		if (wall_calibration_F == 1 && fspeed != 0) {

			if (config.direction == MOVE_FORWARD) {

				if (sensorData.ADC_DATA_LS > wall_config[LS_threshold]
						&& (sensorData.ADC_DATA_LS > sensorData.ADC_DATA_RS
								|| sensorData.ADC_DATA_RS
										< wall_config[RS_threshold])) { // *　壁がある時だけPID操作
					if (sensorData.ADC_DATA_LS <= wall_config[LS_WALL] * 0.95
							&& sensorData.ADC_DATA_LS
									>= wall_config[LS_WALL] * 1.05) {
						deviation_sumL = 0;
					}
					fspeed_R = PID(fspeed_R,
							wall_config[LS_WALL] - wall_config[LS_WALL] % 10,
							sensorData.ADC_DATA_LS
									- sensorData.ADC_DATA_LS % 10,
							&deviation_prevL, &deviation_sumL);
				} else {
					deviation_prevL = 0;
					deviation_sumL = 0;

				}
				if (sensorData.ADC_DATA_RS > wall_config[RS_threshold]
						&& (sensorData.ADC_DATA_LS < sensorData.ADC_DATA_RS
								|| sensorData.ADC_DATA_LS
										< wall_config[LS_threshold])) { // *　壁がある時だけPID操作
					if (sensorData.ADC_DATA_RS <= wall_config[RS_WALL] * 0.95
							&& sensorData.ADC_DATA_RS
									>= wall_config[RS_WALL] * 1.05) {
						deviation_sumR = 0;
					}
					fspeed_L = PID(fspeed_L,
							wall_config[RS_WALL] - wall_config[RS_WALL] % 10,
							sensorData.ADC_DATA_RS
									- sensorData.ADC_DATA_RS % 10,
							&deviation_prevR, &deviation_sumR);
				} else {
					deviation_prevR = 0;
					deviation_sumR = 0;
				}
			}
		}

		//1各モータスピードに代入

		MotorSPEED_R = (float32_t) fspeed_R;
		MotorSPEED_L = (float32_t) fspeed_L;
		osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
		osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);

//		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//			printf(
//					"speedL=%ld,speedR=%ld,plpl=%d,(((MotorStepCount_R + MotorStepCount_L) / 2 = %d,stop_count=%d\n",
//					(int32_t) fspeed_L, (int32_t) fspeed_R, (int32_t) plpl,
//					(MotorStepCount_R + MotorStepCount_L) / 2, stopcount);
//			osMutexRelease(UART_MutexHandle);
//		}

		//　前壁判定
		if (wall_calibration_F == 1
				&& (sensorData.ADC_DATA_LF >= wall_config[LF_WALL]
						&& sensorData.ADC_DATA_RF >= wall_config[RF_WALL])) {

			config.finish_speed = 0;
			break;
		}
		//　前壁判定
		if (wall_calibration_F == 1&& (sensorData.ADC_DATA_LF
				>= (wall_config[LF_threshold]
						+ wall_config[LF_threshold]) * 0.7
				&& sensorData.ADC_DATA_RF
				>= (wall_config[RF_threshold]
						+ wall_config[RF_threshold]) * 0.7)
		&& stop == 0 && config.direction == MOVE_FORWARD) {
			if (plpl > 0) {
				plpl *= -1;
			}
			stop = 1;
		}

		osDelayUntil(osKernelGetTickCount() + 5);
	} while (stopcount > (MotorStepCount_R + MotorStepCount_L) / 2
			|| (stop == 1
					&& (sensorData.ADC_DATA_LF < wall_config[LF_WALL]
							&& sensorData.ADC_DATA_RF < wall_config[RF_WALL])));
//	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("loopfin\n");
//		osMutexRelease(UART_MutexHandle);
//	}
//1走行距離判定　ループ１へ
	if (config.finish_speed == 0) {
//		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//			printf("st\n");
//			osMutexRelease(UART_MutexHandle);
//		}
		motor_stop();
	} else {
//		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//			printf("fin\n");
//			osMutexRelease(UART_MutexHandle);
//		}
		MotorSPEED_R = MotorSPEED_L = config.finish_speed;
		osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
		osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);
	}
	move = (((MotorStepCount_R + MotorStepCount_L) / 2) * STEP_LENGTH)
			/ config.value;
	if (((uint16_t) (((MotorStepCount_R + MotorStepCount_L) / 2) * STEP_LENGTH)
			% (uint16_t) config.value) > config.value * 0.5) {
		move++;
	}
	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_STOP);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_STOP);
//	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("Sfin\n");
//		osMutexRelease(UART_MutexHandle);
//	}
	return move;
}

void turn(RUNConfig config) {
//1移動用パラメータ設定
	uint32_t move = TREAD_CIRCUIT / 360 * config.value; //mm
	uint32_t stopcount = move / STEP_LENGTH; //step
	float32_t plpl = (float32_t) config.acceleration
			/ (uint32_t) configTICK_RATE_HZ;
	int32_t gensoku = -1;
	float32_t speed = config.initial_speed;

//printf("move:%ld,stopcount:%ld,speed:%ld,direction:%d\n",move,stopcount,speed,direction);
//1回転方向設定
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_START);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_START);

//on
	HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_RESET);
	if (config.direction == TURN_R) {
		mortor_direction(ML, MOVE_FORWARD);
		mortor_direction(MR, MOVE_BACK);
	} else {
		mortor_direction(MR, MOVE_FORWARD);
		mortor_direction(ML, MOVE_BACK);
	}
	MotorSPEED_R = MotorSPEED_L = config.initial_speed;
	osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
	osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	do {
		//1スピード変更処理
		if (((((MotorStepCount_R + MotorStepCount_L) / 2 >= gensoku)
				&& gensoku != -1)
				|| (((MotorStepCount_R + MotorStepCount_L) / 2
						>= (stopcount / 2)) && gensoku == -1)) && plpl >= 0) {
			plpl *= -1;
		}
		speed += plpl * 5;
		if (speed >= SPEED_MAX) {
			speed = SPEED_MAX;
			if (gensoku == -1) {
				gensoku = stopcount - (MotorStepCount_R + MotorStepCount_L) / 2;
			}
		}
		if (speed < config.finish_speed && plpl < 0) {
			speed = config.finish_speed;
		}
		if (speed < SPEED_MIN) {
			speed = SPEED_MIN;
		}
		//printf("R:%ld,L:%ld,gensoku:%ld,plpl:%ld,step:%ld\n",speed_R,speed_L,gensoku,plpl,(MotorStepCount_R + MotorStepCount_L) / 2);
		//1各モータスピードに代入
		MotorSPEED_L = speed;
		MotorSPEED_R = speed;
		osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
		osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);
		osDelayUntil(osKernelGetTickCount() + 5);

	} while (stopcount > (MotorStepCount_R + MotorStepCount_L) / 2);

	if (config.finish_speed == 0) {
		motor_stop();
	} else {
		MotorSPEED_R = MotorSPEED_L = config.finish_speed;
		osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
		osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);
	}

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_STOP);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_STOP);
}

void slalom(SLALOMConfig config) {
	//1移動用パラメータ設定

	float32_t plpl = (float32_t) config.config.acceleration
			/ (uint32_t) configTICK_RATE_HZ;
	float32_t fspeedR = config.config.initial_speed, fspeedL =
			config.config.initial_speed;
	float32_t deg_speed = 0, deg = 0;
//	int32_t tick_prev = osKernelGetTickCount(),tick = osKernelGetTickCount();
//	float32_t MAX_Hz = config.config.max_speed, INITIAL_Hz =
//			config.config.initial_speed, FINISH_Hz =
//			config.config.finish_speed;

	//printf("move:%ld,stopcount:%ld,speed:%ld,direction:%d\n",move,stopcount,speed,direction);
	//1回転方向設定
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_START);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_START);
	//on
	HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_RESET);

	mortor_direction(MR, MOVE_FORWARD);
	mortor_direction(ML, MOVE_FORWARD);
	while ((MotorStepCount_R + MotorStepCount_L) / 2
			< SPEEDtoHz(config.before_ofset))
		;

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	do {
		//plpl
//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("plpl\n");
//				osMutexRelease(UART_MutexHandle);
//			}
//
//		tick_prev = tick;
//		tick = osKernelGetTickCount();
		if (deg_speed > 0) {
			deg += HztoSPEED(deg_speed);
		} else {
			deg -= HztoSPEED(deg_speed);
		}

		//1スピード変更処理
		if ((int32_t) (deg * 0.013 * 180 / PI) >= config.config.value * 0.5
				&& plpl >= 0) {
			plpl *= -1;
		}

		deg_speed += plpl * 5.0;
		if (config.config.direction == TURN_R) {
			fspeedL = config.config.initial_speed
					+ deg_speed * TREAD_WIDTH / 2.0;
			fspeedR = config.config.initial_speed
					- deg_speed * TREAD_WIDTH / 2.0;
//				if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//					printf("R=%ld,L=%ld\n",speedR,speedL);
//					osMutexRelease(UART_MutexHandle);
//				}

			if (fspeedL
					<= config.config.finish_speed|| fspeedR >= config.config.finish_speed
					|| fspeedL >= config.config.max_speed || fspeedR < SPEED_MIN) {
				deg_speed -= plpl * 5.0;
				fspeedL = config.config.initial_speed
						+ deg_speed * TREAD_WIDTH / 2.0;
				fspeedR = config.config.initial_speed
						- deg_speed * TREAD_WIDTH / 2.0;
//					if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//						printf("re\n");
//						osMutexRelease(UART_MutexHandle);
//					}
			}
		} else {
			fspeedL = config.config.initial_speed
					- deg_speed * TREAD_WIDTH / 2.0;
			fspeedR = config.config.initial_speed
					+ deg_speed * TREAD_WIDTH / 2.0;
			if (fspeedR
					<= config.config.finish_speed|| fspeedL >= config.config.finish_speed
					|| fspeedR >= config.config.max_speed || fspeedL < SPEED_MIN) {
				deg_speed -= plpl * 5.0;
				fspeedL = config.config.initial_speed
						- deg_speed * TREAD_WIDTH / 2.0;
				fspeedR = config.config.initial_speed
						+ deg_speed * TREAD_WIDTH / 2.0;
			}
		}

		//printf("R:%ld,L:%ld,gensoku:%ld,plpl:%ld,step:%ld\n",speed_R,speed_L,gensoku,plpl,(MotorStepCount_R + MotorStepCount_L) / 2);
		//1各モータスピードに代入
		MotorSPEED_L = fspeedL;
		MotorSPEED_R = fspeedR;
		osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
		osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);
		//角度変更処理

		//tick_prev = osKernelGetTickCount();
		osDelayUntil(osKernelGetTickCount() + 5);
//		osThreadYield();

//		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//			printf(
//					"deg=%ld,deg_do=%ld,speedL=%ld,speedR=%ld,plpl=%d,deg_speed=%ld\n",
//					(int32_t) deg,
//					(int32_t) (deg
//							* 0.005
//							* 180 / PI), (int32_t)fspeedL, (int32_t)fspeedR, (int32_t) plpl,
//					(int32_t) deg_speed);
//			osMutexRelease(UART_MutexHandle);
//		}

	} while ((int32_t) (deg * 0.013 * 180 / PI) < config.config.value);

	if (config.config.finish_speed == 0) {
		motor_stop();
	} else {
		MotorSPEED_R = MotorSPEED_L = config.config.finish_speed;
		osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
		osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);
	}

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	while ((MotorStepCount_R + MotorStepCount_L) / 2
			< SPEEDtoHz(config.after_ofset))
		;
	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_STOP);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_STOP);
}

void sirituke(void) {
	RUNConfig RUN_Config = { MOVE_BACK, 0, 0, 100, 500, BLOCK_LENGTH / 2 };
//	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("sirituke\n");
//		osMutexRelease(UART_MutexHandle);
//	}
	motor_stop();
	straight(RUN_Config);
	/*	RUN_Config.value = (BLOCK_LENGTH - NEZUTAKA_LENGTH) * 0.5;
	 RUN_Config.direction = MOVE_FORWARD;
	 straight(RUN_Config);*/
}
void ajast(void) {
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 0, 300, 300, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.6 };
//	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("ajast\n");
//		osMutexRelease(UART_MutexHandle);
//	}
	straight(tyousei_config);
}

float32_t PID(float32_t speed, int32_t target, int32_t sensor,
		int32_t *deviation_prev, int32_t *devaition_sum) {
	int32_t deviation;
	deviation = target - sensor;
	*devaition_sum += deviation;
	/*	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
	 printf("devaition=%d\n", deviation);
	 osMutexRelease(UART_MutexHandle);
	 }*/

	*deviation_prev = deviation - (*deviation_prev);

	speed += (kp * deviation + kd * (*deviation_prev) + ki * (*devaition_sum));
	if (speed < SPEEDtoHz(SPEED_MIN)) {
		speed = 0;
	}
	if (speed > SPEEDtoHz(SPEED_MAX)) {
		speed = SPEEDtoHz(SPEED_MAX);
	}
	return speed;
}

void chenge_pos(int16_t block) {
	switch (head) {
	case 0:
		posY += block;
		break;
	case 1:
		posX += block;
		break;
	case 2:
		posY -= block;
		break;
	case 3:
		posX -= block;
		break;
	}
}

void chenge_head(uint16_t direction, uint32_t value, int8_t *head_buf) {
	if (direction == TURN_R) {
		*head_buf += value / 90;
	} else {
		*head_buf -= value / 90;
	}
	if (*head_buf > 3) {
		*head_buf -= 4;
	}
	if (*head_buf < 0) {
		*head_buf += 4;
	}
}

void turn_u(void) {
	RUNConfig turn_config = { TURN_R, 0, 0, 800, 1000, 180 };
//	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("U\n");
//		osMutexRelease(UART_MutexHandle);
//	}
	turn(turn_config);
	//sirituke();
	chenge_head(turn_config.direction, turn_config.value, &head);
}
