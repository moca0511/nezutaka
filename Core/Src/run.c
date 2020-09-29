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
extern osThreadId_t POS_CHECK_TASKHandle;
extern uint32_t MotorStepCount_R;
extern uint32_t MotorStepCount_L;
extern uint32_t MotorHz_R;
extern uint32_t MotorHz_L;
extern SensorData sensorData;
extern uint32_t wall_config[12];
extern osThreadId_t MOTOR_R_TaskHandle;
extern osThreadId_t MOTOR_L_TaskHandle;
extern uint8_t wall_calibration_F;

extern int16_t posX, posY;	//　現在の位置
int32_t posX_buf = 0, posY_buf = 0;	//　現在の位置
extern int8_t head;	//　現在向いている方向(北東南西(0,1,2,3))

uint16_t straight(RUNConfig config) {
	uint16_t move = 0;
	float32_t fspeed = SPEEDtoHz(config.initial_speed), fspeed_L = SPEEDtoHz(
			config.initial_speed), fspeed_R = SPEEDtoHz(config.initial_speed);

	float32_t plpl = SPEEDtoHz(
			(float32_t) config.acceleration / (uint32_t) configTICK_RATE_HZ);
	int32_t stopcount = config.value / STEP_LENGTH
			- (MotorStepCount_R + MotorStepCount_L) / 2;
	int32_t gensoku = -1;
	int32_t deviation_prevR = 0, deviation_prevL = 0;
	uint8_t stop = 0; //　移動距離補正フラグ
	int32_t deviation_sumR = 0, deviation_sumL = 0;
	int32_t tick_prev = osKernelGetTickCount();
	float32_t MAX_Hz = SPEEDtoHz(config.max_speed), INITIAL_Hz = SPEEDtoHz(
			config.initial_speed), FINISH_Hz = SPEEDtoHz(config.finish_speed);

	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_START);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_START);
	if (config.max_speed > SPEED_MAX) {
		config.max_speed = SPEED_MAX;
		MAX_Hz = SPEEDtoHz(SPEED_MAX);
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

		if (fspeed >= MAX_Hz) {
			fspeed = MAX_Hz;
			if (gensoku == -1 && config.initial_speed == config.finish_speed) {
				gensoku = stopcount
						- ((MotorStepCount_R + MotorStepCount_L) / 2) * 1.02;
			}
		}
		if (fspeed < FINISH_Hz && plpl < 0) {
			fspeed = FINISH_Hz;
		}
		if (fspeed < SPEEDtoHz(150)
				&& SPEEDtoHz(5)
						< (stopcount - (MotorStepCount_R + MotorStepCount_L) / 2)) {
			fspeed = SPEEDtoHz(150);
		}
		if (fspeed < SPEEDtoHz(SPEED_MIN)
				&& SPEEDtoHz(5)
						> (stopcount - (MotorStepCount_R + MotorStepCount_L) / 2)) {
			fspeed = SPEEDtoHz(SPEED_MIN);
		}
		fspeed_L = fspeed_R = fspeed;
		//PID
		if (wall_calibration_F == 1 && fspeed != 0) {

			if (config.direction == MOVE_FORWARD) {
				if (sensorData.ADC_DATA_LS <= wall_config[LS_WALL] * 0.95
						&& sensorData.ADC_DATA_LS
								>= wall_config[LS_WALL] * 1.05) {
					deviation_sumL = 0;
				}
				if (sensorData.ADC_DATA_LS > wall_config[LS_threshold]
						&& sensorData.ADC_DATA_LS > sensorData.ADC_DATA_RS) { // *　壁がある時だけPID操作
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
						&& sensorData.ADC_DATA_LS < sensorData.ADC_DATA_RS) { // *　壁がある時だけPID操作
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

		MotorHz_R = (int32_t) fspeed_R;
		MotorHz_L = (int32_t) fspeed_L;
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
		if (wall_calibration_F == 1
				&& (sensorData.ADC_DATA_LF
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
//1走行距離判定　ループ１へ
	if (config.finish_speed == 0) {
		mortor_stop();
	} else {
		MotorHz_R = MotorHz_L = FINISH_Hz;
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
	return move;

}

void turn(RUNConfig config) {
//1移動用パラメータ設定
	uint32_t move = TREAD_CIRCUIT / 360 * config.value; //mm
	uint32_t stopcount = move / STEP_LENGTH; //step
	float32_t plpl = SPEEDtoHz(
			(float32_t) config.acceleration / (uint32_t) configTICK_RATE_HZ);
	int32_t gensoku = -1;
	float32_t speed = SPEEDtoHz(config.initial_speed);
	int32_t tick_prev = osKernelGetTickCount();
	float32_t MAX_Hz = SPEEDtoHz(config.max_speed), INITIAL_Hz = SPEEDtoHz(
			config.initial_speed), FINISH_Hz = SPEEDtoHz(config.finish_speed);

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
	MotorHz_R = MotorHz_L = INITIAL_Hz;

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
		if (speed >= MAX_Hz) {
			speed = MAX_Hz;
			if (gensoku == -1) {
				gensoku = stopcount - (MotorStepCount_R + MotorStepCount_L) / 2;
			}
		}
		if (speed < FINISH_Hz && plpl < 0) {
			speed = FINISH_Hz;
		}
		if (speed < SPEEDtoHz(SPEED_MIN)) {
			speed = SPEEDtoHz(SPEED_MIN);
		}
		//printf("R:%ld,L:%ld,gensoku:%ld,plpl:%ld,step:%ld\n",speed_R,speed_L,gensoku,plpl,(MotorStepCount_R + MotorStepCount_L) / 2);
		//1各モータスピードに代入
		MotorHz_L = speed;
		MotorHz_R = speed;
		osDelayUntil(osKernelGetTickCount() + 5);

	} while (stopcount > (MotorStepCount_R + MotorStepCount_L) / 2);

	if (config.finish_speed == 0) {
		mortor_stop();
	} else {
		MotorHz_R = MotorHz_L = FINISH_Hz;
	}

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_STOP);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_STOP);
}

void slalom(RUNConfig config) {
	//1移動用パラメータ設定

	float32_t plpl = SPEEDtoHz(
			(float32_t) config.acceleration / (uint32_t) configTICK_RATE_HZ);
	float32_t fspeedR = SPEEDtoHz(config.initial_speed), fspeedL = SPEEDtoHz(
			config.initial_speed);
	float32_t deg_speed = 0, deg = 0;
//	int32_t tick_prev = osKernelGetTickCount(),tick = osKernelGetTickCount();
	float32_t MAX_Hz = SPEEDtoHz(config.max_speed), INITIAL_Hz = SPEEDtoHz(
			config.initial_speed), FINISH_Hz = SPEEDtoHz(config.finish_speed);

	//printf("move:%ld,stopcount:%ld,speed:%ld,direction:%d\n",move,stopcount,speed,direction);
	//1回転方向設定
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_START);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_START);
	//on
	HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_RESET);

	mortor_direction(MR, MOVE_FORWARD);
	mortor_direction(ML, MOVE_FORWARD);

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
		if ((int32_t) (deg * 0.0053 * 180 / PI) >= config.value * 0.5
				&& plpl >= 0) {
			plpl *= -1;
		}

		deg_speed += plpl * 5;
		if (config.direction == TURN_R) {
			fspeedL = INITIAL_Hz + deg_speed * TREAD_WIDTH / 2;
			fspeedR = INITIAL_Hz - deg_speed * TREAD_WIDTH / 2;
//				if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//					printf("R=%ld,L=%ld\n",speedR,speedL);
//					osMutexRelease(UART_MutexHandle);
//				}

			if (fspeedL <= FINISH_Hz || fspeedR >= FINISH_Hz
					|| fspeedL >= MAX_Hz || fspeedR < SPEEDtoHz(SPEED_MIN)) {
				deg_speed -= plpl * 5;
				fspeedL = INITIAL_Hz + deg_speed * TREAD_WIDTH / 2;
				fspeedR = INITIAL_Hz - deg_speed * TREAD_WIDTH / 2;
//					if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//						printf("re\n");
//						osMutexRelease(UART_MutexHandle);
//					}
			}
		} else {
			fspeedL = INITIAL_Hz - deg_speed * TREAD_WIDTH / 2;
			fspeedR = INITIAL_Hz + deg_speed * TREAD_WIDTH / 2;
			if (fspeedR <= FINISH_Hz || fspeedL >= FINISH_Hz
					|| fspeedR >= MAX_Hz || fspeedL < SPEEDtoHz(SPEED_MIN)) {
				deg_speed -= plpl * 5;
				fspeedL = INITIAL_Hz - deg_speed * TREAD_WIDTH / 2;
				fspeedR = INITIAL_Hz + deg_speed * TREAD_WIDTH / 2;
			}
		}

		//printf("R:%ld,L:%ld,gensoku:%ld,plpl:%ld,step:%ld\n",speed_R,speed_L,gensoku,plpl,(MotorStepCount_R + MotorStepCount_L) / 2);
		//1各モータスピードに代入
		MotorHz_L = fspeedL;
		MotorHz_R = fspeedR;
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

	} while ((int32_t) (deg * 0.0053 * 180 / PI) < config.value);

	if (config.finish_speed == 0) {
		mortor_stop();
	} else {
		MotorHz_R = MotorHz_L = FINISH_Hz;
	}

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_STOP);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_STOP);
}

void sirituke(void) {
	RUNConfig RUN_Config = { MOVE_BACK, 0, 0, 150, 1000, BLOCK_LENGTH / 2 };
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("sirituke\n");
		osMutexRelease(UART_MutexHandle);
	}
	straight(RUN_Config);
	/*	RUN_Config.value = (BLOCK_LENGTH - NEZUTAKA_LENGTH) * 0.5;
	 RUN_Config.direction = MOVE_FORWARD;
	 straight(RUN_Config);*/
}
void ajast(void) {
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 0, 300, 300, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.6 };
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

void run_block(RUNConfig config) {
//osThreadFlagsSet(POS_CHECK_TASKHandle, TASK_START);
	straight(config);
//osThreadFlagsSet(POS_CHECK_TASKHandle, TASK_STOP);
}

void turn_u(void) {
	RUNConfig turn_config = { TURN_R, 0, 0, 800, 1000, 180 };
	turn(turn_config);
	//sirituke();
	chenge_head(turn_config.direction, turn_config.value, &head);
}

/*extern void POS_CHECK(void *argument) {
 uint32_t move = 0, move_buf = 0, move_prev = 0;
 uint8_t wall_set_flag = 1;
 osThreadFlagsWait(TASK_START, osFlagsWaitAny, osWaitForever);
 for (;;) {
 if (osThreadFlagsWait(TASK_STOP, osFlagsWaitAny, 1U) == TASK_STOP) {
 switch (head) {
 case 0:
 if (posY * 100 < posY_buf && posY_buf % 100 >= 50) {
 posY += 1;
 }
 break;
 case 1:
 if (posX * 100 < posX_buf && posX_buf % 100 >= 50) {
 posX += 1;
 }
 break;
 case 2:
 if (posY * 100 > posY_buf && posY_buf % 100 <= 50) {
 posY -= 1;
 }
 break;
 case 3:
 if (posX * 100 > posX_buf && posX_buf % 100 <= 50) {
 posY -= 1;
 }
 break;
 }
 posX_buf /= 100;
 posX_buf *= 100;
 posY_buf /= 100;
 posY_buf *= 100;

 osThreadFlagsWait(TASK_START, osFlagsWaitAny, osWaitForever);
 posX_buf = posX * 100;
 posY_buf = posY * 100;
 move = move_buf = move_prev = 0;
 wall_set_flag = 1;
 }

 move_buf = (uint32_t) ((((MotorStepCount_R + MotorStepCount_L) / 2)
 * 100) / (BLOCK_LENGTH / STEP_LENGTH));
 if (move_buf != 0) {
 move = move_buf - move_prev;
 switch (head) {
 case 0:
 posY_buf += move;
 break;
 case 1:
 posX_buf += move;
 break;
 case 2:
 posY_buf -= move;
 break;
 case 3:
 posX_buf -= move;
 break;
 }

 if (wall_set_flag && move_buf % 100 >= 50) {
 wall_set();
 wall_set_flag = 0;
 }
 if (posY_buf / 100 != posY || posX_buf / 100 != posX) {
 wall_set_flag = 1;
 }
 posY = posY_buf / 100;
 posX = posX_buf / 100;
 move_prev = move_buf;
 }
 osThreadYield();
 Delay_ms(5);
 }
 USER CODE END POS_CHECK
 }*/
