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
extern MAP map[MAP_X_MAX][MAP_Y_MAX];

extern int16_t posX, posY;	//　現在の位置
extern int8_t head;	//　現在向いている方向(北東南西(0,1,2,3))

/*
 * 説明：configのパラメータ通りに台形加速しながら直進
 * 引数：config 走行パラメータ
 * 戻り値:config.value＊ｘ進んだ　のx
 */
uint16_t straight(RUNConfig config, uint8_t pid_F, uint8_t wall_break_F,
		uint8_t front_Adjustment_F) {
	uint16_t move = 0;
	float32_t fspeed = config.initial_speed, fspeed_L = config.initial_speed,
			fspeed_R = config.initial_speed;

	float32_t plpl = (float32_t) config.acceleration
			/ (float32_t) configTICK_RATE_HZ;
	int32_t stopcount = config.value / STEP_LENGTH
			- (MotorStepCount_R + MotorStepCount_L) / 2;
	int32_t gensoku = -1;
	int32_t deviation_prevR = 0, deviation_prevL = 0;
	uint8_t stop_R = 0, stop_L = 0, stop_f = 0; //　移動距離補正フラグ
	int32_t deviation_sumR = 0, deviation_sumL = 0;

	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_START);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_START);
	if (config.max_speed > SPEED_MAX) {
		config.max_speed = SPEED_MAX;
	}

	//1進行方向設定
	mortor_direction(MR, config.direction);
	mortor_direction(ML, config.direction);
	if (sensorData.ADC_DATA_LS < wall_config[LS_threshold]) {
		stop_L = 1;

	}
	if (sensorData.ADC_DATA_RS < wall_config[RS_threshold]) {
		stop_R = 1;
	}
	//on
	HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_RESET);
	//1ループ１
	do {
		//1スピード変更処理
		if (((((MotorStepCount_R + MotorStepCount_L) / 2 >= gensoku)
				&& gensoku != -1)
				|| (((MotorStepCount_R + MotorStepCount_L) / 2
						>= (stopcount * 0.45)) && gensoku == -1)
				|| (config.max_speed == config.initial_speed
						&& config.finish_speed < config.initial_speed))
				&& plpl >= 0) {
			plpl *= -1;
		}

		fspeed += plpl * 5;

		if (fspeed >= MAX_SPEED) {
			fspeed = MAX_SPEED;
			if (gensoku == -1 && config.initial_speed == config.finish_speed) {
				gensoku = stopcount
						- (((MotorStepCount_R + MotorStepCount_L) / 2) * 1.2);
			}
		}
		//speedが終了速度より下がらないように
		if (fspeed < config.finish_speed && plpl < 0) {
			fspeed = config.finish_speed;
		}
		//遅すぎたらPID制御が上手くいかないため
		if (fspeed < 150) {
			fspeed = 150;
		}

		fspeed_L = fspeed_R = fspeed;

		//PID
		if (pid_F == 1 && fspeed != 0) {

			if (config.direction == MOVE_FORWARD) {
				if (sensorData.ADC_DATA_LS > wall_config[LS_threshold]
						&& (sensorData.ADC_DATA_LS > sensorData.ADC_DATA_RS
								|| sensorData.ADC_DATA_RS
										< wall_config[RS_threshold])) { //　壁がある時だけPID操作
					if (sensorData.ADC_DATA_LS <= wall_config[LS_WALL] * 0.95
							&& sensorData.ADC_DATA_LS
									>= wall_config[LS_WALL] * 1.05) { //ほぼ壁なら偏差計リセット
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

		//壁切れ補正
		if (wall_break_F == 1 && config.direction == MOVE_FORWARD
				&& HztoSPEED(
						stopcount - ((MotorStepCount_R + MotorStepCount_L) / 2))
						< 50) {
			if (sensorData.ADC_DATA_LS < wall_config[LS_threshold]
					&& stop_L == 0) {
				stopcount = ((MotorStepCount_R + MotorStepCount_L) / 2)
						+ SPEEDtoHz(75);
				stop_R = stop_L = stop_f = 1;
				if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
					printf("LS=%ld\n", sensorData.ADC_DATA_LS);
					osMutexRelease(UART_MutexHandle);
				}
			} else if (sensorData.ADC_DATA_RS < wall_config[RS_threshold]
					&& stop_R == 0) {
				stopcount = ((MotorStepCount_R + MotorStepCount_L) / 2)
						+ SPEEDtoHz(75);
				stop_R = stop_L = stop_f = 1;
				if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
					printf("RS=%ld\n", sensorData.ADC_DATA_RS);
					osMutexRelease(UART_MutexHandle);
				}
			}
		}

		//　前壁判定
		if (front_Adjustment_F == 1
				&& (sensorData.ADC_DATA_LF >= wall_config[LF_WALL]
						&& sensorData.ADC_DATA_RF >= wall_config[RF_WALL])
				&& stop_f == 0) {

			config.finish_speed = 0;
			break;
		}

		osDelayUntil(osKernelGetTickCount() + 5);
	} while (stopcount > (MotorStepCount_R + MotorStepCount_L) / 2);

	if (config.finish_speed == 0) {
		motor_stop();
	} else {
		MotorSPEED_R = MotorSPEED_L = config.finish_speed;
		osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
		osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);
	}

	move = (((MotorStepCount_R + MotorStepCount_L) / 2) * STEP_LENGTH)
			/ config.value;
	if (((uint16_t) (((MotorStepCount_R + MotorStepCount_L) / 2) * STEP_LENGTH)
			- (uint16_t) config.value * move) > config.value * 0.5) {
		move++;
	}

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_STOP);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_STOP);

	return move;
}

/*
 * 説明：configの通りに超新地旋回
 * 引数:config 走行パラメータ
 * 戻り値：　無し
 */
void turn(RUNConfig config) {
//1移動用パラメータ設定
	uint32_t move = TREAD_CIRCUIT / 360 * config.value; //mm
	uint32_t stopcount = move / STEP_LENGTH; //step
	float32_t plpl = (float32_t) config.acceleration
			/ (uint32_t) configTICK_RATE_HZ;
	int32_t gensoku = -1;
	float32_t speed = config.initial_speed;

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
	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	MotorSPEED_R = MotorSPEED_L = config.initial_speed;
	osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
	osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);

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

//1各モータスピードに代入
		MotorSPEED_L = speed;
		MotorSPEED_R = speed;
		osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
		osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);

		if (stopcount <= (MotorStepCount_R + MotorStepCount_L) / 2) {
			osThreadYield();
			break;
		}
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

/*
 * 説明：configの通りにslalom走行
 * 引数：config 走行パラメータ
 * 戻り値：無し
 */
void slalom(SLALOMConfig config) {
//1移動用パラメータ設定

	float32_t plpl = (float32_t) config.config.acceleration
			/ (uint32_t) configTICK_RATE_HZ;
	float32_t fspeedR = config.config.initial_speed, fspeedL =
			config.config.initial_speed;
	float32_t deg_speed = 0, deg = 0;

//1回転方向設定
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_START);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_START);
//on
	HAL_GPIO_WritePin(SLEEP_R_GPIO_Port, SLEEP_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SLEEP_L_GPIO_Port, SLEEP_L_Pin, GPIO_PIN_RESET);

	mortor_direction(MR, MOVE_FORWARD);
	mortor_direction(ML, MOVE_FORWARD);
//	osDelayUntil(osKernelGetTickCount() + 5);

//位置調整
	while ((MotorStepCount_R + MotorStepCount_L) / 2
			< SPEEDtoHz(config.after_ofset)) {
		osDelayUntil(osKernelGetTickCount() + 5);
		if (sensorData.ADC_DATA_LF >= 850 && sensorData.ADC_DATA_RF >= 850) {
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("1Swall RF:%ld LF:%ld\n", sensorData.ADC_DATA_RF,
						sensorData.ADC_DATA_LF);
				osMutexRelease(UART_MutexHandle);
			}
			break;
		}
	}
	if ((sensorData.ADC_DATA_LF < 850 && sensorData.ADC_DATA_RF < 850)
			&& (sensorData.ADC_DATA_LF >= wall_config[LF_threshold] * 0.8
					&& sensorData.ADC_DATA_RF >= wall_config[RF_threshold] * 0.8)) {
		while ((sensorData.ADC_DATA_LF < 850 && sensorData.ADC_DATA_RF < 850)) {
			osDelayUntil(osKernelGetTickCount() + 5);
		}
		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("2Swall RF:%ld LF:%ld\n", sensorData.ADC_DATA_RF,
					sensorData.ADC_DATA_LF);
			osMutexRelease(UART_MutexHandle);
		}
	}

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	do {

		if (deg_speed > 0) {
			deg += HztoSPEED(deg_speed);
		} else {
			deg -= HztoSPEED(deg_speed);
		}

//1スピード変更処理
		if ((int32_t) (deg * 0.0125 * 180 / PI) >= config.config.value * 0.5
				&& plpl >= 0) {
			plpl *= -1;
		}

		deg_speed += plpl * 5.0;
		if (config.config.direction == TURN_R) {
			fspeedL = config.config.initial_speed
					+ deg_speed * TREAD_WIDTH / 2.0;
			fspeedR = config.config.initial_speed - deg_speed * TREAD_WIDTH / 2;

			if (fspeedL
					<= config.config.finish_speed|| fspeedR >= config.config.finish_speed
					|| fspeedL >= config.config.max_speed || fspeedR < SPEED_MIN) {
				deg_speed -= plpl * 5.0;
				fspeedL = config.config.initial_speed
						+ deg_speed * TREAD_WIDTH / 2.0;
				fspeedR = config.config.initial_speed
						- deg_speed * TREAD_WIDTH / 2;
			}
		} else {
			fspeedL = config.config.initial_speed - deg_speed * TREAD_WIDTH / 2;
			fspeedR = config.config.initial_speed
					+ deg_speed * TREAD_WIDTH / 2.0;
			if (fspeedR
					<= config.config.finish_speed|| fspeedL >= config.config.finish_speed
					|| fspeedR >= config.config.max_speed || fspeedL < SPEED_MIN) {
				deg_speed -= plpl * 5.0;
				fspeedL = config.config.initial_speed
						- deg_speed * TREAD_WIDTH / 2;
				fspeedR = config.config.initial_speed
						+ deg_speed * TREAD_WIDTH / 2.0;
			}
		}

//1各モータスピードに代入
		MotorSPEED_L = fspeedL;
		MotorSPEED_R = fspeedR;
		osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
		osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);
//角度変更処理

		if ((int32_t) (deg * 0.0125 * 180 / PI) >= config.config.value) {
			break;
		}
		osDelayUntil(osKernelGetTickCount() + 5);

	} while ((int32_t) (deg * 0.0125 * 180 / PI) < config.config.value);

	if (config.config.finish_speed == 0) {
		motor_stop();
	} else {
		MotorSPEED_R = MotorSPEED_L = config.config.finish_speed;
		osSemaphoreAcquire(SchengeRSemHandle, osWaitForever);
		osSemaphoreAcquire(SchengeLSemHandle, osWaitForever);
	}

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
//	osDelayUntil(osKernelGetTickCount() + 5);
//位置調整
	while ((MotorStepCount_R + MotorStepCount_L) / 2
			< SPEEDtoHz(config.after_ofset)) {
		osDelayUntil(osKernelGetTickCount() + 5);
		if (sensorData.ADC_DATA_LF >= 700 && sensorData.ADC_DATA_RF >= 700) {
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("1Ewall RF:%ld LF:%ld\n", sensorData.ADC_DATA_RF,
						sensorData.ADC_DATA_LF);
				osMutexRelease(UART_MutexHandle);
			}
			break;
		}
	}
	if ((sensorData.ADC_DATA_LF < 700 && sensorData.ADC_DATA_RF < 700)
			&& (sensorData.ADC_DATA_LF >= wall_config[LF_threshold] * 0.8
					&& sensorData.ADC_DATA_RF >= wall_config[RF_threshold] * 0.8)) {
		while ((sensorData.ADC_DATA_LF < 700 && sensorData.ADC_DATA_RF < 700)) {
			osDelayUntil(osKernelGetTickCount() + 5);
		}
		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("2Ewall RF:%ld LF:%ld\n", sensorData.ADC_DATA_RF,
					sensorData.ADC_DATA_LF);
			osMutexRelease(UART_MutexHandle);
		}
	}

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
	osThreadFlagsSet(MOTOR_R_TaskHandle, TASK_STOP);
	osThreadFlagsSet(MOTOR_L_TaskHandle, TASK_STOP);
}
/*
 * 説明：尻付け動作
 * 引数：無し
 * 戻り値：無し
 */
void sirituke(void) {
	RUNConfig RUN_Config = { MOVE_BACK, 0, 0, 100, 500, BLOCK_LENGTH / 2 };
	motor_stop();
	straight(RUN_Config, 1, 0, 0);
}
/*
 * 説明：マスの端から中央へ移動
 * 引数：無し
 * 戻り値：無し
 */
void ajast(void) {
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 0, 300, 300, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };
	straight(tyousei_config, 0, 0, 0);
}
/*
 * 説明：PID制御
 * 引数：speed 制御対象
 * 		target 目標値
 * 		sensor　現在値
 * 		*deviation_prev 前回の偏差
 * 		*deviation_sum これまでの偏差の合計
 * 戻り値：PID制御後の数値
 */
float32_t PID(float32_t speed, int32_t target, int32_t sensor,
		int32_t *deviation_prev, int32_t *devaition_sum) {
	int32_t deviation;
	deviation = target - sensor;
	*devaition_sum += deviation;
	*deviation_prev = deviation - (*deviation_prev);

	speed += (KP * deviation + KD * (*deviation_prev) + KI * (*devaition_sum));
	if (speed < SPEEDtoHz(SPEED_MIN)) {
		speed = 0;
	}
	if (speed > SPEEDtoHz(SPEED_MAX)) {
		speed = SPEEDtoHz(SPEED_MAX);
	}
	return speed;
}
/*
 * 説明：現在位置変更処理
 * 引数：block 移動マス数
 * 戻り値：無し
 */
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

/*
 * 説明：車体の向き変更
 * 引数：direction 旋回方向
 * 		value 旋回角度
 * 		*head_buf 変更したい進行方向を持つ変数のポインタ
 * 戻り値：無し
 */
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

/*
 * 説明：Uターン
 * 引数：無し
 * 戻り値：無し
 */
void turn_u(void) {
	RUNConfig turn_config = { TURN_R, 0, 0, 800, 1000, 180 };
	turn(turn_config);
	chenge_head(turn_config.direction, turn_config.value, &head);
}
