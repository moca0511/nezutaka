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

extern osThreadId_t POS_CHECK_TASKHandle;
extern uint32_t MotorStepCount_R;
extern uint32_t MotorStepCount_L;
extern uint32_t MOTORSPEED_R;
extern uint32_t MOTORSPEED_L;
extern SensorData sensorData;
extern uint32_t wall_config[12];

extern int16_t posX, posY;	//　現在の位置
int32_t posX_buf = 0, posY_buf = 0;	//　現在の位置
extern uint8_t head;	//　現在向いている方向(北東南西(0,1,2,3))

void straight(RUNConfig config) {
	int32_t speed = config.initial_speed;
	int32_t speed_R = speed;
	int32_t speed_L = speed;
	int32_t plpl = config.acceleration, plpl_count = 2, loop_count = 0;
	int32_t stopcount = config.value / STEP_LENGTH;
	int32_t gensoku = -1;
	int32_t deviation_prevR = 0, deviation_prevL = 0;
	int32_t deviation_sumR = 0, deviation_sumL = 0;
	uint8_t stop = 0; //　移動距離補正フラグ
	if (config.max_speed > SPEED_MAX) {
		config.max_speed = SPEED_MAX;
	}
	if (plpl / 100 < 1 && plpl != 0) {
		plpl = 1;
	} else {
		plpl /= 100;
	}
	//1進行方向設定
	mortor_direction(MR, config.direction);
	mortor_direction(ML, config.direction);
	//1ループ１
	do {
		//1スピード変更処理
		if (((((MotorStepCount_R + MotorStepCount_L) / 2 >= gensoku)
				&& gensoku != -1)
				|| (((MotorStepCount_R + MotorStepCount_L) / 2
						>= (stopcount / 2)) && gensoku == -1)) && plpl >= 0) {
			plpl *= -1;
		}
		if (loop_count >= plpl_count) {
			speed += plpl;
			loop_count = 0;
		} else {
			loop_count++;
		}
		if (speed >= config.max_speed) {
			speed = config.max_speed;
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
		speed_R = speed;
		speed_L = speed;
		//PID
		if (wall_config[LS_threshold] != 0) {
			if (config.direction == MOVE_FORWARD) {
				if (sensorData.ADC_DATA_LS > wall_config[LS_threshold]) { // *　壁がある時だけPID操作
					speed_L = PID(speed, wall_config[LS_WALL],
							sensorData.ADC_DATA_LS, &deviation_prevL);
				}
				if (sensorData.ADC_DATA_RS > wall_config[RS_threshold]) {
					speed_R = PID(speed, wall_config[RS_WALL],
							sensorData.ADC_DATA_RS, &deviation_prevR);
				}
			} else {
				if (sensorData.ADC_DATA_LS > wall_config[LS_threshold]) {
					speed_R = PID(speed, wall_config[LS_WALL],
							sensorData.ADC_DATA_LS, &deviation_prevR);
				} else {
					deviation_prevR = 0;
				}
				if (sensorData.ADC_DATA_RS > wall_config[RS_threshold]) {
					speed_L = PID(speed, wall_config[RS_WALL],
							sensorData.ADC_DATA_RS, &deviation_prevL);
				} else {
					deviation_prevL = 0;
				}
			}
		}
		//printf("R:%ld,L:%ld,gensoku:%ld,plpl:%ld,step:%ld\n",speed_R,speed_L,gensoku,plpl,(MotorStepCount_R + MotorStepCount_L) / 2);
		//1各モータスピードに代入
		MOTORSPEED_R = speed_R;
		MOTORSPEED_L = speed_L;

		/*		//　壁切れ補正 最終マス移動時のみ
		 if (stop
		 == 0&& wall_config[LS_threshold] != 0
		 && (sensorData.ADC_DATA_LS < wall_config[LS_threshold]
		 || sensorData.ADC_DATA_RF < wall_config[RF_threshold])
		 && (stopcount - (MotorStepCount_R + MotorStepCount_L) / 2) < BLOCK_LENGTH / STEP_LENGTH) {
		 stopcount = (MotorStepCount_R + MotorStepCount_L)
		 / 2 + BLOCK_LENGTH / 2 / STEP_LENGTH;
		 stop = 1;
		 }*/

		//　前壁判定
		/*if (stop == 0 && wall_config[LF_threshold] != 0
		 && (sensorData.ADC_DATA_LF >= wall_config[LF_WALL]
		 && sensorData.ADC_DATA_RF >= wall_config[RF_WALL])) {
		 printf("maekabe\n");
		 stop = 1;
		 if (speed >= 400) {		//　速度が早ければ間に合わない？？
		 break;
		 } else {
		 plpl_count /= 2;
		 stopcount = (MotorStepCount_R + MotorStepCount_L)
		 / 2 + 5 / STEP_LENGTH;
		 }
		 }*/

		osDelay(5);
	} while (stopcount > (MotorStepCount_R + MotorStepCount_L) / 2);
	//1走行距離判定　ループ１へ
	MOTORSPEED_R = MOTORSPEED_L = config.finish_speed;

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
}

void turn(RUNConfig config) {
	//1移動用パラメータ設定
	uint32_t move = TREAD_CIRCUIT / 360 * config.value; //mm
	uint32_t stopcount = move / STEP_LENGTH; //step
	int32_t plpl = config.acceleration, plpl_count = 2, loop_count = 0;
	int32_t gensoku = -1;
	int32_t speed = config.initial_speed;
	//printf("move:%ld,stopcount:%ld,speed:%ld,direction:%d\n",move,stopcount,speed,direction);
	//1回転方向設定
	mortor_direction(MR, MOVE_FORWARD);
	mortor_direction(ML, MOVE_FORWARD);
	if (config.direction == TURN_R) {
		mortor_direction(ML, MOVE_FORWARD);
		mortor_direction(MR, MOVE_BACK);
	} else {
		mortor_direction(MR, MOVE_FORWARD);
		mortor_direction(ML, MOVE_BACK);
	}

	if (plpl / 100 < 1 && plpl != 0) {
		plpl = 1;
	} else {
		plpl /= 100;
	}
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
		if (loop_count >= plpl_count) {
			speed += plpl;
			loop_count = 0;
		} else {
			loop_count++;
		}
		if (speed >= config.max_speed) {
			speed = config.max_speed;
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
		MOTORSPEED_L = speed;
		MOTORSPEED_R = speed;
		osDelay(5);
	} while (stopcount > (MotorStepCount_R + MotorStepCount_L) / 2);

	MOTORSPEED_R = MOTORSPEED_L = config.finish_speed;

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
}
void slalom(RUNConfig config) {
	//1移動用パラメータ設定
	uint32_t move = TREAD_CIRCUIT / 360 * config.value; //mm
	uint32_t stopcount = move / STEP_LENGTH; //step
	int32_t plpl = config.acceleration, plpl_count = 2, loop_count = 0;
	int32_t gensoku = -1;
	int32_t speed = config.initial_speed;
	//printf("move:%ld,stopcount:%ld,speed:%ld,direction:%d\n",move,stopcount,speed,direction);
	//1回転方向設定
	mortor_direction(MR, MOVE_FORWARD);
	mortor_direction(ML, MOVE_FORWARD);
	if (config.direction == TURN_U) {
		mortor_direction(MR, MOVE_FORWARD);
		mortor_direction(ML, MOVE_BACK);
	}

	if (plpl / 100 < 1 && plpl != 0) {
		plpl = 1;
	} else {
		plpl /= 100;
	}
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
		if (loop_count >= plpl_count) {
			speed += plpl;
			loop_count = 0;
		} else {
			loop_count++;
		}
		if (speed >= config.max_speed) {
			speed = config.max_speed;
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
		if (config.direction == TURN_R) {
			MOTORSPEED_L = speed;
			MOTORSPEED_R = 0;
		} else if (config.direction == TURN_L) {
			MOTORSPEED_L = 0;
			MOTORSPEED_R = speed;
		} else {
			MOTORSPEED_L = speed;
			MOTORSPEED_R = speed;
		}
		osDelay(5);
	} while (stopcount > (MotorStepCount_R + MotorStepCount_L) / 2);

	MOTORSPEED_R = MOTORSPEED_L = config.finish_speed;

	MotorStepCount_R = 0;
	MotorStepCount_L = 0;
}

void sirituke(void) {
	RUNConfig RUN_Config = { MOVE_BACK, (MOTORSPEED_L + MOTORSPEED_R) / 2, 0,
			200, 1000, BLOCK_LENGTH / 3 };
	straight(RUN_Config);
}

int32_t PID(int32_t speed, int32_t target, int32_t sensor,
		int32_t *deviation_prev) {
	int32_t deviation;
	deviation = target - sensor;
	*deviation_prev = deviation - (*deviation_prev);
	speed -= (kp * deviation + kd * (*deviation_prev));
	if (speed < SPEED_MIN) {
		speed = SPEED_MIN;
	}
	if (speed > SPEED_MAX) {
		speed = SPEED_MAX;
	}
	return speed;
}

void chenge_pos(uint16_t block) {
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

void chenge_head(RUNConfig config) {
	int8_t head_buf = head;
	if (config.direction == TURN_R) {
		head_buf += config.value / 90;
	} else {
		head_buf -= config.value / 90;
	}
	if (head_buf > 3) {
		head_buf -= 4;
	}
	if (head_buf < 0) {
		head_buf += 4;
	}
	head = head_buf;
}

void run_block(RUNConfig config) {
	osThreadFlagsSet(POS_CHECK_TASKHandle, TASK_START);
	straight(config);
	osThreadFlagsSet(POS_CHECK_TASKHandle, TASK_STOP);
}

void turn_u(void) {
	RUNConfig turn_config = { TURN_L, (MOTORSPEED_L + MOTORSPEED_R) / 2, 0, 200,
			500, 180 };
	turn(turn_config);
	sirituke();
	chenge_head(turn_config);
}

extern void POS_CHECK(void *argument) {
	uint32_t move = 0, move_buf = 0, move_prev = 0;
	uint8_t wall_set_flag = 1;
	osThreadFlagsWait(TASK_START, osFlagsWaitAny, osWaitForever);
	for (;;) {
		if (osThreadFlagsWait(TASK_STOP, osFlagsWaitAny, 5U) == TASK_STOP) {
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
			if(posY_buf/100!=posY||posX_buf/100!=posX){
				wall_set_flag=1;
			}
			posY = posY_buf / 100;
			posX = posX_buf / 100;
			move_prev = move_buf;
		}
	}
	/* USER CODE END POS_CHECK */
}
