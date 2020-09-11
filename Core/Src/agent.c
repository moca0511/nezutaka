/*
 * agent.c
 *
 *  Created on: 2020/09/11
 *      Author: junmo
 */
#include "cmsis_os.h"
#include"main.h"
#include "sensor.h"
#include "timer.h"
#include "buzzer.h"
#include "motor.h"
#include "UI.h"
#include"nezutaka.h"
#include"run.h"
#include "maze.h"
extern osMutexId_t UART_MutexHandle;
extern uint32_t MOTORSPEED_R;
extern uint32_t MOTORSPEED_L;
extern BuzzerConfig buzzer_config;
extern SensorData sensorData;
extern uint32_t us;
extern uint8_t sensor_debug_f;
extern osThreadId_t Sensor_TaskHandle;
extern uint32_t wall_config[12];
extern MAP map[MAP_SIZE];
extern uint8_t game_mode;	//　探索(0)・最短(1)　選択
extern int16_t posX, posY;	//　現在の位置
extern uint8_t head;	//　現在向いている方向(北東南西(0,1,2,3))

//足立法探索　ゴール
//1.目標地点確認　現在位置よりステップ数が少なく　未到達かつ壁情報読んでなくてゴールじゃない場所で　一番近い場所
//2.そこまで行くための道のり（最短マップ）算出
//3.最短できなければいけるところまで行く。
//4.ゴールなら終了
//5.1に戻る

void adachi(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 400, 400, 400, 0, BLOCK_LENGTH };
	RUNConfig turn_config = { TURN_R, 400, 400, 400, 0, 90 };
	uint8_t temp_head = 0;
	game_mode = 0;
	printf("adachi\n");
	osDelay(500);
	tone(tone_hiC, 10);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	osDelay(10);

	for (;;) {
		//printf("loop\n");
		if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
			printf("posX=%d,posY=%d,head=%d\n\n", posX, posY, head);
			osMutexRelease(UART_MutexHandle);
		}
		if (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			MOTORSPEED_R = MOTORSPEED_L = 0;
			while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
				osDelay(50);
			}
			tone(tone_hiC, 200);
			break;
		}
		if (posX == goalX && posY == goalY) {
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			MOTORSPEED_L = MOTORSPEED_R = 0;
			break;
		}
		//osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
		wall_set();
		make_smap(goalX, goalY, game_mode);
		print_map();
		//osThreadFlagsSet(Sensor_TaskHandle, TASK_START);

		if ((map[posX + posY * MAP_X_MAX].wall & 0x11) == 0x10
				&& map[posX + posY * MAP_X_MAX].step
						> map[posX + posY * MAP_X_MAX - 1].step) {
			temp_head = 3;
		} else if ((map[posX + posY * MAP_X_MAX].wall & 0x22) == 0x20
				&& map[posX + posY * MAP_X_MAX].step
						> map[posX + (posY - 1) * MAP_X_MAX].step) {
			temp_head = 2;
		} else if ((map[posX + posY * MAP_X_MAX].wall & 0x44) == 0x40
				&& map[posX + posY * MAP_X_MAX].step
						> map[posX + posY * MAP_X_MAX + 1].step) {
			temp_head = 1;
		} else if ((map[posX + posY * MAP_X_MAX].wall & 0x88) == 0x80
				&& map[posX + posY * MAP_X_MAX].step
						> map[posX + (posY + 1) * MAP_X_MAX].step) {
			temp_head = 0;
		} else {
			temp_head = 0;
		}

		temp_head += 4;         //マイナス数防止
		temp_head -= head;    //headの考慮
		if (temp_head >= 4) {    //桁上がりの考慮
			temp_head -= 4;
		}
		switch (temp_head) {
		case 0:
			//RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			//printf("straight\n");
			straight(RUN_config);
			chenge_pos(1);
			break;
		case 1:
			//printf("TURN R\n");
			//turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			turn_config.direction = TURN_R;
			slalom(turn_config);
			chenge_head(turn_config);
			break;
		case 2:
			//	printf("U\n");
			turn_u();
			break;
		case 3:
			//	printf("TURNL\n");
			//turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			turn_config.direction = TURN_L;
			slalom(turn_config);
			chenge_head(turn_config);
		}
		tone(tone_hiC, 100);

	}
}

void hidarite(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 400, 400, 400, 0, BLOCK_LENGTH };
	RUNConfig turn_config = { TURN_R, 400, 400, 400, 0, 90 };
	printf("hidarite\n");
	osDelay(500);
	//wall_calibration();
	tone(tone_hiC, 10);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	osDelay(10);
	for (;;) {

		//print_map();
		if (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
			MOTORSPEED_R = MOTORSPEED_L = 0;
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
				osDelay(50);
			}
			tone(tone_hiC, 200);
			break;
		}
		//osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
		wall_set();
//		make_smap(goalX, goalY, game_mode);
		print_map();
		//osThreadFlagsSet(Sensor_TaskHandle, TASK_START);

		if (wall_check(1) == 0) {
			//	printf("TURNL\n");
			//turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			turn_config.direction = TURN_L;
			slalom(turn_config);
			chenge_head(turn_config);
			//RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			straight(RUN_config);
			chenge_pos(1);
		} else if (wall_check(0) == 0) {
			//RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			//	printf("straight\n");
			straight(RUN_config);
			chenge_pos(1);
		} else if (wall_check(2) == 0) {
			//	printf("TURN R\n");
			//			RUN_config.value = (BLOCK_LENGTH - NEZUTAKA_LENGTH) / 3;
			//			straight(RUN_config);
			//turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			turn_config.direction = TURN_R;
			slalom(turn_config);
			chenge_head(turn_config);
			//RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			straight(RUN_config);
			chenge_pos(1);
		} else {
			//	printf("U\n");
			turn_u();
		}

		tone(tone_hiC, 100);
	}
}
