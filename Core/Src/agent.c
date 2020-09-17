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

void adachi(RUNConfig RUN_config, uint16_t gx, uint16_t gy) {
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 0, 500, 2000, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };
	RUNConfig turn_config = { TURN_R, 0, 0, 800, 1000, 90 };
	RUNConfig U_config = { TURN_R, 0, 0, 200, 500, 180 };
	uint8_t temp_head = 0;
	uint8_t value_buf = 0;
	game_mode = 0;
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("adachi\n");
		osMutexRelease(UART_MutexHandle);
	}

	Delay_ms(500);
	tone(tone_hiC, 10);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	/*Delay_ms(10);
	wall_config[RS_WALL] = read_wall(&sensorData.ADC_DATA_RS);
	wall_config[LS_WALL] = read_wall(&sensorData.ADC_DATA_LS);*/

	for (;;) {

		if (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			mortor_stop();
			while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
				Delay_ms(50);
			}
			tone(tone_hiC, 200);
			break;
		}

		if (posX == gx && posY == gy) {
			tyousei_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
//			straight(tyousei_config);
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			mortor_stop();
			music();
			break;
		}

		make_smap(gx, gy, 0);

		if ((map[posX + posY * MAP_X_MAX].wall & 0x88) == 0x80
				&& map[posX + posY * MAP_X_MAX].step
						> map[posX + (posY + 1) * MAP_X_MAX].step) {
			temp_head = 0;
		} else if ((map[posX + posY * MAP_X_MAX].wall & 0x11) == 0x10
				&& map[posX + posY * MAP_X_MAX].step
						> map[posX + posY * MAP_X_MAX - 1].step) {
			temp_head = 3;
		} else if ((map[posX + posY * MAP_X_MAX].wall & 0x44) == 0x40
				&& map[posX + posY * MAP_X_MAX].step
						> map[posX + posY * MAP_X_MAX + 1].step) {
			temp_head = 1;
		} else if ((map[posX + posY * MAP_X_MAX].wall & 0x22) == 0x20
				&& map[posX + posY * MAP_X_MAX].step
						> map[posX + (posY - 1) * MAP_X_MAX].step) {
			temp_head = 2;
		} else {
//			mortor_sleep();
			mortor_stop();
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			tone(tone_C, 1000);
			Delay_ms(1000);
			break;
		}

		temp_head += 4;         //マイナス数防止
		temp_head -= head;    //headの考慮
		if (temp_head >= 4) {    //桁上がりの考慮
			temp_head -= 4;
		}
		//print_map();

		switch (temp_head) {
		case 0:
			RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			value_buf = RUN_config.value;
			RUN_config.value = value_buf * 0.5;
			//printf("straight\n");

			chenge_pos(straight(RUN_config));
			wall_set(0x02);
			tone(tone_C, 50);
			RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			RUN_config.value = value_buf * 0.5;
			straight(RUN_config);
			wall_set(0x01);
			wall_set_around();
			RUN_config.value = BLOCK_LENGTH;
			break;
		case 1:
			//printf("TURN R\n");
//			tyousei_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
//			straight(tyousei_config);
//			turn_config.initial_speed = 0;
			mortor_stop();
			turn_config.direction = TURN_R;
			turn(turn_config);
			chenge_head(turn_config);
			/*if (((((map[posX + posY * MAP_X_MAX].wall & 0x0f)
			 | (map[posX + posY * MAP_X_MAX].wall << 4)) << head) & 0x20)
			 == 0x20) {
			 sirituke();
			 RUN_config.value = BLOCK_LENGTH;
			 } else {
			 RUN_config.value = BLOCK_LENGTH
			 - ((BLOCK_LENGTH - NEZUTAKA_LENGTH) * 0.5);
			 }*/
			U_config.direction = TURN_L;

			break;
		case 2:
			//	printf("U\n");
			mortor_stop();
			turn(U_config);
			chenge_head(U_config);
			/*if (((((map[posX + posY * MAP_X_MAX].wall & 0x0f)
					| (map[posX + posY * MAP_X_MAX].wall << 4)) << head) & 0x20)
					== 0x20) {
				sirituke();
//				RUN_config.value = BLOCK_LENGTH;
			}*//* else {
			 RUN_config.value = BLOCK_LENGTH
			 - ((BLOCK_LENGTH - NEZUTAKA_LENGTH) * 0.5);
			 }*/

			break;
		case 3:
			//	printf("TURNL\n");
//			tyousei_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
//			straight(tyousei_config);
//			turn_config.initial_speed = 0;
			turn_config.direction = TURN_L;
			mortor_stop();
			turn(turn_config);
			chenge_head(turn_config);
			/*if (((((map[posX + posY * MAP_X_MAX].wall & 0x0f)
			 | (map[posX + posY * MAP_X_MAX].wall << 4)) << head) & 0x20)
			 == 0x20) {
			 sirituke();
			 RUN_config.value = BLOCK_LENGTH;
			 } else {
			 RUN_config.value = BLOCK_LENGTH
			 - ((BLOCK_LENGTH - NEZUTAKA_LENGTH) * 0.5);
			 }*/
			U_config.direction = TURN_R;
		}
		tone(tone_hiC, 50);

	}
}

void hidarite(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 300, 300, 1000, BLOCK_LENGTH };
	RUNConfig turn_config = { TURN_R, 0, 0, 300, 2000, 90 };
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("hidarite\n");
		osMutexRelease(UART_MutexHandle);
	}

	Delay_ms(500);
	//wall_calibration();
	tone(tone_hiC, 10);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(10);
	for (;;) {

		//print_map();
		if (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
			mortor_stop();
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
				Delay_ms(50);
			}
			tone(tone_hiC, 200);
			break;
		}
		//osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
		wall_set(0x03);
//		make_smap(goalX, goalY, game_mode);
		print_map();
		//osThreadFlagsSet(Sensor_TaskHandle, TASK_START);

		if (wall_check(1) == 0) {
			//	printf("TURNL\n");
			turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			turn_config.direction = TURN_L;
			turn(turn_config);
			chenge_head(turn_config);
			RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			straight(RUN_config);
			chenge_pos(1);
		} else if (wall_check(0) == 0) {
			RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			//	printf("straight\n");
			straight(RUN_config);
			chenge_pos(1);
		} else if (wall_check(2) == 0) {
			//	printf("TURN R\n");
			//			RUN_config.value = (BLOCK_LENGTH - NEZUTAKA_LENGTH) / 3;
			//			straight(RUN_config);
			turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			turn_config.direction = TURN_R;
			turn(turn_config);
			chenge_head(turn_config);
			RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			straight(RUN_config);
			chenge_pos(1);
		} else {
			//	printf("U\n");
			turn_u();
		}

		tone(tone_hiC, 100);
	}
}
