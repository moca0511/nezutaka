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
#include"agent.h"

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
extern int8_t head;	//　現在向いている方向(北東南西(0,1,2,3))

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
			chenge_head(turn_config.direction, turn_config.value, &head);
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
			chenge_head(U_config.direction, U_config.value, &head);
			if (((((map[posX + posY * MAP_X_MAX].wall & 0x0f)
					| (map[posX + posY * MAP_X_MAX].wall << 4)) << head) & 0x20)
					== 0x20) {
				mortor_stop();
				sirituke();
				/*RUN_config.value = BLOCK_LENGTH
				 + ((BLOCK_LENGTH - NEZUTAKA_LENGTH) * 0.5);*/
			}/* else {
			 RUN_config.value = BLOCK_LENGTH;
			 }
			 */
			break;
		case 3:
			//	printf("TURNL\n");
//			tyousei_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
//			straight(tyousei_config);
//			turn_config.initial_speed = 0;
			turn_config.direction = TURN_L;
			mortor_stop();
			turn(turn_config);
			chenge_head(turn_config.direction, turn_config.value, &head);
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
			chenge_head(turn_config.direction, turn_config.value, &head);
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
			chenge_head(turn_config.direction, turn_config.value, &head);
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

void saitan(RUNConfig RUN_config, uint16_t gx, uint16_t gy, uint16_t sx,
		uint16_t sy, int8_t shead) {
	RUTE rute[100] = { 0 };
	uint16_t x = sx, y = sy;
	uint16_t i = 0;
	int8_t temp_head = 0, head_buf = shead;
	RUNConfig turn_config = { TURN_R, 0, 0, 800, 1000, 90 };
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		printf("saitan\n");
		osMutexRelease(UART_MutexHandle);
	}

	make_smap(gx, gy, 1);
	print_map();
	//進行方向決定
	while (x != gx || y != gy) {
		if ((map[x + y * MAP_X_MAX].wall & 0x88) == 0x80
				&& map[x + y * MAP_X_MAX].step
						> map[x + (y + 1) * MAP_X_MAX].step) {
			temp_head = 0;
			y++;
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
				printf("kita\n");
				osMutexRelease(UART_MutexHandle);
			}
		} else if ((map[x + y * MAP_X_MAX].wall & 0x11) == 0x10
				&& map[x + y * MAP_X_MAX].step
						> map[x + y * MAP_X_MAX - 1].step) {
			temp_head = 3;
			x--;
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
				printf("nisi\n");
				osMutexRelease(UART_MutexHandle);
			}
		} else if ((map[x + y * MAP_X_MAX].wall & 0x44) == 0x40
				&& map[x + y * MAP_X_MAX].step
						> map[x + y * MAP_X_MAX + 1].step) {
			temp_head = 1;
			x++;
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
				printf("higasi\n");
				osMutexRelease(UART_MutexHandle);
			}
		} else if ((map[x + y * MAP_X_MAX].wall & 0x22) == 0x20
				&& map[x + y * MAP_X_MAX].step
						> map[x + (y - 1) * MAP_X_MAX].step) {
			temp_head = 2;
			y--;
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
				printf("minami\n");
				osMutexRelease(UART_MutexHandle);
			}
		} else {
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
				printf("errer\n");
				osMutexRelease(UART_MutexHandle);
			}
			//			mortor_sleep();
//		mortor_stop();
//		osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
//		tone(tone_C, 1000);
//		Delay_ms(1000);
//		break;
			//ERR
			return;
		}

		temp_head += 4;         //マイナス数防止
		temp_head -= head_buf;    //headの考慮
		if (temp_head >= 4) {    //桁上がりの考慮
			temp_head -= 4;
		}

		switch (temp_head) {
		case 0:
			//printf("straight\n");
			if (i > 0 && rute[i - 1].direction == MOVE_FORWARD) {
				rute[i - 1].value += BLOCK_LENGTH;
				i--;
			} else {
				rute[i].direction = temp_head;
				rute[i].value = BLOCK_LENGTH;
			}
			break;
		case 1:
			//printf("TURN R\n");
			rute[i].direction = temp_head;
			rute[i].value = 90;
			chenge_head(TURN_R, 90, &head_buf);
			i++;
			rute[i].direction = 0;
			rute[i].value = BLOCK_LENGTH;
			break;
		case 2:
			//	printf("U\n");
			rute[i].direction = temp_head;
			rute[i].value = 90;
			i++;
			rute[i].direction = temp_head;
			rute[i].value = 90;
			chenge_head(TURN_R, 180, &head_buf);

			break;
		case 3:
			//	printf("TURNL\n");
			rute[i].direction = temp_head;
			rute[i].value = 90;
			chenge_head(TURN_L, 90, &head_buf);
			i++;
			rute[i].direction = 0;
			rute[i].value = BLOCK_LENGTH;
			break;
		}
		i++;
		if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
			printf("x=%d,y=%d,i=%d\n", x, y, i);
			osMutexRelease(UART_MutexHandle);
		}
	}
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		for (int f = 0; f < i; f++) {
			printf("%d:direction=%d,value=%ld\n", f, rute[f].direction,
					rute[f].value);
		}
		osMutexRelease(UART_MutexHandle);
	}
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		printf("end\n");
		osMutexRelease(UART_MutexHandle);
	}

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(50);
	wall_config[RS_WALL] = read_wall(&sensorData.ADC_DATA_RS);
	wall_config[LS_WALL] = read_wall(&sensorData.ADC_DATA_LS);
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("saitan to goal\n");
		osMutexRelease(UART_MutexHandle);
	}
	rute[0].value += (BLOCK_LENGTH - NEZUTAKA_LENGTH) * 0.5;

	for (int f = 0; f < i; f++) {
		switch (rute[f].direction) {
		case 0:
			//printf("straight\n");
			RUN_config.value = rute[f].value;
			straight(RUN_config);
			break;
		case 1:
			//printf("TURN R\n");
			mortor_stop();
			turn_config.direction = TURN_R;
			turn(turn_config);
			chenge_head(TURN_R, 90, &head);
			break;
		case 2:
			//	printf("U\n");
			mortor_stop();
			turn_config.direction = TURN_R;
			turn(turn_config);
			mortor_stop();
			turn_config.direction = TURN_R;
			turn(turn_config);
			chenge_head(TURN_R, 180, &head);

			break;
		case 3:
			//	printf("TURNL\n");
			mortor_stop();
			turn_config.direction = TURN_L;
			turn(turn_config);
			chenge_head(TURN_L, 90, &head);
			break;
		}
	}
	turn_u();
	posX = gx;
	posY = gy;
	if (((((map[posX + posY * MAP_X_MAX].wall & 0x0f)
			| (map[posX + posY * MAP_X_MAX].wall << 4)) << head) & 0x20)
			== 0x20) {
		mortor_stop();
		sirituke();
	}
	music();

}
