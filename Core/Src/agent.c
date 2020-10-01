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
extern uint32_t MotorHz_R;
extern uint32_t MotorHz_L;
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

void adachi(RUNConfig RUN_config, RUNConfig turn_config,
		SLALOMConfig slalom90_config, uint16_t gx, uint16_t gy) {
//	RUNConfig turn_config = { TURN_R, 300, 300, 2000, 800, 90 };
	RUNConfig U_config = { TURN_R, 0, 0, 200, 500, 180 };
	uint8_t temp_head = 0;
	int32_t speed_buf = RUN_config.finish_speed;
	int8_t move_f = -1;
	game_mode = 0;
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("adachi\n");
		osMutexRelease(UART_MutexHandle);
	}

	tone(tone_hiC, 10);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);

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
			RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
			RUN_config.finish_speed = 0;
			RUN_config.value = BLOCK_LENGTH * 0.5;
			straight(RUN_config);

			mortor_stop();
			if (((((map[posX + posY * MAP_X_MAX].wall & 0x0f)
					| (map[posX + posY * MAP_X_MAX].wall << 4)) << head) & 0x80)
					== 0x80) {
				turn_u();
				mortor_stop();
				sirituke();
				ajast();
			}
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			music();
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("G\n");
				osMutexRelease(UART_MutexHandle);
			}
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
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("err\n");
				osMutexRelease(UART_MutexHandle);
			}
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
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("S\n");
				osMutexRelease(UART_MutexHandle);
			}
			RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
			RUN_config.value = BLOCK_LENGTH * 0.5;
			//printf("straight\n");
			if (straight(RUN_config) != 1 && move_f == -1) {
				wall_set(0x01);
				wall_set_around();
			} else if ((move_f *= -1) == 1) {
				chenge_pos(1);
				wall_set(0x03);
				wall_set_around();
			}
//			tone(tone_C, 50);
//			RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
//			RUN_config.value = BLOCK_LENGTH * 0.2;
//			straight(RUN_config);
//			wall_set(0x01);

			RUN_config.value = BLOCK_LENGTH;
			break;
		case 1:
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("R\n");
				osMutexRelease(UART_MutexHandle);
			}
			if (move_f == -1) {
				RUN_config.initial_speed = HztoSPEED(
						(MotorHz_L + MotorHz_R) / 2);
				RUN_config.finish_speed = 0;
				RUN_config.value = BLOCK_LENGTH * 0.5;
				straight(RUN_config);
				turn_config.direction = TURN_R;
				turn_config.finish_speed = turn_config.initial_speed = 0;
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				RUN_config.finish_speed = speed_buf;
				U_config.direction = TURN_L;
				turn_config.finish_speed = turn_config.initial_speed = 300;
			} else {
				slalom90_config.config.direction = TURN_R;
				slalom(slalom90_config);
				chenge_head(slalom90_config.config.direction, slalom90_config.config.value, &head);
				RUN_config.finish_speed = speed_buf;
				U_config.direction = TURN_L;
				chenge_pos(1);
				wall_set(0x03);
				wall_set_around();
				move_f = 1;
			}

//			RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
//			RUN_config.finish_speed = 0;
//			RUN_config.value = BLOCK_LENGTH * 0.5;
//			straight(RUN_config);
//			move_f = -1;
//			turn_config.direction = TURN_R;
//			turn(turn_config);

//			RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
//			RUN_config.value = BLOCK_LENGTH * 0.05;
//			straight(RUN_config);

			break;
		case 2:
			//	printf("U\n");
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("U\n");
				osMutexRelease(UART_MutexHandle);
			}
			RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
			RUN_config.finish_speed = 0;
			RUN_config.value = BLOCK_LENGTH * 0.6;
			straight(RUN_config);
			move_f = -1;
			mortor_stop();
			turn(U_config);
			chenge_head(U_config.direction, U_config.value, &head);
			if (((((map[posX + posY * MAP_X_MAX].wall & 0x0f)
					| (map[posX + posY * MAP_X_MAX].wall << 4)) << head) & 0x20)
					== 0x20) {
				mortor_stop();
				sirituke();
				ajast();
			}
			RUN_config.finish_speed = speed_buf;
			break;
		case 3:
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("L\n");
				osMutexRelease(UART_MutexHandle);
			}
			if (move_f == -1) {
				RUN_config.initial_speed = HztoSPEED(
						(MotorHz_L + MotorHz_R) / 2);
				RUN_config.finish_speed = 0;
				RUN_config.value = BLOCK_LENGTH * 0.5;
				straight(RUN_config);
				turn_config.direction = TURN_L;
				turn_config.finish_speed = turn_config.initial_speed = 0;
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				RUN_config.finish_speed = speed_buf;
				U_config.direction = TURN_R;
				turn_config.finish_speed = turn_config.initial_speed = 300;
			} else {
				slalom90_config.config.direction = TURN_L;
				slalom(slalom90_config);
				chenge_head(slalom90_config.config.direction, slalom90_config.config.value, &head);
				RUN_config.finish_speed = speed_buf;
				U_config.direction = TURN_R;
				chenge_pos(1);
				wall_set(0x03);
				wall_set_around();
				move_f = 1;
			}
//			RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
//			RUN_config.value = BLOCK_LENGTH * 0.05;
//			straight(RUN_config);
		}
		tone(tone_hiC, 50);

	}
}

void hidarite(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 300, 300, 1000, BLOCK_LENGTH };
	RUNConfig turn_config = { TURN_R, 0, 200, 300, 2000, 90 };
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
			turn_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
			turn_config.direction = TURN_L;
			turn(turn_config);
			chenge_head(turn_config.direction, turn_config.value, &head);
			RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
			straight(RUN_config);
			chenge_pos(1);
		} else if (wall_check(0) == 0) {
			RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
			//	printf("straight\n");
			straight(RUN_config);
			chenge_pos(1);
		} else if (wall_check(2) == 0) {
			//	printf("TURN R\n");
			//			RUN_config.value = (BLOCK_LENGTH - NEZUTAKA_LENGTH) / 3;
			//			straight(RUN_config);
			turn_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
			turn_config.direction = TURN_R;
			turn(turn_config);
			chenge_head(turn_config.direction, turn_config.value, &head);
			RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
			straight(RUN_config);
			chenge_pos(1);
		} else {
			//	printf("U\n");
			turn_u();
		}

		tone(tone_hiC, 100);
	}
}

void saitan(RUNConfig RUN_config, SLALOMConfig slalom90_config,
		SLALOMConfig slalom180_config, uint16_t gx, uint16_t gy, uint16_t sx,
		uint16_t sy, int8_t shead) {
	RUTE rute[100]={0};
	uint16_t x = sx, y = sy;
	uint16_t i = 0;
	int8_t temp_head = 0, head_buf = shead;
	uint8_t temp_wall;
//	RUNConfig turn_config = { TURN_R, 400, 400, 2000, 700, 90 };
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		printf("saitan\n");
		osMutexRelease(UART_MutexHandle);
	}

	make_smap(gx, gy, 1);
	print_map();
//進行方向決定

	while (x != gx || y != gy) {
		temp_wall = (((((map[x + y * MAP_X_MAX].wall & 0x0f)
				+ ((map[x + y * MAP_X_MAX].wall & 0x0f) << 4)) << head_buf)
				& 0xf0) >> 4)
				+ ((((map[x + y * MAP_X_MAX].wall & 0xf0)
						+ (map[x + y * MAP_X_MAX].wall >> 4)) << head_buf)
						& 0xf0);
		temp_head = 4;
		if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
			printf("temp_wall=0x%2x\n", temp_wall);
			osMutexRelease(UART_MutexHandle);
		}
		if ((temp_wall & 0x88) == 0x80) {
			switch (head_buf) {
			case 0:
				if (map[x + y * MAP_X_MAX].step
						> map[x + (y + 1) * MAP_X_MAX].step) {
					y++;
					temp_head = 0;
				}
				break;
			case 1:
				if (map[x + y * MAP_X_MAX].step
						> map[x + y * MAP_X_MAX + 1].step) {
					x++;
					temp_head = 0;
				}
				break;
			case 2:
				if (map[x + y * MAP_X_MAX].step
						> map[x + (y - 1) * MAP_X_MAX].step) {
					y--;
					temp_head = 0;
				}
				break;
			case 3:
				if (map[x + y * MAP_X_MAX].step
						> map[x + y * MAP_X_MAX - 1].step) {
					x--;
					temp_head = 0;
				}
				break;
			}
		}
		if ((temp_wall & 0x44) == 0x40 && temp_head == 4) {
			switch (head_buf) {
			case 0:
				if (map[x + y * MAP_X_MAX].step
						> map[x + y * MAP_X_MAX + 1].step) {
					x++;
					temp_head = 1;
				}
				break;
			case 1:
				if (map[x + y * MAP_X_MAX].step
						> map[x + (y - 1) * MAP_X_MAX].step) {
					y--;
					temp_head = 1;
				}
				break;
			case 2:
				if (map[x + y * MAP_X_MAX].step
						> map[x + y * MAP_X_MAX - 1].step) {
					x--;
					temp_head = 1;
				}
				break;
			case 3:
				if (map[x + y * MAP_X_MAX].step
						> map[x + (y + 1) * MAP_X_MAX].step) {
					y++;
					temp_head = 1;
				}
				break;
			}
		}
		if ((temp_wall & 0x11) == 0x10 && temp_head == 4) {
			switch (head_buf) {
			case 0:
				if (map[x + y * MAP_X_MAX].step
						> map[x + y * MAP_X_MAX - 1].step) {
					x--;
					temp_head = 3;
				}
				break;
			case 1:
				if (map[x + y * MAP_X_MAX].step
						> map[x + (y + 1) * MAP_X_MAX].step) {
					y++;
					temp_head = 3;
				}
				break;
			case 2:
				if (map[x + y * MAP_X_MAX].step
						> map[x + y * MAP_X_MAX + 1].step) {
					x++;
					temp_head = 3;
				}
				break;
			case 3:
				if (map[x + y * MAP_X_MAX].step
						> map[x + (y - 1) * MAP_X_MAX].step) {
					y--;
					temp_head = 3;
				}
				break;
			}
		}
		if ((temp_wall & 0x22) == 0x20 && temp_head == 4) {

			switch (head_buf) {
			case 0:
				if (map[x + y * MAP_X_MAX].step
						> map[x + (y - 1) * MAP_X_MAX].step) {
					y--;
					temp_head = 2;
				}
				break;
			case 1:
				if (map[x + y * MAP_X_MAX].step
						> map[x + y * MAP_X_MAX - 1].step) {
					x--;
					temp_head = 2;
				}
				break;
			case 2:
				if (map[x + y * MAP_X_MAX].step
						> map[x + y * MAP_X_MAX + 1].step) {
					x++;
					temp_head = 2;
				}
				break;
			case 3:
				if (map[x + y * MAP_X_MAX].step
						> map[x + (y + 1) * MAP_X_MAX].step) {
					y++;
					temp_head = 2;
				}
				break;
			}
		}
		if (temp_head == 4) {
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
				printf("errer\n");
				osMutexRelease(UART_MutexHandle);
			}
			//			mortor_sleep();
			mortor_stop();
//		osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
//		tone(tone_C, 1000);
//		Delay_ms(1000);
//		break;
			//ERR
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			return;
		}

//		temp_head += 4;         //マイナス数防止
//		temp_head -= head_buf;    //headの考慮
//		if (temp_head >= 4) {    //桁上がりの考慮
//			temp_head -= 4;
//		}

		switch (temp_head) {
		case 0:
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
				printf("S\n");
				osMutexRelease(UART_MutexHandle);
			}
			//printf("straight\n");
			if (i > 0 && rute[i - 1].direction == MOVE_FORWARD) {
				rute[i - 1].value += BLOCK_LENGTH;
				i--;
			} else {
				rute[i].direction = temp_head;
				if (i > 0
						&& (rute[i - 1].direction == 1
								|| rute[i - 1].direction == 3)) {
					rute[i].value = BLOCK_LENGTH;
				} else {
					rute[i].value = BLOCK_LENGTH / 2;
				}
			}

			break;
		case 1:
			//printf("TURN R\n");
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
				printf("R\n");
				osMutexRelease(UART_MutexHandle);
			}
//			if (rute[i - 1].direction == MOVE_FORWARD) {
//				rute[i - 1].value -= BLOCK_LENGTH / 2;
//
//			}
			rute[i].direction = temp_head;
			rute[i].value = 90;
			chenge_head(TURN_R, 90, &head_buf);
//			i++;
//			rute[i].direction = 0;
//			rute[i].value = BLOCK_LENGTH;
			break;
		case 2:
			//	printf("U\n");
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
				printf("U\n");
				osMutexRelease(UART_MutexHandle);
			}
			rute[i].direction = temp_head;
			rute[i].value = 180;
			chenge_head(TURN_R, 180, &head_buf);
			i++;
			rute[i].direction = 0;
			rute[i].value = BLOCK_LENGTH;

			break;
		case 3:
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
				printf("L\n");
				osMutexRelease(UART_MutexHandle);
			}
			//	printf("TURNL\n");
//			if (rute[i - 1].direction == MOVE_FORWARD) {
//				rute[i - 1].value -= BLOCK_LENGTH / 2;
//
//			}
			rute[i].direction = temp_head;
			rute[i].value = 90;
			chenge_head(TURN_L, 90, &head_buf);
//			i++;
//			rute[i].direction = 0;
//			rute[i].value = BLOCK_LENGTH;
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

	for (int f = 0; f < i; f++) {
		switch (rute[f].direction) {
		case 0:
			//printf("straight\n");
			RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
			RUN_config.value = rute[f].value;
			straight(RUN_config);
			break;
		case 1:
			//printf("TURN R\n");
			//mortor_stop();
			slalom90_config.config.direction = TURN_R;
			slalom90_config.config.value = rute[f].value;
			slalom(slalom90_config);
			chenge_head(TURN_R, 90, &head);
			break;
		case 2:
			//	printf("U\n");
			//mortor_stop();
			turn_u();
			break;
		case 3:
			//	printf("TURNL\n");
			//mortor_stop();
			slalom90_config.config.direction = TURN_L;
			slalom90_config.config.value = rute[f].value;
			slalom(slalom90_config);
			chenge_head(TURN_L, 90, &head);
			break;
		}
	}
	RUN_config.initial_speed = HztoSPEED((MotorHz_L + MotorHz_R) / 2);
	RUN_config.finish_speed = 0;
	RUN_config.value = BLOCK_LENGTH / 2;
	straight(RUN_config);
	turn_u();
	posX = gx;
	posY = gy;
	if (((((map[posX + posY * MAP_X_MAX].wall & 0x0f)
			| (map[posX + posY * MAP_X_MAX].wall << 4)) << head) & 0x20)
			== 0x20) {
//		mortor_stop();
		sirituke();
		ajast();
	}
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	music();

}
