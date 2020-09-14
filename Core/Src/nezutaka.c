/*
 * mode.c
 *
 *  Created on: 2020/07/28
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
uint32_t wall_config[12] = { 0 };
extern MAP map[MAP_SIZE];
extern uint8_t game_mode;	//　探索(0)・最短(1)　選択
extern int16_t posX, posY;	//　現在の位置
extern uint8_t head;	//　現在向いている方向(北東南西(0,1,2,3))

void nezutaka(void) {
	int16_t mode = 0;
	tone(tone_C, 10);
	Delay_ms(10);
	tone(tone_G, 10);
	for (;;) {
		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("mode select\n");
			osMutexRelease(UART_MutexHandle);
		}

		MENU(&mode);
		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("\n");
			osMutexRelease(UART_MutexHandle);
		}

		switch (mode) {
		case 0:
			mode0();
			break;
		case 1:
			mode1();
			break;
		case 2:
			mode2();
			break;
		case 3:
			mode3();
			break;
		case 4:
			mode4();
			break;
		case 5:
			mode5();
			break;
		case 6:
			mode6();
			break;
		case 7:
			mode7();
			//printf("return\n");
			break;
		case 8:
			mode8();
			break;
		case 9:
			mode9();
			break;
		case 10:
			mode10();
			break;
		case 11:
			mode11();
			break;
		case 12:
			mode12();
			break;
		case 13:
			mode13();
			break;
		case 14:
			mode14();
			break;
		case 15:
			mode15();
			break;
		default:
			break;
		}
		Delay_ms(10);
	}
}

void MENU(int16_t *mode) {
	static char *mode_name[16] = { MODE_NAME0, MODE_NAME1, MODE_NAME2,
	MODE_NAME3, MODE_NAME4, MODE_NAME5, MODE_NAME6, MODE_NAME7,
	MODE_NAME8, MODE_NAME9, MODE_NAME10, MODE_NAME11, MODE_NAME12,
	MODE_NAME13, MODE_NAME14, MODE_NAME15 };
	uint8_t flag = 0;
	for (;;) {
		flag = 0;
		if (HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == 0) {
			while (HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == 0) {
				Delay_ms(50);
			}
			tone(tone_E, 100);
			(*mode)++;
			flag = 1;
		} else if (HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == 0) {
			while (HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == 0) {
				Delay_ms(50);
			}
			tone(tone_D, 100);
			(*mode)--;
			flag = 1;
		}
		if (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
			while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
				Delay_ms(50);
			}
			tone(tone_hiC, 200);
			break;
		}
		if (*mode > 15)
			*mode = 0;
		if (*mode < 0)
			*mode = 15;
		UILED_SET((unsigned int) *mode);
		if (flag == 1) {
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {

				printf("\r*mode:%2d %20s", *mode, mode_name[*mode]);
				osMutexRelease(UART_MutexHandle);
			}
		}
		Delay_ms(5);
	}
}
//music
void mode0(void) {
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("music\n");
		osMutexRelease(UART_MutexHandle);
	}
	music();
}
//sensordebug
void mode1(void) {
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("print_sensordata\n");
		osMutexRelease(UART_MutexHandle);
	}

	print_sensordata();
}
//init wall value
void mode2(void) {
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("wall_calibration\n");
		osMutexRelease(UART_MutexHandle);
	}

	Delay_ms(500);
	wall_calibration();
}
//1block run
void mode3(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 0, 800, 1000, BLOCK_LENGTH * 1 };

	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("1block run(180mm)\n");
		osMutexRelease(UART_MutexHandle);
	}
	Delay_ms(500);
	tone(tone_hiC, 10);
	run_block(RUN_config);
	tone(tone_hiC, 100);
}
//turn R 90°
void mode4(void) {
	RUNConfig turn_config = { TURN_R, 0, 0, 800, 1000, 90 };

	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("turn 90°\n");
		osMutexRelease(UART_MutexHandle);
	}
	Delay_ms(500);
	tone(tone_hiC, 10);
	turn(turn_config);
	chenge_head(turn_config);
	tone(tone_hiC, 100);
}
//turn 180° and sirituke
void mode5(void) {
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("turn 180° and sirituke\n");
		osMutexRelease(UART_MutexHandle);
	}

	Delay_ms(500);
	tone(tone_hiC, 10);
	turn_u();
	tone(tone_hiC, 100);
}
//SLALOM_R
void mode6(void) {
	RUNConfig turn_config = { TURN_R, 300, 300, 300, 0, 90 };
	Delay_ms(500);
	slalom(turn_config);
	mortor_stop();
}

void mode7(void) {
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("map init\n");
		osMutexRelease(UART_MutexHandle);
	}

	smap_Init();
}

void mode8(void) {
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("print_map\n");
		osMutexRelease(UART_MutexHandle);
	}

	print_map();

}

void mode9(void) {
	//探索マップ作成
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("tansaku map\n");
		osMutexRelease(UART_MutexHandle);
	}

	game_mode = 0;
	make_smap(goalX, goalY, game_mode);
	print_map();
	return;
}

void mode10(void) {
	//最短マップ作成
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("saitan map\n");
		osMutexRelease(UART_MutexHandle);
	}

	game_mode = 1;
	make_smap(goalX, goalY, game_mode);
	print_map();
	return;
}
void mode11(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 150, 150, 1000, BLOCK_LENGTH };
	adachi(RUN_config);
	return;
}

void mode12(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 200, 200, 1000, BLOCK_LENGTH };
	adachi(RUN_config);
	return;
}
void mode13(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 400, 400, 1000, BLOCK_LENGTH };
	adachi(RUN_config);
	return;
}
void mode14(void) {
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("speed test\n");
		osMutexRelease(UART_MutexHandle);
	}

	RUNConfig RUN_config = { MOVE_FORWARD, 0, 0, 800, 2000, BLOCK_LENGTH * 5 };
	RUNConfig turn_config = { TURN_R, 0, 0, 150, 500, 90 };
	Delay_ms(500);
	//wall_calibration();
	tone(tone_hiC, 10);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(100);

	run_block(RUN_config);			//5kukaku
			/*

			 turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 turn_config.direction = TURN_R;
			 turn(turn_config);
			 chenge_head(turn_config);
			 sirituke();

			 RUN_config.value = BLOCK_LENGTH;
			 RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 run_block(RUN_config);			//5kukaku

			 turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 turn_config.direction = TURN_R;
			 turn(turn_config);
			 chenge_head(turn_config);
			 sirituke();

			 RUN_config.value = BLOCK_LENGTH * 5;
			 RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 run_block(RUN_config); //5kukaku

			 turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 turn_config.direction = TURN_L;
			 turn(turn_config);
			 chenge_head(turn_config);
			 sirituke();

			 RUN_config.value = BLOCK_LENGTH;
			 RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 run_block(RUN_config); //5kukaku

			 turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 turn_config.direction = TURN_L;
			 turn(turn_config);
			 chenge_head(turn_config);
			 sirituke();

			 RUN_config.value = BLOCK_LENGTH * 5;
			 RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 run_block(RUN_config); //5kukaku

			 turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 turn_config.direction = TURN_R;
			 turn(turn_config);
			 chenge_head(turn_config);
			 sirituke();

			 RUN_config.value = BLOCK_LENGTH;
			 RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 run_block(RUN_config); //5kukaku

			 turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 turn_config.direction = TURN_R;
			 turn(turn_config);
			 chenge_head(turn_config);
			 sirituke();

			 RUN_config.value = BLOCK_LENGTH * 5;
			 RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
			 run_block(RUN_config); //5kukaku

			 turn_u();
			 */

	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	return;
}
void mode15(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 0, 800, 1000, BLOCK_LENGTH };
	RUNConfig turn_config = { TURN_R, 0, 0, 300, 500, 90 };
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("guruguru\n");
		osMutexRelease(UART_MutexHandle);
	}

	Delay_ms(500);
	tone(tone_hiC, 10);
	for (;;) {
		if (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
			MOTORSPEED_R = MOTORSPEED_L = 0;
			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
			while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
				Delay_ms(50);
			}
			tone(tone_hiC, 200);
			break;
		}
		RUN_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
		straight(RUN_config);
		turn_config.initial_speed = (MOTORSPEED_L + MOTORSPEED_R) / 2;
		turn(turn_config);
	}
}

