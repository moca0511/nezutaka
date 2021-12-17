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
#include "arm_math.h"
#include "arm_const_structs.h"

uint32_t wall_config[WALL_DATA_MAX] = { 1300, 1300, 2000, 2000, 450, 450, 450,
		450, 650, 650, 600, 600 };
int16_t posX = 0, posY = 0;	//　現在の位置
int8_t head = 0;	//　現在向いている方向(北東南西(0,1,2,3))
extern MAP map[MAP_X_MAX][MAP_Y_MAX];

void nezutaka(void) {

	int16_t mode = 0;

	init_Motor();

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
		tone(tone_C, 10);
		Delay_ms(10);
		tone(tone_G, 10);
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
			tone(tone_E, 50);
			(*mode)++;
			flag = 1;
		} else if (HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == 0) {
			while (HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == 0) {
				Delay_ms(50);
			}
			tone(tone_D, 50);
			(*mode)--;
			flag = 1;
		}
		if (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
			while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
				Delay_ms(50);
			}
			tone(tone_hiC, 50);
			break;
		}
		if (*mode > 15)
			*mode = 0;
		if (*mode < 0)
			*mode = 15;
		UILED_SET((unsigned int) *mode);
		if (flag == 1) {
			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {

				printf("\r*mode:%2d %30s", *mode, mode_name[*mode]);
				osMutexRelease(UART_MutexHandle);
			}
		}
		Delay_ms(5);
	}
}
//music
void mode0(void) {
	RUNConfig RUN_config_def = { MOVE_FORWARD, 0, 400, 1000, 500, BLOCK_LENGTH };
	SLALOMConfig slalom90_config_def = { { TURN_R, 400, 400, 2000, 1200, 90 },
			18, 800, 18, AFTER_OFSET_AD_VALUE }, slalom180_config_def = { {
	TURN_R, 400, 400, 2000, 600, 180 }, 15, 1000, 15,
	AFTER_OFSET_AD_VALUE };

//	mode3();	//sensor calibration
	mode7();	//maze init
	mode11();	//search maze
	mode12();	//search maze return
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	make_smap(GOAL_X, GOAL_Y, 1);
	print_map();
	if (map[START_X][START_Y].step == 255) {
		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("search Err\n");
			osMutexRelease(UART_MutexHandle);
		}
		return;
	}
	UILED_SET(1);
	Delay_ms(2000);
	maze_save();
	music();
	{
		RUNConfig RUN_config =
				{ MOVE_FORWARD, 0, 400, 1300, 2300, BLOCK_LENGTH };
		SLALOMConfig slalom90_config = { { TURN_R, 400, 400, 2000, 1200, 90 },
				15, 900, 15, AFTER_OFSET_AD_VALUE }, slalom180_config = { {
		TURN_R, 400, 400, 2000, 600, 180 }, 15, 1000, 15,
		AFTER_OFSET_AD_VALUE };
		saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y,
				posX, posY, head);
	}
	saitan(RUN_config_def, slalom90_config_def, slalom180_config_def, START_X,
	START_Y, posX, posY, head);
	UILED_SET(3);
	{
		RUNConfig RUN_config =
				{ MOVE_FORWARD, 0, 500, 1000, 2300, BLOCK_LENGTH };
		SLALOMConfig slalom90_config = { { TURN_R, 500, 500, 2000, 1400, 90 },
				10, 800, 10, AFTER_OFSET_AD_VALUE }, slalom180_config = { {
		TURN_R, 300, 300, 2000, 700, 180 }, 15, 900, 15,
		AFTER_OFSET_AD_VALUE };

		Delay_ms(2000);
		music();
		saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y,
				posX, posY, head);

	}
	saitan(RUN_config_def, slalom90_config_def, slalom180_config_def, START_X,
	START_Y, posX, posY, head);
	UILED_SET(7);
	{
		RUNConfig RUN_config =
				{ MOVE_FORWARD, 0, 500, 1300, 2300, BLOCK_LENGTH };
		SLALOMConfig slalom90_config = { { TURN_R, 500, 500, 2000, 1400, 90 },
				10, 800, 10, AFTER_OFSET_AD_VALUE }, slalom180_config = { {
		TURN_R, 300, 300, 2000, 700, 180 }, 15, 900, 15,
		AFTER_OFSET_AD_VALUE };

		Delay_ms(2000);
		music();
		saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y,
				posX, posY, head);

	}
	saitan(RUN_config_def, slalom90_config_def, slalom180_config_def, START_X,
	START_Y, posX, posY, head);
	UILED_SET(15);
	{

		RUNConfig RUN_config =
				{ MOVE_FORWARD, 0, 500, 1300, 2500, BLOCK_LENGTH };
		SLALOMConfig slalom90_config = { { TURN_R, 500, 500, 2000, 1400, 90 },
				10, 800, 10, AFTER_OFSET_AD_VALUE }, slalom180_config = { {
		TURN_R, 300, 300, 2000, 700, 180 }, 15, 900, 15,
		AFTER_OFSET_AD_VALUE };
		Delay_ms(2000);
		music();
		saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y,
				posX, posY, head);

	}
	saitan(RUN_config_def, slalom90_config_def, slalom180_config_def, START_X,
	START_Y, posX, posY, head);
	return;
}
//sensordebug
void mode1(void) {
	RUNConfig RUN_config_def = { MOVE_FORWARD, 0, 400, 1000, 500, BLOCK_LENGTH };
	SLALOMConfig slalom90_config_def = { { TURN_R, 400, 400, 2000, 1200, 90 },
			18, 800, 18, AFTER_OFSET_AD_VALUE }, slalom180_config_def = { {
	TURN_R, 400, 400, 2000, 600, 180 }, 15, 1000, 15,
	AFTER_OFSET_AD_VALUE };

//	mode3();	//sensor calibration
	mode7();	//maze init
	mode11();	//search maze
	mode12();	//search maze return
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	make_smap(GOAL_X, GOAL_Y, 1);
	print_map();
	if (map[START_X][START_Y].step == 255) {
		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("search Err\n");
			osMutexRelease(UART_MutexHandle);
		}
		return;
	}
	UILED_SET(1);
	Delay_ms(2000);
	maze_save();
	music();
	{
		RUNConfig RUN_config =
				{ MOVE_FORWARD, 0, 400, 1300, 2300, BLOCK_LENGTH };
		SLALOMConfig slalom90_config = { { TURN_R, 400, 400, 2000, 1200, 90 },
				15, 900, 15, AFTER_OFSET_AD_VALUE }, slalom180_config = { {
		TURN_R, 400, 400, 2000, 600, 180 }, 15, 1000, 15,
		AFTER_OFSET_AD_VALUE };
		saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y,
				posX, posY, head);
	}
	mode12();
//	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	UILED_SET(3);
	{
		RUNConfig RUN_config =
				{ MOVE_FORWARD, 0, 500, 1000, 2300, BLOCK_LENGTH };
		SLALOMConfig slalom90_config = { { TURN_R, 500, 500, 2000, 1400, 90 },
				10, 800, 10, AFTER_OFSET_AD_VALUE }, slalom180_config = { {
		TURN_R, 300, 300, 2000, 700, 180 }, 15, 900, 15,
		AFTER_OFSET_AD_VALUE };

		Delay_ms(2000);
		music();
		saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y,
				posX, posY, head);

	}
	mode12();
//	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	UILED_SET(7);
	{
		RUNConfig RUN_config =
				{ MOVE_FORWARD, 0, 500, 1300, 2300, BLOCK_LENGTH };
		SLALOMConfig slalom90_config = { { TURN_R, 500, 500, 2000, 1400, 90 },
				10, 800, 10, AFTER_OFSET_AD_VALUE }, slalom180_config = { {
		TURN_R, 300, 300, 2000, 700, 180 }, 15, 900, 15,
		AFTER_OFSET_AD_VALUE };

		Delay_ms(2000);
		music();
		saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y,
				posX, posY, head);

	}
	mode12();
//	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	UILED_SET(15);
	{

		RUNConfig RUN_config =
				{ MOVE_FORWARD, 0, 500, 1300, 2500, BLOCK_LENGTH };
		SLALOMConfig slalom90_config = { { TURN_R, 500, 500, 2000, 1400, 90 },
				10, 800, 10, AFTER_OFSET_AD_VALUE }, slalom180_config = { {
		TURN_R, 300, 300, 2000, 700, 180 }, 15, 900, 15,
		AFTER_OFSET_AD_VALUE };

		Delay_ms(2000);
		music();
		saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y,
				posX, posY, head);

	}
	saitan(RUN_config_def, slalom90_config_def, slalom180_config_def, START_X,
	START_Y, posX, posY, head);
	return;
}
//print debug
void mode2(void) {
	print_sensordata();
}
//init wall value
void mode3(void) {
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(1000);
	wall_config[RS_WALL] = read_wall(RS);
	wall_config[LS_WALL] = read_wall(LS);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);

}
//sensor read
void mode4(void) {
	Delay_ms(500);
	tone(tone_hiC, 10);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	while (1) {
		if (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
			while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
				Delay_ms(50);
			}
			tone(tone_hiC, 50);
			break;
		}
	}
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	tone(tone_hiC, 50);
}
//maze save
void mode5(void) {

	maze_save();
}
//maze load
void mode6(void) {
	maze_load();
}
//maze_init
void mode7(void) {

	smap_Init();
}
//map print
void mode8(void) {

	print_map();
}
//make step map (search)
void mode9(void) {

	make_smap(GOAL_X, GOAL_Y, 0);
	print_map();
	return;
}
//make step map (saitan)
void mode10(void) {
	make_smap(GOAL_X, GOAL_Y, 1);
	print_map();
	return;
}
//search maze by adachi LO
void mode11(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 300, 300, 300, BLOCK_LENGTH };
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 300, 300, 300, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };
	SLALOMConfig slalom90_config = { { TURN_R, 300, 300, 2000, 900, 90 }, 18,
			800, 18, AFTER_OFSET_AD_VALUE }, slalom180_config = { { TURN_R, 300,
			300, 2000, 700, 180 }, 30, 1000, 30, AFTER_OFSET_AD_VALUE };
	RUNConfig turn_config = { TURN_R, 0, 0, 2000, AFTER_OFSET_AD_VALUE, 90 };
	posX = START_X;
	posY = START_Y;
	head = 0;

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(2000);

	straight(tyousei_config, 1, 0, 0);
	adachi(RUN_config, turn_config, slalom90_config, GOAL_X, GOAL_Y);
	if (posX == GOAL_X && posY == GOAL_Y) {
		Delay_ms(10);
		maze_save();
		music();
	} else {
		motor_stop();
		osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
		UILED_SET(15);
		tone(tone_C, 1000);
		return;
	}
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	return;
}
//search maze by adachi HIGH
void mode12(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 300, 300, 300, BLOCK_LENGTH };
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 300, 300, 300, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };
	SLALOMConfig slalom90_config = { { TURN_R, 300, 300, 2000, 900, 90 }, 18,
			900, 18, AFTER_OFSET_AD_VALUE }, slalom180_config = { { TURN_R, 300,
			300, 2000, 700, 180 }, 30, 1000, 30, AFTER_OFSET_AD_VALUE };
	RUNConfig turn_config = { TURN_R, 0, 0, 2000, AFTER_OFSET_AD_VALUE, 90 };

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(2000);

	adachi(RUN_config, turn_config, slalom90_config, START_X, START_Y);
	if (posX == START_X && posY == START_Y) {
		Delay_ms(10);
		maze_save();
		music();
	} else {
		motor_stop();
		osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
		UILED_SET(15);
		tone(tone_C, 1000);
		return;
	}

//	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	return;
}

//saitan LO
void mode13(void) {

	RUNConfig RUN_config =
			{ MOVE_FORWARD, 0, 400, 1300, 2300, BLOCK_LENGTH };
	SLALOMConfig slalom90_config = { { TURN_R, 400, 400, 2000, 1200, 90 },
			15, 900, 15, AFTER_OFSET_AD_VALUE }, slalom180_config = { {
	TURN_R, 400, 400, 2000, 600, 180 }, 15, 1000, 15,
	AFTER_OFSET_AD_VALUE };
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 0, 300, 500, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };

	posX = START_X;
	posY = START_Y;
	head = 0;

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(2000);
	straight(tyousei_config, 1, 0, 0);
	saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y, posX,
			posY, head);

	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	return;
}
//saitan MID
void mode14(void) {

	RUNConfig RUN_config =
			{ MOVE_FORWARD, 0, 500, 1300, 2300, BLOCK_LENGTH };
	SLALOMConfig slalom90_config = { { TURN_R, 500, 500, 2000, 1400, 90 },
			10, 800, 10, AFTER_OFSET_AD_VALUE }, slalom180_config = { {
	TURN_R, 300, 300, 2000, 700, 180 }, 15, 900, 15,
	AFTER_OFSET_AD_VALUE };
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 0, 300, 500, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };
	posX = START_X;
	posY = START_Y;
	head = 0;

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(2000);
	straight(tyousei_config, 1, 0, 0);
	saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y, posX,
			posY, head);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	return;
}
//saitan HIGH
void mode15(void) {
	RUNConfig RUN_config =
			{ MOVE_FORWARD, 0, 500, 1300, 2500, BLOCK_LENGTH };
	SLALOMConfig slalom90_config = { { TURN_R, 500, 500, 2000, 1400, 90 },
			10, 800, 10, AFTER_OFSET_AD_VALUE }, slalom180_config = { {
	TURN_R, 300, 300, 2000, 700, 180 }, 15, 900, 15,
	AFTER_OFSET_AD_VALUE };
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 0, 300, 500, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };
	posX = START_X;
	posY = START_Y;
	head = 0;

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(2000);
	straight(tyousei_config, 1, 0, 0);
	saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y, posX,
			posY, head);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	return;
}

