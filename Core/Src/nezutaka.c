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

extern uint32_t MotorSPEED_R;
extern uint32_t MotorSPEED_L;
extern osMutexId_t UART_MutexHandle;
extern SensorData sensorData;
extern osThreadId_t Sensor_TaskHandle;
uint32_t wall_config[12] = { 1200, 1200, 2600, 2600, 450, 450, 450, 450, 650,
		650, 600, 600 };
extern MAP map[MAP_X_MAX][MAP_Y_MAX];
extern int16_t posX, posY;	//　現在の位置
extern int8_t head;	//　現在向いている方向(北東南西(0,1,2,3))

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

				printf("\r*mode:%2d %20s", *mode, mode_name[*mode]);
				osMutexRelease(UART_MutexHandle);
			}
		}
		Delay_ms(5);
	}
}
//music
void mode0(void) {
//	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("music\n");
//		osMutexRelease(UART_MutexHandle);
//	}
	music();
}
//sensordebug
void mode1(void) {
//	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
////		printf("print_sensordata\n");
//		osMutexRelease(UART_MutexHandle);
//	}

	print_sensordata();
}
//init wall value
void mode2(void) {
//	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("wall_calibration\n");
//		osMutexRelease(UART_MutexHandle);
//	}

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(2000);
	wall_config[RS_WALL] = read_wall(&sensorData.ADC_DATA_RS);
	wall_config[LS_WALL] = read_wall(&sensorData.ADC_DATA_LS);
	Delay_ms(1000);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
}
//1block run
void mode3(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 0, 800, 3000, BLOCK_LENGTH };

//	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("1block run(180mm)\n");
//		osMutexRelease(UART_MutexHandle);
//	}
	Delay_ms(500);
	tone(tone_hiC, 10);
	//ajast();
	straight(RUN_config, 1, 0, 0);
//	mortor_sleep();
	tone(tone_hiC, 50);
}
//turn R 90°
void mode4(void) {
	/*	RUNConfig turn_config = { TURN_R, 0, 0, 800, 1000, 90 };

	 if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
	 printf("turn 90°\n");
	 osMutexRelease(UART_MutexHandle);
	 }
	 Delay_ms(500);
	 tone(tone_hiC, 10);
	 turn(turn_config);
	 //	mortor_sleep();
	 chenge_head(turn_config.direction, turn_config.value, &head);
	 tone(tone_hiC, 50);*/
	/*	SLALOMConfig slalom_config = { { TURN_R, 500, 500, 2000, 800, 90 }, 6, 6 };*/
	RUNConfig turn_config = { TURN_R, 0, 0, 500, 800, 90 };
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(1000);

	turn(turn_config);

//	RUN_config.initial_speed = (MotorHz_L + MotorHz_R) / 2;
//	RUN_config.finish_speed = 0;
//	straight(RUN_config);
	motor_stop();
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
}
//turn 180° and sirituke
void mode5(void) {
	/*	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
	 printf("turn 180° and sirituke\n");
	 osMutexRelease(UART_MutexHandle);
	 }

	 Delay_ms(500);
	 tone(tone_hiC, 10);
	 turn_u();
	 //	mortor_sleep();
	 tone(tone_hiC, 50);*/
	SLALOMConfig slalom90_config =
			{ { TURN_R, 400, 400, 2000, 700, 90 }, 15, 15 };
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 400, 400, 1000, BLOCK_LENGTH / 2 };
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(1000);

	RUN_config.initial_speed = (MotorSPEED_L + MotorSPEED_R) / 2;
	straight(RUN_config, 1, 0, 0);
	slalom(slalom90_config);
	motor_stop();
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
}
//SLALOM_R
void mode6(void) {
	SLALOMConfig slalom90_config =
			{ { TURN_R, 300, 300, 2000, 800, 90 }, 15, 15 };
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 300, 300, 1000, BLOCK_LENGTH / 2 };
	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(1000);
	for (uint16_t i = 0; i < 8; i++) {
		RUN_config.initial_speed = (MotorSPEED_L + MotorSPEED_R) / 2;
		straight(RUN_config, 1, 0, 0);
		slalom(slalom90_config);
	}
	RUN_config.initial_speed = (MotorSPEED_L + MotorSPEED_R) / 2;
	RUN_config.finish_speed = 0;
	straight(RUN_config, 1, 0, 0);

	motor_stop();
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
//	RUNConfig RUN_config = { MOVE_FORWARD, 0, 0, 1000, 4000, BLOCK_LENGTH };
//	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
//	Delay_ms(2000);
//	wall_config[RS_WALL] = read_wall(&sensorData.ADC_DATA_RS);
//	wall_config[LS_WALL] = read_wall(&sensorData.ADC_DATA_LS);
//	ajast();
//	saitan(RUN_config, GOAL_X, GOAL_Y, posX, posY, head);
//	Delay_ms(100);
//	RUN_config.acceleration = RUN_config.max_speed = 800;
//	saitan(RUN_config, START_X, START_Y, GOAL_X, GOAL_Y, head);
	//	turn_u();
	//	saitan(RUN_config,START_X,START_Y,posX,posY,head);
	return;
}

void mode7(void) {
	/*
	 if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
	 printf("map init\n");
	 osMutexRelease(UART_MutexHandle);
	 }
	 */

	smap_Init();
}

void mode8(void) {
	/*	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
	 printf("print_map\n");
	 osMutexRelease(UART_MutexHandle);
	 }*/

	print_map();

}

void mode9(void) {
	//探索マップ作成
	/*	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
	 printf("tansaku map\n");
	 osMutexRelease(UART_MutexHandle);
	 }*/

	make_smap(GOAL_X, GOAL_Y, 0);
	print_map();
	return;
}

void mode10(void) {
	//最短マップ作成
	/*	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
	 printf("saitan map\n");
	 osMutexRelease(UART_MutexHandle);
	 }*/

	make_smap(GOAL_X, GOAL_Y, 1);
	print_map();
	return;
}
void mode11(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 300, 300, 500, BLOCK_LENGTH };
	uint16_t searchX = 0, searchY = 0;
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 300, 300, 500, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };
	SLALOMConfig slalom90_config =
			{ { TURN_R, 300, 300, 2000, 800, 90 }, 15, 15 }, slalom180_config =
			{ { TURN_R, 300, 300, 2000, 700, 180 }, 30, 30 };
	RUNConfig turn_config = { TURN_R, 0, 0, 2000, 800, 90 };
	posX=START_X;
	posY=START_Y;
	head=0;

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(2000);
//	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("adachi to goal\n");
//		osMutexRelease(UART_MutexHandle);
//	}
	//sirituke();
	straight(tyousei_config, 1, 0, 0);
	adachi(RUN_config, turn_config, slalom90_config, GOAL_X, GOAL_Y);
	music();
	/*
	 if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
	 printf("goal\n");
	 osMutexRelease(UART_MutexHandle);
	 }
	 */
	Delay_ms(100);
	make_smap(GOAL_X, GOAL_Y, 0);
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("tansaku map\n");
		osMutexRelease(UART_MutexHandle);
	}
	print_map();
	make_smap(GOAL_X, GOAL_Y, 1);
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("saitan map\n");
		osMutexRelease(UART_MutexHandle);
	}
	print_map();
	searchX = GOAL_X;
	searchY = GOAL_Y;
	check_searchBlock(&searchX, &searchY);
	while (searchX != GOAL_X && searchY != GOAL_Y) {
		//1最短の可能性があり未探索の場所を探索
		//　スタート位置から最短ルートをたどり、最初に来た未探索地区をゴールとした足立法走行を実施。
		//

		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("adachi to searchX=%d,searchY=%d\n", searchX, searchY);
			osMutexRelease(UART_MutexHandle);
		}
		adachi(RUN_config, turn_config, slalom90_config, searchX, searchY);
		make_smap(GOAL_X, GOAL_Y, 0);
		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("tansaku map\n");
			osMutexRelease(UART_MutexHandle);
		}
		print_map();
		make_smap(GOAL_X, GOAL_Y, 1);
		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("saitan map\n");
			osMutexRelease(UART_MutexHandle);
		}
		print_map();
		searchX = GOAL_X;
		searchY = GOAL_Y;
		check_searchBlock(&searchX, &searchY);
		Delay_ms(100);
	}
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("saitan to start\n");
		osMutexRelease(UART_MutexHandle);
	}
	RUN_config.finish_speed = 300;
	RUN_config.initial_speed = 0;
	RUN_config.acceleration = RUN_config.max_speed = 800;
	slalom90_config.after_ofset = slalom90_config.before_ofset =
			slalom180_config.after_ofset = slalom180_config.before_ofset = 15;
	saitan(RUN_config, slalom90_config, slalom180_config, START_X, START_Y,
			posX, posY, head);
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("tansaku fin\n");
		osMutexRelease(UART_MutexHandle);
	}

	/*turn_u();
	 sirituke();
	 ajast();*/
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	//　最短走行
	return;
}

void mode12(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 400, 400, 1500, BLOCK_LENGTH };
	uint16_t searchX = 0, searchY = 0;
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 400, 400, 500, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };

	SLALOMConfig slalom90_config =
			{ { TURN_R, 400, 400, 2000, 820, 90 }, 15, 15 }, slalom180_config =
			{ { TURN_R, 400, 400, 2000, 600, 180 }, 15, 15 };
	RUNConfig turn_config = { TURN_R, 0, 0, 2000, 800, BLOCK_LENGTH };
	posX=START_X;
	posY=START_Y;
	head=0;

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(2000);
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("adachi to goal\n");
		osMutexRelease(UART_MutexHandle);
	}
	//sirituke();
	straight(tyousei_config, 1, 0, 0);
	adachi(RUN_config, turn_config, slalom90_config, GOAL_X, GOAL_Y);
	music();
	/*
	 if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
	 printf("goal\n");
	 osMutexRelease(UART_MutexHandle);
	 }
	 */
	Delay_ms(100);
	make_smap(GOAL_X, GOAL_Y, 0);
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("tansaku map\n");
		osMutexRelease(UART_MutexHandle);
	}
	print_map();
	make_smap(GOAL_X, GOAL_Y, 1);
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("saitan map\n");
		osMutexRelease(UART_MutexHandle);
	}
	print_map();
	searchX = GOAL_X;
	searchY = GOAL_Y;
	check_searchBlock(&searchX, &searchY);
	while (searchX != GOAL_X && searchY != GOAL_Y) {
		//1最短の可能性があり未探索の場所を探索
		//　スタート位置から最短ルートをたどり、最初に来た未探索地区をゴールとした足立法走行を実施。
		//

		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("adachi to searchX=%d,searchY=%d\n", searchX, searchY);
			osMutexRelease(UART_MutexHandle);
		}
		adachi(RUN_config, turn_config, slalom90_config, searchX, searchY);
		make_smap(GOAL_X, GOAL_Y, 0);
		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("tansaku map\n");
			osMutexRelease(UART_MutexHandle);
		}
		print_map();
		make_smap(GOAL_X, GOAL_Y, 1);
		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			printf("saitan map\n");
			osMutexRelease(UART_MutexHandle);
		}
		print_map();
		searchX = GOAL_X;
		searchY = GOAL_Y;
		check_searchBlock(&searchX, &searchY);
		Delay_ms(100);
	}
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("saitan to start\n");
		osMutexRelease(UART_MutexHandle);
	}
	RUN_config.finish_speed = RUN_config.initial_speed = 400;
	RUN_config.acceleration = RUN_config.max_speed = 800;
	print_map();
	saitan(RUN_config, slalom90_config, slalom180_config, START_X, START_Y,
			posX, posY, head);
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//		printf("tansaku fin\n");
		osMutexRelease(UART_MutexHandle);
	}

	/*turn_u();
	 sirituke();
	 ajast();*/

	//　最短走行
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	return;
}
void mode13(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 300, 1000, 2000, BLOCK_LENGTH };
	SLALOMConfig slalom90_config =
			{ { TURN_R, 300, 300, 2000, 800, 90 }, 15, 15 }, slalom180_config =
			{ { TURN_R, 300, 300, 2000, 700, 180 }, 15, 15 };
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 0, 300, 500, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };
	posX=START_X;
	posY=START_Y;
	head=0;

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(2000);
	straight(tyousei_config, 1, 0, 0);
	saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y, posX,
			posY, head);
/*
	RUN_config.acceleration = RUN_config.max_speed = 800;
	RUN_config.finish_speed = slalom90_config.config.initial_speed =
			slalom90_config.config.finish_speed =
					slalom180_config.config.initial_speed =
							slalom180_config.config.finish_speed = 300;
	slalom90_config.config.acceleration = 800;
	slalom180_config.config.acceleration = 700;
	slalom90_config.after_ofset = slalom90_config.before_ofset =
			slalom180_config.after_ofset = slalom180_config.before_ofset = 15;

	saitan(RUN_config, slalom90_config, slalom180_config, START_X, START_Y,
			GOAL_X,
			GOAL_Y, head);*/
	//turn_u();
//	saitan(RUN_config,START_X,START_Y,posX,posY,head);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	return;
}
void mode14(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 300, 1000, 2500, BLOCK_LENGTH };
	SLALOMConfig slalom90_config =
			{ { TURN_R, 300, 300, 2000, 800, 90 }, 15, 15 }, slalom180_config =
			{ { TURN_R, 300, 300, 2000, 700, 180 }, 15, 15 };
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 0, 300, 500, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };
	posX=START_X;
	posY=START_Y;
	head=0;

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(2000);
	straight(tyousei_config, 1, 0, 0);
	saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y, posX,
			posY, head);
/*
	RUN_config.acceleration = RUN_config.max_speed = 800;
	RUN_config.finish_speed = slalom90_config.config.initial_speed =
			slalom90_config.config.finish_speed =
					slalom180_config.config.initial_speed =
							slalom180_config.config.finish_speed = 300;
	slalom90_config.config.acceleration = 800;
	slalom180_config.config.acceleration = 700;
	slalom90_config.after_ofset = slalom90_config.before_ofset =
			slalom180_config.after_ofset = slalom180_config.before_ofset = 15;

	saitan(RUN_config, slalom90_config, slalom180_config, START_X, START_Y,
			GOAL_X,
			GOAL_Y, head);*/
	//	turn_u();
	//	saitan(RUN_config,START_X,START_Y,posX,posY,head);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	return;
}

void mode15(void) {
	RUNConfig RUN_config = { MOVE_FORWARD, 0, 300, 1500, 3000, BLOCK_LENGTH };
	SLALOMConfig slalom90_config =
			{ { TURN_R, 300, 300, 2000, 800, 90 }, 15, 15 }, slalom180_config =
			{ { TURN_R, 300, 300, 2000, 700, 180 }, 15, 15 };
	RUNConfig tyousei_config = { MOVE_FORWARD, 0, 0, 300, 500, (BLOCK_LENGTH
			- NEZUTAKA_LENGTH) * 0.5 };
	posX=START_X;
	posY=START_Y;
	head=0;

	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
	Delay_ms(2000);
	straight(tyousei_config, 1, 0, 0);
	saitan(RUN_config, slalom90_config, slalom180_config, GOAL_X, GOAL_Y, posX,
			posY, head);

/*	RUN_config.acceleration = RUN_config.max_speed = 800;
	RUN_config.finish_speed = slalom90_config.config.initial_speed =
			slalom90_config.config.finish_speed =
					slalom180_config.config.initial_speed =
							slalom180_config.config.finish_speed = 300;
	slalom90_config.config.acceleration = 800;
	slalom180_config.config.acceleration = 700;
	slalom90_config.after_ofset = slalom90_config.before_ofset =
			slalom180_config.after_ofset = slalom180_config.before_ofset = 15;

	saitan(RUN_config, slalom90_config, slalom180_config, START_X, START_Y,
			GOAL_X,
			GOAL_Y, head);*/
	//	turn_u();
	//	saitan(RUN_config,START_X,START_Y,posX,posY,head);
	osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
	return;
}

