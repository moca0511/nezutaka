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

extern uint32_t wall_config[12];
extern MAP map[MAP_X_MAX][MAP_Y_MAX];
extern int16_t posX, posY;	//　現在の位置
extern int8_t head;	//　現在向いている方向(北東南西(0,1,2,3))

//足立法探索　ゴール
//1.目標地点確認　現在位置よりステップ数が少なく　未到達かつ壁情報読んでなくてゴールじゃない場所で　一番近い場所
//2.そこまで行くための道のり（最短マップ）算出
//3.最短できなければいけるところまで行く。
//4.ゴールなら終了
//5.1に戻る

/*
 * 説明：足立法探索
 * 引数：RUN_config 直進パラメータ
 * 　　　turn_config 超新地旋回パラメータ
 * 　　　slalom90_config 90°スラロームパラメータ
 * 　　　gx 目標X座標
 * 　　　gy 目標Y座標
 * 戻り値：無し
 */
void adachi(RUNConfig RUN_config, RUNConfig turn_config,
		SLALOMConfig slalom90_config, uint16_t gx, uint16_t gy) {
	uint8_t temp_head = 0;
	int32_t speed_buf = RUN_config.finish_speed;
	int8_t move_f = -1;
	int8_t move = 0;
	int8_t remenber = 1;

	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("adachi\n");
		osMutexRelease(UART_MutexHandle);
	}

	tone(tone_hiC, 10);

	for (;;) {
		osDelayUntil(osKernelGetTickCount() + 5);
		if (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
			motor_stop();
			while (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
				Delay_ms(50);
			}
			tone(tone_hiC, 200);
			break;
		}

		if (map[gx][gy].wall == 0xff) {
//			motor_stop();
			if (move_f == 1) {    //中央に移動
				RUN_config.initial_speed = get_MotorSpeed();
				RUN_config.max_speed = 200;
				RUN_config.value = BLOCK_LENGTH * 0.5;
				RUN_config.finish_speed = 0;
				RUN_config.acceleration = 1000;
				straight(RUN_config, 1, 0, 0);
			}
			motor_stop();
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("can't to goal\n");
				osMutexRelease(UART_MutexHandle);
			}
			break;
		}

		if (posX == gx && posY == gy) {
			if (move_f == 1) {    //中央に移動
				RUN_config.initial_speed = get_MotorSpeed();
				RUN_config.max_speed = 200;
				RUN_config.finish_speed = 0;
				RUN_config.acceleration = 1000;
				RUN_config.value = BLOCK_LENGTH / 2;
				straight(RUN_config, 1, 0, 1);
			}

			motor_stop();
			if (((((map[posX][posY].wall & 0x0f) | (map[posX][posY].wall << 4))
					<< head) & 0xc0) == 0xc0) {
				turn_config.direction = TURN_L;
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
			} else if (((((map[posX][posY].wall & 0x0f)
					| (map[posX][posY].wall << 4)) << head) & 0x90) == 0x90) {
				turn_config.direction = TURN_R;
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
			} else if (((((map[posX][posY].wall & 0x0f)
					| (map[posX][posY].wall << 4)) << head) & 0x80) == 0x80) {
				turn_u();
				sirituke();
				ajast();
			}
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("G\n");
				osMutexRelease(UART_MutexHandle);
			}
			break;
		}

		make_smap(gx, gy, 0);
		temp_head = 99;
		remenber = 99;
		if ((map[posX][posY].wall & 0x22) == 0x20
				&& map[posX][posY].step > map[posX][posY - 1].step) {
			temp_head = 2;
			if ((map[posX][posY - 1].wall & 0xf0) != 0xf0) {
				remenber = 2;
			}
		}

		if ((map[posX][posY].wall & 0x44) == 0x40
				&& map[posX][posY].step > map[posX + 1][posY].step) {
			temp_head = 1;
			if ((map[posX + 1][posY].wall & 0xf0) != 0xf0) {
				remenber = 1;
			}
		}
		if ((map[posX][posY].wall & 0x11) == 0x10
				&& map[posX][posY].step > map[posX - 1][posY].step) {
			temp_head = 3;
			if ((map[posX - 1][posY].wall & 0xf0) != 0xf0) {
				remenber = 3;
			}
		}
		if ((map[posX][posY].wall & 0x88) == 0x80
				&& map[posX][posY].step > map[posX][posY + 1].step) {
			temp_head = 0;
			if ((map[posX][posY + 1].wall & 0xf0) != 0xf0) {
				remenber = 0;
			}
		}
		if (remenber != 99) {
			temp_head = remenber;
		}
		if (temp_head == 99) {
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("err\n");
				osMutexRelease(UART_MutexHandle);
			}
			if (move_f == 1) {    //中央に移動
				RUN_config.initial_speed = get_MotorSpeed();
				RUN_config.max_speed = 200;
				RUN_config.finish_speed = 0;
				RUN_config.acceleration = 1000;
				RUN_config.value = BLOCK_LENGTH / 2;
				straight(RUN_config, 1, 0, 1);
			}
			if (((((map[posX][posY].wall & 0x0f) | (map[posX][posY].wall << 4))
					<< head) & 0xc0) == 0xc0) {
				turn_config.direction = TURN_L;
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
			} else if (((((map[posX][posY].wall & 0x0f)
					| (map[posX][posY].wall << 4)) << head) & 0x90) == 0x90) {
				turn_config.direction = TURN_R;
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
			} else if (((((map[posX][posY].wall & 0x0f)
					| (map[posX][posY].wall << 4)) << head) & 0x80) == 0x80) {
				turn_u();
				sirituke();
				ajast();
			}
			motor_stop();
			break;
		}

		temp_head += 4;         //マイナス数防止
		temp_head -= head;    //headの考慮
		if (temp_head >= 4) {    //桁上がりの考慮
			temp_head -= 4;
		}
//		print_map();
//		if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//			printf("x:%d y:%d\n", posX, posY);
//			osMutexRelease(UART_MutexHandle);
//		}

		switch (temp_head) {
		case 0:
//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("S\n");
//				osMutexRelease(UART_MutexHandle);
//			}
			RUN_config.initial_speed = get_MotorSpeed();
			RUN_config.value = BLOCK_LENGTH * 0.5;

			if (move_f == -1) {    //中央から区切りへ
				move = straight(RUN_config, 1, 0, 1);
			} else {    //区切りから中央へ
				if (posX == gx && posY == gy) {
					RUN_config.max_speed = 200;
					RUN_config.finish_speed = 0;
					RUN_config.acceleration = 1000;
					RUN_config.value = BLOCK_LENGTH / 2;
				}
				move = straight(RUN_config, 1, 0, 1);

			}
			/*			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			 printf("move_F:%d move:%d\n", move_f, move);
			 osMutexRelease(UART_MutexHandle);
			 }*/
			if (move == 0) {    //ERRなら前方確認
				wall_set(0x01);
				wall_set_around();
			} else if ((move_f *= -1) == 1) {    //区切り
				chenge_pos(1, &posX, &posY, head);
				wall_set(0x03);
				wall_set_around();
			}

			break;
		case 1:
//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("R\n");
//				osMutexRelease(UART_MutexHandle);
//			}
			if (move_f == -1) {
				if (((((map[posX][posY].wall & 0x0f)
						| (map[posX][posY].wall << 4)) << head) & 0xc0)
						== 0xc0) {
					turn_config.direction = TURN_L;
					turn(turn_config);
					chenge_head(turn_config.direction, turn_config.value,
							&head);
					sirituke();
					ajast();
					turn(turn_config);
					chenge_head(turn_config.direction, turn_config.value,
							&head);
					sirituke();
					ajast();
				} else if (((((map[posX][posY].wall & 0x0f)
						| (map[posX][posY].wall << 4)) << head) & 0x90)
						== 0x90) {
					turn_config.direction = TURN_R;
					turn(turn_config);
					chenge_head(turn_config.direction, turn_config.value,
							&head);
					sirituke();
					ajast();
					turn(turn_config);
					chenge_head(turn_config.direction, turn_config.value,
							&head);
					sirituke();
					ajast();
				} else if (((((map[posX][posY].wall & 0x0f)
						| (map[posX][posY].wall << 4)) << head) & 0x80)
						== 0x80) {
					turn_u();
					sirituke();
					ajast();
				}
				turn_config.finish_speed = turn_config.initial_speed = 0;
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				if (((((map[posX][posY].wall & 0x0f)
						| (map[posX][posY].wall << 4)) << head) & 0x20)
						== 0x20) {
					motor_stop();
					sirituke();
					ajast();
				}
				RUN_config.finish_speed = speed_buf;
				turn_config.finish_speed = turn_config.initial_speed = 300;
			} else {
				slalom90_config.config.direction = TURN_R;
				slalom(slalom90_config);
				chenge_head(slalom90_config.config.direction,
						slalom90_config.config.value, &head);
				RUN_config.finish_speed = speed_buf;
				chenge_pos(1, &posX, &posY, head);
				wall_set(0x03);
				wall_set_around();
				move_f = 1;
			}

			break;
		case 2:

//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("U\n");
//				osMutexRelease(UART_MutexHandle);
//			}
			if (move_f == 1) {
				RUN_config.initial_speed = get_MotorSpeed();
				RUN_config.finish_speed = 0;
				RUN_config.value = BLOCK_LENGTH * 0.5;
				straight(RUN_config, 1, 0, 1);
			}
			move_f = -1;
			motor_stop();
//			turn_u();
			if (((((map[posX][posY].wall & 0x0f) | (map[posX][posY].wall << 4))
					<< head) & 0xc0) == 0xc0) {
				turn_config.direction = TURN_L;
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
			} else if (((((map[posX][posY].wall & 0x0f)
					| (map[posX][posY].wall << 4)) << head) & 0x90) == 0x90) {
				turn_config.direction = TURN_R;
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				sirituke();
				ajast();
			} else if (((((map[posX][posY].wall & 0x0f)
					| (map[posX][posY].wall << 4)) << head) & 0x80) == 0x80) {
				turn_u();
				sirituke();
				ajast();
			}else{
				turn_u();
			}
			RUN_config.finish_speed = speed_buf;
			break;
		case 3:
//			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
//				printf("L\n");
//				osMutexRelease(UART_MutexHandle);
//			}
			if (move_f == -1) {
				if (((((map[posX][posY].wall & 0x0f)
						| (map[posX][posY].wall << 4)) << head) & 0xc0)
						== 0xc0) {
					turn_config.direction = TURN_L;
					turn(turn_config);
					chenge_head(turn_config.direction, turn_config.value,
							&head);
					sirituke();
					ajast();
					turn(turn_config);
					chenge_head(turn_config.direction, turn_config.value,
							&head);
					sirituke();
					ajast();
				} else if (((((map[posX][posY].wall & 0x0f)
						| (map[posX][posY].wall << 4)) << head) & 0x90)
						== 0x90) {
					turn_config.direction = TURN_R;
					turn(turn_config);
					chenge_head(turn_config.direction, turn_config.value,
							&head);
					sirituke();
					ajast();
					turn(turn_config);
					chenge_head(turn_config.direction, turn_config.value,
							&head);
					sirituke();
					ajast();
				} else if (((((map[posX][posY].wall & 0x0f)
						| (map[posX][posY].wall << 4)) << head) & 0x80)
						== 0x80) {
					turn_u();
					sirituke();
					ajast();
				}
				turn_config.finish_speed = turn_config.initial_speed = 0;
				turn(turn_config);
				chenge_head(turn_config.direction, turn_config.value, &head);
				if (((((map[posX][posY].wall & 0x0f)
						| (map[posX][posY].wall << 4)) << head) & 0x20)
						== 0x20) {
					motor_stop();
					sirituke();
					ajast();
				}
				RUN_config.finish_speed = speed_buf;
				turn_config.finish_speed = turn_config.initial_speed = 300;
			} else {
				slalom90_config.config.direction = TURN_L;
				slalom(slalom90_config);
				chenge_head(slalom90_config.config.direction,
						slalom90_config.config.value, &head);
				RUN_config.finish_speed = speed_buf;
				chenge_pos(1, &posX, &posY, head);
				wall_set(0x03);
				wall_set_around();
				move_f = 1;
			}
		}
	}
}

/*void hidarite(void) {
 RUNConfig RUN_config = { MOVE_FORWARD, 0, 300, 300, 1000, BLOCK_LENGTH };
 RUNConfig turn_config = { TURN_R, 0, 200, 300, 2000, 90 };
 if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
 printf("hidarite\n");
 osMutexRelease(UART_MutexHandle);
 }

 Delay_ms(500);
 //wall_calibration();
 tone(tone_hiC, 10);
 //	osThreadFlagsSet(Sensor_TaskHandle, TASK_START);
 Delay_ms(10);
 for (;;) {

 //print_map();
 if (HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin) == 0) {
 motor_stop();
 //			osThreadFlagsSet(Sensor_TaskHandle, TASK_STOP);
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
 turn_config.initial_speed = (MotorSPEED_L + MotorSPEED_R) / 2;
 turn_config.direction = TURN_L;
 turn(turn_config);
 chenge_head(turn_config.direction, turn_config.value, &head);
 RUN_config.initial_speed = (MotorSPEED_L + MotorSPEED_R) / 2;
 straight(RUN_config);
 chenge_pos(1,&posX,&posY,head);
 } else if (wall_check(0) == 0) {
 RUN_config.initial_speed = (MotorSPEED_L + MotorSPEED_R) / 2;
 //	printf("straight\n");
 straight(RUN_config);
 chenge_pos(1,&posX,&posY,head);
 } else if (wall_check(2) == 0) {
 //	printf("TURN R\n");
 //			RUN_config.value = (BLOCK_LENGTH - NEZUTAKA_LENGTH) / 3;
 //			straight(RUN_config);
 turn_config.initial_speed = (MotorSPEED_L + MotorSPEED_R) / 2;
 turn_config.direction = TURN_R;
 turn(turn_config);
 chenge_head(turn_config.direction, turn_config.value, &head);
 RUN_config.initial_speed = (MotorSPEED_L + MotorSPEED_R) / 2;
 straight(RUN_config);
 chenge_pos(1,&posX,&posY,head);
 } else {
 //	printf("U\n");
 turn_u();
 }

 tone(tone_hiC, 100);
 }
 }*/

/*
 * 説明：最短走行
 * 引数：RUN_config 直進パラメータ
 * 　　　slalom90_config 90°スラロームパラメータ
 * 　　　slalom180_config 180°スラロームパラメータ
 * 　　　gx 目標X座標
 * 　　　gy 目標Y座標
 * 　　　sx スタートX座標
 * 　　　sy スタートY座標
 * 　　　shead スタート時の機体の向き
 * 戻り値：無し
 */
void saitan(RUNConfig RUN_config, SLALOMConfig slalom90_config,
		SLALOMConfig slalom180_config, uint16_t gx, uint16_t gy, uint16_t sx,
		uint16_t sy, int8_t shead) {
	RUTE rute[100] = { 0 };
	uint16_t x = sx, y = sy;
	uint16_t i = 0;
	int8_t temp_head = 0, head_buf = shead;
	uint8_t temp_wall;
<<<<<<< HEAD
	RUNConfig turn_config = { TURN_R, 400, 400, 2000, 700, 90 };
	uint32_t maxspeed_buf = RUN_config.max_speed;
=======
	RUNConfig turn_config = { TURN_R, 0, 0, 300, 300, 90 };
	uint32_t acceleration_buf = RUN_config.acceleration;
>>>>>>> work
//	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
//		printf("saitan\n");
//		osMutexRelease(UART_MutexHandle);
//	}

	make_smap(gx, gy, 1);
//	print_map();

	while (x != gx || y != gy) {
		temp_wall =
				(((((map[x][y].wall & 0x0f) + ((map[x][y].wall & 0x0f) << 4))
						<< head_buf) & 0xf0) >> 4)
						+ ((((map[x][y].wall & 0xf0) + (map[x][y].wall >> 4))
								<< head_buf) & 0xf0);
		temp_head = 4;
//		if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
//			printf("temp_wall=0x%2x,head_buf=%d\n", temp_wall, head_buf);
//			osMutexRelease(UART_MutexHandle);
//		}
		if ((temp_wall & 0x88) == 0x80) {
			switch (head_buf) {
			case 0:
				if (map[x][y].step > map[x][y + 1].step
						&& (map[x][y + 1].wall & 0xf0) == 0xf0) {
					y++;
					temp_head = 0;
				}
				break;
			case 1:
				if (map[x][y].step > map[x + 1][y].step
						&& (map[x + 1][y].wall & 0xf0) == 0xf0) {
					x++;
					temp_head = 0;
				}
				break;
			case 2:
				if (map[x][y].step > map[x][y - 1].step
						&& (map[x][y - 1].wall & 0xf0) == 0xf0) {
					y--;
					temp_head = 0;
				}
				break;
			case 3:
				if (map[x][y].step > map[x - 1][y].step
						&& (map[x - 1][y].wall & 0xf0) == 0xf0) {
					x--;
					temp_head = 0;
				}
				break;
			}
		}
		if ((temp_wall & 0x44) == 0x40 && temp_head == 4) {
			switch (head_buf) {
			case 0:
				if (map[x][y].step > map[x + 1][y].step
						&& (map[x + 1][y].wall & 0xf0) == 0xf0) {
					x++;
					temp_head = 1;
				}
				break;
			case 1:
				if (map[x][y].step > map[x][y - 1].step
						&& (map[x][y - 1].wall & 0xf0) == 0xf0) {
					y--;
					temp_head = 1;
				}
				break;
			case 2:
				if (map[x][y].step > map[x - 1][y].step
						&& (map[x - 1][y].wall & 0xf0) == 0xf0) {
					x--;
					temp_head = 1;
				}
				break;
			case 3:
				if (map[x][y].step > map[x][y + 1].step
						&& (map[x][y + 1].wall & 0xf0) == 0xf0) {
					y++;
					temp_head = 1;
				}
				break;
			}
		}
		if ((temp_wall & 0x11) == 0x10 && temp_head == 4) {
			switch (head_buf) {
			case 0:
				if (map[x][y].step > map[x - 1][y].step
						&& (map[x - 1][y].wall & 0xf0) == 0xf0) {
					x--;
					temp_head = 3;
				}
				break;
			case 1:
				if (map[x][y].step > map[x][y + 1].step
						&& (map[x][y + 1].wall & 0xf0) == 0xf0) {
					y++;
					temp_head = 3;
				}
				break;
			case 2:
				if (map[x][y].step > map[x + 1][y].step
						&& (map[x + 1][y].wall & 0xf0) == 0xf0) {
					x++;
					temp_head = 3;
				}
				break;
			case 3:
				if (map[x][y].step > map[x][y - 1].step
						&& (map[x][y - 1].wall & 0xf0) == 0xf0) {
					y--;
					temp_head = 3;
				}
				break;
			}
		}
		if ((temp_wall & 0x22) == 0x20 && temp_head == 4) {

			switch (head_buf) {
			case 0:
				if (map[x][y].step > map[x][y - 1].step
						&& (map[x][y - 1].wall & 0xf0) == 0xf0) {
					y--;
					temp_head = 2;
				}
				break;
			case 1:
				if (map[x][y].step > map[x - 1][y].step
						&& (map[x - 1][y].wall & 0xf0) == 0xf0) {
					x--;
					temp_head = 2;
				}
				break;
			case 2:
				if (map[x][y].step > map[x][y + 1].step
						&& (map[x][y + 1].wall & 0xf0) == 0xf0) {
					y++;
					temp_head = 2;
				}
				break;
			case 3:
				if (map[x][y].step > map[x + 1][y].step
						&& (map[x + 1][y].wall & 0xf0) == 0xf0) {
					x++;
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
			motor_stop();
			return;
		}

		switch (temp_head) {
		case 0:
//			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
//				printf("S\n");
//				osMutexRelease(UART_MutexHandle);
//			}
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
//			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
//				printf("R\n");
//				osMutexRelease(UART_MutexHandle);
//			}
			rute[i].direction = temp_head;
			rute[i].value = 90;
			chenge_head(TURN_R, 90, &head_buf);
			break;
		case 2:
//			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
//				printf("U\n");
//				osMutexRelease(UART_MutexHandle);
//			}
			rute[i].direction = temp_head;
			rute[i].value = 180;
			chenge_head(TURN_R, 180, &head_buf);
			i++;
			rute[i].direction = 0;
			rute[i].value = BLOCK_LENGTH / 2;

			break;
		case 3:
//			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
//				printf("L\n");
//				osMutexRelease(UART_MutexHandle);
//			}
			rute[i].direction = temp_head;
			rute[i].value = 90;
			chenge_head(TURN_L, 90, &head_buf);
			break;
		}
		i++;
//		if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
//			printf("x=%d y=%d head=%d i=%d\n", x, y, head_buf, i);
//			osMutexRelease(UART_MutexHandle);
//		}
	}
<<<<<<< HEAD

	/*ゴール後中央へ移動*/
	if (rute[i - 1].direction == 0) {
		rute[i - 1].value += BLOCK_LENGTH / 2;
	} else {
		rute[i].direction = 0;
		rute[i].value = BLOCK_LENGTH / 2;
		i++;
	}
=======
>>>>>>> work

	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		for (int f = 0; f < i; f++) {
			printf("%d:direction=%d value=%ld\n", f, rute[f].direction,
					rute[f].value);
		}
		osMutexRelease(UART_MutexHandle);
	}
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		printf("end\n");
		osMutexRelease(UART_MutexHandle);
	}

	osDelay(100);

	for (int f = 0; f < i; f++) {
		switch (rute[f].direction) {
		case 0:
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("S\n");
				osMutexRelease(UART_MutexHandle);
			}
			RUN_config.initial_speed = get_MotorSpeed();
			RUN_config.value = rute[f].value;
			if (rute[f].value <= 180) {
<<<<<<< HEAD
				RUN_config.max_speed = slalom90_config.config.initial_speed;
			} else {
				RUN_config.max_speed = maxspeed_buf;
			}
			if(f==i-1){
				RUN_config.finish_speed = 0;
=======
				RUN_config.acceleration *= 0.8;
>>>>>>> work
			}
			straight(RUN_config, 1, 1, 1);
			if (f == 0) {
				RUN_config.value += 90;
			}
			chenge_pos(RUN_config.value / BLOCK_LENGTH, &posX, &posY, head);
<<<<<<< HEAD
=======
			RUN_config.acceleration = acceleration_buf;
>>>>>>> work
			break;
		case 1:
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("R\n");
				osMutexRelease(UART_MutexHandle);
			}
			if (f == 0) {
				turn_config.direction = TURN_R;
				turn_config.finish_speed = turn_config.initial_speed = 0;
				turn(turn_config);
				if (((((map[posX][posY].wall & 0x0f)
						| (map[posX][posY].wall << 4)) << head) & 0x20)
						== 0x20) {
					sirituke();
					ajast();
				}
				RUN_config.initial_speed = get_MotorSpeed();
				RUN_config.value = BLOCK_LENGTH * 0.5;
				straight(RUN_config, 1, 0, 0);
			} else {
				slalom90_config.config.direction = TURN_R;
				slalom90_config.config.value = rute[f].value;
				slalom(slalom90_config);
			}
			chenge_head(TURN_R, 90, &head);
			chenge_pos(1, &posX, &posY, head);
			break;
		case 2:
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("U\n");
				osMutexRelease(UART_MutexHandle);
			}
			turn_u();
			break;
		case 3:
			if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
				printf("L\n");
				osMutexRelease(UART_MutexHandle);
			}
			if (f == 0) {
				turn_config.direction = TURN_L;
				turn_config.finish_speed = turn_config.initial_speed = 0;
				turn(turn_config);
				if (((((map[posX][posY].wall & 0x0f)
						| (map[posX][posY].wall << 4)) << head) & 0x20)
						== 0x20) {
					sirituke();
					ajast();
				}
				RUN_config.initial_speed = get_MotorSpeed();
				RUN_config.value = BLOCK_LENGTH * 0.5;
				straight(RUN_config, 1, 0, 0);
			} else {
				slalom90_config.config.direction = TURN_L;
				slalom90_config.config.value = rute[f].value;
				slalom(slalom90_config);
			}
			chenge_head(TURN_L, 90, &head);
			chenge_pos(1, &posX, &posY, head);
			break;
		}
	}
<<<<<<< HEAD
//	RUN_config.initial_speed = 300;
//	RUN_config.max_speed = 300;
//	RUN_config.finish_speed = 0;
//	RUN_config.acceleration = 1000;
//	RUN_config.value = BLOCK_LENGTH / 2;
//	straight(RUN_config, 1, 0, 1);
//	chenge_pos(1, &posX, &posY, head);
=======
	RUN_config.initial_speed = 300;
	RUN_config.max_speed = 300;
	RUN_config.finish_speed = 0;
	RUN_config.acceleration = 1000;
	RUN_config.value = BLOCK_LENGTH / 2;
	straight(RUN_config, 1, 0, 1);
	chenge_pos(1, &posX, &posY, head);
>>>>>>> work
	motor_stop();
	posX = gx;
	posY = gy;
	if (((((map[posX][posY].wall & 0x0f) | (map[posX][posY].wall << 4)) << head)
			& 0xc0) == 0xc0) {
		turn_config.direction = TURN_L;
		turn(turn_config);
		chenge_head(turn_config.direction, turn_config.value, &head);
		sirituke();
		ajast();
		turn(turn_config);
		chenge_head(turn_config.direction, turn_config.value, &head);
		sirituke();
		ajast();
	} else if (((((map[posX][posY].wall & 0x0f) | (map[posX][posY].wall << 4))
			<< head) & 0x90) == 0x90) {
		turn_config.direction = TURN_R;
		turn(turn_config);
		chenge_head(turn_config.direction, turn_config.value, &head);
		sirituke();
		ajast();
		turn(turn_config);
		chenge_head(turn_config.direction, turn_config.value, &head);
		sirituke();
		ajast();
	} else if (((((map[posX][posY].wall & 0x0f) | (map[posX][posY].wall << 4))
			<< head) & 0x80) == 0x80) {
		turn_u();
		sirituke();
		ajast();
	} else {
		turn_u();
	}
	music();
}
