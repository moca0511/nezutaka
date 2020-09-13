/*
 * maze.c
 *
 *  Created on: Sep 5, 2020
 *      Author: junmo
 */

#include "maze.h"
#include "sensor.h"
#include "nezutaka.h"
#include "cmsis_os.h"
extern osMutexId_t UART_MutexHandle;
MAP map[MAP_SIZE]; //ｓマップ情報
int16_t posX = 0, posY = 0;	//　現在の位置
uint8_t head = 0;	//　現在向いている方向(北東南西(0,1,2,3))
uint8_t game_mode = 0;	//　探索(0)・最短(1)　選択
extern SensorData sensorData;
extern uint32_t wall_config[12];

void print_map(void) {
	/*\	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
	 printf("posX=%d,posY=%d,head=%d\n\n", posX, posY, head);
	 printf("    ");
	 for (int i = 0; i < MAP_X_MAX; i++) {
	 printf("  %d%d", i / 10, i % 10);
	 }
	 printf("\n");
	 for (int i = 0; i <= MAP_X_MAX; i++) {
	 printf("----");
	 }
	 printf("\n");
	 for (int i = MAP_SIZE - MAP_X_MAX; i >= 0; i -= MAP_X_MAX) {
	 printf("%3d|", i / MAP_X_MAX);
	 for (int f = 0; f < MAP_X_MAX; f++) {
	 printf("%4d", map[i + f].step);
	 }
	 printf("\n");
	 }
	 osMutexRelease(UART_MutexHandle);
	 }*/
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		printf("\n");
		printf("    ");
		for (int i = 0; i < MAP_X_MAX; i++) {
			printf("  %d%d", i / 10, i % 10);
		}
		printf("\n");
		for (int i = 0; i <= MAP_X_MAX; i++) {
			printf("----");
		}
		printf("\n");
		for (int i = MAP_SIZE - MAP_X_MAX; i >= 0; i -= MAP_X_MAX) {
			printf("%3d|", i / MAP_X_MAX);
			for (int f = 0; f < MAP_X_MAX; f++) {
				printf("%4x", map[i + f].wall);
			}
			printf("\n");
		}

		printf("\n");
		printf("    ");
		for (int i = 0; i < MAP_X_MAX; i++) {
			printf("  %d%d", i / 10, i % 10);
		}
		printf("\n");
		for (int i = 0; i <= MAP_X_MAX; i++) {
			printf("----");
		}
		printf("\n");
		for (int i = MAP_SIZE - MAP_X_MAX; i >= 0; i -= MAP_X_MAX) {
			printf("%3d|", i / MAP_X_MAX);
			for (int f = 0; f < MAP_X_MAX; f++) {
				if (i == posY * MAP_X_MAX && f == posX) {
					switch (head) {
					case 0:
						printf(" ^^ ");
						break;
					case 1:
						printf(" -> ");
						break;
					case 2:
						printf(" vv ");
						break;
					case 3:
						printf(" <- ");
						break;
					}
				} else {
					printf("%4d", map[i + f].step);
				}
			}
			printf("\n");
		}
		osMutexRelease(UART_MutexHandle);
	}
}

void smap_Init(void) {
	/*
	 マップ設定
	 */
	posX = startX;
	posY = startY;
	head = 0;
	for (int i = 0; i < MAP_SIZE; i++) {
		map[i].check = 0;
		map[i].step = 255;
		map[i].wall = 0x00;
	}

	game_mode = 0;
	for (int i = 0; i < MAP_SIZE; i += MAP_X_MAX) {
		map[i].wall += 0x11;
	}
	for (int i = 0; i < MAP_X_MAX; i++) {
		map[i].wall += 0x22;
	}
	for (int i = MAP_SIZE - 1; i >= MAP_SIZE - MAP_X_MAX; i--) {
		map[i].wall += 0x88;
	}
	for (int i = MAP_X_MAX - 1; i < MAP_SIZE; i += MAP_X_MAX) {
		map[i].wall += 0x44;
	}
	map[start].wall += 0x44;
	map[start].wall += 0x80;
	map[start + 1].wall += 0x11;
	map[start + MAP_X_MAX].wall += 0x20;
}

void make_smap(uint16_t gx, uint16_t gy, uint8_t mode) {
	int16_t value = 0;
	int16_t buf1[MAP_SIZE];
	int16_t cnt1 = 0;
	int16_t buf2[MAP_SIZE];
	int16_t cnt2 = 0;
	int16_t pos;

	//	 ｓ等高線マップを初期化する。
	for (int i = 0; i < MAP_SIZE; i++) {
		map[i].step = 255;  //ｓ値未設定を表す値を設定する。
	}

	/* ｓ目標区画に値を設定する。 */
	map[buf1[cnt1++] = gx + gy * MAP_X_MAX].step = value++;

	if (mode == 1) {
		while (1) {
			do {
				/* ｓバッファから値設定済み区画の座標を1つ取り出す。 */
				pos = buf1[--cnt1];
				//printf("\n\n**buf1 pos:%d\n\n", pos);

				/* ｓ隣接区画に値を設定する。 */
				/* ｓ北。 */
				if (pos + MAP_X_MAX < MAP_SIZE) {
					if ((map[pos].wall & 0x88) == 0x80
							&& map[pos + MAP_X_MAX].step == 255) {
						map[buf2[cnt2++] = pos + MAP_X_MAX].step = value;
						//printf("kita map[%d]=%d\n", pos + MAP_X_MAX, value);
					}
				}
				/* ｓ東。 */
				if ((pos + 1) % MAP_X_MAX != 0) {
					if ((map[pos].wall & 0x44) == 0x40
							&& map[pos + 1].step == 255) {
						map[buf2[cnt2++] = pos + 1].step = value;
						//printf("higasi map[%d]=%d\n", pos + 1, value);
					}
				}
				/* ｓ南。 */
				if (pos - MAP_X_MAX >= 0) {
					if ((map[pos].wall & 0x22) == 0x20
							&& map[pos - MAP_X_MAX].step == 255) {
						map[buf2[cnt2++] = pos - MAP_X_MAX].step = value;
						//printf("minami map[%d]=%d\n", pos - MAP_X_MAX, value);
					}
				}
				/* ｓ西。 */
				if (pos % MAP_X_MAX != 0) {
					if ((map[pos].wall & 0x11) == 0x10
							&& map[pos - 1].step == 255) {
						map[buf2[cnt2++] = pos - 1].step = value;
						//printf("nisi map[%d]=%d\n", pos - 1, value);
					}
				}

				//printf("\nbuf1\n");
				for (int i = cnt1 - 1; i >= 0; i--) {
					//printf("[%d]:%d", i, buf1[cnt1]);
				}
				//printf("\nbuf2\n");
				for (int i = cnt2 - 1; i >= 0; i--) {
					//printf("[%d]:%d", i, buf2[cnt2]);
				}
			} while (cnt1 != 0); /* ｓバッファが空になるまで繰り返す。 */

			/* ｓもう一方のバッファが空なら終了する。 */
			if (cnt2 == 0) {
				break;
			}

			/* ｓ設定する値を更新する。 */
			++value;

			do {
				/* ｓバッファから値設定済み区画の座標を1つ取り出す。 */
				pos = buf2[--cnt2];
				//printf("\n\n**buf2 pos:%d\n\n", pos);
				/* ｓ隣接区画に値を設定する。 */
				/* ｓ北。 */
				if (pos + MAP_X_MAX < MAP_SIZE) {
					if ((map[pos].wall & 0x88) == 0x80
							&& map[pos + MAP_X_MAX].step == 255) {
						map[buf1[cnt1++] = pos + MAP_X_MAX].step = value;
						//printf("kita map[%d]=%d\n", pos + MAP_X_MAX, value);
					}
				}
				/* ｓ東。 */
				if ((pos + 1) % MAP_X_MAX != 0) {
					if ((map[pos].wall & 0x44) == 0x40
							&& map[pos + 1].step == 255) {
						map[buf1[cnt1++] = pos + 1].step = value;
						//printf("higasi map[%d]=%d\n", pos + 1, value);
					}
				}
				/* ｓ南。 */
				if (pos - MAP_X_MAX >= 0) {
					if ((map[pos].wall & 0x22) == 0x20
							&& map[pos - MAP_X_MAX].step == 255) {
						map[buf1[cnt1++] = pos - MAP_X_MAX].step = value;
						//printf("minami map[%d]=%d\n", pos - MAP_X_MAX, value);
					}
				}
				/* ｓ西。 */
				if (pos % MAP_X_MAX != 0) {
					if ((map[pos].wall & 0x11) == 0x10
							&& map[pos - 1].step == 255) {
						map[buf1[cnt1++] = pos - 1].step = value;
						//printf("nisi map[%d]=%d\n", pos - 1, value);
					}
				}
				//printf("\nbuf1\n");
				for (int i = cnt1 - 1; i >= 0; i--) {
					//printf("[%d]:%d", i, buf1[cnt1]);
				}
				//printf("\nbuf2\n");
				for (int i = cnt2 - 1; i >= 0; i--) {
					//printf("[%d]:%d", i, buf2[cnt2]);
				}
			} while (cnt2 != 0); /* ｓバッファが空になるまで繰り返す。 */

			/* ｓもう一方のバッファが空なら終了する。 */
			if (cnt1 == 0) {
				break;
			}

			/* ｓ設定する値を更新する。 */
			++value;
		}
	} else {
		while (1) {
			do {
				/* ｓバッファから値設定済み区画の座標を1つ取り出す。 */
				pos = buf1[--cnt1];
				//printf("\n\n**buf1 pos:%d\n\n", pos);
				/* ｓ隣接区画に値を設定する。 */
				/* ｓ北。 */
				if (pos + MAP_X_MAX < MAP_SIZE) {
					if ((map[pos].wall & 0x08) == 0x00
							&& map[pos + MAP_X_MAX].step == 255) {
						map[buf2[cnt2++] = pos + MAP_X_MAX].step = value;
						//printf("kita map[%d]=%d\n", pos + MAP_X_MAX, value);
					}
				}
				/* ｓ東。 */
				if ((pos + 1) % MAP_X_MAX != 0) {
					if ((map[pos].wall & 0x04) == 0x00
							&& map[pos + 1].step == 255) {
						map[buf2[cnt2++] = pos + 1].step = value;
						//printf("higasi map[%d]=%d\n", pos + 1, value);
					}
				}
				/* ｓ南。 */
				if (pos - MAP_X_MAX >= 0) {
					if ((map[pos].wall & 0x02) == 0x00
							&& map[pos - MAP_X_MAX].step == 255) {
						map[buf2[cnt2++] = pos - MAP_X_MAX].step = value;
						//printf("minami map[%d]=%d\n", pos - MAP_X_MAX, value);
					}
				}
				/* ｓ西。 */
				if (pos % MAP_X_MAX != 0) {
					if ((map[pos].wall & 0x01) == 0x00
							&& map[pos - 1].step == 255) {
						map[buf2[cnt2++] = pos - 1].step = value;
						//printf("nisi map[%d]=%d\n", pos - 1, value);
					}
				}
				//printf("\nbuf1\n");
				for (int i = cnt1 - 1; i >= 0; i--) {
					//printf("[%d]:%d", i, buf1[cnt1]);
				}
				//printf("\nbuf2\n");
				for (int i = cnt2 - 1; i >= 0; i--) {
					//printf("[%d]:%d", i, buf2[cnt2]);
				}
			} while (cnt1 != 0); /* ｓバッファが空になるまで繰り返す。 */

			/* ｓもう一方のバッファが空なら終了する。 */
			if (cnt2 == 0) {
				break;
			}

			/* ｓ設定する値を更新する。 */
			++value;

			do {
				/* ｓバッファから値設定済み区画の座標を1つ取り出す。 */
				pos = buf2[--cnt2];
				//printf("\n\n**buf2 pos:%d\n\n", pos);
				/* ｓ隣接区画に値を設定する。 */
				/* ｓ北。 */
				if (pos + MAP_X_MAX < MAP_SIZE) {
					if ((map[pos].wall & 0x08) == 0x00
							&& map[pos + MAP_X_MAX].step == 255) {
						map[buf1[cnt1++] = pos + MAP_X_MAX].step = value;
						//printf("kita map[%d]=%d\n", pos + MAP_X_MAX, value);
					}
				}
				/* ｓ東。 */
				if ((pos + 1) % MAP_X_MAX != 0) {
					if ((map[pos].wall & 0x04) == 0x00
							&& map[pos + 1].step == 255) {
						map[buf1[cnt1++] = pos + 1].step = value;
						//printf("higasi map[%d]=%d\n", pos + 1, value);
					}
				}
				/* ｓ南。 */
				if (pos - MAP_X_MAX >= 0) {
					if ((map[pos].wall & 0x02) == 0x00
							&& map[pos - MAP_X_MAX].step == 255) {
						map[buf1[cnt1++] = pos - MAP_X_MAX].step = value;
						//printf("minami map[%d]=%d\n", pos - MAP_X_MAX, value);
					}
				}
				/* ｓ西。 */
				if (pos % MAP_X_MAX != 0) {
					if ((map[pos].wall & 0x01) == 0x00
							&& map[pos - 1].step == 255) {
						map[buf1[cnt1++] = pos - 1].step = value;
						//printf("nisi map[%d]=%d\n", pos - 1, value);
					}
				}
				//printf("\nbuf1\n");
				for (int i = cnt1 - 1; i >= 0; i--) {
					//printf("[%d]:%d", i, buf1[cnt1]);
				}
				//printf("\nbuf2\n");
				for (int i = cnt2 - 1; i >= 0; i--) {
					//printf("[%d]:%d", i, buf2[cnt2]);
				}
			} while (cnt2 != 0); /* あ　バッファが空になるまで繰り返す。 */

			/* 　ｓもう一方のバッファが空なら終了する。 */
			if (cnt1 == 0) {
				break;
			}

			/* 　あ設定する値を更新する。 */
			++value;
		}
	}
	//printf("make fin\n");
}

void wall_set(void) {

	//printf("wallset Y=%d,X=%d\n", posY, posX);
	uint8_t wall_info = 0;

	//北壁判定

	if (sensorData.ADC_DATA_LF >= wall_config[LF_threshold]
			|| sensorData.ADC_DATA_RF >= wall_config[RF_threshold]) {
		wall_info += 0x88;
	}
	if (sensorData.ADC_DATA_LS >= wall_config[LS_threshold]) {
		wall_info += 0x11;
	}
	if (sensorData.ADC_DATA_RS >= wall_config[RS_threshold]) {
		wall_info += 0x44;
	}
	wall_info >>= head;
	wall_info &= 0x0f;
	wall_info |= (wall_info << 4);
	wall_info |= (((0xdd >> head) & 0x0f) << 4);
//
//	printf("wall_info=%2x,wall=%2x\n", wall_info,
//			map[posX + posY * MAP_X_MAX].wall);
	map[posX + posY * MAP_X_MAX].wall |= wall_info;

	if ((map[posX + posY * MAP_X_MAX].wall & 0x88) == 0x88) {
		//	printf("N");
		if (posY < MAP_X_MAX - 1) {
			if ((map[posX + (posY + 1) * MAP_X_MAX].wall & 0x20) == 0x00) {
				map[posX + (posY + 1) * MAP_X_MAX].wall += 0x22;
			}
		}
	} else {
		if (posY < MAP_X_MAX - 1) {
			if ((map[posX + (posY + 1) * MAP_X_MAX].wall & 0x20) == 0x00) {
				map[posX + (posY + 1) * MAP_X_MAX].wall += 0x20;
			}
		}
	}

	if ((map[posX + posY * MAP_X_MAX].wall & 0x44) == 0x44) {
		//	printf("S");
		if (posX + 1 < MAP_X_MAX) {
			if ((map[posX + posY * MAP_X_MAX + 1].wall & 0x10) == 0x00) {
				map[posX + posY * MAP_X_MAX + 1].wall += 0x11;
			}
		}
	} else {
		if (posX + 1 < MAP_X_MAX) {
			if ((map[posX + posY * MAP_X_MAX + 1].wall & 0x10) == 0x00) {
				map[posX + posY * MAP_X_MAX + 1].wall += 0x10;
			}
		}
	}

	if ((map[posX + posY * MAP_X_MAX].wall & 0x22) == 0x22) {
		//	printf("N");
		if (posY > 0) {
			if ((map[posX + (posY - 1) * MAP_X_MAX].wall & 0x80) == 0x00) {
				map[posX + (posY - 1) * MAP_X_MAX].wall += 0x88;
			}
		}
	} else {
		if (posY > 0) {
			if ((map[posX + (posY - 1) * MAP_X_MAX].wall & 0x80) == 0x00) {
				map[posX + (posY - 1) * MAP_X_MAX].wall += 0x80;
			}
		}
	}

	if ((map[posX + posY * MAP_X_MAX].wall & 0x11) == 0x11) {
		//	printf("S");
		if (posX > 0) {
			if ((map[posX + posY * MAP_X_MAX - 1].wall & 0x40) == 0x00) {
				map[posX + posY * MAP_X_MAX - 1].wall += 0x44;
			}
		}
	} else {
		if (posX > 0) {
			if ((map[posX + posY * MAP_X_MAX - 1].wall & 0x40) == 0x00) {
				map[posX + posY * MAP_X_MAX - 1].wall += 0x40;
			}
		}
	}
}

uint8_t wall_check(uint8_t direction) {
	uint8_t and = 0x00;
	switch (direction) {
	case 0:	//前方
		switch (head) {
		case 0:
			and = 0x88;
			break;
		case 1:
			and = 0x44;
			break;
		case 2:
			and = 0x22;
			break;
		case 3:
			and = 0x11;
			break;

		}
		break;
	case 1:	//左方
		switch (head) {
		case 0:
			and = 0x11;
			break;
		case 1:
			and = 0x88;
			break;
		case 2:
			and = 0x44;
			break;
		case 3:
			and = 0x22;
			break;

		}
		break;
	case 2:	//右方
		switch (head) {
		case 0:
			and = 0x44;
			break;
		case 1:
			and = 0x22;
			break;
		case 2:
			and = 0x11;
			break;
		case 3:
			and = 0x88;
			break;
		}
		break;
	}
	if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
		printf("and:%4x,wall:%4x,wall_check:%4x\n", and,
				map[posX + posY * MAP_X_MAX].wall,
				map[posX + posY * MAP_X_MAX].wall & and);
		osMutexRelease(UART_MutexHandle);
	}

	return (map[posX + posY * MAP_X_MAX].wall & and) % 0x10;
}

int16_t step_check(uint16_t posX, uint16_t posY, uint8_t direction) {
	int16_t step = -1;
	switch (direction) {
	case 0:
		switch (head) {
		case 0:
			if (posY + 1 < MAP_Y_MAX) {
				step = map[posX + (posY + 1) * MAP_X_MAX].step
						- map[posX + posY * MAP_X_MAX].step;
			}
			break;
		case 1:
			if (posX + 1 < MAP_X_MAX) {
				step = map[posX + posY * MAP_X_MAX + 1].step
						- map[posX + posY * MAP_X_MAX].step;
			}
			break;
		case 2:
			if (posY > 0) {
				step = map[posX + (posY - 1) * MAP_X_MAX].step
						- map[posX + posY * MAP_X_MAX].step;
			}
			break;
		case 3:
			if (posX > 0) {
				step = map[posX + posY * MAP_X_MAX + 1].step
						- map[posX + posY * MAP_X_MAX].step;
			}
			break;
		}
		break;
	case 1:
		switch (head) {
		case 0:
			if (posX > 0) {
				step = map[posX + posY * MAP_X_MAX].step
						- map[posX + posY * MAP_X_MAX - 1].step;
			}
			break;
		case 1:
			if (posY + 1 < MAP_Y_MAX) {
				step = map[posX + posY * MAP_X_MAX].step
						- map[posX + (posY + 1) * MAP_X_MAX].step;
			}
			break;
		case 2:
			if (posX + 1 < MAP_X_MAX) {
				step = map[posX + posY * MAP_X_MAX].step
						- map[posX + posY * MAP_X_MAX + 1].step;
			}
			break;
		case 3:
			if (posY > 0) {
				step = map[posX + posY * MAP_X_MAX].step
						- map[posX + (posY - 1) * MAP_X_MAX].step;
			}
			break;
		}
		break;
	case 2:
		switch (head) {
		case 0:
			if (posX + 1 < MAP_X_MAX) {
				step = map[posX + posY * MAP_X_MAX].step
						- map[posX + posY * MAP_X_MAX + 1].step;
			}
			break;
		case 1:
			if (posY > 0) {
				step = map[posX + posY * MAP_X_MAX].step
						- map[posX + (posY - 1) * MAP_X_MAX].step;
			}
			break;
		case 2:
			if (posX > 0) {
				step = map[posX + posY * MAP_X_MAX].step
						- map[posX + posY * MAP_X_MAX - 1].step;
			}
			break;
		case 3:
			if (posY + 1 < MAP_Y_MAX) {
				step = map[posX + posY * MAP_X_MAX].step
						- map[posX + (posY + 1) * MAP_X_MAX].step;
			}
			break;
		}
		break;
	}
	return step;
}

