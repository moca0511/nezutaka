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
#include "UI.h"
extern osMutexId_t UART_MutexHandle;
MAP map[MAP_X_MAX][MAP_Y_MAX]; //ｓマップ情報
int16_t posX = 0, posY = 0;	//　現在の位置
int8_t head = 0;	//　現在向いている方向(北東南西(0,1,2,3))
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
		for (int i = MAP_Y_MAX-1; i >= 0; i--) {
			printf("%3d|", i);
			for (int f = 0; f < MAP_X_MAX; f++) {
				printf("%4x", map[f][i].wall);
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
		for (int i = MAP_Y_MAX-1; i >= 0; i--) {
			printf("%3d|", i);
			for (int f = 0; f < MAP_X_MAX; f++) {
				if (i == posY && f == posX) {
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
					printf("%4d", map[f][i].step);
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

	for (int i = 0; i < MAP_Y_MAX; i++) {
		for (int f = 0; f < MAP_X_MAX; f++) {
			map[f][i].step = 255;
			map[f][i].wall = 0x00;
		}
	}

	game_mode = 0;
	for (int i = 0; i < MAP_Y_MAX; i++) {
		map[0][i].wall += 0x11;
		map[MAP_X_MAX - 1][i].wall += 0x44;
	}
	for (int i = 0; i < MAP_X_MAX; i++) {
		map[i][0].wall += 0x22;
		map[i][MAP_Y_MAX - 1].wall += 0x88;
	}

	map[startX][startY].wall |= 0x44;
	map[startX][startY].wall |= 0x80;
	map[startX + 1][startY].wall |= 0x11;
	map[startX][startY + 1].wall |= 0x20;
}

void make_smap(uint16_t gx, uint16_t gy, uint8_t mode) {
	int16_t value = 0;
	int16_t buf1[2][MAP_SIZE];
	int16_t cnt1 = 0;
	int16_t buf2[2][MAP_SIZE];
	int16_t cnt2 = 0;
	int16_t posX, posY;

	//	 ｓ等高線マップを初期化する。
	for (int i = 0; i < MAP_Y_MAX; i++) {
		for (int f = 0; f < MAP_X_MAX; f++) {
			map[f][i].step = 255;
		}
	}

	/* ｓ目標区画に値を設定する。 */
	map[buf1[0][cnt1] = gx][buf1[1][cnt1] = gy].step = value++;
	cnt1++;
	if (mode == 1) {
		while (1) {
			do {
				/* ｓバッファから値設定済み区画の座標を1つ取り出す。 */
				posX = buf1[0][--cnt1];
				posY = buf1[1][cnt1];
				//printf("\n\n**buf1 pos:%d\n\n", pos);

				/* ｓ隣接区画に値を設定する。 */
				/* ｓ北。 */
				if (posY + 1 < MAP_Y_MAX) {
					if ((map[posX][posY].wall & 0x88) == 0x80
							&& map[posX][posY + 1].step == 255) {
						map[buf2[0][cnt2] = posX][buf2[1][cnt2] = posY + 1].step =
								value;
						cnt2++;
						//printf("kita map[%d]=%d\n", pos + MAP_X_MAX, value);
					}
				}
				/* ｓ東。 */
				if (posX + 1 < MAP_X_MAX) {
					if ((map[posX][posY].wall & 0x44) == 0x40
							&& map[posX + 1][posY].step == 255) {
						map[buf2[0][cnt2] = posX + 1][buf2[1][cnt2] = posY].step =
								value;
						cnt2++;
						//printf("higasi map[%d]=%d\n", pos + 1, value);
					}
				}
				/* ｓ南。 */
				if (posY != 0) {
					if ((map[posX][posY].wall & 0x22) == 0x20
							&& map[posX][posY - 1].step == 255) {
						map[buf2[0][cnt2] = posX][buf2[1][cnt2] = posY - 1].step =
								value;
						cnt2++;
						//printf("minami map[%d]=%d\n", pos - MAP_X_MAX, value);
					}
				}
				/* ｓ西。 */
				if (posX != 0) {
					if ((map[posX][posY].wall & 0x11) == 0x10
							&& map[posX - 1][posY].step == 255) {
						map[buf2[0][cnt2] = posX - 1][buf2[1][cnt2] = posY].step =
								value;
						cnt2++;
						//printf("nisi map[%d]=%d\n", pos - 1, value);
					}
				}

			} while (cnt1 != 0); /* ｓバッファが空になるまで繰り返す。 */

			/* ｓもう一方のバッファが空なら終了する。 */
			if (cnt2 == 0) {
				break;
			}

			/* ｓ設定する値を更新する。 */
			++value;
			/*if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			 printf("osThreadYield()=%d\n", osThreadYield());
			 osMutexRelease(UART_MutexHandle);
			 }*/

			do {
				/* ｓバッファから値設定済み区画の座標を1つ取り出す。 */
				posX = buf2[0][--cnt2];
				posY = buf2[1][cnt2];
				//printf("\n\n**buf1 pos:%d\n\n", pos);

				/* ｓ隣接区画に値を設定する。 */
				/* ｓ北。 */
				if (posY + 1 < MAP_Y_MAX) {
					if ((map[posX][posY].wall & 0x08) == 0x00
							&& map[posX][posY + 1].step == 255) {
						map[buf1[0][cnt1] = posX][buf1[1][cnt1] = posY + 1].step =
								value;
						cnt1++;
						//printf("kita map[%d]=%d\n", pos + MAP_X_MAX, value);
					}
				}
				/* ｓ東。 */
				if (posX + 1 < MAP_X_MAX) {
					if ((map[posX][posY].wall & 0x04) == 0x00
							&& map[posX + 1][posY].step == 255) {
						map[buf1[0][cnt1] = posX + 1][buf1[1][cnt1] = posY].step =
								value;
						cnt1++;
						//printf("higasi map[%d]=%d\n", pos + 1, value);
					}
				}
				/* ｓ南。 */
				if (posY != 0) {
					if ((map[posX][posY].wall & 0x02) == 0x00
							&& map[posX][posY - 1].step == 255) {
						map[buf1[0][cnt1] = posX][buf1[1][cnt1] = posY - 1].step =
								value;
						cnt1++;
						//printf("minami map[%d]=%d\n", pos - MAP_X_MAX, value);
					}
				}
				/* ｓ西。 */
				if (posX != 0) {
					if ((map[posX][posY].wall & 0x01) == 0x00
							&& map[posX - 1][posY].step == 255) {
						map[buf1[0][cnt1] = posX - 1][buf1[1][cnt1] = posY].step =
								value;
						cnt1++;
						//printf("nisi map[%d]=%d\n", pos - 1, value);
					}
				}
			} while (cnt2 != 0); /* ｓバッファが空になるまで繰り返す。 */

			/* ｓもう一方のバッファが空なら終了する。 */
			if (cnt1 == 0) {
				break;
			}

			/* ｓ設定する値を更新する。 */
			++value;
			/*if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			 printf("osThreadYield()=%d\n", osThreadYield());
			 osMutexRelease(UART_MutexHandle);
			 }*/
		}
	} else {
		while (1) {
			do {
				/* ｓバッファから値設定済み区画の座標を1つ取り出す。 */
				posX = buf1[0][--cnt1];
				posY = buf1[1][cnt1];
				//printf("\n\n**buf1 pos:%d\n\n", pos);

				/* ｓ隣接区画に値を設定する。 */
				/* ｓ北。 */
				if (posY + 1 < MAP_Y_MAX) {
					if ((map[posX][posY].wall & 0x08) == 0x00
							&& map[posX][posY + 1].step == 255) {
						map[buf2[0][cnt2] = posX][buf2[1][cnt2] = posY + 1].step =
								value;
						cnt2++;
						//printf("kita map[%d]=%d\n", pos + MAP_X_MAX, value);
					}
				}
				/* ｓ東。 */
				if (posX + 1 < MAP_X_MAX) {
					if ((map[posX][posY].wall & 0x04) == 0x00
							&& map[posX + 1][posY].step == 255) {
						map[buf2[0][cnt2] = posX + 1][buf2[1][cnt2] = posY].step =
								value;
						cnt2++;
						//printf("higasi map[%d]=%d\n", pos + 1, value);
					}
				}
				/* ｓ南。 */
				if (posY != 0) {
					if ((map[posX][posY].wall & 0x02) == 0x00
							&& map[posX][posY - 1].step == 255) {
						map[buf2[0][cnt2] = posX][buf2[1][cnt2] = posY - 1].step =
								value;
						cnt2++;
						//printf("minami map[%d]=%d\n", pos - MAP_X_MAX, value);
					}
				}
				/* ｓ西。 */
				if (posX != 0) {
					if ((map[posX][posY].wall & 0x01) == 0x00
							&& map[posX - 1][posY].step == 255) {
						map[buf2[0][cnt2] = posX - 1][buf2[1][cnt2] = posY].step =
								value;
						cnt2++;
						//printf("nisi map[%d]=%d\n", pos - 1, value);
					}
				}

			} while (cnt1 != 0); /* ｓバッファが空になるまで繰り返す。 */

			/* ｓもう一方のバッファが空なら終了する。 */
			if (cnt2 == 0) {
				break;
			}

			/* ｓ設定する値を更新する。 */
			++value;
			/*if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			 printf("osThreadYield()=%d\n", osThreadYield());
			 osMutexRelease(UART_MutexHandle);
			 }*/

			do {
				/* ｓバッファから値設定済み区画の座標を1つ取り出す。 */
				posX = buf2[0][--cnt2];
				posY = buf2[1][cnt2];
				//printf("\n\n**buf1 pos:%d\n\n", pos);

				/* ｓ隣接区画に値を設定する。 */
				/* ｓ北。 */
				if (posY + 1 < MAP_Y_MAX) {
					if ((map[posX][posY].wall & 0x08) == 0x00
							&& map[posX][posY + 1].step == 255) {
						map[buf1[0][cnt1] = posX][buf1[1][cnt1] = posY + 1].step =
								value;
						cnt1++;
						//printf("kita map[%d]=%d\n", pos + MAP_X_MAX, value);
					}
				}
				/* ｓ東。 */
				if (posX + 1 < MAP_X_MAX) {
					if ((map[posX][posY].wall & 0x04) == 0x00
							&& map[posX + 1][posY].step == 255) {
						map[buf1[0][cnt1] = posX + 1][buf1[1][cnt1] = posY].step =
								value;
						cnt1++;
						//printf("higasi map[%d]=%d\n", pos + 1, value);
					}
				}
				/* ｓ南。 */
				if (posY != 0) {
					if ((map[posX][posY].wall & 0x02) == 0x00
							&& map[posX][posY - 1].step == 255) {
						map[buf1[0][cnt1] = posX][buf1[1][cnt1] = posY - 1].step =
								value;
						cnt1++;
						//printf("minami map[%d]=%d\n", pos - MAP_X_MAX, value);
					}
				}
				/* ｓ西。 */
				if (posX != 0) {
					if ((map[posX][posY].wall & 0x01) == 0x00
							&& map[posX - 1][posY].step == 255) {
						map[buf1[0][cnt1] = posX - 1][buf1[1][cnt1] = posY].step =
								value;
						cnt1++;
						//printf("nisi map[%d]=%d\n", pos - 1, value);
					}
				}
			} while (cnt2 != 0); /* ｓバッファが空になるまで繰り返す。 */

			/* 　ｓもう一方のバッファが空なら終了する。 */
			if (cnt1 == 0) {
				break;
			}

			/* 　あ設定する値を更新する。 */
			++value;
			/*if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
			 printf("osThreadYield()=%d\n", osThreadYield());
			 osMutexRelease(UART_MutexHandle);
			 }*/
		}
	}
	//printf("make fin\n");
}

void wall_set(uint8_t mode) {

	//printf("wallset Y=%d,X=%d\n", posY, posX);
	uint8_t wall_info = 0;
	SensorData sensor_buf;

	if ((mode & 0x01) == 0x01) {
		sensor_buf.ADC_DATA_LF = read_wall(&sensorData.ADC_DATA_LF);
		sensor_buf.ADC_DATA_RF = read_wall(&sensorData.ADC_DATA_RF);
		if (sensor_buf.ADC_DATA_LF >= wall_config[LF_threshold]
				&& sensor_buf.ADC_DATA_RF >= wall_config[RF_threshold]) {
			wall_info += 0x88;
		}
	}
	if ((mode & 0x02) == 0x02) {
		sensor_buf.ADC_DATA_LS = read_wall(&sensorData.ADC_DATA_LS);
		sensor_buf.ADC_DATA_RS = read_wall(&sensorData.ADC_DATA_RS);
		if (sensor_buf.ADC_DATA_LS >= wall_config[LS_threshold]) {
			wall_info += 0x11;
		}
		if (sensor_buf.ADC_DATA_RS >= wall_config[RS_threshold]) {
			wall_info += 0x44;
		}
	}
	wall_info >>= head;
	wall_info &= 0x0f;
	wall_info |= (wall_info << 4);

	if (mode == 0x02) {
		wall_info |= (((0x55 >> head) & 0x0f) << 4);
	} else if (mode == 0x01) {
		wall_info |= (((0x88 >> head) & 0x0f) << 4);
	} else if (mode == 0x03) {
		wall_info |= (((0xdd >> head) & 0x0f) << 4);
	}

	/*if (osMutexWait(UART_MutexHandle, 0U) == osOK) {
	 printf("wall:0x%2x | info:0x%2x = %2x \n",
	 map[posX][posY].wall, wall_info,
	 map[posX][posY].wall | wall_info);
	 osMutexRelease(UART_MutexHandle);
	 }*/

	map[posX][posY].wall |= wall_info;

	UILED_SET(map[posX][posY].wall & 0x0f);

//	if ((map[posX][posY].wall & 0x88) == 0x88) {
//		//	printf("N");
//		if (posY < MAP_Y_MAX - 1) {
//			if ((map[posX][posY + 1].wall & 0x20) == 0x00) {
//				map[posX][posY + 1].wall += 0x22;
//			}
//		}
//	} else {
//		if (posY < MAP_Y_MAX - 1) {
//			if ((map[posX][posY + 1].wall & 0x20) == 0x00) {
//				map[posX][posY + 1].wall += 0x20;
//			}
//		}
//	}
//
//	if ((map[posX][posY].wall & 0x44) == 0x44) {
//		//	printf("S");
//		if (posX + 1 < MAP_X_MAX) {
//			if ((map[posX + 1][posY].wall & 0x10) == 0x00) {
//				map[posX + 1][posY].wall += 0x11;
//			}
//		}
//	} else {
//		if (posX + 1 < MAP_X_MAX) {
//			if ((map[posX + 1][posY].wall & 0x10) == 0x00) {
//				map[posX + 1][posY].wall += 0x10;
//			}
//		}
//	}
//
//	if ((map[posX][posY].wall & 0x22) == 0x22) {
//		//	printf("N");
//		if (posY > 0) {
//			if ((map[posX][posY - 1].wall & 0x80) == 0x00) {
//				map[posX][posY - 1].wall += 0x88;
//			}
//		}
//	} else {
//		if (posY > 0) {
//			if ((map[posX][posY - 1].wall & 0x80) == 0x00) {
//				map[posX][posY - 1].wall += 0x80;
//			}
//		}
//	}
//
//	if ((map[posX][posY].wall & 0x11) == 0x11) {
//		//	printf("S");
//		if (posX > 0) {
//			if ((map[posX - 1][posY].wall & 0x40) == 0x00) {
//				map[posX - 1][posY].wall += 0x44;
//			}
//		}
//	} else {
//		if (posX > 0) {
//			if ((map[posX - 1][posY].wall & 0x40) == 0x00) {
//				map[posX - 1][posY].wall += 0x40;
//			}
//		}
//	}
}
void wall_set_around(void) {

	/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
	 printf("check around\n");
	 printf("x:%d,y:%d,wall=0x%2x\n", posX, posY,
	 map[posX][posY].wall);
	 osMutexRelease(UART_MutexHandle);
	 }*/

	/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
	 printf("map[posX][posY].wall & 0x88= %2x\n",
	 (map[posX][posY].wall & 0x88));
	 osMutexRelease(UART_MutexHandle);
	 }*/
	if ((map[posX][posY].wall & 0x88) == 0x88) {
		/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		 printf("O\n");
		 osMutexRelease(UART_MutexHandle);
		 }*/
		if (posY < MAP_Y_MAX - 1) {
			map[posX][posY + 1].wall |= 0x22;
			/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
			 printf("ue ON\n");
			 osMutexRelease(UART_MutexHandle);
			 }*/

		}
	} else {
		/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		 printf("X\n");
		 osMutexRelease(UART_MutexHandle);
		 }*/
		if (posY < MAP_Y_MAX - 1) {

			map[posX][posY + 1].wall |= 0x20;
			/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
			 printf("ue NO\n");
			 osMutexRelease(UART_MutexHandle);
			 }*/

		}
	}
	/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
	 printf("map[posX][posY].wall & 0x44= %2x\n",
	 (map[posX][posY].wall & 0x44));
	 osMutexRelease(UART_MutexHandle);
	 }*/
	if ((map[posX][posY].wall & 0x44) == 0x44) {
		/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		 printf("O\n");
		 osMutexRelease(UART_MutexHandle);
		 }*/
		if (posX < MAP_X_MAX - 1) {
			map[posX + 1][posY].wall |= 0x11;
			/*			if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
			 printf("migi ON\n");
			 osMutexRelease(UART_MutexHandle);

			 }*/
		}
	} else {
		/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		 printf("X\n");
		 osMutexRelease(UART_MutexHandle);
		 }*/
		if (posX < MAP_X_MAX - 1) {

			map[posX + 1][posY].wall |= 0x10;

			/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
			 printf("migi NO\n");
			 osMutexRelease(UART_MutexHandle);
			 }*/

		}
	}
	/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
	 printf("map[posX][posY].wall & 0x22= %2x\n",
	 (map[posX][posY].wall & 0x22));
	 osMutexRelease(UART_MutexHandle);
	 }*/
	if ((map[posX][posY].wall & 0x22) == 0x22) {
		/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		 printf("O\n");
		 osMutexRelease(UART_MutexHandle);
		 }*/
		if (posY > 0) {

			map[posX][posY - 1].wall |= 0x88;

			/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
			 printf("sita ON\n");
			 osMutexRelease(UART_MutexHandle);
			 }*/

		}
	} else {
		/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		 printf("X\n");
		 osMutexRelease(UART_MutexHandle);
		 }*/
		if (posY > 0) {

			map[posX][posY - 1].wall |= 0x80;

			/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
			 printf("sita NO\n");
			 osMutexRelease(UART_MutexHandle);
			 }*/

		}
	}
	/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
	 printf("map[posX][posY].wall & 0x11= %2x\n",
	 (map[posX][posY].wall & 0x11));
	 osMutexRelease(UART_MutexHandle);
	 }*/
	if ((map[posX][posY].wall & 0x11) == 0x11) {
		/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		 printf("O\n");
		 osMutexRelease(UART_MutexHandle);
		 }*/
		if (posX > 0) {

			map[posX - 1][posY].wall |= 0x44;

			/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
			 printf("hidari ON\n");
			 osMutexRelease(UART_MutexHandle);
			 }*/

		}
	} else {
		/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		 printf("X\n");
		 osMutexRelease(UART_MutexHandle);
		 }*/
		if (posX > 0) {

			map[posX - 1][posY].wall |= 0x40;

			/*if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
			 printf("hidari NO\n");
			 osMutexRelease(UART_MutexHandle);
			 }*/

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
		printf("and:%4x,wall:%4x,wall_check:%4x\n", and, map[posX][posY].wall,
				map[posX][posY].wall & and);
		osMutexRelease(UART_MutexHandle);
	}

	return (map[posX][posY].wall & and) % 0x10;
}

int16_t step_check(uint16_t posX, uint16_t posY, uint8_t direction) {
	int16_t step = -1;
	switch (direction) {
	case 0:
		switch (head) {
		case 0:
			if (posY + 1 < MAP_Y_MAX) {
				step = map[posX][posY + 1].step - map[posX][posY].step;
			}
			break;
		case 1:
			if (posX + 1 < MAP_X_MAX) {
				step = map[posX + 1][posY].step - map[posX][posY].step;
			}
			break;
		case 2:
			if (posY > 0) {
				step = map[posX][posY - 1].step - map[posX][posY].step;
			}
			break;
		case 3:
			if (posX > 0) {
				step = map[posX + 1][posY].step - map[posX][posY].step;
			}
			break;
		}
		break;
	case 1:
		switch (head) {
		case 0:
			if (posX > 0) {
				step = map[posX][posY].step - map[posX - 1][posY].step;
			}
			break;
		case 1:
			if (posY + 1 < MAP_Y_MAX) {
				step = map[posX][posY].step - map[posX][posY + 1].step;
			}
			break;
		case 2:
			if (posX + 1 < MAP_X_MAX) {
				step = map[posX][posY].step - map[posX + 1][posY].step;
			}
			break;
		case 3:
			if (posY > 0) {
				step = map[posX][posY].step - map[posX][posY - 1].step;
			}
			break;
		}
		break;
	case 2:
		switch (head) {
		case 0:
			if (posX + 1 < MAP_X_MAX) {
				step = map[posX][posY].step - map[posX + 1][posY].step;
			}
			break;
		case 1:
			if (posY > 0) {
				step = map[posX][posY].step - map[posX][posY - 1].step;
			}
			break;
		case 2:
			if (posX > 0) {
				step = map[posX][posY].step - map[posX - 1][posY].step;
			}
			break;
		case 3:
			if (posY + 1 < MAP_Y_MAX) {
				step = map[posX][posY].step - map[posX][posY + 1].step;
			}
			break;
		}
		break;
	}
	return step;
}

void check_searchBlock(uint16_t *searchX, uint16_t *searchY) {
	*searchX = startX;
	*searchY = startY;
	int16_t value = 0;
	int16_t buf1[2][MAP_SIZE];
	int16_t cnt1 = 0;
	int16_t buf2[2][MAP_SIZE];
	int16_t cnt2 = 0;
	int16_t posX, posY;

	/* ｓ目標区画に値を設定する。 */
	map[buf1[0][cnt1] = *searchX][buf1[1][cnt1] = *searchY].step = value++;
	cnt1++;

	while (1) {
		do {
			/* ｓバッファから値設定済み区画の座標を1つ取り出す。 */
			posX = buf1[0][--cnt1];
			posY = buf1[1][cnt1];
			/* ｓ隣接区画に値を設定する。 */
			/* ｓ北。 */
			if (posY + 1 < MAP_SIZE) {
				if (map[posX][posY + 1].step < map[posX][posY].step) {
					if ((map[posX][posY + 1].wall & 0xf0) == 0xf0) {
						buf2[0][cnt2] = posX;
						buf2[1][cnt2++] = posY + 1;
					} else {
						*searchX = posX;
						*searchY = posY + 1;
						return;
					}
				}
			}
			/* ｓ東。 */
			if (posX != MAP_X_MAX - 1) {
				if (map[posX + 1][posY].step < map[posX][posY].step) {
					if ((map[posX + 1][posY].wall & 0xf0) == 0xf0) {
						buf2[0][cnt2] = posX + 1;
						buf2[1][cnt2++] = posY;
					} else {
						*searchX = posX + 1;
						*searchY = posY;
						return;
					}
				}
			}
			/* ｓ南。 */
			if (posY != 0) {
				if (map[posX][posY - 1].step < map[posX][posY].step) {
					if ((map[posX][posY].wall & 0xf0) == 0xf0) {
						buf2[0][cnt2] = posX;
						buf2[1][cnt2++] = posY - 1;
					} else {
						*searchX = posX;
						*searchY = posY-1;
						return;
					}
				}
			}
			/* ｓ西。 */
			if (posX != 0) {
				if (map[posX - 1][posY].step < map[posX][posY].step) {
					if ((map[posX - 1][posY].wall & 0xf0) == 0xf0) {
						buf2[0][cnt2] = posX - 1;
						buf2[1][cnt2++] = posY;
					} else {
						*searchX = posX - 1;
						*searchY = posY;
						return;
					}
				}
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
			posX = buf2[0][--cnt2];
			posY = buf2[1][cnt2];
			/* ｓ隣接区画に値を設定する。 */
			/* ｓ北。 */
			if (posY + 1 < MAP_SIZE) {
				if (map[posX][posY + 1].step < map[posX][posY].step) {
					if ((map[posX][posY + 1].wall & 0xf0) == 0xf0) {
						buf1[0][cnt1] = posX;
						buf1[1][cnt1++] = posY + 1;
					} else {
						*searchX = posX;
						*searchY = posY + 1;
						return;
					}
				}
			}
			/* ｓ東。 */
			if (posX != MAP_X_MAX - 1) {
				if (map[posX + 1][posY].step < map[posX][posY].step) {
					if ((map[posX + 1][posY].wall & 0xf0) == 0xf0) {
						buf1[0][cnt1] = posX + 1;
						buf1[1][cnt1++] = posY;
					} else {
						*searchX = posX + 1;
						*searchY = posY;
						return;
					}
				}
			}
			/* ｓ南。 */
			if (posY != 0) {
				if (map[posX][posY - 1].step < map[posX][posY].step) {
					if ((map[posX][posY].wall & 0xf0) == 0xf0) {
						buf1[0][cnt1] = posX;
						buf1[1][cnt1++] = posY - 1;
					} else {
						*searchX = posX;
						*searchY = posY-1;
						return;
					}
				}
			}
			/* ｓ西。 */
			if (posX != 0) {
				if (map[posX - 1][posY].step < map[posX][posY].step) {
					if ((map[posX - 1][posY].wall & 0xf0) == 0xf0) {
						buf1[0][cnt1] = posX - 1;
						buf1[1][cnt1++] = posY;
					} else {
						*searchX = posX - 1;
						*searchY = posY;
						return;
					}
				}
			}
		} while (cnt2 != 0); /* ｓバッファが空になるまで繰り返す。 */

		/* ｓもう一方のバッファが空なら終了する。 */
		if (cnt1 == 0) {
			break;
		}

		/* ｓ設定する値を更新する。 */
		++value;
	}

}

