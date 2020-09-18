/*
 * maze.h
 *
 *  Created on: Sep 5, 2020
 *      Author: junmo
 */

#ifndef INC_MAZE_H_
#define INC_MAZE_H_
#include"main.h"

#define MAP_X_MAX 16
#define MAP_Y_MAX 16
#define MAP_SIZE	MAP_X_MAX*MAP_Y_MAX	//　マップマスの数
#define goalX 3
#define goalY 5
#define startX 0
#define startY 0
#define goal (goalY * MAP_X_MAX + goalX) //ｓゴールの添え字
#define start (startY * MAP_Y_MAX + startX)	//ｓスタートの添え字

typedef struct MAP {
	uint8_t step;	//　ゴールまでの距離
	uint8_t wall;	//　北東南西北東南西
	uint8_t check :1;	//　探査済み
} MAP;

void smap_Init(void);  // 歩数マップ初期化
void make_smap(uint16_t gx, uint16_t gy, uint8_t mode); //	gx,gyを目標値とした歩数を設定
void print_map(void);	//	歩数マップ表示
void wall_set(uint8_t mode);
void wall_set_around(void);
uint8_t wall_check(uint8_t direction);
int16_t step_check(uint16_t posX, uint16_t posY, uint8_t direction);
void check_searchBlock(uint16_t *searchX,uint16_t *searchY);

#endif /* INC_MAZE_H_ */
