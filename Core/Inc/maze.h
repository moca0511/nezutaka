/*
 * maze.h
 *
 *  Created on: Sep 5, 2020
 *      Author: junmo
 */

#ifndef INC_MAZE_H_
#define INC_MAZE_H_
#include"main.h"

#define MAP_X_MAX 4
#define MAP_Y_MAX 6
#define MAP_SIZE	MAP_X_MAX*MAP_Y_MAX	//　マップマスの数
#define GOAL_X 3
#define GOAL_Y 1
#define START_X 0
#define START_Y 0
#define GOAL (GOAL_Y * MAP_X_MAX + GOAL_X) //ｓゴールの添え字
#define START (START_Y * MAP_Y_MAX + START_X)	//ｓスタートの添え字

typedef struct MAP {
	uint8_t step;	//　ゴールまでの距離
	uint8_t wall;	//　北東南西北東南西
} MAP;

void smap_Init(void);  // 歩数マップ初期化
void make_smap(uint16_t gx, uint16_t gy, uint8_t mode); //	gx,gyを目標値とした歩数を設定
void print_map(void);	//	歩数マップ表示
void wall_set(uint8_t mode);
void wall_set_around(void);
uint8_t wall_check(uint8_t direction);
int16_t step_check(uint16_t posX, uint16_t posY, uint8_t direction);
void check_searchBlock(uint16_t *searchX,uint16_t *searchY);
void maze_save(void);
void maze_load(void);


#endif /* INC_MAZE_H_ */
