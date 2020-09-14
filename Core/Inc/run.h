/*
 * run.h
 *
 *  Created on: Jul 28, 2020
 *      Author: junmo
 */

#ifndef INC_RUN_H_
#define INC_RUN_H_
#include"main.h"
#include"motor.h"
#include"nezutaka.h"

typedef struct {
	uint8_t direction;	//進行方向 (前後)
	float initial_speed;	//初速 (mm/s)
	float finish_speed;	//終了速度 (mm/s)
	float max_speed;	//最大速度 (mm/s)
	float acceleration;	//加速度
	float value;	//各設定値
} RUNConfig;

//進行方向
#define MOVE_FORWARD GPIO_PIN_RESET	//前進
#define MOVE_BACK GPIO_PIN_SET			//後退

#define TURN_R 0
#define TURN_L 1
#define TURN_U 2

#define SPEED_MAX 65536/STEP_LENGTH-1
#define SPEED_MIN (1680000/(65536+1))/STEP_LENGTH+1

#define kp 0.003
#define ki 0.001
#define kd 0.0001

void straight(RUNConfig config);
void turn(RUNConfig config);
void slalom(RUNConfig config);
void sirituke(void);
int32_t PID(float speed, int32_t target, int32_t sensor,
		int32_t *deviation_prev);
void chenge_head(RUNConfig config) ;
void chenge_pos(uint16_t block);
void turn_u(void);
void run_block(RUNConfig config);
#endif /* INC_RUN_H_ */
