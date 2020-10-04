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
	uint32_t initial_speed;	//初速 (mm/s)
	uint32_t finish_speed;	//終了速度 (mm/s)
	uint32_t max_speed;	//最大速度 (mm/s)
	uint32_t acceleration;	//加速度
	uint32_t value;	//各設定値
} RUNConfig;

typedef struct {
	RUNConfig config;
	uint32_t before_ofset;
	uint32_t after_ofset;
} SLALOMConfig;

//進行方向
#define MOVE_FORWARD GPIO_PIN_RESET	//前進
#define MOVE_BACK GPIO_PIN_SET			//後退

#define TURN_R 0
#define TURN_L 1
#define TURN_U 2

#define SPEED_MAX 65536/STEP_LENGTH-1
#define SPEED_MIN (1680000/(65536+1))/STEP_LENGTH+1

#define kp 0.07
#define ki 0.001
#define kd 0.05

uint16_t straight(RUNConfig config);
void turn(RUNConfig config);
void slalom(SLALOMConfig config);
void sirituke(void);
float32_t PID(float32_t speed, int32_t target, int32_t sensor,
        int32_t *deviation_prev,int32_t *devaition_sum);
void chenge_head(uint16_t direction,uint32_t value,int8_t* head_buf);
void chenge_pos(int16_t block);
void turn_u(void);
void run_block(RUNConfig config);
void ajast(void);
#endif /* INC_RUN_H_ */
