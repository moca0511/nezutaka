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
	uint32_t before_ofset_AD;
	uint32_t after_ofset;
	uint32_t after_ofset_AD;
} SLALOMConfig;

//スラローム終了位置(ADC)
#define AFTER_OFSET_AD_VALUE 800

//進行方向
#define MOVE_FORWARD GPIO_PIN_RESET	//前進
#define MOVE_BACK GPIO_PIN_SET			//後退

#define TURN_R 0
#define TURN_L 1
#define TURN_U 2

#define SPEED_MAX 1700
#define SPEED_MIN 100

#define KP 0.08
#define KI 0.0001
#define KD 0.05

uint16_t straight(RUNConfig config,uint8_t pid_F,uint8_t wall_break_F,uint8_t front_Adjustment_F);
void turn(RUNConfig config);
void slalom(SLALOMConfig config);
void sirituke(void);
float32_t pid_calc(float32_t speed, int32_t target, int32_t sensor,
        int32_t *deviation_prev,int32_t *devaition_sum);
void chenge_head(uint16_t direction,uint32_t value,int8_t* head_buf);
void chenge_pos(int16_t block,int16_t* temp_posX,int16_t* temp_posY,int8_t temp_head);
void turn_u(void);
void ajast(void);
void motor_stop(void);
#endif /* INC_RUN_H_ */
