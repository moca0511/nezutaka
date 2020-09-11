/*
 * motor.h
 *
 *  Created on: Jul 16, 2020
 *      Author: junmo
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

typedef struct {
	uint8_t direction;	//進行方向 (前後)
	uint32_t initial_speed;	//初速 (mm/s)
	uint32_t finish_speed;	//終了速度 (mm/s)
	uint32_t max_speed;	//最大速度 (mm/s)
	uint32_t acceleration;	//加速度
	uint32_t distance;	//走行距離 (mm)
} MotorConfig;

//タイマチャンネル
#define STEPPER_CLOCK_R_CHANNEL TIM_CHANNEL_1
#define STEPPER_CLOCK_L_CHANNEL TIM_CHANNEL_1

//進行方向
#define MOTOR_FORWARD GPIO_PIN_RESET	//前進
#define MOTOR_BACK GPIO_PIN_SET			//後退

//機体パラメータ
#define TIRE_DIAMETER   52.0					//タイヤの直径　52mm
#define TIRE_CIRCUIT    (PI * TIRE_DIAMETER)	//タイヤの円周 163.363mm
#define TREAD_WIDTH     83.0					//トレッド幅　83.0mm(再計測)
#define TREAD_CIRCUIT   (TREAD_WIDTH * PI)		//360度旋回時にタイヤが動く距離　83*3.14mm 260mm
#define	STEP_DEGREE  (1.8 / 2.0)				//ステッピングモータ1-2相励ステップ角（度/step) 0.9°
#define	STEP_LENGTH	(TIRE_CIRCUIT * STEP_DEGREE / 360.0)	//1ステップで進む距離　0.408mm
//迷路のパラメータ
#define	BLOCK_LENGTH  180.0					//1区画 180mm
//定数定義
#define	PI            3.14159265358979		//円周率

//最大速度
#define MAX_SPEED (1680000*STEP_LENGTH-1)
#define MIN_SPEED (1680000/65536*STEP_LENGTH+1)//ドライバの最大

uint32_t SpeedToHz(uint32_t speed);

#endif /* INC_MOTOR_H_ */
