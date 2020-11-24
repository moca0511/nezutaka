/*
 * motor.h
 *
 *  Created on: Jul 16, 2020
 *      Author: junmo
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"



//motor
#define MR (0)
#define ML (1)
//タイマチャンネル
#define STEPPER_CLOCK_R_CHANNEL TIM_CHANNEL_1
#define STEPPER_CLOCK_L_CHANNEL TIM_CHANNEL_1





float32_t SPEEDtoHz(float32_t speed);//mm/s→hz変換
float32_t HztoSPEED(float32_t Hz);//hz→mm/s変換
void mortor_direction(uint8_t motor,uint8_t direction);//指定モータの前転後転を指定
//各種getter,setter
uint32_t get_MotorSpeed_L(void);
uint32_t get_MotorSpeed_R(void);
uint32_t get_MotorSpeed(void);
void set_MotorSpeed_L(uint32_t speed);
void set_MotorSpeed_R(uint32_t speed);
void set_MotorSpeed(uint32_t speed);
uint32_t get_MotorStepCount_L(void);
uint32_t get_MotorStepCount_R(void);
uint32_t get_MotorStepCount(void);
void reset_MotorStepCount_L(void);
void reset_MotorStepCount_R(void);
void reset_MotorStepCount(void);
#endif /* INC_MOTOR_H_ */
