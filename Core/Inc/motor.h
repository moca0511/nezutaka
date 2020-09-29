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
#define MR 0
#define ML 1
//タイマチャンネル
#define STEPPER_CLOCK_R_CHANNEL TIM_CHANNEL_1
#define STEPPER_CLOCK_L_CHANNEL TIM_CHANNEL_1





float32_t SPEEDtoHz(float32_t speed);
float32_t HztoSPEED(float32_t Hz);
void mortor_direction(uint8_t motor,uint8_t direction);
void mortor_stop(void);
#endif /* INC_MOTOR_H_ */
