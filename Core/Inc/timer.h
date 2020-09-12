/*
 * timer.h
 *
 *  Created on: Jun 22, 2020
 *      Author: junmo
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "main.h"

typedef struct {
	TIM_HandleTypeDef *htim;
	uint8_t pin;
	uint32_t hz;
	uint32_t duty;
	uint32_t timeclock;
} PWMconfig;

int PWM_Set(PWMconfig *config);
void wait_us(int32_t wait);
void Delay_ms(uint32_t ms);

#endif /* INC_TIMER_H_ */
