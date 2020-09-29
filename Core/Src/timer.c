/*
 * timer.c
 *
 *  Created on: Jun 22, 2020
 *      Author: junmo
 */

#include "timer.h"
#include "cmsis_os.h"


extern TIM_HandleTypeDef htim2;
int32_t us = 20;
extern osSemaphoreId_t wait_usSemHandle;

int PWM_Set(PWMconfig *config) {
	config->htim->Init.Prescaler = config->timeclock / 100 / (config->hz) - 1;
	config->htim->Init.Period = 100;
	if (HAL_TIM_PWM_Init(config->htim) != HAL_OK) {
		return -1;
	}
	__HAL_TIM_SET_COMPARE(config->htim, config->pin, config->duty);
	return 0;
}

void wait_us(int32_t wait) {
	us = wait;
	HAL_TIM_Base_Start_IT(&htim2);
	osSemaphoreAcquire(wait_usSemHandle, osWaitForever);
}

void Delay_ms(uint32_t ms){
	osDelayUntil(osKernelGetTickCount()+(uint32_t)configTICK_RATE_HZ /1000 * ms);
}

