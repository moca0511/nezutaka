/*
 * buzzer.c
 *
 *  Created on: Jul 7, 2020
 *      Author: junmo
 */
#include "buzzer.h"
#include "timer.h"
#include "cmsis_os.h"

BuzzerConfig buzzer_config = { tone_C, 1 };
extern osMutexId_t UART_MutexHandle;
extern osThreadId_t BUZZER_TaskHandle;
extern TIM_HandleTypeDef htim4;

extern void BUZZER(void *argument)
{
  /* USER CODE BEGIN BUZZER */
	Delay_ms(10);
	PWMconfig pwm_config = { &htim4, BUZZER_CHANNEL, tone_A, 10, 84000000 };
	/* Infinite loop */
	for (;;) {
		while(osThreadFlagsWait(TASK_START|TASK_STOP, osFlagsWaitAny, osWaitForever)!=TASK_START);
//		printf("buzzer:%ldhz,%ldms\n", ((BuzzerConfig*) argument)->tone,
//				((BuzzerConfig*) argument)->ms);
		pwm_config.hz = buzzer_config.tone;
		PWM_Set(&pwm_config);
		HAL_TIM_PWM_Start_IT(&htim4, BUZZER_CHANNEL);
//		printf("start\n");
		Delay_ms(buzzer_config.ms);
		HAL_TIM_PWM_Stop_IT(&htim4, BUZZER_CHANNEL);
//		printf("stop\n");
	}
  /* USER CODE END BUZZER */
}


void tone(uint32_t tone,uint32_t ms){
	buzzer_config.tone =tone;
	buzzer_config.ms = ms;
	osThreadFlagsSet(BUZZER_TaskHandle, TASK_START);
}

//music
void music(void) {
	Delay_ms(50);
	tone(tone_C, 50);
	Delay_ms(50);
	tone(tone_D, 50);
	Delay_ms(50);
	tone(tone_E, 50);
	Delay_ms(50);
	tone(tone_F, 50);
	Delay_ms(50);
	tone(tone_G, 50);
	Delay_ms(50);
	tone(tone_A, 50);
	Delay_ms(50);
	tone(tone_B, 50);
	Delay_ms(50);
	tone(tone_hiC, 50);
	Delay_ms(50);
}
