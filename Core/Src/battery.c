/*
 * battery.c
 *
 *  Created on: Jul 23, 2020
 *      Author: junmo
 */
#include"main.h"
#include "cmsis_os.h"
#include"adc.h"
#include"battery.h"
#include"buzzer.h"
#include "UI.h"
extern osMutexId_t UART_MutexHandle;
extern ADC_HandleTypeDef hadc2;
extern void BatteryCheck(void *argument) {
	/* USER CODE BEGIN BatteryCheck */
	/* Infinite loop */
	ADCSet ADC_config = { &hadc2, ADC_CHANNEL_BATTERY };
	uint32_t batt_level = 0;
	uint8_t sound=0;
	uint8_t count=0;
	//printf("MAX:%ld,NORMAL:%ld,WARNING:%ld\n",BATT_FULL,BATT_NORMAL,BATT_WARNING);
	//int plpl = 1;
	for (;;) {
		batt_level = ADConv(&ADC_config);
		//printf("battlevel:%ld\r", batt_level);
		if (batt_level <= BATT_WARNING) {
			count++;
			if(count>10)
				count=10;
			RGBLED_SET(LED_ON,LED_OFF,LED_OFF);
			if(sound==0&&count==10){
				sound=1;
				tone(tone_hiC, 1000);
			}else{
				sound=0;
			}
		}else if(batt_level <= BATT_NORMAL){
			count=0;
			RGBLED_SET(LED_OFF,LED_OFF,LED_ON);
		}else{
			count=0;
			RGBLED_SET(LED_OFF,LED_ON,LED_OFF);
		}
		osDelay(1000); //1s
	}
	/* USER CODE END BatteryCheck */
}
