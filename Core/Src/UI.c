/*
 * UI.c
 *
 *  Created on: Jul 23, 2020
 *      Author: junmo
 */
#include "UI.h"

void RGBLED_SET(uint8_t R,uint8_t G,uint8_t B){
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin,R);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,G);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin,B);
}
void UILED_SET(uint8_t HEX){
	if((HEX&0b0001)==0b0001){
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,LED_ON);
	}else{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,LED_OFF);
	}
	if((HEX&0b0010)==0b0010){
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,LED_ON);
		}else{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,LED_OFF);
		}
	if((HEX&0b0100)==0b0100){
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,LED_ON);
		}else{
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,LED_OFF);
		}
	if((HEX&0b1000)==0b1000){
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin,LED_ON);
		}else{
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin,LED_OFF);
		}
}
