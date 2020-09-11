/*
 * buzzer.h
 *
 *  Created on: Jul 7, 2020
 *      Author: junmo
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "main.h"


#define BUZZER_CHANNEL TIM_CHANNEL_2

//ブザー設定
typedef struct {
	uint32_t tone;
	uint32_t ms;
} BuzzerConfig;

//音階周波数
typedef enum {
	tone_C = 523,
	tone_D = 587,
	tone_E = 659,
	tone_F = 698,
	tone_G = 784,
	tone_A = 880,
	tone_B = 988,
	tone_hiC = 1046
} BUZZER_TONE;

void tone(uint32_t tone,uint32_t ms);
void music(void);


#endif /* INC_BUZZER_H_ */
