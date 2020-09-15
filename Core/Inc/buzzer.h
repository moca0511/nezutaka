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
	tone_C = 1046,
	tone_D = 1174,
	tone_E = 1318,
	tone_F = 1396,
	tone_G = 1567,
	tone_A = 1760,
	tone_B = 1975,
	tone_hiC = 2093
} BUZZER_TONE;

void tone(uint32_t tone,uint32_t ms);
void music(void);


#endif /* INC_BUZZER_H_ */
