/*
 * adc.h
 *
 *  Created on: 2020/07/23
 *      Author: junmo
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_
#include "main.h"

//ADC
typedef struct {
	ADC_HandleTypeDef *hadc;	//ADCのハンドル
	uint32_t channel;	//ADCのチャンネル
} ADCSet;
uint32_t ADConv(ADCSet *config);

#define ADC_MAX 4095


#endif /* INC_ADC_H_ */
