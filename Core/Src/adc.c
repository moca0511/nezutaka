/*
 * adc.c
 *
 *  Created on: 2020/07/23
 *      Author: junmo
 */
#include "adc.h"

uint32_t ADConv(ADCSet *config) {
	ADC_ChannelConfTypeDef sConfig;
	uint32_t ADC_Date=0;
	sConfig.Channel = config->channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(config->hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	HAL_ADC_Start(config->hadc);
	while (HAL_ADC_PollForConversion(config->hadc, 1000) != HAL_OK);
	ADC_Date = HAL_ADC_GetValue(config->hadc);
	HAL_ADC_Stop(config->hadc);
	return ADC_Date;
}

