/*
 * battery.h
 *
 *  Created on: Jul 23, 2020
 *      Author: junmo
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

#include "adc.h"
#include"main.h"

#define ADC_CHANNEL_BATTERY ADC_CHANNEL_1
#define BATT_WARNING (uint32_t)(ADC_MAX/3.3*0.95)
#define BATT_NORMAL (uint32_t)(ADC_MAX/3.3*1)
#define BATT_FULL (uint32_t)(ADC_MAX/3.3*1.1)
#endif /* INC_BATTERY_H_ */
