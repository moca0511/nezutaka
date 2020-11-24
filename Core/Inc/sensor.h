/*
 * sensor.h
 *
 *  Created on: Jun 22, 2020
 *      Author: junmo
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"


//センサーデータ一覧
typedef struct {
	uint32_t ADC_DATA_RS;
	uint32_t ADC_DATA_RF;
	uint32_t ADC_DATA_LS;
	uint32_t ADC_DATA_LF;
} SensorData;

typedef enum {
	RS,
	RF,
	LS,
	LF
} SensorDataSelect;

//センサーアナログ入力チャンネル
#define ADC_CHANNEL_RS ADC_CHANNEL_13
#define ADC_CHANNEL_RF ADC_CHANNEL_11
#define ADC_CHANNEL_LS ADC_CHANNEL_12
#define ADC_CHANNEL_LF ADC_CHANNEL_10
//LED PWM出力チャンネル
#define PWM_CHANNEL_RS TIM_CHANNEL_4
#define PWM_CHANNEL_RF TIM_CHANNEL_2
#define PWM_CHANNEL_LS TIM_CHANNEL_3
#define PWM_CHANNEL_LF TIM_CHANNEL_1

uint32_t read_wall(uint8_t select);//指定のセンサのADC値を複数回取得し平均値を返す
void print_sensordata(void);//debug情報表示タスク開始・終了
void wall_calibration(void);//センサリファレンス値取得
uint32_t get_sensordata(uint8_t select);//指定センサの値を取得

#endif /* INC_SENSOR_H_ */
