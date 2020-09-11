/*
 * UI.h
 *
 *  Created on: Jul 23, 2020
 *      Author: junmo
 */

#ifndef INC_UI_H_
#define INC_UI_H_

#include "main.h"
void RGBLED_SET(uint8_t R,uint8_t G,uint8_t B);
void UILED_SET(uint8_t HEX);

#define LED_ON GPIO_PIN_RESET
#define LED_OFF GPIO_PIN_SET


#endif /* INC_UI_H_ */
