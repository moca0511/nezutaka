/*
 * agent.h
 *
 *  Created on: 2020/09/11
 *      Author: junmo
 */

#ifndef INC_AGENT_H_
#define INC_AGENT_H_

#include"main.h"

typedef struct RUTE {
	uint8_t direction;	//　ゴールまでの距離
	uint32_t value;	//　進行距離
} RUTE;


void adachi(RUNConfig RUN_config,RUNConfig turn_config,SLALOMConfig slalom90_config,uint16_t gx,uint16_t gy);
void hidarite(void);
void saitan(RUNConfig RUN_config,SLALOMConfig slalom90_config,SLALOMConfig slalom180_config,uint16_t gx,uint16_t gy,uint16_t sx,uint16_t sy,int8_t shead);


#endif /* INC_AGENT_H_ */
