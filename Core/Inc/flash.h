/*
 * flash.h
 *
 *  Created on: 2020/10/07
 *      Author: junmo
 */

#include"main.h"
#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#define MAZE_FLASH_START_ADD 0x8004000 //sentor1 start address
#define MAZE_FLASH_END_ADD 0x8007FFF // sector1 end address

void eraseFlash( void );
void writeFlash(uint32_t address, uint8_t *data, uint32_t size  );
void loadFlash(uint32_t address, uint8_t *data, uint32_t size );

#endif /* INC_FLUSH_H_ */

