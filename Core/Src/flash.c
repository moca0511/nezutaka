#include "flash.h"
#include"main.h"
#include "cmsis_os.h"
extern osMutexId_t UART_MutexHandle;
/*
 *@brief erase sector1
 */
void eraseFlash(void) {
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_SECTORS;    // select sector
	erase.Sector = FLASH_SECTOR_1;               // set selector1
	erase.NbSectors = 1;        // set to erase one sector
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; // set voltage range (2.7 to 3.6V)

	uint32_t pageError = 0;

	HAL_FLASHEx_Erase(&erase, &pageError);    // erase sector
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data write data
 * @param uint32_t size write data size
 */
void writeFlash(uint32_t address, uint8_t *data, uint32_t size) {
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {

		HAL_FLASH_Unlock();        // unlock flash
		printf("UNLOCK\n");
		eraseFlash();            // erease sector11
		printf("erase\n");
		for (uint32_t add = address; add < (address + size); add++) {
			printf("%d,%p=0x%2x→%p\n",
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, add, *data), data,
					*data, (uint32_t*) add); // write byte
			data++;  // add data pointer
			printf("%p=%ld\n", (uint32_t*) add, *((uint32_t*) add));
		}
		printf("data set\n");

		HAL_FLASH_Lock();        // lock flash
		printf("LOCK\n");
		osMutexRelease(UART_MutexHandle);
	}
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data read data
 * @param uint32_t size read data size
 */
void loadFlash(uint32_t address, uint8_t *data, uint32_t size) {
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		printf("%p=%ld\n", (uint32_t*) address, *(uint32_t*) address);
		memcpy(data, (uint8_t*) address, size); // copy data
		osMutexRelease(UART_MutexHandle);
	}
}
