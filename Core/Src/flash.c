#include "flash.h"

/*
 *@brief erase sector1
*/
void eraseFlash( void )
{
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;    // select sector
    erase.Sector = FLASH_SECTOR_1;               // set selector11
    erase.NbSectors = 1;        // set to erase one sector
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;    // set voltage range (2.7 to 3.6V)

    uint32_t pageError = 0;

    HAL_FLASHEx_Erase(&erase, &pageError);    // erase sector
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data write data
 * @param uint32_t size write data size
*/
void writeFlash(uint32_t address, uint8_t *data, uint32_t size  )
{

    HAL_FLASH_Unlock();        // unlock flash
    printf("UNLOCK\n");
    eraseFlash();            // erease sector11
    printf("erase\n");
  for ( uint32_t add = address; add < (address + size); add++ ){
        printf("%d,%p=0x%2x→%p\n",HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, add, *data),data,*data,(uint32_t*)add); // write byte
    data++;  // add data pointer
    printf("%p=%ld\n",(uint32_t*)add,*((uint32_t*)add));
    }
  printf("data set\n");

    HAL_FLASH_Lock();        // lock flash
    printf("LOCK\n");
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data read data
 * @param uint32_t size read data size
*/
void loadFlash(uint32_t address, uint8_t *data, uint32_t size )
{
    printf("%p=%ld\n",(uint32_t*)address,*(uint32_t*)address);
    memcpy(data, (uint8_t*)address, size); // copy data
}
