#ifndef _Z_FLASH_PARAM_SAVE_H
#define _Z_FLASH_PARAM_SAVE_H
#include "stm32f1xx_hal.h"
#define PARAM_FLASH_ADDRESS   0x0803F800    //254k~256k  
void flash_paramRead(uint8_t *buff, uint16_t lens);
void flash_SaveInit(void);
void flash_writeMoreData( uint16_t *toWriteData, uint16_t toSize);
#endif