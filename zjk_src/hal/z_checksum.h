#ifndef _Z_CHECKSUM_H
#define _Z_CHECKSUM_H
#include "stm32f1xx_hal.h"

uint8_t tx_chexkSum(uint8_t *buf,uint8_t lens);

uint8_t rx_chexkSum(uint8_t *buf,uint8_t lens);
#endif