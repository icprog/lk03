#ifndef _Z_SERIAL_H
#define _Z_SERIAL_H

#include "usart.h"
#define SERIAL_TYPE_BOOL   uint8_t
#define SERIAL_TRUE  1
#define SERIAL_FALSE 0
void z_serial_write(uint8_t *ch, uint32_t lens);
void z_serial_init(void);
void get_revLens(uint16_t *data);
void usartIdleInt(void);
typedef void (* _listenFunc)(uint8_t *buf);
typedef struct 
{
  _listenFunc listFunc;
}_dma_listen;

SERIAL_TYPE_BOOL addUartDmaRevListen(_listenFunc func);
#endif


