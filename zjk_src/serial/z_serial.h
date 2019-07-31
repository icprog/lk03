#ifndef _Z_SERIAL_H
#define _Z_SERIAL_H
#include "usart.h"
#define RT_PRINTF_PRECISION
#define RT_USING_DEVICE


#define SERIAL_TYPE_BOOL   uint8_t
#define SERIAL_TRUE  1
#define SERIAL_FALSE 0
void z_serial_write(uint8_t *ch, uint32_t lens);
void z_serial_init(void);
void get_revLens(uint16_t *data);
void usartIdleInt(void);
#define sensor_rs485_dir_ouput()     HAL_GPIO_WritePin(sensor_rs485_dir_GPIO_Port,sensor_rs485_dir_Pin,GPIO_PIN_SET)
#define sensor_rs485_dir_input()     HAL_GPIO_WritePin(sensor_rs485_dir_GPIO_Port,sensor_rs485_dir_Pin,GPIO_PIN_RESET)


typedef void (* _listenFunc)(uint8_t *buf);
typedef struct 
{
  _listenFunc listFunc;
}_dma_listen;

SERIAL_TYPE_BOOL addUartDmaRevListen(_listenFunc func);
void z_serial_baudRateCfg(uint32_t baudRate);


#endif


