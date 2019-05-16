#ifndef _Z_ANALOG_H_
#define _Z_ANALOG_H_
#include "adc.h"
#include "stdbool.h"
#define Z_ANALOG_ERRO   0
#define Z_ANALOG_SUEED   1
#define BUF_SIZE     10

uint16_t z_analog_covertDMA (void);
uint16_t z_analog_convertNorml(void);
uint16_t Get_AnalogDMA_Value(void);
bool z_analog_convert(uint16_t *value);
extern ADC_HandleTypeDef  *z_anlog;
extern uint16_t adc_buf[BUF_SIZE];
extern bool ifDMAisComplete ;   //DMA是否转换完成
#endif

