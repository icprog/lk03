#ifndef _Z_ANALOG_H_
#define _Z_ANALOG_H_
#include "adc.h"
#define Z_ANALOG_ERRO   0
#define Z_ANALOG_SUEED   1


uint16_t z_analog_covertDMA (void);
uint16_t z_analog_convertNorml(void);
uint16_t Get_AnalogDMA_Value(void);
uint8_t z_analog_convert(uint16_t *value);
#endif

