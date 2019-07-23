#ifndef _Z_SENSOR_ACK_H
#define _Z_SENSOR_ACK_H

#include "z_include.h"
#include "z_param.h"
#include "z_flashParamSave.h"
#include "z_lko3.h"
#include "FreeRTOS.h"
#define BigtoLittle16(A)   (( (*(uint16_t*)A & 0xff00) >> 8)  | (( (*(uint16_t*)A & 0x00ff) << 8)))

extern bool if_debug;
extern bool cureent_qcStandmode;

/*dist ack*/
void sensor_distContinu_ack(uint16_t dist);
void sensor_distOnce_ack(uint16_t dist);
void sensor_distStop_ack(void);
void sensor_getAllParam_void(void);

void sensor_getBaudRate_ack(void);
void sensor_getFrontSwich_ack(void);
void sensor_getDisBase_ack(void);
void sensor_getBackSwich_ack(void);
void sensor_getPowerOnMode_ack(void);
void sensor_getOutDataFreq_ack(void);
void sensor_setAllParam_ack(TF_Msg *msg);
void sensor_setBaudRate_ack(TF_Msg *msg);
void sensor_setFrontSwich_ack(TF_Msg *msg);
void sensor_setBackSwich_ack(TF_Msg *msg);
void sensor_setDisBase_ack(TF_Msg *msg);
void sensor_setPowerOnMode_ack(TF_Msg *msg);
void sensor_setOutDataFreq_ack(TF_Msg *msg);
void sensor_system_boot_paramReset_ack(void);
void sensor_system_firmware_ctl_ack(void);
void sensor_system_firmware_pakage_ack(void);

void sensor_struct_loop(sensor_struct_ * p);

//debug
void sensor_debug_data(uint16_t dist, uint16_t agc, uint16_t sighal, uint8_t gears);
////
void baudRateCfg_select(sensor_baudRate_typeEnum cmd);
#endif


