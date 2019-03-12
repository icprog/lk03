#ifndef _Z_INCLUDE_H_
#define _Z_INCLUDE_H_

//board include
#include "z_lko3.h"


//z driver include 
#include "tdc_gp21.h"
#include "tlc5618.h"
#include "z_analog.h"
#include "z_serial.h"

//z serial to pc  include
#include "Connect_format.h"
#include "TinyFrame.h"
#include "z_param.h"

//z func include

#include "z_flashParamSave.h"

//task create
void z_taskCreate(void);

#define DISTANCE_RCV_SIZE   5
//typedef
typedef struct {

	uint16_t vol;
	uint8_t vol_statu;

}_tdc_voltage ;


typedef struct {

	float Kp;
	float Ki;
	float ki_sum;
	int16_t setpoint;

}_tdc_pid ;

typedef struct{

 uint8_t time_out;
 uint16_t txSignalCnt;

}_tdc_statu;

typedef struct   
{
	uint32_t gp21_statu_INT;
	float gp21_dist_reult;
	uint32_t gp21_distance[DISTANCE_RCV_SIZE];
	uint16_t z_analog_erro_cnt;
	uint8_t isGp21Complete;
	uint16_t tdc_distance;
	uint16_t tlc_resualt;
 _tdc_voltage rev_siganl;
	_tdc_statu statu;   
	_tdc_pid pid;
	
}_TDC_typ;
extern _TDC_typ _TDC_GP21;

#endif





