#ifndef _Z_LK03_H
#define _Z_LK03_H
#include "spi.h"
#include "tim.h"
#include "main.h"
#include "tim.h"

//
#include "z_include.h"



#define tdc_laser_light_on()      HAL_GPIO_WritePin(Laser_Light_GPIO_Port,Laser_Light_Pin,GPIO_PIN_SET)
#define tdc_laser_light_off()     HAL_GPIO_WritePin(Laser_Light_GPIO_Port,Laser_Light_Pin,GPIO_PIN_SET)
#define tdc_vol_ctl1_Set()        HAL_GPIO_WritePin(TX_Vol_Ctrl1_GPIO_Port,TX_Vol_Ctrl1_Pin,GPIO_PIN_SET)
#define tdc_vol_ctl1_ReSet()      HAL_GPIO_WritePin(TX_Vol_Ctrl1_GPIO_Port,TX_Vol_Ctrl1_Pin,GPIO_PIN_RESET)

#define tdc_vol_ctl2_Set()        HAL_GPIO_WritePin(TX_Vol_Ctrl2_GPIO_Port,TX_Vol_Ctrl2_Pin,GPIO_PIN_SET)
#define tdc_vol_ctl2_ReSet()      HAL_GPIO_WritePin(TX_Vol_Ctrl2_GPIO_Port,TX_Vol_Ctrl2_Pin,GPIO_PIN_RESET)


#define tdc_rx_voltge_high()      HAL_GPIO_WritePin(TDC_Sighal_AngleReles_GPIO_Port,TDC_Sighal_AngleReles_Pin,GPIO_PIN_SET)
#define tdc_rx_voltge_low()       HAL_GPIO_WritePin(TDC_Sighal_AngleReles_GPIO_Port,TDC_Sighal_AngleReles_Pin,GPIO_PIN_RESET)

 void tdc_rx_voltge_relese(void);
 
/*≤Œ ˝≈‰÷√*/
#define AD603_AGC_DEFAULT   300  
#define AD603_AGC_MIN     200 //0.16V -10DB
#define AD603_AGC_MAX     1200//0.720V 20DB
#define  TX_HIGH_VOL_TLC5618   1200  //




#define PID_KP      0.1
#define PID_KI      0.01
#define PID_SETPOINT 1200




 void tdc_board_init(void);
void gp21_distance_cal(uint32_t *dit,uint8_t dislens);



#endif

