#ifndef _Z_LK03_H
#define _Z_LK03_H
#include "spi.h"
#include "tim.h"
#include "tdc_gp21.h"
#include "tlc5618.h"
#include "z_analog.h"
#include "z_serial.h"
#include "main.h"
#include "tim.h"
#define DISTANCE_RCV_SIZE   5

typedef enum{ trig_onece_complete =1,trig_enough_complete,trig_time_out} TDC_TRIGSTATU;
typedef enum  {VOL_CTL1,VOL_CTL2,VOL_CTL3}TX_VOL_ENUM_TYP;
typedef enum  {START,FIRST,SECOND,THIRD,STYLE}RUN_STATU;
typedef enum  {FIRST_PARAM=0,SECOND_PARAM,THIRD_PARAM}HIGH_VOL_PARAM;
typedef struct {

	uint16_t vol;
	uint8_t vol_statu;

}_tdc_voltage ;


typedef struct {

	float Kp;
	float Ki;
	float ki_sum;
	int16_t setpoint;
  bool ifTrunOn;
}_tdc_pid ;

/*硬件电压参数*/
typedef struct{
	uint16_t  tx5618_value;  //发送高压参数
	uint16_t  rx_vol_value;  //接收高压参数	
	TX_VOL_ENUM_TYP tx_vol_ctl;
	bool ifBootVolCtl;
} HIGHL_VOL_GP21;


/*档位距离*/
typedef struct{
	uint16_t  first_distance;  //第1档距离
	uint16_t  second_distance;  //第2档距离
	uint16_t  third_distance;  //第3档距离
} gear_distance_typ;

typedef struct   
{
	HIGHL_VOL_GP21  vol_param[3];
	/*当前运行状态*/
	RUN_STATU  running_statu; 
	/*GP21获取数据状态*/
	TDC_TRIGSTATU statu;
	/*数据缓存*/
	uint32_t buff[DISTANCE_RCV_SIZE];
	/*是否完成数据转换*/
	bool ifComplete;
	/*转换好的距离*/
	uint16_t distance;
	/*AD603增益*/
	uint16_t pid_resualt;
	/*峰值信号电压*/
 _tdc_voltage siganl;
	/*pid 参数*/
	_tdc_pid pid;
	
}_TDC_TYP;





#define tdc_laser_light_on()      HAL_GPIO_WritePin(Laser_Light_GPIO_Port,Laser_Light_Pin,GPIO_PIN_SET)
#define tdc_laser_light_off()     HAL_GPIO_WritePin(Laser_Light_GPIO_Port,Laser_Light_Pin,GPIO_PIN_SET)
#define tdc_vol_ctl1_Set()        HAL_GPIO_WritePin(TX_Vol_Ctrl1_GPIO_Port,TX_Vol_Ctrl1_Pin,GPIO_PIN_SET)
#define tdc_vol_ctl1_ReSet()      HAL_GPIO_WritePin(TX_Vol_Ctrl1_GPIO_Port,TX_Vol_Ctrl1_Pin,GPIO_PIN_RESET)

#define tdc_vol_ctl2_Set()        HAL_GPIO_WritePin(TX_Vol_Ctrl2_GPIO_Port,TX_Vol_Ctrl2_Pin,GPIO_PIN_SET)
#define tdc_vol_ctl2_ReSet()      HAL_GPIO_WritePin(TX_Vol_Ctrl2_GPIO_Port,TX_Vol_Ctrl2_Pin,GPIO_PIN_RESET)

#define tdc_vol_ctl3_Set()        HAL_GPIO_WritePin(TX_Vol_Ctrl3_GPIO_Port,TX_Vol_Ctrl3_Pin,GPIO_PIN_SET)
#define tdc_vol_ctl3_ReSet()      HAL_GPIO_WritePin(TX_Vol_Ctrl3_GPIO_Port,TX_Vol_Ctrl3_Pin,GPIO_PIN_RESET)

#define tdc_rx_voltge_high()      HAL_GPIO_WritePin(TDC_Sighal_AngleReles_GPIO_Port,TDC_Sighal_AngleReles_Pin,GPIO_PIN_SET)
#define tdc_rx_voltge_low()       HAL_GPIO_WritePin(TDC_Sighal_AngleReles_GPIO_Port,TDC_Sighal_AngleReles_Pin,GPIO_PIN_RESET)


#define tdc_rx_voltgeHighFreq_high()    TDC_Sighal_AngleReles_GPIO_Port->BSRR = TDC_Sighal_AngleReles_Pin

#define tdc_boot_vol_high()      HAL_GPIO_WritePin(boot_vol_ctl_GPIO_Port,boot_vol_ctl_Pin,GPIO_PIN_SET)
#define tdc_boot_vol_low()      HAL_GPIO_WritePin(boot_vol_ctl_GPIO_Port,boot_vol_ctl_Pin,GPIO_PIN_RESET)


#define High_Vol_Ctl_on()      HAL_GPIO_WritePin(High_Vol_Ctl_GPIO_Port,High_Vol_Ctl_Pin,GPIO_PIN_RESET)
#define High_Vol_Ctl_off()     HAL_GPIO_WritePin(High_Vol_Ctl_GPIO_Port,High_Vol_Ctl_Pin,GPIO_PIN_SET)

 void tdc_rx_voltge_relese(void);
/*
#define  TX_HIGH_VOL_TLC5618 1200  //50.1v
#define  TX_HIGH_VOL_TLC5618 400  //60v
#define  TX_HIGH_VOL_TLC5618 1300  //45v
*/
 

 
#define GP21_STATU_CH1      0x0038U
#define GP21_STATU_TIMEOUT  0x0100U
	
#define first_vol_param {\
				.tx5618_value=1210,\
				.rx_vol_value=100,\
				.tx_vol_ctl=VOL_CTL2,\
				.ifBootVolCtl=true}

			
#define second_vol_param {\
	      .tx5618_value=600,\
				.rx_vol_value=100,\
				.tx_vol_ctl=VOL_CTL1,\
				.ifBootVolCtl=false}

//三挡500us,2khz
#define third_vol_param {\
	      .tx5618_value=600,\
				.rx_vol_value=100,\
				.tx_vol_ctl=VOL_CTL1,\
				.ifBootVolCtl=false}					
				
/*参数配置*/
#define AD603_AGC_DEFAULT   500  
#define AD603_AGC_MIN     100 //0.16V -10DB
#define AD603_AGC_MAX     700//0.720V 20DB


#define PID_KP      0.1
#define PID_KI      0.05
#define PID_SETPOINT 1000

#define Debug_Pid   1

extern _TDC_TYP _TDC_GP21;
void tdc_board_init(void);
void gear_select(HIGHL_VOL_GP21 *g);
uint16_t gp21_distance_cal(uint32_t *dit,uint8_t dislens);

#endif

