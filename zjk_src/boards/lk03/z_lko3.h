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
typedef enum  {IDLE,START,FIRST,SECOND,THIRD,LONG_DISTANCE_MODE,STYLE}RUN_STATU;
typedef enum{lk03_first_gears=0,lk03_second_gears,lk03_third_gears} _sensor_gesr_enum;
typedef enum{GP21_MESSGE1=1,GP21_MESSGE2=2,}GP21_MESSAGE_MODE;  //GP21测量模式
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

//传感器运行参数
typedef  struct  
{
	#define SENSOR_LENGTH  9     //传感器长度 9cm
	uint8_t dist_sensor_lenth; //传感器长度
	uint8_t  dist_base;   //当前基准
   uint16_t qc_offset[3] ;   //标定偏差 ,1,2,3档
	 uint16_t dist_offset;    //当前运行偏差值
	 uint16_t front_switch; //前开关量距离
	 uint16_t back_switch; //后开关量距离
	 uint16_t output_freq; // 数据输出频率 ms
	 bool  qc_ifStand[3]; //1,2,3档是否标定	
}sensor_struct_typ;           


/*硬件电压参数*/
typedef struct{
	uint16_t  tx5618_value;  //发送高压参数
	uint16_t  rx_vol_value;  //接收高压参数	
	TX_VOL_ENUM_TYP tx_vol_ctl;
	bool ifBootVolCtl;
} high_value_control_;


/*档位距离*/
typedef struct{
	uint16_t  first_distance;  //第1档距离
	uint16_t  second_distance;  //第2档距离
	uint16_t  third_distance;  //第3档距离
} gear_distance_typ;

typedef struct{
	 GP21_MESSAGE_MODE measure_mode;
	 TDC_TRIGSTATU     status;
	 uint32_t          erro_cnt;
} tdc_gp2x_statu_;


typedef struct{
	high_value_control_  high_value_defconfg[3];   //高压控制参数
  uint8_t cureent_gear;     /*当前的档位*/
	RUN_STATU  running_statu;  /*当前运行状态*/
} system_statu_;

typedef struct   
{
	system_statu_  system_statu;  
	/*s时间转换芯片gp21状态*/
	tdc_gp2x_statu_ tdc_gp2x;
	/*数据缓存*/
	uint32_t buff[DISTANCE_RCV_SIZE];
	/*是否完成数据转换*/
	bool ifComplete;
	/*转换好的距离*/
	uint16_t distance;
	/*有符号距离*/
	int16_t distance_inl;
	bool ifAverageComplete;
    /*平均值*/
	uint16_t average;
	/*无效信号标记*/
	bool ifDistanceNull;
		/*开机正常后才输出*/
	bool ifMachineFine;
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


#define sensor_ouput_switch_high()    HAL_GPIO_WritePin(senor_switch_GPIO_Port,senor_switch_Pin,GPIO_PIN_SET)
#define sensor_ouput_switch_low()     HAL_GPIO_WritePin(senor_switch_GPIO_Port,senor_switch_Pin,GPIO_PIN_RESET)

#define tdc_5v_power_off()    HAL_GPIO_WritePin(TDC_Power_Ctl_GPIO_Port,TDC_Power_Ctl_Pin,GPIO_PIN_SET)
#define tdc_5v_power_on()     HAL_GPIO_WritePin(TDC_Power_Ctl_GPIO_Port,TDC_Power_Ctl_Pin,GPIO_PIN_RESET)

#define tdc_txHigh_power_off()    HAL_GPIO_WritePin(High_Vol_Ctl_GPIO_Port,High_Vol_Ctl_Pin,GPIO_PIN_SET)
#define tdc_txHigh_power_on()     HAL_GPIO_WritePin(High_Vol_Ctl_GPIO_Port,High_Vol_Ctl_Pin,GPIO_PIN_RESET)

#define tdc_rxHigh_power_off()    HAL_GPIO_WritePin(Rx_Power_Ctl_GPIO_Port,Rx_Power_Ctl_Pin,GPIO_PIN_SET)
#define tdc_rxHigh_power_on()     HAL_GPIO_WritePin(Rx_Power_Ctl_GPIO_Port,Rx_Power_Ctl_Pin,GPIO_PIN_RESET)

void tdc_rx_voltge_relese(void);
/*
#define  TX_HIGH_VOL_TLC5618 1200  //50.1v
#define  TX_HIGH_VOL_TLC5618 400  //60v
#define  TX_HIGH_VOL_TLC5618 1300  //45v
*/
 

 
#define GP21_STATU_CH1      0x0038U
#define GP21_STATU_TIMEOUT  0x0100U
	
#define first_vol_param {\
				.tx5618_value=1380,\
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
				.rx_vol_value=120,\
				.tx_vol_ctl=VOL_CTL1,\
				.ifBootVolCtl=false}					
				
/*参数配置*/
#define AD603_AGC_DEFAULT   500  
#define AD603_AGC_MIN     50 //0.16V -10DB
#define AD603_AGC_MAX     650//0.720V 20DB

#define PID_KP      0.1
#define PID_KI      0.05
#define PID_SETPOINT 1000

#define Debug_Pid   1
void lk_bsp_power_on(void);   //开始连续测量时候打开
void lk_bsp_power_off(void);	//停止测量测量关闭			
void  lk_gp21MessgeMode_switch(GP21_MESSAGE_MODE messge_mode);
extern _TDC_TYP _TDC_GP21;
void tdc_board_init(void);
void gear_select(high_value_control_ *g);
uint16_t gp21_distance_cal(uint32_t *dit,uint8_t dislens);
void gear_select_switch(_sensor_gesr_enum gear_index);
				
#endif

