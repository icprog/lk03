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
typedef enum{GP21_MESSGE1=1,GP21_MESSGE2=2,}GP21_MESSAGE_MODE;  //GP21����ģʽ
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

//���������в���
typedef  struct  
{
	#define SENSOR_LENGTH  12     //���������� 12cm
	uint8_t dist_sensor_lenth; //����������
	uint8_t  dist_base;   //��ǰ��׼
   uint16_t qc_offset[3] ;   //�궨ƫ�� ,1,2,3��
	 uint16_t dist_offset;    //��ǰ����ƫ��ֵ
	 uint16_t front_switch; //ǰ����������
	 uint16_t back_switch; //�󿪹�������
	 uint16_t output_freq; // �������Ƶ�� ms
	 bool  qc_ifStand[3]; //1,2,3���Ƿ�궨	
}sensor_struct_typ;           


/*Ӳ����ѹ����*/
typedef struct{
	uint16_t  tx5618_value;  //���͸�ѹ����
	uint16_t  rx_vol_value;  //���ո�ѹ����	
	TX_VOL_ENUM_TYP tx_vol_ctl;
	bool ifBootVolCtl;
} high_value_control_;


/*��λ����*/
typedef struct{
	uint16_t  first_distance;  //��1������
	uint16_t  second_distance;  //��2������
	uint16_t  third_distance;  //��3������
} gear_distance_typ;

typedef struct{
	 GP21_MESSAGE_MODE measure_mode;
	 TDC_TRIGSTATU     status;
	 uint32_t          erro_cnt;
} tdc_gp2x_statu_;


typedef struct{
	high_value_control_  high_value_defconfg[3];   //��ѹ���Ʋ���
  uint8_t cureent_gear;     /*��ǰ�ĵ�λ*/
	RUN_STATU  running_statu;  /*��ǰ����״̬*/
} system_statu_;

typedef struct   
{
	system_statu_  system_statu;  
	/*sʱ��ת��оƬgp21״̬*/
	tdc_gp2x_statu_ tdc_gp2x;
	/*���ݻ���*/
	uint32_t buff[DISTANCE_RCV_SIZE];
	/*�Ƿ��������ת��*/
	bool ifComplete;
	/*ת���õľ���*/
	uint16_t distance;
	/*��Ч�źű��*/
	bool ifDistanceNull;
		/*��������������*/
	bool ifMachineFine;
	/*AD603����*/
	uint16_t pid_resualt;
	/*��ֵ�źŵ�ѹ*/
 _tdc_voltage siganl;
	/*pid ����*/
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
				.tx5618_value=1330,\
				.rx_vol_value=100,\
				.tx_vol_ctl=VOL_CTL2,\
				.ifBootVolCtl=true}

			
#define second_vol_param {\
	      .tx5618_value=600,\
				.rx_vol_value=100,\
				.tx_vol_ctl=VOL_CTL1,\
				.ifBootVolCtl=false}

//����500us,2khz
#define third_vol_param {\
	      .tx5618_value=600,\
				.rx_vol_value=120,\
				.tx_vol_ctl=VOL_CTL1,\
				.ifBootVolCtl=false}					
				
/*��������*/
#define AD603_AGC_DEFAULT   500  
#define AD603_AGC_MIN     100 //0.16V -10DB
#define AD603_AGC_MAX     650//0.720V 20DB

#define PID_KP      0.1
#define PID_KI      0.05
#define PID_SETPOINT 1000

#define Debug_Pid   1
void lk_bsp_power_on(void);   //��ʼ��������ʱ���
void lk_bsp_power_off(void);	//ֹͣ���������ر�			
void  lk_gp21MessgeMode_switch(GP21_MESSAGE_MODE messge_mode);
extern _TDC_TYP _TDC_GP21;
void tdc_board_init(void);
void gear_select(high_value_control_ *g);
uint16_t gp21_distance_cal(uint32_t *dit,uint8_t dislens);
void gear_select_switch(_sensor_gesr_enum gear_index);
				
#endif

