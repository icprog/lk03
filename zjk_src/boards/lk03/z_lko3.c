#include "z_lko3.h"

_TDC_TYP _TDC_GP21;
TIM_HandleTypeDef *z_tlc_rxHv_pwm= &htim2;
 uint16_t GP21_TNS_GLOBAL=0;
high_value_control_ gp21_highVolCrlParm[3]=
{
  first_vol_param,
  second_vol_param,	
  third_vol_param,	
};
/*tx pwm high power cotrol*/
 void tx_VolCtl(TX_VOL_ENUM_TYP HVN)
 {
	 
   switch(HVN)
	 {
	   case VOL_CTL1 :
		 {
		   tdc_vol_ctl1_Set();
			 tdc_vol_ctl2_ReSet();
			 tdc_vol_ctl3_ReSet();
		 }break;
	   case VOL_CTL2 :
		 {
       tdc_vol_ctl3_ReSet();
			 tdc_vol_ctl1_ReSet();
       tdc_vol_ctl2_Set();
		 }break;		 
	   case VOL_CTL3 :
		 {
			tdc_vol_ctl1_ReSet();
			tdc_vol_ctl2_ReSet();
		  tdc_vol_ctl3_Set();
		 }break;		 
	 }
    
 
 }
 
 /*rx pwm high power cotrol*/
 void rx_pwmHv(uint16_t vlue)
 {
 
    z_tlc_rxHv_pwm->Instance->CCR2 = vlue;
 
 } 
 
 
 void start_rx_tim(void)
 {
     HAL_TIM_PWM_Start(z_tlc_rxHv_pwm, TIM_CHANNEL_2);	
 }


 
 void tdc_rx_voltge_relese(void)
{

  tdc_rx_voltge_high();
	
	tdc_delay(500);
	
   tdc_rx_voltge_low();

}
/*bsp 鐢垫簮鎵撳紑**/

void lk_bsp_power_on(void)
{
	tdc_5v_power_on();
	tdc_txHigh_power_on();
	tdc_rxHigh_power_on();
}

/*bsp 鐢垫簮鍏抽棴**/

void lk_bsp_power_off(void)
{
	tdc_5v_power_off();
	tdc_txHigh_power_off();
	tdc_rxHigh_power_off();
}
/*鍒濆鍖�**/
 void tdc_board_init(void)
 {  
	lk_bsp_power_off(); 
	 for(int i=0;i<3;i++)
	 {
			_TDC_GP21.system_statu.high_value_defconfg[i] = gp21_highVolCrlParm[i];
	 
	 }
	 	tlc5618_write(_TDC_GP21.system_statu.high_value_defconfg[lk03_first_gears].tx5618_value,AD603_AGC_DEFAULT); /*LK  AGC DAC Voltage control*/  
	   tlc5618_writeAchannal(_TDC_GP21.system_statu.high_value_defconfg[lk03_first_gears].tx5618_value);	  
		start_rx_tim();
		lk_gp2x_init(); 
	//	
(OPC_START_TOF);		 /*LK  gp21 Init*/	

    _TDC_GP21.pid.Kp = PID_KP;
    _TDC_GP21.pid.Ki = PID_KI;	 
	  _TDC_GP21.pid.setpoint = PID_SETPOINT;

 }
 

#define GP22_TNS  500       //gp21 锟斤拷锟斤拷 1000ns 1MHZ
//#define GP22_TNS  1000       //gp21 锟斤拷锟斤拷 1000ns 1MHZ
float test_dit=0,test_distf=0;
uint16_t gp21_distance_cal(uint32_t *dit,uint8_t dislens)
{
	volatile uint8_t minIndex=0;
  volatile	uint32_t tem=0;
	float dist_av=0,dist_f;
   for( int i=0;i<dislens-1;i++)    // selection sort 
	  {
		     minIndex = i;
			  for( int j=i+1;j<dislens;j++)
		   	{
					 if(dit[j]<dit[minIndex])     //寻锟斤拷锟斤拷小锟斤拷锟斤拷
					 {
					    minIndex = j;
					 }
				}
				tem= dit[i];
				dit[i] = dit[minIndex];
				dit[minIndex] = tem;
		}		
		for(int i=0;i<dislens;i++)
		{
		   
		    dist_av += dit[i] ;		
		}
		dist_av = dist_av/dislens;
    dist_f = (((float)dist_av)/65536.0) * (GP21_TNS_GLOBAL/2) * C_VELOCITY;
		test_dit = dist_av;
		test_distf = dist_f;
		return test_distf;
  // dist_av = 0;
}

void  lk_gp21MessgeMode_switch(GP21_MESSAGE_MODE messge_mode)
{
	_TDC_GP21.tdc_gp2x.measure_mode=messge_mode;
   switch(_TDC_GP21.tdc_gp2x.measure_mode)
	 {
		 case GP21_MESSGE1:
		 {
			GP21_TNS_GLOBAL=1000;
		   lk_gp2x_messgeMode1();	
           lk_gp2x_write(OPC_INIT);			 
		 }break;
		 case GP21_MESSGE2:
		 {
			GP21_TNS_GLOBAL=250;
		    lk_gp2x_messgeMode2();
        lk_gp2x_write(OPC_INIT);			 
		 }break;	 
	 
	 }
}


 /*妗ｄ綅閫夋嫨*/
void gear_select(high_value_control_ *g)
{
	
	high_value_control_ *p=g; 
	tlc5618_writeAchannal(p->tx5618_value);	  	/*LK  AGC DAC Voltage control*/  
	rx_pwmHv (p->rx_vol_value);   //鎺ユ敹楂樺帇
  tx_VolCtl(p->tx_vol_ctl);    /*tx high voltage control*/ 
	if(p->ifBootVolCtl)
	{
	  tdc_boot_vol_high();
	}else
  {
	    tdc_boot_vol_low();
	}		
}

extern void sensor_distOffset_calculate(_sensor_gesr_enum index);

void gear_select_switch(_sensor_gesr_enum gear_index)
{
  _sensor_gesr_enum index = gear_index;
	high_value_control_ *p=NULL;
	p=&_TDC_GP21.system_statu.high_value_defconfg[index];
	 sensor_distOffset_calculate(index);
	switch(index)
	{
		case lk03_first_gears:
		{
//			 _TDC_GP21.messge_mode=GP21_MESSGE1;
//			 lk_gp21MessgeMode_switch(&_TDC_GP21);				
			_TDC_GP21.pid.setpoint = 900;
			_TDC_GP21.system_statu.cureent_gear = 1;
		    __HAL_TIM_SET_AUTORELOAD(&htim3,100);  //璁惧畾100us鍛ㄦ湡
		}break;
		case lk03_second_gears:
		{
//			 _TDC_GP21.messge_mode=GP21_MESSGE1;
//			 lk_gp21MessgeMode_switch(&_TDC_GP21);				
		  _TDC_GP21.pid.setpoint = 900;
			_TDC_GP21.system_statu.cureent_gear = 2;
       __HAL_TIM_SET_AUTORELOAD(&htim3,100);  //璁惧畾100us鍛ㄦ湡				
		}break;
		case lk03_third_gears:
		{
//			 _TDC_GP21.messge_mode=GP21_MESSGE2;
//			 lk_gp21MessgeMode_switch(&_TDC_GP21);			
			_TDC_GP21.pid.setpoint = 800;
			_TDC_GP21.system_statu.cureent_gear = 3;
		   __HAL_TIM_SET_AUTORELOAD(&htim3,500);  //璁惧畾500us鍛ㄦ湡
		}break;		
	}
  gear_select(p);
}

