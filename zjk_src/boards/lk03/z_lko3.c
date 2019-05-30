#include "z_lko3.h"

_TDC_TYP _TDC_GP21;
TIM_HandleTypeDef *z_tlc_rxHv_pwm= &htim2;
HIGHL_VOL_GP21 gp21_highVolCrlParm[3]=
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
 void tdc_board_init(void)
 {  
	 for(int i=0;i<3;i++)
	 {
			_TDC_GP21.vol_param[i] = gp21_highVolCrlParm[i];
	 
	 }
	 	 tlc5618_write(_TDC_GP21.vol_param[FIRST_PARAM].tx5618_value,AD603_AGC_DEFAULT); /*LK  AGC DAC Voltage control*/  
	  tlc5618_writeAchannal(_TDC_GP21.vol_param[FIRST_PARAM].tx5618_value);	  
		start_rx_tim();
		GP21_Init(); 
		gp21_write(OPC_RESET);		 /*LK  gp21 Init*/	
		gp21_defaultcofg();		
    
    _TDC_GP21.pid.Kp = PID_KP;
    _TDC_GP21.pid.Ki = PID_KI;	 
	  _TDC_GP21.pid.setpoint = PID_SETPOINT;

 }

#define GP22_TNS  1000       //gp21 ���� 1000ns 1MHZ
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
					 if(dit[j]<dit[minIndex])     //Ѱ����С����
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
    dist_f = (((float)dist_av)/65536.0) * (GP22_TNS/2) * C_VELOCITY;
		test_dit = dist_av;
		test_distf = dist_f;
		return test_distf;
  // dist_av = 0;
}

 /*档位选择*/
void gear_select(HIGHL_VOL_GP21 *g)
{
	
	HIGHL_VOL_GP21 *p=g; 
	tlc5618_writeAchannal(p->tx5618_value);	  	/*LK  AGC DAC Voltage control*/  
	rx_pwmHv (p->rx_vol_value);   //接收高压
  tx_VolCtl(p->tx_vol_ctl);    /*tx high voltage control*/ 
	if(p->ifBootVolCtl)
	{
	  tdc_boot_vol_high();
	}else
  {
	    tdc_boot_vol_low();
	}		
}