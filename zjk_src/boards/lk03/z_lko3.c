#include "z_lko3.h"

_TDC_typ _TDC_GP21;

TIM_HandleTypeDef *z_tlc_txHv_pwm= &htim3;
TIM_HandleTypeDef *z_tlc_rxHv_pwm= &htim2;
typedef enum  {VOL_CTL1,VOL_CTL2,VOL_CTL3}TX_VOL_ENUM_TYP;
/*tx pwm high power cotrol*/
 void tx_VolCtl(TX_VOL_ENUM_TYP HVN)
 {
	 
   switch(HVN)
	 {
	   case VOL_CTL1 :
		 {
		   tdc_vol_ctl1_Set();
		 }break;
	   case VOL_CTL2 :
		 {
		    tdc_vol_ctl2_Set();
		 }break;		 
	   case VOL_CTL3 :
		 {
		 
		 }break;		 
	 }
    
 
 }
 
 /*rx pwm high power cotrol*/
 void rx_pwmHv(uint16_t vlue)
 {
 
    z_tlc_rxHv_pwm->Instance->CCR2 = vlue;
 
 } 
 
 
 void start_tx_tim(void)
 {
  	HAL_TIM_PWM_Start(z_tlc_txHv_pwm, TIM_CHANNEL_2);   
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
	 	tlc5618_write(TX_HIGH_VOL_TLC5618,AD603_AGC_DEFAULT); /*LK  AGC DAC Voltage control*/  
	  tlc5618_writeAchannal(TX_HIGH_VOL_TLC5618);	  
	 
		start_tx_tim(); 
		start_rx_tim();	 
	  tx_VolCtl(VOL_CTL1);    /*tx high voltage control  25->14v*/ 
		rx_pwmHv (100);  //84v /*rx high voltage control*/  	 
		GP21_Init(); 
		gp21_write(OPC_RESET);		 /*LK  gp21 Init*/	
		gp21_defaultcofg();		
    
    _TDC_GP21.pid.Kp = PID_KP;
    _TDC_GP21.pid.Ki = PID_KI;	 
	  _TDC_GP21.pid.setpoint = PID_SETPOINT;


 }

#define GP22_TNS  1000       //gp21 ���� 1000ns 1MHZ
float test_dit=0,test_distf=0;
void gp21_distance_cal(uint32_t *dit,uint8_t dislens)
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
		_TDC_GP21.tdc_distance=test_distf = dist_f;
  // dist_av = 0;
}

 