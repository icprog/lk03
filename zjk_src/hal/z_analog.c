#include "z_analog.h"

#define REF_VALUSE   3300                    //ref  voltage  
ADC_HandleTypeDef  *z_anlog = &hadc1;

uint16_t adc_buf[BUF_SIZE]= {0};
uint16_t adc_value_DMA =0;   //DMA ��ȡƽ��ֵ
bool ifDMAisComplete =false;   //DMA�Ƿ�ת�����


  /*delay ns */
 void analog_delay(uint32_t cval)
 {
    while(cval--);
 
   
 }
 uint8_t erro_count=0;
bool z_analog_convert(uint16_t *value)
{
  HAL_ADCEx_Calibration_Start(z_anlog);  //Need to calibrate,Otherwise it's not accurate 
  HAL_ADC_Start_DMA(z_anlog,(uint32_t*)adc_buf,BUF_SIZE);
  // z_analog_convertNorml();
	//analog_delay(1000);  //��ʱ�ȴ�DMAת�����
	if(ifDMAisComplete)
	{
		ifDMAisComplete =false;
	  adc_value_DMA = z_analog_covertDMA ();
		*value = adc_value_DMA;
		return true;
  }
	else
	{
	  erro_count ++;
		return false;
	}
	 
}


void z_analog_Norml(void)
{
	 
	
}

uint16_t z_analog_value =0;
uint16_t z_analog_convertNorml(void)
{
	    uint32_t value =0;
	   HAL_ADCEx_Calibration_Start(z_anlog);  //Need to calibrate,Otherwise it's not accurate
     HAL_ADC_Start(z_anlog);
	   HAL_ADC_PollForConversion(z_anlog,0xff) ;
     if( HAL_IS_BIT_SET(HAL_ADC_GetState(z_anlog),HAL_ADC_STATE_REG_EOC) )
		 {
		     value  = HAL_ADC_GetValue(z_anlog);
			    z_analog_value =value * REF_VALUSE/4095;
			   
		 } 
		 return z_analog_value;
}


float value = 0;
float test_valuenum=0;
uint16_t z_analog_covertDMA (void)
{
	 float num =0;
   for(int i=0;i< BUF_SIZE;i++)
	 {
	
	   num += adc_buf[i];   //2^4 = 16 ƽ����
	
	 }
	 num = num/BUF_SIZE;
	 test_valuenum = num;
	 value = num  * REF_VALUSE/4096;  // 3300/4096
	 return value;

}


uint16_t Get_AnalogDMA_Value(void)
{

  return adc_value_DMA;
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{

   HAL_ADC_Stop_DMA(z_anlog);
	 adc_value_DMA = z_analog_covertDMA ();
	 ifDMAisComplete = true;
}


