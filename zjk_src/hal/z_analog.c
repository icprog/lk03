#include "z_analog.h"

#define REF_VALUSE   3300                    //ref  voltage  
ADC_HandleTypeDef  *z_anlog = &hadc1;

uint16_t adc_buf[BUF_SIZE]= {0};
uint16_t adc_value_DMA =0;   //DMA 获取平均值
bool ifDMAisComplete =false;   //DMA是否转换完成


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
	//analog_delay(1000);  //延时等待DMA转换完成
//	if(ifDMAisComplete)
//	{
//		ifDMAisComplete =false;
//	  adc_value_DMA = z_analog_covertDMA ();
//		*value = adc_value_DMA;
//		return true;
//  }
//	else
//	{
//	  erro_count ++;
//		return false;
//	}
	 return true;
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
uint16_t test_adc=0;
uint16_t z_analog_covertDMA (void)
{
  	volatile uint8_t minIndex=0;
   volatile	uint32_t tem=0;	
	  for( int i=0;i<BUF_SIZE-1;i++)    // selection sort 
	  {
		     minIndex = i;
			  for( int j=i+1;j<BUF_SIZE;j++)
		   	{
					 if(adc_buf[j]<adc_buf[minIndex])     
					 {
					    minIndex = j;
					 }
				}
				tem= adc_buf[i];
				adc_buf[i] = adc_buf[minIndex];
				adc_buf[minIndex] = tem;
		}		
		test_adc=adc_buf[2];
//   for(int i=0;i< BUF_SIZE;i++)
//	 {
//	
//	   num += adc_buf[i];   //
//	
//	 }
//	 num = num/BUF_SIZE;
//	 test_valuenum = num;
//	 value = num  * REF_VALUSE/4096;  // 3300/4096
	  value = test_adc  * REF_VALUSE/4096;  // 3300/4096
	 return value;

}


uint16_t Get_AnalogDMA_Value(void)
{

  return adc_value_DMA;
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{

   HAL_ADC_Stop_DMA(z_anlog);
	// adc_value_DMA = z_analog_covertDMA ();
	 ifDMAisComplete = true;
}


