/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
//zjk include
#include "z_include.h"
#include "z_lko3.h"
#include "Connect_format.h"
#include "TinyFrame.h"
#include "z_param.h"
#include "z_flashParamSave.h"

//
#include "z_include.h"
/*pid ��������*/


/*ȫ�ֱ�������*/
 TIM_HandleTypeDef *z_tlc_TxSignl_pwm= &htim3;
 int textCount =0;
int trighCount=0;
int trigCount = 0;  //�����ɼ����Ĵ�������
int erroTimeOutCount = 0;  //tdc ʱ�䳬ʱ�жϴ�����
bool ifEnoughTrigComplete = false; //�ɼ�һ�β��������Ƿ����
 TDC_TRIGSTATU tdc_statu;
 //�ź���
 SemaphoreHandle_t  tdcSignalSemaphore =NULL;  //�����ź��������ڴ���TDC�������趨�Ĵ����󴥷����Ʒ�ֵ��ѹ����������
/*����������*/
static TaskHandle_t xHandleSerial = NULL;
TaskHandle_t xHandleGp21Trig = NULL;
static TaskHandle_t xHandleSerialDriver = NULL;
 static TaskHandle_t xHandleSensorParam = NULL;
/*��������*/
void SerialTask(void  * argument);
void Gp21TrigTask(void  * argument);
 void LK_sensorParamTask(void *argument);
extern void z_serialDriverTask(void  * argument);
extern void z_tiny_test(void);
 uint16_t tdc_agc_Default_control(uint16_t nowData,int16_t setPoint);
uint16_t tdc_agc_control(uint16_t nowData,int16_t setPoint);
void trigEnough(void);
TDC_TRIGSTATU trigGetData(void);
void trigOnce(void);
void start_singnal(void);
void z_taskCreate(void)
{
	xTaskCreate(
	              LK_sensorParamTask,    //������
	              "LK_sensorParamTask",  //������
								128,          //����ջ��С��Ҳ����4���ֽ�
	              NULL,
								1,            //�������ȼ�
								&xHandleSensorParam
	            );	
	xTaskCreate(
	             SerialTask,    //������
	             "SerialTask",  //������
								128,          //����ջ��С��Ҳ����4���ֽ�
	              NULL,
								2,            //�������ȼ�
								&xHandleSerial
	            );	
	
	xTaskCreate(
	             z_serialDriverTask,  //������
	             "serialDriverTask",  //������
								128,                //����ջ��С��Ҳ����4���ֽ�
	              NULL,
								3,                  //�������ȼ�
								&xHandleSerialDriver
	            );

	xTaskCreate(
	             Gp21TrigTask,    //������
	             "Gp21TrigTask",  //������
								512,            //����ջ��С��Ҳ����4���ֽ�
	              NULL, 
								4,                //�������ȼ�
								&xHandleGp21Trig
	            );


}
arrayByte_ paramBuff;
arrayByte_ flashParam;
void LK_sensorParamTask(void *argument)
{
  flash_SaveInit();
	paramBuff = structToBytes(&lk_parm);
	flashParam = structToBytes(&lk_flash);
	flash_paramRead(flashParam.point,flashParam.lens); //��ȡ����
	if(lk_flash.ifHasConfig != 0x01)   //��û������
	{	
		lk_flash.ifHasConfig = 0x01;   //��������
	  lk_flash = lk_parm;
    flash_writeMoreData( (uint16_t *)(paramBuff.point),paramBuff.lens/2+1);		
	}
  for(;;)
	{
	  if(lk_param_statu.ifParamSave)
		{ 
		 flash_writeMoreData( (uint16_t *)(paramBuff.point),paramBuff.lens/2+1);
		 lk_param_statu.ifParamSave = false;
		}
		if(lk_param_statu.ifParamGet)
		{
			 parmSend(&lk_parm);
		  lk_param_statu.ifParamGet = false;
		}
		if(lk_param_statu.ifQCStand) //�궨
		{
			start_singnal();
		  lk_param_statu.ifQCStand = false;
		}
    if(lk_param_statu.ifGetOnceDist)	//���β���
		{
		
		}
    if(lk_param_statu.ifContinuDist)	//��������
		{
			start_txSignl_Tim();   //��ʼpwm���巢��
			HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);	
			trigOnce();				
			lk_param_statu.ifContinuDist=false;
		}		
	 osDelay(500);
	}
}

void start_singnal(void)
{
		start_txSignl_Tim();   //��ʼpwm���巢��
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);	
		trigOnce();			 
}


/* USER CODE BEGIN Header_SerialTask */
/**
* @brief Function implementing the z_serial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SerialTask */
RUN_STATU lk_statu;
void SerialTask(void  *argument)
{
  z_tiny_test();
	 
  /* Infinite loop */
  for(;;)
  {
     if(_TDC_GP21.ifComplete)
		 {
			 _TDC_GP21.ifComplete = false;
		   Send_Pose_Data(&_TDC_GP21.siganl.vol,&_TDC_GP21.distance,&_TDC_GP21.pid_resualt);
//       uint8_t *sendBuf =(uint8_t*)(&_TDC_GP21.tdc_distance);
//			 zTF_sendOnceDist(sendBuf,2);			 
		 }
		 switch(_TDC_GP21.running_statu)
		 {
			 case START:
			 {
				  if(_TDC_GP21.siganl.vol>1000)
					{
						_TDC_GP21.pid.ifTrunOnPid = true;
					   _TDC_GP21.running_statu = FIRST;
					}
					else if(_TDC_GP21.siganl.vol<1000)
					{			
					  _TDC_GP21.pid.ifTrunOnPid = true;
						_TDC_GP21.running_statu = SECOND;
					}
			 }break;
			 case FIRST:
			 {
				 
			 }break;
			 case SECOND:
			 {
				 
			 }break;		 
			 case THIRD:
			 {
				 
			 }break;		 			 
		 }
			 
    osDelay(5);
  }
  /* USER CODE END SerialTask */
}


/**
* @brief Function implementing the z_gp21_trig thread.
* @param argument: Not used
* @retval None
*/
int erroDmaTimeOutAdc=0;
#define Close_Trig() HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4)
#define OpenTrig() HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_4)
#define  TrigPluse_on() __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,1);
#define  TrigPluse_off() __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,0);

void Gp21TrigTask(void *argument)
{
	
  tdcSignalSemaphore = xSemaphoreCreateBinary();	 //
  tdc_board_init();   /*��ʼ�������*/
	gear_select(&_TDC_GP21.vol_param[FIRST_PARAM]);  //����Ĭ�ϵ�һ��λ
	start_singnal();
  /* Infinite loop */
  for(;;)
  { 
		
    if(trighCount>10)
	 {
	   trighCount = 0;
		  gp21_write(OPC_START_TOF);

	 	z_analog_convert(&_TDC_GP21.siganl.vol);
		tdc_rx_voltge_relese();   /*��ѹ�źŲɼ��ͷ�*/
		_TDC_GP21.pid_resualt= tdc_agc_Default_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid���Ʒ�ֵ��ѹ	 
	 }
   osDelay(100);
  }
   
  /* USER CODE END Gp21TrigTask */
}

void trigOnce(void)
{
	gp21_write(OPC_START_TOF);
  gp21_en_startSignal();	  //ʹ�ܿ�ʼ�ź�
	gp21_en_stop1Signal();	
}

void closeTdc(void)
{
   gp21_close_startSignal();	  
	 gp21_close_stop1Signal();	     
}


TDC_TRIGSTATU trigGetData(void)
{
	uint32_t gp21_statu_INT;

  gp21_statu_INT = get_gp21_statu();	 
 	
	if(gp21_statu_INT & GP21_STATU_CH1)
	{
		closeTdc();		
    _TDC_GP21.buff[trigCount++] = gp21_read_diatance();//�ռ������������		
    if(trigCount == DISTANCE_RCV_SIZE) 	
		{
			trigCount = 0;
		
		  return trig_enough_complete;
		}	
		else 
		{
		  return trig_onece_complete;
		}
		
	}
if(gp21_statu_INT & GP21_STATU_TIMEOUT)  //����ʱ�����
	{
		  erroTimeOutCount ++ ;
      closeTdc();		
		return trig_time_out;
	}	
	return false;
}


/*�ɼ����㹻���ݺ�ʼ���ݴ���*/
void trigEnough(void)
{

	tdc_rx_voltge_relese();   /*��ѹ�źŲɼ��ͷ�*/
	gp21_distance_cal(_TDC_GP21.buff,DISTANCE_RCV_SIZE);
	_TDC_GP21.ifComplete = true;		
}
 
/*AGC Control
@input: input voltage feedback value, unit mv


@return control value AGC AD603
 */
uint16_t tdc_agc_control(uint16_t nowData,int16_t setPoint)
{
	int16_t pid_resualt=0,ad603_resualt=0;
   int16_t error_t=0;   //error  setpoint- input
	 error_t = setPoint - nowData; 
	 
	  if(error_t >=100)
		{
		  error_t =error_t/10;
		
		}
		else if(error_t <-100)
		{
		   error_t =error_t/10;

		}
		else
		{
			  _TDC_GP21.pid.Kp =0;
		}
		
		//error_t =error_t/10;
		_TDC_GP21.pid.ki_sum+=error_t*_TDC_GP21.pid.Ki;
	  pid_resualt = error_t*_TDC_GP21.pid.Kp+_TDC_GP21.pid.ki_sum; // 
		 ad603_resualt = AD603_AGC_DEFAULT+pid_resualt;
		 if(ad603_resualt<AD603_AGC_MIN)
		 {
				ad603_resualt= AD603_AGC_MIN;
		 }
		 else if(ad603_resualt>AD603_AGC_MAX)
		 {
		    ad603_resualt= AD603_AGC_MAX;
		 }
	 #if  Debug_Pid
      tlc5618_writeBchannal(ad603_resualt);  
 		#endif 
   
   return  ad603_resualt;
}


uint16_t tdc_agc_Default_control(uint16_t nowData,int16_t setPoint)
{
	int16_t pid_resualt=0,ad603_resualt=0;	 
   int16_t error_t=0;   //error  setpoint- input
	 error_t = setPoint - nowData; 
	
		//error_t =error_t/10;
		_TDC_GP21.pid.ki_sum+=error_t*0.5;
	  pid_resualt = error_t*2+_TDC_GP21.pid.ki_sum; // 
		 ad603_resualt = AD603_AGC_DEFAULT+pid_resualt;
		 if(ad603_resualt<AD603_AGC_MIN)
		 {
				ad603_resualt= AD603_AGC_MIN;
		 }
		 else if(ad603_resualt>AD603_AGC_MAX)
		 {
		    ad603_resualt= AD603_AGC_MAX;
		 }
	 #if  Debug_Pid
      tlc5618_writeBchannal(ad603_resualt);  
 		#endif 
   
   return  ad603_resualt;
}


   /* gp21 intn interrupt callback */
BaseType_t xHigherPriorityTaskWoken = pdTRUE;
uint16_t vol_signal=0,statu_erro=0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
   
  if(GPIO_Pin ==GP21_INTN_Pin )
	{ 
			
		uint32_t gp21_statu_INT;
		gp21_statu_INT = get_gp21_statu();	
    textCount++;  	
		if(gp21_statu_INT & GP21_STATU_CH1)
		{	
			_TDC_GP21.buff[trigCount++] = gp21_read_diatance();//�ռ������������	
      gp21_write(OPC_START_TOF);			
			if(trigCount == DISTANCE_RCV_SIZE) 	
			{
				
				trigCount = 0;
				textCount = 0;
				trighCount = 0;
			  //��ʼ�ɼ���ѹ
			//	xSemaphoreGiveFromISR(tdcSignalSemaphore,&xHigherPriorityTaskWoken); //�ź�
				statu_erro = z_analog_convert(&_TDC_GP21.siganl.vol);
				tdc_rx_voltge_relese();   /*��ѹ�źŲɼ��ͷ�*/
				if(_TDC_GP21.pid.ifTrunOnPid)
				{
						_TDC_GP21.pid_resualt= tdc_agc_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid���Ʒ�ֵ��ѹ
				}
	      gp21_distance_cal(_TDC_GP21.buff,DISTANCE_RCV_SIZE); //���ݴ���
				_TDC_GP21.ifComplete = true;				//����	
		  				
			}
			else
			{
			   _TDC_GP21.statu=trig_time_out;		
			}
			
		}		
	}

}  

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
	  trighCount++;
	}
}