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
/*pid 控制设置*/


/*全局变量定义*/
 TIM_HandleTypeDef *z_tlc_TxSignl_pwm= &htim3;
 int textCount =0;
int trighCount=0;
int trigCount = 0;  //触发采集到的次数计数
int erroTimeOutCount = 0;  //tdc 时间超时中断错误标记
bool ifEnoughTrigComplete = false; //采集一次测量数据是否完成
 TDC_TRIGSTATU tdc_statu;
 //信号量
 SemaphoreHandle_t  tdcSignalSemaphore =NULL;  //创建信号量，用于串口TDC接收完设定的次数后触发控制峰值电压及处理数据
/*任务句柄创建*/
static TaskHandle_t xHandleSerial = NULL;
TaskHandle_t xHandleGp21Trig = NULL;
static TaskHandle_t xHandleSerialDriver = NULL;
 static TaskHandle_t xHandleSensorParam = NULL;
/*函数声明*/
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
	              LK_sensorParamTask,    //任务函数
	              "LK_sensorParamTask",  //任务名
								128,          //任务栈大小，也就是4个字节
	              NULL,
								1,            //任务优先级
								&xHandleSensorParam
	            );	
	xTaskCreate(
	             SerialTask,    //任务函数
	             "SerialTask",  //任务名
								128,          //任务栈大小，也就是4个字节
	              NULL,
								2,            //任务优先级
								&xHandleSerial
	            );	
	
	xTaskCreate(
	             z_serialDriverTask,  //任务函数
	             "serialDriverTask",  //任务名
								128,                //任务栈大小，也就是4个字节
	              NULL,
								3,                  //任务优先级
								&xHandleSerialDriver
	            );

	xTaskCreate(
	             Gp21TrigTask,    //任务函数
	             "Gp21TrigTask",  //任务名
								512,            //任务栈大小，也就是4个字节
	              NULL, 
								4,                //任务优先级
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
	flash_paramRead(flashParam.point,flashParam.lens); //读取参数
	if(lk_flash.ifHasConfig != 0x01)   //还没有配置
	{	
		lk_flash.ifHasConfig = 0x01;   //代表配置
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
		if(lk_param_statu.ifQCStand) //标定
		{
			start_singnal();
		  lk_param_statu.ifQCStand = false;
		}
    if(lk_param_statu.ifGetOnceDist)	//单次测量
		{
		
		}
    if(lk_param_statu.ifContinuDist)	//连续测量
		{
			start_txSignl_Tim();   //开始pwm脉冲发射
			HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);	
			trigOnce();				
			lk_param_statu.ifContinuDist=false;
		}		
	 osDelay(500);
	}
}

void start_singnal(void)
{
		start_txSignl_Tim();   //开始pwm脉冲发射
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
  tdc_board_init();   /*初始化激光板*/
	gear_select(&_TDC_GP21.vol_param[FIRST_PARAM]);  //开机默认第一档位
	start_singnal();
  /* Infinite loop */
  for(;;)
  { 
		
    if(trighCount>10)
	 {
	   trighCount = 0;
		  gp21_write(OPC_START_TOF);

	 	z_analog_convert(&_TDC_GP21.siganl.vol);
		tdc_rx_voltge_relese();   /*高压信号采集释放*/
		_TDC_GP21.pid_resualt= tdc_agc_Default_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid控制峰值电压	 
	 }
   osDelay(100);
  }
   
  /* USER CODE END Gp21TrigTask */
}

void trigOnce(void)
{
	gp21_write(OPC_START_TOF);
  gp21_en_startSignal();	  //使能开始信号
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
    _TDC_GP21.buff[trigCount++] = gp21_read_diatance();//收集激光测量数据		
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
if(gp21_statu_INT & GP21_STATU_TIMEOUT)  //超出时间测量
	{
		  erroTimeOutCount ++ ;
      closeTdc();		
		return trig_time_out;
	}	
	return false;
}


/*采集到足够数据后开始数据处理*/
void trigEnough(void)
{

	tdc_rx_voltge_relese();   /*高压信号采集释放*/
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
			_TDC_GP21.buff[trigCount++] = gp21_read_diatance();//收集激光测量数据	
      gp21_write(OPC_START_TOF);			
			if(trigCount == DISTANCE_RCV_SIZE) 	
			{
				
				trigCount = 0;
				textCount = 0;
				trighCount = 0;
			  //开始采集电压
			//	xSemaphoreGiveFromISR(tdcSignalSemaphore,&xHigherPriorityTaskWoken); //信号
				statu_erro = z_analog_convert(&_TDC_GP21.siganl.vol);
				tdc_rx_voltge_relese();   /*高压信号采集释放*/
				if(_TDC_GP21.pid.ifTrunOnPid)
				{
						_TDC_GP21.pid_resualt= tdc_agc_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid控制峰值电压
				}
	      gp21_distance_cal(_TDC_GP21.buff,DISTANCE_RCV_SIZE); //数据处理
				_TDC_GP21.ifComplete = true;				//结束	
		  				
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