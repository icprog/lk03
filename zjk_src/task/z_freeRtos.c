/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
//zjk include
#include "z_include.h"
 typedef enum{ trig_onece_complete =1,trig_enough_complete,trig_time_out} TDC_TRIGSTATU;
/*全局变量定义*/
int trigCount = 0;  //触发采集到的次数计数
int erroTimeOutCount = 0;  //tdc 时间超时中断错误标记
bool ifEnoughTrigComplete = false; //采集一次测量数据是否完成
 TDC_TRIGSTATU tdc_statu;
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

uint16_t tdc_agc_control(void);
void trigEnough(void);
TDC_TRIGSTATU trigGetData(void);
void trigOnce(void);

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
	 osDelay(500);
	}
}

/* USER CODE BEGIN Header_SerialTask */
/**
* @brief Function implementing the z_serial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SerialTask */
void SerialTask(void  *argument)
{
  z_tiny_test();
	
  /* Infinite loop */
  for(;;)
  {
     if(_TDC_GP21.isGp21Complete)
		 {
			 _TDC_GP21.isGp21Complete = 0;
		   Send_Pose_Data(&_TDC_GP21.rev_siganl.vol,&_TDC_GP21.tdc_distance,&_TDC_GP21.tlc_resualt);
//       uint8_t *sendBuf =(uint8_t*)(&_TDC_GP21.tdc_distance);
//			 zTF_sendOnceDist(sendBuf,2);			 
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

void Gp21TrigTask(void *argument)
{

  tdc_board_init();   /*初始化激光板*/
//	if((lk_param_statu.ifGetOnceDist == false) ||(lk_param_statu.ifContinuDist ==false))
//	{
//		 vTaskSuspend(NULL);  //任务挂起如果没有命令接收到
//	}
	lk_param_statu.ifContinuDist = true;
  /* Infinite loop */
  for(;;)
  {
 		  while(gp21_read_intn() == GPIO_PIN_SET)
				{
           trigOnce();
					 //parmSend(&lk_parm);
					_TDC_GP21.statu.txSignalCnt++;
					if(_TDC_GP21.statu.txSignalCnt>=2)
					{
						tdc_agc_control();
					}
					tdc_delay(500); 
				};	
			 _TDC_GP21.statu.txSignalCnt =0;
       tdc_statu = trigGetData(); //收集数据		
    			
        if(tdc_statu == trig_enough_complete)
				{
				  _TDC_GP21.tlc_resualt= tdc_agc_control(); 	
				   trigEnough();  //处理数据
					if(lk_param_statu.ifGetOnceDist)   //单次测量
					{
						  uint8_t *sendBuf =(uint8_t*)(&_TDC_GP21.tdc_distance);
						 zTF_sendOnceDist(sendBuf,2);
						 lk_param_statu.ifGetOnceDist = false;
						 vTaskSuspend(NULL);
					}
					if(lk_param_statu.ifContinuDist)   //多次测量
					{
						//  uint8_t *sendBuf =(uint8_t*)(&_TDC_GP21.tdc_distance);
						// zTF_sendOnceDist(sendBuf,2);
						osDelay(1);
					}					
				}	     
         				 	 
	 }

  /* USER CODE END Gp21TrigTask */
}

void trigOnce(void)
{
	gp21_write(OPC_START_TOF);					
	gp21_en_stop1Signal();	
	gp21_startOneSignal();	/*trig a start signal*/		
}

TDC_TRIGSTATU trigGetData(void)
{
	uint32_t gp21_statu_INT;

  gp21_statu_INT = get_gp21_statu();	 
 	
	if(gp21_statu_INT & GP21_STATU_CH1)
	{
		gp21_close_stop1Signal();		
    _TDC_GP21.gp21_distance[trigCount++] = gp21_read_diatance();//收集激光测量数据		
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
		return trig_time_out;
	}	
	return false;
}


/*采集到足够数据后开始数据处理*/
void trigEnough(void)
{
	trigCount = 0;
	tdc_rx_voltge_relese();   /*高压信号采集释放*/
	gp21_distance_cal(_TDC_GP21.gp21_distance,DISTANCE_RCV_SIZE);
	_TDC_GP21.isGp21Complete = 1;		
}
/*AGC Control
@input: input voltage feedback value, unit mv


@return control value AGC AD603
 */
#define LIMIT_VOL_NOcTL   20     /*-20 ~ 20mv NO control*/
uint16_t tdc_agc_control(void)
{
	int16_t pid_resualt=0,ad603_resualt=0;
  _TDC_GP21.rev_siganl.vol_statu = z_analog_convert(&_TDC_GP21.rev_siganl.vol);		/*vologe convert*/
		
	  if(_TDC_GP21.rev_siganl.vol_statu == Z_ANALOG_ERRO)
			{
			  _TDC_GP21. z_analog_erro_cnt++;		
			}	
	 
   int16_t error_t=0;   //error  setpoint- input
   error_t = _TDC_GP21.pid.setpoint - _TDC_GP21.rev_siganl.vol; 
		
// if((error_t>-LIMIT_VOL_NOcTL)	&& (error_t < LIMIT_VOL_NOcTL))
// {
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
	 //if (_TDC_GP21.rev_siganl.vol < LIMIT_SIGNAL_VOL )	return ad603_resualt;	 
    tlc5618_writeBchannal(ad603_resualt); 
//	 }		 
   return  ad603_resualt;
}


   /* gp21 intn interrupt callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
  
  if(GPIO_Pin ==GP21_INTN_Pin )
	{ 
		 tdc_statu = trigGetData(); 	 //收集采集到的测量数据		
	}

}  


typedef enum{FIRST=1,SECOND,THIRED} GEAR_TRIG_;

void z_Gear_Selection(void)
{
	  GEAR_TRIG_ GEAR_LK;
    switch(GEAR_LK)
		{
		  case FIRST:
			{
			
			}break;
			case SECOND:
			{
			
			}break;
			case THIRED:
			{
			
			
			}break;
		
		}


}
