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
extern TIM_HandleTypeDef htim3;

TIM_HandleTypeDef *singhlTim=&htim3;
/*全局变量定义*/ 
#define FIRST_OFFSET  4    //根据实验数据在10m黑板标定下，再补一个正4cm
#define SECOND_OFFSET  0    
#define THIRD_OFFSET  0    
static uint16_t offset_dist[3]={FIRST_OFFSET,SECOND_OFFSET,THIRD_OFFSET};
SqQueue lk_distQueue;  //循环缓存
uint16_t dist_offset=0;  //偏差值
 TIM_HandleTypeDef *z_tlc_TxSignl_pwm= &htim3;
 int textCount =0;
int trighCount=0;
int trigCount = 0;  //触发采集到的次数计数
int erroTimeOutCount = 0;  //tdc 时间超时中断错误标记
bool ifEnoughTrigComplete = false; //采集一次测量数据是否完成
bool ifPick=false;    //是否允许变化,变化超过50cm就为true
 TDC_TRIGSTATU tdc_statu;
 
 //信号量
 SemaphoreHandle_t  tdcSignalSemaphore =NULL;  //创建信号量，用于串口TDC接收完设定的次数后触发控制峰值电压及处理数据
/*任务句柄创建*/
static TaskHandle_t xHandleSerial = NULL;
TaskHandle_t xHandleGp21Trig = NULL;
static TaskHandle_t xHandleSerialDriver = NULL;
 static TaskHandle_t xHandleSensorParam = NULL;
/*函数声明*/
void qc_param_send(void);
void swStandSave(_LK03_STAND index);
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
	paramBuff = structToBytes(&lk_defaultParm);
	flashParam = structToBytes(&lk_flash);
	flash_paramRead(flashParam.point,flashParam.lens); //读取参数
	if(lk_flash.ifHasConfig != 0x01)   //还没有配置
	{		
	  lk_flash = lk_defaultParm;
		lk_flash.ifHasConfig = 0x01;   //代表配置
    flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);		
	}
	else  //已经配置过
	{
	   lk_defaultParm = lk_flash;  //将flash内参数付给默认的参数配置  
	}
#if TEST_QC 
	lk_flash.QC[SECOND_PARAM].ifHavedStand=first_test;
	lk_flash.QC[FIRST_PARAM].ifHavedStand=second_test;
	lk_flash.QC[SECOND_PARAM].qc_stand_dist=DIST_SECOND_OFFSET;
	lk_flash.QC[FIRST_PARAM].qc_stand_dist=DIST_FIRST_OFFSET;
	lk_param_statu.ifContinuDist = true;
	#else 
	lk_param_statu.ifContinuDist = false;
		//lk_flash.QC[SECOND_PARAM].ifHavedStand=true;
#endif	

	
	
  for(;;)
	{
	  if(lk_param_statu.ifParamSave)
		{ 
		 flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
			/*这里应该有flash保存失败的处理！！！*/
		 lk_param_statu.ifParamSave = false;
		}
		if(lk_param_statu.ifParamGet)
		{
			 parmSend(&lk_flash);
		  lk_param_statu.ifParamGet = false;
		}
			for(int i=0;i<LK03_STAND_COUNTS;i++)  //查看需要存储标定的信息
			{
			  if( lk_param_statu.ifQCStand[i]) //标定
		    {
				  _LK03_STAND index =(_LK03_STAND) i;
					 dist_offset=lk_flash.QC[index].qc_stand_dist-offset_dist[index];
					 swStandSave(index);
				}
				if(lk_param_statu.ifQCgetParmReset[i])  //档位切换后再校准
				{
					HIGH_VOL_PARAM index = (HIGH_VOL_PARAM) i;
					 //lk_flash.QC[i].ifHavedStand=false;
					lk_param_statu.ifQCgetParmReset[i]=false;
				   gear_select(&_TDC_GP21.vol_param[index]);   
				}		
			}
    if(lk_param_statu.ifQCgetParm)  //获取标定参数
		{
		    qc_param_send();
			lk_param_statu.ifQCgetParm = false;
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

/*标定选项*/
void swStandSave(_LK03_STAND index)
{
	 _LK03_STAND _lk_standIndex = index;
		lk_flash.QC[_lk_standIndex].qc_stand_dist= lk_defaultParm.QC[_lk_standIndex].qc_stand_dist;  //
		lk_flash.QC[_lk_standIndex].qc_ad603Gain=lk_defaultParm.QC[_lk_standIndex].qc_ad603Gain;
		lk_flash.QC[_lk_standIndex].ifHavedStand=true;   //标定成功
		/*在这里保存flash*/
		flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
		/*这里应该有flash保存失败的处理！！！*/
		lk_param_statu.ifQCStand[_lk_standIndex]= false;
}


void start_singnal(void)
{
		start_txSignl_Tim();   //开始pwm脉冲发射
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);	
		trigOnce();			 
}
/*无校验的的数据发送*/
void noCheckSend(uint8_t *buff, uint8_t len)
{
		uint8_t buffToSend[len+2];
	  for(int i=1;i<len+1;i++)
	{
	  buffToSend[i]=buff[i];	
	}
  buffToSend[0]=0xAA;
	buffToSend[len+1]=0xBB;
	z_serial_write(buffToSend,len+2);   //发送以\r\n结尾的字符			
}

//发送距离，信号电压，增益
uint8_t sendbuf[8]={0};
void send_lk_paramNocheck(void)
{
  sendbuf[0]=0xAA;
	sendbuf[1] = _TDC_GP21.siganl.vol>>8;
	sendbuf[2] = _TDC_GP21.siganl.vol&0xff;
	sendbuf[3] = _TDC_GP21.distance>>8;
	sendbuf[4] = _TDC_GP21.distance&0xff;	
	sendbuf[5] = _TDC_GP21.pid_resualt>>8;
	sendbuf[6] = _TDC_GP21.pid_resualt&0xff;		
  sendbuf[7]=0xEE;

	z_serial_write(sendbuf,8);   //发送以\r\n结尾的字符
}
uint8_t buf[6]={0};
void buff_distTosend(uint16_t dist)
{
  buf[0] = _TDC_GP21.siganl.vol>>8;
	buf[1] = _TDC_GP21.siganl.vol&0xff;
	buf[2] = dist>>8;
	buf[3] = dist&0xff;	
	buf[4] = _TDC_GP21.pid_resualt>>8;
	buf[5] = _TDC_GP21.pid_resualt&0xff;	
  zTF_sendOnceDist(buf,6);	
}

//标定参数信息
uint8_t buf_qc[LK03_STAND_COUNTS*5]={0};
void qc_param_send(void)
{
	for(int i=0;i<LK03_STAND_COUNTS;i++)
	{
		buf_qc[0+5*i]=lk_flash.QC[i].qc_stand_dist >>8;
		buf_qc[1+5*i]=lk_flash.QC[i].qc_stand_dist &0xff;
		buf_qc[2+5*i]=lk_flash.QC[i].qc_ad603Gain >>8;
		buf_qc[3+5*i]=lk_flash.QC[i].qc_ad603Gain &0xff;
		buf_qc[4+5*i]=lk_flash.QC[i].ifHavedStand &0xff;
	}
  QCparmSend(buf_qc,LK03_STAND_COUNTS*5);
}

//平均
uint16_t average=0;
uint16_t lk_average(uint16_t *buff,int len)
{
		volatile uint8_t minIndex=0;
  volatile	uint32_t tem=0,num=0;
 for( int i=0;i<len-1;i++)    // selection sort 
	  {
		     minIndex = i;
			  for( int j=i+1;j<len;j++)
		   	{
					 if(buff[j]<buff[minIndex])     //
					 {
					    minIndex = j;
					 }
				}
				tem= buff[i];
				buff[i] = buff[minIndex];
				buff[minIndex] = tem;
		}		
  for(int i=0;i<len;i++)
		{
		   
		    num += buff[i] ;		
		}
	average = num/len;
		return average;
}
/* USER CODE BEGIN Header_SerialTask */
/**
* @brief Function implementing the z_serial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SerialTask */
RUN_STATU lk_statu;
uint16_t disPlayDistBufer[100]={0};
uint8_t counts_index=0,QE_FLASG;
int queue_lenth=0;
#define AVERAGE_SIZE 80U

//uint16_t first_dist = 2000;    //第一档2000

void SerialTask(void  *argument)
{
  z_tiny_test();
	 QueueInit(&lk_distQueue); 

  /* Infinite loop */
  for(;;)
  {
     if(_TDC_GP21.ifComplete)
		 {
			 _TDC_GP21.ifComplete = false;
			 queue_lenth = QueueLength(&lk_distQueue);
			 if( queue_lenth >=AVERAGE_SIZE)
			 {
				 QE_FLASG=0;
					 for(counts_index=0;counts_index<80;counts_index++)
				 {
						if(Queue_pop(&lk_distQueue,&disPlayDistBufer[counts_index]) ==Q_ERROR)
						{
							QE_FLASG=0;
						}
				 }
			 
			 }
       QE_FLASG ++;
			lk_average(&disPlayDistBufer[0],AVERAGE_SIZE);
		 
			 #if DEBUG_DISPLAY
			 if(_TDC_GP21.running_statu==FIRST)
			 {
			   _TDC_GP21.siganl.vol=1000;
			 }
			 else if(_TDC_GP21.running_statu==SECOND)
			 {
			     _TDC_GP21.siganl.vol=2000;
			 }
			 Send_Pose_Data(&_TDC_GP21.siganl.vol,&average,&_TDC_GP21.pid_resualt);
	     //  Send_Pose_Data(&_TDC_GP21.siganl.vol,&_TDC_GP21.distance,&_TDC_GP21.pid_resualt);
       #else
			   buff_distTosend(average);
			 #endif
			 // buff_distTosend();
//   send_lk_paramNocheck();
			 
		switch(_TDC_GP21.running_statu)
		 {
			 case START:
			 {

        if((lk_flash.QC[FIRST_PARAM].ifHavedStand)&(lk_flash.QC[SECOND_PARAM].ifHavedStand))
				{
					 dist_offset=lk_flash.QC[FIRST_PARAM].qc_stand_dist-offset_dist[FIRST_PARAM];
					_TDC_GP21.running_statu = FIRST;
				}
        if(lk_flash.QC[THIRD_PARAM].ifHavedStand)
				{
				
				}				
//			 gear_select(&_TDC_GP21.vol_param[FIRST_PARAM]);  //开机默认第一档位
	//			 _TDC_GP21.running_statu = FIRST;
			 }break;
			 case FIRST:
			 {
				   if(_TDC_GP21.pid_resualt >600)   //第1档增益大于600时 (_TDC_GP21.pid_resualt >620)&(
					 {
						  dist_offset=lk_flash.QC[SECOND_PARAM].qc_stand_dist-offset_dist[SECOND_PARAM];
						  _TDC_GP21.running_statu = SECOND;
						  gear_select(&_TDC_GP21.vol_param[SECOND_PARAM]); 
					 }
				 
			 }break;
			 case SECOND:
			 {
				 	  if((_TDC_GP21.pid_resualt >600)&(_TDC_GP21.distance>5000))   //第2档增益大于600时切换第三档
					 {
						 __HAL_TIM_SET_AUTORELOAD(singhlTim,500);  //设定500us周期
						  gear_select(&_TDC_GP21.vol_param[THIRD_PARAM]);  //
						  _TDC_GP21.running_statu = THIRD;
					 }
					 else if(_TDC_GP21.pid_resualt <280) //切换第一档((_TDC_GP21.pid_resualt <280)&(_TDC_GP21.distance<3500))
					 {
					 
						  dist_offset=lk_flash.QC[FIRST_PARAM].qc_stand_dist-offset_dist[FIRST_PARAM];
					     gear_select(&_TDC_GP21.vol_param[FIRST_PARAM]);  //开机默认第一档位
						  _TDC_GP21.running_statu = FIRST;
					 }
				
			 }break;		 
			 case THIRD:
			 {
						if(_TDC_GP21.pid_resualt <200)  //切换第一档
					 {
					     __HAL_TIM_SET_AUTORELOAD(singhlTim,100);  //设定100us周期
					     gear_select(&_TDC_GP21.vol_param[FIRST_PARAM]);  //开机默认第一档位
						  _TDC_GP21.running_statu = FIRST;
					 }			
       
			 }break;		 	
			 case STYLE:
			 {
			 
			 }break;				 
		 }
			 
  
		}		 
		  osDelay(50);	 
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

	High_Vol_Ctl_on();
	gear_select(&_TDC_GP21.vol_param[FIRST_PARAM]);  //开机默认第一档位 SECOND_PARAM
	
	_TDC_GP21.pid.ifTrunOn = true;  //先关闭pid

  /* Infinite loop */
  for(;;)
  { 
		
    if(trighCount>10)
	 {
	   trighCount = 0;
		static uint32_t gp21_statu_INT;
	//	gp21_statu_INT = get_gp21_statu();	
			 gp21_write(OPC_START_TOF);
			 z_analog_convert(&_TDC_GP21.siganl.vol);
			 if(ifDMAisComplete)
				{
						ifDMAisComplete =false;
						_TDC_GP21.siganl.vol = z_analog_covertDMA ();
				}
				else
				{
						//erro_count_test ++;
				}		 
			tdc_rx_voltge_relese();   /*高压信号采集释放*/
			_TDC_GP21.pid_resualt= tdc_agc_Default_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid控制峰值电压	 		
	 
	 }
   osDelay(1);
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
uint8_t tesr_1=0;
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
		if(_TDC_GP21.pid.ki_sum <-1000)
		{
		   _TDC_GP21.pid.ki_sum=-1000;
		}
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
		 if(ad603_resualt >600)
		 {
		   tesr_1=4;
		 }
	 #if  Debug_Pid
      tlc5618_writeBchannal(ad603_resualt);  
 		#endif 
   
   return  ad603_resualt;
}

float ki_sum=0;
uint16_t tdc_agc_Default_control(uint16_t nowData,int16_t setPoint)
{
//	static int pid_resualt=0,ad603_resualt=0;	 
//   int16_t error_t=0;   //error  setpoint- input
//	 error_t = setPoint - nowData; 
//	
//		//error_t =error_t/10;
//		ki_sum+=error_t*0.5;
//	  pid_resualt = error_t*2+ki_sum; // 
//		 ad603_resualt = AD603_AGC_DEFAULT+pid_resualt;
//	 
//		 if((ad603_resualt<200) |(ad603_resualt<0))
//		 {
//			pid_resualt=	ad603_resualt= 200;
//		 }
//		 else if(ad603_resualt>AD603_AGC_MAX)
//		 {
//		   pid_resualt = ad603_resualt= AD603_AGC_MAX;
//		 }
//	 #if  Debug_Pid
//      tlc5618_writeBchannal(ad603_resualt);  
// 		#endif 
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
		if(_TDC_GP21.pid.ki_sum <-1000)
		{
		   _TDC_GP21.pid.ki_sum=-1000;
		}
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
		 if(ad603_resualt >600)
		 {
		   tesr_1=4;
		 }
	 #if  Debug_Pid
      tlc5618_writeBchannal(ad603_resualt);  
 		#endif 
      
   return  ad603_resualt;
}
   /* gp21 intn interrupt callback */
BaseType_t xHigherPriorityTaskWoken = pdTRUE;
uint16_t vol_signal=0,statu_erro=0,dist_checkLast=0,dist_checkNow;
uint8_t flag=0,erro_count_test=0;

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
			    	z_analog_convert(&_TDC_GP21.siganl.vol); 	
           dist_checkNow = 	gp21_distance_cal(_TDC_GP21.buff,DISTANCE_RCV_SIZE)-lk_flash.QC[0].qc_stand_dist; //数据处理 //	
						if(((dist_checkNow - dist_checkLast)< -500)|((dist_checkNow - dist_checkLast)>500))
						{
						   ifPick=true;
						}else ifPick=false;
				  if(_TDC_GP21.pid.ifTrunOn)
						{
									if(ifDMAisComplete)
									{
											ifDMAisComplete =false;
											_TDC_GP21.siganl.vol = z_analog_covertDMA ();
									}
									else
									{
										_TDC_GP21.siganl.vol = z_analog_covertDMA ();
											erro_count_test ++;
									}
								tdc_rx_voltge_relese();   /*高压信号采集释放*/	
								_TDC_GP21.pid_resualt= tdc_agc_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid控制峰值电压
						}
						dist_checkLast=_TDC_GP21.distance=  gp21_distance_cal(_TDC_GP21.buff,DISTANCE_RCV_SIZE)-dist_offset; //数据处理
						if( Queue_push(&lk_distQueue,_TDC_GP21.distance) ==Q_ERROR)
						{
							 flag = 1; //队列满
						}
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