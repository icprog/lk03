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
#include "z_checksum.h"
#include "z_sensor_ack.h"
//
#include "z_include.h"

//
#include "z_os.h"
extern TIM_HandleTypeDef htim3;

TIM_HandleTypeDef *singhlTim=&htim3;
/*全局变量定义*/ 
#define FIRST_OFFSET  4    //根据实验数据在10m黑板标定下，再补一个正4cm
#define SECOND_OFFSET  0    
#define THIRD_OFFSET  0  

sensor_struct_typ sensor_running_vaile;
sensor_runnig_cmd_typEnum sensor_runnig_cmd =sensor_idle;     //当前运行的命令
extern sensor_struct_ sensor_strct;    //传感器结构
uint16_t ms2_erro=0,ms1_err0=0;   //超时计数
uint8_t flag=0,erro_count_test=0,reg_index;
static uint16_t offset_dist[3]={FIRST_OFFSET,SECOND_OFFSET,THIRD_OFFSET};
TypedSelextMode slect_mode;   //开机后模式选择
SqQueue lk_distQueue;  //循环缓存
bool  ifDebug_strShow=false;   //调试信息显示
bool  ifUser_strShow=false;		 //用户信息显示
bool  ifLkProtecl_show=false;  //协议显示

 TIM_HandleTypeDef *z_tlc_TxSignl_pwm= &htim3;
 int textCount =0;
int trighCount=0;
int trigCount = 0;  //触发采集到的次数计数
int erroTimeOutCount = 0;  //tdc 时间超时中断错误标记
bool ifEnoughTrigComplete = false; //采集一次测量数据是否完成
bool ifPick=false;    //是否允许变化,变化超过50cm就为true
 TDC_TRIGSTATU tdc_statu;
bool mes1_have_sighal=false,mes2_have_sighal=false,ifFirstStart=false;
 //信号量
 SemaphoreHandle_t  tdcSignalSemaphore =NULL;  //创建信号量，用于串口TDC接收完设定的次数后触发控制峰值电压及处理数据
/*任务句柄创建*/
static TaskHandle_t xHandleSerial = NULL;
TaskHandle_t xHandleGp21Trig = NULL;
static TaskHandle_t xHandleSerialDriver = NULL;
 static TaskHandle_t xHandleSensorParam = NULL;
 TaskHandle_t xHandleDataOutFeq = NULL;
/*函数声明*/
//标定信息复位
void user_dispaly_enable();/*用户信息输出显示使能*/
void debug_display_enable();/*调试信息输出使能*/
void lk_protecl_enable();/*协议信息输出使能*/

void sensor_powerOn_flashParamCfg(void);
void sensor_distOffset_calculate(_sensor_gesr_enum index);
void flasStandReset(_sensor_gesr_enum index);
void selected_mesg_mode(TypedSelextMsgMode mode);
void select_mode_ifStart(TypedSelextMode mode);
void lk_cmdAck(uint8_t type,uint8_t id,bool ack);
void qc_param_send(void);
void swStandSave(_sensor_gesr_enum index);
void SerialTask(void  * argument);
void Gp21TrigTask(void  * argument);
 void LK_sensorParamTask(void *argument);
 void lk_sensor_outData_Task(void *argument);
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
	              lk_sensor_outData_Task,    //任务函数
	              "lk_outData_freq",  //任务名
								128,          //任务栈大小，也就是4个字节
	              NULL,
								3,            //任务优先级
								&xHandleDataOutFeq
	            );		
	
	xTaskCreate(
	             z_serialDriverTask,  //任务函数
	             "serialDriverTask",  //任务名
								128,                //任务栈大小，也就是4个字节
	              NULL,
								4,                  //任务优先级
								&xHandleSerialDriver
	            );

	xTaskCreate(
	             Gp21TrigTask,    //任务函数
	             "Gp21TrigTask",  //任务名
								512,            //任务栈大小，也就是4个字节
	              NULL, 
								5,                //任务优先级
								&xHandleGp21Trig
	            );


}
arrayByte_ paramBuff;
arrayByte_ flashParam;
uint8_t text_debug=0x00;
extern void test_send_cmd(void);
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
	   //lk_defaultParm = lk_flash;  //将flash内参数付给默认的参数配置  
	}
	
#if TEST_QC    //模拟校准测试已经通过
		lk_flash.QC[SECOND_PARAM].ifHavedStand=true;
		lk_flash.QC[FIRST_PARAM].ifHavedStand=true;
		lk_flash.QC[THIRD_PARAM].ifHavedStand=true;		
		lk_flash.QC[SECOND_PARAM].qc_stand_dist=DIST_SECOND_OFFSET;
		lk_flash.QC[FIRST_PARAM].qc_stand_dist=DIST_FIRST_OFFSET;
		lk_flash.QC[THIRD_PARAM].qc_stand_dist=DIST_THIED_OFFSET;	
	  lk_param_statu.ifContinuDist = true;
#else 
 
    sensor_powerOn_flashParamCfg();
	 //test 
 //   sensor_strct.cmd = dist_continue_ack_cmd;
 //  ifDebug_strShow = true;

	//sensor_ouput_switch_high();
#endif	
  for(;;)
	{
	//	test_send_cmd();
     sensor_struct_loop(&sensor_strct); 
	 osDelay(500);
	}
}

/*输出外部开关量*/
void snesor_ouput_switch(uint16_t dist)
{
	 if((dist<sensor_running_vaile.front_switch*100)&&(dist>sensor_running_vaile.back_switch*100))
	 {     
	   sensor_ouput_switch_high();
	 }
   else
	 {
	   sensor_ouput_switch_low();
	 }
}


void sensor_distOffset_calculate(_sensor_gesr_enum index)
{
	_sensor_gesr_enum select_stande =index;
	
	if(sensor_running_vaile.dist_base == 1)  //前基准
	{
		sensor_running_vaile.dist_sensor_lenth = SENSOR_LENGTH;
	}
	else
	{
		sensor_running_vaile.dist_sensor_lenth = 0;
	}
 sensor_running_vaile.dist_offset = lk_flash.QC[select_stande].qc_stand_dist;	
}
/*加载flash 后 自动配置参数*/
void sensor_powerOn_flashParamCfg(void)
{
  sensor_baudRate_typeEnum baud_selet = (sensor_baudRate_typeEnum )lk_flash.baud_rate;
  baudRateCfg_select(baud_selet);  //波特率设置

  for(int i=0;i<3;i++)
	{
	   sensor_running_vaile.qc_offset[i]=lk_flash.QC[i].qc_stand_dist;//
	}
	
	sensor_running_vaile.dist_base = lk_flash.front_or_base;
	sensor_running_vaile.front_switch = lk_flash.front_limit_trigger;
	sensor_running_vaile.back_switch = lk_flash.back_limit_trigger;
	if(lk_flash.outFreq !=0)
	{
		sensor_running_vaile.output_freq = (1000/lk_flash.outFreq);   //hz
	}
	else 
	{
	  sensor_running_vaile.output_freq = 1000;
	}
	sensor_distOffset_calculate(lk03_first_gears);	//偏差值计算
	if(lk_flash.autoRunMode == 1)  //自动运行
	{
	   sensor_strct.cmd = dist_continue_ack_cmd;
	}
	else
	{
	   sensor_strct.cmd = sensor_idle;
	}
	
	if((lk_flash.QC[lk03_first_gears].ifHavedStand)&(lk_flash.QC[lk03_second_gears].ifHavedStand)&(lk_flash.QC[lk03_third_gears].ifHavedStand))  //全部标定完
  {
	   _TDC_GP21.sensor_runMode = normal_typeMode;  //正常模式
	}
	else _TDC_GP21.sensor_runMode = qc_typeMode; //标定模式
	
	if(lk_flash.displayMode == 1)
	{
	  user_dispaly_enable();
	}
	else if(lk_flash.displayMode == 2)
	{
	  lk_protecl_enable();
	
	}
}

void start_singnal(void)
{
		start_txSignl_Tim();   //开始pwm脉冲发射
		trigOnce();
}

//平均
uint16_t average=0;
uint32_t tem=0,num=0;
uint16_t lk_average(uint16_t *buff,int len)
{
  volatile uint8_t minIndex=0;
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

	 tem=0,num=0;
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
void SerialTask(void  *argument)
{
  z_tiny_test();
  QueueInit(&lk_distQueue); 
  /* Infinite loop */
  for(;;)
  { 			
		switch(_TDC_GP21.system_statu.running_statu)   //档位切换状态
		 {
			case IDLE:
			{
   
					
			}break;				
			 case START:
			 {

					 if((lk_flash.QC[lk03_first_gears].ifHavedStand)&(lk_flash.QC[lk03_second_gears].ifHavedStand)&(lk_flash.QC[lk03_third_gears].ifHavedStand))  //全部标定完才挡位切换
					{
						 lk_gear_switch(lk03_first_gears);
						_TDC_GP21.system_statu.running_statu = FIRST;
					}
				
			 }break;
			 case FIRST:
			 {
				   if(_TDC_GP21.pid_resualt >550 ) //第1档增益大于600时 (_TDC_GP21.pid_resualt >620)&(
					 {
						   lk_gear_switch(lk03_second_gears);
						 _TDC_GP21.system_statu.running_statu = SECOND;
					 }
				 
			 }break;
			 case SECOND:
			 {
				 	  if(_TDC_GP21.pid_resualt >550)   //第2档增益大于600时切换第三档
					 {
						  lk_gear_switch(lk03_third_gears);
						 _TDC_GP21.system_statu.running_statu = THIRD;
					 }
					 else if(_TDC_GP21.pid_resualt <280) //切换第一档((_TDC_GP21.pid_resualt <280)&(_TDC_GP21.distance<3500))
					 {
						   lk_gear_switch(lk03_first_gears);
						  _TDC_GP21.system_statu.running_statu = FIRST;
					 }
				
			 }break;		 
			 case THIRD:
			 {
						if(_TDC_GP21.pid_resualt <400)  //切换第2档
					 {
					  	 lk_gp21MessgeMode_switch(GP21_MESSGE1);
					     lk_gear_switch(lk03_second_gears);  //第2档
						  _TDC_GP21.system_statu.running_statu = SECOND;
					 }
					 	if(_TDC_GP21.distance >30000)  //大于300米
					 {
						 lk_gp21MessgeMode_switch(GP21_MESSGE2);
						  _TDC_GP21.system_statu.running_statu = LONG_DISTANCE_MODE;
					 }					 			    
			 }break;		
			 case LONG_DISTANCE_MODE:
			 {			 
				 		if(_TDC_GP21.distance <20000)  //小于200米
					 {
						 _TDC_GP21.system_statu.running_statu = THIRD;
					 }	 	 
			 }break;			 
			 case STYLE:
			 {			 
				 select_mode_ifStart(slect_mode);		 	 
			 }break;				 
		 }	
         osDelay(50);
		}		 
		 	 
	 
		
  /* USER CODE END SerialTask */
}

extern char *runTypeModeString;

/*数据输出任务*/
void lk_sensor_outData_Task(void *argument)
{

  vTaskSuspend(xHandleDataOutFeq);   //挂起任务
	
	for(;;)
	{
		if((_TDC_GP21.ifDistanceNull==false)&(_TDC_GP21.ifMachineFine))
		 {
			 if(ifDebug_strShow)
			 {
				 zt_printf("d: %d, s:%d pid: %d gs:%d  md:%s\r\n",_TDC_GP21.average,_TDC_GP21.siganl.vol,_TDC_GP21.pid_resualt,_TDC_GP21.system_statu.cureent_gear,runTypeModeString);
			 }
			 else if(ifUser_strShow)
			 {
			   zt_printf("%d cm\r\n",_TDC_GP21.average);
			 }
			 else if(ifLkProtecl_show)
			 {
         sensor_distContinu_ack(_TDC_GP21.average); 	 
			 }			
		 } 
		 else if((_TDC_GP21.ifDistanceNull==true)&(_TDC_GP21.ifMachineFine))
		 {
		     
			 if(ifDebug_strShow)
			 {
				  zt_printf("d: %d, s:%d pid: %d  gears:%d\r\n",_TDC_GP21.distance_inl,_TDC_GP21.siganl.vol,_TDC_GP21.pid_resualt,_TDC_GP21.system_statu.cureent_gear);
			 }
			 else if(ifUser_strShow)
			 {
			   zt_printf("null\r\n",_TDC_GP21.average);
			 }
			 else if(ifLkProtecl_show) //数据无效显示
			 {
          zTF_NullDistAck();
			 }			
		 }
		osDelay(sensor_running_vaile.output_freq);
	}
}





bool ifStartCplet=false;
void select_mode_ifStart(TypedSelextMode mode)
{
  TypedSelextMode m=mode;
	switch(m)
	{
	  case first_mes1:
		{
		    if(ms2_erro>0)
				{
				  ms2_erro = 0;
				  __HAL_TIM_SET_AUTORELOAD(singhlTim,100);  //设定100us周期
				  gear_select(&_TDC_GP21.system_statu.high_value_defconfg[lk03_first_gears]);  
	              lk_gp21MessgeMode_switch(GP21_MESSGE1); 
				  slect_mode=first_mes2;
				  lk_gp2x_write(OPC_START_TOF);
				}
				else if(mes2_have_sighal)
				{
				  _TDC_GP21.system_statu.running_statu=THIRD;
				}
				
		}break;
	  case first_mes2:
		{
  	    
	     if(mes1_have_sighal)
				{
					  _TDC_GP21.system_statu.running_statu = START;
					  ifStartCplet = true;
				}	  
		
		}break;	
	}


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
uint16_t text_dist=0;

void Gp21TrigTask(void *argument)
{ 	
  tdcSignalSemaphore = xSemaphoreCreateBinary();	 //
  tdc_board_init();   /*初始化激光板*/
//	High_Vol_Ctl_on();
	_TDC_GP21.pid.ifTrunOn = true;  //pid打开
	lk_gear_switch(lk03_first_gears);  //开机默认第一档配置
	lk_gp21MessgeMode_switch(GP21_MESSGE1);//近距离模式
	selected_mesg_mode(msg_qcStard);
   
  /* Infinite loop */
  for(;;)
  { 		
		  if((ifFirstStart)&(ifStartCplet))
			 {
				  osDelay(200);
				 _TDC_GP21.ifMachineFine=true;
				  ifFirstStart = false;
			 }
			
			 osDelay(100);
	}
   
  /* USER CODE END Gp21TrigTask */
}

void system_runStatu_select(_sensor_cureent_typeMode rmod)
{
	_sensor_cureent_typeMode mode = rmod;
	switch(mode)
	{
		case debug_typeMode :  //调试模式
		{
			_TDC_GP21.system_statu.running_statu = IDLE;
			ifFirstStart = true;
			ifStartCplet = true;
		}break;
		case qc_typeMode :  //标定模式
		{
			_TDC_GP21.system_statu.running_statu = IDLE;
			ifFirstStart = true;
			ifStartCplet = true; 				
		}break;
		case normal_typeMode :  //正常模式
		{			
			_TDC_GP21.system_statu.running_statu = FIRST;
			ifFirstStart = true;
			ifStartCplet = true; 			
		}break;		
	}
}


//模式选择
void selected_mesg_mode(TypedSelextMsgMode mode)
{
   TypedSelextMsgMode slect_index=mode;
   switch(slect_index)
	 {
		 case msg_thirdStard:
		 {
			 __HAL_TIM_SET_AUTORELOAD(singhlTim,500);  //设定500us周期
			 gear_select(&_TDC_GP21.system_statu.high_value_defconfg[lk03_third_gears]);  //			
			 lk_gp21MessgeMode_switch(GP21_MESSGE2);
			 _TDC_GP21.system_statu.running_statu=STYLE;
			 slect_mode = first_mes1;  
			 ifFirstStart = true;
			 ifStartCplet = false;
		 }break;
		 case msg_firstStard:
		 {
			gear_select(&_TDC_GP21.system_statu.high_value_defconfg[lk03_first_gears]);  //			
			lk_gp21MessgeMode_switch(GP21_MESSGE1);
			_TDC_GP21.system_statu.running_statu=FIRST;
			ifFirstStart = true;
			ifStartCplet = true; 
		 }break;
		 case msg_qcStard:
		 {
		 	lk_gear_switch(lk03_first_gears);  //
			// lk_gear_switch(lk03_second_gears);  //
			lk_gp21MessgeMode_switch(GP21_MESSGE1);
			_TDC_GP21.system_statu.running_statu=START;
			ifFirstStart = true;
			ifStartCplet = true; 
		 }break;			 
	 }


}

/*用户信息输出显示使能*/
void user_dispaly_enable()
{
	  ifDebug_strShow=false;   //调试信息显示
	  ifUser_strShow=true;		 //用户信息显示
	  ifLkProtecl_show=false;  //协议显示

}
/*调试信息输出使能*/
void debug_display_enable()
{
	  ifDebug_strShow=true;   //调试信息显示
	  ifUser_strShow=false;		 //用户信息显示
	  ifLkProtecl_show=false;  //协议显示

}
/*协议信息输出使能*/
void lk_protecl_enable()
{
	  ifDebug_strShow=false;   //调试信息显示
	  ifUser_strShow=false;		 //用户信息显示
	  ifLkProtecl_show=true;  //协议显示
}

void trigOnce(void)
{
	lk_gp2x_write(OPC_START_TOF);
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
    gp21_statu_INT = lk_gp2x_read_regStatu();	 
	if(gp21_statu_INT & GP21_STATU_CH1)
	{
		closeTdc();		
       _TDC_GP21.buff[trigCount++] = lk_gp2x_read_regResult(0);//收集激光测量数据		
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
		 	if(_TDC_GP21.pid.ki_sum > 1000)
		{
		   _TDC_GP21.pid.ki_sum=1000;
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
#define AVERAGE_TIMES 30
void lk_distance_average(uint16_t dist)
{
  static uint8_t count=0;
  static float l_dist=0,r_dist=0;
	count++;
	l_dist+=dist;
	if(count  >= AVERAGE_TIMES)
	{
		count =0;
	   _TDC_GP21.average= l_dist/AVERAGE_TIMES;
	  _TDC_GP21.ifAverageComplete = true;
	 l_dist =0;
	}
    else _TDC_GP21.ifAverageComplete = false;
 
}
   /* gp21 intn interrupt callback */
BaseType_t xHigherPriorityTaskWoken = pdTRUE;
uint16_t vol_signal=0,statu_erro=0;
uint32_t GP21_REG;
uint32_t gp21_statu_INT;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
   int distance=0;
  if(GPIO_Pin ==GP21_INTN_Pin )
	{ 
			
		gp21_statu_INT = lk_gp2x_read_regStatu();	
    textCount++; 
    reg_index=gp21_statu_INT&0x03; //取结果寄存器地址
		GP21_REG = lk_gp2x_read_regResult(0);//收集激光测量数据  
		if(gp21_statu_INT &0x400)
		{
		   ms2_erro++;
		}
		if(gp21_statu_INT &0x200)
		{
		   ms1_err0++;
		}
		trigCount++;
  if(((gp21_statu_INT &0x400)==false)&&((gp21_statu_INT &0x200)==false)&&(gp21_statu_INT!=0)) //无效值
			{		
				if(_TDC_GP21.ifMachineFine)  //机器正常运行情况下
				{
				  _TDC_GP21.buff[trigCount-1] = GP21_REG;//收集激光测量数据
				}
				lk_gp2x_write(OPC_INIT);			
				if(trigCount == DISTANCE_RCV_SIZE) 	
				{				
					trigCount = 0;
					textCount = 0;
					trighCount = 0;
                    erro_count_test=0; 
					 _TDC_GP21.tdc_gp2x.status=trig_onece_complete;
					if(ifFirstStart)
					{
							if(_TDC_GP21.tdc_gp2x.measure_mode==GP21_MESSGE1)
							{
								mes1_have_sighal=true;
							}
							if(_TDC_GP21.tdc_gp2x.measure_mode==GP21_MESSGE2)
							{
								mes2_have_sighal=true;
							}							
					}
					//开始采集电压
							z_analog_convert(&_TDC_GP21.siganl.vol); 	
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
										}
									tdc_rx_voltge_relese();   /*高压信号采集释放*/	
									_TDC_GP21.pid_resualt= tdc_agc_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid控制峰值电压
							}
							if(_TDC_GP21.ifMachineFine)  //机器正常运行情况下
							{
								distance = gp21_distance_cal(_TDC_GP21.buff,DISTANCE_RCV_SIZE)-sensor_running_vaile.dist_offset-sensor_running_vaile.dist_sensor_lenth;
								if(distance>=0)
								{
									_TDC_GP21.ifDistanceNull=false;
									_TDC_GP21.distance=distance;
								}else 
								{ 
									_TDC_GP21.ifDistanceNull=true;
									_TDC_GP21.distance_inl = distance;
								}  //数据小于偏差值数据无效
								if(_TDC_GP21.distance>500)
								{
								  _TDC_GP21.ifComplete = false;
								}
								lk_distance_average(_TDC_GP21.distance);
								_TDC_GP21.ifComplete = true;				//结束											
							}
				}	//接收完足够数据
			}
			else  //无效值
	   	{
				erro_count_test ++;	 
				_TDC_GP21.tdc_gp2x.status=trig_time_out;			
				z_analog_convert(&_TDC_GP21.siganl.vol); 					  
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
					   if(trigCount == DISTANCE_RCV_SIZE) 	
						{				
							  trigCount = 0;
							  tdc_rx_voltge_relese();   /*高压信号采集释放*/	
							  _TDC_GP21.pid_resualt= tdc_agc_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid控制峰值电压
						}
					}	//endif _TDC_GP21.pid.ifTrunOn
	          lk_gp2x_write(OPC_INIT);						
		}				

	}
}  

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
	 // trighCount++;
	}
}

