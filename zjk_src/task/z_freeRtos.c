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
extern TIM_HandleTypeDef htim3;

TIM_HandleTypeDef *singhlTim=&htim3;
/*ȫ�ֱ�������*/ 
#define FIRST_OFFSET  4    //����ʵ��������10m�ڰ�궨�£��ٲ�һ����4cm
#define SECOND_OFFSET  0    
#define THIRD_OFFSET  0  

sensor_struct_typ sensor_running_vaile;
sensor_runnig_cmd_typEnum sensor_runnig_cmd =sensor_idle;     //��ǰ���е�����
extern sensor_struct_ sensor_strct;    //�������ṹ
uint16_t ms2_erro=0,ms1_err0=0;   //��ʱ����
uint8_t flag=0,erro_count_test=0,reg_index;
static uint16_t offset_dist[3]={FIRST_OFFSET,SECOND_OFFSET,THIRD_OFFSET};
TypedSelextMode slect_mode;   //������ģʽѡ��
SqQueue lk_distQueue;  //ѭ������

 TIM_HandleTypeDef *z_tlc_TxSignl_pwm= &htim3;
 int textCount =0;
int trighCount=0;
int trigCount = 0;  //�����ɼ����Ĵ�������
int erroTimeOutCount = 0;  //tdc ʱ�䳬ʱ�жϴ�����
bool ifEnoughTrigComplete = false; //�ɼ�һ�β��������Ƿ����
bool ifPick=false;    //�Ƿ�����仯,�仯����50cm��Ϊtrue
 TDC_TRIGSTATU tdc_statu;
bool mes1_have_sighal=false,mes2_have_sighal=false,ifFirstStart=false;
 //�ź���
 SemaphoreHandle_t  tdcSignalSemaphore =NULL;  //�����ź��������ڴ���TDC�������趨�Ĵ����󴥷����Ʒ�ֵ��ѹ����������
/*����������*/
static TaskHandle_t xHandleSerial = NULL;
TaskHandle_t xHandleGp21Trig = NULL;
static TaskHandle_t xHandleSerialDriver = NULL;
 static TaskHandle_t xHandleSensorParam = NULL;
 TaskHandle_t xHandleDataOutFeq = NULL;
/*��������*/
//�궨��Ϣ��λ
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
	              lk_sensor_outData_Task,    //������
	              "lk_outData_freq",  //������
								128,          //����ջ��С��Ҳ����4���ֽ�
	              NULL,
								3,            //�������ȼ�
								&xHandleDataOutFeq
	            );		
	
	xTaskCreate(
	             z_serialDriverTask,  //������
	             "serialDriverTask",  //������
								128,                //����ջ��С��Ҳ����4���ֽ�
	              NULL,
								4,                  //�������ȼ�
								&xHandleSerialDriver
	            );

	xTaskCreate(
	             Gp21TrigTask,    //������
	             "Gp21TrigTask",  //������
								512,            //����ջ��С��Ҳ����4���ֽ�
	              NULL, 
								5,                //�������ȼ�
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
	flash_paramRead(flashParam.point,flashParam.lens); //��ȡ����
	if(lk_flash.ifHasConfig != 0x01)   //��û������
	{		
	  lk_flash = lk_defaultParm;
		lk_flash.ifHasConfig = 0x01;   //��������
    flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);		
	}
	else  //�Ѿ����ù�
	{
	   //lk_defaultParm = lk_flash;  //��flash�ڲ�������Ĭ�ϵĲ�������  
	}
#if TEST_QC    //ģ��У׼�����Ѿ�ͨ��
		lk_flash.QC[SECOND_PARAM].ifHavedStand=true;
		lk_flash.QC[FIRST_PARAM].ifHavedStand=true;
		lk_flash.QC[THIRD_PARAM].ifHavedStand=true;		
		lk_flash.QC[SECOND_PARAM].qc_stand_dist=DIST_SECOND_OFFSET;
		lk_flash.QC[FIRST_PARAM].qc_stand_dist=DIST_FIRST_OFFSET;
		lk_flash.QC[THIRD_PARAM].qc_stand_dist=DIST_THIED_OFFSET;	
	  lk_param_statu.ifContinuDist = true;
#else 
 
    sensor_powerOn_flashParamCfg();
   // sensor_strct.cmd = dist_continue_ack_cmd;  
	//sensor_ouput_switch_high();
#endif	
  for(;;)
	{
	//	test_send_cmd();
     sensor_struct_loop(&sensor_strct); 
	 osDelay(500);
	}
}

/*����ⲿ������*/
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
	
	if(sensor_running_vaile.dist_base == 1)  //ǰ��׼
	{
		sensor_running_vaile.dist_sensor_lenth = SENSOR_LENGTH;
	}
	else
	{
		sensor_running_vaile.dist_sensor_lenth = 0;
	}
 sensor_running_vaile.dist_offset = lk_flash.QC[select_stande].qc_stand_dist;	
}
/*����flash �� �Զ����ò���*/
void sensor_powerOn_flashParamCfg(void)
{
	sensor_baudRate_typeEnum baud_selet = (sensor_baudRate_typeEnum )lk_flash.baud_rate;
  baudRateCfg_select(baud_selet);  //����������

  for(int i=0;i<3;i++)
	{
	   sensor_running_vaile.qc_offset[i]=lk_flash.QC[i].qc_stand_dist;//
		 sensor_running_vaile.qc_ifStand[i] = lk_flash.QC[i].ifHavedStand; 
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
	sensor_distOffset_calculate(lk03_first_gears);	//ƫ��ֵ����
	if(lk_flash.autoRunMode == 1)  //�Զ�����
	{
	   sensor_strct.cmd = dist_continue_ack_cmd;
	}
	else
	{
	   sensor_strct.cmd = sensor_idle;
	}
	
	

}

void start_singnal(void)
{
		start_txSignl_Tim();   //��ʼpwm���巢��
		trigOnce();			 
}

//ƽ��
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
				 lk_average(&disPlayDistBufer[0],AVERAGE_SIZE);					 
			 }else
			 {      /*����ʱ���ݲ��㹻ʱ*/
				 	for(counts_index=0;counts_index<queue_lenth;counts_index++)
				 {
						if(Queue_pop(&lk_distQueue,&disPlayDistBufer[counts_index]) ==Q_ERROR)
						{
							QE_FLASG=0;
						}
				 }
			   lk_average(&disPlayDistBufer[1],queue_lenth-1);  //�����л�ģʽʱ�������׸����ݲ��������
			 }			 
       QE_FLASG ++;
			 snesor_ouput_switch(average);  //�ⲿ���������
		} //end _TDC_GP21.ifComplete			
		switch(_TDC_GP21.system_statu.running_statu)   //��λ�л�״̬
		 {
			case IDLE:
			{
			  
			}break;				
			 case START:
			 {

        if((lk_flash.QC[lk03_first_gears].ifHavedStand)&(lk_flash.QC[lk03_second_gears].ifHavedStand)&(lk_flash.QC[lk03_third_gears].ifHavedStand))  //ȫ���궨��ŵ�λ�л�
				{
					 gear_select_switch(lk03_first_gears);
					_TDC_GP21.system_statu.running_statu = FIRST;
				}
			 }break;
			 case FIRST:
			 {
				   if(_TDC_GP21.pid_resualt >600 ) //��1���������600ʱ (_TDC_GP21.pid_resualt >620)&(
					 {
						   gear_select_switch(lk03_second_gears);
						 _TDC_GP21.system_statu.running_statu = SECOND;
					 }
				 
			 }break;
			 case SECOND:
			 {
				 	  if(_TDC_GP21.pid_resualt >600)   //��2���������600ʱ�л�������
					 {
						  gear_select_switch(lk03_third_gears);
						 _TDC_GP21.system_statu.running_statu = THIRD;
					 }
					 else if(_TDC_GP21.pid_resualt <500) //�л���һ��((_TDC_GP21.pid_resualt <280)&(_TDC_GP21.distance<3500))
					 {
						   gear_select_switch(lk03_first_gears);
						  _TDC_GP21.system_statu.running_statu = FIRST;
					 }
				
			 }break;		 
			 case THIRD:
			 {
						if(_TDC_GP21.pid_resualt <500)  //�л���2��
					 {
					  	 lk_gp21MessgeMode_switch(GP21_MESSGE1);
					     gear_select_switch(lk03_second_gears);  //��2��
						  _TDC_GP21.system_statu.running_statu = SECOND;
					 }
					 	if(_TDC_GP21.distance >30000)  //����300��
					 {
						 lk_gp21MessgeMode_switch(GP21_MESSGE2);
						  _TDC_GP21.system_statu.running_statu = LONG_DISTANCE_MODE;
					 }
//           if((_TDC_GP21.statu==trig_time_out)&(_TDC_GP21.siganl.vol>=_TDC_GP21.pid.setpoint))//�ڲ���ģʽ2��time_out ���źŴ����趨��ֵ
//					 {
//					   	 _TDC_GP21.messge_mode=GP21_MESSGE1;
//	             lk_gp21MessgeMode_switch(&_TDC_GP21);
//						    _TDC_GP21.running_statu = FIRST;
//					 }						 			    
			 }break;		
			 case LONG_DISTANCE_MODE:
			 {			 
				 		if(_TDC_GP21.distance <20000)  //С��200��
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


/*�����������*/
void lk_sensor_outData_Task(void *argument)
{

  vTaskSuspend(xHandleDataOutFeq);   //��������
	
	for(;;)
	{
		if((_TDC_GP21.ifDistanceNull==false)&(_TDC_GP21.ifMachineFine))
		 {
			 if(if_debug == true)
			 {
				zt_protecl_printf("distance: %d, sighal_value :%d  pid_result : %d\r\n",_TDC_GP21.distance,_TDC_GP21.siganl.vol,_TDC_GP21.pid_resualt);
			 }
			 else
			 {
				 // Send_Pose_Data(&_TDC_GP21.siganl.vol,&average,&_TDC_GP21.pid_resualt);
				 // Send_Pose_Data(&_TDC_GP21.siganl.vol,&_TDC_GP21.distance,&_TDC_GP21.pid_resualt);
                 sensor_distContinu_ack(_TDC_GP21.distance); 	
                // zt_protecl_printf("distance: %d, sighal_value :%d  pid_result : %d\r\n",_TDC_GP21.distance,_TDC_GP21.siganl.vol,_TDC_GP21.pid_resualt);				 
			 }
				//sensor_distContinu_ack(_TDC_GP21.distance);   
			// sensor_distContinu_ack(average);   
			
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
				  __HAL_TIM_SET_AUTORELOAD(singhlTim,100);  //�趨100us����
				  gear_select(&_TDC_GP21.system_statu.high_value_defconfg[lk03_first_gears]);  
	              lk_gp21MessgeMode_switch(GP21_MESSGE1); 
				  slect_mode=first_mes2;
				  gp21_write_reg(OPC_START_TOF);
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
  tdc_board_init();   /*��ʼ�������*/
//	High_Vol_Ctl_on();
	_TDC_GP21.pid.ifTrunOn = true;  //
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

//ģʽѡ��
void selected_mesg_mode(TypedSelextMsgMode mode)
{
   TypedSelextMsgMode slect_index=mode;
   switch(slect_index)
	 {
		 case msg_thirdStard:
		 {
			 __HAL_TIM_SET_AUTORELOAD(singhlTim,500);  //�趨500us����
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
			gear_select(&_TDC_GP21.system_statu.high_value_defconfg[lk03_first_gears]);  //			
			lk_gp21MessgeMode_switch(GP21_MESSGE1);
			_TDC_GP21.system_statu.running_statu=START;
			ifFirstStart = true;
			ifStartCplet = true; 
		 }break;			 
	 }


}



void trigOnce(void)
{
	gp21_write_reg(OPC_START_TOF);
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
    _TDC_GP21.buff[trigCount++] = gp21_read_diatance(0);//�ռ������������		
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
uint16_t vol_signal=0,statu_erro=0;
uint32_t GP21_REG;
uint32_t gp21_statu_INT;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
   int distance=0;
  if(GPIO_Pin ==GP21_INTN_Pin )
	{ 
			
		gp21_statu_INT = get_gp21_statu();	
    textCount++; 
    reg_index=gp21_statu_INT&0x03; //ȡ����Ĵ�����ַ
		GP21_REG = gp21_read_diatance(0);//�ռ������������  
		if(gp21_statu_INT &0x400)
		{
		   ms2_erro++;
		}
		if(gp21_statu_INT &0x200)
		{
		   ms1_err0++;
		}
		trigCount++;
  if(((gp21_statu_INT &0x400)==false)&&((gp21_statu_INT &0x200)==false)&&(gp21_statu_INT!=0)) //��Чֵ
			{		
				if(_TDC_GP21.ifMachineFine)  //�����������������
				{
				  _TDC_GP21.buff[trigCount-1] = GP21_REG;//�ռ������������
				}
				gp21_write_reg(OPC_INIT);			
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
					//��ʼ�ɼ���ѹ
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
									tdc_rx_voltge_relese();   /*��ѹ�źŲɼ��ͷ�*/	
									_TDC_GP21.pid_resualt= tdc_agc_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid���Ʒ�ֵ��ѹ
							}
							if(_TDC_GP21.ifMachineFine)  //�����������������
							{
								distance = gp21_distance_cal(_TDC_GP21.buff,DISTANCE_RCV_SIZE)-sensor_running_vaile.dist_offset-sensor_running_vaile.dist_sensor_lenth;
								if(distance>=0)
								{
									_TDC_GP21.ifDistanceNull=false;
									_TDC_GP21.distance=distance;
								}else { _TDC_GP21.ifDistanceNull=true;}  //����С��ƫ��ֵ������Ч
								if(_TDC_GP21.distance>500)
								{
								  _TDC_GP21.ifComplete = false;
								}
								if( Queue_push(&lk_distQueue,_TDC_GP21.distance) ==Q_ERROR)
								{
									 flag = 1; //������
								}
								_TDC_GP21.ifComplete = true;				//����											
							}
				}	//�������㹻����
			}
			else  //��Чֵ
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
							tdc_rx_voltge_relese();   /*��ѹ�źŲɼ��ͷ�*/	
							_TDC_GP21.pid_resualt= tdc_agc_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid���Ʒ�ֵ��ѹ
					}
					}	//endif _TDC_GP21.pid.ifTrunOn
	          gp21_write_reg(OPC_INIT);						
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

