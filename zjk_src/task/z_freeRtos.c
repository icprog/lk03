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
//
#include "z_include.h"
extern TIM_HandleTypeDef htim3;

TIM_HandleTypeDef *singhlTim=&htim3;
/*ȫ�ֱ�������*/ 
#define FIRST_OFFSET  4    //����ʵ��������10m�ڰ�궨�£��ٲ�һ����4cm
#define SECOND_OFFSET  0    
#define THIRD_OFFSET  0  
uint16_t ms2_erro=0,ms1_err0=0;   //��ʱ����
uint8_t flag=0,erro_count_test=0,reg_index;
static uint16_t offset_dist[3]={FIRST_OFFSET,SECOND_OFFSET,THIRD_OFFSET};
TypedSelextMode slect_mode;   //������ģʽѡ��
SqQueue lk_distQueue;  //ѭ������
uint16_t dist_offset=0;  //ƫ��ֵ
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
/*��������*/
void select_mode_ifStart(TypedSelextMode mode);
void lk_cmdAck(uint8_t type,uint8_t id,bool ack);
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
	   lk_defaultParm = lk_flash;  //��flash�ڲ�������Ĭ�ϵĲ�������  
	}
#if TEST_QC    //ģ��У׼�����Ѿ�ͨ��
	lk_flash.QC[SECOND_PARAM].ifHavedStand=true;
	lk_flash.QC[FIRST_PARAM].ifHavedStand=true;
	lk_flash.QC[THIRD_PARAM].ifHavedStand=true;		
	lk_flash.QC[SECOND_PARAM].qc_stand_dist=DIST_SECOND_OFFSET;
	lk_flash.QC[FIRST_PARAM].qc_stand_dist=DIST_FIRST_OFFSET;
	lk_flash.QC[THIRD_PARAM].qc_stand_dist=DIST_THIED_OFFSET;
//	lk_param_statu.ifContinuDist = true;
	#else 
	lk_param_statu.ifContinuDist = false;
		//lk_flash.QC[SECOND_PARAM].ifHavedStand=true;
#endif	

	
	
  for(;;)
	{
	  if(lk_param_statu.ifParamSave)
		{ 
		 flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
			/*����Ӧ����flash����ʧ�ܵĴ�������*/
		 lk_param_statu.ifParamSave = false;
		}
		if(lk_param_statu.ifParamGet)
		{
			 parmSend(&lk_flash);
		  lk_param_statu.ifParamGet = false;
		}
			for(int i=0;i<LK03_STAND_COUNTS;i++)  //�鿴��Ҫ�洢�궨����Ϣ
			{
			  if( lk_param_statu.ifQCStand[i]) //�궨
		    {
				  _LK03_STAND index =(_LK03_STAND) i;
					 dist_offset=lk_flash.QC[index].qc_stand_dist-offset_dist[index];
					 swStandSave(index);
					lk_cmdAck(QC,index+StandParamFirst,true);
				}
				if(lk_param_statu.ifQCgetParmReset[i])  //��λ�л�����У׼
				{
					HIGH_VOL_PARAM index = (HIGH_VOL_PARAM) i;
					lk_param_statu.ifQCgetParmReset[i]=false;
				   gear_select(&_TDC_GP21.vol_param[index]);   
					lk_cmdAck(QC,index+StandParamFirstReset,true);
				}		
			}
    if(lk_param_statu.ifQCgetParm)  //��ȡ�궨����
		{
		    qc_param_send();
			lk_param_statu.ifQCgetParm = false;
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

/*�궨ѡ��*/
void swStandSave(_LK03_STAND index)
{
	 _LK03_STAND _lk_standIndex = index;
		lk_flash.QC[_lk_standIndex].qc_stand_dist= lk_defaultParm.QC[_lk_standIndex].qc_stand_dist;  //
		lk_flash.QC[_lk_standIndex].qc_ad603Gain=lk_defaultParm.QC[_lk_standIndex].qc_ad603Gain;
		lk_flash.QC[_lk_standIndex].ifHavedStand=true;   //�궨�ɹ�
		/*�����ﱣ��flash*/
		flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
		/*����Ӧ����flash����ʧ�ܵĴ�������*/
		lk_param_statu.ifQCStand[_lk_standIndex]= false;
}


void start_singnal(void)
{
		start_txSignl_Tim();   //��ʼpwm���巢��
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);	
		trigOnce();			 
}
/*��У��ĵ����ݷ���*/
void noCheckSend(uint8_t *buff, uint8_t len)
{
		uint8_t buffToSend[len+2];
	  for(int i=1;i<len+1;i++)
	{
	  buffToSend[i]=buff[i];	
	}
  buffToSend[0]=0xAA;
	buffToSend[len+1]=0xBB;
	z_serial_write(buffToSend,len+2);   //������\r\n��β���ַ�			
}

//���;��룬�źŵ�ѹ������
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

	z_serial_write(sendbuf,8);   //������\r\n��β���ַ�
}
uint8_t buf[10]={0};
void buff_distTosend(uint16_t dist)
{
 	uint8_t index=0;
	buf[index++] = 0xFF;
	buf[index++] = dist_cmd;
  buf[index++] = _TDC_GP21.siganl.vol>>8;
	buf[index++] = _TDC_GP21.siganl.vol&0xff;
	buf[index++] = dist>>8;
	buf[index++] = dist&0xff;	
	buf[index++] = _TDC_GP21.pid_resualt>>8;
	buf[index++] = _TDC_GP21.pid_resualt&0xff;	
	buf[index++] = tx_chexkSum(buf,8);    //У��ͷ���
	z_serial_write(buf,index);

}


/*����Ӧ��
�ɹ�Ӧ��0xff+ ��Ӧ������+ id+ +true/false+У��� ;

*/
void lk_cmdAck(uint8_t type,uint8_t id,bool ack)
{
	uint8_t buf[10] ={0};
	uint8_t index=0;
	buf[index++] = 0xFF;
	buf[index++] = ack_cmd;	
	buf[index++] = type;
  buf[index++] = id;
	buf[index++] = ack;
	buf[index++] = 0;	
	buf[index++] = 0;
	buf[index++] = 0;		
	buf[index++] = tx_chexkSum(buf,8);    //У��ͷ���
	z_serial_write(buf,index);
}

//�궨������Ϣ
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

//ƽ��
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
			  else if(_TDC_GP21.running_statu==THIRD)
			 {
			     _TDC_GP21.siganl.vol=3000;
			 }
			if(_TDC_GP21.distance > 30000)
			 {
			     _TDC_GP21.siganl.vol=5000;
			 }
			 Send_Pose_Data(&_TDC_GP21.siganl.vol,&average,&_TDC_GP21.pid_resualt);
	     //  Send_Pose_Data(&_TDC_GP21.siganl.vol,&_TDC_GP21.distance,&_TDC_GP21.pid_resualt);
       #else
			   buff_distTosend(average);
			 #endif
			} 
		switch(_TDC_GP21.running_statu)
		 {
			 case START:
			 {

        if((lk_flash.QC[FIRST_PARAM].ifHavedStand)&(lk_flash.QC[SECOND_PARAM].ifHavedStand)&(lk_flash.QC[THIRD_PARAM].ifHavedStand))  //ȫ���궨��ŵ�λ�л�
				{
					 dist_offset=lk_flash.QC[FIRST_PARAM].qc_stand_dist-offset_dist[FIRST_PARAM];
					_TDC_GP21.running_statu = FIRST;
				}
        if(lk_flash.QC[THIRD_PARAM].ifHavedStand)
				{
				
				}
			 }break;
			 case FIRST:
			 {
				   if((_TDC_GP21.pid_resualt >600)&(_TDC_GP21.distance>2000) )  //��1���������600ʱ (_TDC_GP21.pid_resualt >620)&(
					 {
						  dist_offset=lk_flash.QC[SECOND_PARAM].qc_stand_dist-offset_dist[SECOND_PARAM];
						  _TDC_GP21.running_statu = SECOND;
						  gear_select(&_TDC_GP21.vol_param[SECOND_PARAM]); 
					 }
				 
			 }break;
			 case SECOND:
			 {
				 	  if((_TDC_GP21.pid_resualt >600)&(_TDC_GP21.distance>4500))   //��2���������600ʱ�л�������
					 {
						 gp21_messgeModeTwo();  //�л�Զ����ģʽ
						 __HAL_TIM_SET_AUTORELOAD(singhlTim,500);  //�趨500us����
						  gear_select(&_TDC_GP21.vol_param[THIRD_PARAM]);  //
						 dist_offset=lk_flash.QC[FIRST_PARAM].qc_stand_dist-offset_dist[THIRD_PARAM];
						 _TDC_GP21.running_statu = THIRD;
					 }
					 else if((_TDC_GP21.pid_resualt <280)&(_TDC_GP21.distance<4500)) //�л���һ��((_TDC_GP21.pid_resualt <280)&(_TDC_GP21.distance<3500))
					 {
					 
						  dist_offset=lk_flash.QC[FIRST_PARAM].qc_stand_dist-offset_dist[FIRST_PARAM];
					     gear_select(&_TDC_GP21.vol_param[FIRST_PARAM]);  //����Ĭ�ϵ�һ��λ
						  _TDC_GP21.running_statu = FIRST;
					 }
				
			 }break;		 
			 case THIRD:
			 {
						if((_TDC_GP21.distance <10000)&(_TDC_GP21.distance>4500))  //�л���2��
					 {
					     __HAL_TIM_SET_AUTORELOAD(singhlTim,100);  //�趨100us����
					     gear_select(&_TDC_GP21.vol_param[SECOND_PARAM]);  //����Ĭ�ϵ�2��λ
						 	 _TDC_GP21.messge_mode=GP21_MESSGE1;
	             lk_gp21MessgeMode_switch(&_TDC_GP21);
						  _TDC_GP21.running_statu = SECOND;
					 }
           if((_TDC_GP21.statu==trig_time_out)&(_TDC_GP21.siganl.vol>=_TDC_GP21.pid.setpoint))//�ڲ���ģʽ2��time_out ���źŴ����趨��ֵ
					 {
//					   	 _TDC_GP21.messge_mode=GP21_MESSGE1;
//	             lk_gp21MessgeMode_switch(&_TDC_GP21);
//						    _TDC_GP21.running_statu = FIRST;
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
uint8_t test_flag=0;

void select_mode_ifStart(TypedSelextMode mode)
{
  TypedSelextMode m=mode;
	switch(m)
	{
	  case first_mes1:
		{
 
			  if(mes2_have_sighal)
				{
				    test_flag =2;
					  _TDC_GP21.running_statu=THIRD;
					 
				}		
		    if(ms2_erro>100)
				{
					__HAL_TIM_SET_AUTORELOAD(singhlTim,100);  //�趨100us����
				 gear_select(&_TDC_GP21.vol_param[FIRST_PARAM]);  //����Ĭ�ϵ�2��λ
				  _TDC_GP21.messge_mode=GP21_MESSGE1;
	        lk_gp21MessgeMode_switch(&_TDC_GP21); 
					 slect_mode=first_mes2;
				  gp21_write(OPC_START_TOF);
				}
				
		}break;
	  case first_mes2:
		{
  	    
	     if(mes1_have_sighal)
				{
					   
						  _TDC_GP21.running_statu = START;
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
	High_Vol_Ctl_on();
//  gear_select(&_TDC_GP21.vol_param[FIRST_PARAM]);  //����Ĭ�ϵ�һ��λ SECOND_PARAM
//  _TDC_GP21.messge_mode=GP21_MESSGE1;
//	lk_gp21MessgeMode_switch(&_TDC_GP21);
	_TDC_GP21.pid.ifTrunOn = true;  //�ȹر�pid
	__HAL_TIM_SET_AUTORELOAD(singhlTim,500);  //�趨500us����
	 gear_select(&_TDC_GP21.vol_param[THIRD_PARAM]);  //			
	 _TDC_GP21.messge_mode=GP21_MESSGE2;
	lk_gp21MessgeMode_switch(&_TDC_GP21);
 _TDC_GP21.running_statu=STYLE;
  slect_mode = first_mes1;  
	ifFirstStart = true;
	lk_param_statu.ifContinuDist = true;
  /* Infinite loop */
  for(;;)
  { 		
    if(trighCount>10)
	 {
	   trighCount = 0;
		static uint32_t gp21_statu_INT;	
		//	 gp21_write(OPC_START_TOF);
		  text_dist= gp21_read_diatance(0);//�ռ������������
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
			tdc_rx_voltge_relese();   /*��ѹ�źŲɼ��ͷ�*/
	 	_TDC_GP21.pid_resualt= tdc_agc_Default_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid���Ʒ�ֵ��ѹ	 		
	 
	 }
   osDelay(1);
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
    if(((gp21_statu_INT &0x400)==false)&&((gp21_statu_INT &0x200)==false)) //��Чֵ
			{	
				_TDC_GP21.buff[trigCount++] = GP21_REG;//�ռ������������	
				gp21_write(OPC_INIT);			
				if(trigCount == DISTANCE_RCV_SIZE) 	
				{				
					trigCount = 0;
					textCount = 0;
					trighCount = 0;
          erro_count_test=0; 
					 _TDC_GP21.statu=trig_onece_complete;
					if(ifFirstStart)
					{
							if(_TDC_GP21.messge_mode==GP21_MESSGE1)
							{
								mes1_have_sighal=true;
							}
							if(_TDC_GP21.messge_mode==GP21_MESSGE2)
							{
								mes2_have_sighal=true;
							}							
					}
//						if(_TDC_GP21.messge_mode==GP21_MESSGE1)
//							{
//								ms1_err0 = 0;
//							}
//							if(_TDC_GP21.messge_mode==GP21_MESSGE2)
//							{
//								ms2_erro = 0;
//							}				
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
							_TDC_GP21.distance=  gp21_distance_cal(_TDC_GP21.buff,DISTANCE_RCV_SIZE)-dist_offset; //���ݴ���
							if( Queue_push(&lk_distQueue,_TDC_GP21.distance) ==Q_ERROR)
							{
								 flag = 1; //������
							}
							_TDC_GP21.ifComplete = true;				//����
				}	
			}
			else  //��Чֵ
	   	{
				erro_count_test ++;	 
				_TDC_GP21.statu=trig_time_out;			
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
				_TDC_GP21.pid_resualt= tdc_agc_control(_TDC_GP21.siganl.vol,_TDC_GP21.pid.setpoint); //pid���Ʒ�ֵ��ѹ
					}	//endif _TDC_GP21.pid.ifTrunOn
	   gp21_write(OPC_INIT);						
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