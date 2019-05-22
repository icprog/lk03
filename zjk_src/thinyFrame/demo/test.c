#include <string.h>
#include "TinyFrame.h"
#include "utils.h"
#include "cmsis_os.h"
//zjk include
#include "z_serial.h"
#include "z_param.h"

#include "z_include.h"
TinyFrame *demo_tf=NULL;
TinyFrame zjk_tf;	
bool do_corrupt = false;
extern const char *romeo;
bool isFlashParam;   //�Ƿ񱣴�����
void z_ListenerInit(void);
extern TaskHandle_t xHandleGp21Trig;
typedef bool z_lkParmStatuType;

lk_statu_ lk_param_statu={
  .ifParamSave =   false,
	.ifParamGet =    false,
	.ifGetOnceDist = false,
	.ifContinuDist = false,
	.ifStopContinu = false,
	.ifQCStand = false,
	.ifQCgetParm = false	
};


void parmSend(parm_ *parm);
/*�ֽ�����ת����Ӧ�Ľṹ��*/
parm_* byteToStruct(uint8_t *buff)
{
  parm_* parm = (parm_ *)buff;
	
  return parm;
}

/*�ṹ��ת����Ӧ�ֽ�*/
arrayByte_ structToBytes(parm_ *p)
{
	  arrayByte_ arraybuff;
    arraybuff.point =  (uint8_t*)(p);
	  arraybuff.lens = sizeof(* p);
	  return arraybuff;
} 

///*�������������*/
parm_ lk_defaultParm ={
	.product = 0x02,     //��Ʒ��lk03
	.baud_rate = 115200, //������
	.limit_trigger = 100, //100�״���
	.red_laser_light = 0x01, //��:0x01 �رգ�0
  .front_or_base = 0,      //ǰ��׼��1 ���׼��0
	.ifHasConfig = 0,      //��һ����дflash�����0x01
	.outFreq = 100
};



/*�������������*/
//parm_ lk_parm ={
//	.product = 0,     //��Ʒ��lk03
//	.baud_rate = 0, //������
//	.limit_trigger = 0, //100�״���
//	.red_laser_light = 0, //��:0x01 �رգ�0
//  .front_or_base = 0,      //ǰ��׼��1 ���׼��0
//};
parm_ lk_flash=
 { 
		.product = 0,     //��Ʒ��lk03
		.baud_rate = 0, //������
		.limit_trigger = 0, //100�״���
		.red_laser_light = 0, //��:0x01 �رգ�0
		.front_or_base = 0,      //ǰ��׼��1 ���׼��0
	 .ifHasConfig = 0,      //��һ����дflash�����0x01
};

typedef enum  { DataDistSend = 1, ParmsConfig = 2, ParmaSend=3, QC=6,ErroSend }FRAME_TYPE_CMD ;
typedef enum  { ParamAll=1}FRAME_GetParam_CMD;
typedef enum  { DistOnce = 1, DistContinue,DistStop}FRAME_GetDataID_CMD;
typedef enum  { BarudRate = 1, RedLight, FrontOrBase,AutoMel}FRAME_ParmSaveID_CMD;
typedef enum  { standStart = 1,StandParamFirst,StandParamSecond,StandParamThird,StandParamFirstReset, StandParamSecondReset, StandParamThirdReset,GetParam}FRAME_ParmQC_CMD;
/*���ݻ�ȡ����*/
void dataGetCmdSlect(FRAME_GetDataID_CMD  DATA_GET, TF_Msg *msg)
{

	switch(DATA_GET)
	{
		case DistOnce :    //���β�������
		{
			 lk_param_statu.ifGetOnceDist = true;
		}break;
	  case DistContinue:  //������������
		{
			  lk_param_statu.ifContinuDist = true;
		}break;
		case DistStop:   //ֹͣ����
		{
		    stop_txSignl_Tim();    //ֹͣ�����ź�
			  lk_param_statu.ifGetOnceDist = false;		
        lk_param_statu.ifContinuDist = false;
				lk_param_statu.ifStopContinu = false;		
		}break;
	}
	 if((lk_param_statu.ifGetOnceDist) || (lk_param_statu.ifContinuDist))
	 {
	    // vTaskResume(xHandleGp21Trig);   //�ָ�����״̬
	 }

}
/*������ȡ*/
void paramGetCmdSlect(FRAME_GetParam_CMD Param_GET, TF_Msg *msg)
{
  
   switch(Param_GET)  //������ȡ
	 {
		 case ParamAll:
		 {
		   lk_param_statu.ifParamGet= true; 
		 }break;
	 
	 }
}


/*������������*/
void paramDataSaveCMD(FRAME_ParmSaveID_CMD PAMRM_SAVE, TF_Msg *msg)
{

	switch(PAMRM_SAVE)
	{
		case BarudRate:
		{
        lk_defaultParm.baud_rate = *(int*)(msg->data);
		}break;
		
		case RedLight:
		{
          lk_defaultParm.red_laser_light = *(uint8_t *)(msg->data);
		}break;
		
		case FrontOrBase:
		{
         lk_defaultParm.front_or_base =  *(uint8_t *)(msg->data);
		}break;
		case AutoMel:  //�Զ�����
		{
         
		}break;		
	}
  lk_param_statu.ifParamSave =true;
}


/*QC �������*/
void QC_CMD(FRAME_ParmQC_CMD qc, TF_Msg *msg)
{
   FRAME_ParmQC_CMD cnd=qc;
	switch(cnd)
	{
		case standStart:
		{
       
		}break;
		case StandParamFirst:  //��λ����1���궨ֵ
		{
        lk_defaultParm.QC[LK03_FIRST_STAND].qc_stand_dist= msg->data[0]<<8|msg->data[1];
			  lk_defaultParm.QC[LK03_FIRST_STAND].qc_ad603Gain= msg->data[2]<<8|msg->data[3];
			  lk_param_statu.ifQCStand[LK03_FIRST_STAND] =true;
		}break;
		case StandParamSecond:  //��λ����2���궨ֵ
		{
        lk_defaultParm.QC[LK03_SECOND_STAND].qc_stand_dist= msg->data[0]<<8|msg->data[1];
			  lk_defaultParm.QC[LK03_SECOND_STAND].qc_ad603Gain= msg->data[2]<<8|msg->data[3];
			  lk_param_statu.ifQCStand[LK03_SECOND_STAND] =true;
		}break;	
		case StandParamThird:  //��λ����3���궨ֵ
		{
        lk_defaultParm.QC[LK03_THIRD_STAND].qc_stand_dist= msg->data[0]<<8|msg->data[1];
			  lk_defaultParm.QC[LK03_THIRD_STAND].qc_ad603Gain= msg->data[2]<<8|msg->data[3];
			  lk_param_statu.ifQCStand[LK03_THIRD_STAND] =true;
		}break;	
		case StandParamSecondReset:  //��1������У׼
		{
			lk_param_statu.ifQCgetParmReset[LK03_FIRST_STAND] = true;
		}break;
		case StandParamThirdReset:  //��2������У׼
		{
		 	lk_param_statu.ifQCgetParmReset[LK03_SECOND_STAND] = true;
		}break;	
		case StandParamFirstReset:  //��3������У׼
		{
			lk_param_statu.ifQCgetParmReset[LK03_THIRD_STAND] = true;
		}break;	
    case GetParam :  //��ȡ����
		{
			  lk_param_statu.ifQCgetParm = true;
		
		}break;			
	}
 
}
/**
 * This function should be defined in the application code.
 * It implements the lowest layer - sending bytes to UART (or other)
 */
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{		
		//zjk
		z_serial_write((uint8_t*)buff,len);
}

/*ͨ�ü����ص�����*/
TF_Msg *cmdMsg =NULL;
 TF_Result myGenericListener(TinyFrame *tf, TF_Msg *msg)
{
 
	cmdMsg = msg;
	FRAME_TYPE_CMD typeCMD = (FRAME_TYPE_CMD) (cmdMsg->type);
	 switch(typeCMD)
	{
	  case DataDistSend:/*��������*/
		{
			FRAME_GetDataID_CMD getDataCmd =  (FRAME_GetDataID_CMD) (cmdMsg->frame_id);
		  dataGetCmdSlect(getDataCmd,msg);
		}break;
	  case ParmaSend: /*������ȡ*/
		{
			FRAME_GetParam_CMD getParamCmd =  (FRAME_GetParam_CMD) (cmdMsg->frame_id);
		  paramGetCmdSlect(getParamCmd,msg);
		}break;		
	  case ParmsConfig: /*��������*/
		{
		  FRAME_ParmSaveID_CMD parmaSaveCmd = (FRAME_ParmSaveID_CMD) (cmdMsg->frame_id);
		  paramDataSaveCMD(parmaSaveCmd,msg);
		}	break;
		case QC:   /*�궨����*/
		{
		  FRAME_ParmQC_CMD QCCmd = (FRAME_ParmQC_CMD) (cmdMsg->frame_id);
		  QC_CMD(QCCmd,msg);
		}break;		
		case ErroSend:
		{
		  
		}break;
		
	}
	
    return TF_STAY;
}


void tinyRecFunc(uint8_t *buf)
{
		 uint16_t lens =0;
     uint8_t testbuf[50] = {0};
		 get_revLens(&lens);
		 for(int i=0;i<lens;i++)
		{
		  testbuf[i] = buf[i];
		
		}
	   TF_Accept(demo_tf, buf, lens);
	
	}


void parmSend(parm_ *parm)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);	
 	  arrayByte_ arrayBuff; 
	  arrayBuff = structToBytes(parm); 
  	msg.type = ParmaSend;
	  msg.frame_id = ParamAll;
	  msg.data = arrayBuff.point;
    msg.len = arrayBuff.lens;
  	TF_Respond(demo_tf, &msg);	
	
}
//qc�궨��������
void QCparmSend(uint8_t *data,uint8_t lens)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);	
  	msg.type = QC;
	  msg.frame_id = GetParam;
	  msg.data = data;
    msg.len = lens;
  	TF_Respond(demo_tf, &msg);	
	
}

void z_tiny_test(void)
{
   z_ListenerInit();
	 parmSend(&lk_defaultParm);
	 if(addUartDmaRevListen(tinyRecFunc)) 
	 {	 
		 
	 }
	 else
	 {
	    //add false
	 }
 
}

void zTF_sendOnceDist(uint8_t *data,uint8_t lens)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);
    msg.type = DataDistSend;
	  msg.frame_id = DistOnce;
    msg.data = data;
    msg.len = lens;
  //  TF_Send(demo_tf, &msg);
  	TF_Respond(demo_tf, &msg);
}

/*id listener add*/
void z_ListenerInit(void)
{
    demo_tf = &zjk_tf;
    TF_InitStatic(demo_tf, TF_MASTER);
    TF_AddGenericListener(demo_tf, myGenericListener); 
}