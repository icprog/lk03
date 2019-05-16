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
bool isFlashParam;   //是否保存数据
void z_ListenerInit(void);
extern TaskHandle_t xHandleGp21Trig;
typedef bool z_lkParmStatuType;

lk_statu_ lk_param_statu={
  .ifParamSave =   false,
	.ifParamGet =    false,
	.ifGetOnceDist = false,
	.ifContinuDist = false,
	.ifStopContinu = false,
	.ifQCStand = false
};


void parmSend(parm_ *parm);
/*字节数组转换对应的结构体*/
parm_* byteToStruct(uint8_t *buff)
{
  parm_* parm = (parm_ *)buff;
	
  return parm;
}

/*结构体转换对应字节*/
arrayByte_ structToBytes(parm_ *p)
{
	  arrayByte_ arraybuff;
    arraybuff.point =  (uint8_t*)(p);
	  arraybuff.lens = sizeof(* p);
	  return arraybuff;
} 

///*激光器保存参数*/
parm_ lk_parm ={
	.product = 0x02,     //产品号lk03
	.baud_rate = 115200, //波特率
	.limit_trigger = 100, //100米触发
	.red_laser_light = 0x01, //打开:0x01 关闭：0
  .front_or_base = 0,      //前基准：1 后基准：0
	.ifHasConfig = 0,      //第一次烧写flash后会变成0x01
	.outFreq = 100
};

/*QC 参数*/

typedef struct{
	uint8_t qc_stand_dist ;   //QC 标定距离 

}QC_TYP;

QC_TYP qc_param;
/*激光器保存参数*/
//parm_ lk_parm ={
//	.product = 0,     //产品号lk03
//	.baud_rate = 0, //波特率
//	.limit_trigger = 0, //100米触发
//	.red_laser_light = 0, //打开:0x01 关闭：0
//  .front_or_base = 0,      //前基准：1 后基准：0
//};
parm_ lk_flash=
 { 
		.product = 0,     //产品号lk03
		.baud_rate = 0, //波特率
		.limit_trigger = 0, //100米触发
		.red_laser_light = 0, //打开:0x01 关闭：0
		.front_or_base = 0,      //前基准：1 后基准：0
	 .ifHasConfig = 0,      //第一次烧写flash后会变成0x01
};

typedef enum  { DataDistSend = 1, ParmsConfig = 2, ParmaSend=3, QC=6,ErroSend }FRAME_TYPE_CMD ;
typedef enum  { ParamAll=1}FRAME_GetParam_CMD;
typedef enum  { DistOnce = 1, DistContinue,DistStop}FRAME_GetDataID_CMD;
typedef enum  { BarudRate = 1, RedLight, FrontOrBase }FRAME_ParmSaveID_CMD;
typedef enum  { stand = 1 }FRAME_ParmQC_CMD;
/*数据获取命令*/
void dataGetCmdSlect(FRAME_GetDataID_CMD  DATA_GET, TF_Msg *msg)
{

	switch(DATA_GET)
	{
		case DistOnce :    //单次测量命令
		{
			 lk_param_statu.ifGetOnceDist = true;
		}break;
	  case DistContinue:  //连续测量命令
		{
			  lk_param_statu.ifContinuDist = true;
		}break;
		case DistStop:   //停止测量
		{
		    stop_txSignl_Tim();    //停止发射信号
			  lk_param_statu.ifGetOnceDist = false;		
        lk_param_statu.ifContinuDist = false;
				lk_param_statu.ifStopContinu = false;		
		}break;
	}
	 if((lk_param_statu.ifGetOnceDist) || (lk_param_statu.ifContinuDist))
	 {
	    // vTaskResume(xHandleGp21Trig);   //恢复任务状态
	 }

}
/*参数获取*/
void paramGetCmdSlect(FRAME_GetParam_CMD Param_GET, TF_Msg *msg)
{
  
   switch(Param_GET)  //参数获取
	 {
		 case ParamAll:
		 {
		   lk_param_statu.ifParamGet= true; 
		 }break;
	 
	 }
}


/*参数保存命令*/
void paramDataSaveCMD(FRAME_ParmSaveID_CMD PAMRM_SAVE, TF_Msg *msg)
{

	switch(PAMRM_SAVE)
	{
		case BarudRate:
		{
        lk_parm.baud_rate = *(int*)(msg->data);
		}break;
		
		case RedLight:
		{
          lk_parm.red_laser_light = *(uint8_t *)(msg->data);
		}break;
		
		case FrontOrBase:
		{
         lk_parm.front_or_base =  *(uint8_t *)(msg->data);
		}break;
	}
  lk_param_statu.ifParamSave =true;
}


/*QC 检测命令*/
void QC_CMD(FRAME_ParmQC_CMD qc, TF_Msg *msg)
{
   FRAME_ParmQC_CMD cnd=qc;
	switch(cnd)
	{
		case stand:
		{
        qc_param.qc_stand_dist= *(uint8_t*)(msg->data);
		}break;
	}
  lk_param_statu.ifQCStand =true;
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

/*通用监听回调函数*/
TF_Msg *cmdMsg =NULL;
 TF_Result myGenericListener(TinyFrame *tf, TF_Msg *msg)
{
 
	cmdMsg = msg;
	FRAME_TYPE_CMD typeCMD = (FRAME_TYPE_CMD) (cmdMsg->type);
	 switch(typeCMD)
	{
	  case DataDistSend:/*测量命令*/
		{
			FRAME_GetDataID_CMD getDataCmd =  (FRAME_GetDataID_CMD) (cmdMsg->frame_id);
		  dataGetCmdSlect(getDataCmd,msg);
		}break;
	  case ParmaSend: /*参数获取*/
		{
			FRAME_GetParam_CMD getParamCmd =  (FRAME_GetParam_CMD) (cmdMsg->frame_id);
		  paramGetCmdSlect(getParamCmd,msg);
		}break;		
	  case ParmsConfig: /*参数配置*/
		{
		  FRAME_ParmSaveID_CMD parmaSaveCmd = (FRAME_ParmSaveID_CMD) (cmdMsg->frame_id);
		  paramDataSaveCMD(parmaSaveCmd,msg);
		}	break;
		case QC:   /*标定命令*/
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

void z_tiny_test(void)
{
   z_ListenerInit();
	 parmSend(&lk_parm);
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
