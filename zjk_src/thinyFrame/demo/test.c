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
extern sensor_runnig_cmd_typEnum sensor_runnig_cmd;
bool ifDebug=false;  //是否调试标记
bool do_corrupt = false;
extern const char *romeo;
bool isFlashParam;   //是否保存数据
void z_ListenerInit(void);
extern TaskHandle_t xHandleGp21Trig;
typedef bool z_lkParmStatuType;
sensor_struct_ sensor_strct;    //传感器结构

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


/*激光器保存参数*/
parm_ lk_defaultParm ={
	.product = 0x02,     //产品号lk03
	.baud_rate = 5, //波特率0:9600;1:4400;2:19200;3:38400;4:57600;5:115200
	.front_limit_trigger = 100, //100米触发
  .back_limit_trigger = 0, //0米触发
  .front_or_base = 0,      //前基准：1 后基准：0
	.ifHasConfig = 0,      //第一次烧写flash后会变成0x01
  .autoRunMode =0,       //自动运行关闭
	.outFreq = 10,         //输出频率10hz 
  .displayMode = 1 ,         //默认字符输出显示
};



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
	.front_limit_trigger = 0, //100米触发
  .back_limit_trigger = 0, //0米触发
  .front_or_base = 0,      //前基准：1 后基准：0
	.ifHasConfig = 0,      //第一次烧写flash后会变成0x01
  .autoRunMode =0,       //自动运行关闭
	.outFreq = 0         //输出频率1000hz
};


/*通用控制*/
void dataGetCmdSlect(revFrame_distCtl_id_typEnum  dit_ctrl_cmd, TF_Msg *msg)
{

	switch(dit_ctrl_cmd)
	{
		case dist_once :    //单次测量命令
		{
				sensor_strct.cmd= dist_once_ack_cmd;		 
			  sensor_strct.msg = msg;
		}break;
	  case dist_continue:  //连续测量命令
		{
			 	sensor_strct.cmd= dist_continue_ack_cmd;		 
			  sensor_strct.msg = msg;
		}break;
		case dist_stop:   //停止测量
		{
			 	sensor_strct.cmd= dist_stop_ack_cmd;		 
			  sensor_strct.msg = msg;
		}break;
	}

}
/*参数获取*/
void paramGetCmdSlect(revFrame_paramCfg_getId_typEnum Param_GET, TF_Msg *msg)
{
  sensor_strct.msg = msg;
   switch(Param_GET)  //参数获取
	 {
		 case lk_all_get:  //所有参数
		 {
			 	sensor_strct.cmd= get_paramAll_base_cmd;		  
		 }break;
		 case baudRate_get:  //波特率获取
		 {
		   	sensor_strct.cmd= getParam_baudRate_ack_cmd;
		 }break;
		 case frontSwich_get: 
		 {
        sensor_strct.cmd= getParam_frontSwich_ack_cmd;
		 }break;
		 case backSwich_get:
		 {
		    sensor_strct.cmd= getParam_backSwich_ack_cmd;
		 }break;
		 case disBase_get:
		 {
        sensor_strct.cmd= getParam_disBase_ack_cmd;
		 }break;
		 case powerOn_mode_get:
		 {
        sensor_strct.cmd= getParam_powerOn_mode_ack_cmd;
		 }break;	
		 case outData_freq_get:
		 {
       	sensor_strct.cmd= getParam_outData_freq_ack_cmd;
		 }break;	
		 case cfgParam_strShow_get:
		 {
       	sensor_strct.cmd= getParam_stringShow_cmd;
		 }break;		 
		 
	 }
}


/*参数保存命令*/
void paramDataSaveCMD(revFrame_paramCfg_setId_typEnum PAMRM_SAVE, TF_Msg *msg)
{
  sensor_strct.msg = msg;
	switch(PAMRM_SAVE)
	{
		case lk_all_set:
		{
		    sensor_strct.cmd = cfgParam_all_cmd;

		}break;
		case baudRate_set:
		{
      sensor_strct.cmd = cfgParam_baudRate_ack_cmd;

		}break;
		
		case frontSwich_set:
		{
      sensor_strct.cmd = cfgParam_frontSwich_ack_cmd;

		}break;
		
		case backSwich_set:
		{
        sensor_strct.cmd = cfgParam_backSwich_ack_cmd;

		}break;
		case disBase_set:  
		{
          sensor_strct.cmd = cfgParam_distBase_ack_cmd;
		
		}break;		
		case powerOn_mode_set:  
		{
         sensor_strct.cmd = cfgParam_powerOn_mode_ack_cmd;
		
		}break;				
		case outData_freq_set:  
		{
          sensor_strct.cmd = cfgParam_outData_freq_ack_cmd;
		
		}break;
		case cfgParam_strShow_set:  
		{
         sensor_strct.cmd = cfgParam_stringShow_cmd;
		
		}break;		
	}
	
}


/*QC 检测命令*/
void programer_cmd(revFrame_programer_id_typEnum qc, TF_Msg *msg)
{
  revFrame_programer_id_typEnum cnd=qc;
	sensor_strct.msg = msg;
	switch(cnd)
	{

		case qc_standFirst_save:  //上位机第1档标定值
		{
			sensor_strct.cmd = qc_standFirst_save_cmd;

		}break;
		case qc_standSecond_save:  //上位机第2档标定值
		{
				sensor_strct.cmd = qc_standSecond_save_cmd;

		}break;	
		case qc_standthird_save:  //上位机第3档标定值
		{
				sensor_strct.cmd = qc_standthird_save_cmd;
	
		}break;	
		case qc_standFirst_reset:  //第1档从新校准
		{
				sensor_strct.cmd = qc_standFirst_reset_cmd;
		}break;
		case qc_standSecond_reset:  //第2档从新校准
		{
				sensor_strct.cmd = qc_standSecond_reset_cmd;
		}break;	
		case qc_standthird_reset:  //第3档从新校准
		{
				sensor_strct.cmd = qc_standthird_reset_cmd;
		}break;	
		case qc_standFirst_switch:  //档位1切换
		{
				sensor_strct.cmd = qc_standFirst_switch_cmd;
		}break;
		case qc_standSecond_switch:  //档位2切换
		{
				sensor_strct.cmd = qc_standSecond_switch_cmd;
		}break;	
		case qc_standthird_switch:  //档位3切换
		{
				sensor_strct.cmd = qc_standthird_switch_cmd;
		}break;			
    case qc_get_param :  //获取参数
		{
				sensor_strct.cmd = qc_get_param_cmd;
		}break;			
    case sensor_debugMode_switch :  //调试模式
		{
				sensor_strct.cmd = programer_debugMode_cmd;
		}break;	
    case sensor_standMode_switch :  //标定模式
		{
				sensor_strct.cmd = programer_qcStamdMode_cmd;
		}break;
    case sensor_normalMode_switch :  //正常模式
		{
				sensor_strct.cmd = programer_normalMode_cmd;
		}break;		
    case sensor_debug_stringShow :  //调试信息显示
		{
				sensor_strct.cmd = programer_debug_stringShow_cmd;
		}break;		
		
	}
 
}

void system_ctl_cmd(revFrame_system_ctl_id_typEnum id, TF_Msg *msg)
{
		sensor_strct.msg = msg;
    revFrame_system_ctl_id_typEnum cmd = id;
	  switch(cmd)
		{
			case system_param_reset:
			{
			  sensor_strct.cmd = system_boot_paramReset_ack_cmd;
			
			}break;
		
		}

}




//调试命令
void debug_cmd(FRAME_DEBUG_CMD cmd,TF_Msg *msg)
{
   FRAME_DEBUG_CMD cmd_id=cmd;
   switch(cmd_id) 
	 {
	   case debug_ID:
		 {
			  if(msg->data[0]==1)
				{
				  ifDebug = true;
				}
			  if(msg->data[0]==0)
				{
				  ifDebug = false;
				}		 	  
		 }break;
	 
	 }

}

//固件升级命令
void download_cmd(revFrame_firmware_ctl_id_typEnum cmd,TF_Msg *msg)
{
   revFrame_firmware_ctl_id_typEnum cmd_id=cmd;
   switch(cmd_id) 
	 {
	   case firmware_begin:
		 {
        
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

/*通用监听回调函数*/
TF_Msg cmdMsg ; uint8_t msg_data[50] = {0};

void clear_msgData(TF_Msg *msg)
{
   for(int i=0;i<msg->len;i++)
	{
	  msg_data[i] = 0;
	}
}

 TF_Result myGenericListener(TinyFrame *tf, TF_Msg *msg)
{
  TF_Msg *sensorMsg ;                   
  for(int i=0;i<msg->len;i++)
	{
	   msg_data[i] = msg->data[i];
	}
	cmdMsg.frame_id = msg->frame_id;
	cmdMsg.type = msg->type;
	cmdMsg.data = msg_data;
	sensorMsg = &cmdMsg;
	revFrame_Type_typEnum recve_type = (revFrame_Type_typEnum) (sensorMsg->type);
	 switch(recve_type)
	{
	  case user_dist_ctl:/*测量命令*/
		{
			revFrame_distCtl_id_typEnum dist_ctl_id =  (revFrame_distCtl_id_typEnum) (sensorMsg->frame_id);
		  dataGetCmdSlect(dist_ctl_id,sensorMsg);
		}break;
	  case usr_paramCfg_get: /*参数获取*/
		{
			revFrame_paramCfg_getId_typEnum get_param_id =  (revFrame_paramCfg_getId_typEnum) (sensorMsg->frame_id);
		  paramGetCmdSlect(get_param_id,sensorMsg);
		}break;		
	  case user_paramCfg_set: /*参数配置*/
		{
		  revFrame_paramCfg_setId_typEnum set_param_id = (revFrame_paramCfg_setId_typEnum) (sensorMsg->frame_id);
		  paramDataSaveCMD(set_param_id,sensorMsg);
		}	break;
	  case system_boot_firmware_ctl: /*系统固件控制*/
		{

		}break;		
	  case system_boot_firmware_pakage: /*系统固件包*/
		{

		}	break;	

	  case system_boot_param: /*系统参数*/
		{
      revFrame_system_ctl_id_typEnum system_id = (revFrame_system_ctl_id_typEnum) sensorMsg->frame_id;   
	    system_ctl_cmd(system_id,sensorMsg); 
		}	break;			
	  case programer_ctl: /*开发人员控制*/
		{ 
			revFrame_programer_id_typEnum cmd_id = (revFrame_programer_id_typEnum) sensorMsg->frame_id;
      programer_cmd(cmd_id,sensorMsg);
		}	break;		
				
	}
	
    return TF_STAY;
}

uint8_t testbuf[50] = {0};
void tinyRecFunc(uint8_t *buf)
{
		 uint16_t lens =0;
     
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
  	msg.type = usr_ack;
	  msg.frame_id = get_paramAll_base;
	  msg.data = arrayBuff.point;
    msg.len = arrayBuff.lens;
  	TF_Respond(demo_tf, &msg);	
}
//qc标定参数发送
void QCparmSend(uint8_t *data,uint8_t lens)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);	
  	msg.type = programer_ack;
	  msg.frame_id = qc_get_param_ack;
	  msg.data = data;
    msg.len = lens;
  	TF_Respond(demo_tf, &msg);	
	
}

//调试debug命令测试
void debugParmSend(uint8_t *data,uint8_t lens)
{
//    TF_Msg msg;
//    TF_ClearMsg(&msg);	
//  	msg.type = lk_debug;
//	  msg.frame_id = debug_ID;
//	  msg.data = data;
//    msg.len = lens;
//  	TF_Respond(demo_tf, &msg);	
}
TF_Msg msga;
void test_send_cmd(void)
{
    
    TF_ClearMsg(&msga);	
  	msga.type = 0xff;
	  msga.frame_id = lk_download_ack;
  	TF_Respond(demo_tf, &msga);

}

void z_tiny_test(void)
{

   z_ListenerInit();
	 test_send_cmd();
	 if(addUartDmaRevListen(tinyRecFunc)) 
	 {	 
		 
	 }
	 else
	 {
	    //add false
	 }
 
}
/***********************************************
   user_dist_ctl 应答 列表
***********************************************/
//单次测量应答数据
void zTF_sendOnceDistAck(uint8_t *data,uint8_t lens)
{
   lk_user_ack(dist_once_ack,data,lens);
}
//停止测量应答
void zTF_StopDistAck(void)
{
  lk_user_ack(dist_stop_ack,NULL,NULL);
}

//无效数据发生
void zTF_NullDistAck(void)
{
  lk_user_ack(dist_null_ack,NULL,NULL);
}
//连续测量应答数据
TF_Msg dist_continue_msg;
void zTF_sendContinueDistAck(uint8_t *data,uint8_t lens)
{
    dist_continue_msg.data = data;
    dist_continue_msg.len = lens;
  	TF_Respond(demo_tf, &dist_continue_msg);
}

/*========获取参数应答================*/
void zTF_paramCfg_getAll_Ack(uint8_t *data,uint8_t lens)
{
  lk_user_ack(get_paramAll_base,data,lens);
}

void zTF_paramCfg_getBaudRate_Ack(uint8_t *data,uint8_t lens)
{
  lk_user_ack(getParam_baudRate_ack,data,lens);
}

void zTF_paramCfg_getFrontSwich_Ack(uint8_t *data,uint8_t lens)
{
  lk_user_ack(getParam_frontSwich_ack,data,lens);
}

void zTF_paramCfg_getBackSwich_Ack(uint8_t *data,uint8_t lens)
{
  lk_user_ack(getParam_backSwich_ack,data,lens);
}

void zTF_paramCfg_getDisBase_Ack(uint8_t *data,uint8_t lens)
{
  lk_user_ack(getParam_disBase_ack,data,lens);
}

void zTF_paramCfg_getPowerOnMode_Ack(uint8_t *data,uint8_t lens)
{
  lk_user_ack(getParam_powerOn_mode_ack,data,lens);
}
void zTF_paramCfg_getOutDataFreq_Ack(uint8_t *data,uint8_t lens)
{
  lk_user_ack(getParam_outData_freq_ack,data,lens);
}

void zTF_paramCfg_get_dispalyMode_Ack(uint8_t *data,uint8_t lens)
{
  lk_user_ack(getParam_strShow_ack,data,lens);
}
/*========参数配置应答================*/
void zTF_paramCfg_setAll_Ack(void)
{
  lk_user_ack(cfgParam_all,NULL,NULL);
}

void zTF_paramCfg_setBaudRate_Ack(void)
{
  lk_user_ack(cfgParam_baudRate_ack,NULL,NULL);
}

void zTF_paramCfg_setFrontSwich_Ack(void)
{
  lk_user_ack(cfgParam_frontSwich_ack,NULL,NULL);
}

void zTF_paramCfg_setBackSwich_Ack(void)
{
  lk_user_ack(cfgParam_backSwich_ack,NULL,NULL);
}

void zTF_paramCfg_setDisBase_Ack(void)
{
  lk_user_ack(cfgParam_distBase_ack,NULL,NULL);
}

void zTF_paramCfg_setPowerOnMode_Ack(void)
{
  lk_user_ack(cfgParam_powerOn_mode_ack,NULL,NULL);
}

void zTF_paramCfg_setOutDataFreq_Ack(void)
{
  lk_user_ack(cfgParam_outData_freq_ack,NULL,NULL);
}

void zTF_paramCfg_set_displayMode_Ack(void)
{
  lk_user_ack(cfgParam_strShow_ack,NULL,NULL);
}

/*========system应答================*/
void zTF_system_boot_paramReset_Ack(void)
{
  lk_user_ack(system_boot_paramReset_ack,NULL,NULL);
}

void zTF_system_firmware_ctl_Ack(void)
{
  lk_user_ack(system_boot_firmware_ctl_ack,NULL,NULL);
}

void zTF_system_firmware_pakage_Ack(void)
{
  lk_user_ack(system_boot_firmware_pakage_ack,NULL,NULL);
}

/*========programer应答================*/

void zTF_programer_qc_getParam_Ack(uint8_t *data,uint8_t lens)
{
  lk_programer_ack(qc_get_param_ack,data,lens);
}
//切换
void zTF_programer_qc_standFirst_switch_ack(void)
{
  lk_programer_ack(qc_standFirst_switch_ack,NULL,NULL);
}

void zTF_programer_qc_standSecond_switch_ack(void)
{
  lk_programer_ack(qc_standSecond_switch_ack,NULL,NULL);
}

void zTF_programer_qc_standthird_switch_ack(void)
{
  lk_programer_ack(qc_standthird_switch_ack,NULL,NULL);
}

//复位
void zTF_programer_qc_standFirst_reset_ack(void)
{
  lk_programer_ack(qc_standFirst_reset_ack,NULL,NULL);
}


void zTF_programer_qc_standSecond_reset_ack(void)
{
  lk_programer_ack(qc_standSecond_reset_ack,NULL,NULL);
}

void zTF_programer_qc_standthird_reset_ack(void)
{
  lk_programer_ack(qc_standthird_reset_ack,NULL,NULL);
}

//存储
void zTF_programer_qc_standFirst_save_ack(void)
{
  lk_programer_ack(qc_standFirst_save_ack,NULL,NULL);
}


void zTF_programer_qc_standSecond_save_ack(void)
{
  lk_programer_ack(qc_standSecond_save_ack,NULL,NULL);
}


void zTF_programer_qc_standthird_save_ack(void)
{
  lk_programer_ack(qc_standthird_save_ack,NULL,NULL);
}


//调试应答
void zTF_programer_debug_sensorParam_ack(uint8_t *data,uint8_t lens)
{
  lk_programer_ack(sensor_debugMode_switch_ack,data,lens);
}

void zTF_programer_standMode_switch_ack(void)
{
  lk_programer_ack(sensor_dist_standMode_switch_ack,NULL,NULL);
}


/***********************************************
   用户通用 应答
***********************************************/
void lk_user_ack(sendframe_user_ackId_typEnum id,uint8_t *data,uint8_t lens)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);
    msg.type = usr_ack;
	  msg.frame_id = id;                          
	  msg.data = data;
    msg.len = lens;
  	TF_Respond(demo_tf, &msg);
}

/***********************************************
   开发人员应答
***********************************************/
void lk_programer_ack(sendframe_programer_ackId_typEnum id,uint8_t *data,uint8_t lens)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);
    msg.type = programer_ack;
	  msg.frame_id = id;                          
	  msg.data = data;
    msg.len = lens;
  	TF_Respond(demo_tf, &msg);
}



void dist_continu_msgInit(void)
{
	
    TF_ClearMsg(&dist_continue_msg);
    dist_continue_msg.type = usr_ack;
	  dist_continue_msg.frame_id = dist_continue_ack;

}
/*id listener add*/
void z_ListenerInit(void)
{
    demo_tf = &zjk_tf;
    TF_InitStatic(demo_tf, TF_MASTER);
    TF_AddGenericListener(demo_tf, myGenericListener); 
	  dist_continu_msgInit();
}
