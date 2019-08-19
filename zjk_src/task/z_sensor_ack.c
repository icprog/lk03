#include "z_sensor_ack.h"

extern arrayByte_ flashParam;
extern TaskHandle_t xHandleDataOutFeq;
extern void sensor_distOffset_calculate(_sensor_gesr_enum index);
extern sensor_struct_typ sensor_running_vaile;
extern void system_runStatu_select(_sensor_cureent_typeMode rmod);
extern bool  ifLkProtecl_show;
extern bool  ifDebug_strShow;
extern bool  ifUser_strShow;
char *runTypeModeString = NULL;
 char *typeModeString[]={
   "dbg",
   "qc",
   "normal", 	 
 };

void sensor_distContinu_ack(uint16_t dist)
{
  uint8_t dist_buf[2] = {0};
	dist_buf[0] = dist>>8;
	dist_buf[1] = dist&0xff;
	zTF_sendContinueDistAck(dist_buf,2);
}
void sensor_distOnce_ack(uint16_t dist)
{
  uint8_t dist_buf[2] = {0};
	dist_buf[0] = dist>>8;
	dist_buf[1] = dist&0xff;
	zTF_sendOnceDistAck(dist_buf,2);
}
void sensor_distStop_ack(void)
{
	  stop_txSignl_Tim();    //停止发射信号 
    zTF_StopDistAck();
}

void sensor_getAllParam_ack(void)
{
	    uint8_t sensor_data[10] ={0};
			sensor_data[0] = lk_flash.baud_rate;
			sensor_data[1] = lk_flash.front_limit_trigger >>8;
			sensor_data[2] = lk_flash.front_limit_trigger &0xff;
			sensor_data[3] = lk_flash.back_limit_trigger >>8;
			sensor_data[4] = lk_flash.back_limit_trigger &0xff;		
			sensor_data[5] = lk_flash.front_or_base;
			sensor_data[6] = lk_flash.autoRunMode;		
			sensor_data[7] = lk_flash.outFreq >>8;
			sensor_data[8] = lk_flash.outFreq &0xff;
			sensor_data[9] = lk_flash.displayMode;
      zTF_paramCfg_getAll_Ack(sensor_data,10);
}

void sensor_getBaudRate_ack(void)
{
      zTF_paramCfg_getBaudRate_Ack(&lk_flash.baud_rate,1);
}

void sensor_getFrontSwich_ack(void)
{
     uint8_t sensor_data[2] ={0};		 
		sensor_data[0] = lk_flash.front_limit_trigger >>8;
		sensor_data[1] = lk_flash.front_limit_trigger &0xff;	
	 zTF_paramCfg_getFrontSwich_Ack(sensor_data,2);
}

void sensor_getDisBase_ack(void)
{
	 zTF_paramCfg_getDisBase_Ack(&lk_flash.front_or_base,1);
}

void sensor_getBackSwich_ack(void)
{
     uint8_t sensor_data[2] ={0};		 
		sensor_data[0] = lk_flash.front_limit_trigger >>8;
		sensor_data[1] = lk_flash.front_limit_trigger &0xff;  
	 zTF_paramCfg_getBackSwich_Ack(sensor_data,2);	      
}

void sensor_getPowerOnMode_ack(void)
{
      zTF_paramCfg_getPowerOnMode_Ack(&lk_flash.autoRunMode,1);
}

void sensor_getOutDataFreq_ack(void)
{
   uint8_t sensor_data[2] ={0};		 
	 sensor_data[0] = lk_flash.front_limit_trigger >>8;
	 sensor_data[1] = lk_flash.front_limit_trigger &0xff;  
	 zTF_paramCfg_getOutDataFreq_Ack(sensor_data,2);	   
}
void sensor_setAllParam_ack(TF_Msg *msg)
{
	    
      zTF_paramCfg_setAll_Ack();
	     clear_msgData(msg);
}


void sensor_setBaudRate_ack(TF_Msg *msg)
{
	
	 lk_flash.baud_rate = *(uint8_t *)msg->data;
	  clear_msgData(msg);
	 sensor_baudRate_typeEnum baud_selet = (sensor_baudRate_typeEnum )lk_flash.baud_rate;
	 flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
   zTF_paramCfg_setBaudRate_Ack();
	 baudRateCfg_select(baud_selet);
}
void sensor_setFrontSwich_ack(TF_Msg *msg)
{
	 lk_flash.front_limit_trigger =BigtoLittle16(msg->data);
	 sensor_running_vaile.front_switch = lk_flash.front_limit_trigger;
	  clear_msgData(msg);
	 flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
   zTF_paramCfg_setFrontSwich_Ack();
}
void sensor_setBackSwich_ack(TF_Msg *msg)
{
	  lk_flash.back_limit_trigger = BigtoLittle16(msg->data);
	  sensor_running_vaile.back_switch = lk_flash.back_limit_trigger;
	  clear_msgData(msg);
	  flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
      zTF_paramCfg_setBackSwich_Ack();
	  
}

void sensor_setDisBase_ack(TF_Msg *msg)
{
	   lk_flash.front_or_base = *(uint8_t *)msg->data;
	   clear_msgData(msg);
	   flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
	   if(lk_flash.front_or_base==1)
		 {
			 sensor_running_vaile.dist_sensor_lenth =SENSOR_LENGTH;
		 }
		 else {sensor_running_vaile.dist_sensor_lenth = 0;}

      zTF_paramCfg_setDisBase_Ack();
}

void sensor_setPowerOnMode_ack(TF_Msg *msg)
{
	   lk_flash.autoRunMode = *(uint8_t *)msg->data;
	   clear_msgData(msg);
	   flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
      zTF_paramCfg_setPowerOnMode_Ack();
}

void sensor_setOutDataFreq_ack(TF_Msg *msg)
{
	   lk_flash.outFreq = BigtoLittle16(msg->data);
	   if(lk_flash.outFreq>1000)
		 {
		   lk_flash.outFreq = 1000;
		 }
		 else if(lk_flash.outFreq ==0)
		 {
		   lk_flash.outFreq = 1;
		 }
		 sensor_running_vaile.output_freq = (1000/lk_flash.outFreq);
	   flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
      zTF_paramCfg_setOutDataFreq_Ack();
}
void sensor_displaySet_ack(TF_Msg *msg)
{
	uint8_t set =*(uint8_t *) msg->data;
  if(set == 1)//字符显示
	{
		lk_flash.displayMode = 1;
		ifLkProtecl_show = false;
		ifUser_strShow = true;
	}
	else if(set == 2)//协议显示
	{
		lk_flash.displayMode = 2;
		ifUser_strShow = false;
		ifLkProtecl_show = true;
	}
	flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
	zTF_paramCfg_set_displayMode_Ack();
}

void sensor_displayGet_ack(TF_Msg *msg)
{


}
//参数配置结束


extern void sensor_powerOn_flashParamCfg(void);
void sensor_system_boot_paramReset_ack(void)
{
	   lk_flash.baud_rate = lk_defaultParm.baud_rate;
	   lk_flash.autoRunMode = lk_defaultParm.autoRunMode;
	   lk_flash.front_limit_trigger = lk_defaultParm.front_limit_trigger;
	   lk_flash.back_limit_trigger = lk_defaultParm.back_limit_trigger;
	   lk_flash.outFreq = lk_defaultParm.outFreq;
	   lk_flash.front_or_base = lk_defaultParm.front_or_base;
	   flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
	
	   sensor_powerOn_flashParamCfg();
     zTF_system_boot_paramReset_Ack();
}

void sensor_system_firmware_ctl_ack(void)
{
      zTF_system_firmware_ctl_Ack();
}
void sensor_system_firmware_pakage_ack(void)
{
      zTF_system_firmware_pakage_Ack();
}

typedef enum{mode_start=0, stand_switch_type=0x01,pid_ctrl_type  }_debug_type_enum;

typedef union 
{
	uint8_t data[2];
  _debug_type_enum type;
	uint8_t id;
}dbug_modeType_union_;

void praogramer_debug_mode(TF_Msg *msg)
{
    dbug_modeType_union_  *dbug_typeId= (dbug_modeType_union_*)msg->data;
    switch(dbug_typeId->type)
		{
			case mode_start:   //调试模式认证。。。
			{
			      if(dbug_typeId->id == 78)  //验证成功
						{
						   
						}
						else
						{
						   
						}
			
			}break;
			case stand_switch_type:
			{
					if(dbug_typeId->id ==0)  //自动切换
					{
						_TDC_GP21.system_statu.running_statu = FIRST;  //不需要标定会自动切换
					}
					else if(dbug_typeId->id ==1)
					{
						_TDC_GP21.system_statu.running_statu = IDLE;   //不切换
						lk_gear_switch(lk03_first_gears);
						sensor_distOffset_calculate(lk03_first_gears);
					}
					else if(dbug_typeId->id ==2)
					{
						_TDC_GP21.system_statu.running_statu = IDLE;   //不切换
					 lk_gear_switch(lk03_second_gears);
					 sensor_distOffset_calculate(lk03_second_gears);
					}
					else if(dbug_typeId->id ==3)
					{
						_TDC_GP21.system_statu.running_statu = IDLE;   //不切换
					 lk_gear_switch(lk03_third_gears);
					 sensor_distOffset_calculate(lk03_third_gears);
					}	
			}break;
			case pid_ctrl_type:
			{
			
			}break;			
		}
}


//////////////
/*标定选项*/
void sensor_qc_getParam(void)
{
   uint8_t sensor_data[3*5] ={0};
	 for(int i=0;i<3;i++)
	 {
	   sensor_data[i*5+0] = lk_flash.QC[i].qc_stand_dist>>8;
	   sensor_data[i*5+1] = lk_flash.QC[i].qc_stand_dist&0xff;
	   sensor_data[i*5+2] = lk_flash.QC[i].qc_ad603Gain>>8;
	   sensor_data[i*5+3] = lk_flash.QC[i].qc_ad603Gain&0xff;		 
	   sensor_data[i*5+4] = lk_flash.QC[i].ifHavedStand;		 
	 }
  zTF_programer_qc_getParam_Ack(sensor_data,15);
}

void sensor_qc_struct_Save(_sensor_gesr_enum index,TF_Msg *msg)
{
	 _sensor_gesr_enum _lk_standIndex = index;
		lk_flash.QC[_lk_standIndex].qc_stand_dist=BigtoLittle16(msg->data);  //
		lk_flash.QC[_lk_standIndex].qc_ad603Gain=BigtoLittle16(&msg->data[2]);
		lk_flash.QC[_lk_standIndex].ifHavedStand=true;   //标定成功
	   clear_msgData(msg);
		/*在这里保存flash*/
		flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
	  sensor_distOffset_calculate(_lk_standIndex);
}

//标定信息复位
void flasStandReset(_sensor_gesr_enum index)
{
	 _sensor_gesr_enum _lk_standIndex = index;
		lk_flash.QC[_lk_standIndex].qc_stand_dist= 0;  //
		lk_flash.QC[_lk_standIndex].qc_ad603Gain=0;
		lk_flash.QC[_lk_standIndex].ifHavedStand=false;   //未标定
		/*在这里保存flash*/
    sensor_distOffset_calculate(_lk_standIndex);
		flash_writeMoreData( (uint16_t *)(flashParam.point),flashParam.lens/2+1);
		/*这里应该有flash保存失败的处理！！！*/
}

void sensor_qc_get_param_ack(void)
{
  
}
void sensor_qc_standFirst_switch_ack(void)
{
		lk_gear_switch(lk03_first_gears);
	  sensor_distOffset_calculate(lk03_first_gears);
	  zTF_programer_qc_standFirst_switch_ack();
}
void sensor_qc_standSecond_switch_ack(void)
{
		lk_gear_switch(lk03_second_gears);
	 sensor_distOffset_calculate(lk03_second_gears);
	  zTF_programer_qc_standSecond_switch_ack();
}
void sensor_qc_standthird_switch_ack(void)
{
		lk_gear_switch(lk03_third_gears);
	  sensor_distOffset_calculate(lk03_third_gears);
	  zTF_programer_qc_standthird_switch_ack();
}
void sensor_qc_standFirst_reset_ack(void)
{	
		flasStandReset(lk03_first_gears);    
	  zTF_programer_qc_standFirst_reset_ack();
}
void sensor_qc_standSecond_reset_ack(void)
{
    flasStandReset(lk03_second_gears);
	  zTF_programer_qc_standSecond_reset_ack();
}
void sensor_qc_standthird_reset_ack(void)
{
    flasStandReset(lk03_third_gears);
	  zTF_programer_qc_standthird_reset_ack();
}


void sensor_qc_standFirst_save_ack(TF_Msg *msg)
{
     sensor_qc_struct_Save(lk03_first_gears,msg);   //存储标定信息
  	zTF_programer_qc_standFirst_save_ack();    
}
void sensor_qc_standSecond_save_ack(TF_Msg *msg)
{
     sensor_qc_struct_Save(lk03_second_gears,msg);
    zTF_programer_qc_standSecond_save_ack();
}
void sensor_qc_standthird_save_ack(TF_Msg *msg)
{
   sensor_qc_struct_Save(lk03_third_gears,msg);
	zTF_programer_qc_standthird_save_ack();
}

void sensor_debuMde_standSwich_ack(TF_Msg *msg)
{
   uint8_t ctl_msg= (uint8_t) msg->data;
	if(ctl_msg ==0)  //自动切换
	{
		_TDC_GP21.system_statu.running_statu = FIRST;  //不需要标定会自动切换
	}
	else if(ctl_msg ==1)
	{
		_TDC_GP21.system_statu.running_statu = IDLE;   //不切换
		lk_gear_switch(lk03_first_gears);
	  sensor_distOffset_calculate(lk03_first_gears);
	}
	else if(ctl_msg ==2)
	{
		_TDC_GP21.system_statu.running_statu = IDLE;   //不切换
	 lk_gear_switch(lk03_second_gears);
	 sensor_distOffset_calculate(lk03_second_gears);
	}
	else if(ctl_msg ==3)
	{
		_TDC_GP21.system_statu.running_statu = IDLE;   //不切换
	 lk_gear_switch(lk03_third_gears);
	 sensor_distOffset_calculate(lk03_third_gears);
	}	
}

void sensor_debuMde_pidCtl_ack(TF_Msg *msg)
{
	uint8_t ctl_msg= (uint8_t) msg->data;
	if(ctl_msg ==1)
	{
	  _TDC_GP21.pid.ifTrunOn = true;  //pid打开
	}
	else if(ctl_msg ==0)
	{
		_TDC_GP21.pid.ifTrunOn = false;  //pid关闭
	}
}

void sensor_debug_programerStrShow_ack(TF_Msg *msg)
{
	uint8_t ctl_msg= (uint8_t) msg->data;
	if(ctl_msg ==1)
	{
	  ifDebug_strShow = true;  //pid打开
	}
	else if(ctl_msg ==0)
	{
		ifDebug_strShow = false;  //pid关闭
	}
}

void sensor_user_StrShow_ack(TF_Msg *msg)
{
	uint8_t ctl_msg= (uint8_t) msg->data;
	if(ctl_msg ==1)
	{
	  ifUser_strShow = true;  //pid打开
	}
	else if(ctl_msg ==0)
	{
		ifUser_strShow = false;  //pid关闭
	}


}

extern void start_singnal(void);
 void sensor_struct_loop(sensor_struct_ * p)
 {
 	  switch(p->cmd)
		{
		  case sensor_idle:
			{
			
			}break;
		  case dist_continue_ack_cmd:
			{
				lk_bsp_power_on();
				start_singnal();
				vTaskResume(xHandleDataOutFeq);  //恢复数据输出任务
			}break;		
		  case dist_once_ack_cmd:
			{
			    
			}break;			
		  case dist_stop_ack_cmd:
			{
				 lk_bsp_power_off();
				  vTaskSuspend(xHandleDataOutFeq);   //挂起任务
			    sensor_distStop_ack();   //停止测量
			}break;
		  case dist_null_cmd:
			{
				 zTF_NullDistAck();  //无效数据应答
			}break;			
		  case get_paramAll_base_cmd:
			{
			    sensor_getAllParam_ack();
			}break;			
		  case getParam_baudRate_ack_cmd:
			{
			     sensor_getBaudRate_ack();
			}break;
		  case getParam_frontSwich_ack_cmd:
			{
			    sensor_getFrontSwich_ack();
			}break;
		  case getParam_backSwich_ack_cmd:
			{
				  sensor_getBackSwich_ack();
			}break;	
		  case getParam_disBase_ack_cmd:
			{
			    sensor_getDisBase_ack();
			}break;
		  case getParam_powerOn_mode_ack_cmd:
			{
			     sensor_getPowerOnMode_ack();
			}break;			
		  case getParam_outData_freq_ack_cmd:
			{
			     sensor_getOutDataFreq_ack();
			}break;
		  case cfgParam_all_cmd:
			{
			    sensor_setAllParam_ack(p->msg);
			}break;
		  case cfgParam_baudRate_ack_cmd:
			{
			    sensor_setBaudRate_ack(p->msg);
			}break;
		  case cfgParam_frontSwich_ack_cmd:
			{
			    sensor_setFrontSwich_ack(p->msg);
			}break;
		  case cfgParam_backSwich_ack_cmd:
			{
			    sensor_setBackSwich_ack(p->msg);   
			}break;
		  case cfgParam_distBase_ack_cmd:
			{
			    sensor_setDisBase_ack(p->msg);
			}break;
		  case cfgParam_powerOn_mode_ack_cmd:
			{
			    sensor_setPowerOnMode_ack(p->msg);
			}break;
		  case cfgParam_outData_freq_ack_cmd:
			{
			    sensor_setOutDataFreq_ack(p->msg);  
			}break;
		  case qc_get_param_cmd:
			{
			    sensor_qc_getParam();
			}break;			
		  case qc_standFirst_switch_cmd:
			{
			   sensor_qc_standFirst_switch_ack();
			}break;			
		  case qc_standSecond_switch_cmd:
			{
			  sensor_qc_standSecond_switch_ack();
			}break;			
		  case qc_standthird_switch_cmd:
			{
			  sensor_qc_standthird_switch_ack();
			}break;			
		  case qc_standFirst_reset_cmd:
			{
				if(_TDC_GP21.sensor_runMode == qc_typeMode)  //在标定模式下
				{
					sensor_qc_standFirst_reset_ack();
				}

			}break;	
		  case qc_standSecond_reset_cmd:
			{
				if(_TDC_GP21.sensor_runMode == qc_typeMode)  //在标定模式下
				{
					sensor_qc_standSecond_reset_ack();
				}
			}break;	
		  case qc_standthird_reset_cmd:
			{
				if(_TDC_GP21.sensor_runMode == qc_typeMode)  //在标定模式下
				{				
					sensor_qc_standthird_reset_ack();
				}
			}break;	
		  case qc_standFirst_save_cmd:
			{
				if(_TDC_GP21.sensor_runMode == qc_typeMode)  //在标定模式下
				{				
					sensor_qc_standFirst_save_ack(p->msg);
				}
			}break;	
		  case qc_standSecond_save_cmd:
			{
				if(_TDC_GP21.sensor_runMode == qc_typeMode)  //在标定模式下
				{				
			   sensor_qc_standSecond_save_ack(p->msg);
				}
			}break;				
		  case qc_standthird_save_cmd:
			{
				if(_TDC_GP21.sensor_runMode == qc_typeMode)  //在标定模式下
				{				
					sensor_qc_standthird_save_ack(p->msg);
				} 
			}break;				
		  case system_boot_paramReset_ack_cmd:
			{
			   sensor_system_boot_paramReset_ack();
			}break;
		  case system_boot_firmware_ctl_ack_cmd:
			{
			
			}break;
		  case system_boot_firmware_pakage_ack_cmd:
			{
			
			}break;
		  case programer_debugMode_cmd: //调试模式
			{
				runTypeModeString = typeModeString[0];
				praogramer_debug_mode(p->msg);
			}break;
		  case programer_qcStamdMode_cmd:  //标定模式
			{	
				runTypeModeString = typeModeString[1];
				_TDC_GP21.sensor_runMode = qc_typeMode ;
				system_runStatu_select(_TDC_GP21.sensor_runMode);
				zTF_programer_standMode_switch_ack();
			}break; 
		  case programer_normalMode_cmd:  //正常模式
			{
				runTypeModeString = typeModeString[2];
        _TDC_GP21.sensor_runMode = normal_typeMode ;
        system_runStatu_select(_TDC_GP21.sensor_runMode);				
			}break;
			case programer_debug_stringShow_cmd :  //调试信息显示
			{
				
			  
			}break;
			case cfgParam_stringShow_cmd :  //用户字符串显示
			{
				 sensor_displaySet_ack(p->msg);
			   
			}break;	
			case getParam_stringShow_cmd :  //获取字符串显示设置
			{
				
			  
			}break;				
		}
    p->cmd = sensor_idle;
 
 }


 void baudRateCfg_select(sensor_baudRate_typeEnum cmd)
 {
	 sensor_baudRate_typeEnum baudRate = cmd;
    switch(baudRate)
		{
			case baudRate_9600:
			{
				 z_serial_baudRateCfg(9600);
			}break;
			case baudRate_14400:
			{
				z_serial_baudRateCfg(14400);
			}break;
			case baudRate_19200:
			{
				z_serial_baudRateCfg(19200);
			}break;
			case baudRate_38400:
			{
				z_serial_baudRateCfg(38400);
			}break;
			case baudRate_57600:
			{
				z_serial_baudRateCfg(57600);
			}break;
			case baudRate_115200:
			{
				z_serial_baudRateCfg(115200);
			}break;
			
		}
 
 }
 
 