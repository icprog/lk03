#ifndef _Z_PARAM_H
#define _Z_PARAM_H
#include "stdbool.h"
#include "cmsis_os.h"
#include "TinyFrame.h"
//799 10mhei
//#3 f:798,g450;s:780,g450
//#2 f:716,g389;s:695,g383
#define TEST_QC 0
#define DEBUG_DISPLAY 0
#if TEST_QC 
//#define DIST_FIRST_OFFSET   716
//#define DIST_SECOND_OFFSET   695
//2#�ṹ
#define DIST_FIRST_OFFSET    0
#define DIST_SECOND_OFFSET   780
#define DIST_THIED_OFFSET    780

//3#�ṹ
//#define DIST_FIRST_OFFSET    594
//#define DIST_SECOND_OFFSET   780
//#define DIST_THIED_OFFSET    780

#define first_test   true
#define second_test   true
	
#endif
#define LK03_STAND_COUNTS  3     //�궨����3

#define  PROTECL_HEAD_TAIL_SIZE 3  //0xff+cmd+tail=3

typedef enum{dist_cmd=1,ack_cmd=2,stand_param_cmd}TypedSend;  //����Э�鹦�ܶ���
typedef enum{first_mes1=1,first_mes2=2,}TypedSelextMode;  //
typedef enum{msg_thirdStard=1,msg_firstStard=2,msg_qcStard}TypedSelextMsgMode;  //
/*����״̬*/
//typedef struct
//{
//  bool ifParamSave;
//	bool ifParamGet;
//	bool ifGetOnceDist;
//	bool ifContinuDist;
//	bool ifStopContinu;
//  bool ifQCStand[LK03_STAND_COUNTS];    //����궨
//	bool ifQCgetParm;
//	bool ifQCgetParmReset[LK03_STAND_COUNTS];  //��λ�궨
//	bool ifstandSwitch[LK03_STAND_COUNTS];  //��λ�л�
//	bool  ifDownloadStart;   //  �̼�����
//}lk_statu_;



typedef enum{   sensor_idle, dist_continue_ack_cmd, dist_once_ack_cmd, dist_stop_ack_cmd,dist_null_cmd,
                get_paramAll_base_cmd= 0x10, getParam_baudRate_ack_cmd, getParam_frontSwich_ack_cmd, getParam_backSwich_ack_cmd, getParam_disBase_ack_cmd, getParam_powerOn_mode_ack_cmd,getParam_outData_freq_ack_cmd,
                cfgParam_all_cmd= 0x40, cfgParam_baudRate_ack_cmd, cfgParam_frontSwich_ack_cmd, cfgParam_backSwich_ack_cmd, cfgParam_distBase_ack_cmd, cfgParam_powerOn_mode_ack_cmd, cfgParam_outData_freq_ack_cmd,
                qc_get_param_cmd = 0x50, qc_standFirst_switch_cmd, qc_standSecond_switch_cmd, qc_standthird_switch_cmd, qc_standFirst_reset_cmd, qc_standSecond_reset_cmd, qc_standthird_reset_cmd, qc_standFirst_save_cmd, qc_standSecond_save_cmd, qc_standthird_save_cmd,
	              programer_debugMode_cmd = 0x60,programer_qcStamdMode_cmd,
	              system_boot_paramReset_ack_cmd = 0xf0, system_boot_firmware_ctl_ack_cmd, system_boot_firmware_pakage_ack_cmd,
}sensor_runnig_cmd_typEnum;

typedef enum { baudRate_9600=0,baudRate_14400,baudRate_19200,baudRate_38400,baudRate_57600,baudRate_115200   }sensor_baudRate_typeEnum;
typedef struct 	
{
  sensor_runnig_cmd_typEnum cmd;
	TF_Msg *msg;
}sensor_struct_;
/*�ṹ��Ӧ����ָ������ݳ���*/
typedef struct
{
 uint8_t *point;
 uint8_t lens;
}arrayByte_;


/*QC ����*/
typedef struct{
	uint16_t qc_stand_dist ;   //У׼ֵ
  uint16_t qc_ad603Gain;    //����ֵ
	bool ifHavedStand;   //�Ƿ��Ѿ��궨�����궨��ֵΪtrue,��֮Ϊfalse
}QC_TYP;


/*������flash��*/

#pragma pack(1)
typedef struct 
{
  uint8_t product;   //��Ʒ��� ; LK03 :0X02  LK02: 0X01
	uint8_t baud_rate; //�����ʣ�������0:9600;1:4400;2:19200;3:38400;4:57600;5:115200
	uint16_t front_limit_trigger; //ǰ���޾��봥��
	uint16_t back_limit_trigger;//�����޾��봥��
	uint8_t front_or_base;//ǰ���׼
  uint8_t ifHasConfig;     //�Ƿ��Ѿ�����
	uint8_t autoRunMode;  //�Զ�����ģʽ
	uint16_t outFreq;
  volatile QC_TYP QC[LK03_STAND_COUNTS];   //����3���궨
}parm_;

/*��������*/
typedef enum  { user_dist_ctl = 0x01,usr_paramCfg_get=0x02, user_paramCfg_set=0x03,
							  system_boot_firmware_ctl=0x20, system_boot_firmware_pakage,
							  system_boot_param = 0x30,
							  programer_ctl = 0xe0,
               }revFrame_Type_typEnum ;

					
typedef enum  { dist_continue = 0x01, dist_once=0x02, dist_stop=0x03}revFrame_distCtl_id_typEnum;
typedef enum  { lk_all_set = 0x00, baudRate_set, frontSwich_set, backSwich_set, disBase_set, powerOn_mode_set, outData_freq_set }revFrame_paramCfg_getId_typEnum;
typedef enum  { lk_all_get = 0x00, baudRate_get, frontSwich_get, backSwich_get, disBase_get, powerOn_mode_get, outData_freq_get }revFrame_paramCfg_setId_typEnum;
typedef enum  { qc_get_param = 0x00, qc_standFirst_switch, qc_standSecond_switch, qc_standthird_switch, qc_standFirst_reset, qc_standSecond_reset, qc_standthird_reset, qc_standFirst_save, qc_standSecond_save, qc_standthird_save,
	              debug_dist_continue=0xf1,sensor_dist_standMode_switch,
								}revFrame_programer_id_typEnum;  
typedef enum  { firmware_begin = 1 }revFrame_firmware_ctl_id_typEnum; 

typedef enum  { system_param_reset = 1 }revFrame_system_ctl_id_typEnum; 
//sendFrame
typedef enum {usr_ack = 0x10,programer_ack = 0xf0}sendFrame_Type_typEnum ;		
typedef enum  { dist_base=0,dist_continue_ack, dist_once_ack, dist_stop_ack,dist_null_ack,
                get_paramAll_base= 0x10, getParam_baudRate_ack, getParam_frontSwich_ack, getParam_backSwich_ack, getParam_disBase_ack, getParam_powerOn_mode_ack,getParam_outData_freq_ack,
                cfgParam_all= 0x40, cfgParam_baudRate_ack, cfgParam_frontSwich_ack, cfgParam_backSwich_ack, cfgParam_distBase_ack, cfgParam_powerOn_mode_ack, cfgParam_outData_freq_ack,
                system_boot_paramReset_ack = 0xf0, system_boot_firmware_ctl_ack, system_boot_firmware_pakage_ack,
						   }sendframe_user_ackId_typEnum;

typedef enum {qc_get_param_ack = 0x00, qc_standFirst_switch_ack, qc_standSecond_switch_ack, qc_standthird_switch_ack, qc_standFirst_reset_ack, qc_standSecond_reset_ack, qc_standthird_reset_ack, qc_standFirst_save_ack, qc_standSecond_save_ack, qc_standthird_save_ack,
							sensor_debugMode_switch_ack=0xf1,sensor_dist_standMode_switch_ack,
              }sendframe_programer_ackId_typEnum; 


//lk debug cmd id  
typedef enum  { debug_ID=1}FRAME_DEBUG_CMD;
typedef enum  { lk_getData_ack = 1, lk_saveParm_ack, lk_getParm_ack, lk_QC_ack = 6, lk_debug_ack = 7, lk_download_ack = 0xfe,lk_download_pakage_ack}FRAME_ACK_CMD;
typedef enum{z_type=0,z_id} z_type_id;

extern bool ifDebug;  //�Ƿ���Ա��


#pragma pack()
#define DIST_ONCE_BIT_0      (1<<0)
#define DIST_CONTINUE_BIT_1  (1<<1)
#define Dist_Stop_BIT_2      (1<<2)
#define EVENT_ALL_BIT (DIST_ONCE_BIT_0|DIST_CONTINUE_BIT_1|Dist_Stop_BIT_2)
//����
extern QC_TYP qc_param;
extern parm_ lk_defaultParm;
extern parm_ lk_flash;

//����
/***********************************************
   ������ԱӦ��
***********************************************/
void lk_programer_ack(sendframe_programer_ackId_typEnum id,uint8_t *data,uint8_t lens);
/***********************************************
   �û�ͨ�� Ӧ��
***********************************************/

void lk_user_ack(sendframe_user_ackId_typEnum id,uint8_t *data,uint8_t lens);
void zTF_sendOnceDistAck(uint8_t *data,uint8_t lens);
void zTF_StopDistAck(void);
void zTF_sendContinueDistAck(uint8_t *data,uint8_t lens);
//��Ч���ݷ���
void zTF_NullDistAck(void);
/*========��ȡ����Ӧ��================*/
void zTF_paramCfg_getAll_Ack(uint8_t *data,uint8_t lens);
void zTF_paramCfg_getBaudRate_Ack(uint8_t *data,uint8_t lens);
void zTF_paramCfg_getFrontSwich_Ack(uint8_t *data,uint8_t lens);
void zTF_paramCfg_getBackSwich_Ack(uint8_t *data,uint8_t lens);
void zTF_paramCfg_getDisBase_Ack(uint8_t *data,uint8_t lens);
void zTF_paramCfg_getPowerOnMode_Ack(uint8_t *data,uint8_t lens);
void zTF_paramCfg_getOutDataFreq_Ack(uint8_t *data,uint8_t lens);
/*========��������Ӧ��================*/
void zTF_paramCfg_setAll_Ack(void);
void zTF_paramCfg_setBaudRate_Ack(void);
void zTF_paramCfg_setFrontSwich_Ack(void);
void zTF_paramCfg_setBackSwich_Ack(void);
void zTF_paramCfg_setDisBase_Ack();
void zTF_paramCfg_setPowerOnMode_Ack(void);
void zTF_paramCfg_setOutDataFreq_Ack(void);
/*========systemӦ��================*/
void zTF_system_boot_paramReset_Ack(void);
void zTF_system_firmware_ctl_Ack(void);
void zTF_system_firmware_pakage_Ack(void);

/*========programerӦ��================*/
void zTF_programer_qc_getParam_Ack(uint8_t *data,uint8_t lens);
//�л�
void zTF_programer_qc_standFirst_switch_ack(void);
void zTF_programer_qc_standSecond_switch_ack(void);
void zTF_programer_qc_standthird_switch_ack(void);
//��λ
void zTF_programer_qc_standFirst_reset_ack(void);
void zTF_programer_qc_standSecond_reset_ack(void);
void zTF_programer_qc_standthird_reset_ack(void);
//�洢
void zTF_programer_qc_standFirst_save_ack(void);
void zTF_programer_qc_standSecond_save_ack(void);
void zTF_programer_qc_standthird_save_ack(void);
void clear_msgData(TF_Msg *msg);
//����Ӧ��
void zTF_programer_debug_sensorParam_ack(uint8_t *data,uint8_t lens);
void zTF_programer_standMode_switch_ack(void);
//qc�궨��������
void QCparmSend(uint8_t *data,uint8_t lens);
void parmSend(parm_ *parm);
void zTF_sendOnceDist(uint8_t *data,uint8_t lens);   //���Ͳ�������
arrayByte_ structToBytes(parm_ *p);       //�����ṹת��Ӧ����ṹ
extern EventGroupHandle_t xTinyFrameEventGroup;
void debugParmSend(uint8_t *data,uint8_t lens);
#endif
