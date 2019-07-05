#ifndef _Z_PARAM_H
#define _Z_PARAM_H
#include "stdbool.h"
#include "cmsis_os.h"
//799 10mhei
//#3 f:798,g450;s:780,g450
//#2 f:716,g389;s:695,g383
#define TEST_QC 0
#define DEBUG_DISPLAY 0
#if TEST_QC 
//#define DIST_FIRST_OFFSET   716
//#define DIST_SECOND_OFFSET   695
//2#结构
#define DIST_FIRST_OFFSET    0
#define DIST_SECOND_OFFSET   780
#define DIST_THIED_OFFSET    780

//3#结构
//#define DIST_FIRST_OFFSET    594
//#define DIST_SECOND_OFFSET   780
//#define DIST_THIED_OFFSET    780

#define first_test   true
#define second_test   true
	
#endif
#define LK03_STAND_COUNTS  3     //标定次数3

#define  PROTECL_HEAD_TAIL_SIZE 3  //0xff+cmd+tail=3
typedef enum{LK03_FIRST_STAND=0,LK03_SECOND_STAND,LK03_THIRD_STAND} _LK03_STAND;
typedef enum{dist_cmd=1,ack_cmd=2,stand_param_cmd}TypedSend;  //发送协议功能定义
typedef enum{first_mes1=1,first_mes2=2,}TypedSelextMode;  //
typedef enum{msg_mode_one=1,msg_mode_second=2}TypedSelextMsgMode;  //
/*参数状态*/
typedef struct
{
  bool ifParamSave;
	bool ifParamGet;
	bool ifGetOnceDist;
	bool ifContinuDist;
	bool ifStopContinu;
  bool ifQCStand[LK03_STAND_COUNTS];    //保存标定
	bool ifQCgetParm;
	bool ifQCgetParmReset[LK03_STAND_COUNTS];  //复位标定
	bool ifstandSwitch[LK03_STAND_COUNTS];  //档位切换
}lk_statu_;

/*结构对应数据指针和数据长度*/
typedef struct
{
 uint8_t *point;
 uint8_t lens;
}arrayByte_;


/*QC 参数*/
typedef struct{
	uint16_t qc_stand_dist ;   //校准值
  uint16_t qc_ad603Gain;    //增益值
	bool ifHavedStand;   //是否已经标定过，标定过值为true,反之为false
}QC_TYP;


/*保存在flash中*/
#pragma pack(1)
typedef struct 
{
  uint8_t product;   //产品编号 ; LK03 :0X02  LK02: 0X01
	uint32_t baud_rate; //波特率
	uint16_t limit_trigger; //门限距离触发
	uint8_t  red_laser_light; //红外激光
	uint8_t front_or_base;//前后基准
  uint8_t ifHasConfig;     //是否已经配置
	uint16_t outFreq;
  volatile QC_TYP QC[LK03_STAND_COUNTS];   //设置3挡标定
}parm_;

/*发送命令*/
typedef enum  { DataDistSend = 1, ParmsConfig = 2, ParmaSend=3, QC=6,lk_debug,ErroSend }FRAME_TYPE_CMD ;
typedef enum  { ParamAll=1}FRAME_GetParam_CMD;
typedef enum  { DistOnce = 1, DistContinue,DistStop}FRAME_GetDataID_CMD;
typedef enum  { BarudRate = 1, RedLight, FrontOrBase,AutoMel}FRAME_ParmSaveID_CMD;
typedef enum  { standStart = 1,StandParamFirst,StandParamSecond,StandParamThird,StandParamFirstReset, StandParamSecondReset, StandParamThirdReset,StandFirstSwitch, StandSecondSwitch, StandThirdSwitch,GetParam}FRAME_ParmQC_CMD;

//lk debug cmd id  
typedef enum  { debug_ID=1}FRAME_DEBUG_CMD;

typedef enum{z_type=0,z_id} z_type_id;
extern const uint8_t distance_setCmd[DistStop][2];
extern const uint8_t param_getCmd[ParamAll][2];
extern const uint8_t param_configCmd[AutoMel][2];
extern const uint8_t qc_Cmd[GetParam][2];
extern bool ifDebug;  //是否调试标记


#pragma pack()
#define DIST_ONCE_BIT_0      (1<<0)
#define DIST_CONTINUE_BIT_1  (1<<1)
#define Dist_Stop_BIT_2      (1<<2)
#define EVENT_ALL_BIT (DIST_ONCE_BIT_0|DIST_CONTINUE_BIT_1|Dist_Stop_BIT_2)
//变量
extern QC_TYP qc_param;
extern parm_ lk_defaultParm;
extern parm_ lk_flash;

//函数
//qc标定参数发送
void QCparmSend(uint8_t *data,uint8_t lens);
void parmSend(parm_ *parm);
void zTF_sendOnceDist(uint8_t *data,uint8_t lens);   //发送测量距离
arrayByte_ structToBytes(parm_ *p);       //参数结构转对应数组结构
extern lk_statu_ lk_param_statu;
extern EventGroupHandle_t xTinyFrameEventGroup;
void debugParmSend(uint8_t *data,uint8_t lens);
#endif
