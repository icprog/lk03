#ifndef _Z_PARAM_H
#define _Z_PARAM_H
#include "stdbool.h"
#include "cmsis_os.h"
//799 10mhei
//#3 f:798,g450;s:780,g450
//#2 f:716,g389;s:695,g383
#define TEST_QC 1
#define DEBUG_DISPLAY 1
#if TEST_QC 
//#define DIST_FIRST_OFFSET   716
//#define DIST_SECOND_OFFSET   695
#define DIST_FIRST_OFFSET   798
#define DIST_SECOND_OFFSET   780
#define first_test   true
#define second_test   true
#endif
#define LK03_STAND_COUNTS  3     //�궨����3
typedef enum{LK03_FIRST_STAND=0,LK03_SECOND_STAND,LK03_THIRD_STAND} _LK03_STAND;
typedef enum{DIST_TYPE=1}TypedSend;  //����Э�鹦�ܶ���
/*����״̬*/
typedef struct
{
  bool ifParamSave;
	bool ifParamGet;
	bool ifGetOnceDist;
	bool ifContinuDist;
	bool ifStopContinu;
  bool ifQCStand[LK03_STAND_COUNTS];
	bool ifQCgetParm;
	bool ifQCgetParmReset[LK03_STAND_COUNTS];
}lk_statu_;

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
	uint32_t baud_rate; //������
	uint16_t limit_trigger; //���޾��봥��
	uint8_t  red_laser_light; //���⼤��
	uint8_t front_or_base;//ǰ���׼
  uint8_t ifHasConfig;     //�Ƿ��Ѿ�����
	uint16_t outFreq;
	QC_TYP QC[LK03_STAND_COUNTS];   //����3���궨
}parm_;



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
//qc�궨��������
void QCparmSend(uint8_t *data,uint8_t lens);
void parmSend(parm_ *parm);
void zTF_sendOnceDist(uint8_t *data,uint8_t lens);   //���Ͳ�������
arrayByte_ structToBytes(parm_ *p);       //�����ṹת��Ӧ����ṹ
extern lk_statu_ lk_param_statu;
extern EventGroupHandle_t xTinyFrameEventGroup;
#endif
