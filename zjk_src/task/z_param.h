#ifndef _Z_PARAM_H
#define _Z_PARAM_H
#include "stdbool.h"
#include "cmsis_os.h"

#define DRIVER_SERIAL_HUART huart1
#define DRIVER_ADC_HADC hadc1
#define DRIVER_SPI_TDC   hspi2
#define DRIVER_SPI_TLC5618   hspi1
/*����״̬*/
typedef struct
{
  bool ifParamSave;
	bool ifParamGet;
	bool ifGetOnceDist;
	bool ifContinuDist;
	bool ifStopContinu;

}lk_statu_;

/*�ṹ��Ӧ����ָ������ݳ���*/
typedef struct
{
 uint8_t *point;
 uint8_t lens;
}arrayByte_;

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
}parm_;


#pragma pack()
#define DIST_ONCE_BIT_0      (1<<0)
#define DIST_CONTINUE_BIT_1  (1<<1)
#define Dist_Stop_BIT_2      (1<<2)
#define EVENT_ALL_BIT (DIST_ONCE_BIT_0|DIST_CONTINUE_BIT_1|Dist_Stop_BIT_2)
//����
extern parm_ lk_parm;
extern parm_ lk_flash;
//����
void parmSend(parm_ *parm);
void zTF_sendOnceDist(uint8_t *data,uint8_t lens);   //���Ͳ�������
arrayByte_ structToBytes(parm_ *p);       //�����ṹת��Ӧ����ṹ
extern lk_statu_ lk_param_statu;
extern EventGroupHandle_t xTinyFrameEventGroup;
#endif
