#ifndef _Z_PARAM_H
#define _Z_PARAM_H
#include "stdbool.h"
#include "cmsis_os.h"

#define DRIVER_SERIAL_HUART huart1
#define DRIVER_ADC_HADC hadc1
#define DRIVER_SPI_TDC   hspi2
#define DRIVER_SPI_TLC5618   hspi1
/*参数状态*/
typedef struct
{
  bool ifParamSave;
	bool ifParamGet;
	bool ifGetOnceDist;
	bool ifContinuDist;
	bool ifStopContinu;

}lk_statu_;

/*结构对应数据指针和数据长度*/
typedef struct
{
 uint8_t *point;
 uint8_t lens;
}arrayByte_;

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
}parm_;


#pragma pack()
#define DIST_ONCE_BIT_0      (1<<0)
#define DIST_CONTINUE_BIT_1  (1<<1)
#define Dist_Stop_BIT_2      (1<<2)
#define EVENT_ALL_BIT (DIST_ONCE_BIT_0|DIST_CONTINUE_BIT_1|Dist_Stop_BIT_2)
//变量
extern parm_ lk_parm;
extern parm_ lk_flash;
//函数
void parmSend(parm_ *parm);
void zTF_sendOnceDist(uint8_t *data,uint8_t lens);   //发送测量距离
arrayByte_ structToBytes(parm_ *p);       //参数结构转对应数组结构
extern lk_statu_ lk_param_statu;
extern EventGroupHandle_t xTinyFrameEventGroup;
#endif
