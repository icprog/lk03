/**
  ******************************************************************************
  * @file           : ringqueue.h
  * @author         : liuchengfei
  * @version        : v1.0
  * @date           : 2017.09
  * @brief          : 
  *
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RINGQUEUE_H
#define __RINGQUEUE_H	 
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private types -------------------------------------------------------------*/
/**
  * @brief  ���д�С
  */
#define  MAXSIZE   1024
/**
  * @brief  
  */
typedef uint16_t QElemType;
typedef enum 
{
  Q_OK = 0, 
  Q_ERROR = !Q_OK
} QStatus;

/**
  * @brief  ѭ������˳��洢�ṹ 
  */
typedef struct
{
	QElemType data[MAXSIZE];
	int front;   /* ͷָ�� */
	int rear;    /* βָ�� */
}SqQueue;

/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/ 
QStatus QueueInit(SqQueue* Q);
int QueueLength(SqQueue *Q);
QStatus Queue_push(SqQueue*Q,QElemType e);
QStatus Queue_pop(SqQueue*Q,QElemType* e);



#endif /* __RINGQUEUE_H */
/************************ (C) COPYRIGHT ZiFiSense *****END OF FILE****/
