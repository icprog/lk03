/**
  ******************************************************************************
  * @file           : ringqueue.c
  * @author         : zjk
  * @version        : v1.0
  * @date           : 
  * @brief          : 
  *
  ******************************************************************************
  * @attention  ˳��洢ѭ������
  *
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "ringQueue.h" 


/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Exported function ----------------------------------------------------------*/

/**
  * @brief  ѭ�����г�ʼ��
  * @param  None
  * @retval None
  */
 QStatus QueueInit(SqQueue* Q)
 {
	  Q->front = 0;
   	Q->rear = 0;
	 return Q_OK;
 }
/**
  * @brief  ����Q��Ԫ�صĸ�����Ҳ���ǵ�ǰ���еĳ���
  * @param  None
  * @retval None
  */
 int QueueLength(SqQueue *Q)
 {
	 return (Q->rear - Q->front + MAXSIZE)%MAXSIZE;
 }
 /**
  * @brief  ����У�������δ���������Ԫ��eΪ�µĶ�βԪ��
  * @param  None
  * @retval None
  */
 QStatus Queue_push(SqQueue*Q,QElemType e)
 {
	 if((Q->rear + 1)%MAXSIZE == Q->front)   /*���� �� */
	 return Q_ERROR;
	 Q->data[Q->rear] = e;   //��Ԫ��e��ֵ����β
	 Q->rear =  (Q->rear +1 )%MAXSIZE;  /*real ָ������ƶ������������ת������ͷ��*/
     return Q_OK;
 }
  /**
  * @brief  �����У������в��գ���ɾ��Q�ж�ͷԪ�أ���e������ֵ
  * @param  None
  * @retval None
  */
QStatus Queue_pop(SqQueue*Q,QElemType* e)
{
    if(Q->front == Q->rear)   /*���п��ж� */
	return Q_ERROR;
	*e = Q->data[Q->front];   //����ͷԪ�ظ���e
	Q->front = (Q->front + 1)%MAXSIZE;
    return Q_OK;
 }
 
 	









/************************ (C) COPYRIGHT ZiFiSense *****END OF FILE****/
