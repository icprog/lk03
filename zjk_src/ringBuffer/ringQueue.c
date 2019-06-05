/**
  ******************************************************************************
  * @file           : ringqueue.c
  * @author         : zjk
  * @version        : v1.0
  * @date           : 
  * @brief          : 
  *
  ******************************************************************************
  * @attention  顺序存储循环队列
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
  * @brief  循环队列初始化
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
  * @brief  返回Q的元素的个数，也就是当前队列的长度
  * @param  None
  * @retval None
  */
 int QueueLength(SqQueue *Q)
 {
	 return (Q->rear - Q->front + MAXSIZE)%MAXSIZE;
 }
 /**
  * @brief  入队列，若队列未满，则插入元素e为新的队尾元素
  * @param  None
  * @retval None
  */
 QStatus Queue_push(SqQueue*Q,QElemType e)
 {
	 if((Q->rear + 1)%MAXSIZE == Q->front)   /*队列 满 */
	 return Q_ERROR;
	 Q->data[Q->rear] = e;   //将元素e赋值给队尾
	 Q->rear =  (Q->rear +1 )%MAXSIZE;  /*real 指针向后移动，若到最后则转到数组头部*/
     return Q_OK;
 }
  /**
  * @brief  出队列，若队列不空，则删除Q中对头元素，用e返回其值
  * @param  None
  * @retval None
  */
QStatus Queue_pop(SqQueue*Q,QElemType* e)
{
    if(Q->front == Q->rear)   /*队列空判断 */
	return Q_ERROR;
	*e = Q->data[Q->front];   //将队头元素付给e
	Q->front = (Q->front + 1)%MAXSIZE;
    return Q_OK;
 }
 
 	









/************************ (C) COPYRIGHT ZiFiSense *****END OF FILE****/
