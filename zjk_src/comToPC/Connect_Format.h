#ifndef _Connect_Format_H
#define _Connect_Format_H

#include "stm32f1xx_hal.h"
#include "z_serial.h"

#define  usart1_send_char(c)   z_serial_write(c,1)
void usart1_niming_report(uint8_t fun,uint8_t*data,uint8_t len);
void usart1_niming_report(uint8_t fun,uint8_t*data,uint8_t len);
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);

//发送上位机 V4.2
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
void Send_Pose_Data(uint16_t *a,uint16_t *b,uint16_t *c);
void Data_Send_Status(float Pitch,float Roll,float Yaw,int16_t *gyro,int16_t *accel);
void Send_Pose_IData(uint16_t *Gyro,int16_t *Accel);
#endif

