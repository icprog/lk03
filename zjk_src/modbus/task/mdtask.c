#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "mb.h"
#include "mbport.h"
#include "mbutils.h"
#include "z_os.h"

#define REG_HOLDING_ENABLE

#ifdef REG_INPUT_ENABLE
//输入寄存器起始地址
#define REG_INPUT_START 1000
//输入寄存器数量
#define REG_INPUT_NREGS 8
//寄存器起始地址
static USHORT usRegInputStart = REG_INPUT_START;
//输入寄存器内容
static USHORT usRegInputBuf[REG_INPUT_NREGS];
#endif

#ifdef REG_HOLDING_ENABLE
//保持寄存器起始地址
#define REG_HOLDING_START     0x0000
//保持寄存器数量
#define REG_HOLDING_NREGS     8
//保持寄存器内容
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0x147b,0x3f8e,0x147b,0x400e,0x1eb8,0x4055,0x147b,0x408e};
//保持寄存器起始地址
uint16_t usRegHoldingStart = REG_HOLDING_START;

#endif


#ifdef  REG_COILS_ENABLE
//线圈起始地址
#define REG_COILS_START       0x0000
//线圈数量
#define REG_COILS_SIZE        16

#endif

#ifdef REG_DISCRETE_ENABLE
//开关寄存器起始地址
#define REG_DISCRETE_START    0x0000
//开关寄存器数量
#define REG_DISCRETE_SIZE     16
#endif




#ifdef  REG_COILS_ENABLE
//线圈状态
uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = {0x01,0x02};
//开关输入状态
uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8] = {0x01,0x02};
#endif
void ModbusRTUTask(void const * argument)
{ 

  /* ABCDEF */
//  usRegInputBuf[0] = 11;
//  usRegInputBuf[1] = 22;
//  usRegInputBuf[2] = 33;
//  usRegInputBuf[3] = 44;
//  usRegInputBuf[4] = 55;
//  usRegInputBuf[5] = 66;
//  usRegInputBuf[6] = 77;
//  usRegInputBuf[7] = 88;  
  
  eMBErrorCode eStatus = eMBInit( MB_RTU, 1, 3, 19200, MB_PAR_NONE );
  eStatus = eMBEnable();
  
  while(1) {
    eMBPoll();
	 osDelay(100);  
  }

}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
	#ifdef REG_INPUT_ENABLE
    int             iRegIndex;
  //查询是否在寄存器范围内
  //为了避免警告，修改为有符号整数
    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
		//获得操作偏移量，本次操作起始地址-输入寄存器的初始地址
        iRegIndex = ( int )( usAddress - usRegInputStart );
		 //逐个赋值
        while( usNRegs > 0 )
        {
		  //赋值高字节
		  *pucRegBuffer++ = ( uint8_t )( usRegInputBuf[iRegIndex] >> 8 );
		  //赋值低字节
		  *pucRegBuffer++ = ( uint8_t )( usRegInputBuf[iRegIndex] & 0xFF );
		  //偏移量增加
		  iRegIndex++;
		  //被操作寄存器数量递减
		  usNRegs--;
        }
				
		//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    }
    else
    {
		//返回错误状态，无寄存器  
	    //  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
        eStatus = MB_ENOREG;			
    }
   
    return eStatus;
	 #endif
}

/**
  * @brief  保持寄存器处理函数，保持寄存器可读，可读可写
  * @param  pucRegBuffer  读操作时--返回数据指针，写操作时--输入数据指针
  *         usAddress     寄存器起始地址
  *         usNRegs       寄存器长度
  *         eMode         操作方式，读或者写
  * @retval eStatus       寄存器状态
  */
eMBErrorCode 
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
  //错误状态
  eMBErrorCode    eStatus = MB_ENOERR;
  //偏移量
  int16_t         iRegIndex;
  
  //判断寄存器是不是在范围内
  if( ( (int16_t)usAddress >= REG_HOLDING_START ) \
     && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
  {
    //计算偏移量
    iRegIndex = ( int16_t )( usAddress - usRegHoldingStart );
    
    switch ( eMode )
    {
      //读处理函数  
      case MB_REG_READ:
        while( usNRegs > 0 )
        {
          *pucRegBuffer++ = ( uint8_t )( usRegHoldingBuf[iRegIndex] >> 8 );
          *pucRegBuffer++ = ( uint8_t )( usRegHoldingBuf[iRegIndex] & 0xFF );
          iRegIndex++;
          usNRegs--;
        }
        break;

      //写处理函数 
      case MB_REG_WRITE:
        while( usNRegs > 0 )
        {
          usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
          usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
          iRegIndex++;
          usNRegs--;
        }
        break;
     }
  }
  else
  {
    //返回错误状态
    eStatus = MB_ENOREG;
  }
  
  return eStatus;
}



/**
  * @brief  线圈寄存器处理函数，线圈寄存器可读，可读可写
  * @param  pucRegBuffer  读操作---返回数据指针，写操作--返回数据指针
  *         usAddress     寄存器起始地址
  *         usNRegs       寄存器长度
  *         eMode         操作方式，读或者写
  * @retval eStatus       寄存器状态
  */
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
  //错误状态
  eMBErrorCode    eStatus = MB_ENOERR;
	#ifdef  REG_COILS_ENABLE
  //寄存器个数
  int16_t         iNCoils = ( int16_t )usNCoils;
  //寄存器偏移量
  int16_t         usBitOffset;

  //检查寄存器是否在指定范围内
  if( ( (int16_t)usAddress >= REG_COILS_START ) &&
        ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
  {
    //计算寄存器偏移量
    usBitOffset = ( int16_t )( usAddress - REG_COILS_START );
    switch ( eMode )
    {
      //读操作
      case MB_REG_READ:
        while( iNCoils > 0 )
        {
          *pucRegBuffer++ = xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
                                          ( uint8_t )( iNCoils > 8 ? 8 : iNCoils ) );
          iNCoils -= 8;
          usBitOffset += 8;
        }
        break;

      //写操作
      case MB_REG_WRITE:
        while( iNCoils > 0 )
        {
          xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
                        ( uint8_t )( iNCoils > 8 ? 8 : iNCoils ),
                        *pucRegBuffer++ );
          iNCoils -= 8;
        }
        break;
    }

  }
  else
  {
    eStatus = MB_ENOREG;
  }
  #endif
  return eStatus;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
	
  //错误状态
  eMBErrorCode    eStatus = MB_ENOERR;
	#ifdef REG_DISCRETE_ENABLE
  //操作寄存器个数
  int16_t         iNDiscrete = ( int16_t )usNDiscrete;
  //偏移量
  uint16_t        usBitOffset;

  //判断寄存器时候再制定范围内
  if( ( (int16_t)usAddress >= REG_DISCRETE_START ) &&
        ( usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE ) )
  {
    //获得偏移量
    usBitOffset = ( uint16_t )( usAddress - REG_DISCRETE_START );
    
    while( iNDiscrete > 0 )
    {
      *pucRegBuffer++ = xMBUtilGetBits( ucRegDiscreteBuf, usBitOffset,
                                      ( uint8_t)( iNDiscrete > 8 ? 8 : iNDiscrete ) );
      iNDiscrete -= 8;
      usBitOffset += 8;
    }

  }
  else
  {
    eStatus = MB_ENOREG;
  }
  #endif
  return eStatus;
}