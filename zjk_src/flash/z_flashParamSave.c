#include "z_flashParamSave.h"

#define FLASH_SIZE 256 //所选MCU的FLASH容量大小（单位K）

#if FLASH_SIZE<256
	#define NUMBER_PAGE 1   //字节
#else
	#define NUMBER_PAGE 2   //页
#endif
FLASH_EraseInitTypeDef FlashTypeDef;	


void flash_writeMoreData( uint16_t *toWriteData, uint16_t toSize)
{

  HAL_FLASH_Unlock();  //解锁flash
  uint32_t PageError = 0;   
	HAL_FLASHEx_Erase(&FlashTypeDef,&PageError);     //擦除Flash
	
	for(int index=0;index<toSize;index++)
	{
	   HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,PARAM_FLASH_ADDRESS+index*2,toWriteData[index]); 
	}
 
  HAL_FLASH_Lock();  //上锁
}


void flash_SaveInit(void)
{
  
	FlashTypeDef.TypeErase = FLASH_TYPEERASE_PAGES;
	FlashTypeDef.PageAddress = PARAM_FLASH_ADDRESS;
	FlashTypeDef.NbPages = NUMBER_PAGE;
}

void flash_paramRead(uint8_t *buff, uint16_t lens)
{
	uint8_t *poitBuff = (uint8_t *)PARAM_FLASH_ADDRESS;
	  memcpy(buff,poitBuff,lens);  
}
