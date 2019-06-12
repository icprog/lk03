#include "z_checksum.h"

uint8_t tx_chexkSum(uint8_t *buf,uint8_t lens)
{
	uint8_t ret=0;
	for(int i=0; i<lens;i++)
	{
	   ret+=*(buf++);
	}
	ret =~ret;
	return ret;
}



uint8_t rx_chexkSum(uint8_t *buf,uint8_t lens)
{
	uint8_t ret=0;
	for(int i=0; i<lens;i++)
	{
	   ret+=*(buf++);
	}
	ret = ret;
	return ret;
}


