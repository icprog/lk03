#include "tlc5618.h"
#include "z_os.h"
extern SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef *tlc5618_spi = &hspi1 ;
#define tlc5618_idle()      HAL_GPIO_WritePin(tlc5618_cs_GPIO_Port,tlc5618_cs_Pin,GPIO_PIN_SET)
#define tlc5618_select()    HAL_GPIO_WritePin(tlc5618_cs_GPIO_Port,tlc5618_cs_Pin,GPIO_PIN_RESET)
#define tlc5618_release()   HAL_GPIO_WritePin(tlc5618_cs_GPIO_Port,tlc5618_cs_Pin,GPIO_PIN_SET)

//#define tlc5618_clk_high()  HAL_GPIO_WritePin(tlc5618_clk_GPIO_Port,tlc5618_clk_Pin,GPIO_PIN_SET)
//#define tlc5618_clk_low()   HAL_GPIO_WritePin(tlc5618_clk_GPIO_Port,tlc5618_clk_Pin,GPIO_PIN_RESET)

//#define tlc5618_dat_high()      HAL_GPIO_WritePin(tlc5618_dat_GPIO_Port,tlc5618_dat_Pin,GPIO_PIN_SET)         
//#define tlc5618_dat_low()       HAL_GPIO_WritePin(tlc5618_dat_GPIO_Port,tlc5618_dat_Pin,GPIO_PIN_RESET)        

#define ACHANNELCODE 0x8000
#define BCHANNELCODE 0x1000

#define BANDTWO 0x0000     //写寄存器B和双缓存器
void tlc5618_delay(uint16_t dly_val)
{
	for(int i = 0;i<dly_val;i++)
	{
	}
}


void tlc5618_send_byte(uint8_t byte)
{
  tlc5618_select();
  HAL_SPI_Transmit(tlc5618_spi,&byte,1,0xff);
	 tlc5618_release();
}

void tlc5618_send_16byte(uint16_t reg)
{
	HAL_StatusTypeDef tlc_tatus=HAL_ERROR;
	uint8_t sedata[2]={0};
    tlc5618_select();
	sedata[0]=reg>>8;
	sedata[1]=reg&0xff;
	tlc_tatus = HAL_SPI_Transmit(tlc5618_spi,sedata,2,0xff); 

	 Z_ASSERT(tlc_tatus == HAL_OK); 	

   tlc5618_release();
}



static inline void tlc5618_write_reg(uint16_t tlc_reg)
{

	HAL_StatusTypeDef tlc_tatus=HAL_ERROR;
	uint8_t sedata[2]={0};
    tlc5618_select();
	sedata[0]=tlc_reg>>8;
	sedata[1]=tlc_reg&0xff;
	tlc_tatus = HAL_SPI_Transmit(tlc5618_spi,sedata,2,0xff); 

	 //Z_ASSERT(tlc_tatus == HAL_OK);
   if(tlc_tatus != HAL_OK)
	 {
	  // zt_printf("tlc 5618 tansmit erro\r\n");
	 }		 
   tlc5618_release();
}


void tlc5618_writeBchannal(uint16_t tlc_reg)
{
	 tlc5618_write_reg(BANDTWO|tlc_reg);
}

void tlc5618_writeAchannal(uint16_t tlc_reg)
{
     tlc5618_write_reg(ACHANNELCODE|tlc_reg);	
}


void tlc5618_write(uint16_t tlc_a,uint16_t tlc_b)
{	
    tlc5618_write_reg(ACHANNELCODE|tlc_a);	  
    tlc5618_write_reg(BANDTWO|tlc_b);
}

