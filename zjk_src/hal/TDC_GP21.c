#include "tdc_gp21.h"

//

HAL_StatusTypeDef  tdc_gp_statu;
bool tdc_cfg_complete = false;

#define CONTENT_REG0   ((uint32_t)( REG0_ANZ_FIRE(0) | REG0_DIV_FIRE(0) | REG0_ANZ_PER_CALRES(0) |     \
                        REG0_DIV_CLKHS(2) | REG0_START_CLKHS(1) |  REG0_ANZ_PORT(0) | REG0_TCYCLE(0) |  \
												REG0_ANZ_FAKE(0)  | REG0_SEL_ECLK_TEMP(1)| REG0_CALIBRATE(1) | REG0_NO_CAL_AUTO(0) | REG0_MESSB2(0) ))

#define CONTENT_REG1   (uint32_t)( REG1_HIT2(0) | REG1_HIT1(1) | REG1_HITIN2(0) |       \
												REG1_HITIN1(1) | REG1_EN_FAST_INIT(0) | REG1_CURR32K(0) |    \
												REG1_SEL_START_FIRE(0) | REG1_SEL_TSTO2(0) | REG1_SEL_TSTO1(0) )

#define CONTENT_REG2    (uint32_t)( REG2_EN_INT(0b101) | REG2_RFEDGE2(0) | REG2_RFEDGE1(0))

/* Opcodes -------------------------------------------------------------------*/   
uint8_t   Init =                0x70;   
uint8_t   Power_On_Reset =      0x50;   
uint8_t   Start_TOF =           0x01;   
uint8_t   Start_Temp =          0x02;   
uint8_t   Start_Cal_Resonator = 0x03;   
uint8_t   Start_Cal_TDC =       0x04;   
uint8_t   Start_TOF_Restart =   0x05;   
uint8_t   Start_Temp_Restart =  0x06;   
//variable

/* function -------------------------------------------------------------------*/
static void lk_gp2x_write_opcode(uint8_t op_code, uint32_t reg);
SPI_HandleTypeDef  *gp21_spi = &hspi2;
uint8_t rxbuf[5] = {0};
uint64_t id =0;

void lk_gp2x_init(void)
{
  gp21_rstn_idle();
	HAL_Delay(10);
	gp21_rstn_rsow();
  HAL_Delay(10);
	gp21_rstn_idle();
	HAL_Delay(10);
	gp21_close_startSignal();
	gp21_close_stop1Signal();
}


void gp21_hard_rst(void)
{
  gp21_rstn_idle();
	HAL_Delay(1);
	gp21_rstn_rsow();
  HAL_Delay(1);
	gp21_rstn_idle();
	HAL_Delay(1);
}

bool gp21_defaultcofg(void)
{
	
   lk_gp2x_write_opcode(OP_CODE_WR(0x00), 0x00242012);   //4��Ƶ
   //gp21_write_cfg(OP_CODE_WR(0x00), 0x00011202);   //����Ƶ,�ر��Զ�У׼
	lk_gp2x_write_opcode(OP_CODE_WR(0x01), 0x01410025);//0x01420023 //STOPͨ��1�����壬stopͨ��2�رգ����ٳ�ʼ����������,ALU��ǰ���ݴ���ļ��� stop ch1 -start
	//bit29 = 1 ALU ok
	//bit30 = 1 the received pulse counter is ready
	//bit31 = 1 TDC timeout overflow
	//gp21_write_cfg(OP_CODE_WR(0x02), 0xE0000011); //Timeout End Hits ALU�жϴ���, �������½���
	lk_gp2x_write_opcode(OP_CODE_WR(0x02), 0x20000011);
	lk_gp2x_write_opcode(OP_CODE_WR(0x03), 0x20000012); //����timeout ǿ��ALUд��0XFFFFFFFF������Ĵ������ر�		
	lk_gp2x_write_opcode(OP_CODE_WR(0x04), 0x20000013);  //Ĭ������
	lk_gp2x_write_opcode(OP_CODE_WR(0x05), 0x00000014);  //���崥�����رգ�������Ԫ�ر�
	HAL_Delay(5);
	lk_gp2x_write_opcode(OP_CODE_WR(0x06), 0x00000015);  //������..�ر�			
	id = lk_gp2x_get_id();

//������Χ2
   if(id == 0x12251112131415)
	 {
	   return true;
	 }
	 return false;
}


//����ģʽ2,Զ����
void gp21_messgeModeTwo(void)
{
	lk_gp2x_write_opcode(OP_CODE_WR(0x00), 0x00042812);   //0��Ƶ,������Χ2
	lk_gp2x_write_opcode(OP_CODE_WR(0x01), 0x21422025);//0x01420023 //STOPͨ��1�����壬stopͨ��2�رգ����ٳ�ʼ����������,ALU��ǰ���ݴ���ļ��� stop ch1 -start
	//bit29 = 1 ALU ok
	//bit30 = 1 the received pulse counter is ready
	//bit31 = 1 TDC timeout overflow
	//gp21_write_cfg(OP_CODE_WR(0x02), 0xE0000011); //Timeout End Hits ALU�жϴ���, �������½���
	lk_gp2x_write_opcode(OP_CODE_WR(0x02), 0xC0000011);
	lk_gp2x_write_opcode(OP_CODE_WR(0x03), 0x20000012); //����timeout ǿ��ALUд��0XFFFFFFFF������Ĵ������ر�		
	lk_gp2x_write_opcode(OP_CODE_WR(0x04), 0x20000013);  //Ĭ������
	lk_gp2x_write_opcode(OP_CODE_WR(0x05), 0x00000014);  //���崥�����رգ�������Ԫ�ر�
	HAL_Delay(5);
	lk_gp2x_write_opcode(OP_CODE_WR(0x06), 0x00000015);  //������..�ر�			
	lk_gp2x_get_id();	
}



HAL_StatusTypeDef gp21_statu;


/**
 * gp2x sensor regisiter write 
 *
 * @param reg:  configure
 * @return None
 */
void gp21_write_reg(uint8_t reg)
{
	uint8_t rg=reg;	
    lk_gp2x_select();	
	gp21_statu =  HAL_SPI_Transmit(gp21_spi,&rg,1,0xff); 
	lk_gp2x_release();
	if(gp21_statu !=HAL_OK) 
	{
	   	
	}	
}

/**
 * gp2x sensor regisiter write configure
 *
 * @param op_code:  gp2x op_code 
 * @param reg:  configure
 * @return None
 */
void lk_gp2x_write_opcode(uint8_t op_code, uint32_t reg)
{
	 uint8_t trant=op_code;
	 uint8_t cfg[4] = {0};
	 cfg[0] = reg>>24;
	 cfg[1] = reg>>16;	 
	 cfg[2] = reg>>8;	 	
	 cfg[3] = reg;	 		 
     lk_gp2x_select();
     gp21_statu =  HAL_SPI_Transmit(gp21_spi,&trant,1,0xff); 
	 HAL_SPI_Transmit(gp21_spi,cfg,4,0xff); 
	 lk_gp2x_release();
}
  

/**
 * gp2x sensor get id
 *
 * @param id  8bytes buff point
 * @return None
 */
uint64_t lk_gp2x_get_id(void)
{
  static uint64_t gp_id = 0;
  uint8_t gp2x_id[8]={0};
  uint8_t txcmd[8]= {OPC_ID,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
  lk_gp2x_select();
  HAL_SPI_TransmitReceive(gp21_spi,txcmd,gp2x_id,8,0xff);
  lk_gp2x_release();
  gp_id = (((uint64_t)gp2x_id[0]<<56 )|((uint64_t)gp2x_id[1]<<48 )|((uint64_t)gp2x_id[2]<<40 )|((uint64_t)gp2x_id[3]<<32 ) |
        ((uint64_t)gp2x_id[4]<<24 )|((uint64_t)gp2x_id[5]<<16 )| ((uint64_t)gp2x_id[4]<<24 )|((uint64_t)gp2x_id[6]<<8 ) |((uint64_t)gp2x_id[7] ));
  return gp_id;	 
}


uint32_t gp21_read_dword(uint8_t opcode)
{
   uint32_t result=0;
	//uint8_t txcmd[5] ={opcode,opcode,opcode,opcode,opcode};
	uint8_t txcmd[5] ={opcode,0xFF,0xff,0xff,0xff};
	lk_gp2x_select();
	HAL_SPI_TransmitReceive(gp21_spi,txcmd,rxbuf,5,0xffff);
//	HAL_SPI_Transmit(gp21_spi,txcmd,1,0xfff);
//	HAL_SPI_Receive(gp21_spi,rxbuf,5,0xffff);
   lk_gp2x_release();
	 result  = rxbuf[1]<<24 | rxbuf[2] <<16 | rxbuf[3] <<8 | rxbuf[4];   //���ݴ� �±�1��ʼ��Ч
  return result;
}

uint32_t status_gp2 = 0;
uint16_t  get_gp21_statu(void)
{
	
  status_gp2=gp21_read_dword(OP_CODE_RD(0x04));	
  status_gp2>>=16;
  return  status_gp2;
	
}

uint32_t   gp21_read_diatance(uint8_t index)
{

   return  gp21_read_dword(OP_CODE_RD(index));
}


  /*delay ns */
 void tdc_delay(uint32_t cval)
 {
    while(cval--);  
 }






