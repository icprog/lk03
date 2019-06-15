#ifndef Y_TINYFRAMEFILE_H
#define Y_TINYFRAMEFILE_H
#include "TinyFrame.h"
#include "stm32f1xx_hal.h"
#include "flash_if.h"
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Packet structure defines */


typedef enum
{
   HEAD_PARAM=1,
	 PACKED,
}Z_FILE_StatueTypedef;


/* Apps Address */
#define USR_APP_SIZE  60
#define FILE_NAME_LENGTH        ((uint32_t)64)
#define FILE_SIZE_LENGTH        ((uint32_t)16)


bool z_ReceivePacket(TF_Msg *msg);
#endif

