#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "z_os.h"



/* zos error code definitions  zdef.h*/


/*zdebug.h*/





struct z_device
{

};



/////////////////////////////////////////////////
struct serial_configure
{
	z_uint32_t baud_rate;
	z_uint32_t data_bits 				 :4;
	z_uint32_t stop_bits 				 :2;
	z_uint32_t parity    				 :2;
	z_uint32_t bit_order             	 :1;
	z_uint32_t invert                    :1;
	z_uint32_t bufsz                     :16;
	z_uint32_t reserved                  :6;	
};

/*
 * Serial FIFO mode 
 */
struct z_serial_rx_fifo
{
    /* software fifo */
    z_uint8_t *buffer;

    z_uint16_t put_index, get_index;

    z_bool_t is_full;
};

struct z_serial_device
{


};

typedef struct z_serial_device z_serial_t;



/**
 * uart operators
 */
 struct z_usart_ops
 {
	z_err_t (*configure)(struct z_serial_device *serial,struct serial_configure *cfg);
	z_err_t (*control)(struct z_serial_device *serial, int cmd, void *arg); 
	int (*putc)(struct z_serial_device *serial, char c);
	int (*getc)(struct z_serial_device *serial); 
	z_size_t (*dma_transmit)(struct z_serial_device *serial, z_uint8_t *buf, z_size_t size, int direction);	 
 };



#endif
