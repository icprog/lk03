#include "z_serial.h"
#include "cmsis_os.h"
#include "stdarg.h"
#include "ctype.h"
#define MAX_LIST_SIZES 10

UART_HandleTypeDef  *z_serial = &huart1;
static void z_serial_dma_start(void);
uint16_t rev_lens = 0;
SemaphoreHandle_t  serialSemaphore =NULL;  //�����ź��������ڴ��ڿ����жϱ�־����֪ͨ����ִ��
uint8_t serial_rxbuf[50] = {0};
_dma_listen dma_listen[MAX_LIST_SIZES]={NULL};
uint16_t GetRevBytes(void);

/*
�������е������
*/
void z_serialDriverTask(void const * argument)
{
   	 BaseType_t xResult;
	   z_serial_init();    //dma start
	
   for(;;)
	  {
			xResult = xSemaphoreTake(serialSemaphore,portMAX_DELAY);
			if(xResult == pdTRUE)  //���յ����ڽ�������ź�
			{
				rev_lens = GetRevBytes(); //
					for(int i=0;i<MAX_LIST_SIZES;i++)
				{
					 if(dma_listen[i].listFunc != NULL)
					 {
						 dma_listen[i].listFunc(&serial_rxbuf[0]);
					 }
				}	   
				HAL_UART_DMAStop(z_serial);
				z_serial_dma_start(); 			
			}			
		
		}

}
//���ò�����
void z_serial_baudRateCfg(uint32_t baudRate)
{
	
  huart1.Instance = USART1;
  huart1.Init.BaudRate = baudRate;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }


}

//��ȡ���ո���
void get_revLens(uint16_t *data)
{
   *data = rev_lens;
}

/*
@func:������ݽ�����ɼ�������������������
������ɿ��к��Զ�����

*/
 SERIAL_TYPE_BOOL addUartDmaRevListen(_listenFunc func)
{
	
	   static uint8_t count=0;
	   dma_listen[count++].listFunc =func;		 
		 if(count >MAX_LIST_SIZES)  return SERIAL_FALSE;
		 return SERIAL_TRUE;
}

static void z_serial_dma_start(void)
{
  HAL_UART_Receive_DMA(z_serial,serial_rxbuf,sizeof(serial_rxbuf));

}

void z_serial_write( uint8_t *ch, uint32_t lens)
{
   sensor_rs485_dir_ouput();
   HAL_UART_Transmit(z_serial,ch,lens,0xff);
	 sensor_rs485_dir_input();
}

int fputc(int ch, FILE* stream)
{
     HAL_UART_Transmit(z_serial,(uint8_t*)&ch,1,0xff);
    return ch;
}

//���ڳ�ʼ��
void z_serial_init(void)
{
	  serialSemaphore = xSemaphoreCreateBinary();
    z_serial_dma_start();   //����dma����
	   __HAL_UART_ENABLE_IT(z_serial, UART_IT_IDLE);//ʹ�ܽ��տ�����
    if(serialSemaphore == NULL) //create fail
		{
		
		
		}		

}

//�ڴ����жϺ����е��ã�����Ƿ���У����DMA���ղ�����������
void usartIdleInt(void)
{
   uint32_t rcflag=0;
	BaseType_t XHigherPriorityTaskWoken = pdTRUE;
	rcflag = __HAL_UART_GET_FLAG(z_serial,UART_FLAG_IDLE);
	if(rcflag !=RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(z_serial);  //��������жϱ�־
		xSemaphoreGiveFromISR(serialSemaphore,&XHigherPriorityTaskWoken);
	}

}

/*�����ַ�����������ת��*/
static __inline int skip_atoi(const char **s)
{
    register int i = 0;
    while (isdigit(**s))
        i = i * 10 + *((*s)++) - '0';

    return i;
}
#define ZEROPAD     (1 << 0)    /* pad with zero */
#define SIGN        (1 << 1)    /* unsigned/signed long */
#define PLUS        (1 << 2)    /* show plus */
#define SPACE       (1 << 3)    /* space if plus */
#define LEFT        (1 << 4)    /* left justified */
#define SPECIAL     (1 << 5)    /* 0x */
#define LARGE       (1 << 6)    /* use 'ABCDEF' instead of 'abcdef' */

/**
 * This function will return the length of a string, which terminate will
 * null character.
 *
 * @param s the string
 *
 * @return the length of string
 */
uint32_t zt_strlen(const char *s)
{
    const char *sc;

    for (sc = s; *sc != '\0'; ++sc) /* nothing */
        ;

    return sc - s;
}

static __inline int divide(long *n, int base)
{
    int res;

    /* optimized for processor which does not support divide instructions. */
    if (base == 10)
    {
        res = (int)(((unsigned long)*n) % 10U);
        *n = (long)(((unsigned long)*n) / 10U);
    }
    else
    {
        res = (int)(((unsigned long)*n) % 16U);
        *n = (long)(((unsigned long)*n) / 16U);
    }

    return res;
}


#ifdef RT_PRINTF_PRECISION
static char *print_number(char *buf,
                          char *end,
#ifdef RT_PRINTF_LONGLONG
                          long long  num,
#else
                          long  num,
#endif
                          int   base,
                          int   s,
                          int   precision,
                          int   type)
#else
static char *print_number(char *buf,
                          char *end,
#ifdef RT_PRINTF_LONGLONG
                          long long  num,
#else
                          long  num,
#endif
                          int   base,
                          int   s,
                          int   type)
#endif
{
    char c, sign;
#ifdef RT_PRINTF_LONGLONG
    char tmp[32];
#else
    char tmp[16];
#endif
    int precision_bak = precision;
    const char *digits;
    static const char small_digits[] = "0123456789abcdef";
    static const char large_digits[] = "0123456789ABCDEF";
    register int i;
    register int size;

    size = s;

    digits = (type & LARGE) ? large_digits : small_digits;
    if (type & LEFT)
        type &= ~ZEROPAD;

    c = (type & ZEROPAD) ? '0' : ' ';

    /* get sign */
    sign = 0;
    if (type & SIGN)
    {
        if (num < 0)
        {
            sign = '-';
            num = -num;
        }
        else if (type & PLUS)
            sign = '+';
        else if (type & SPACE)
            sign = ' ';
    }

#ifdef RT_PRINTF_SPECIAL
    if (type & SPECIAL)
    {
        if (base == 16)
            size -= 2;
        else if (base == 8)
            size--;
    }
#endif

    i = 0;
    if (num == 0)
        tmp[i++] = '0';
    else
    {
        while (num != 0)
            tmp[i++] = digits[divide(&num, base)];
    }

#ifdef RT_PRINTF_PRECISION
    if (i > precision)
        precision = i;
    size -= precision;
#else
    size -= i;
#endif

    if (!(type & (ZEROPAD | LEFT)))
    {
        if ((sign) && (size > 0))
            size--;

        while (size-- > 0)
        {
            if (buf < end)
                *buf = ' ';
            ++ buf;
        }
    }

    if (sign)
    {
        if (buf < end)
        {
            *buf = sign;
        }
        -- size;
        ++ buf;
    }

#ifdef RT_PRINTF_SPECIAL
    if (type & SPECIAL)
    {
        if (base == 8)
        {
            if (buf < end)
                *buf = '0';
            ++ buf;
        }
        else if (base == 16)
        {
            if (buf < end)
                *buf = '0';
            ++ buf;
            if (buf < end)
            {
                *buf = type & LARGE ? 'X' : 'x';
            }
            ++ buf;
        }
    }
#endif

    /* no align to the left */
    if (!(type & LEFT))
    {
        while (size-- > 0)
        {
            if (buf < end)
                *buf = c;
            ++ buf;
        }
    }

#ifdef RT_PRINTF_PRECISION
    while (i < precision--)
    {
        if (buf < end)
            *buf = '0';
        ++ buf;
    }
#endif

    /* put number in the temporary buffer */
    while (i-- > 0 && (precision_bak != 0))
    {
        if (buf < end)
            *buf = tmp[i];
        ++ buf;
    }

    while (size-- > 0)
    {
        if (buf < end)
            *buf = ' ';
        ++ buf;
    }

    return buf;
}
uint32_t zt_vsnprintf(char       *buf,
                        uint32_t   size,
                        const char *fmt,
                        va_list     args)
{
#ifdef RT_PRINTF_LONGLONG
    unsigned long long num;
#else
    uint32_t num;
#endif
    int i, len;
    char *str, *end, c;
    const char *s;

    uint8_t base;            /* the base of number */
    uint8_t flags;           /* flags to print number */
    uint8_t qualifier;       /* 'h', 'l', or 'L' for integer fields */
    int32_t field_width;     /* width of output field */

#ifdef RT_PRINTF_PRECISION
    int precision;      /* min. # of digits for integers and max for a string */
#endif

    str = buf;
    end = buf + size;

    /* Make sure end is always >= buf */
    if (end < buf)
    {
        end  = ((char *) - 1);
        size = end - buf;
    }

    for (; *fmt ; ++fmt)
    {
        if (*fmt != '%')
        {
            if (str < end)
                *str = *fmt;
            ++ str;
            continue;
        }

        /* process flags */
        flags = 0;

        while (1)
        {
            /* skips the first '%' also */
            ++ fmt;
            if (*fmt == '-') flags |= LEFT;
            else if (*fmt == '+') flags |= PLUS;
            else if (*fmt == ' ') flags |= SPACE;
            else if (*fmt == '#') flags |= SPECIAL;
            else if (*fmt == '0') flags |= ZEROPAD;
            else break;
        }

        /* get field width */
        field_width = -1;
        if (isdigit(*fmt)) field_width = skip_atoi(&fmt);
        else if (*fmt == '*')
        {
            ++ fmt;
            /* it's the next argument */
            field_width = va_arg(args, int);
            if (field_width < 0)
            {
                field_width = -field_width;
                flags |= LEFT;
            }
        }

#ifdef RT_PRINTF_PRECISION
        /* get the precision */
        precision = -1;
        if (*fmt == '.')
        {
            ++ fmt;
            if (isdigit(*fmt)) precision = skip_atoi(&fmt);
            else if (*fmt == '*')
            {
                ++ fmt;
                /* it's the next argument */
                precision = va_arg(args, int);
            }
            if (precision < 0) precision = 0;
        }
#endif
        /* get the conversion qualifier */
        qualifier = 0;
#ifdef RT_PRINTF_LONGLONG
        if (*fmt == 'h' || *fmt == 'l' || *fmt == 'L')
#else
        if (*fmt == 'h' || *fmt == 'l')
#endif
        {
            qualifier = *fmt;
            ++ fmt;
#ifdef RT_PRINTF_LONGLONG
            if (qualifier == 'l' && *fmt == 'l')
            {
                qualifier = 'L';
                ++ fmt;
            }
#endif
        }

        /* the default base */
        base = 10;

        switch (*fmt)
        {
        case 'c':
            if (!(flags & LEFT))
            {
                while (--field_width > 0)
                {
                    if (str < end) *str = ' ';
                    ++ str;
                }
            }

            /* get character */
            c = (uint8_t)va_arg(args, int);
            if (str < end) *str = c;
            ++ str;

            /* put width */
            while (--field_width > 0)
            {
                if (str < end) *str = ' ';
                ++ str;
            }
            continue;

        case 's':
            s = va_arg(args, char *);
            if (!s) s = "(NULL)";

            len = zt_strlen(s);
#ifdef RT_PRINTF_PRECISION
            if (precision > 0 && len > precision) len = precision;
#endif

            if (!(flags & LEFT))
            {
                while (len < field_width--)
                {
                    if (str < end) *str = ' ';
                    ++ str;
                }
            }

            for (i = 0; i < len; ++i)
            {
                if (str < end) *str = *s;
                ++ str;
                ++ s;
            }

            while (len < field_width--)
            {
                if (str < end) *str = ' ';
                ++ str;
            }
            continue;

        case 'p':
            if (field_width == -1)
            {
                field_width = sizeof(void *) << 1;
                flags |= ZEROPAD;
            }
#ifdef RT_PRINTF_PRECISION
            str = print_number(str, end,
                               (long)va_arg(args, void *),
                               16, field_width, precision, flags);
#else
            str = print_number(str, end,
                               (long)va_arg(args, void *),
                               16, field_width, flags);
#endif
            continue;

        case '%':
            if (str < end) *str = '%';
            ++ str;
            continue;

        /* integer number formats - set up the flags and "break" */
        case 'o':
            base = 8;
            break;

        case 'X':
            flags |= LARGE;
        case 'x':
            base = 16;
            break;

        case 'd':
        case 'i':
            flags |= SIGN;
        case 'u':
            break;

        default:
            if (str < end) *str = '%';
            ++ str;

            if (*fmt)
            {
                if (str < end) *str = *fmt;
                ++ str;
            }
            else
            {
                -- fmt;
            }
            continue;
        }

#ifdef RT_PRINTF_LONGLONG
        if (qualifier == 'L') num = va_arg(args, long long);
        else if (qualifier == 'l')
#else
        if (qualifier == 'l')
#endif
        {
            num = va_arg(args, uint32_t);
            if (flags & SIGN) num = (int32_t)num;
        }
        else if (qualifier == 'h')
        {
            num = (uint16_t)va_arg(args, int32_t);
            if (flags & SIGN) num = (int16_t)num;
        }
        else
        {
            num = va_arg(args, uint32_t);
            if (flags & SIGN) num = (int32_t)num;
        }
#ifdef RT_PRINTF_PRECISION
        str = print_number(str, end, num, base, field_width, precision, flags);
#else
        str = print_number(str, end, num, base, field_width, flags);
#endif
    }

    if (size > 0)
    {
        if (str < end) *str = '\0';
        else
        {
            end[-1] = '\0';
        }
    }

    /* the trailing null byte doesn't count towards the total
    * ++str;
    */
    return str - buf;
}


/**
 * This function will print a formatted string on system console
 *
 * @param fmt the format
 */
#define ZT_CONSOLEBUF_SIZE   128
void zt_printf(const char *fmt, ...)
{
    va_list args;
	uint32_t length;
	static char z_log_buf[ZT_CONSOLEBUF_SIZE];
	va_start(args, fmt);
    /* the return value of vsnprintf is the number of bytes that would be
     * written to buffer had if the size of the buffer been sufficiently
     * large excluding the terminating null byte. If the output string
     * would be larger than the rt_log_buf, we have to adjust the output
     * length. */
    length = zt_vsnprintf(z_log_buf, sizeof(z_log_buf) - 1, fmt, args);	
    if (length > ZT_CONSOLEBUF_SIZE - 1)
        length = ZT_CONSOLEBUF_SIZE - 1;
	
#ifdef RT_USING_DEVICE
       HAL_UART_Transmit(z_serial,(uint8_t *)z_log_buf,length,0xff);
#else
    rt_hw_console_output(rt_log_buf);
#endif
    va_end(args);
}

void zt_protecl_printf(const char *fmt, ...)
{
    va_list args;
	uint32_t length;
	static char z_log_buf[ZT_CONSOLEBUF_SIZE];
	va_start(args, fmt);
    /* the return value of vsnprintf is the number of bytes that would be
     * written to buffer had if the size of the buffer been sufficiently
     * large excluding the terminating null byte. If the output string
     * would be larger than the rt_log_buf, we have to adjust the output
     * length. */
    length = zt_vsnprintf(z_log_buf, sizeof(z_log_buf) - 1, fmt, args);	
    if (length > ZT_CONSOLEBUF_SIZE - 1)
        length = ZT_CONSOLEBUF_SIZE - 1;
	
#ifdef RT_USING_DEVICE
	extern void zTF_programer_debug_sensorParam_ack(uint8_t *data,uint8_t lens);
       zTF_programer_debug_sensorParam_ack((uint8_t *)z_log_buf,length);
#else
    rt_hw_console_output(rt_log_buf);
#endif
    va_end(args);
}

//��ȡdma �����ֽڸ���
uint16_t GetRevBytes(void)
{
	uint16_t bytes=0;
  bytes = __HAL_DMA_GET_COUNTER(z_serial->hdmarx);
  bytes = sizeof(serial_rxbuf)-bytes;
	return bytes;
}

