#ifndef __Z_OS_H__ 
#define __Z_OS_H__ 
#include "zdebug.h"
#include "zdef.h"
#include "stm32f1xx_hal.h"

/*
 * general kernel service
 */
void zt_printf(const char *fmt, ...);
void zt_protecl_printf(const char *fmt, ...);
void zt_assert_handler(const char *ex_string, const char *func, z_size_t line);



#endif	// __Z_OS_H__

