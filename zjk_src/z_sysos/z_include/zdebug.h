#ifndef __ZDEBUG_H__ 
#define __ZDEBUG_H__ 
#include "ztconfig.h"
#define Z_ASSERT(EX)                                                         \
if (!(EX))                                                                   \
{                                                                            \
   z_assert_handler(#EX, __FUNCTION__, __LINE__);                          \
}


#endif	// __ZDEBUG_H__