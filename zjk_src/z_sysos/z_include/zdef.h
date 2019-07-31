#ifndef __ZDEF_H__ 
#define __ZDEF_H__ 

#define Z_EOK                          0               /**< There is no error */
#define Z_ERROR                        1               /**< A generic error happens */
#define Z_ETIMEOUT                     2               /**< Timed out */
#define Z_EFULL                        3               /**< The resource is full */
#define Z_EEMPTY                       4               /**< The resource is empty */
#define Z_ENOMEM                       5               /**< No memory */
#define Z_ENOSYS                       6               /**< No system */
#define Z_EBUSY                        7               /**< Busy */
#define Z_EIO                          8               /**< IO error */
#define Z_EINTR                        9               /**< Interrupted system call */
#define Z_EINVAL                       10              /**< Invalid argument */



/**
 * @ingroup BasicDef
 *
 * @def RT_NULL
 * Similar as the \c NULL in C library.
 */
#define RT_NULL                         (0)

typedef signed   char                   z_int8_t;      /**<  8bit integer type */
typedef signed   short                  z_int16_t;     /**< 16bit integer type */
typedef signed   int                    z_int32_t;     /**< 32bit integer type */
typedef unsigned char                   z_uint8_t;     /**<  8bit unsigned integer type */
typedef unsigned short                  z_uint16_t;    /**< 16bit unsigned integer type */
typedef unsigned int                    z_uint32_t;    /**< 32bit unsigned integer type */

typedef long                           z_base_t;      /**< Nbit CPU related date type */
typedef unsigned long                  z_ubase_t;     /**< Nbit unsigned CPU related data type */
typedef z_base_t                       z_err_t;       /**< Type for error number */
typedef z_ubase_t                      z_size_t;      /**< Type for size number */

typedef int                             z_bool_t;      /**< boolean type */


#endif	// __ZDEF_H__