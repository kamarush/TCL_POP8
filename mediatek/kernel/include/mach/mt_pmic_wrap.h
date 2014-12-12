#ifndef __MT_PMIC_WRAP_H__
#define __MT_PMIC_WRAP_H__
//#include <mach/typedefs.h>
//#include <linux/smp.h>
#include <mach/mt_typedefs.h>
#include <linux/device.h>


struct mt_pmic_wrap_driver{

    struct device_driver driver;
	S32 (*wacs2_hal)( U32  write, U32  adr, U32  wdata, U32 *rdata );

	S32 (*show_hal)(char *buf);
	S32 (*store_hal)(const char *buf, size_t count);

	S32 (*suspend)(void);
	void (*resume)(void);
};
typedef enum {
    PWRAP_READ	= 0,
    PWRAP_WRITE	= 1,
}PWRAP_OPS;

//------external API for pmic_wrap user--------------------------------------------------
S32 pwrap_read( U32  adr, U32 *rdata );
S32 pwrap_write( U32  adr, U32  wdata );

S32 pwrap_wacs2( U32  write, U32  adr, U32  wdata, U32 *rdata );
S32 pwrap_init( void );
struct mt_pmic_wrap_driver *get_mt_pmic_wrap_drv(void);

#endif //__MT_PMIC_WRAP_H__
