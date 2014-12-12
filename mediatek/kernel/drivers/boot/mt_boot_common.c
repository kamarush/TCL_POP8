#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/kfifo.h>

#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>

#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot_common.h>

#define MOD "BOOT_COMMON"

/* this vairable will be set by mt_fixup.c */
BOOTMODE g_boot_mode __nosavedata = UNKNOWN_BOOT;
boot_reason_t g_boot_reason __nosavedata = BR_UNKNOWN;

/* return boot reason */
boot_reason_t get_boot_reason(void)
{
    return g_boot_mode;
}

/* set boot reason */
void set_boot_reason (boot_reason_t br)
{
    g_boot_reason = br;
}

/* return boot mode */
BOOTMODE get_boot_mode(void)
{
    return g_boot_mode;
}

/* set boot mode */
void set_boot_mode (BOOTMODE bm)
{
    g_boot_mode = bm;
}

/* for convenience, simply check is meta mode or not */
bool is_meta_mode(void)
{   
    if(g_boot_mode == META_BOOT)
    {   
        return true;
    }
    else
    {   
        return false;
    }
}

bool is_advanced_meta_mode(void)
{
    if (g_boot_mode == ADVMETA_BOOT)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static int boot_mode_proc(char *page, char **start, off_t off,int count, int *eof, void *data)
{
    char *p = page;
    int len = 0; 

    p += sprintf(p, "\n\rMTK BOOT MODE : " );
    switch(g_boot_mode)
    {
        case NORMAL_BOOT :
            p += sprintf(p, "NORMAL BOOT\n");
            break;
        case META_BOOT :
            p += sprintf(p, "META BOOT\n");
            break;
        case ADVMETA_BOOT :
            p += sprintf(p, "Advanced META BOOT\n");
            break;   
        case ATE_FACTORY_BOOT :
            p += sprintf(p, "ATE_FACTORY BOOT\n");
            break;
        case ALARM_BOOT :
            p += sprintf(p, "ALARM BOOT\n");
            break;
        default :
            p += sprintf(p, "UNKNOWN BOOT\n");
            break;
    }  
    *start = page + off;
    len = p - page;
    if (len > off)
        len -= off;
    else
        len = 0;

    return len < count ? len  : count;     
}

static int __init boot_common_init(void)
{
    /* create proc entry at /proc/boot_mode */
    create_proc_read_entry("boot_mode", S_IRUGO, NULL, boot_mode_proc, NULL);
    
    return 0;
}

static void __exit boot_common_exit(void)
{
    
}

module_init(boot_common_init);
module_exit(boot_common_exit);
MODULE_DESCRIPTION("MTK Boot Information Common Driver");
MODULE_LICENSE("Proprietary");
EXPORT_SYMBOL(is_meta_mode);
EXPORT_SYMBOL(is_advanced_meta_mode);
EXPORT_SYMBOL(get_boot_mode);
