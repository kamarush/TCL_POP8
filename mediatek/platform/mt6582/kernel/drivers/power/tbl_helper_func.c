#include <linux/xlog.h>
#include <mach/mt_typedefs.h>

#ifdef MTK_BQ24196_SUPPORT
#include "bq24196.h"
#endif

#ifdef MTK_BQ24297_SUPPORT
#include "bq24297.h"
#endif

/************* ATTENTATION ***************/
/* IF ANY NEW CHARGER IC SUPPORT IN THIS FILE, */
/* REMEMBER TO NOTIFY USB OWNER TO MODIFY OTG RELATED FILES!! */

#ifdef CONFIG_USB_MTK_HDRC_HCD
void tbl_charger_otg_vbus(int mode)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[tbl_charger_otg_vbus] mode = %d\n", mode);	
    
    if(mode&0xFF)
    {
#ifdef MTK_BQ24196_SUPPORT    
        bq24196_set_chg_config(0x3); //OTG
        bq24196_set_boost_lim(0x1); //1.3A on VBUS
        bq24196_set_en_hiz(0x0);
#endif        

#ifdef MTK_BQ24297_SUPPORT    
        bq24297_set_otg_config(0x1); //OTG
        bq24297_set_boost_lim(0x1); //1.5A on VBUS
        bq24297_set_en_hiz(0x0);
#endif        
    }
    else
    {
#ifdef MTK_BQ24196_SUPPORT        
        bq24196_set_chg_config(0x0); //OTG & Charge disabled
#endif        

#ifdef MTK_BQ24297_SUPPORT        
        bq24297_set_otg_config(0x0); //OTG & Charge disabled
#endif        
    }
};
#endif