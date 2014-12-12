#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
#include <mach/pmic_mt6329_hw_bank1.h> 
#include <mach/pmic_mt6329_sw_bank1.h> 
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#include "a080ean01_dsi_vdo.h"

extern int disp_bls_set_backlight(unsigned int level);
extern bool is_pop8_TRUST_LCM;
#define ERROR_BL_LEVEL 0xFFFFFFFF


#define BL_A  40
#define BL_B  1023
#define BL_A1  10
#define BL_B1  1023     //908
#define BL_B1_TRUST  908
unsigned int brightness_mapping(unsigned int level)
{
	
	unsigned int mapped_level;
	
	mapped_level = level;
	
	if (mapped_level > 1023)
		mapped_level = 1023;

#if TCL_PROTO_PCB 
	mapped_level = (mapped_level*24)/100;
#else
	if(mapped_level==0)
	{
		//printk("brightness_mapping:mapped_level=0\n");
	}
	else if(mapped_level<BL_A)
	{
		//printk("brightness_mapping:mapped_level<%d\n",BL_A);
		mapped_level = BL_A1;  //mapped_level/4;
	}	
	else
	{
		if(is_pop8_TRUST_LCM)
			mapped_level=((mapped_level-BL_A)*(BL_B1_TRUST-BL_A1))/(BL_B-BL_A)+BL_A1;
		else
			mapped_level=((mapped_level-BL_A)*(BL_B1-BL_A1))/(BL_B-BL_A)+BL_A1;
	}
#endif
	printk("brightness_mapping:level=%d      map level=%d\n",level,mapped_level);

	return mapped_level;
}


/*
unsigned int Cust_SetBacklight(int level, int div)
{
	kal_uint32 ret=0;
//    mtkfb_set_backlight_pwm(div);
//    mtkfb_set_backlight_level(brightness_mapping(level));

//	hwPWMsetting(MT65XX_PMIC_PWM_NUMBER PWMmode, kal_uint32 duty, kal_uint32 freqSelect);
//	hwBacklightBoostTuning(kal_uint32 MODE, kal_uint32 VRSEL, 0);
//	hwBacklightBoostTurnOn();
//	hwPWMsetting(0, , kal_uint32 freqSelect);
//	hwBacklightBoostTuning(kal_uint32 MODE, kal_uint32 VRSEL, 0);
//	hwBacklightBoostTurnOn();
//echo 15 13 > pmic_access_bank1
//echo 40 0A > pmic_access_bank1
//echo 3F 91 > pmic_access_bank1
//echo 2E 1F > pmic_access_bank1 
	printk("backlight temp solution, level=%d, div=%d\n", level, div);
	ret=pmic_bank1_config_interface(0x15, 0x13, 0xFF, 0x0);
	//backlight voltage
	ret=pmic_bank1_config_interface(0x40, 0x0A, 0xFF, 0x0);
	//bit0=1, enable boost backlight; bit2=0, CABC disable;bit5-bit4=01,PWM1;bit7=1,boost mode;
	ret=pmic_bank1_config_interface(0x3F, 0x91, 0xFF, 0x0);
	//PWM1 duty=32/32
	ret=pmic_bank1_config_interface(0x2E, 0x1F, 0xFF, 0x0);
    
    return 0;
}
*/


static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK2,{0}},
	{"green",             MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	{"blue",              MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
	{"button-backlight",  MT65XX_LED_MODE_NONE, -1,{0}},
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (int)disp_bls_set_backlight,{0,0,0,0,0}},
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

