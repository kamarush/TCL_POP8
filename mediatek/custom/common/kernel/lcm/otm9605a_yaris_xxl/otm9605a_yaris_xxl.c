#if defined(BUILD_LK)
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#if !defined(BUILD_LK)
#include <linux/string.h>
#endif
#include "lcm_drv.h"
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(540)
#define FRAME_HEIGHT 										(960)

#define REGFLAG_DELAY             								0xFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_OTM9605A									0x9605

#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	
{0x00,1,{0x00}},
{0xFF,3,{0x96,0x05,0x01}},
{0x00,1,{0x80}},
{0xFF,2,{0x96,0x05}},
{0x00,1,{0x92}},
{0xFF,2,{0x10,0x02}}, // Fix to 2lane setting

{0x00,1,{0x00}},
{0x00,1,{0x00}},
{0x00,1,{0x00}},
{0x00,1,{0x00}},

{0x00,1,{0x91}},
{0xC5,1,{0x76}},
{0x00,1,{0x00}},
{0xD8,2,{0x8F,0x8F}}, //0x6F,0x6F
{0x00,1,{0x00}},
{0xD9,1,{0x28}},
{0x00,1,{0xA6}},
{0xC1,1,{0x01}},
{0x00,1,{0xB4}},
{0xC0,1,{0x50}}, // 1+2 dot :0x10; column: 0x50;

//c089
//{0x00,1,{0x89}},
{0x00,1,{0x80}},
{0xC1,2,{0x36,0x66}},
{0x00,1,{0x89}},
{0xC0,1,{0x01}}, 
{0x00,1,{0xB1}},
{0xC5,1,{0x28}}, // adjust vdd1.8v level
{0x00,1,{0xB2}},
{0xF5,4,{0x15,0x00,0x15,0x00}},
{0x00,1,{0xC0}},
{0xC5,1,{0x00}}, // thermo disable
{0x00,1,{0x88}},
{0xC4,1,{0x01}}, // CR enable
{0x00,1,{0xA2}},
{0xC0,3,{0x20,0x18,0x09}},// cr,pl.pcg
{0x00,1,{0x80}},// source output levels during porch
{0xC4,1,{0x9C}},
////////////////// CE function improve code end ///////////////////////////
{0x00,1,{0xA0}},
{0xC1,2,{0x02}}, //for esd, close the function that switch Screen IC to control Scraper when ESD is working


//{REGFLAG_DELAY, 100, {}},

{0x00,1,{0x80}},
{0xCB,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0x90}},
{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xA0}},
{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0xB0}},
{0xCB,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0xC0}},
{0xCB,15,{0x00,0x00,0x04,0x04,0x04,0x04,0x00,0x00,0x04,0x04,0x04,0x04,0x00,0x00,0x00}},
{0x00,1,{0xD0}},
{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x00,0x00,0x04,0x04}},
{0x00,1,{0xE0}},
{0xCB,10,{0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xF0}},
{0xCB,10,{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
{0x00,1,{0x80}},
{0xCC,10,{0x00,0x00,0x26,0x25,0x02,0x06,0x00,0x00,0x0A,0x0E}},
{0x00,1,{0x90}},
{0xCC,15,{0x0C,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x25,0x01}},
{0x00,1,{0xA0}},
{0xCC,15,{0x05,0x00,0x00,0x09,0x0D,0x0B,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0xB0}},
{0xCC,10,{0x00,0x00,0x25,0x26,0x05,0x01,0x00,0x00,0x0F,0x0B}},

{0x00,1,{0xC0}},
{0xCC,15,{0x0D,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x26,0x06}},
{0x00,1,{0xD0}},
{0xCC,15,{0x02,0x00,0x00,0x10,0x0C,0x0E,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x00,1,{0x80}},
{0xCE,12,{0x87,0x03,0x28,0x86,0x03,0x28,0x00,0x0F,0x00,0x00,0x0F,0x00}},
{0x00,1,{0x90}},
{0xCE,14,{0x33,0xBE,0x28,0x33,0xBF,0x28,0xF0,0x00,0x00,0xF0,0x00,0x00,0x00,0x00}},
{0x00,1,{0xA0}},
{0xCE,14,{0x38,0x03,0x83,0xC0,0x8A,0x18,0x00,0x38,0x02,0x83,0xC1,0x89,0x18,0x00}},

{0x00,1,{0xB0}},
{0xCE,14,{0x38,0x01,0x83,0xC2,0x88,0x18,0x00,0x38,0x00,0x83,0xC3,0x87,0x18,0x00}},
{0x00,1,{0xC0}},
{0xCE,14,{0x30,0x00,0x83,0xC4,0x86,0x18,0x00,0x30,0x01,0x83,0xC5,0x85,0x18,0x00}},
{0x00,1,{0xD0}},
{0xCE,14,{0x30,0x02,0x83,0xC6,0x84,0x18,0x00,0x30,0x03,0x83,0xC7,0x83,0x18,0x00}},
{0x00,1,{0xC0}},
{0xCF,10,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x01,0x00,0x00}},

{0x00,1,{0x00}},
{0xE1,16,{0x00,0x11,0x18,0x0E,0x07,0x10,0x0C,0x0B,0x02,0x06,0x0C,0x08,0x0E,0x0F,0x09,0x03}},
{0x00,1,{0x00}},
{0xE2,16,{0x00,0x11,0x18,0x0E,0x07,0x10,0x0C,0x0B,0x02,0x06,0x0C,0x08,0x0E,0x0F,0x09,0x03}}, 

{0x00,1,{0x00}},
{0xEC,33,{0x40,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x44,0x34,0x04}},               

{0x00,1,{0x00}},
{0xED,33,{0x40,0x55,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x33,0x04}},               


{0x00,1,{0x00}},
{0xEE,33,{0x40,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x33,0x33,0x44,0x44,0x34,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x45,0x55,0x55,0x54,0x04}},               


{0x00,1,{0x00}},
{0xFF,3,{0xFF,0xFF,0xFF}},

{0x11, 1, {0}},
{REGFLAG_DELAY, 120, {}},
{0x29, 1, {0}},
{REGFLAG_DELAY, 10, {}},

{REGFLAG_END_OF_TABLE, 0x00, {}}

	
};



static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 8;
		params->dsi.vertical_frontporch					= 8;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 6;
		params->dsi.horizontal_backporch				= 37;
		params->dsi.horizontal_frontporch				= 37;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	// Bit rate calculation
   	 params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    	params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
   	 params->dsi.fbk_div =19;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real//20)	///==24



		/* ESD or noise interference recovery For video mode LCM only. */ // Send TE packet to LCM in a period of n frames and check the response. 
		params->dsi.lcm_int_te_monitor = FALSE; 
		params->dsi.lcm_int_te_period = 1; // Unit : frames 
 
		// Need longer FP for more opportunity to do int. TE monitor applicably. 
		if(params->dsi.lcm_int_te_monitor) 
			params->dsi.vertical_frontporch *= 2; 
 
		// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.) 
		params->dsi.lcm_ext_te_monitor = FALSE; 
		// Non-continuous clock 
		params->dsi.noncont_clock = TRUE; 
		params->dsi.noncont_clock_period = 2; // Unit : frames
}


static void lcm_init(void)
{
/*
#if defined(BUILD_LK)
        printf(" BUILD_LK lcm_init_0914");
        printf("BUILD_LK lcm_init_0914");
#elif defined(BUILD_UBOOT)
        printf(" BUILD_UBOOT lcm_init_0914");
        printf("BUILD_UBOOT lcm_init_0914");
#else
        printk(" lcm_compare_id lcm_init_0914");
        printk("lcm_compare_id lcm_init_0914");
#endif
*/
SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
//	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	lcm_init();
	
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

/*
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;


	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}


static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 145;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

*/

static int lcm_esd_check()
{
#ifndef BUILD_UBOOT
#ifndef BUILD_LK

	unsigned int array[16];
	unsigned char buffer_0a[1];
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0A,buffer_0a, 1);

	printk("lcm 0x0a is %x--------------\n", buffer_0a[0]);
	if(buffer_0a[0] == 0x9c)
		return 0;
	else
		return 1;
#endif
#endif
}
static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_UBOOT
#ifndef BUILD_LK
	lcm_init();
	return 1;
#endif 
#endif 
}

static unsigned int lcm_compare_id(void)
{
    int array[4];
    char buffer[5];
    char id_high=0;
    char id_low=0;
    int id=0;
    static int checktimes = 1;

    if (checktimes == 3)
        return 1;
    checktimes++;  // checktimes should be less than 3 times, there is an error for LCD (NO LCD or LCD is broken).

   SET_RESET_PIN(1);
    MDELAY(10);
   SET_RESET_PIN(0);
   MDELAY(10);
   SET_RESET_PIN(1);
   MDELAY(120);

	array[0] = 0x00053700;
      dsi_set_cmdq(array, 1, 1);
   	MDELAY(10); 
	read_reg_v2(0xa1, buffer, 5);   

	id_high = buffer[2];
	id_low = buffer[3];
	id = (id_high<<8) | id_low;


   #ifdef BUILD_LK
       printf("lcm_compare_id OTM9605A uboot %s \n", __func__);
       printf("lcm_compare_id %s id = 0x%08x \n", __func__, id);
   #else
       printk("lcm_compare_id OTM9605A kernel %s \n", __func__);
       printk("lcm_compare_id %s id = 0x%08x \n", __func__, id);
   #endif

	return (LCM_ID_OTM9605A == id)?1:0;
}


LCM_DRIVER otm9605a_yaris_xxl_byd_drv = 
{
    .name			= "otm9605a_yaris_xxl_byd_qhd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
#ifndef MMITEST_ESD_SKIP
    .esd_check      = lcm_esd_check,
    .esd_recover    = lcm_esd_recover,
#endif
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};

