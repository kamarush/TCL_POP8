#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(540)
#define FRAME_HEIGHT 										(960)
#define LCM_DSI_CMD_MODE 0
#define LCM_ID_NT35517                                                          	 0x5517                              

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define REGFLAG_DELAY             							0XFD
#define REGFLAG_END_OF_TABLE      							0xFE   // END OF REGISTERS MARKER


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)     

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
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

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out

    {0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},


    // Display ON
    {0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 0, {0x00}},

    // Sleep Mode On
    {0x10, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	
	{REGFLAG_DELAY, 100, {}},

	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
	{0xB1,3,{0x7C,0x04,0x00}},
	{0xB8,4,{0x01,0x02,0x02,0x02}},
	{0xBB,3,{0x63,0x03,0x63}},
	{0xC9,6,{0x61,0x06,0x0D,0x1A,0x17,0x00}},
	  
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
	{0xB0,3,{0x05,0x05,0x05}},
	{0xB1,3,{0x05, 0x05, 0x05}},
	{0xB2,3,{0x01, 0x01, 0x01}},
	{0xB3,3,{0x0E, 0x0E, 0x0E}},
	{0xB4,3,{0x0A, 0x0A, 0x0A}},
	{0xB6,3,{0x44,0x44,0x44}},
	{0xB7,3,{0x34,0x34,0x34}},
	{0xB8,3,{0x24, 0x24, 0x24}},
	{0xB9,3,{0x26, 0x26, 0x26}},
	{0xBA,3,{0x24, 0x24, 0x24}},

	{0xBC,3,{0x00,0xC8,0x00}},
	{0xBD,3,{0x00, 0xC8, 0x00}},

	{0xBC,3,{0x00,0xA0,0x00}},
	{0xBD,3,{0x00,0xA0,0x00}},
	//{0xBE,1,{0x4B}},
	{0xBF,1,{0x3d}},        // 3F
	{0xC0,2,{0x00,0x08}},
	{0xCA,1,{0x00}},

	{0xD0,4,{0x0A,0x10,0x0D,0x0F}},
	{0xD1,16,{0x00,0x15,0x00,0x40,0x00,0x6D,0x00,0x8C,0x00,0x9E,0x00,0xCB,0x00,0xEC,0x01,0x1C}},
	{0xD5,16,{0x00,0x15,0x00,0x40,0x00,0x6D,0x00,0x8C,0x00,0x9E,0x00,0xCB,0x00,0xEC,0x01,0x1C}},
	{0xD9,16,{0x00,0x15,0x00,0x40,0x00,0x6D,0x00,0x8C,0x00,0x9E,0x00,0xCB,0x00,0xEC,0x01,0x1C}},
	{0xE0,16,{0x00,0x15,0x00,0x40,0x00,0x6D,0x00,0x8C,0x00,0x9E,0x00,0xCB,0x00,0xEC,0x01,0x1C}},
	{0xE4,16,{0x00,0x15,0x00,0x40,0x00,0x6D,0x00,0x8C,0x00,0x9E,0x00,0xCB,0x00,0xEC,0x01,0x1C}},
	{0xE8,16,{0x00,0x15,0x00,0x40,0x00,0x6D,0x00,0x8C,0x00,0x9E,0x00,0xCB,0x00,0xEC,0x01,0x1C}},

	{0xD2,16,{0x01,0x40,0x01,0x7B,0x01,0xA8,0x01,0xED,0x02,0x26,0x02,0x28,0x02,0x5F,0x02,0x9B}},
	{0xD6,16,{0x01,0x40,0x01,0x7B,0x01,0xA8,0x01,0xED,0x02,0x26,0x02,0x28,0x02,0x5F,0x02,0x9B}},
	{0xDD,16,{0x01,0x40,0x01,0x7B,0x01,0xA8,0x01,0xED,0x02,0x26,0x02,0x28,0x02,0x5F,0x02,0x9B}},
	{0xE1,16,{0x01,0x40,0x01,0x7B,0x01,0xA8,0x01,0xED,0x02,0x26,0x02,0x28,0x02,0x5F,0x02,0x9B}},
	{0xE5,16,{0x01,0x40,0x01,0x7B,0x01,0xA8,0x01,0xED,0x02,0x26,0x02,0x28,0x02,0x5F,0x02,0x9B}},
	{0xE9,16,{0x01,0x40,0x01,0x7B,0x01,0xA8,0x01,0xED,0x02,0x26,0x02,0x28,0x02,0x5F,0x02,0x9B}},

	{0xD3,16,{0x02,0xC2,0x02,0xF7,0x03,0x1D,0x03,0x4F,0x03,0x72,0x03,0x9C,0x03,0xB4,0x03,0xD6}},
	{0xD7,16,{0x02,0xC2,0x02,0xF7,0x03,0x1D,0x03,0x4F,0x03,0x72,0x03,0x9C,0x03,0xB4,0x03,0xD6}},
	{0xDE,16,{0x02,0xC2,0x02,0xF7,0x03,0x1D,0x03,0x4F,0x03,0x72,0x03,0x9C,0x03,0xB4,0x03,0xD6}},
	{0xE2,16,{0x02,0xC2,0x02,0xF7,0x03,0x1D,0x03,0x4F,0x03,0x72,0x03,0x9C,0x03,0xB4,0x03,0xD6}},
	{0xE6,16,{0x02,0xC2,0x02,0xF7,0x03,0x1D,0x03,0x4F,0x03,0x72,0x03,0x9C,0x03,0xB4,0x03,0xD6}},
	{0xEA,16,{0x02,0xC2,0x02,0xF7,0x03,0x1D,0x03,0x4F,0x03,0x72,0x03,0x9C,0x03,0xB4,0x03,0xD6}},

	{0xD4,4,{0x03,0xF6,0x03,0xFF}},
	{0xD8,4,{0x03,0xF6,0x03,0xFF}},
	{0xDF,4,{0x03,0xF6,0x03,0xFF}},
	{0xE3,4,{0x03,0xF6,0x03,0xFF}},
	{0xE7,4,{0x03,0xF6,0x03,0xFF}},
	{0xEB,4,{0x03,0xF6,0x03,0xFF}},

	{0xC0,1,{0xC0}},
	{0xC2,1,{0x20}},

	{0xFF,4,{0xAA,0x55,0x25,0x01}},
	{0x6F,1,{0x0B}},
	{0xF4,4,{0x12,0x12,0x56,0x13}},
     
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Display ON
	{0x29, 1, {0x00}},
	//{REGFLAG_DELAY, 50, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}


};

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

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = BURST_VDO_MODE;
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
    params->dsi.intermediat_buffer_num = 0; //2;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active				= 4;//2
    params->dsi.vertical_backporch					= 40;//50
    params->dsi.vertical_frontporch					= 40;//20
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 4;//4//
    params->dsi.horizontal_backporch				= 80;
    params->dsi.horizontal_frontporch				= 80;
    //params->dsi.horizontal_blanking_pixel 			=60; //60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    params->dsi.noncont_clock = TRUE; 


    // Use EXT TE monitor
    //params->dsi.lcm_ext_te_monitor = 1;


    // Bit rate calculation
    params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
    params->dsi.fbk_div =19;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real//20)	///==24

    /*params->dsi.LPX = 4;
    params->dsi.TA_GET= 20;
    params->dsi.TA_GO= 16;
    params->dsi.TA_SURE= 6;*/
}

static void init_lcm_registers(void)
{
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

extern void lcm_init_notp(void);
extern void lcm_init_otp(void);
static void lcm_init(void)
{
   unsigned int array[16];
   unsigned char buffer_ef[1];
   static struct LCM_setting_table lcm_initialization_page1[]={
   {0xF0,5,{0x55,0xAA,0x52,0x08,0x01}}
   };
   
   SET_RESET_PIN(1);
   MDELAY(50);
   SET_RESET_PIN(0);
   MDELAY(10);
   SET_RESET_PIN(1);
   //MDELAY(120);
   MDELAY(100); 
   push_table(lcm_initialization_page1, sizeof(lcm_initialization_page1) / sizeof(struct LCM_setting_table), 1);
   MDELAY(10);
   array[0] = 0x00013700;
   dsi_set_cmdq(array, 1, 1);
   MDELAY(10);
   read_reg_v2(0xEF,buffer_ef, 1);
#ifndef BUILD_LK
   printk("chlu*****************%x\n",buffer_ef[0]);
#endif
    if(buffer_ef[0] == 0x8B)
              lcm_init_otp();
    else
	      lcm_init_notp();
}
 void lcm_init_notp(void)
{

    init_lcm_registers();

}

 void lcm_init_otp(void)
{
    unsigned int array[16];  
    init_lcm_registers();

    array[0]=0x00023902;
    array[1]=0x00003DBF;
    dsi_set_cmdq(array, 2, 1); 
}


static void lcm_suspend(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);	
    MDELAY(120);

    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static int lcm_esd_check()
{
#ifndef BUILD_UBOOT
#ifndef BUILD_LK

	unsigned int array[16];
	unsigned char buffer_0a[1];
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0A,buffer_0a, 1);

	printk("lcm leibo 0x0a is %x--------------\n", buffer_0a[0]);
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
static void lcm_resume(void)
{
    lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0, id1 = 0, id2 = 0;
	unsigned char buffer[2];
	unsigned int data_array[16];
	static int checktimes = 1;

	if (checktimes == 3)
		return 1;
	checktimes++;  // checktimes should be less than 3 times, there is an error for LCD (NO LCD or LCD is broken).

	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);	

		
//*************Enable CMD2 Page1  *******************//
	data_array[0]=0x00063902;
	data_array[1]=0x52AA55F0;
	data_array[2]=0x00000108;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10); 


	
	data_array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10); 
	read_reg_v2(0xC5, buffer, 2);
	id1 = buffer[0]; //we only need ID
	id2= buffer[1]; //we test buffer 1
	id = (id1<<8) | id2;


	/*
	#ifdef BUILD_LK
		printf("lcm_compare_id NT35517 uboot %s \n", __func__);
		printf("lcm_compare_id %s id = 0x%08x \n", __func__, id);
	#else
		printk("lcm_compare_id NT35517 uboot %s \n", __func__);
		printk("lcm_compare_id %s id = 0x%08x \n", __func__, id);
	#endif
*/
       return (LCM_ID_NT35517 == id)?1:0;

}

LCM_DRIVER  nt35517_yaris_xxl_tdt_drv = 
{
    .name			= "nt35517_yaris_xxl_tdt_qhd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
#ifndef MMITEST_ESD_SKIP
   .esd_check      = lcm_esd_check,
   .esd_recover    = lcm_esd_recover,
#endif
};
