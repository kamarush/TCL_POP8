

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

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFFF  // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0
#define LCD_ID_IC_ILI9806C_ID                                                     (0x9816)
#define LCM_ID_GPIO_ILI9806C_BYD                                              (0x3)
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

 //C0504   
{0xFF, 3,{0xff,0x98,0x16}},
{0xBA, 1,{0x60}}, 
{0xB0, 1,{0x01}}, 
//{0xB6, 1,{0x22}}, 
{0xBC,18,{0x03,0x0D,0x03,0x63,0x01,0x01,0x1b,0x11,0x6E,0x00,0x00,0x00,0x01,0x01,0x16,0x00,0xff,0xf2}},
          
{0xBD, 8,{0x02,0x13,0x45,0x67,0x45,0x67,0x01,0x23}},

{0xBE,17,{0x03,0x22,0x22,0x22,0x22,0xdd,0xcc,0xbb,0xaa,0x66,0x77,0x22,0x22,0x22,0x22,0x22,0x22}},
 
{0xED, 2,{0x7f,0x0f}}, 

{0xF3,1,{0x70}}, 

{0xB4, 1,{0x02}}, 
 
{0xC0, 3,{0x0f,0x0b,0x0a}}, 
 
{0xC1, 4,{0x17,0x85,0x56,0x20}}, 
 
{0xD8, 1,{0x50}}, 
 
{0xFC, 1,{0x07}}, 
 
{0xE0,16,{0x00,0x05,0x14,0x14,0x12,0x1A,0xCC,0x0C,0x03,0x07,0x03,0x0F,0x11,0x2C,0x26,0x00}},
          
{0xE1,16,{0x00,0x01,0x02,0x05,0x0E,0x10,0x75,0x03,0x06,0x0B,0x0A,0x0C,0x0B,0x33,0x2C,0x00}},
 
{0xD5, 8,{0x0A,0x08,0x07,0x08,0xCB,0xA5,0x01,0x04}},
 
{0xF7, 1,{0x89}},
{0xC7, 1,{0x57}}, 
{0x35,1 ,{0x00}},   
{0x11, 0,{0}},  
{REGFLAG_DELAY, 120, {}}, 
{0x29, 0,{0}}, 
{REGFLAG_DELAY, 20, {}}, 
{REGFLAG_END_OF_TABLE, 0x00, {}}



#if 0//C0500    
{0xFF, 3,{0xff,0x98,0x16}},
{0xBA, 1,{0x60}}, 
{0xB0, 1,{0x01}}, 
//{0xB6, 1,{0x22}}, 
{0xBC,18,{0x03,0x0D,0x61,0xFF,0x01,0x01,0x1b,0x11,0x38,0x63,0xFF,0xFF,0x01,0x01,0x10,0x00,0xff,0xf2}},
          
{0xBD, 8,{0x02,0x13,0x45,0x67,0x45,0x67,0x01,0x23}},

{0xBE,17,{0x03,0x22,0x22,0x22,0x22,0xdd,0xcc,0xbb,0xaa,0x66,0x77,0x22,0x22,0x22,0x22,0x22,0x22}},
 
{0xED, 2,{0x7f,0x0f}}, 

{0xF3,1,{0x70}}, 

{0xB4, 1,{0x02}}, 
 
{0xC0, 3,{0x0f,0x0b,0x0a}}, 
 
{0xC1, 4,{0x17,0x80,0x68,0x20}}, 
 
{0xD8, 1,{0x50}}, 
 
{0xFC, 1,{0x07}}, 
 
{0xE0,16,{0x00,0x04,0x0C,0x12,0x13,0x1D,0xCA,0x09,0x04,0x0B,0x03,0x0B,0x0E,0x2D,0x2A,0x00}},
          
{0xE1,16,{0x00,0x01,0x04,0x0A,0x0E,0x11,0x79,0x09,0x04,0x08,0x08,0x0B,0x09,0x34,0x2E,0x00}},
 
{0xD5, 8,{0x0D,0x0A,0x05,0x05,0xCB,0xA5,0x01,0x04}},
 
{0xF7, 1,{0x89}},
{0xC7, 1,{0x6f}}, 
//{0x35,1 ,{0x00}},   
{0x11, 0,{0}},  
{REGFLAG_DELAY, 120, {}}, 
{0x29, 0,{0}}, 
{REGFLAG_DELAY, 20, {}}, 
{REGFLAG_END_OF_TABLE, 0x00, {}}
#endif

    
#if 0 //9178h
{0xFF, 3,{0xff,0x98,0x16}},
{0xBA, 1,{0x60}}, 
{0xB0, 1,{0x01}}, 
//{0xB6, 1,{0x22}}, 
{0xBC,18,{0x03,0x0D,0x03,0x63,0x01,0x01,0x1b,0x11,0x6E,0x00,0x00,0x00,0x01,0x01,0x16,0x00,0xff,0xf2}},
          
{0xBD, 8,{0x02,0x13,0x45,0x67,0x45,0x67,0x01,0x23}},

{0xBE,17,{0x03,0x22,0x22,0x22,0x22,0xdd,0xcc,0xbb,0xaa,0x66,0x77,0x22,0x22,0x22,0x22,0x22,0x22}},
 
{0xED, 2,{0x7f,0x0f}}, 

{0xF3,1,{0x70}}, 

{0xB4, 1,{0x02}}, 
 
{0xC0, 3,{0x0f,0x0b,0x0a}}, 
 
{0xC1, 4,{0x17,0x88,0x70,0x20}}, 
 
{0xD8, 1,{0x50}}, 
 
{0xFC, 1,{0x07}}, 
 
{0xE0,16,{0x00,0x03,0x0D,0x11,0x14,0x1F,0xCa,0x09,0x02,0x09,0x02,0x0B,0x0D,0x2B,0x27,0x00}},
          
{0xE1,16,{0x00,0x02,0x06,0x0a,0x0F,0x14,0x7A,0x09,0x04,0x09,0x07,0x0b,0x0B,0x32,0x2D,0x00}},
 
{0xD5, 8,{0x0A,0x09,0x06,0x08,0xCB,0xA5,0x01,0x04}},
 
{0xF7, 1,{0x89}},
{0xC7, 1,{0x6f}}, 
//{0x35,1 ,{0x00}},   
{0x11, 0,{0}},  
{REGFLAG_DELAY, 120, {}}, 
{0x29, 0,{0}}, 
{REGFLAG_DELAY, 20, {}}, 
{REGFLAG_END_OF_TABLE, 0x00, {}}

#endif

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

		params->dsi.vertical_sync_active				= 4;// 4;2
		params->dsi.vertical_backporch					= 16;// 8;2
		params->dsi.vertical_frontporch					= 20;// 8;2
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;//6
		params->dsi.horizontal_backporch				= 20;//37
		params->dsi.horizontal_frontporch				= 20;//37
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

                params->dsi.pll_div1=0;//1;		// div1=0,1,2,3;div1_real=1,2,4,4
		params->dsi.pll_div2=2;//1;		// div2=0,1,2,3;div2_real=1,2,4,4
		//params->dsi.fbk_sel=1;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
		params->dsi.fbk_div =26;//38;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	

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
    SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(100);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{	
    unsigned int data_array[16];
	data_array[0] = 0x00101500;	// Display inversion On
	dsi_set_cmdq(&data_array, 1, 1);	
}


static void lcm_resume(void)
{
    //unsigned int data_array[16];
	//data_array[0] = 0x00111500;	// Display inversion On
	//dsi_set_cmdq(&data_array, 1, 1);
	lcm_init();
}



static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(&data_array, 7, 0);

}

static unsigned int lcm_compare_id(void)
{
	volatile unsigned int lcm_id_gpio = 0;
	volatile unsigned int lcd_id_ic = 0;
	volatile unsigned char buffer[4];
	unsigned int data_array[16];
	
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);

	data_array[0] = 0x00043700;
       dsi_set_cmdq(data_array, 1, 1);

	MDELAY(10); 
       memset(buffer,0x0,4);
	read_reg_v2(0xD3, buffer, 4);  //Read  2 bytes 5510 means NT35510 from IC Spec

	lcd_id_ic = buffer[1]<<8 | buffer[2];

#ifdef BUILD_LK
    printf("%s, lcm_compare_id LK  debug: ili9806c_yaris5 hd_id = 0x%08x\n", __func__, lcd_id_ic);
#else
    printk("%s, lcm_compare_id kernel  horse debug: ili9806c_yaris5 hd_id = 0x%08x\n", __func__, lcd_id_ic);
#endif

	if(LCD_ID_IC_ILI9806C_ID == lcd_id_ic)
		{
			return 1;
		}
	else
		{
    mt_set_gpio_mode(GPIO40_LCD_ID1,GPIO40_LCD_ID1_M_GPIO);  // gpio mode   high
    //mt_set_gpio_pull_enable(GPIO40_LCD_ID1,0);
    mt_set_gpio_dir(GPIO40_LCD_ID1,0);  //input
    lcm_id_gpio = mt_get_gpio_in(GPIO40_LCD_ID1);//should 
   
    mt_set_gpio_mode(GPIO39_LCD_ID0,GPIO40_LCD_ID1_M_GPIO);  // gpio mode   high
    //mt_set_gpio_pull_enable(GPIO39_LCD_ID0,0);
    mt_set_gpio_dir(GPIO39_LCD_ID0,0);  //input
   lcm_id_gpio = lcm_id_gpio<<1 |(mt_get_gpio_in(GPIO39_LCD_ID0));
   
#ifdef BUILD_LK
    printf("%s, lcm_compare_id LK  debug: ili9806c_yaris5 hd_id = 0x%08x\n", __func__, lcm_id_gpio);
#else
    printk("%s, lcm_compare_id kernel  horse debug: ili9806c_yaris5 hd_id = 0x%08x\n", __func__, lcm_id_gpio);
#endif

    if (LCM_ID_GPIO_ILI9806C_BYD == lcm_id_gpio) //1=>hx8379a_tdt hw lcd;0=>ili9806c_byd hw lcd
    {
        return 1;
    }
    else
    {
        return 0;
    }
		}

    

#if 0
       volatile unsigned char lcd_id = 4;
	volatile unsigned int id = 0;
	volatile unsigned char buffer[4];
	unsigned int data_array[16];
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);
	
	data_array[0] = 0x00043700;
       dsi_set_cmdq(data_array, 1, 1);

	//data_array[0] = 0x00023700;// read id return two byte,version and id
	//dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10); 
       memset(buffer,0x0,4);
	read_reg_v2(0xD3, buffer, 4);  //Read  2 bytes 5510 means NT35510 from IC Spec

	id = buffer[1]<<8 | buffer[2];

    return (LCM9806_ID == id)?1:0;

#endif
}


LCM_DRIVER ili9806c_yaris_xl_byd_drv = 
{
    .name			= "ili9806c_yaris_xl_byd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
	#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
    #endif
};


