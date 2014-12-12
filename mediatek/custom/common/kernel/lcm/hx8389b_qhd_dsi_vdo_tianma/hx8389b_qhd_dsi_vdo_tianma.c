#ifndef BUILD_LK
#include <linux/string.h>
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

#define FRAME_WIDTH  (540)
#define FRAME_HEIGHT (960)

#define LCM_ID_HX8389B 0x89


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0


static  LCM_setting_table_V3 lcm_sleep_out_setting[] = {

	// Sleep Out
	{0x05,0x11, 0, {0}},
    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {0}},

    // Display ON
	{0x05,0x29, 0, {0}}
	//{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static  LCM_setting_table_V3 lcm_sleep_in_setting[] = {

	// Display off sequence
	{0x05,0x28, 0, {0}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {0}},

    // Sleep Mode On
	{0x05,0x10, 0, {0}},

	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {0}},
	{0x15,0x4F, 1, {0x01}}
	//{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static LCM_setting_table_V3 lcm_initialization_setting[] = {
	
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

	 {0x39,0xB9,3,{0xFF,0x83,0x89}},
	  	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 1, {}},

	{0x39,0xBA,7,{0x41,0x93,0x00,0x16,0xA4,0x10,0x18}},
		{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 1, {}},

	{0x15,0xC6,1,{0xE8}},
//------------ HX5186 set power-------------------------------//

	{0x39,0xB1,19,{0x00,0x00,0x04,0xE8,0x99,0x10,0x11,0xD1,0xf1,0x36,
	               0x3e,0x2A,0x2A,0x43,0x01,0x5a,0xF2,0x20,0x80}},
		{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},

//------------------------------------------------------
 	{0x39,0xDE,3,{0x05,0x58,0x12}},

 	{0x39,0xB2,7,{0x00,0x00,0x78,0x0E,0x05,0x3F,0x80}},
		{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 1, {}},

 	{0x39,0xB4,23,{0x80,0x08,0x00,0x32,0x10,0x07,0x32,0x10,0x02,0x32,
 	               0x10,0x00,0x37,0x05,0x40,0x0B,0x37,0x05,0x48,0x14,
 	               0x50,0x53,0x0a}},		
 	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}}, 	
 	{0x39,0xD5,48,{0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x01,0x60,0x00,
 	               0x99,0x88,0x88,0x88,0x88,0x23,0x88,0x01,0x88,0x67,
 	               0x88,0x45,0x01,0x23,0x23,0x45,0x88,0x88,0x88,0x88,
 	               0x99,0x88,0x88,0x88,0x54,0x88,0x76,0x88,0x10,0x88,
 	               0x32,0x32,0x10,0x88,0x54,0x88,0x88,0x88}},		
 	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},	     
 	{0x39,0xC1,32,{0x01,0x00,0x08,0x10,0x18,0x21,0x28,0x30,0x38,0x41,
 	               0x49,0x51,0x59,0x61,0x68,0x70,0x78,0x81,0x89,0x90,
 	               0x98,0xA0,0xA8,0xB0,0xB8,0xC1,0xC9,0xD1,0xD7,0xE2,
 	               0xEA,0xF2}}, 	              	     
 	{0x29,0xc1,32,{0xF8,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
 	               0x00,0x00,0x08,0x10,0x18,0x20,0x28,0x30,0x38,0x40,
 	               0x48,0x50,0x58,0x60,0x68,0x70,0x78,0x80,0x88,0x90,
 	               0x98,0xA0}},

 	             
 	{0x29,0xc1,32,{0xA8,0xB0,0xB8,0xC0,0xC8,0xD0,0xD8,0xE0,0xE8,0xF0,
 	               0xF8,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
 	               0x00,0x00,0x08,0x10,0x18,0x22,0x2A,0x32,0x3B,0x43,
 	               0x4B,0x54}}, 	             
 	{0x29,0xc1,31,{0x5C,0x64,0x6C,0x74,0x7D,0x85,0x8E,0x96,0x9E,0xA6,
 	               0xAE,0xB6,0xBE,0xC6,0xCE,0xD6,0xDE,0xE5,0xED,0xF5,
 	               0xF8,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
 	               0x00}},
 	{0x39,0xE0,34,{0x16,0x2C,0x32,0x30,0x35,0x3F,0x3D,0x52,0x08,0x0E,
 	               0x0F,0x13,0x15,0x13,0x14,0x19,0x1C,0x16,0x2C,0x32,
 	               0x30,0x35,0x3F,0x3D,0x52,0x07,0x0D,0x0F,0x13,0x15,
 	               0x13,0x14,0x19,0x1C}},

		 {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 5, {}},
 
 	{0x39,0xB6,4,{0x00,0x88,0x00,0x88}},
		 {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 1, {}},

	   
 	{0x15,0xCC,1,{0x02}},
	
	   
 	{0x39,0xB7,3,{0x00,0x00,0x50}},


	{0x15,0x51,1,{0xFF}},
	{0x15,0x53,1,{0x2C}},
	{0x15,0x55,1,{0x02}},
	

	{0x05,0x11,0,{}},		
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},
	{0x05,0x29,0,{}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},



};



#if 0
static void init_lcm_registers(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00043902;
		data_array[1] = 0x8983ffb9; 				
		dsi_set_cmdq(data_array, 2, 1);

		data_array[0] = 0x00083902;
		data_array[1] = 0x009341ba; 
		data_array[2] = 0x1810a416;
		dsi_set_cmdq(data_array, 3, 1);

		
		data_array[0] = 0xe8c61500;				
		dsi_set_cmdq(data_array, 1, 1);



		data_array[0]=0x00143902;
		data_array[1]=0x040000B1;
		data_array[2]=0x111099e8;
		data_array[3]=0x3e36f1D1;
		data_array[4]=0x01432A2A;
		data_array[5]=0x8020f25a;
		dsi_set_cmdq(data_array, 6, 1);

		data_array[0] = 0x00043902;
		data_array[1] = 0x125805DE;
		dsi_set_cmdq(data_array, 2, 1);
		
		data_array[0] = 0x00083902;
		data_array[1] = 0x780000B2;
		data_array[2] = 0x803F050E;
		dsi_set_cmdq(data_array, 3, 1);


		data_array[0] =0x00183902;
		data_array[1] =0x000880B4;
		data_array[2] =0x32071032;
		data_array[3] =0x10320210;
		data_array[4] =0x40053700; 
		data_array[5] =0x4805370B;
		data_array[6] =0x0a535014;
		dsi_set_cmdq(data_array, 7, 1);



		data_array[0] =0x00313902;
		data_array[1] =0x000000D5;
		data_array[2] =0x00000100;
		data_array[3] =0x99006001;
		data_array[4] =0x88888888;
		data_array[5] =0x88018823;
		data_array[6] =0x01458867;
		data_array[7] =0x88452323;
		data_array[8] =0x99888888;
		data_array[9] =0x54888888;
		data_array[10] =0x10887688;
		data_array[11] =0x10323288;
		data_array[12] =0x88885488;
		data_array[13] =0x88;
		dsi_set_cmdq(data_array, 14, 1);

		data_array[0] =0x00233902;
		data_array[1] =0x181405E0;
		data_array[2] =0x203F3434;
		data_array[3] =0x0e0E083C;
		data_array[4] =0x12101311;
		data_array[5] =0x14051C1A;
		data_array[6] =0x3F342D18;
		data_array[7] =0x0E083C20;
		data_array[8] =0x1013110e;
		data_array[9] =0x1C1A12;
		dsi_set_cmdq(data_array,9,1);

		data_array[0] =0x00053902;
		data_array[1] =0x008500B6;
		data_array[2] =0x85;
		dsi_set_cmdq(data_array, 3, 1);

		data_array[0] =0x02cc1500;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] =0x00043902;
		data_array[1] =0x500000B7;
		dsi_set_cmdq(data_array, 2, 1);


		data_array[0] =0xff511500;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] =0x24531500;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] =0x02551500;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] =0x00110500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(150);

		data_array[0] =0x00290500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(10);

}
#endif
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

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active				= 0x05;// 3    2
		params->dsi.vertical_backporch					= 14;// 20   1
		params->dsi.vertical_frontporch					= 12; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 0x16;// 50  2
		params->dsi.horizontal_backporch				= 0x38;
		params->dsi.horizontal_frontporch				= 0x18;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8; 

		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_select=1;
		//params->dsi.PLL_CLOCK  = LCM_DSI_6589_PLL_CLOCK_377;
		params->dsi.PLL_CLOCK=250;
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
#if (LCM_DSI_CMD_MODE)
		params->dsi.fbk_div =9;
#else
		params->dsi.fbk_div =9;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#endif
		//params->dsi.compatibility_for_nvk = 1;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
}

static void lcm_init(void)
{

		
		SET_RESET_PIN(0);
		MDELAY(20); 
		SET_RESET_PIN(1);
		MDELAY(20); 
		dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);
		
    
}



static void lcm_suspend(void)
{
	unsigned int data_array[16];

	//data_array[0]=0x00280500; // Display Off
	//dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120); 

	
	SET_RESET_PIN(1);	
	SET_RESET_PIN(0);
	MDELAY(20); // 1ms
	
	SET_RESET_PIN(1);
	MDELAY(120);      
}


static void lcm_resume(void)
{
	lcm_init();

    #ifdef BUILD_LK
	  printf("[LK]------hx8389b----%s------\n",__func__);
    #else
	  printk("[KERNEL]------hx8389b----%s------\n",__func__);
    #endif	
}
         
#if (LCM_DSI_CMD_MODE)
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
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];  

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(10);//Must over 6 ms

	array[0]=0x00043902;
	array[1]=0x8983FFB9;// page enable
	dsi_set_cmdq(&array, 2, 1);
	MDELAY(10);

	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(&array, 1, 1);
	MDELAY(10);

	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; 
	
#if defined(BUILD_UBOOT)
	printf("%s, id = 0x%08x\n", __func__, id);
#endif

	return (LCM_ID_HX8389B == id)?1:0;

}



static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x36, buffer, 1);
	if(buffer[0]==0x90)
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	lcm_resume();

	return TRUE;
}




LCM_DRIVER hx8389b_qhd_dsi_vdo_tianma_lcm_drv = 
{
    .name			= "hx8389b_qhd_dsi_vdo_tianma",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.compare_id     = lcm_compare_id,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
