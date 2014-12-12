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
#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define SET_RESET_PIN(v)    									(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)	        							(lcm_util.set_gpio_out((n), (v)))
#define UDELAY(n) 												(lcm_util.udelay(n))
#define MDELAY(n) 												(lcm_util.mdelay(n))

#define LCM_ID                                                                                 (0x01)
#define LCM_ID_GPIO_HX8379A_TDT                                                (0x02)
#define LCD_ID_IC_HX8379A_TDT                                                     (0x79)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)			lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)											lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)						lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static LCM_UTIL_FUNCS   										lcm_util = {0};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
struct LCM_setting_table {
unsigned char cmd;
unsigned char count;
unsigned char para_list[64];
};

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

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

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->dsi.mode   					= SYNC_PULSE_VDO_MODE;
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
	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=720*3;

	params->dsi.vertical_sync_active	=5;//
	params->dsi.vertical_backporch		= 6;//
	params->dsi.vertical_frontporch		= 5;//
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 36;//
	params->dsi.horizontal_backporch	= 36;//
	params->dsi.horizontal_frontporch	= 36;//
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;
        //params->dsi.HS_PRPR = 4;
    //    params->dsi.CLK_HS_PRPR = 6;
	// Bit rate calculation
	
	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=2;		// div2=0,1,2,3;div1_real=1,2,4,4
	params->dsi.fbk_div =27;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
	
}

static struct LCM_setting_table lcm_initialization_ret[] = {
	
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


	//must use 0x39 for init setting for all register.

	{0XB9, 3, {0XFF,0X83,0X79}},
	{REGFLAG_DELAY, 10, {}},

	{0X11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

    //SET POWER
    {0XB1, 31, {0X00,0X50,0X44,
    0XEA,0X90,0X08,0X11,
    0X11,0X71,0X2F,0X37,
    0X9A,0X1A,0X42,0X0B,
    0X6E,0XF1,0X00,0XE6,
    0XE6,0XE6,0XE6,0XE6,
    0X00,0X04,0X05,0X0A,
    0X0B,0X04,0X05,0X6F}}, 
  
    //SET DISPLAY
    {0XB2, 13, {0X00,0X00,0XFE,
    0X08,0X0C,0X19,0X22,
    0X00,0XFF,0X08,0X0C,
    0X19,0X20}}, 

    //SET CYC
    {0XB4, 31, {0X80,0X0C,0X00,
    0X32,0X10,0X01,0X43,
    0X13,0X65,0X00,0X00,
    0X00,0X37,0X01,0X42,
    0X03,0X37,0X01,0X42,
    0X08,0X30,0X30,0X04,
    0X00,0X40,0X08,0X28,
    0X08,0X30,0X30,0X04}}, 
			
    //SET GIP		
    {0XD5, 46, {0X00,0X00,0X0A,
    0X00,0X01,0X00,0X00,
    0X48,0X04,0X88,0X88,
    0X01,0X01,0X23,0X45,
    0X67,0X88,0XAA,0XBB,
    0X88,0X45,0X88,0X88,
    0X67,0X88,0X88,0X88,
    0X54,0X10,0X76,0X54,
    0X32,0X32,0XAA,0XBB,
    0X88,0X10,0X88,0X88,
    0X76,0X88,0X39,0X01,
    0X00,0X00,0X00,0X00}}, 

    //SET GAMMA 2.2
    {0XE0, 35, {0X79,0X07,0X12,
    0X14,0X3F,0X3F,0X3F,
    0X25,0X4F,0X06,0X0C,
    0X0E,0X12,0X17,0X12,
    0X13,0X14,0X1F,0X07,
    0X12,0X14,0X3F,0X3F,
    0X3F,0X25,0X4F,0X06,
    0X0C,0X0E,0X12,0X17,
    0X12,0X13,0X14,0X1F}},     
	
    //SET PANEL
	{0XCC, 1,   {0X02}},

    //DISPLAY ON
	{0X29, 1,   {0X00}},
	{REGFLAG_DELAY, 10, {}},

	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_init(void)
{
	unsigned int data_array[16]; 

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);	
	MDELAY(120);
        //set password
	data_array[0]=0x00043902;
	data_array[1]=0x7983FFB9;
	dsi_set_cmdq(&data_array,2,1);

    data_array[0]=0x51BA1502; //zhfan
    data_array[1]=0x00000093; //zhfan    
	dsi_set_cmdq(&data_array,2,1);
    
        //SET POWER
	data_array[0]=0x00203902;
	data_array[1]=0x245000B1;
	data_array[2]=0x110899EA;
	data_array[3]=0x221A7111;
	data_array[4]=0x0B221A9A;
	data_array[5]=0xE600F176;
	data_array[6]=0xE6E6E6E6;
	data_array[7]=0x0A050400;
	data_array[8]=0x6F05040B;
	dsi_set_cmdq(&data_array,9,1);
    
    //SET DISPLAY
	data_array[0]=0x000E3902;
	data_array[1]=0xFE0000B2;
	data_array[2]=0x22190C08;
	data_array[3]=0x0C08FF00;
	data_array[4]=0x00002019;
	dsi_set_cmdq(&data_array,5,1);

    //SET CYC
	data_array[0]=0x00203902;
	data_array[1]=0x000682B4;
	data_array[2]=0x32031032;
	data_array[3]=0x10325F13;
	data_array[4]=0x28013508;
	data_array[5]=0x3C003707;
	data_array[6]=0x083E3E0A;
	data_array[7]=0x28084000;
	data_array[8]=0x04303008;
	dsi_set_cmdq(&data_array,9,1);

//SET GIP		

	data_array[0]=0x00303902;
	data_array[1]=0x0A0000D5;
	data_array[2]=0x00050100;
	data_array[3]=0x99881800;
	data_array[4]=0x88450188;
	data_array[5]=0x23450188;
	data_array[6]=0x88888867;
	data_array[7]=0x88888888;
	data_array[8]=0x88105499;
	data_array[9]=0x54327688;
	data_array[10]=0x88888810;
	data_array[11]=0x00008888;
	data_array[12]=0x00000000;
	dsi_set_cmdq(&data_array,13,1);

	data_array[0]=0x00043902;
	data_array[1]=0x047005DE;
	dsi_set_cmdq(&data_array,2,1);

//SET GAMMA 2.2 0806
	data_array[0]=0x00243902;
	data_array[1]=0x100879E0;
	data_array[2]=0x3E1E1313;
	data_array[3]=0x0A023629;
	data_array[4]=0x14161410;
	data_array[5]=0x08181315;
	data_array[6]=0x1E131310;
	data_array[7]=0x0236293E;
	data_array[8]=0x1614100A;
	data_array[9]=0x18131515;
	dsi_set_cmdq(&data_array,10,1);
    
//SET GAMMA 2.2
	/*data_array[0]=0x00243902;
	data_array[1]=0x131279E0;
	data_array[2]=0x3F161514;
	data_array[3]=0x12063625;
	data_array[4]=0x14161410;
	data_array[5]=0x12161115;
	data_array[6]=0x16151413;
	data_array[7]=0x0636253F;
	data_array[8]=0x16141012;
	data_array[9]=0x16111514;
	dsi_set_cmdq(&data_array,10,1);*/

//SET GAMMA 2.5
	/*data_array[0]=0x00243902;
	data_array[1]=0x131279E0;
	data_array[2]=0x3F1E1D14;
	data_array[3]=0x0A023E25;
	data_array[4]=0x16181511;
	data_array[5]=0x12161315;
	data_array[6]=0x1E1D1413;
	data_array[7]=0x023E253F;
	data_array[8]=0x1815110A;
	data_array[9]=0x16131516;
	dsi_set_cmdq(&data_array,10,1);*/

        //SET PANEL
	data_array[0]=0x02CC1502;
	dsi_set_cmdq(&data_array,1,1);

	data_array[0]=0x00350500;
	dsi_set_cmdq(data_array, 1, 1);

    //SLEEP OUT
	data_array[0]=0x00110500;
	dsi_set_cmdq(&data_array,1,1);
	//MDELAY(520);
	MDELAY(120);
	
	data_array[0]=0x00053902;
	data_array[1]=0x007700B6;
	data_array[2]=0x00000077;
	dsi_set_cmdq(&data_array,3,1);

	data_array[0]=0x00290500;
	dsi_set_cmdq(&data_array,1,1);
	MDELAY(50);

}

static void lcm_suspend(void)
{

      SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);	
	MDELAY(120);
	
/*

unsigned int data_array[16];

	data_array[0]=0x00280500; 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x00100500; 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	SET_RESET_PIN(0);
*/
}

static void lcm_resume(void)
{
	lcm_init();
}

static int dummy_delay = 0;
static unsigned int lcm_esd_check(void)
{  
    //#ifndef BUILD_LK
    unsigned int  data_array[16];
    unsigned char buffer_09 = 0;
    unsigned char buffer_0a = 0;
    unsigned char buffer_0b = 0;
    unsigned char buffer_0d = 0;
    unsigned char buffer_0c = 0;
    unsigned char buffer_b6[4] = {0};
    
    unsigned int retval = 0;
    
    dummy_delay ++;

    if (dummy_delay >=10000)
        dummy_delay = 0;
    
    if(dummy_delay %2 == 0)
    {    
     	data_array[0]=0x00033902;
	    data_array[1]=0x009351BA;
	    dsi_set_cmdq(&data_array,2,1);
	    MDELAY(5);

	    data_array[0] = 0x00013700;
	    dsi_set_cmdq(data_array, 1, 1);
	    read_reg_v2(0x09,&buffer_09, 1);

	    data_array[0] = 0x00013700;
	    dsi_set_cmdq(data_array, 1, 1);
	    read_reg_v2(0x0a,&buffer_0a, 1);

	    data_array[0] = 0x00013700;
	    dsi_set_cmdq(data_array, 1, 1);
	    read_reg_v2(0x0b,&buffer_0b, 1);

	    /*data_array[0] = 0x00013700;
	    dsi_set_cmdq(data_array, 1, 1);
	    read_reg_v2(0x0c,&buffer_0c, 1);*/
        
	    data_array[0] = 0x00013700;
	    dsi_set_cmdq(data_array, 1, 1);
	    read_reg_v2(0x0d,&buffer_0d, 1);

	    data_array[0] = 0x00023700;
	    dsi_set_cmdq(data_array, 1, 1);
	    read_reg_v2(0xb6,&buffer_b6, 2);
        
            #ifdef BUILD_LK
	    printf("build lk lcm_esd_check lcm -----------------\n");
	    printf("lcm_esd_check lcm 0x09 is %x-----------------\n", buffer_09);
            printf("lcm_esd_check lcm 0x0A is %x-----------------\n", buffer_0a);
	    printf("lcm_esd_check lcm 0x0B is %x-----------------\n", buffer_0b);
	    //printf("lcm_esd_check lcm 0x0C is %x-----------------\n", buffer_0c);
	    printf("lcm_esd_check lcm 0x0D is %x-----------------\n", buffer_0d);
	    printf("lcm_esd_check lcm 0xb6[0] is %x-----------------\n", buffer_b6[0]);
	    printf("lcm_esd_check lcm 0xb6[1] is %x-----------------\n", buffer_b6[1]);
	    printf("lcm_esd_check lcm 0xb6[2] is %x-----------------\n", buffer_b6[2]);
	    printf("lcm_esd_check lcm 0xb6[3] is %x-----------------\n", buffer_b6[3]);
            #else
	    printk("not build lk lcm_esd_check lcm -----------------\n");
	    printk("lcm_esd_check lcm 0x09 is %x-----------------\n", buffer_09);
            printk("lcm_esd_check lcm 0x0A is %x-----------------\n", buffer_0a);
	    printk("lcm_esd_check lcm 0x0B is %x-----------------\n", buffer_0b);
	    //printk("lcm_esd_check lcm 0x0C is %x-----------------\n", buffer_0c);
	    printk("lcm_esd_check lcm 0x0D is %x-----------------\n", buffer_0d);
	    printk("lcm_esd_check lcm 0xb6[0] is %x-----------------\n", buffer_b6[0]);
	    printk("lcm_esd_check lcm 0xb6[1] is %x-----------------\n", buffer_b6[1]);
	    printk("lcm_esd_check lcm 0xb6[2] is %x-----------------\n", buffer_b6[2]);
	    printk("lcm_esd_check lcm 0xb6[3] is %x-----------------\n", buffer_b6[3]);
            #endif
	
	    if ((buffer_09==0x80) && (buffer_0a==0x1C)&&(buffer_0b==0x00)/*&&(buffer_0c==0x70)*/&&(buffer_0d==0x00)
                /*&& (buffer_b6[0]==0x00) && (buffer_b6[1]==0x77)*/)
            {
		    //printk("diablox_lcd lcm_esd_check done\n");
		    retval = 0;
	    }else{
		    //printk("diablox_lcd lcm_esd_check return true\n");
		    retval = 1;
	    }
    }

	return retval;
    //#endif
}

static unsigned int lcm_esd_recover(void)
{
    //printk("%s \n",__FUNCTION__);
    
    //lcm_resume();
    lcm_init();

    return 1;
}


#if 1
static unsigned int lcm_compare_id(void)
{
	unsigned int lcm_id_gpio = 0;
	unsigned int data_array[16];
        unsigned char buff[3];
        unsigned char lcd_id_ic=0;

	SET_RESET_PIN(1);
        SET_RESET_PIN(0);
        MDELAY(10);  // 2
        SET_RESET_PIN(1);
        MDELAY(10);  //10

	data_array[0]=0x00043902;
	data_array[1]=0x7983FFB9;
	dsi_set_cmdq(&data_array,2,1);
	MDELAY(10);

 	data_array[0]=0x00033902;
	data_array[1]=0x009351BA;
	dsi_set_cmdq(&data_array,2,1);
	MDELAY(5);
    
	data_array[0] = 0x00023700;
	dsi_set_cmdq(&data_array, 1, 1);
    	MDELAY(10); 
        
	read_reg_v2(0xF4,buff, 2);
       lcd_id_ic = buff[0];
#ifdef BUILD_LK
    printf("%s, lcm_compare_id LK  debug: nt35596_ScribePro lcd_id = 0x%08x\n", __func__, lcd_id_ic);
#else
    printk("%s, lcm_compare_id kernel  horse debug: nt35596_ScribePro lcd_id = 0x%08x\n", __func__, lcd_id_ic);
#endif

	if((lcd_id_ic & 0xff) == LCD_ID_IC_HX8379A_TDT) 
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
   lcm_id_gpio =lcm_id_gpio<<1 |(mt_get_gpio_in(GPIO39_LCD_ID0));
   
#ifdef BUILD_LK
    printf("%s, lcm_compare_id LK  debug: hx8379a_yarisxl hd_id = 0x%08x\n", __func__, lcm_id_gpio);
#else
    printk("%s, lcm_compare_id kernel  horse debug: hx8379a_yarisxl hd_id = 0x%08x\n", __func__, lcm_id_gpio);
#endif

    if (LCM_ID_GPIO_HX8379A_TDT == lcm_id_gpio) //1=>hx8379a_tdt hw lcd;0=>ili9806c_byd hw lcd
    {
        return 1;
    }
    else
    {
        return 0;
    }
		}
#if 0
	unsigned int data_array[16];
	unsigned int id=0;
        unsigned char buff[3];
        unsigned char id_F4=0;

        SET_RESET_PIN(1);
        SET_RESET_PIN(0);
        MDELAY(10);  // 2
        SET_RESET_PIN(1);
        MDELAY(10);  //10

        
 	data_array[0]=0x00043902;
	data_array[1]=0x7983FFB9;
	dsi_set_cmdq(&data_array,2,1);
	MDELAY(10);

 	data_array[0]=0x00033902;
	data_array[1]=0x009351BA;
	dsi_set_cmdq(&data_array,2,1);
	MDELAY(5);
    
	data_array[0] = 0x00023700;
	dsi_set_cmdq(&data_array, 1, 1);
    	MDELAY(10); 
        
	read_reg_v2(0xF4,buff, 2);
        id_F4 = buff[0];
        
	/*mt_set_gpio_mode(142,0);  // gpio mode   high
	mt_set_gpio_pull_enable(142,0);
	mt_set_gpio_dir(142,0);  //input
	id = mt_get_gpio_in(142);//should be 0 */

#if defined(BUILD_LK)
        printf(" BUILD_LK m_compare_id id_F4 =  %x,buff = %x,%x.%x-----------------\n", id_F4,buff[0],buff[1],buff[2]);
        printf("BUILD_LK cm_compare_id id =  %x-----------------\n", id);
#elif defined(BUILD_UBOOT)
        printf(" BUILD_UBOOT m_compare_id id_F4 =  %x,buff = %x,%x.%x-----------------\n", id_F4,buff[0],buff[1],buff[2]);
        printf("BUILD_UBOOT cm_compare_id id =  %x-----------------\n", id);
#else
        printk(" lcm_compare_id id_F4 =  %x,buff = %x,%x.%x-----------------\n", id_F4,buff[0],buff[1],buff[2]);
        printk("lcm_compare_id id =  %x-----------------\n", id);
#endif
         
        if((id_F4 & 0xff) == 0x79) 
        {
            return 1;
        }
	else
	{
		return 0;
	}
#endif
}
#endif
//for new patch ,modify none
LCM_DRIVER hx8379a_yaris_xl_vdo_drv = 
{
    .name			= "hx8379a_yaris_xl",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
        .update         = lcm_update,
#endif
    //.esd_check		= lcm_esd_check,
    //.esd_recover	= lcm_esd_recover,
};
