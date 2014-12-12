#ifdef BUILD_LK
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#endif
#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (800)

#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_ID       (0x55)
#define LCM_ID1       (0xC1)
#define LCM_ID2       (0x80)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
    
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
            
                if (cmd != 0xFF && cmd != 0x2C && cmd != 0x3C) {
                    //#if defined(BUILD_UBOOT)
                    //	printf("[DISP] - uboot - REG_R(0x%x) = 0x%x. \n", cmd, table[i].para_list[0]);
                    //#endif
                    while(read_reg(cmd) != table[i].para_list[0]);		
                }
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
    params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
    params->dsi.mode   = CMD_MODE;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
    
    params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=800;
    
    // Bit rate calculation
    params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=0;		// div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_sel=0;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
    params->dsi.fbk_div =17;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		
}


static void init_lcm_registers(void)
{
#if 0
    unsigned int data_array[16];

    //*************Enable CMD2 Page1  *******************//
    data_array[0]=0x00063902;
    data_array[1]=0x52aa55f0;
    data_array[2]=0x00000108;
    dsi_set_cmdq(data_array, 3, 1);
    
    //************* AVDD: manual  *******************//
    data_array[0]=0x00043902;
    data_array[1]=0x343434b9;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x343434ba;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x007800bc;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x007800bd;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00033902;
    data_array[1]=0x00ac00be;
    dsi_set_cmdq(data_array, 2, 1);
    
    // ********************  EABLE CMD2 PAGE 0 **************//
    data_array[0]=0x00063902;
    data_array[1]=0x52aa55f0;
    data_array[2]=0x00000008;
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]=0x00033902;
    data_array[1]=0x00000cb1;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x000001bc;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0]= 0x00053902;
    data_array[1]= 0x0100002a;
    data_array[2]= 0x000000df;
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]= 0x00053902;
    data_array[1]= 0x0300002b;
    data_array[2]= 0x00000055;
    dsi_set_cmdq(data_array, 3, 1);
 	//xuecheng, enable te
    //data_array[0]=0x00350500;
    data_array[0]=0x00351500;
    dsi_set_cmdq(data_array, 1, 1);
    
   
    data_array[0]= 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
#else
	unsigned int data_array[16];

	data_array[0]=0x00053902;
	data_array[1]=0x2555AAFF;
	data_array[2]=0x00000001;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]=0x00123902;
	data_array[1]=0x000201F8;
	data_array[2]=0x00133320;
	data_array[3]=0x23000040;
	data_array[4]=0x00C89902;
	data_array[5]=0x00001100;
	dsi_set_cmdq(data_array, 6, 1);


//*************Enable CMD2 Page1  *******************//
	data_array[0]=0x00063902;
	data_array[1]=0x52AA55F0;
	data_array[2]=0x00000108;
	dsi_set_cmdq(data_array, 3, 1);

//************* AVDD: manual  *******************//
	data_array[0]=0x00043902;
	data_array[1]=0x343434B6;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00043902;
	data_array[1]=0x090909B0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00043902;//AVEE: manual, -6V 
	data_array[1]=0x242424B7;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00043902;//AVEE voltage, Set AVEE -6V
	data_array[1]=0x090909B1;
	dsi_set_cmdq(data_array, 2, 1);

	//Power Control for VCL
	data_array[0]=0x34B81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0]=0x00B21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0]=0x00043902;//VGH: Clamp Enable
	data_array[1]=0x242424B9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00043902;
	data_array[1]=0x050505B3;
	dsi_set_cmdq(data_array, 2, 1);

	//data_array[0]=0x01BF1500;
	//dsi_set_cmdq(data_array, 1, 1);

	data_array[0]=0x00043902;//VGL(LVGL)
	data_array[1]=0x242424BA;
	dsi_set_cmdq(data_array, 2, 1);

	//VGL_REG(VGLO)
	data_array[0]=0x00043902;
	data_array[1]=0x0B0B0BB5;
	dsi_set_cmdq(data_array, 2, 1);

	//VGMP/VGSP
	data_array[0]=0x00043902;
	data_array[1]=0x00A300BC;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00043902;//VGMN/VGSN  
	data_array[1]=0x00A300BD;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00033902;//VCOM=-0.1
	data_array[1]=0x005000BE;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00353902;
	data_array[1]=0x003700D1;
	data_array[2]=0x007B0052;
	data_array[3]=0x00B10099;
	data_array[4]=0x01F600D2;
	data_array[5]=0x014E0127;
	data_array[6]=0x02BE018C;
	data_array[7]=0x0248020B;
	data_array[8]=0x027E024A;
	data_array[9]=0x03E102BC;
	data_array[10]=0x03310310;
	data_array[11]=0x0373035A;
	data_array[12]=0x039F0394;
	data_array[13]=0x03B903B3;
	data_array[14]=0x000000C1;
	dsi_set_cmdq(data_array, 15, 1);

	data_array[0]=0x00353902;
	data_array[1]=0x003700D2;
	data_array[2]=0x007B0052;
	data_array[3]=0x00B10099;
	data_array[4]=0x01F600D2;
	data_array[5]=0x014E0127;
	data_array[6]=0x02BE018C;
	data_array[7]=0x0248020B;
	data_array[8]=0x027E024A;
	data_array[9]=0x03E102BC;
	data_array[10]=0x03310310;
	data_array[11]=0x0373035A;
	data_array[12]=0x039F0394;
	data_array[13]=0x03B903B3;
	data_array[14]=0x000000C1;
	dsi_set_cmdq(data_array, 15, 1);
	
	data_array[0]=0x00353902;
	data_array[1]=0x003700D3;
	data_array[2]=0x007B0052;
	data_array[3]=0x00B10099;
	data_array[4]=0x01F600D2;
	data_array[5]=0x014E0127;
	data_array[6]=0x02BE018C;
	data_array[7]=0x0248020B;
	data_array[8]=0x027E024A;
	data_array[9]=0x03E102BC;
	data_array[10]=0x03310310;
	data_array[11]=0x0373035A;
	data_array[12]=0x039F0394;
	data_array[13]=0x03B903B3;
	data_array[14]=0x000000C1;
	dsi_set_cmdq(data_array, 15, 1);

	data_array[0]=0x00353902;
	data_array[1]=0x003700D4;
	data_array[2]=0x007B0052;
	data_array[3]=0x00B10099;
	data_array[4]=0x01F600D2;
	data_array[5]=0x014E0127;
	data_array[6]=0x02BE018C;
	data_array[7]=0x0248020B;
	data_array[8]=0x027E024A;
	data_array[9]=0x03E102BC;
	data_array[10]=0x03310310;
	data_array[11]=0x0373035A;
	data_array[12]=0x039F0394;
	data_array[13]=0x03B903B3;
	data_array[14]=0x000000C1;
	dsi_set_cmdq(data_array, 15, 1);

	data_array[0]=0x00353902;
	data_array[1]=0x003700D5;
	data_array[2]=0x007B0052;
	data_array[3]=0x00B10099;
	data_array[4]=0x01F600D2;
	data_array[5]=0x014E0127;
	data_array[6]=0x02BE018C;
	data_array[7]=0x0248020B;
	data_array[8]=0x027E024A;
	data_array[9]=0x03E102BC;
	data_array[10]=0x03310310;
	data_array[11]=0x0373035A;
	data_array[12]=0x039F0394;
	data_array[13]=0x03B903B3;
	data_array[14]=0x000000C1;
	dsi_set_cmdq(data_array, 15, 1);

	data_array[0]=0x00353902;
	data_array[1]=0x003700D6;
	data_array[2]=0x007B0052;
	data_array[3]=0x00B10099;
	data_array[4]=0x01F600D2;
	data_array[5]=0x014E0127;
	data_array[6]=0x02BE018C;
	data_array[7]=0x0248020B;
	data_array[8]=0x027E024A;
	data_array[9]=0x03E102BC;
	data_array[10]=0x03310310;
	data_array[11]=0x0373035A;
	data_array[12]=0x039F0394;
	data_array[13]=0x03B903B3;
	data_array[14]=0x000000C1;
	dsi_set_cmdq(data_array, 15, 1);
	
// ********************  EABLE CMD2 PAGE 0 **************//

	data_array[0]=0x00063902;
	data_array[1]=0x52AA55F0;
	data_array[2]=0x00000008;
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]=0x00063902;//I/F Setting
	data_array[1]=0x020500B0;
	data_array[2]=0x00000205;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]=0x0AB61500;//SDT
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0]=0x00033902;//Set Gate EQ 
	data_array[1]=0x000000B7;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00053902;//Set Source EQ
	data_array[1]=0x050501B8;
	data_array[2]=0x00000005;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]=0x00043902;//Inversion: Column inversion (NVT)
	data_array[1]=0x020202BC;//0x000000BC
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00043902;//BOE's Setting (default)
	data_array[1]=0x000003CC;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0]=0x00063902;//Display Timing
	data_array[1]=0x078401BD;
	data_array[2]=0x00000031;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]=0x01BA1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0]=0x00053902;
	data_array[1]=0x2555AAF0;
	data_array[2]=0x00000001;
	dsi_set_cmdq(data_array, 3, 1);

/*
	data_array[0]=0x00053902;//Enable Test mode
	data_array[1]=0x2555AAFF;
	data_array[2]=0x00000001;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10);	
*/

	data_array[0]=0x773A1500;//TE ON 
	dsi_set_cmdq(data_array, 1, 1);

//	data_array[0] = 0x00351500;// TE ON
//	dsi_set_cmdq(data_array, 1, 1);
//	MDELAY(10);
/*
	data_array[0] = 0x00110500;		// Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	
	data_array[0] = 0x00290500;		// Display On
	dsi_set_cmdq(data_array, 1, 1);
*/	
//******************* ENABLE PAGE0 **************//
	data_array[0]=0x00063902;
	data_array[1]=0x52AA55F0;
	data_array[2]=0x00000008;
	dsi_set_cmdq(data_array, 3, 1);
	
/*	data_array[0]=0x02C71500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);	
	
	data_array[0]=0x00053902;
	data_array[1]=0x000011C9;
	data_array[2]=0x00000000;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10);
	
	data_array[0]=0x00211500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);	
*/
	data_array[0] = 0x00351500;// TE ON
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x00033902;
	data_array[1]= 0x0000E8B1;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00023902;
	data_array[1]= 0x0051;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00023902;
	data_array[1]= 0x2453;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00023902;
	data_array[1]= 0x0155;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00023902;
	data_array[1]= 0x705e;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00033902;
	data_array[1]= 0x000301E0;
	dsi_set_cmdq(data_array, 2, 1);
#endif
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(20);

    init_lcm_registers();
}


static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
    lcm_init();
	
    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
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
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]= 0x00290508; //HW bug, so need send one HS packet
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}


static void lcm_setbacklight(unsigned int level)
{
    unsigned int data_array[16];


#if defined(BUILD_LK)
    printf("%s, %d\n", __func__, level);
#elif defined(BUILD_UBOOT)
    printf("%s, %d\n", __func__, level);
#else
    printk("lcm_setbacklight = %d\n", level);
#endif
  
    if(level > 255) 
        level = 255;
    
    data_array[0]= 0x00023902;
    data_array[1] =(0x51|(level<<8));
    dsi_set_cmdq(data_array, 2, 1);
}


static void lcm_setpwm(unsigned int divider)
{
    // TBD
}


static unsigned int lcm_getpwm(unsigned int divider)
{
    // ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
    // pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
    unsigned int pwm_clk = 23706 / (1<<divider);	


    return pwm_clk;
}


static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0, id2 = 0;
    unsigned char buffer[2];
    unsigned int data_array[16];
    

    SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10);	
    
    /*	
    data_array[0] = 0x00110500;		// Sleep Out
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    */
    
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
    id = buffer[0]; //we only need ID
    id2= buffer[1]; //we test buffer 1
    #ifdef BUILD_LK
	printf("lcm_compare_id, id=0x%08x\n", id);
    #else
	printk("lcm_compare_id, id=0x%08x\n", id);
    #endif
    
    return (LCM_ID == id)?1:0;
}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER nt35510_dsi_cmd_6572_drv = 
{
    .name			= "nt35510_dsi_cmd_6572",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .set_backlight	= lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
    .compare_id    = lcm_compare_id,
    .update         = lcm_update
};

