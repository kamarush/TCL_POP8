#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/fb.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <linux/wait.h>
#include <mach/mt_typedefs.h>
#include "disp_hal.h"
#include "disp_drv_log.h"
#include "mtkfb.h"
#include "debug.h"
#include "lcm_drv.h"
#include "ddp_ovl.h"
#include "ddp_path.h"
#include "disp_drv.h"
#include "lcd_drv.h"
#include <linux/delay.h>

struct MTKFB_MMP_Events_t MTKFB_MMP_Events;

extern LCM_DRIVER *lcm_drv;
extern LCM_PARAMS *lcm_params;
extern unsigned int EnableVSyncLog;

#define MTKFB_DEBUG_FS_CAPTURE_LAYER_CONTENT_SUPPORT

// ---------------------------------------------------------------------------
//  External variable declarations
// ---------------------------------------------------------------------------

extern long tpd_last_down_time;
extern int  tpd_start_profiling;
extern void mtkfb_log_enable(int enable);
extern void disp_log_enable(int enable);
extern void mtkfb_vsync_log_enable(int enable);
extern void mtkfb_capture_fb_only(bool enable);
extern void esd_recovery_pause(BOOL en);
extern int mtkfb_set_backlight_mode(unsigned int mode);
extern void mtkfb_pan_disp_test(void);
extern void mtkfb_show_sem_cnt(void);
extern void mtkfb_hang_test(bool en);
extern void mtkfb_switch_normal_to_factory(void);
extern void mtkfb_switch_factory_to_normal(void);
extern void mtkfb_late_resume(struct early_suspend *h);
extern void mtkfb_early_suspend(struct early_suspend *h);

extern unsigned int gCaptureLayerEnable;
extern unsigned int gCaptureLayerDownX;
extern unsigned int gCaptureLayerDownY;

extern unsigned int gCaptureOvlThreadEnable;
extern unsigned int gCaptureOvlDownX;
extern unsigned int gCaptureOvlDownY;
extern struct task_struct *captureovl_task;

extern unsigned int gCaptureFBEnable;
extern unsigned int gCaptureFBDownX;
extern unsigned int gCaptureFBDownY;
extern unsigned int gCaptureFBPeriod;
extern struct task_struct *capturefb_task;
extern wait_queue_head_t gCaptureFBWQ;

#ifdef MTKFB_DEBUG_FS_CAPTURE_LAYER_CONTENT_SUPPORT
struct dentry *mtkfb_layer_dbgfs[DDP_OVL_LAYER_MUN];

extern OVL_CONFIG_STRUCT cached_layer_config[DDP_OVL_LAYER_MUN];

typedef struct {
    UINT32 layer_index;
    UINT32 working_buf;
    UINT32 working_size;
} MTKFB_LAYER_DBG_OPTIONS;

MTKFB_LAYER_DBG_OPTIONS mtkfb_layer_dbg_opt[DDP_OVL_LAYER_MUN];

#endif
extern LCM_DRIVER *lcm_drv;
// ---------------------------------------------------------------------------
//  Debug Options
// ---------------------------------------------------------------------------

static const long int DEFAULT_LOG_FPS_WND_SIZE = 30;

typedef struct {
    unsigned int en_fps_log;
    unsigned int en_touch_latency_log;
    unsigned int log_fps_wnd_size;
    unsigned int force_dis_layers;
} DBG_OPTIONS;

static DBG_OPTIONS dbg_opt = {0};

static char STR_HELP[] =
    "\n"
    "USAGE\n"
    "        echo [ACTION]... > /d/mtkfb\n"
    "\n"
    "ACTION\n"
	"        mtkfblog:[on|off]\n"
	"             enable/disable [MTKFB] log\n"
	"\n"
	"        displog:[on|off]\n"
	"             enable/disable [DISP] log\n"
	"\n"
	"        mtkfb_vsynclog:[on|off]\n"
	"             enable/disable [VSYNC] log\n"
	"\n"
	"        log:[on|off]\n"
	"             enable/disable above all log\n"
	"\n"
    "        fps:[on|off]\n"
    "             enable fps and lcd update time log\n"
	"\n"
    "        tl:[on|off]\n"
    "             enable touch latency log\n"
    "\n"
    "        layer\n"
    "             dump lcd layer information\n"
    "\n"
    "        suspend\n"
    "             enter suspend mode\n"
    "\n"
    "        resume\n"
    "             leave suspend mode\n"
    "\n"
    "        lcm:[on|off|init]\n"
    "             power on/off lcm\n"
    "\n"
    "        cabc:[ui|mov|still]\n"
    "             cabc mode, UI/Moving picture/Still picture\n"
    "\n"
    "        lcd:[on|off]\n"
    "             power on/off display engine\n"
    "\n"
    "        te:[on|off]\n"
    "             turn on/off tearing-free control\n"
    "\n"
    "        tv:[on|off]\n"
    "             turn on/off tv-out\n"
    "\n"
    "        tvsys:[ntsc|pal]\n"
    "             switch tv system\n"
    "\n"
    "        reg:[lcd|dpi|dsi|tvc|tve]\n"
    "             dump hw register values\n"
    "\n"
    "        regw:addr=val\n"
    "             write hw register\n"
    "\n"
    "        regr:addr\n"
    "             read hw register\n"
    "\n"
    "       cpfbonly:[on|off]\n"
    "             capture UI layer only on/off\n"
    "\n"
    "       esd:[on|off]\n"
    "             esd kthread on/off\n"
    "       HQA:[NormalToFactory|FactoryToNormal]\n"
    "             for HQA requirement\n"
    "\n"
    "       mmp\n"
    "             Register MMProfile events\n"
	"\n"
	"       dump_fb:[on|off[,down_sample_x[,down_sample_y,[delay]]]]\n"
	"             Start/end to capture framebuffer every delay(ms)\n"
	"\n"
	"       dump_ovl:[on|off[,down_sample_x[,down_sample_y]]]\n"
	"             Start to capture OVL only once\n"
	"\n"
	"       dump_layer:[on|off[,down_sample_x[,down_sample_y]]]\n"
	"             Start/end to capture current enabled OVL layer every frame\n"
    ;


// ---------------------------------------------------------------------------
//  Information Dump Routines
// ---------------------------------------------------------------------------

void init_mtkfb_mmp_events(void)
{
    if (MTKFB_MMP_Events.MTKFB == 0)
    {
        MTKFB_MMP_Events.MTKFB = MMProfileRegisterEvent(MMP_RootEvent, "MTKFB");
        MTKFB_MMP_Events.PanDisplay = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "PanDisplay");
        MTKFB_MMP_Events.SetOverlayLayer = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "SetOverlayLayer");
        MTKFB_MMP_Events.SetOverlayLayers = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "SetOverlayLayers");
        MTKFB_MMP_Events.TrigOverlayOut = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "TrigOverlayOut");
        MTKFB_MMP_Events.UpdateScreenImpl = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "UpdateScreenImpl");
        MTKFB_MMP_Events.VSync = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "VSync");
        MTKFB_MMP_Events.UpdateConfig = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "UpdateConfig");
        MTKFB_MMP_Events.EsdCheck = MMProfileRegisterEvent(MTKFB_MMP_Events.UpdateConfig, "EsdCheck");
        MTKFB_MMP_Events.ConfigOVL = MMProfileRegisterEvent(MTKFB_MMP_Events.UpdateConfig, "ConfigOVL");
        MTKFB_MMP_Events.ConfigAAL = MMProfileRegisterEvent(MTKFB_MMP_Events.UpdateConfig, "ConfigAAL");
        MTKFB_MMP_Events.ConfigMemOut = MMProfileRegisterEvent(MTKFB_MMP_Events.UpdateConfig, "ConfigMemOut");
        MTKFB_MMP_Events.ScreenUpdate = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "ScreenUpdate");
        MTKFB_MMP_Events.CaptureFramebuffer = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "CaptureFB");
        MTKFB_MMP_Events.RegUpdate = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "RegUpdate");
        MTKFB_MMP_Events.EarlySuspend = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "EarlySuspend");
        MTKFB_MMP_Events.DispDone = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "DispDone");
        MTKFB_MMP_Events.DSICmd = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "DSICmd");
        MTKFB_MMP_Events.DSIIRQ = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "DSIIrq");
        MTKFB_MMP_Events.WaitVSync = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "WaitVSync");
        MTKFB_MMP_Events.LayerDump = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "LayerDump");
        MTKFB_MMP_Events.Layer[0] = MMProfileRegisterEvent(MTKFB_MMP_Events.LayerDump, "Layer0");
        MTKFB_MMP_Events.Layer[1] = MMProfileRegisterEvent(MTKFB_MMP_Events.LayerDump, "Layer1");
        MTKFB_MMP_Events.Layer[2] = MMProfileRegisterEvent(MTKFB_MMP_Events.LayerDump, "Layer2");
        MTKFB_MMP_Events.Layer[3] = MMProfileRegisterEvent(MTKFB_MMP_Events.LayerDump, "Layer3");
        MTKFB_MMP_Events.OvlDump = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "OvlDump");
        MTKFB_MMP_Events.FBDump = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "FBDump");
        MTKFB_MMP_Events.DSIRead = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "DSIRead");
        MTKFB_MMP_Events.GetLayerInfo = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "GetLayerInfo");
        MTKFB_MMP_Events.LayerInfo[0] = MMProfileRegisterEvent(MTKFB_MMP_Events.GetLayerInfo, "LayerInfo0");
        MTKFB_MMP_Events.LayerInfo[1] = MMProfileRegisterEvent(MTKFB_MMP_Events.GetLayerInfo, "LayerInfo1");
        MTKFB_MMP_Events.LayerInfo[2] = MMProfileRegisterEvent(MTKFB_MMP_Events.GetLayerInfo, "LayerInfo2");
        MTKFB_MMP_Events.LayerInfo[3] = MMProfileRegisterEvent(MTKFB_MMP_Events.GetLayerInfo, "LayerInfo3");
        MTKFB_MMP_Events.IOCtrl = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "IOCtrl");
        MTKFB_MMP_Events.Debug = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "Debug");
        MMProfileEnableEventRecursive(MTKFB_MMP_Events.MTKFB, 1);
    }
}

static __inline int is_layer_enable(unsigned int roi_ctl, unsigned int layer)
{
    return (roi_ctl >> (31 - layer)) & 0x1;
}

static void dump_layer_info(void)
{
	unsigned int i;
	for(i=0;i<4;i++){
		printk("LayerInfo in LCD driver, layer=%d,layer_en=%d, source=%d, fmt=%d, addr=0x%x, x=%d, y=%d \n\
		w=%d, h=%d, pitch=%d, keyEn=%d, key=%d, aen=%d, alpha=%d \n ",
	    cached_layer_config[i].layer,   // layer
	    cached_layer_config[i].layer_en,
	    cached_layer_config[i].source,   // data source (0=memory)
	    cached_layer_config[i].fmt,
	    cached_layer_config[i].addr, // addr
	    cached_layer_config[i].dst_x,  // x
	    cached_layer_config[i].dst_y,  // y
	    cached_layer_config[i].dst_w, // width
	    cached_layer_config[i].dst_h, // height
	    cached_layer_config[i].src_pitch, //pitch, pixel number
	    cached_layer_config[i].keyEn,  //color key
	    cached_layer_config[i].key,  //color key
	    cached_layer_config[i].aen, // alpha enable
	    cached_layer_config[i].alpha);
	}
}


// ---------------------------------------------------------------------------
//  FPS Log
// ---------------------------------------------------------------------------

typedef struct {
    long int current_lcd_time_us;
    long int current_te_delay_time_us;
    long int total_lcd_time_us;
    long int total_te_delay_time_us;
    long int start_time_us;
    long int trigger_lcd_time_us;
    unsigned int trigger_lcd_count;

    long int current_hdmi_time_us;
    long int total_hdmi_time_us;
    long int hdmi_start_time_us;
    long int trigger_hdmi_time_us;
    unsigned int trigger_hdmi_count;
} FPS_LOGGER;

static FPS_LOGGER fps = {0};
static FPS_LOGGER hdmi_fps = {0};

static long int get_current_time_us(void)
{
    struct timeval t;
    do_gettimeofday(&t);
    return (t.tv_sec & 0xFFF) * 1000000 + t.tv_usec;
}


static void __inline reset_fps_logger(void)
{
    memset(&fps, 0, sizeof(fps));
}

static void __inline reset_hdmi_fps_logger(void)
{
    memset(&hdmi_fps, 0, sizeof(hdmi_fps));
}

void DBG_OnTriggerLcd(void)
{
    if (!dbg_opt.en_fps_log && !dbg_opt.en_touch_latency_log) return;

    fps.trigger_lcd_time_us = get_current_time_us();
    if (fps.trigger_lcd_count == 0) {
        fps.start_time_us = fps.trigger_lcd_time_us;
    }
}

void DBG_OnTriggerHDMI(void)
{
    if (!dbg_opt.en_fps_log && !dbg_opt.en_touch_latency_log) return;

    hdmi_fps.trigger_hdmi_time_us = get_current_time_us();
    if (hdmi_fps.trigger_hdmi_count == 0) {
        hdmi_fps.hdmi_start_time_us = hdmi_fps.trigger_hdmi_time_us;
    }
}

void DBG_OnTeDelayDone(void)
{
    long int time;

    if (!dbg_opt.en_fps_log && !dbg_opt.en_touch_latency_log) return;

    time = get_current_time_us();
    fps.current_te_delay_time_us = (time - fps.trigger_lcd_time_us);
    fps.total_te_delay_time_us += fps.current_te_delay_time_us;
}


void DBG_OnLcdDone(void)
{
    long int time;

    if (!dbg_opt.en_fps_log && !dbg_opt.en_touch_latency_log) return;

    // deal with touch latency log

    time = get_current_time_us();
    fps.current_lcd_time_us = (time - fps.trigger_lcd_time_us);

#if 0   // FIXME
    if (dbg_opt.en_touch_latency_log && tpd_start_profiling) {

        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "Touch Latency: %ld ms\n",
               (time - tpd_last_down_time) / 1000);

        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "LCD update time %ld ms (TE delay %ld ms + LCD %ld ms)\n",
               fps.current_lcd_time_us / 1000,
               fps.current_te_delay_time_us / 1000,
               (fps.current_lcd_time_us - fps.current_te_delay_time_us) / 1000);

        tpd_start_profiling = 0;
    }
#endif

    if (!dbg_opt.en_fps_log) return;

    // deal with fps log

    fps.total_lcd_time_us += fps.current_lcd_time_us;
    ++ fps.trigger_lcd_count;

    if (fps.trigger_lcd_count >= dbg_opt.log_fps_wnd_size) {

        long int f = fps.trigger_lcd_count * 100 * 1000 * 1000
                     / (time - fps.start_time_us);

        long int update = fps.total_lcd_time_us * 100
                          / (1000 * fps.trigger_lcd_count);

        long int te = fps.total_te_delay_time_us * 100
                      / (1000 * fps.trigger_lcd_count);

        long int lcd = (fps.total_lcd_time_us - fps.total_te_delay_time_us) * 100
                       / (1000 * fps.trigger_lcd_count);

        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "MTKFB FPS: %ld.%02ld, Avg. update time: %ld.%02ld ms "
               "(TE delay %ld.%02ld ms, LCD %ld.%02ld ms)\n",
               f / 100, f % 100,
               update / 100, update % 100,
               te / 100, te % 100,
               lcd / 100, lcd % 100);
		reset_fps_logger();
	}
}

void DBG_OnHDMIDone(void)
{
    long int time;

    if (!dbg_opt.en_fps_log && !dbg_opt.en_touch_latency_log) return;

    // deal with touch latency log

    time = get_current_time_us();
    hdmi_fps.current_hdmi_time_us = (time - hdmi_fps.trigger_hdmi_time_us);


    if (!dbg_opt.en_fps_log) return;

    // deal with fps log

    hdmi_fps.total_hdmi_time_us += hdmi_fps.current_hdmi_time_us;
    ++ hdmi_fps.trigger_hdmi_count;

    if (hdmi_fps.trigger_hdmi_count >= dbg_opt.log_fps_wnd_size) {

        long int f = hdmi_fps.trigger_hdmi_count * 100 * 1000 * 1000
                     / (time - hdmi_fps.hdmi_start_time_us);

        long int update = hdmi_fps.total_hdmi_time_us * 100
                          / (1000 * hdmi_fps.trigger_hdmi_count);

        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "[HDMI] FPS: %ld.%02ld, Avg. update time: %ld.%02ld ms\n",
               f / 100, f % 100,
               update / 100, update % 100);

        reset_hdmi_fps_logger();
    }
}

// ---------------------------------------------------------------------------
//  Command Processor
// ---------------------------------------------------------------------------
extern void mtkfb_clear_lcm(void);
extern void hdmi_force_init(void);

static void process_dbg_opt(const char *opt)
{
    if (0 == strncmp(opt, "hdmion", 6))
    {
//	hdmi_force_init();
    }
    else if (0 == strncmp(opt, "fps:", 4))
    {
        if (0 == strncmp(opt + 4, "on", 2)) {
            dbg_opt.en_fps_log = 1;
        } else if (0 == strncmp(opt + 4, "off", 3)) {
            dbg_opt.en_fps_log = 0;
        } else {
            goto Error;
        }
        reset_fps_logger();
    }
    else if (0 == strncmp(opt, "tl:", 3))
    {
        if (0 == strncmp(opt + 3, "on", 2)) {
            dbg_opt.en_touch_latency_log = 1;
        } else if (0 == strncmp(opt + 3, "off", 3)) {
            dbg_opt.en_touch_latency_log = 0;
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "black", 5))
    {
	mtkfb_clear_lcm();
    }
    else if (0 == strncmp(opt, "suspend", 4))
    {
        #if 0
        DISP_PanelEnable(FALSE);
        DISP_PowerEnable(FALSE);
        #else
        mtkfb_early_suspend(NULL);
        #endif
    }
    else if (0 == strncmp(opt, "resume", 4))
    {
        #if 0
        DISP_PowerEnable(TRUE);
        DISP_PanelEnable(TRUE);
        #else
        mtkfb_late_resume(NULL);
        #endif
    }
    else if (0 == strncmp(opt, "lcm:", 4))
    {
        if (0 == strncmp(opt + 4, "on", 2))
	{
            DISP_PanelEnable(TRUE);
        }
	else if (0 == strncmp(opt + 4, "off", 3))
	{
            DISP_PanelEnable(FALSE);
        }
	else if (0 == strncmp(opt + 4, "init", 4))
	{
		if (NULL != lcm_drv && NULL != lcm_drv->init)
		{
        		lcm_drv->init();
    		}
        }
	else if (0 == strncmp(opt + 4, "reset", 5))
	{
		LCD_SetResetSignal(1);
		msleep(10);
		LCD_SetResetSignal(0);
		msleep(10);
		LCD_SetResetSignal(1);
    	}
    }
    	else if (0 == strncmp(opt, "cabc:", 5))
    	{
        	if (0 == strncmp(opt + 5, "ui", 2))
		{
			mtkfb_set_backlight_mode(1);
        	}
		else if (0 == strncmp(opt + 5, "mov", 3))
		{
			mtkfb_set_backlight_mode(3);
        	}
		else if (0 == strncmp(opt + 5, "still", 5))
		{
			mtkfb_set_backlight_mode(2);
        	}
		else
		{
           		goto Error;
        	}
    	}
    else if (0 == strncmp(opt, "lcd:", 4))
    {
        if (0 == strncmp(opt + 4, "on", 2)) {
            DISP_PowerEnable(TRUE);
        } else if (0 == strncmp(opt + 4, "off", 3)) {
            DISP_PowerEnable(FALSE);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "vsynclog:", 9))
    {
        if (0 == strncmp(opt + 9, "on", 2))
        {
            EnableVSyncLog = 1;
        } else if (0 == strncmp(opt + 9, "off", 3))
        {
            EnableVSyncLog = 0;
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "layer", 5))
    {
        dump_layer_info();
    }
    else if (0 == strncmp(opt, "regw:", 5))
    {
        char *p = (char *)opt + 5;
        unsigned long addr = simple_strtoul(p, &p, 16);
        unsigned long val  = simple_strtoul(p + 1, &p, 16);

        if (addr) {
            OUTREG32(addr, val);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "regr:", 5))
    {
        char *p = (char *)opt + 5;
        unsigned int addr = (unsigned int) simple_strtoul(p, &p, 16);

        if (addr) {
            DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "Read register 0x%08x: 0x%08x\n", addr, INREG32(addr));
        } else {
            goto Error;
        }
    }
    else if(0 == strncmp(opt, "bkl:", 4))
    {
        char *p = (char *)opt + 4;
        unsigned int level = (unsigned int) simple_strtoul(p, &p, 10);

        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "process_dbg_opt(), set backlight level = %d\n", level);
        DISP_SetBacklight(level);
    }
    else if(0 == strncmp(opt, "dither:", 7))
    {
        unsigned lrs, lgs, lbs, dbr, dbg, dbb;
        char *p = (char *)opt + 7;

        lrs = (unsigned int) simple_strtoul(p, &p, 16);
        p++;
        lgs = (unsigned int) simple_strtoul(p, &p, 16);
        p++;
        lbs = (unsigned int) simple_strtoul(p, &p, 16);
        p++;
        dbr = (unsigned int) simple_strtoul(p, &p, 16);
        p++;
        dbg = (unsigned int) simple_strtoul(p, &p, 16);
        p++;
        dbb = (unsigned int) simple_strtoul(p, &p, 16);

        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "process_dbg_opt(), %d %d %d %d %d %d\n", lrs, lgs, lbs, dbr, dbg, dbb);
    }
    else if (0 == strncmp(opt, "mtkfblog:", 9))
    {
        if (0 == strncmp(opt + 9, "on", 2)) {
            mtkfb_log_enable(true);
        } else if (0 == strncmp(opt + 9, "off", 3)) {
            mtkfb_log_enable(false);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "displog:", 8))
    {
        if (0 == strncmp(opt + 8, "on", 2)) {
            disp_log_enable(true);
        } else if (0 == strncmp(opt + 8, "off", 3)) {
            disp_log_enable(false);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "mtkfb_vsynclog:", 15))
    {
        if (0 == strncmp(opt + 15, "on", 2)) {
            mtkfb_vsync_log_enable(true);
        } else if (0 == strncmp(opt + 15, "off", 3)) {
            mtkfb_vsync_log_enable(false);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "log:", 4))
    {
        if (0 == strncmp(opt + 4, "on", 2)) {
			mtkfb_log_enable(true);
			disp_log_enable(true);
        } else if (0 == strncmp(opt + 4, "off", 3)) {
            mtkfb_log_enable(false);
			disp_log_enable(false);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "update", 6))
    {
		DISP_UpdateScreen(0, 0, DISP_GetScreenWidth(), DISP_GetScreenHeight());
    }
    else if (0 == strncmp(opt, "pan_disp", 8))
    {
		mtkfb_pan_disp_test();
    }
    else if (0 == strncmp(opt, "sem_cnt", 7))
    {
		mtkfb_show_sem_cnt();
    }
    else if (0 == strncmp(opt, "hang:", 5))
    {
        if (0 == strncmp(opt + 5, "on", 2)) {
            mtkfb_hang_test(true);
        } else if (0 == strncmp(opt + 5, "off", 3)) {
            mtkfb_hang_test(false);
        } else{
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "cpfbonly:", 9))
    {
        if (0 == strncmp(opt + 9, "on", 2))
        {
            mtkfb_capture_fb_only(true);
        }
        else if (0 == strncmp(opt + 9, "off", 3))
        {
            mtkfb_capture_fb_only(false);
        }
    }
    else if (0 == strncmp(opt, "esd:", 4))
    {
        if (0 == strncmp(opt + 4, "on", 2))
        {
            esd_recovery_pause(FALSE);
        }
        else if (0 == strncmp(opt + 4, "off", 3))
        {
            esd_recovery_pause(TRUE);
        }
    }
    else if (0 == strncmp(opt, "HQA:", 4))
    {
        if (0 == strncmp(opt + 4, "NormalToFactory", 15))
        {
            mtkfb_switch_normal_to_factory();
        }
        else if (0 == strncmp(opt + 4, "FactoryToNormal", 15))
        {
            mtkfb_switch_factory_to_normal();
        }
    }
    else if (0 == strncmp(opt, "mmp", 3))
    {
        init_mtkfb_mmp_events();
    }
    else if (0 == strncmp(opt, "dump_layer:", 11))
    {
        if (0 == strncmp(opt + 11, "on", 2))
        {
            char *p = (char *)opt + 14;
            gCaptureLayerDownX = simple_strtoul(p, &p, 10);
            gCaptureLayerDownY = simple_strtoul(p+1, &p, 10);
            gCaptureLayerEnable = 1;
        }
        else if (0 == strncmp(opt + 11, "off", 3))
        {
            gCaptureLayerEnable = 0;
        }
    }
    else if (0 == strncmp(opt, "dump_ovl:", 9))
    {
        if (0 == strncmp(opt + 9, "on", 2))
        {
            char *p = (char *)opt + 12;
            gCaptureOvlDownX = simple_strtoul(p, &p, 10);
            gCaptureOvlDownY = simple_strtoul(p+1, &p, 10);
            gCaptureOvlThreadEnable = 1;
			wake_up_process(captureovl_task);
        }
        else if (0 == strncmp(opt + 9, "off", 3))
        {
            gCaptureOvlThreadEnable = 0;
        }
    }
    else if (0 == strncmp(opt, "dump_fb:", 8))
    {
        if (0 == strncmp(opt + 8, "on", 2))
        {
            char *p = (char *)opt + 11;
            gCaptureFBDownX = simple_strtoul(p, &p, 10);
            gCaptureFBDownY = simple_strtoul(p+1, &p, 10);
            gCaptureFBPeriod = simple_strtoul(p+1, &p, 10);
            gCaptureFBEnable = 1;
			wake_up_interruptible(&gCaptureFBWQ);
        }
        else if (0 == strncmp(opt + 8, "off", 3))
        {
            gCaptureFBEnable = 0;
        }
    }
    else
	{
	    if (disphal_process_dbg_opt(opt))
		goto Error;
	}

    return;

Error:
    DISP_LOG_PRINT(ANDROID_LOG_INFO, "ERROR", "parse command error!\n\n%s", STR_HELP);
}


static void process_dbg_cmd(char *cmd)
{
    char *tok;

    DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "[mtkfb_dbg] %s\n", cmd);

    while ((tok = strsep(&cmd, " ")) != NULL)
    {
        process_dbg_opt(tok);
    }
}


// ---------------------------------------------------------------------------
//  Debug FileSystem Routines
// ---------------------------------------------------------------------------

struct dentry *mtkfb_dbgfs = NULL;


static ssize_t debug_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}


static char debug_buffer[2048];

static ssize_t debug_read(struct file *file,
                          char __user *ubuf, size_t count, loff_t *ppos)
{
    const int debug_bufmax = sizeof(debug_buffer) - 1;
    int n = 0;

    n += scnprintf(debug_buffer + n, debug_bufmax - n, STR_HELP);
    debug_buffer[n++] = 0;

    return simple_read_from_buffer(ubuf, count, ppos, debug_buffer, n);
}


static ssize_t debug_write(struct file *file,
                           const char __user *ubuf, size_t count, loff_t *ppos)
{
    const int debug_bufmax = sizeof(debug_buffer) - 1;
	size_t ret;

	ret = count;

	if (count > debug_bufmax)
        count = debug_bufmax;

	if (copy_from_user(&debug_buffer, ubuf, count))
		return -EFAULT;

	debug_buffer[count] = 0;

    process_dbg_cmd(debug_buffer);

    return ret;
}


static struct file_operations debug_fops = {
	.read  = debug_read,
    .write = debug_write,
	.open  = debug_open,
};

#ifdef MTKFB_DEBUG_FS_CAPTURE_LAYER_CONTENT_SUPPORT

static ssize_t layer_debug_open(struct inode *inode, struct file *file)
{
    MTKFB_LAYER_DBG_OPTIONS *dbgopt;
    ///record the private data
    file->private_data = inode->i_private;
    dbgopt = (MTKFB_LAYER_DBG_OPTIONS *)file->private_data;

    dbgopt->working_size = DISP_GetScreenWidth()*DISP_GetScreenHeight()*2 + 32;
    dbgopt->working_buf = (UINT32)vmalloc(dbgopt->working_size);
    if(dbgopt->working_buf == 0)
        DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "Vmalloc to get temp buffer failed\n");

    return 0;
}


static ssize_t layer_debug_read(struct file *file,
                          char __user *ubuf, size_t count, loff_t *ppos)
{
	return 0;
}


static ssize_t layer_debug_write(struct file *file,
                           const char __user *ubuf, size_t count, loff_t *ppos)
{
    MTKFB_LAYER_DBG_OPTIONS *dbgopt = (MTKFB_LAYER_DBG_OPTIONS *)file->private_data;

    DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "mtkfb_layer%d write is not implemented yet \n", dbgopt->layer_index);

    return count;
}

static int layer_debug_release(struct inode *inode, struct file *file)
{
    MTKFB_LAYER_DBG_OPTIONS *dbgopt;
    dbgopt = (MTKFB_LAYER_DBG_OPTIONS *)file->private_data;

    if(dbgopt->working_buf != 0)
        vfree((void *)dbgopt->working_buf);

    dbgopt->working_buf = 0;

    return 0;
}


static struct file_operations layer_debug_fops = {
	.read  = layer_debug_read,
    .write = layer_debug_write,
	.open  = layer_debug_open,
    .release = layer_debug_release,
};

#endif

void DBG_Init(void)
{
    mtkfb_dbgfs = debugfs_create_file("mtkfb",
        S_IFREG|S_IRUGO, NULL, (void *)0, &debug_fops);

    memset(&dbg_opt, sizeof(dbg_opt), 0);
	memset(&fps, sizeof(fps), 0);
    dbg_opt.log_fps_wnd_size = DEFAULT_LOG_FPS_WND_SIZE;

#ifdef MTKFB_DEBUG_FS_CAPTURE_LAYER_CONTENT_SUPPORT
    {
        unsigned int i;
        unsigned char a[13];

        a[0] = 'm';
        a[1] = 't';
        a[2] = 'k';
        a[3] = 'f';
        a[4] = 'b';
        a[5] = '_';
        a[6] = 'l';
        a[7] = 'a';
        a[8] = 'y';
        a[9] = 'e';
        a[10] = 'r';
        a[11] = '0';
        a[12] = '\0';

        for(i=0;i<DDP_OVL_LAYER_MUN;i++)
        {
            a[11] = '0' + i;
            mtkfb_layer_dbg_opt[i].layer_index = i;
            mtkfb_layer_dbgfs[i] = debugfs_create_file(a,
                S_IFREG|S_IRUGO, NULL, (void *)&mtkfb_layer_dbg_opt[i], &layer_debug_fops);
        }
    }
#endif
}


void DBG_Deinit(void)
{
    debugfs_remove(mtkfb_dbgfs);
#ifdef MTKFB_DEBUG_FS_CAPTURE_LAYER_CONTENT_SUPPORT
    {
        unsigned int i;

        for(i=0;i<DDP_OVL_LAYER_MUN;i++)
            debugfs_remove(mtkfb_layer_dbgfs[i]);
    }
#endif
}
//****************************************************************************
// the next part is for customization parameters of D-IC and DSI .
// ****************************************************************************
//extern LCM_DRIVER* lcm_driver_list[];
extern LCM_DRIVER *lcm_drv;
extern LCM_UTIL_FUNCS fbconfig_lcm_utils;
extern BOOL is_early_suspended;

#define FBCONFIG_MDELAY(n)	(fbconfig_lcm_utils.mdelay((n)))
#define SET_RESET_PIN(v)	(fbconfig_lcm_utils.set_reset_pin((v)))
#define dsi_set_cmdq(pdata, queue_size, force_update)		fbconfig_lcm_utils.dsi_set_cmdq(pdata, queue_size, force_update)
#define FBCONFIG_DEBUG 0
/* IOCTL commands. */

#define FBCONFIG_IOW(num, dtype)     _IOW('X', num, dtype)
#define FBCONFIG_IOR(num, dtype)     _IOR('X', num, dtype)
#define FBCONFIG_IOWR(num, dtype)    _IOWR('X', num, dtype)
#define FBCONFIG_IO(num)             _IO('X', num)

#define LCM_GET_ID     FBCONFIG_IOR(45, unsigned int)
#define LCM_GET_ESD    FBCONFIG_IOR(46, unsigned int)
#define DRIVER_IC_CONFIG    FBCONFIG_IOR(47, unsigned int)


#define MIPI_SET_CLK     FBCONFIG_IOW(51, unsigned int)
#define MIPI_SET_LANE    FBCONFIG_IOW(52, unsigned int)
#define MIPI_SET_TIMING  FBCONFIG_IOW(53, unsigned int)
#define MIPI_SET_VM      FBCONFIG_IOW(54, unsigned int) //mipi video mode timing setting
#define MIPI_SET_CC  	 FBCONFIG_IOW(55, unsigned int) //mipi non-continuous clock

#define TE_SET_ENABLE  FBCONFIG_IOW(61, unsigned int)
#define FB_LAYER_DUMP  FBCONFIG_IOW(62, unsigned int)


struct dentry *ConfigPara_dbgfs = NULL;
static bool record_list_initialed =0;
CONFIG_RECORD  * record_head =NULL;
CONFIG_RECORD  * tmp_last =NULL;//always point to the last node
CONFIG_RECORD  * record_tmp =NULL;
extern struct semaphore sem_early_suspend;
extern void fbconfig_disp_set_mipi_timing(MIPI_TIMING timing);
static void print_record (CONFIG_RECORD * record_head)
{
int z =0 ;
printk("sxk=>[PRINT_RECORD] type is %d\n",record_head->type);
for(z=0;z<record_head->ins_num;z++)
printk("sxk=>[PRINT_RECORD]data is 0x%x\n",record_head->ins_array[z]);
}

static void record_list_init(void)
{
	printk("sxk=>call record_list_init!!\n");
	record_head = (CONFIG_RECORD *)kmalloc(sizeof(CONFIG_RECORD), GFP_KERNEL);
	record_head->next = NULL;

	tmp_last = record_head ;//tmp_last always point to the last node;
	memset(record_head->ins_array,0x00,sizeof(int)*MAX_INSTRUCTION);
	record_tmp = (CONFIG_RECORD *)kmalloc(sizeof(CONFIG_RECORD), GFP_KERNEL);
	record_tmp->next = NULL ;
	memset(record_tmp->ins_array,0x00,sizeof(int)*MAX_INSTRUCTION);


}
static void record_list_add(void )
{
	CONFIG_RECORD * record_add = (CONFIG_RECORD *)kmalloc(sizeof(CONFIG_RECORD), GFP_KERNEL);
//ecord_add = record_tmp ;
	memcpy(record_add,record_tmp,sizeof(CONFIG_RECORD));
		if(tmp_last->next == NULL)
			tmp_last->next = record_add ;

	tmp_last = record_add ;
	tmp_last->next = NULL ;
	#if FBCONFIG_DEBUG
	print_record(tmp_last);
	#endif
	/*clear the data content in record_tmp */
	memset(record_tmp->ins_array,0x00,sizeof(int)*MAX_INSTRUCTION);
}

static void print_from_head_to_tail(void)
{
int z = 0 ;
CONFIG_RECORD  * tmp = record_head ;
printk("sxk=>print_from_head_to_tail:START\n");
while(tmp !=NULL)
{
printk("sxk=>record type is %d;record ins_num is %d\n",tmp->type,tmp->ins_num);
for(z=0;z<tmp->ins_num;z++)
	printk("sxk=>record conten is :0x%x\n",tmp->ins_array[z]);
tmp = tmp->next;
}
printk("sxk=>print_from_head_to_tail:END\n");

}

//RECORD_CMD = 0,
//RECORD_MS = 1,
//RECORD_PIN_SET	= 2,

 int fb_config_execute_cmd(void)
{
CONFIG_RECORD  * tmp = record_head ;
printk("sxk=>execute_cmd:START\n");
while(tmp !=NULL)
{
switch(tmp->type)
{
case RECORD_CMD:
	dsi_set_cmdq(tmp->ins_array, tmp->ins_num, 1);
	break;
case RECORD_MS:
	//FBCONFIG_MDELAY(tmp->ins_array[0]);
	msleep(tmp->ins_array[0]);
	break;
case RECORD_PIN_SET:
	SET_RESET_PIN(tmp->ins_array[0]);
	break;
default:
	printk("sxk=>No such Type!!!!!\n");
}
tmp = tmp->next;
}
printk("sxk=>execute_cmd:END\n");
return 0;

}



static void free_list_memory(void)
{
CONFIG_RECORD  * f1=record_head;
CONFIG_RECORD  * f2=f1;
if(NULL == record_head)
	return ;
while(NULL != f1->next)
{
f2 = f1->next ;
kfree(f1);
f1 = f2;
}
kfree(f1);
record_head = NULL ;
return ;
}


static ssize_t fbconfig_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}


static char fbconfig_buffer[2048];

static ssize_t fbconfig_read(struct file *file,
                          char __user *ubuf, size_t count, loff_t *ppos)
{
    const int debug_bufmax = sizeof(fbconfig_buffer) - 1;//2047
    int n = 0;

    n += scnprintf(fbconfig_buffer + n, debug_bufmax - n, "sxkhome");
    fbconfig_buffer[n++] = 0;
	//n = 5 ;
	//memcpy(fbconfig_buffer,"sxkhome",6);
    return simple_read_from_buffer(ubuf, count, ppos, fbconfig_buffer, n);
}

static ssize_t fbconfig_write(struct file *file,
                           const char __user *ubuf, size_t count, loff_t *ppos)
{
return 0 ;
}



static long fbconfig_ioctl(struct file * file, unsigned int cmd, unsigned long arg)
{
int ret =0 ;
void __user *argp = (void __user *)arg;
printk("sxk=>run in fbconfig_ioctl** \n");

switch (cmd)
    {
	case LCM_GET_ID:
	{
		LCM_DRIVER * lcm = lcm_drv;
		// get_lcm_id() need implemented in lcm driver ...
		#if 0
		unsigned int lcm_id =lcm->get_lcm_id();
		#else
		unsigned int lcm_id = 0 ;
		#endif
		return copy_to_user(argp, &lcm_id,  sizeof(lcm_id)) ? -EFAULT : 0;
	}
	case DRIVER_IC_CONFIG:
	{
	printk("sxk=>run in case:DRIVER_IC_CONFIG** \n");
	if(record_list_initialed == 0)
		{
		record_list_init();
		  if (copy_from_user(record_head, (void __user *)arg, sizeof(CONFIG_RECORD))) {
			printk("sxk=>copy_from_user failed! line:%d \n", __LINE__);
			return -EFAULT;
		    }
		  print_record(record_head);
			record_list_initialed = 1;
		}
		else
		{
			if (copy_from_user(record_tmp, (void __user *)arg, sizeof(CONFIG_RECORD))) {
			printk("[DRIVER_IC_CONFIG]: copy_from_user failed! line:%d \n", __LINE__);
			return -EFAULT;
			}
			#if FBCONFIG_DEBUG
			printk("sxk=>will print before add to list \n");
			print_record(record_tmp);
			#endif
			record_list_add();//add new node to list ;

		}

		return  0;
	}
	case MIPI_SET_CLK:
	{
		unsigned int clk;
		printk("sxk=>debug.c call set mipi clk line:%d \n", __LINE__);
		if(copy_from_user(&clk,(void __user*)argp,sizeof(clk)))
        {
            printk("[MIPI_SET_CLK]: copy_from_user failed! line:%d \n", __LINE__);
            return -EFAULT;
        }
        else
        {
        printk("sxk=>debug.c call set mipi clk line:%d \n", __LINE__);
		fbconfig_disp_set_mipi_clk(clk);
        }
		return 0 ;
	}
	case MIPI_SET_LANE:
	{
		unsigned int lane_num;
		if(copy_from_user(&lane_num,(void __user*)argp,sizeof(lane_num)))
        {
            printk("[MIPI_SET_LANE]: copy_from_user failed! line:%d \n", __LINE__);
            return -EFAULT;
        }
        else
        {
		fbconfig_disp_set_mipi_lane_num(lane_num);
        }
		return 0 ;
	}
	case MIPI_SET_TIMING:
	{
		if (!is_early_suspended)
	  {
		MIPI_TIMING timing;
		if(copy_from_user(&timing,(void __user*)argp,sizeof(timing)))
        {
            printk("[MIPI_SET_TIMING]: copy_from_user failed! line:%d \n", __LINE__);
            return -EFAULT;
        }
        else
        {
		fbconfig_disp_set_mipi_timing(timing);
        }
		return 0 ;
	 }
	else
	return -EFAULT;
	}
	case TE_SET_ENABLE:
	{
		char enable;
		if(copy_from_user(&enable,(void __user*)argp,sizeof(enable)))
        {
            printk("[TE_SET_ENABLE]: copy_from_user failed! line:%d \n", __LINE__);
            return -EFAULT;
        }
        else
        {
		fbconfig_disp_set_te_enable(enable);
        }
		return 0 ;
	}
	case LCM_GET_ESD:
	{
		#if 0
		int displayid = 0;
		if (copy_from_user(&displayid, (void __user *)arg, sizeof(displayid))) {
			MTKFB_LOG("[FB]: copy_from_user failed! line:%d \n", __LINE__);
			return -EFAULT;
		}
		printk("%s, display_id=%d\n", __func__, displayid);
		MTKFB_FUNC();
		if (displayid > MTKFB_MAX_DISPLAY_COUNT) {
			MTKFB_LOG("[FB]: invalid display id:%d \n", displayid);
			return -EFAULT;
		}
		if (copy_to_user((void __user *)arg, &(dispif_info[displayid]),  sizeof(mtk_dispif_info_t))) {
			MTKFB_LOG("[FB]: copy_to_user failed! line:%d \n", __LINE__);
			r = -EFAULT;
		}
		return (r);
		#endif
	}
	//case MTKFB_POWEROFF:
	default :
	 return ret ;


}
}


static int fbconfig_release(struct inode *inode, struct file *file)
{
if(record_head ==NULL)
return 0 ;
else
{
record_list_initialed = 0;
#if 1//FBCONFIG_DEBUG
print_from_head_to_tail();
#endif
#if 1
if(lcm_params->dsi.mode != CMD_MODE)
{
printk("sxk=>01will exec cmd!! in fbconfig_release()\n");

if (down_interruptible(&sem_early_suspend)) {
				printk("sxk=>can't get semaphore in execute_cmd()\n");
				return 0;
		   }
fbconfig_dsi_vdo_prepare();//video mode
printk("sxk=>FINISH!!\n");

up(&sem_early_suspend);
}
else{//cmd mode
if (down_interruptible(&sem_early_suspend)) {
            printk("sxk=>can't get semaphore in execute_cmd()\n");
            return 0;
       }
fb_config_execute_cmd();
up(&sem_early_suspend);
}
#endif
/*free the memory .....*/
printk("sxk=>will free the memory \n");
free_list_memory();
return 0;
}
}




static struct file_operations fbconfig_fops = {
	.read  = fbconfig_read,
    .write = fbconfig_write,
	.open  = fbconfig_open,
	.unlocked_ioctl = fbconfig_ioctl,
	.release =	fbconfig_release,
};

void ConfigPara_Init(void)
{
    ConfigPara_dbgfs = debugfs_create_file("fbconfig",
        S_IFREG|S_IRUGO, NULL, (void *)0, &fbconfig_fops);

}

void ConfigPara_Deinit(void)
{
    debugfs_remove(ConfigPara_dbgfs);
}

