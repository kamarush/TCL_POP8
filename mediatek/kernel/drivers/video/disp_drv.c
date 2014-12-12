#include <linux/delay.h>
#include <linux/fb.h>
#include "mtkfb.h"
#include <asm/uaccess.h>

#include "disp_drv.h"
#include "ddp_hal.h"
#include "disp_drv_log.h"
#include "lcm_drv.h"
#include "debug.h"

#include <linux/disp_assert_layer.h>

#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
//#include "mach/mt_clkmgr.h"
#include <linux/vmalloc.h>
#include "mtkfb_info.h"
#include <linux/dma-mapping.h>
extern unsigned int lcd_fps;
extern BOOL is_early_suspended;
extern struct semaphore sem_early_suspend;

extern unsigned int EnableVSyncLog;

#define LCM_ESD_CHECK_MAX_COUNT 5

#define ALIGN_TO(x, n)  \
	(((x) + ((n) - 1)) & ~((n) - 1))

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

unsigned int FB_LAYER = DISP_DEFAULT_UI_LAYER_ID;
static const DISP_IF_DRIVER *disp_if_drv = NULL;
const LCM_DRIVER  *lcm_drv  = NULL;
extern LCM_PARAMS *lcm_params;

static volatile int direct_link_layer = -1;
static UINT32 disp_fb_bpp = 32;     ///ARGB8888
static UINT32 disp_fb_pages = 3;     ///double buffer

BOOL is_engine_in_suspend_mode = FALSE;
BOOL is_lcm_in_suspend_mode    = FALSE;
static UINT32 dal_layerPA;
static UINT32 dal_layerVA;

static unsigned long u4IndexOfLCMList = 0;

static wait_queue_head_t config_update_wq;
static struct task_struct *config_update_task = NULL;
static int config_update_task_wakeup = 0;
extern atomic_t OverlaySettingDirtyFlag;
extern atomic_t OverlaySettingApplied;
extern unsigned int PanDispSettingPending;
extern unsigned int PanDispSettingDirty;
extern unsigned int PanDispSettingApplied;

extern unsigned int fb_pa;

extern bool is_ipoh_bootup;

extern struct mutex OverlaySettingMutex;
extern wait_queue_head_t reg_update_wq;
static wait_queue_head_t vsync_wq;
static bool vsync_wq_flag = false;

static struct hrtimer cmd_mode_update_timer;
static ktime_t cmd_mode_update_timer_period;

static bool needStartEngine = true;

extern unsigned int need_esd_check;
extern wait_queue_head_t esd_check_wq;
extern BOOL esd_kthread_pause;

DEFINE_MUTEX(MemOutSettingMutex);
static struct disp_path_config_mem_out_struct MemOutConfig;
static BOOL is_immediateupdate = false;

DEFINE_MUTEX(LcmCmdMutex);

unsigned int disp_running = 0;
DECLARE_WAIT_QUEUE_HEAD(disp_done_wq);

OVL_CONFIG_STRUCT cached_layer_config[DDP_OVL_LAYER_MUN] =
{
    {.layer = 0, .isDirty = 1},
    {.layer = 1, .isDirty = 1},
    {.layer = 2, .isDirty = 1},
    {.layer = 3, .isDirty = 1}
};

static OVL_CONFIG_STRUCT _layer_config[2][DDP_OVL_LAYER_MUN];
static unsigned int layer_config_index = 0;
OVL_CONFIG_STRUCT* captured_layer_config = _layer_config[0];
OVL_CONFIG_STRUCT* realtime_layer_config = _layer_config[0];

struct DBG_OVL_CONFIGS
{
    OVL_CONFIG_STRUCT Layer0;
    OVL_CONFIG_STRUCT Layer1;
    OVL_CONFIG_STRUCT Layer2;
    OVL_CONFIG_STRUCT Layer3;
};

unsigned int gCaptureLayerEnable = 0;
unsigned int gCaptureLayerDownX = 10;
unsigned int gCaptureLayerDownY = 10;

struct task_struct *captureovl_task = NULL;
static int _DISP_CaptureOvlKThread(void *data);
static unsigned int gWakeupCaptureOvlThread = 0;
unsigned int gCaptureOvlThreadEnable = 0;
unsigned int gCaptureOvlDownX = 10;
unsigned int gCaptureOvlDownY = 10;

struct task_struct *capturefb_task = NULL;
static int _DISP_CaptureFBKThread(void *data);
#ifdef USER_BUILD_KERNEL
unsigned int gCaptureFBEnable = 0;
#else
unsigned int gCaptureFBEnable = 0;
#endif
unsigned int gCaptureFBDownX = 10;
unsigned int gCaptureFBDownY = 10;
unsigned int gCaptureFBPeriod = 100;
DECLARE_WAIT_QUEUE_HEAD(gCaptureFBWQ);

extern struct fb_info *mtkfb_fbi;

unsigned int is_video_mode_running = 0;

DEFINE_SEMAPHORE(sem_update_screen);//linux 3.0 porting
static BOOL isLCMFound 					= FALSE;
/// Some utilities
#define ALIGN_TO_POW_OF_2(x, n)  \
                (((x) + ((n) - 1)) & ~((n) - 1))

#define ALIGN_TO(x, n)  \
                (((x) + ((n) - 1)) & ~((n) - 1))



static size_t disp_log_on = false;
#define DISP_LOG(fmt, arg...) \
    do { \
        if (disp_log_on) DISP_LOG_PRINT(ANDROID_LOG_WARN, "COMMON", fmt, ##arg); \
    }while (0)

#define DISP_FUNC()	\
    do { \
        if(disp_log_on) DISP_LOG_PRINT(ANDROID_LOG_INFO, "COMMON", "[Func]%s\n", __func__); \
    }while (0)

void disp_log_enable(int enable)
{
    disp_log_on = enable;
    DISP_LOG("disp common log %s\n", enable?"enabled":"disabled");
}
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

extern LCM_DRIVER* lcm_driver_list[];
extern unsigned int lcm_count;

static void disp_dump_lcm_parameters(LCM_PARAMS *lcm_params)
{
    unsigned char *LCM_TYPE_NAME[] = {"DBI", "DPI", "DSI"};
    unsigned char *LCM_CTRL_NAME[] = {"NONE", "SERIAL", "PARALLEL", "GPIO"};

    if(lcm_params == NULL)
        return;

    printk("[mtkfb] LCM TYPE: %s\n", LCM_TYPE_NAME[lcm_params->type]);
    printk("[mtkfb] LCM INTERFACE: %s\n", LCM_CTRL_NAME[lcm_params->ctrl]);
    printk("[mtkfb] LCM resolution: %d x %d\n", lcm_params->width, lcm_params->height);

    return;
}

char disp_lcm_name[256] = {0};
BOOL disp_get_lcm_name_boot(char *cmdline)
{
    BOOL ret = FALSE;
    char *p, *q;

    p = strstr(cmdline, "lcm=");
    if(p == NULL)
    {
        // we can't find lcm string in the command line,
        // the uboot should be old version, or the kernel is loaded by ICE debugger
        return DISP_SelectDeviceBoot(NULL);
    }

    p += 4;
    if((p - cmdline) > strlen(cmdline+1))
    {
        ret = FALSE;
        goto done;
    }

    isLCMFound = strcmp(p, "0");
    printk("[mtkfb] LCM is %sconnected\n", ((isLCMFound)?"":"not "));
    p += 2;
    q = p;
    while(*q != ' ' && *q != '\0')
        q++;

    memset((void*)disp_lcm_name, 0, sizeof(disp_lcm_name));
    strncpy((char*)disp_lcm_name, (const char*)p, (int)(q-p));

    if(DISP_SelectDeviceBoot(disp_lcm_name))
        ret = TRUE;

done:
    return ret;
}

static BOOL disp_drv_init_context(void)
{
    if (disp_if_drv != NULL && lcm_drv != NULL){
        return TRUE;
    }

    if(!isLCMFound)
        DISP_DetectDevice();

    disphal_init_ctrl_if();

    disp_if_drv = disphal_get_if_driver();

    if (!disp_if_drv) return FALSE;

    return TRUE;
}

BOOL DISP_IsLcmFound(void)
{
    return isLCMFound;
}

BOOL DISP_IsContextInited(void)
{
    if(lcm_params && disp_if_drv && lcm_drv)
        return TRUE;
    else
        return FALSE;
}

BOOL DISP_SelectDeviceBoot(const char* lcm_name)
{
    LCM_DRIVER *lcm = NULL;
    int i;

    printk("%s\n", __func__);

    if(lcm_name == NULL)
    {
        // we can't do anything in boot stage if lcm_name is NULL
        return false;
    }
    lcm_drv = disphal_get_lcm_driver(lcm_name, (unsigned int *)&u4IndexOfLCMList);

    if (NULL == lcm_drv)
    {
        printk("%s, get_lcm_driver returns NULL\n", __func__);
        return FALSE;
    }
    isLCMFound = TRUE;

    disp_if_drv = disphal_get_if_driver();

    disp_dump_lcm_parameters(lcm_params);
    return TRUE;
}

BOOL DISP_SelectDevice(const char* lcm_name)
{
    lcm_drv = disphal_get_lcm_driver(lcm_name, &u4IndexOfLCMList);
    if (NULL == lcm_drv)
    {
        printk("%s, disphal_get_lcm_driver() returns NULL\n", __func__);
        return FALSE;
    }
    isLCMFound = TRUE;

    disp_dump_lcm_parameters(lcm_params);
    return disp_drv_init_context();
}

BOOL DISP_DetectDevice(void)
{
    lcm_drv = disphal_get_lcm_driver(NULL, &u4IndexOfLCMList);
    if (NULL == lcm_drv)
    {
        printk("%s, disphal_get_lcm_driver() returns NULL\n", __func__);
        return FALSE;
    }
    isLCMFound = TRUE;

    disp_dump_lcm_parameters(lcm_params);

    return TRUE;
}

// ---------------------------------------------------------------------------
//  DISP Driver Implementations
// ---------------------------------------------------------------------------
extern mtk_dispif_info_t dispif_info[MTKFB_MAX_DISPLAY_COUNT];

DISP_STATUS DISP_Init(UINT32 fbVA, UINT32 fbPA, BOOL isLcmInited)
{
    DISP_STATUS r = DISP_STATUS_OK;

    captureovl_task = kthread_create(_DISP_CaptureOvlKThread, NULL, "disp_captureovl_kthread");
    if (IS_ERR(captureovl_task))
    {
        DISP_LOG("DISP_InitVSYNC(): Cannot create capture ovl kthread\n");
    }
    if (gWakeupCaptureOvlThread)
        wake_up_process(captureovl_task);

    capturefb_task = kthread_create(_DISP_CaptureFBKThread, mtkfb_fbi, "disp_capturefb_kthread");
    if (IS_ERR(capturefb_task))
    {
        DISP_LOG("DISP_InitVSYNC(): Cannot create capture fb kthread\n");
    }
    wake_up_process(capturefb_task);

    if (!disp_drv_init_context()) {
        return DISP_STATUS_NOT_IMPLEMENTED;
    }

    disphal_init_ctrl_if();
    disp_path_clock_on("mtkfb");

    r = (disp_if_drv->init) ?
        (disp_if_drv->init(fbVA, fbPA, isLcmInited)) :
          DISP_STATUS_NOT_IMPLEMENTED;

    DISP_InitVSYNC((100000000/lcd_fps) + 1);//us

    {
        DAL_STATUS ret;

        /// DAL init here
        fbVA += DISP_GetFBRamSize();
        fbPA += DISP_GetFBRamSize();
        ret = DAL_Init(fbVA, fbPA);
        ASSERT(DAL_STATUS_OK == ret);
        dal_layerPA = fbPA;
        dal_layerVA = fbVA;
    }
    // check lcm status
    if(lcm_drv->check_status)
        lcm_drv->check_status();

    memset((void*)(&dispif_info[MTKFB_DISPIF_PRIMARY_LCD]), 0, sizeof(mtk_dispif_info_t));

    switch(lcm_params->type)
    {
        case LCM_TYPE_DBI:
        {
            dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayType = DISPIF_TYPE_DBI;
            dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayMode = DISPIF_MODE_COMMAND;
            dispif_info[MTKFB_DISPIF_PRIMARY_LCD].isHwVsyncAvailable = 1;
            printk("DISP Info: DBI, CMD Mode, HW Vsync enable\n");
            break;
        }
        case LCM_TYPE_DPI:
        {
            dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayType = DISPIF_TYPE_DPI;
            dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayMode = DISPIF_MODE_VIDEO;
            dispif_info[MTKFB_DISPIF_PRIMARY_LCD].isHwVsyncAvailable = 1;
            printk("DISP Info: DPI, VDO Mode, HW Vsync enable\n");
            break;
        }
        case LCM_TYPE_DSI:
        {
            dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayType = DISPIF_TYPE_DSI;
            if(lcm_params->dsi.mode == CMD_MODE)
            {
                dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayMode = DISPIF_MODE_COMMAND;
                dispif_info[MTKFB_DISPIF_PRIMARY_LCD].isHwVsyncAvailable = 1;
                printk("DISP Info: DSI, CMD Mode, HW Vsync enable\n");
            }
            else
            {
                dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayMode = DISPIF_MODE_VIDEO;
                dispif_info[MTKFB_DISPIF_PRIMARY_LCD].isHwVsyncAvailable = 1;
                printk("DISP Info: DSI, VDO Mode, HW Vsync enable\n");
            }

            break;
        }
        default:
        break;
    }


    if(disp_if_drv->get_panel_color_format())
    {
        switch(disp_if_drv->get_panel_color_format())
        {
        case PANEL_COLOR_FORMAT_RGB565:
            dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayFormat = DISPIF_FORMAT_RGB565;
        case PANEL_COLOR_FORMAT_RGB666:
            dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayFormat = DISPIF_FORMAT_RGB666;
        case PANEL_COLOR_FORMAT_RGB888:
            dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayFormat = DISPIF_FORMAT_RGB888;
            default:
                break;
        }
    }

    dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayWidth = DISP_GetScreenWidth();
    dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayHeight = DISP_GetScreenHeight();
    dispif_info[MTKFB_DISPIF_PRIMARY_LCD].vsyncFPS = lcd_fps;

    if(dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayWidth * dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayHeight <= 240*432)
    {
        dispif_info[MTKFB_DISPIF_PRIMARY_LCD].physicalHeight= dispif_info[MTKFB_DISPIF_PRIMARY_LCD].physicalWidth= 0;
    }
    else if(dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayWidth * dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayHeight <= 320*480)
    {
        dispif_info[MTKFB_DISPIF_PRIMARY_LCD].physicalHeight= dispif_info[MTKFB_DISPIF_PRIMARY_LCD].physicalWidth= 0;
    }
    else if(dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayWidth * dispif_info[MTKFB_DISPIF_PRIMARY_LCD].displayHeight <= 480*854)
    {
        dispif_info[MTKFB_DISPIF_PRIMARY_LCD].physicalHeight= dispif_info[MTKFB_DISPIF_PRIMARY_LCD].physicalWidth= 0;
    }
    else
    {
        dispif_info[MTKFB_DISPIF_PRIMARY_LCD].physicalHeight= dispif_info[MTKFB_DISPIF_PRIMARY_LCD].physicalWidth= 0;
    }

    dispif_info[MTKFB_DISPIF_PRIMARY_LCD].isConnected = 1;


    return r;
}


DISP_STATUS DISP_Deinit(void)
{
    DISP_CHECK_RET(DISP_PanelEnable(FALSE));
    DISP_CHECK_RET(DISP_PowerEnable(FALSE));

    return DISP_STATUS_OK;
}

// -----

DISP_STATUS DISP_PowerEnable(BOOL enable)
{
    DISP_STATUS ret = DISP_STATUS_OK;

    static BOOL s_enabled = TRUE;

    if (enable != s_enabled)
        s_enabled = enable;
    else
        return ret;

    if (down_interruptible(&sem_update_screen)) {
        printk("ERROR: Can't get sem_update_screen in DISP_PowerEnable()\n");
        return DISP_STATUS_ERROR;
    }

    disp_drv_init_context();

    is_engine_in_suspend_mode = enable ? FALSE : TRUE;

    if (!is_ipoh_bootup)
        needStartEngine = true;

    if (enable && lcm_drv && lcm_drv->resume_power)
    {
			lcm_drv->resume_power();
    }
    // No need for IPO-H reboot, or white screen flash will happen
    if((!is_ipoh_bootup) || (is_ipoh_bootup && (lcm_params->type==LCM_TYPE_DSI && lcm_params->dsi.mode == CMD_MODE)))
    {
    	ret = (disp_if_drv->enable_power) ?
    			(disp_if_drv->enable_power(enable)) :
    			DISP_STATUS_NOT_IMPLEMENTED;
    }

    if (enable) {
        DAL_OnDispPowerOn();
    }
    else if (lcm_drv && lcm_drv->suspend_power)
    {
        lcm_drv->suspend_power();
    }

    up(&sem_update_screen);


    return ret;
}

DISP_STATUS DISP_LCMEnable(BOOL enable)
{
	disp_drv_init_context();
	
	if (!lcm_drv->suspend || !lcm_drv->resume)
	{		
		return DISP_STATUS_NOT_IMPLEMENTED;
	}
	
	if (enable)
	{		
		lcm_drv->resume();
	}	
	else
	{
		lcm_drv->suspend();
	}

	return DISP_STATUS_OK;
}

DISP_STATUS DISP_PanelEnable(BOOL enable)
{
    static BOOL s_enabled = TRUE;
    DISP_STATUS ret = DISP_STATUS_OK;

    DISP_LOG("panel is %s\n", enable?"enabled":"disabled");

    if (down_interruptible(&sem_update_screen))
    {
        printk("ERROR: Can't get sem_update_screen in DISP_PanelEnable()\n");
        return DISP_STATUS_ERROR;
    }

    disp_drv_init_context();

    is_lcm_in_suspend_mode = enable ? FALSE : TRUE;

    if (is_ipoh_bootup)
        s_enabled = TRUE;

    if (!lcm_drv->suspend || !lcm_drv->resume)
    {
        ret = DISP_STATUS_NOT_IMPLEMENTED;
        goto End;
    }

    if (enable && !s_enabled)
    {
        s_enabled = TRUE;
        disphal_panel_enable(lcm_drv, &LcmCmdMutex, TRUE);
    }
    else if (!enable && s_enabled)
    {
        s_enabled = FALSE;
        disphal_panel_enable(lcm_drv, &LcmCmdMutex, FALSE);
    }

End:
    up(&sem_update_screen);

    return ret;
}

DISP_STATUS DISP_SetBacklight(UINT32 level)
{
    DISP_STATUS ret = DISP_STATUS_OK;

    if (down_interruptible(&sem_update_screen)) {
        printk("ERROR: Can't get sem_update_screen in DISP_SetBacklight()\n");
        return DISP_STATUS_ERROR;
    }

    disp_drv_init_context();

    disphal_wait_not_busy();

    if (!lcm_drv->set_backlight) {
        ret = DISP_STATUS_NOT_IMPLEMENTED;
        goto End;
    }

    disphal_set_backlight(lcm_drv, &LcmCmdMutex, level);
End:

    up(&sem_update_screen);

    return ret;
}

DISP_STATUS DISP_SetBacklight_mode(UINT32 mode)
{
    DISP_STATUS ret = DISP_STATUS_OK;

    if (down_interruptible(&sem_update_screen)) {
        printk("ERROR: Can't get sem_update_screen in DISP_SetBacklight_mode()\n");
        return DISP_STATUS_ERROR;
    }

    disp_drv_init_context();

    disphal_wait_not_busy();

    if (!lcm_drv->set_backlight) {
        ret = DISP_STATUS_NOT_IMPLEMENTED;
        goto End;
    }

    disphal_set_backlight_mode(lcm_drv, &LcmCmdMutex, mode);
End:

    up(&sem_update_screen);

    return ret;

}

DISP_STATUS DISP_SetPWM(UINT32 divider)
{
    DISP_STATUS ret = DISP_STATUS_OK;

    if (down_interruptible(&sem_update_screen)) {
        printk("ERROR: Can't get sem_update_screen in DISP_SetPWM()\n");
        return DISP_STATUS_ERROR;
    }

    disp_drv_init_context();

    disphal_wait_not_busy();

    if (!lcm_drv->set_pwm) {
        ret = DISP_STATUS_NOT_IMPLEMENTED;
        goto End;
    }

    disphal_set_pwm(lcm_drv, &LcmCmdMutex, divider);
End:

    up(&sem_update_screen);

    return ret;
}

DISP_STATUS DISP_GetPWM(UINT32 divider, unsigned int *freq)
{
    DISP_STATUS ret = DISP_STATUS_OK;

    disp_drv_init_context();

    if (!lcm_drv->get_pwm) {
        ret = DISP_STATUS_NOT_IMPLEMENTED;
        goto End;
    }

    disphal_get_pwm(lcm_drv, &LcmCmdMutex, divider, freq);
End:
    return ret;
}

#if defined(MTK_LCD_HW_3D_SUPPORT)
static BOOL is3denabled = FALSE;
static BOOL ispwmenabled = FALSE;
static BOOL ismodechanged = FALSE;

static BOOL gCurrentMode = FALSE;
static BOOL gUsingMode = FALSE;


DISP_STATUS DISP_Set3DPWM(BOOL enable, BOOL landscape)
{
    unsigned int temp_reg;

    if (enable && (!ispwmenabled || ismodechanged))
    {
        struct  pwm_easy_config pwm_setting;
        // Set GPIO66, GPIO67, GPIO68 to PWM2, PWM1, PWM3
        mt_set_gpio_mode(GPIO187, GPIO_MODE_GPIO);
        mt_set_gpio_out(GPIO187, GPIO_OUT_ONE);

        mt_set_gpio_mode(GPIO66, GPIO_MODE_07);
        mt_set_gpio_mode(GPIO67, GPIO_MODE_07);
        mt_set_gpio_mode(GPIO68, GPIO_MODE_07);

        pwm_setting.clk_src = PWM_CLK_OLD_MODE_32K;
        pwm_setting.duty = 50;
        pwm_setting.clk_div = CLK_DIV1;
        pwm_setting.duration = 533;

        pwm_setting.pwm_no = PWM1;
        pwm_set_easy_config(&pwm_setting);
        pwm_setting.pwm_no = PWM2;
        pwm_set_easy_config(&pwm_setting);
        pwm_setting.pwm_no = PWM3;
        pwm_set_easy_config(&pwm_setting);

        temp_reg=INREG32(INFRA_SYS_CFG_BASE+0x700);

        temp_reg&=~0xF000000;

        if(landscape)
            temp_reg|=0x9000000;
        else
            temp_reg|=0x3000000;

        OUTREG32((INFRA_SYS_CFG_BASE+0x700), temp_reg);

        ispwmenabled = TRUE;

        DISP_LOG("3D PWM is enabled. landscape:%d ! \n", landscape);
    }
    else if (!enable && ispwmenabled)
    {
        mt_set_gpio_mode(GPIO187, GPIO_MODE_GPIO);
        mt_set_gpio_out(GPIO187, GPIO_OUT_ZERO);

        mt_set_gpio_mode(GPIO66, GPIO_MODE_GPIO);
        mt_set_gpio_mode(GPIO67, GPIO_MODE_GPIO);
        mt_set_gpio_mode(GPIO68, GPIO_MODE_GPIO);

        ispwmenabled = FALSE;

        DISP_LOG("3D PWM is disabled ! \n");
    }
    return DISP_STATUS_OK;
}


BOOL DISP_Is3DEnabled(void)
{
    is3denabled = FALSE;

    return is3denabled;
}

BOOL DISP_is3DLandscapeMode(void)
{
    // OVL driver not yet export this function
    gCurrentMode = FALSE;

    if (gCurrentMode != gUsingMode)
        ismodechanged = TRUE;
    else
        ismodechanged = FALSE;

    gUsingMode = gCurrentMode;

    return FALSE;
}
#endif

// -----

DISP_STATUS DISP_SetFrameBufferAddr(UINT32 fbPhysAddr)
{
	cached_layer_config[FB_LAYER].addr = fbPhysAddr;
    cached_layer_config[FB_LAYER].isDirty = true;
    return DISP_STATUS_OK;
}

// -----

static BOOL is_overlaying = FALSE;

DISP_STATUS DISP_EnterOverlayMode(void)
{
    DISP_FUNC();
    if (is_overlaying) {
        return DISP_STATUS_ALREADY_SET;
    } else {
        is_overlaying = TRUE;
    }

    return DISP_STATUS_OK;
}


DISP_STATUS DISP_LeaveOverlayMode(void)
{
    DISP_FUNC();
    if (!is_overlaying) {
        return DISP_STATUS_ALREADY_SET;
    } else {
        is_overlaying = FALSE;
    }

    return DISP_STATUS_OK;
}


// -----

DISP_STATUS DISP_EnableDirectLinkMode(UINT32 layer)
{
    ///since MT6573 we do not support DC mode
    return DISP_STATUS_OK;
}


DISP_STATUS DISP_DisableDirectLinkMode(UINT32 layer)
{
    ///since MT6573 we do not support DC mode
    return DISP_STATUS_OK;
}

// -----

extern int MT6516IDP_EnableDirectLink(void);

DISP_STATUS DISP_UpdateScreen(UINT32 x, UINT32 y, UINT32 width, UINT32 height)
{
    unsigned int is_video_mode;

//	return DISP_STATUS_OK;
    DISP_LOG("update screen, (%d,%d),(%d,%d)\n", x, y, width, height);

    if ((lcm_params->type==LCM_TYPE_DPI) ||
        ((lcm_params->type==LCM_TYPE_DSI) && (lcm_params->dsi.mode != CMD_MODE)))
        is_video_mode = 1;
    else
        is_video_mode = 0;

    if (down_interruptible(&sem_update_screen)) {
        printk("ERROR: Can't get sem_update_screen in DISP_UpdateScreen()\n");
        return DISP_STATUS_ERROR;
    }
#if defined(MTK_LCD_HW_3D_SUPPORT)
    DISP_Set3DPWM( DISP_Is3DEnabled(), DISP_is3DLandscapeMode() );
#endif
    // if LCM is powered down, LCD would never recieve the TE signal
    //
    if (is_lcm_in_suspend_mode || is_engine_in_suspend_mode) goto End;
    if (is_video_mode && is_video_mode_running)
        needStartEngine = false;
    if (needStartEngine)
    {
        disphal_update_screen(lcm_drv, &LcmCmdMutex, x, y, width, height);
    }
    if (-1 != direct_link_layer) {
        //MT6516IDP_EnableDirectLink();     // FIXME
    }
    else
    {
        if (needStartEngine)
            disp_if_drv->update_screen(FALSE);
    }
    needStartEngine = false;
End:
    up(&sem_update_screen);

    return DISP_STATUS_OK;
}

DISP_STATUS DISP_WaitForLCDNotBusy(void)
{
    disphal_wait_not_busy();
    return DISP_STATUS_OK;
}

DISP_STATUS _DISP_ConfigUpdateScreen(UINT32 x, UINT32 y, UINT32 width, UINT32 height)
{
#if defined(MTK_LCD_HW_3D_SUPPORT)
    DISP_Set3DPWM( DISP_Is3DEnabled(), DISP_is3DLandscapeMode());
#endif
    // if LCM is powered down, LCD would never recieve the TE signal
    //
    if (is_lcm_in_suspend_mode || is_engine_in_suspend_mode) return DISP_STATUS_ERROR;
    disphal_update_screen(lcm_drv, &LcmCmdMutex, x, y, width, height);
    disp_if_drv->update_screen(TRUE);

    return DISP_STATUS_OK;
}

#define DISP_CB_MAXCNT 2
typedef struct {
    DISP_EXTRA_CHECKUPDATE_PTR checkupdate_cb[DISP_CB_MAXCNT];
    DISP_EXTRA_CONFIG_PTR config_cb[DISP_CB_MAXCNT];
} CONFIG_CB_ARRAY;
CONFIG_CB_ARRAY g_CB_Array = { {NULL , NULL},{NULL , NULL}};//if DISP_CB_MAXCNT == 2
DEFINE_MUTEX(UpdateRegMutex);
void GetUpdateMutex(void)
{
    mutex_lock(&UpdateRegMutex);
}
int TryGetUpdateMutex(void)
{
    return mutex_trylock(&UpdateRegMutex);
}
void ReleaseUpdateMutex(void)
{
    mutex_unlock(&UpdateRegMutex);
}

int DISP_RegisterExTriggerSource(DISP_EXTRA_CHECKUPDATE_PTR pCheckUpdateFunc , DISP_EXTRA_CONFIG_PTR pConfFunc)
{
    int index = 0;
    int hit = 0;
    if((NULL == pCheckUpdateFunc) || (NULL == pConfFunc))
    {
        printk("Warnning! [Func]%s register NULL function : %p,%p\n", __func__ , pCheckUpdateFunc , pConfFunc);
        return -1;
    }

    GetUpdateMutex();

    for(index = 0 ; index < DISP_CB_MAXCNT ; index += 1)
    {
        if(NULL == g_CB_Array.checkupdate_cb[index])
        {
            g_CB_Array.checkupdate_cb[index] = pCheckUpdateFunc;
            g_CB_Array.config_cb[index] = pConfFunc;
            hit = 1;
            break;
        }
    }

    ReleaseUpdateMutex();

    return (hit ? index : (-1));
}

void DISP_UnRegisterExTriggerSource(int u4ID)
{
    if(DISP_CB_MAXCNT < (u4ID+1))
    {
        printk("Warnning! [Func]%s unregister a never registered function : %d\n", __func__ , u4ID);
        return;
    }

    GetUpdateMutex();

    g_CB_Array.checkupdate_cb[u4ID] = NULL;
    g_CB_Array.config_cb[u4ID] = NULL;

    ReleaseUpdateMutex();
}
#if defined(MTK_HDMI_SUPPORT)
extern bool is_hdmi_active(void);
#endif



static int _DISP_CaptureFBKThread(void *data)
{
    struct fb_info* pInfo = (struct fb_info*) data;
    MMP_MetaDataBitmap_t Bitmap;

    while(1)
    {
        wait_event_interruptible(gCaptureFBWQ, gCaptureFBEnable);
        Bitmap.data1 = pInfo->var.yoffset;
        Bitmap.width = DISP_GetScreenWidth();
        Bitmap.height = DISP_GetScreenHeight() * 2;
        Bitmap.bpp = pInfo->var.bits_per_pixel;

        switch (pInfo->var.bits_per_pixel)
        {
            case 16:
                Bitmap.format = MMProfileBitmapRGB565;
                break;
            case 32:
                 Bitmap.format = MMProfileBitmapBGRA8888;
                 break;
            default:
                Bitmap.format = MMProfileBitmapRGB565;
                Bitmap.bpp = 16;
                break;
        }

        Bitmap.start_pos = 0;
        Bitmap.pitch = pInfo->fix.line_length;
        if (Bitmap.pitch == 0)
        {
            Bitmap.pitch = ALIGN_TO(Bitmap.width, 1) * Bitmap.bpp / 8;
        }
        Bitmap.data_size = Bitmap.pitch * Bitmap.height;
        Bitmap.down_sample_x = gCaptureFBDownX;
        Bitmap.down_sample_y = gCaptureFBDownY;
        Bitmap.pData = ((struct mtkfb_device*)pInfo->par)->fb_va_base;
        MMProfileLogMetaBitmap(MTKFB_MMP_Events.FBDump, MMProfileFlagPulse, &Bitmap);
        msleep(gCaptureFBPeriod);
    }
    return 0;
}

static int _DISP_CaptureOvlKThread(void *data)
{
    unsigned int index = 0;
    unsigned int mva[2];
    void* va[2];
    unsigned int buf_size;
    unsigned int init = 0;
    unsigned int enabled = 0;
    int wait_ret = 0;
    MMP_MetaDataBitmap_t Bitmap;
    buf_size = DISP_GetScreenWidth() * DISP_GetScreenHeight() * 4;

    while(1)
    {
    	wait_ret = wait_event_interruptible(reg_update_wq, gWakeupCaptureOvlThread);
        DISP_LOG("[WaitQ] wait_event_interruptible() ret = %d, %d\n", wait_ret, __LINE__);
        gWakeupCaptureOvlThread = 0;

        if (init == 0)
        {
            va[0] = vmalloc(buf_size);
            va[1] = vmalloc(buf_size);
            memset(va[0], 0, buf_size);
            memset(va[1], 0, buf_size);
            disphal_map_overlay_out_buffer((unsigned int)va[0], buf_size, &(mva[0]));
            disphal_map_overlay_out_buffer((unsigned int)va[1], buf_size, &(mva[1]));
            disphal_init_overlay_to_memory();
            init = 1;
        }
        if (!gCaptureOvlThreadEnable)
        {
            if (enabled)
            {
                DISP_Config_Overlay_to_Memory(mva[index], 0);
                enabled = 0;
            }
            continue;
        }
        DISP_Config_Overlay_to_Memory(mva[index], 1);
        enabled = 1;

        disphal_sync_overlay_out_buffer((unsigned int)va[index], DISP_GetScreenHeight()*DISP_GetScreenWidth()*24/8);

        Bitmap.data1 = index;
        Bitmap.data2 = mva[index];
        Bitmap.width = DISP_GetScreenWidth();
        Bitmap.height = DISP_GetScreenHeight();
        Bitmap.format = MMProfileBitmapBGR888;
        Bitmap.start_pos = 0;
        Bitmap.bpp = 24;
        Bitmap.pitch = DISP_GetScreenWidth()*3;
        Bitmap.data_size = Bitmap.pitch * Bitmap.height;
        Bitmap.down_sample_x = gCaptureOvlDownX;
        Bitmap.down_sample_y = gCaptureOvlDownY;
        Bitmap.pData = va[index];
        MMProfileLogMetaBitmap(MTKFB_MMP_Events.OvlDump, MMProfileFlagPulse, &Bitmap);
        index = 1 - index;
    }
    return 0;
}

static void _DISP_DumpLayer(OVL_CONFIG_STRUCT* pLayer)
{
    if (gCaptureLayerEnable && pLayer->layer_en)
    {
        MMP_MetaDataBitmap_t Bitmap;
        Bitmap.data1 = pLayer->vaddr;
        Bitmap.width = pLayer->dst_w;
        Bitmap.height = pLayer->dst_h;
        switch (pLayer->fmt)
        {
        case eRGB565: Bitmap.format = MMProfileBitmapRGB565; Bitmap.bpp = 16; break;
        case eRGB888: Bitmap.format = MMProfileBitmapBGR888; Bitmap.bpp = 24; break;
        case eARGB8888:
        case ePARGB8888:
          Bitmap.format = MMProfileBitmapBGRA8888; Bitmap.bpp = 32; break;
        case eBGR888: Bitmap.format = MMProfileBitmapRGB888; Bitmap.bpp = 24; break;
        case eABGR8888:
        case ePABGR8888:
          Bitmap.format = MMProfileBitmapRGBA8888; Bitmap.bpp = 32; break;
        default: printk("error: _DISP_DumpLayer(), unknow format=%d \n", pLayer->fmt); return;
        }
        Bitmap.start_pos = 0;
        Bitmap.pitch = pLayer->src_pitch;
        Bitmap.data_size = Bitmap.pitch * Bitmap.height;
        Bitmap.down_sample_x = gCaptureLayerDownX;
        Bitmap.down_sample_y = gCaptureLayerDownY;
        if (pLayer->addr >= fb_pa &&
            pLayer->addr < (fb_pa+DISP_GetVRamSize()))
        {
            Bitmap.pData = (void*) pLayer->vaddr;
            MMProfileLogMetaBitmap(MTKFB_MMP_Events.Layer[pLayer->layer], MMProfileFlagPulse, &Bitmap);
        }
        else
        {
            disphal_dma_map_kernel(pLayer->addr, Bitmap.data_size, (unsigned int*)&Bitmap.pData, &Bitmap.data_size);
            MMProfileLogMetaBitmap(MTKFB_MMP_Events.Layer[pLayer->layer], MMProfileFlagPulse, &Bitmap);
            disphal_dma_unmap_kernel(pLayer->addr, Bitmap.data_size, (unsigned int)Bitmap.pData);
        }
    }
}

static void _DISP_VSyncCallback(void* pParam);

void DISP_StartConfigUpdate(void)
{
    if(((LCM_TYPE_DSI == lcm_params->type) && (CMD_MODE == lcm_params->dsi.mode)) || (LCM_TYPE_DBI == lcm_params->type))
    {
        config_update_task_wakeup = 1;
        wake_up_interruptible(&config_update_wq);
    }
}

static int _DISP_ESD_Check(int* dirty)
{
    unsigned int esd_check_count;
    if (need_esd_check && (!is_early_suspended))
    {
        esd_check_count = 0;
        disp_running = 1;
        MMProfileLog(MTKFB_MMP_Events.EsdCheck, MMProfileFlagStart);
        while (esd_check_count < LCM_ESD_CHECK_MAX_COUNT)
        {
            if(DISP_EsdCheck())
			{
			    MMProfileLogEx(MTKFB_MMP_Events.EsdCheck, MMProfileFlagPulse, 0, 0);
				DISP_EsdRecover();
			}
			else
			{
			    break;
			}
			esd_check_count++;
        }
        if (esd_check_count >= LCM_ESD_CHECK_MAX_COUNT)
        {
			MMProfileLogEx(MTKFB_MMP_Events.EsdCheck, MMProfileFlagPulse, 2, 0);
			esd_kthread_pause = TRUE;
        }
        if (esd_check_count)
        {
                disphal_update_screen(lcm_drv, &LcmCmdMutex, 0, 0, DISP_GetScreenWidth(), DISP_GetScreenHeight());
                *dirty = 1;
        }
        need_esd_check = 0;
        wake_up_interruptible(&esd_check_wq);
        MMProfileLog(MTKFB_MMP_Events.EsdCheck, MMProfileFlagEnd);
        return 1;
    }
    return 0;
}


static int _DISP_ConfigUpdateKThread(void *data)
{
    int i;
    int dirty = 0;
    int overlay_dirty = 0;
    int aal_dirty = 0;
    int index = 0;
    struct disp_path_config_mem_out_struct mem_out_config;
    struct sched_param param = { .sched_priority = RTPM_PRIO_SCRN_UPDATE };
    sched_setscheduler(current, SCHED_RR, &param);
    while(1)
    {
        wait_event_interruptible(config_update_wq, config_update_task_wakeup);
        config_update_task_wakeup = 0;
        //MMProfileLog(MTKFB_MMP_Events.UpdateConfig, MMProfileFlagStart);
        dirty = 0;
        overlay_dirty = 0;
        aal_dirty = 0;

        if (down_interruptible(&sem_early_suspend)) {
            printk("[FB Driver] can't get semaphore in mtkfb_early_suspend()\n");
            continue;
        }
        //MMProfileLogEx(MTKFB_MMP_Events.EarlySuspend, MMProfileFlagStart, 1, 0);

		if (!((lcm_params->type == LCM_TYPE_DSI) && (lcm_params->dsi.mode != CMD_MODE)))
		{
			if (_DISP_ESD_Check(&dirty))
			{
				disp_running = 0;
			}
        }

        if (!is_early_suspended)
        {
            if (mutex_trylock(&OverlaySettingMutex))
            {
                overlay_dirty = atomic_read(&OverlaySettingDirtyFlag);
                if (overlay_dirty)
                {
                    layer_config_index = 1 - layer_config_index;
                    captured_layer_config = _layer_config[layer_config_index];
                    DISP_LOG("========= cached --> captured ===========\n");
                    memcpy(captured_layer_config, cached_layer_config, sizeof(OVL_CONFIG_STRUCT)*DDP_OVL_LAYER_MUN);
                    for (i=0; i<DDP_OVL_LAYER_MUN; i++)
                        cached_layer_config[i].isDirty = false;
                    MMProfileLogStructure(MTKFB_MMP_Events.ConfigOVL, MMProfileFlagPulse, captured_layer_config, struct DBG_OVL_CONFIGS);
                    atomic_set(&OverlaySettingDirtyFlag, 0);
                    PanDispSettingDirty = 0;
                    dirty = 1;
                }
                mutex_unlock(&OverlaySettingMutex);
            }
            aal_dirty = overlay_dirty;//overlay refresh means AAL also needs refresh
            //GetUpdateMutex();
            if(1 == TryGetUpdateMutex())
            {
            for(index = 0 ; index < DISP_CB_MAXCNT ; index += 1)
            {
                if((NULL != g_CB_Array.checkupdate_cb[index]) && g_CB_Array.checkupdate_cb[index](overlay_dirty))
                {
                    dirty = 1;
                    aal_dirty = 1;
                    break;
                }
            }
            ReleaseUpdateMutex();
            }

            if (mutex_trylock(&MemOutSettingMutex))
            {
                if (MemOutConfig.dirty)
                {
                    memcpy(&mem_out_config, &MemOutConfig, sizeof(MemOutConfig));
                    MemOutConfig.dirty = 0;
                    dirty = 1;
                }
                else
                    mem_out_config.dirty = 0;
                mutex_unlock(&MemOutSettingMutex);
            }
        }
        if(((LCM_TYPE_DSI == lcm_params->type) && (CMD_MODE == lcm_params->dsi.mode)) || (LCM_TYPE_DBI == lcm_params->type))
        {
            _DISP_VSyncCallback(NULL);
        }

        if (dirty)
        {
            // Apply configuration here.
            disp_running = 1;
            disp_path_get_mutex();
            if (overlay_dirty)
            {
                MMProfileLog(MTKFB_MMP_Events.ConfigOVL, MMProfileFlagStart);
                for (i=0; i<DDP_OVL_LAYER_MUN; i++)
                {
                    if (captured_layer_config[i].isDirty)
                    {
                        _DISP_DumpLayer(&captured_layer_config[i]);
                        disp_path_config_layer(&captured_layer_config[i]);
                        captured_layer_config[i].isDirty = false;
                    }
                }
                if ((lcm_params->type == LCM_TYPE_DBI) ||
                    ((lcm_params->type == LCM_TYPE_DSI) && (lcm_params->dsi.mode == CMD_MODE)))
                {
                    if ((PanDispSettingPending==1) && (PanDispSettingDirty==0))
                    {
                        PanDispSettingApplied = 1;
                        PanDispSettingPending = 0;
                    }
                    atomic_set(&OverlaySettingApplied, 1);
                    wake_up(&reg_update_wq);
                }
                MMProfileLog(MTKFB_MMP_Events.ConfigOVL, MMProfileFlagEnd);


            }

            // Apply AAL config here.
            if (aal_dirty)
            {
                MMProfileLog(MTKFB_MMP_Events.ConfigAAL, MMProfileFlagStart);

                //GetUpdateMutex();
                if(1 == TryGetUpdateMutex())
                {
                    for(index = 0 ; index < DISP_CB_MAXCNT ; index += 1)
                {
                    if((NULL != g_CB_Array.checkupdate_cb[index]) && g_CB_Array.checkupdate_cb[index](overlay_dirty))
                    {
                        g_CB_Array.config_cb[index](overlay_dirty);
                    }
                }
                ReleaseUpdateMutex();
                }
                MMProfileLog(MTKFB_MMP_Events.ConfigAAL, MMProfileFlagEnd);
            }

            // Apply memory out config here.
            if (mem_out_config.dirty)
            {
                MMProfileLogEx(MTKFB_MMP_Events.ConfigMemOut, MMProfileFlagStart, mem_out_config.enable, mem_out_config.dstAddr);
                disp_path_config_mem_out(&mem_out_config);
                MMProfileLogEx(MTKFB_MMP_Events.ConfigMemOut, MMProfileFlagEnd, 0, 0);
            }
#if defined(MTK_HDMI_SUPPORT)
            if(/*is_hdmi_active() && */!DISP_IsVideoMode())
            {
                MMProfileLogEx(MTKFB_MMP_Events.ConfigMemOut, MMProfileFlagStart, mem_out_config.enable, mem_out_config.dstAddr);
                memcpy(&mem_out_config, &MemOutConfig, sizeof(MemOutConfig));
                disp_path_config_mem_out(&mem_out_config);
                MMProfileLogEx(MTKFB_MMP_Events.ConfigMemOut, MMProfileFlagEnd, 1, 0);
            }
#endif
            // Trigger interface engine for cmd mode.
            if ((lcm_params->type == LCM_TYPE_DBI) ||
                ((lcm_params->type == LCM_TYPE_DSI) && (lcm_params->dsi.mode == CMD_MODE)))
            {
                DISP_STATUS ret = _DISP_ConfigUpdateScreen(0, 0, DISP_GetScreenWidth(), DISP_GetScreenHeight());
                if ((ret != DISP_STATUS_OK) && (is_early_suspended == 0))
                    hrtimer_start(&cmd_mode_update_timer, cmd_mode_update_timer_period, HRTIMER_MODE_REL);
            }
            disp_path_release_mutex();
        }
        else
        {
            if ((lcm_params->type == LCM_TYPE_DBI) ||
                ((lcm_params->type == LCM_TYPE_DSI) && (lcm_params->dsi.mode == CMD_MODE)))
            {
                // Start update timer.
                if (!is_early_suspended)
                {
                	  if (is_immediateupdate)
                	  	hrtimer_start(&cmd_mode_update_timer, ktime_set(0 , 5000000), HRTIMER_MODE_REL);
                	  else
                	  	hrtimer_start(&cmd_mode_update_timer, cmd_mode_update_timer_period, HRTIMER_MODE_REL);
                }
            }
        }
		if ((lcm_params->type == LCM_TYPE_DSI) && (lcm_params->dsi.mode != CMD_MODE))
		{
			if (_DISP_ESD_Check(&dirty))
			{
				disp_running = 1;
			}
		}

        //MMProfileLog(MTKFB_MMP_Events.UpdateConfig, MMProfileFlagEnd);
        //MMProfileLogEx(MTKFB_MMP_Events.EarlySuspend, MMProfileFlagEnd, 1, 0);
        up(&sem_early_suspend);
        if (kthread_should_stop())
            break;
    }

    return 0;
}

static void _DISP_HWDoneCallback(void* pParam)
{
    MMProfileLogEx(MTKFB_MMP_Events.DispDone, MMProfileFlagPulse, is_early_suspended, 0);
    disp_running = 0;
    wake_up_interruptible(&disp_done_wq);
}

static void _DISP_VSyncCallback(void* pParam)
{
    MMProfileLog(MTKFB_MMP_Events.VSync, MMProfileFlagPulse);
    vsync_wq_flag = 1;
    wake_up_interruptible(&vsync_wq);
}

static void _DISP_RegUpdateCallback(void* pParam)
{
    MMProfileLog(MTKFB_MMP_Events.RegUpdate, MMProfileFlagPulse);
    DISP_LOG("========= captured --> realtime ===========\n");
    realtime_layer_config = captured_layer_config;
    if ((lcm_params->type == LCM_TYPE_DPI) ||
        ((lcm_params->type == LCM_TYPE_DSI) && (lcm_params->dsi.mode != CMD_MODE)))
    {
        if ((PanDispSettingPending==1) && (PanDispSettingDirty==0))
        {
            PanDispSettingApplied = 1;
            PanDispSettingPending = 0;
        }
        atomic_set(&OverlaySettingApplied, 1);
    }
    gWakeupCaptureOvlThread = 1;
    wake_up(&reg_update_wq);
}

static void _DISP_TargetLineCallback(void* pParam)
{
    //tasklet_hi_schedule(&ConfigUpdateTask);
    MMProfileLogEx(MTKFB_MMP_Events.UpdateConfig, MMProfileFlagPulse, 0, 0);
    config_update_task_wakeup = 1;
    wake_up_interruptible(&config_update_wq);
}

static void _DISP_CmdDoneCallback(void* pParam)
{
    //tasklet_hi_schedule(&ConfigUpdateTask);
    MMProfileLogEx(MTKFB_MMP_Events.UpdateConfig, MMProfileFlagPulse, 1, 0);
    config_update_task_wakeup = 1;
    wake_up_interruptible(&config_update_wq);
    _DISP_HWDoneCallback(NULL);
}

static enum hrtimer_restart _DISP_CmdModeTimer_handler(struct hrtimer *timer)
{
    MMProfileLogEx(MTKFB_MMP_Events.UpdateConfig, MMProfileFlagPulse, 2, 0);
    config_update_task_wakeup = 1;
    wake_up_interruptible(&config_update_wq);
    return HRTIMER_NORESTART;
}

void DISP_InitVSYNC(unsigned int vsync_interval)
{
    init_waitqueue_head(&config_update_wq);
    init_waitqueue_head(&vsync_wq);
    config_update_task = kthread_create(
        _DISP_ConfigUpdateKThread, NULL, "disp_config_update_kthread");

    if (IS_ERR(config_update_task))
    {
        DISP_LOG("DISP_InitVSYNC(): Cannot create config update kthread\n");
        return;
    }
    wake_up_process(config_update_task);

    disphal_register_event("DISP_CmdDone", _DISP_CmdDoneCallback);
    disphal_register_event("DISP_RegUpdate", _DISP_RegUpdateCallback);
    disphal_register_event("DISP_VSync", _DISP_VSyncCallback);
    disphal_register_event("DISP_TargetLine", _DISP_TargetLineCallback);
    disphal_register_event("DISP_HWDone", _DISP_HWDoneCallback);
    if ((LCM_TYPE_DBI == lcm_params->type) ||
        ((LCM_TYPE_DSI == lcm_params->type) && (CMD_MODE == lcm_params->dsi.mode)))
    {
        cmd_mode_update_timer_period = ktime_set(0 , vsync_interval*1000);
        printk("[MTKFB] vsync timer_period=%d \n", vsync_interval);
        hrtimer_init(&cmd_mode_update_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        cmd_mode_update_timer.function = _DISP_CmdModeTimer_handler;
        config_update_task_wakeup = 1;
        wake_up_interruptible(&config_update_wq);
    }

}

void DISP_WaitVSYNC(void)
{
	MMProfileLog(MTKFB_MMP_Events.WaitVSync, MMProfileFlagStart);
	vsync_wq_flag = 0;
	if (wait_event_interruptible_timeout(vsync_wq, vsync_wq_flag, HZ/10) == 0)
	{
		printk("[DISP] Wait VSync timeout. early_suspend=%d\n", is_early_suspended);
		//FIXME: Error handling for DDP hw error
		if (!is_early_suspended) {
			printk("[DISP] wake up! \n");
			_DISP_RegUpdateCallback(NULL);
			wake_up_interruptible(&config_update_wq);
		}
	}
	MMProfileLog(MTKFB_MMP_Events.WaitVSync, MMProfileFlagEnd);
}

DISP_STATUS DISP_PauseVsync(BOOL enable)
{
    if((LCM_TYPE_DBI == lcm_params->type) ||
        (LCM_TYPE_DSI == lcm_params->type && lcm_params->dsi.mode == CMD_MODE))
    {
        if (enable)
            hrtimer_cancel(&cmd_mode_update_timer);
    }
    else if((LCM_TYPE_DPI == lcm_params->type) ||
        (LCM_TYPE_DSI == lcm_params->type && lcm_params->dsi.mode != CMD_MODE))
    {
    }
    else
    {
        DISP_LOG("DISP_PauseVSYNC():unknown interface\n");
    }
    return DISP_STATUS_OK;
}

DISP_STATUS DISP_ConfigDither(int lrs, int lgs, int lbs, int dbr, int dbg, int dbb)
{
    DISP_LOG("DISP_ConfigDither lrs:0x%x, lgs:0x%x, lbs:0x%x, dbr:0x%x, dbg:0x%x, dbb:0x%x \n", lrs, lgs, lbs, dbr, dbg, dbb);

    return DISP_STATUS_OK;
}


// ---------------------------------------------------------------------------
//  Retrieve Information
// ---------------------------------------------------------------------------

BOOL DISP_IsVideoMode(void)
{
    disp_drv_init_context();
    if(lcm_params)
        return lcm_params->type==LCM_TYPE_DPI || (lcm_params->type==LCM_TYPE_DSI && lcm_params->dsi.mode != CMD_MODE);
    else
    {
        printk("WARNING!! DISP_IsVideoMode is called before display driver inited!\n");
        return 0;
    }
}

UINT32 DISP_GetScreenWidth(void)
{
    disp_drv_init_context();
    if(lcm_params)
        return lcm_params->width;
    else
    {
        printk("WARNING!! get screen width before display driver inited!\n");
        return 0;
    }
}
EXPORT_SYMBOL(DISP_GetScreenWidth);

UINT32 DISP_GetScreenHeight(void)
{
    disp_drv_init_context();
    if(lcm_params)
        return lcm_params->height;
    else
    {
        printk("WARNING!! get screen height before display driver inited!\n");
        return 0;
    }
}
UINT32 DISP_GetActiveHeight(void)
{
    disp_drv_init_context();
    if(lcm_params)
    {
        printk("[wwy]lcm_parms->active_height = %d\n",lcm_params->active_height);
        return lcm_params->active_height;
    }
    else
    {
        printk("WARNING!! get active_height before display driver inited!\n");
        return 0;
    }
}

UINT32 DISP_GetActiveWidth(void)
{
    disp_drv_init_context();
    if(lcm_params)
    {
        printk("[wwy]lcm_parms->active_width = %d\n",lcm_params->active_width);
        return lcm_params->active_width;
    }
    else
    {
        printk("WARNING!! get active_width before display driver inited!\n");
        return 0;
    }
}

DISP_STATUS DISP_SetScreenBpp(UINT32 bpp)
{
    ASSERT(bpp != 0);

    if( bpp != 16   &&          \
        bpp != 24   &&          \
        bpp != 32   &&          \
        1 )
    {
        DISP_LOG("DISP_SetScreenBpp error, not support %d bpp\n", bpp);
        return DISP_STATUS_ERROR;
    }

    disp_fb_bpp = bpp;
    DISP_LOG("DISP_SetScreenBpp %d bpp\n", bpp);

    return DISP_STATUS_OK;
}

UINT32 DISP_GetScreenBpp(void)
{
    return disp_fb_bpp;
}

DISP_STATUS DISP_SetPages(UINT32 pages)
{
    ASSERT(pages != 0);

    disp_fb_pages = pages;
    DISP_LOG("DISP_SetPages %d pages\n", pages);

    return DISP_STATUS_OK;
}

UINT32 DISP_GetPages(void)
{
    return disp_fb_pages;   // Double Buffers
}


BOOL DISP_IsDirectLinkMode(void)
{
    return (-1 != direct_link_layer) ? TRUE : FALSE;
}


BOOL DISP_IsInOverlayMode(void)
{
    return is_overlaying;
}

UINT32 DISP_GetFBRamSize(void)
{
    return ALIGN_TO(DISP_GetScreenWidth(), disphal_get_fb_alignment()) *
           ALIGN_TO(DISP_GetScreenHeight(), disphal_get_fb_alignment()) *
           ((DISP_GetScreenBpp() + 7) >> 3) *
           DISP_GetPages();
}


UINT32 DISP_GetVRamSize(void)
{
    // Use a local static variable to cache the calculated vram size
    //
    static UINT32 vramSize = 0;

    if (0 == vramSize)
    {
        disp_drv_init_context();

        ///get framebuffer size
        vramSize = DISP_GetFBRamSize();

        ///get DXI working buffer size
        vramSize += disp_if_drv->get_working_buffer_size();

        // get assertion layer buffer size
        vramSize += DAL_GetLayerSize();

        // Align vramSize to 1MB
        //
        vramSize = ALIGN_TO_POW_OF_2(vramSize, 0x100000);

        DISP_LOG("DISP_GetVRamSize: %u bytes\n", vramSize);
    }

    return vramSize;
}

UINT32 DISP_GetVRamSizeBoot(char *cmdline)
{
    static UINT32 vramSize = 0;

    if(vramSize)
    {
        return vramSize;
    }

    disp_get_lcm_name_boot(cmdline);

    // if can't get the lcm type from uboot, we will return 0x800000 for a safe value
    if(disp_if_drv)
        vramSize = DISP_GetVRamSize();
    else
    {
        printk("%s, can't get lcm type, reserved memory size will be set as 0x800000\n", __func__);
        return 0x1400000;
    }
    // Align vramSize to 1MB
    //
    vramSize = ALIGN_TO_POW_OF_2(vramSize, 0x100000);

    printk("DISP_GetVRamSizeBoot: %u bytes[%dMB]\n", vramSize, (vramSize>>20));

    return vramSize;
}

PANEL_COLOR_FORMAT DISP_GetPanelColorFormat(void)
{
    disp_drv_init_context();

    return (disp_if_drv->get_panel_color_format) ?
        (disp_if_drv->get_panel_color_format()) :
                DISP_STATUS_NOT_IMPLEMENTED;
}

UINT32 DISP_GetPanelBPP(void)
{
    PANEL_COLOR_FORMAT fmt;
    disp_drv_init_context();

    if(disp_if_drv->get_panel_color_format == NULL)
    {
        return DISP_STATUS_NOT_IMPLEMENTED;
    }

    fmt = disp_if_drv->get_panel_color_format();
    switch(fmt)
    {
        case PANEL_COLOR_FORMAT_RGB332:
            return 8;
        case PANEL_COLOR_FORMAT_RGB444:
            return 12;
        case PANEL_COLOR_FORMAT_RGB565:
            return 16;
        case PANEL_COLOR_FORMAT_RGB666:
            return 18;
        case PANEL_COLOR_FORMAT_RGB888:
            return 24;
        default:
            return 0;
    }
}

UINT32 DISP_GetOutputBPPforDithering(void)
{
    disp_drv_init_context();

    return (disp_if_drv->get_dithering_bpp) ?
        (disp_if_drv->get_dithering_bpp()) :
                DISP_STATUS_NOT_IMPLEMENTED;
}

DISP_STATUS DISP_Config_Overlay_to_Memory(unsigned int mva, int enable)
{
    int wait_ret = 0;

    //	struct disp_path_config_mem_out_struct mem_out = {0};

    if(enable)
    {
        MemOutConfig.outFormat = eRGB888;

        MemOutConfig.enable = 1;
        MemOutConfig.dstAddr = mva;
        MemOutConfig.srcROI.x = 0;
        MemOutConfig.srcROI.y = 0;
        MemOutConfig.srcROI.height= DISP_GetScreenHeight();
        MemOutConfig.srcROI.width= DISP_GetScreenWidth();

#if !defined(MTK_HDMI_SUPPORT)
        mutex_lock(&MemOutSettingMutex);
        MemOutConfig.dirty = 1;
        mutex_unlock(&MemOutSettingMutex);
#endif
    }
    else
    {
        MemOutConfig.outFormat = eRGB888;
        MemOutConfig.enable = 0;
        MemOutConfig.dstAddr = mva;
        MemOutConfig.srcROI.x = 0;
        MemOutConfig.srcROI.y = 0;
        MemOutConfig.srcROI.height= DISP_GetScreenHeight();
        MemOutConfig.srcROI.width= DISP_GetScreenWidth();

        //#if !defined(MTK_WFD_SUPPORT) && !defined(MTK_HDMI_SUPPORT)
        mutex_lock(&MemOutSettingMutex);
        MemOutConfig.dirty = 1;
        mutex_unlock(&MemOutSettingMutex);
        //#endif

        // Wait for reg update.
        wait_ret = wait_event_interruptible(reg_update_wq, !MemOutConfig.dirty);
        DISP_LOG("[WaitQ] wait_event_interruptible() ret = %d, %d\n", wait_ret, __LINE__);
    }

    return DISP_STATUS_OK;
}


DISP_STATUS DISP_Capture_Framebuffer( unsigned int pvbuf, unsigned int bpp, unsigned int is_early_suspended )
{
    unsigned int mva;
    unsigned int ret = 0;
    int wait_ret = 0;
	int i;
    DISP_FUNC();
    for (i=0; i<DDP_OVL_LAYER_MUN; i++)
    {
        if (cached_layer_config[i].layer_en && cached_layer_config[i].security)
            break;
    }
    if (i < DDP_OVL_LAYER_MUN)
    {
        // There is security layer.
        memset((void*)pvbuf, 0, DISP_GetScreenHeight()*DISP_GetScreenWidth()*bpp/8);
        return DISP_STATUS_OK;
    }
    disp_drv_init_context();

    MMProfileLogEx(MTKFB_MMP_Events.CaptureFramebuffer, MMProfileFlagPulse, 0, pvbuf);
    MMProfileLogEx(MTKFB_MMP_Events.CaptureFramebuffer, MMProfileFlagPulse, 1, bpp);\

    ret = disphal_map_overlay_out_buffer(pvbuf, DISP_GetScreenHeight()*DISP_GetScreenWidth()*bpp/8, &mva);
    if(ret!=0)
    {
        printk("disphal_map_overlay_out_buffer fail! \n");
        return DISP_STATUS_OK;
    }
    disphal_init_overlay_to_memory();

    mutex_lock(&MemOutSettingMutex);
    if(bpp == 32)
        MemOutConfig.outFormat = eARGB8888;
    else if(bpp == 16)
        MemOutConfig.outFormat = eRGB565;
    else if(bpp == 24)
        MemOutConfig.outFormat = eRGB888;
    else
    {
        printk("DSI_Capture_FB, fb color format not support\n");
        MemOutConfig.outFormat = eRGB888;
    }

    MemOutConfig.enable = 1;
    MemOutConfig.dstAddr = mva;
    MemOutConfig.srcROI.x = 0;
    MemOutConfig.srcROI.y = 0;
    MemOutConfig.srcROI.height= DISP_GetScreenHeight();
    MemOutConfig.srcROI.width= DISP_GetScreenWidth();
    if (is_early_suspended == 0)
        MemOutConfig.dirty = 1;
    mutex_unlock(&MemOutSettingMutex);
    MMProfileLogEx(MTKFB_MMP_Events.CaptureFramebuffer, MMProfileFlagPulse, 2, mva);
    if (is_early_suspended)
    {
        disp_path_get_mutex();
        disp_path_config_mem_out_without_lcd(&MemOutConfig);
        disp_path_release_mutex();
        // Wait for mem out done.
        disp_path_wait_mem_out_done();
        MMProfileLogEx(MTKFB_MMP_Events.CaptureFramebuffer, MMProfileFlagPulse, 3, 0);
        MemOutConfig.enable = 0;
            disp_path_get_mutex();
        disp_path_config_mem_out_without_lcd(&MemOutConfig);
            disp_path_release_mutex();
    }
    else
    {
        // Wait for mem out done.
        disp_path_wait_mem_out_done();
        MMProfileLogEx(MTKFB_MMP_Events.CaptureFramebuffer, MMProfileFlagPulse, 3, 0);

        mutex_lock(&MemOutSettingMutex);
        MemOutConfig.enable = 0;
        MemOutConfig.dirty = 1;
        mutex_unlock(&MemOutSettingMutex);
        // Wait for reg update.
        wait_ret = wait_event_interruptible(reg_update_wq, !MemOutConfig.dirty);
        DISP_LOG("[WaitQ] wait_event_interruptible() ret = %d, %d\n", wait_ret, __LINE__);
        MMProfileLogEx(MTKFB_MMP_Events.CaptureFramebuffer, MMProfileFlagPulse, 4, 0);
    }

    BOOL deinit_o2m = TRUE;

#if defined(MTK_HDMI_SUPPORT)
    deinit_o2m &= true;//!is_hdmi_active();
#endif


    if(deinit_o2m)
    {
        disphal_deinit_overlay_to_memory();
    }

    disphal_unmap_overlay_out_buffer(pvbuf, DISP_GetScreenHeight()*DISP_GetScreenWidth()*bpp/8, mva);

    return DISP_STATUS_OK;
}

DISP_STATUS DISP_Capture_Videobuffer(unsigned int pvbuf, unsigned int bpp, unsigned int video_rotation)
{
    return DISP_STATUS_OK;
}
// xuecheng, 2010-09-19
// this api is for mATV signal interfere workaround.
// immediate update == (TE disabled + delay update in overlay mode disabled)
DISP_STATUS DISP_ConfigImmediateUpdate(BOOL enable)
{
    disp_drv_init_context();
    if(enable == TRUE)
    {
		disphal_enable_te(FALSE);
    }
    else
    {
		if(disp_if_drv->init_te_control)
			disp_if_drv->init_te_control();
        else
            return DISP_STATUS_NOT_IMPLEMENTED;
    }

    is_immediateupdate = enable;

    return DISP_STATUS_OK;
}

BOOL DISP_IsImmediateUpdate(void)
{
    return is_immediateupdate;
}

DISP_STATUS DISP_FMDesense_Query()
{
    return disphal_fm_desense_query();
}

DISP_STATUS DISP_FM_Desense(unsigned long freq)
{
    DISP_STATUS ret = DISP_STATUS_OK;

    if (down_interruptible(&sem_update_screen)) {
        DISP_LOG("ERROR: Can't get sem_update_screen in DISP_FM_Desense()\n");
        return DISP_STATUS_ERROR;
    }

    ret = disphal_fm_desense(freq);

    up(&sem_update_screen);
    return ret;
}

DISP_STATUS DISP_Reset_Update()
{
    DISP_STATUS ret = DISP_STATUS_OK;

    if (down_interruptible(&sem_update_screen)) {
        DISP_LOG("ERROR: Can't get sem_update_screen in DISP_Reset_Update()\n");
        return DISP_STATUS_ERROR;
    }

    ret = disphal_reset_update();

    up(&sem_update_screen);

    return ret;
}

DISP_STATUS DISP_Get_Default_UpdateSpeed(unsigned int *speed)
{
    return disphal_get_default_updatespeed(speed);
}

DISP_STATUS DISP_Get_Current_UpdateSpeed(unsigned int *speed)
{
    return disphal_get_current_updatespeed(speed);
}

DISP_STATUS DISP_Change_Update(unsigned int speed)
{
    DISP_STATUS ret = DISP_STATUS_OK;

    if (down_interruptible(&sem_update_screen)) {
        DISP_LOG("ERROR: Can't get sem_update_screen in DISP_Change_Update()\n");
        return DISP_STATUS_ERROR;
    }

    ret = disphal_change_updatespeed(speed);

    up(&sem_update_screen);

    return ret;
}

extern UINT32 color;
unsigned int DISP_AutoTest()
{
	unsigned int ret = 0;
    if (down_interruptible(&sem_update_screen)) {
        printk("ERROR: Can't get sem_update_screen in DISP_Change_Update()\n");
        return DISP_STATUS_ERROR;
    }
    ret = disphal_check_lcm(color);

	up(&sem_update_screen);
	return ret;
}

unsigned int DISP_BLS_Query(void)
{
	return disphal_bls_query();
}

void DISP_BLS_Enable(BOOL enable)
{
	disphal_bls_enable(enable);
}

const char* DISP_GetLCMId(void)
{
    if(lcm_drv)
        return lcm_drv->name;
    else
        return NULL;
}


BOOL DISP_EsdCheck(void)
{
    BOOL result = FALSE;

    disp_drv_init_context();
    MMProfileLogEx(MTKFB_MMP_Events.EsdCheck, MMProfileFlagPulse, 0x10, 0);

    if(lcm_drv->esd_check == NULL && disp_if_drv->esd_check == NULL)
    {
        return FALSE;
    }

    if (down_interruptible(&sem_update_screen)) {
        printk("ERROR: Can't get sem_update_screen in DISP_EsdCheck()\n");
        return FALSE;
    }
    MMProfileLogEx(MTKFB_MMP_Events.EsdCheck, MMProfileFlagPulse, 0x11, 0);

    if(is_lcm_in_suspend_mode)
    {
        up(&sem_update_screen);
        return FALSE;
    }

    if(disp_if_drv->esd_check)
        result |= disp_if_drv->esd_check();
    MMProfileLogEx(MTKFB_MMP_Events.EsdCheck, MMProfileFlagPulse, 0x12, 0);

    up(&sem_update_screen);

    return result;
}
extern void disp_dsi_late_prepare(void);
extern void disp_dsi_post(void);

BOOL fbconfig_dsi_vdo_prepare(void)
{
	BOOL result = FALSE;
if ( !is_early_suspended)
	{
		disp_drv_init_context();

		if (down_interruptible(&sem_update_screen)) {
			printk("ERROR: Can't get sem_update_screen in fbconfig_dsi_vdo_prepare()\n");
			return FALSE;
		}

		if(is_lcm_in_suspend_mode)
		{
			up(&sem_update_screen);
			return FALSE;
		}
	//to do: set to cmd mode and other preparation....
	printk("sxk=>02will set cmd mode in disp_drv.c!!\n");
		disp_dsi_late_prepare();
	printk("sxk=>exec cmd !!\n");
		fb_config_execute_cmd();//execute my cmds from config file....
	printk("sxk=>restore to vdo mode !!\n");
		disp_dsi_post();

		up(&sem_update_screen);
		printk("sxk=>will call disphal_update_scn !!\n");
		//disphal_update_screen(lcm_drv, &LcmCmdMutex, 0, 0, DISP_GetScreenWidth(), DISP_GetScreenHeight());

		return TRUE;
	}
 	else
	return FALSE;
}







BOOL DISP_EsdRecoverCapbility(void)
{
    if(!disp_drv_init_context())
        return FALSE;

    if((lcm_drv->esd_check && lcm_drv->esd_recover) || (lcm_params->dsi.lcm_ext_te_monitor) || (lcm_params->dsi.lcm_int_te_monitor))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

BOOL DISP_EsdRecover(void)
{
    BOOL result = FALSE;
    DISP_LOG("DISP_EsdRecover enter");

    if(lcm_drv->esd_recover == NULL)
    {
        return FALSE;
    }

    if (down_interruptible(&sem_update_screen)) {
        printk("ERROR: Can't get sem_update_screen in DISP_EsdRecover()\n");
        return FALSE;
    }

    if(is_lcm_in_suspend_mode)
    {
        up(&sem_update_screen);
        return FALSE;
    }

    disphal_wait_not_busy();

     DISP_LOG("DISP_EsdRecover do LCM recover");

    // do necessary configuration reset for LCM re-init
    if(disp_if_drv->esd_reset)
        disp_if_drv->esd_reset();

    /// LCM recover
    mutex_lock(&LcmCmdMutex);
    result = lcm_drv->esd_recover();
    mutex_unlock(&LcmCmdMutex);

	if ((lcm_params->type == LCM_TYPE_DSI) && (lcm_params->dsi.mode != CMD_MODE))
		{
		   is_video_mode_running = false;
		   needStartEngine = true;
		}

    up(&sem_update_screen);

	DISP_UpdateScreen(0, 0, DISP_GetScreenWidth(), DISP_GetScreenHeight());

    return result;
}

unsigned long DISP_GetLCMIndex(void)
{
    return u4IndexOfLCMList;
}

DISP_STATUS DISP_PrepareSuspend(void)
{
    disphal_prepare_suspend();
    return DISP_STATUS_OK;
}

DISP_STATUS DISP_GetLayerInfo(DISP_LAYER_INFO *pLayer)
{
    int id = pLayer->id;
    mutex_lock(&OverlaySettingMutex);
    pLayer->curr_en = captured_layer_config[id].layer_en;
    pLayer->next_en = cached_layer_config[id].layer_en;
    pLayer->hw_en = realtime_layer_config[id].layer_en;
    pLayer->curr_idx = captured_layer_config[id].buff_idx;
    pLayer->next_idx = cached_layer_config[id].buff_idx;
    pLayer->hw_idx = realtime_layer_config[id].buff_idx;
    pLayer->curr_identity = captured_layer_config[id].identity;
    pLayer->next_identity = cached_layer_config[id].identity;
    pLayer->hw_identity = realtime_layer_config[id].identity;
    pLayer->curr_conn_type = captured_layer_config[id].connected_type;
    pLayer->next_conn_type = cached_layer_config[id].connected_type;
    pLayer->hw_conn_type = realtime_layer_config[id].connected_type;
    mutex_unlock(&OverlaySettingMutex);
    return DISP_STATUS_OK;
}
void DISP_Change_LCM_Resolution(unsigned int width, unsigned int height)
{
	if(lcm_params)
	{
		printk("LCM Resolution will be changed, original: %dx%d, now: %dx%d\n", lcm_params->width, lcm_params->height, width, height);
		if(width >lcm_params->width || height > lcm_params->height || width == 0 || height == 0)
			printk("Invalid resolution: %dx%d\n", width, height);
		lcm_params->width = width;
		lcm_params->height = height;
	}
}
/*
called from SET_MIPI_CLK ioctl in debug.c
Purpose : set mipi clk ,cover cmd mode and video mode
*/
extern int fbconfig_mipi_clk_set(unsigned int clk);
static BOOL fbconfig_disp_set_clk_prepare(unsigned int clk)
{
	BOOL result = FALSE;
if ( !is_early_suspended)
	{
		disp_drv_init_context();

		if (down_interruptible(&sem_update_screen)) {
			printk("ERROR: Can't get sem_update_screen in fbconfig_disp_set_clk_prepare()\n");
			return FALSE;
		}

			if(is_lcm_in_suspend_mode)
			{
				up(&sem_update_screen);
				return FALSE;
			}
		//to do: set to cmd mode and other preparation....
			disp_dsi_late_prepare();
		printk("sxk==>in disp_drv 2124:%d\n",clk);
			fbconfig_mipi_clk_set(clk);//execute :clk setting .....
			disp_dsi_post();
			up(&sem_update_screen);
			return TRUE;
		}
		else
		return FALSE;
}




void fbconfig_disp_set_mipi_clk(unsigned int clk)
{
	if(lcm_params->dsi.mode != CMD_MODE)
	{
	if (down_interruptible(&sem_early_suspend)) {
					printk("sxk=>can't get semaphore in fbconfig_disp_set_mipi_clk()\n");
					return 0;
			   }

	fbconfig_disp_set_clk_prepare(clk);
	up(&sem_early_suspend);
	}
	else{//cmd mode
	if (down_interruptible(&sem_early_suspend)) {
				printk("sxk=>can't get semaphore in fbconfig_disp_set_mipi_clk()\n");
				return 0;
		   }
	fbconfig_mipi_clk_set(clk);//execute :clk setting .....
	up(&sem_early_suspend);
	}
}
extern void fbconfig_mipi_lane_set(unsigned int lane_num);
void fbconfig_disp_set_mipi_lane_num(unsigned int lane_num)
{
	fbconfig_mipi_lane_set(lane_num);
}

extern void fbconfig_mipi_timing_set(MIPI_TIMING timing);

void fbconfig_disp_set_mipi_timing(MIPI_TIMING timing)
{
	fbconfig_mipi_timing_set(timing);

}



