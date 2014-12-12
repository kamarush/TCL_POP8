#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/autoconf.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/param.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <mach/mt_smi.h>

#include <linux/xlog.h>
#include <linux/proc_fs.h>  //proc file use
//ION
#include <linux/ion.h>
#include <linux/ion_drv.h>
//#include <mach/m4u.h>

#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>


#include <mach/irqs.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_irq.h>
#include <mach/irqs.h>
#include <mach/mt_clkmgr.h> // ????
#include <mach/mt_irq.h>
#include <mach/sync_write.h>

#include "ddp_drv.h"
#include "ddp_reg.h"
#include "ddp_hal.h"
#include "ddp_path.h"
#include "ddp_debug.h"
#include "ddp_color.h"
#include "disp_drv_ddp.h"
#include "ddp_wdma.h"
#include "ddp_cmdq.h"
#include "ddp_bls.h"

//#include <asm/tcm.h>
unsigned int dbg_log = 0;
unsigned int irq_log = 0;  // must disable irq level log by default, else will block uart output, open it only for debug use
unsigned int irq_err_log = 0;

#if 0  // defined in ddp_debug.h
#define DISP_WRN(string, args...) if(dbg_log) printk("[DSS]"string,##args)
#define DISP_MSG(string, args...) if(0) printk("[DSS]"string,##args)
#define DISP_ERR(string, args...) if(dbg_log) printk("[DSS]error:"string,##args)
#define DISP_IRQ(string, args...) if(irq_log) printk("[DSS]"string,##args)
#endif

#define DISP_DEVNAME "mtk_disp"
// device and driver
static dev_t disp_devno;
static struct cdev *disp_cdev;
static struct class *disp_class = NULL;

//ION

unsigned char ion_init=0;
unsigned char dma_init=0;

//NCSTool for Color Tuning
unsigned char ncs_tuning_mode = 0;

//flag for gamma lut update
unsigned char bls_gamma_dirty = 0;

struct ion_client *cmdqIONClient;
struct ion_handle *cmdqIONHandle;
struct ion_mm_data mm_data;
unsigned long * cmdq_pBuffer;
unsigned int cmdq_pa;
unsigned int cmdq_pa_len;
struct ion_sys_data sys_data;
//M4U_PORT_STRUCT m4uPort;
//irq
#define DISP_REGISTER_IRQ(irq_num){\
    if(request_irq( irq_num , (irq_handler_t)disp_irq_handler, IRQF_TRIGGER_LOW, DISP_DEVNAME , NULL))\
    { DISP_ERR("ddp register irq failed! %d\n", irq_num); }}

//-------------------------------------------------------------------------------//
// global variables
typedef struct
{
    spinlock_t irq_lock;
    unsigned int irq_src;  //one bit represent one module
} disp_irq_struct;

typedef struct
{
    pid_t open_pid;
    pid_t open_tgid;
    struct list_head testList;
    unsigned int u4LockedMutex;
    unsigned int u4LockedResource;
    unsigned int u4Clock;
    spinlock_t node_lock;
} disp_node_struct;

#define DISP_MAX_IRQ_CALLBACK   10
static DDP_IRQ_CALLBACK g_disp_irq_table[DISP_MODULE_MAX][DISP_MAX_IRQ_CALLBACK];

disp_irq_struct g_disp_irq;
static DECLARE_WAIT_QUEUE_HEAD(g_disp_irq_done_queue);
static DECLARE_WAIT_QUEUE_HEAD(gMutexWaitQueue);

// cmdq thread

unsigned char cmdq_thread[CMDQ_THREAD_NUM] = {1, 1, 1, 1, 1, 1, 1};
spinlock_t gCmdqLock;
extern spinlock_t gCmdqMgrLock;

wait_queue_head_t cmq_wait_queue[CMDQ_THREAD_NUM];

// Hardware Mutex Variables
#define ENGINE_MUTEX_NUM 8
spinlock_t gMutexLock;
int mutex_used[ENGINE_MUTEX_NUM] = {1, 1, 1, 1, 0, 0, 0, 0};    // 0 for FB, 1 for Bitblt, 2 for HDMI, 3 for BLS

//G2d Variables
spinlock_t gResourceLock;
unsigned int gLockedResource;//lock dpEngineType_6582
static DECLARE_WAIT_QUEUE_HEAD(gResourceWaitQueue);

// Overlay Variables
spinlock_t gOvlLock;
int disp_run_dp_framework = 0;
int disp_layer_enable = 0;
int disp_mutex_status = 0;

DISP_OVL_INFO disp_layer_info[DDP_OVL_LAYER_MUN];

//AAL variables
static unsigned long u4UpdateFlag = 0;

//Register update lock
spinlock_t gRegisterUpdateLock;
spinlock_t gPowerOperateLock;
//Clock gate management
//static unsigned long g_u4ClockOnTbl = 0;

//PQ variables
extern UINT32 fb_width;
extern UINT32 fb_height;
extern unsigned char aal_debug_flag;

// IRQ log print kthread
static struct task_struct *disp_irq_log_task = NULL;
static wait_queue_head_t disp_irq_log_wq;
static int disp_irq_log_module = 0;

static DISPLAY_TDSHP_T g_TDSHP_Index; 

static int g_irq_err_print = 0; // print aee warning 2s one time
#define DDP_ERR_IRQ_INTERVAL_TIME 5
static unsigned int disp_irq_err = 0;
unsigned int cnt_rdma_underflow = 0;
unsigned int cnt_rdma_abnormal = 0;
unsigned int cnt_ovl_underflow = 0;
unsigned int cnt_ovl_abnormal = 0;
unsigned int cnt_wdma_underflow = 0;
#define DDP_IRQ_OVL_L0_ABNORMAL  (1<<0)
#define DDP_IRQ_OVL_L1_ABNORMAL  (1<<1)
#define DDP_IRQ_OVL_L2_ABNORMAL  (1<<2)
#define DDP_IRQ_OVL_L3_ABNORMAL  (1<<3)
#define DDP_IRQ_OVL_L0_UNDERFLOW (1<<4)
#define DDP_IRQ_OVL_L1_UNDERFLOW (1<<5)
#define DDP_IRQ_OVL_L2_UNDERFLOW (1<<6)
#define DDP_IRQ_OVL_L3_UNDERFLOW (1<<7)
#define DDP_IRQ_RDMA_ABNORMAL       (1<<8)
#define DDP_IRQ_RDMA_UNDERFLOW      (1<<9)
#define DDP_IRQ_WDMA_ABNORMAL       (1<<10)
#define DDP_IRQ_WDMA_UNDERFLOW      (1<<11)

static struct timer_list disp_irq_err_timer;
static void disp_irq_err_timer_handler(unsigned long lparam)
{
    g_irq_err_print = 1;
}

DISPLAY_TDSHP_T *get_TDSHP_index(void)  
{    
    return &g_TDSHP_Index;
}


// internal function
static int disp_wait_intr(DISP_MODULE_ENUM module, unsigned int timeout_ms);
#if 0
static int disp_set_overlay_roi(int layer, int x, int y, int w, int h, int pitch);
static int disp_set_overlay_addr(int layer, unsigned int addr, DpColorFormat fmt);
static int disp_set_overlay(int layer, int enable);
static int disp_is_dp_framework_run(void);
static int disp_set_mutex_status(int enable);
#endif
static int disp_get_mutex_status(void);
static int disp_set_needupdate(DISP_MODULE_ENUM eModule , unsigned long u4En);
static void disp_power_off(DISP_MODULE_ENUM eModule , unsigned int * pu4Record);
static void disp_power_on(DISP_MODULE_ENUM eModule , unsigned int * pu4Record);
extern void DpEngine_COLORonConfig(unsigned int srcWidth,unsigned int srcHeight);
extern void DpEngine_COLORonInit(void);

extern void cmdqForceFreeAll(int cmdqThread);
extern void cmdqForceFree_SW(int taskID);
bool checkMdpEngineStatus(unsigned int engineFlag);

#if 0
struct disp_path_config_struct
{
    DISP_MODULE_ENUM srcModule;
    unsigned int addr; 
    unsigned int inFormat; 
    unsigned int pitch;
    struct DISP_REGION srcROI;        // ROI

    unsigned int layer;
    bool layer_en;
    enum OVL_LAYER_SOURCE source; 
    struct DISP_REGION bgROI;         // background ROI
    unsigned int bgColor;  // background color
    unsigned int key;     // color key
    bool aen;             // alpha enable
    unsigned char alpha;

    DISP_MODULE_ENUM dstModule;
    unsigned int outFormat; 
    unsigned int dstAddr;  // only take effect when dstModule=DISP_MODULE_WDMA1
};
int disp_path_enable();
int disp_path_config(struct disp_path_config_struct* pConfig);
#endif

unsigned int* pRegBackup = NULL;

//-------------------------------------------------------------------------------//
// functions

static int disp_irq_log_kthread_func(void *data)
{
    unsigned int i=0;
    char temp_buf[30];
    while(1)
    {
        wait_event_interruptible(disp_irq_log_wq, disp_irq_log_module||disp_irq_err);
        DISP_MSG("disp_irq_log_kthread_func dump intr register: disp_irq_log_module=%d \n", disp_irq_log_module);
        if(disp_irq_log_module!=0)
        {
            for(i=0;i<DISP_MODULE_MAX;i++)
            {
                if( (disp_irq_log_module&(1<<i))!=0 )
                {
                    disp_dump_reg(i);
                }
            }
        }
        // reset wakeup flag
        disp_irq_log_module = 0;

        if((disp_irq_err!=0) && (g_irq_err_print==1))
        {
            #if 0
            if(disp_irq_err&DDP_IRQ_OVL_L0_ABNORMAL)
            {
               DDP_IRQ_ERR("OVL_RDMA0_ABNORMAL");
            }
            if(disp_irq_err&DDP_IRQ_OVL_L1_ABNORMAL)
            {
               DDP_IRQ_ERR("OVL_RDMA1_ABNORMAL");
            }
            if(disp_irq_err&DDP_IRQ_OVL_L2_ABNORMAL)
            {
               DDP_IRQ_ERR("OVL_RDMA2_ABNORMAL");
            }
            if(disp_irq_err&DDP_IRQ_OVL_L3_ABNORMAL)
            {
               DDP_IRQ_ERR("OVL_RDMA3_ABNORMAL");
            }
            if(disp_irq_err&DDP_IRQ_OVL_L0_UNDERFLOW)
            {
               DDP_IRQ_ERR("OVL_RDMA0_ABNORMAL");
            }
            if(disp_irq_err&DDP_IRQ_OVL_L1_UNDERFLOW)
            {
               DDP_IRQ_ERR("OVL_RDMA1_ABNORMAL");
            }
            if(disp_irq_err&DDP_IRQ_OVL_L2_UNDERFLOW)
            {
               DDP_IRQ_ERR("OVL_RDMA2_ABNORMAL");
            }
            if(disp_irq_err&DDP_IRQ_OVL_L3_UNDERFLOW)
            {
               DDP_IRQ_ERR("OVL_RDMA3_ABNORMAL");
            }
            #endif            
            if(disp_irq_err&DDP_IRQ_RDMA_ABNORMAL)
            {
               sprintf((char*)temp_buf, "DDP_RDMA_ABNORMAL %d", cnt_rdma_abnormal);
               DDP_IRQ_ERR((char*)temp_buf);
            }
            if(disp_irq_err&DDP_IRQ_RDMA_UNDERFLOW)
            {
               sprintf(temp_buf, "DDP_RDMA_UNDERFLOW %d", cnt_rdma_underflow);
               DDP_IRQ_ERR((char*)temp_buf);
            }
            if(disp_irq_err&DDP_IRQ_WDMA_UNDERFLOW)
            {
               sprintf(temp_buf, "DDP_WDMA_UNDERFLOW %d", cnt_wdma_underflow);
               DDP_IRQ_ERR((char*)temp_buf);
            }            
            
            g_irq_err_print = 0;  // at most, 5s print one frame
            mod_timer(&disp_irq_err_timer, jiffies + DDP_ERR_IRQ_INTERVAL_TIME*HZ);
        }
        disp_irq_err = 0;
        
    }

    return 0;
}

unsigned int disp_ms2jiffies(unsigned long ms)
{
    return ((ms*HZ + 512) >> 10);
}

#if 1
int disp_lock_mutex(void)
{
    int id = -1;
    int i;
    spin_lock(&gMutexLock);

    for(i = 0 ; i < ENGINE_MUTEX_NUM ; i++)
        if(mutex_used[i] == 0)
        {
            id = i;
            mutex_used[i] = 1;
            //DISP_REG_SET_FIELD((1 << i) , DISP_REG_CONFIG_MUTEX_INTEN , 1);
            break;
        }
    spin_unlock(&gMutexLock);

    return id;
}

int disp_unlock_mutex(int id)
{
    if(id < 0 && id >= ENGINE_MUTEX_NUM) 
        return -1;

    spin_lock(&gMutexLock);
    
    mutex_used[id] = 0;
    //DISP_REG_SET_FIELD((1 << id) , DISP_REG_CONFIG_MUTEX_INTEN , 0);
    
    spin_unlock(&gMutexLock);

    return 0;
}
#endif //0

int disp_lock_cmdq_thread(void)
{
    int i=0;

    printk("disp_lock_cmdq_thread()called \n");
    
    spin_lock(&gCmdqLock);
    for (i = 0; i < CMDQ_THREAD_NUM; i++)
    {
        if (cmdq_thread[i] == 1) 
        {
            cmdq_thread[i] = 0;
            break;
        }
    } 
    spin_unlock(&gCmdqLock);

    printk("disp_lock_cmdq_thread(), i=%d \n", i);

    return (i>=CMDQ_THREAD_NUM)? -1 : i;
    
}

int disp_unlock_cmdq_thread(unsigned int idx)
{
    if(idx >= CMDQ_THREAD_NUM)
        return -1;

    spin_lock(&gCmdqLock);        
    cmdq_thread[idx] = 1;  // free thread availbility
    spin_unlock(&gCmdqLock);

    return 0;
}

// if return is not 0, should wait again
static int disp_wait_intr(DISP_MODULE_ENUM module, unsigned int timeout_ms)
{
    int ret;
    unsigned long flags;

    unsigned long long end_time = 0;
    unsigned long long start_time = sched_clock();
        
    MMProfileLogEx(DDP_MMP_Events.WAIT_INTR, MMProfileFlagStart, 0, module);
    // wait until irq done or timeout
    ret = wait_event_interruptible_timeout(
                    g_disp_irq_done_queue, 
                    g_disp_irq.irq_src & (1<<module), 
                    disp_ms2jiffies(timeout_ms) );
                    
    /*wake-up from sleep*/
    if(ret==0) // timeout
    {
        MMProfileLogEx(DDP_MMP_Events.WAIT_INTR, MMProfileFlagPulse, 0, module);
        MMProfileLog(DDP_MMP_Events.WAIT_INTR, MMProfileFlagEnd);
        DISP_ERR("Wait Done Timeout! pid=%d, module=%d \n", current->pid ,module);
        if(module==DISP_MODULE_WDMA0)
        {
            printk("======== WDMA0 timeout, dump all registers! \n");
            disp_dump_reg(DISP_MODULE_WDMA0);
            disp_dump_reg(DISP_MODULE_CONFIG);
        }
        else
        {
            disp_dump_reg(module);
        }

        return -EAGAIN;        
    }
    else if(ret<0) // intr by a signal
    {
        MMProfileLogEx(DDP_MMP_Events.WAIT_INTR, MMProfileFlagPulse, 1, module);
        MMProfileLog(DDP_MMP_Events.WAIT_INTR, MMProfileFlagEnd);
        DISP_ERR("Wait Done interrupted by a signal! pid=%d, module=%d \n", current->pid ,module);
        disp_dump_reg(module);
        return -EAGAIN;                 
    }

    MMProfileLogEx(DDP_MMP_Events.WAIT_INTR, MMProfileFlagEnd, 0, module);
    spin_lock_irqsave( &g_disp_irq.irq_lock , flags );
    g_disp_irq.irq_src &= ~(1<<module);    
    spin_unlock_irqrestore( &g_disp_irq.irq_lock , flags );

    end_time = sched_clock();
   	DISP_DBG("disp_wait_intr wait %d us\n", ((unsigned int)end_time-(unsigned int)start_time)/1000);
    
              
    return 0;
}

int disp_register_irq(DISP_MODULE_ENUM module, DDP_IRQ_CALLBACK cb)
{
    int i;
    if (module >= DISP_MODULE_MAX)
    {
        DISP_ERR("Register IRQ with invalid module ID. module=%d\n", module);
        return -1;
    }
    if (cb == NULL)
    {
        DISP_ERR("Register IRQ with invalid cb.\n");
        return -1;
    }
    for (i=0; i<DISP_MAX_IRQ_CALLBACK; i++)
    {
        if (g_disp_irq_table[module][i] == cb)
            break;
    }
    if (i < DISP_MAX_IRQ_CALLBACK)
    {
        // Already registered.
        return 0;
    }
    for (i=0; i<DISP_MAX_IRQ_CALLBACK; i++)
    {
        if (g_disp_irq_table[module][i] == NULL)
            break;
    }
    if (i == DISP_MAX_IRQ_CALLBACK)
    {
        DISP_ERR("No enough callback entries for module %d.\n", module);
        return -1;
    }
    g_disp_irq_table[module][i] = cb;
    return 0;
}

int disp_unregister_irq(DISP_MODULE_ENUM module, DDP_IRQ_CALLBACK cb)
{
    int i;
    for (i=0; i<DISP_MAX_IRQ_CALLBACK; i++)
    {
        if (g_disp_irq_table[module][i] == cb)
        {
            g_disp_irq_table[module][i] = NULL;
            break;
        }
    }
    if (i == DISP_MAX_IRQ_CALLBACK)
    {
        DISP_ERR("Try to unregister callback function with was not registered. module=%d cb=0x%08X\n", module, (unsigned int)cb);
        return -1;
    }
    return 0;
}

void disp_invoke_irq_callbacks(DISP_MODULE_ENUM module, unsigned int param)
{
    int i;
    for (i=0; i<DISP_MAX_IRQ_CALLBACK; i++)
    {
        if (g_disp_irq_table[module][i])
        {
            //DISP_ERR("Invoke callback function. module=%d param=0x%X\n", module, param);
            g_disp_irq_table[module][i](param);
        }
    }
}
#if defined(MTK_HDMI_SUPPORT)
extern void hdmi_setorientation(int orientation);
void hdmi_power_on(void);
void hdmi_power_off(void);
extern void hdmi_update_buffer_switch(void);
extern bool is_hdmi_active(void);
extern void hdmi_update(void);
extern void hdmi_source_buffer_switch(void);
#endif

//extern void hdmi_test_switch_buffer(void);
static /*__tcmfunc*/ irqreturn_t disp_irq_handler(int irq, void *dev_id)
{
    unsigned long reg_val; 
    //unsigned long index;
    unsigned long value;
    int i;
    //struct timeval tv;
    int taskid;
       
    /*1. Process ISR*/
    switch(irq)
    {
            
        case MT6582_DISP_OVL_IRQ_ID:  
                reg_val = DISP_REG_GET(DISP_REG_OVL_INTSTA);
                if(reg_val&(1<<0))
                {
                      DISP_IRQ("IRQ: OVL reg update done! \n");
                }    
                if(reg_val&(1<<1))
                {
                      DISP_IRQ("IRQ: OVL frame done! \n");
                      g_disp_irq.irq_src |= (1<<DISP_MODULE_OVL);
                }
                if(reg_val&(1<<2))
                {
                	///TODO: remove this
                	if (cnt_ovl_underflow % 250 == 0) {
                		DISP_MSG("0xf0000000=0x%x, xf0000050=0x%x\n", 
                	      *(volatile unsigned int*)(0xf0000000),
                	      *(volatile unsigned int*)(0xf0000050));
                        disp_irq_log_module |= (1<<DISP_MODULE_CONFIG);
                        disp_irq_log_module |= (1<<DISP_MODULE_MUTEX);
                        disp_irq_log_module |= (1<<DISP_MODULE_OVL);
                        disp_irq_log_module |= (1<<DISP_MODULE_RDMA1);

                	}
                      DISP_ERR("IRQ: OVL frame underrun! cnt=%d \n", cnt_ovl_underflow++);
                }
                if(reg_val&(1<<3))
                {
                      DISP_IRQ("IRQ: OVL SW reset done! \n");
                }
                if(reg_val&(1<<4))
                {
                      DISP_IRQ("IRQ: OVL HW reset done! \n");
                }
                if(reg_val&(1<<5))
                {
                      DISP_ERR("IRQ: OVL-L0 not complete untill EOF! \n");
                      disp_irq_err |= DDP_IRQ_OVL_L0_ABNORMAL;
                }      
                if(reg_val&(1<<6))
                {
                      DISP_ERR("IRQ: OVL-L1 not complete untill EOF! \n");
                      disp_irq_err |= DDP_IRQ_OVL_L1_ABNORMAL;
                }
                if(reg_val&(1<<7))
                {
                      DISP_ERR("IRQ: OVL-L2 not complete untill EOF! \n");
                      disp_irq_err |= DDP_IRQ_OVL_L2_ABNORMAL;
                }        
                if(reg_val&(1<<8))
                {
                      DISP_ERR("IRQ: OVL-L3 not complete untill EOF! \n");
                      disp_irq_err |= DDP_IRQ_OVL_L3_ABNORMAL;
                }
                if(reg_val&(1<<9))
                {
                      DISP_ERR("IRQ: OVL-L0 fifo underflow! \n");
                      disp_irq_err |= DDP_IRQ_OVL_L0_UNDERFLOW;
                }      
                if(reg_val&(1<<10))
                {
                      DISP_ERR("IRQ: OVL-L1 fifo underflow! \n");
                      disp_irq_err |= DDP_IRQ_OVL_L1_UNDERFLOW;
                }
                if(reg_val&(1<<11))
                {
                      DISP_ERR("IRQ: OVL-L2 fifo underflow! \n");
                      disp_irq_err |= DDP_IRQ_OVL_L2_UNDERFLOW;
                }    
                if(reg_val&(1<<12))
                {
                      DISP_ERR("IRQ: OVL-L3 fifo underflow! \n");
                      disp_irq_err |= DDP_IRQ_OVL_L3_UNDERFLOW;
                }                                                                                                  
                //clear intr
                DISP_REG_SET(DISP_REG_OVL_INTSTA, ~reg_val);     
                MMProfileLogEx(DDP_MMP_Events.OVL_IRQ, MMProfileFlagPulse, reg_val, 0);
                disp_invoke_irq_callbacks(DISP_MODULE_OVL, reg_val);
            break;
            
        case MT6582_DISP_WDMA_IRQ_ID:
                reg_val = DISP_REG_GET(DISP_REG_WDMA_INTSTA);
                if(reg_val&(1<<0))
                {
                    DISP_IRQ("IRQ: WDMA0 frame done! cnt=%d \n", cnt_wdma_underflow++);
                    g_disp_irq.irq_src |= (1<<DISP_MODULE_WDMA0);
                }    
                if(reg_val&(1<<1))
                {
                      DISP_ERR("IRQ: WDMA0 underrun! \n");
                      disp_irq_err |= DDP_IRQ_WDMA_UNDERFLOW;
                }  
                //clear intr
                DISP_REG_SET(DISP_REG_WDMA_INTSTA, ~reg_val);           
                MMProfileLogEx(DDP_MMP_Events.WDMA0_IRQ, MMProfileFlagPulse, reg_val, DISP_REG_GET(DISP_REG_WDMA_CLIP_SIZE));
                disp_invoke_irq_callbacks(DISP_MODULE_WDMA0, reg_val);
            break;
            

        case MT6582_DISP_RDMA_IRQ_ID:
                reg_val = DISP_REG_GET(DISP_REG_RDMA_INT_STATUS);
                if(reg_val&(1<<0))
                {
                      DISP_IRQ("IRQ: RDMA1 reg update done! \n");
                }    
                if(reg_val&(1<<1))
                {
                      DISP_IRQ("IRQ: RDMA1 frame start! \n");
//	                      if(disp_needWakeUp())
//	                      {
//	                          disp_update_hist();
//	                          disp_wakeup_aal();
//	                      }
                      on_disp_aal_alarm_set();
                }
                if(reg_val&(1<<2))
                {
                      DISP_IRQ("IRQ: RDMA1 frame done! \n");
                      g_disp_irq.irq_src |= (1<<DISP_MODULE_RDMA1);
                }
                if(reg_val&(1<<3))
                {
                	///TODO: remove this
                	if (cnt_rdma_abnormal % 250 == 0) {
                	    DISP_MSG("0xf0000000=0x%x, xf0000050=0x%x\n", 
                	      *(volatile unsigned int*)(0xf0000000),
                	      *(volatile unsigned int*)(0xf0000050));
                		disp_irq_log_module |= (1<<DISP_MODULE_CONFIG);
                        disp_irq_log_module |= (1<<DISP_MODULE_MUTEX);
                        disp_irq_log_module |= (1<<DISP_MODULE_OVL);
                        disp_irq_log_module |= (1<<DISP_MODULE_RDMA1);
                	}
                	DISP_ERR("IRQ: RDMA1 abnormal! cnt=%d \n", cnt_rdma_abnormal++);
                	disp_irq_err |= DDP_IRQ_RDMA_ABNORMAL;
                }
                if(reg_val&(1<<4))
                {
                      DISP_ERR("IRQ: RDMA1 underflow! cnt=%d \n", cnt_rdma_underflow++);
                      disp_irq_err |= DDP_IRQ_RDMA_UNDERFLOW;
                }
                //clear intr
                DISP_REG_SET(DISP_REG_RDMA_INT_STATUS, ~reg_val);           
                MMProfileLogEx(DDP_MMP_Events.RDMA1_IRQ, MMProfileFlagPulse, reg_val, 0);
                disp_invoke_irq_callbacks(DISP_MODULE_RDMA1, reg_val);
            break;  

        case MT6582_DISP_COLOR_IRQ_ID:
            reg_val = DISP_REG_GET(DISPSYS_COLOR_BASE+0x0F08);

            // read LUMA histogram
            if (reg_val & 0x2)
            {
//TODO : might want to move to other IRQ~ -S       
                //disp_update_hist();
                //disp_wakeup_aal();
//TODO : might want to move to other IRQ~ -E
            }

            //clear intr
            DISP_REG_SET(DISPSYS_COLOR_BASE+0x0F08, ~reg_val);
            MMProfileLogEx(DDP_MMP_Events.COLOR_IRQ, MMProfileFlagPulse, reg_val, 0);
//            disp_invoke_irq_callbacks(DISP_MODULE_COLOR, reg_val);
            break;
                        
        case MT6582_DISP_BLS_IRQ_ID:
            reg_val = DISP_REG_GET(DISP_REG_BLS_INTSTA);

            // read LUMA & MAX(R,G,B) histogram
            if (reg_val & 0x1)
            {
                  disp_update_hist();
                  disp_wakeup_aal();
            }

            //clear intr
            DISP_REG_SET(DISP_REG_BLS_INTSTA, ~reg_val);
            MMProfileLogEx(DDP_MMP_Events.BLS_IRQ, MMProfileFlagPulse, reg_val, 0);
            break;

        case MT6582_DISP_MUTEX_IRQ_ID:  // can not do reg update done status after release mutex(for ECO requirement), 
                                        // so we have to check update timeout intr here
            reg_val = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA) & 0x0FF0F;
            
            if(reg_val & 0x0FF00) // udpate timeout intr triggered
            {
                unsigned int reg = 0;
                unsigned int mutexID = 0;
                
                for(mutexID=0;mutexID<4;mutexID++)
                {
                    if((DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA) & (1<<(mutexID+8))) == (1<<(mutexID+8)))
                    {
                        DISP_ERR("disp_path_release_mutex() timeout! \n");
                        disp_dump_reg(DISP_MODULE_CONFIG);
                        //print error engine
                        reg = DISP_REG_GET(DISP_REG_CONFIG_REG_COMMIT);
                        if(reg!=0)
                        {
                            if(reg&(1<<3))  { DISP_MSG(" OVL update reg timeout! \n"); disp_dump_reg(DISP_MODULE_OVL); }
                            if(reg&(1<<7))  { DISP_MSG(" COLOR update reg timeout! \n");    disp_dump_reg(DISP_MODULE_COLOR); }
                            if(reg&(1<<6))  { DISP_MSG(" WDMA0 update reg timeout! \n"); disp_dump_reg(DISP_MODULE_WDMA0); }
                            if(reg&(1<<10))  { DISP_MSG(" RDMA1 update reg timeout! \n"); disp_dump_reg(DISP_MODULE_RDMA1); }
                            if(reg&(1<<9))  { DISP_MSG(" BLS update reg timeout! \n"); disp_dump_reg(DISP_MODULE_BLS); }
                        }  
                     
                        //reset mutex
                        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_RST(mutexID), 1);
                        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_RST(mutexID), 0);
                        DISP_MSG("mutex reset done! \n");
                    }
                 }
            }            
            DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTSTA, ~reg_val);      
            disp_invoke_irq_callbacks(DISP_MODULE_MUTEX, reg_val);
            break;

            
        case MT6582_DISP_CMDQ_IRQ_ID:
            
            reg_val = DISP_REG_GET(DISP_REG_CMDQ_IRQ_FLAG) & 0x03fff; 
            for(i = 0; ((0x03fff != reg_val) && (i < CMDQ_THREAD_NUM)); i++)
            {
                if (0x0 == (reg_val & (1 << i)))
                {
                    value = DISP_REG_GET(DISP_REG_CMDQ_THRx_IRQ_FLAG(i));
                    if(value & 0x12)
                    {
                        cmdqHandleError(i);
                    }    
                    else if (value & 0x01)
                    {
                        cmdqHandleDone(i);
                    }
                    reg_val |= ~(1 << i);
                }

                MMProfileLogEx(DDP_MMP_Events.CMDQ_IRQ, MMProfileFlagPulse, reg_val, i);
            }
            break;
#if 0      
        case MT6582_G2D_IRQ_ID:
            reg_val = DISP_REG_GET(DISP_REG_G2D_IRQ);
            if(reg_val&G2D_IRQ_STA_BIT)
            {
				  unsigned long set_val = reg_val & ~(G2D_IRQ_STA_BIT); 
                  DISP_IRQ("IRQ: G2D done! \n");
				  g_disp_irq.irq_src |= (1<<DISP_MODULE_G2D);
				  //clear intr
				  DISP_REG_SET(DISP_REG_G2D_IRQ, set_val);
            }                                   
            
            disp_invoke_irq_callbacks(DISP_MODULE_G2D, reg_val);
            break;			
#endif
        default: DISP_ERR("invalid irq=%d \n ", irq); break;
    }        

    // Wakeup event
    mb();   // Add memory barrier before the other CPU (may) wakeup
    wake_up_interruptible(&g_disp_irq_done_queue);    

    if((disp_irq_log_module!=0) || (disp_irq_err!=0))
    {
        wake_up_interruptible(&disp_irq_log_wq);
        //DISP_MSG("disp_irq_log_wq waked!, %d, %d \n", disp_irq_log_module, disp_irq_err);
    }

             
    return IRQ_HANDLED;
}


static void disp_power_on(DISP_MODULE_ENUM eModule , unsigned int * pu4Record)
{  
    unsigned long flag;
    //unsigned int ret = 0;
    spin_lock_irqsave(&gPowerOperateLock , flag);

#ifdef DDP_82_72_TODO
    if((1 << eModule) & g_u4ClockOnTbl)
    {
        DISP_MSG("DDP power %lu is already enabled\n" , (unsigned long)eModule);
    }
    else
    {
        switch(eModule)
        {

            case DISP_MODULE_WDMA0 :
                enable_clock(MT_CG_DISP0_DISP_WDMA , "DDP_DRV");
                //enable_clock(MT_CG_DISP0_WDMA0_SMI , "DDP_DRV");
            break;
 
            case DISP_MODULE_G2D :
                //enable_clock(MT_CG_DISP0_G2D_ENGINE , "DDP_DRV");
				//enable_clock(MT_CG_DISP0_G2D_SMI , "DDP_DRV");
            break;
            default :
                DISP_ERR("disp_power_on:unknown module:%d\n" , eModule);
                ret = -1;
            break;
        }

        if(0 == ret)
        {
            if(0 == g_u4ClockOnTbl)
            {
                enable_clock(MT_CG_DISP0_SMI_LARB0 , "DDP_DRV");
            }
            g_u4ClockOnTbl |= (1 << eModule);
            *pu4Record |= (1 << eModule);
        }
    }
#endif

    spin_unlock_irqrestore(&gPowerOperateLock , flag);
}

static void disp_power_off(DISP_MODULE_ENUM eModule , unsigned int * pu4Record)
{  
    unsigned long flag;
    //unsigned int ret = 0;
    spin_lock_irqsave(&gPowerOperateLock , flag);
    
#ifdef DDP_82_72_TODO
//    DISP_MSG("power off : %d\n" , eModule);

    if((1 << eModule) & g_u4ClockOnTbl)
    {
        switch(eModule)
        {
            case DISP_MODULE_WDMA0 :
            	  WDMAStop(0);
            	  WDMAReset(0);
                disable_clock(MT_CG_DISP0_DISP_WDMA , "DDP_DRV");
                //disable_clock(MT_CG_DISP0_WDMA0_SMI , "DDP_DRV");
            break;
            case DISP_MODULE_G2D :
                //disable_clock(MT_CG_DISP0_G2D_ENGINE , "DDP_DRV");
                //disable_clock(MT_CG_DISP0_G2D_SMI , "DDP_DRV");
            break;            
            default :
                DISP_ERR("disp_power_off:unsupported format:%d\n" , eModule);
                ret = -1;
            break;
        }

        if(0 == ret)
        {
            g_u4ClockOnTbl &= (~(1 << eModule));
            *pu4Record &= (~(1 << eModule));

            if(0 == g_u4ClockOnTbl)
            {
                disable_clock(MT_CG_DISP0_SMI_LARB0 , "DDP_DRV");
            }

        }
    }
    else
    {
        DISP_MSG("DDP power %lu is already disabled\n" , (unsigned long)eModule);
    }

#endif

    spin_unlock_irqrestore(&gPowerOperateLock , flag);
}

unsigned int inAddr=0, outAddr=0;

static int disp_set_needupdate(DISP_MODULE_ENUM eModule , unsigned long u4En)
{
    unsigned long flag;
    spin_lock_irqsave(&gRegisterUpdateLock , flag);

    if(u4En)
    {
        u4UpdateFlag |= (1 << eModule);
    }
    else
    {
        u4UpdateFlag &= ~(1 << eModule);
    }

    spin_unlock_irqrestore(&gRegisterUpdateLock , flag);

    return 0;
}

void DISP_REG_SET_FIELD(unsigned long field, unsigned long reg32, unsigned long val)
{
    unsigned long flag;
    spin_lock_irqsave(&gRegisterUpdateLock , flag);
    //*(volatile unsigned int*)(reg32) = ((*(volatile unsigned int*)(reg32) & ~(REG_FLD_MASK(field))) |  REG_FLD_VAL((field), (val)));
     mt65xx_reg_sync_writel( (*(volatile unsigned int*)(reg32) & ~(REG_FLD_MASK(field)))|REG_FLD_VAL((field), (val)), reg32);
     spin_unlock_irqrestore(&gRegisterUpdateLock , flag);
}

int CheckAALUpdateFunc(int i4IsNewFrame)
{
    return (((1 << DISP_MODULE_BLS) & u4UpdateFlag) || i4IsNewFrame || is_disp_aal_alarm_on()) ? 1 : 0;
}

int ConfAALFunc(int i4IsNewFrame)
{
    disp_onConfig_aal(i4IsNewFrame);
    disp_set_needupdate(DISP_MODULE_BLS , 0);
    return 0;
}

static int AAL_init = 0;
void disp_aal_lock()
{
    if(0 == AAL_init)
    {
        //printk("disp_aal_lock: register update func\n");
        DISP_RegisterExTriggerSource(CheckAALUpdateFunc , ConfAALFunc);
        AAL_init = 1;
    }
    GetUpdateMutex();
}

void disp_aal_unlock()
{
    ReleaseUpdateMutex();
    disp_set_needupdate(DISP_MODULE_BLS , 1);
}

int CheckColorUpdateFunc(int i4NotUsed)
{
    return (((1 << DISP_MODULE_COLOR) & u4UpdateFlag) || bls_gamma_dirty) ? 1 : 0;
}

int ConfColorFunc(int i4NotUsed)
{
    DISP_MSG("ConfColorFunc: BLS_EN=0x%x, bls_gamma_dirty=%d\n", DISP_REG_GET(DISP_REG_BLS_EN), bls_gamma_dirty);
    if(bls_gamma_dirty != 0)
    {
        // disable BLS
        if (DISP_REG_GET(DISP_REG_BLS_EN) & 0x1)
        {
            DISP_MSG("ConfColorFunc: Disable BLS\n");
            DISP_REG_SET(DISP_REG_BLS_EN, 0x00010000);
        }
    }
    else
    {
        if(ncs_tuning_mode == 0) //normal mode
        {
            DpEngine_COLORonInit();
            DpEngine_COLORonConfig(fb_width,fb_height);
        }
        else
        {
            ncs_tuning_mode = 0;
        }
        // enable BLS
        DISP_REG_SET(DISP_REG_BLS_EN, 0x00010001);
        disp_set_needupdate(DISP_MODULE_COLOR , 0);
    }
    DISP_MSG("ConfColorFunc done: BLS_EN=0x%x, bls_gamma_dirty=%d\n", DISP_REG_GET(DISP_REG_BLS_EN), bls_gamma_dirty);
    return 0;
}

static long disp_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    DISP_WRITE_REG wParams;
    DISP_READ_REG rParams;
    DISP_EXEC_COMMAND cParams;
    unsigned int ret = 0;
    unsigned int value;
    DISP_MODULE_ENUM module;
    DISP_OVL_INFO ovl_info;
    DISP_PQ_PARAM * pq_param;
    DISP_PQ_PARAM * pq_cam_param;
    DISP_PQ_PARAM * pq_gal_param;
    DISPLAY_PQ_T * pq_index;
    DISPLAY_TDSHP_T * tdshp_index;
    DISPLAY_GAMMA_T * gamma_index;
    //DISPLAY_PWM_T * pwm_lut;
    int layer, mutex_id;
    disp_wait_irq_struct wait_irq_struct;
    unsigned long lcmindex = 0;
//    unsigned int status;
    unsigned long flags;
    int count;

#if defined(MTK_AAL_SUPPORT)
    DISP_AAL_PARAM * aal_param;
#endif

#ifdef DDP_DBG_DDP_PATH_CONFIG
    struct disp_path_config_struct config;
#endif

    disp_node_struct *pNode = (disp_node_struct *)file->private_data;

#if 0
    if(inAddr==0)
    {
        inAddr = kmalloc(800*480*4, GFP_KERNEL);
        memset((void*)inAddr, 0x55, 800*480*4);
        DISP_MSG("inAddr=0x%x \n", inAddr);
    }
    if(outAddr==0)
    {
        outAddr = kmalloc(800*480*4, GFP_KERNEL);
        memset((void*)outAddr, 0xff, 800*480*4);
        DISP_MSG("outAddr=0x%x \n", outAddr);
    }
#endif
    DISP_DBG("cmd=0x%x, arg=0x%x \n", cmd, (unsigned int)arg);
    switch(cmd)
    {   
        case DISP_IOCTL_EXEC_COMMAND:
            if(copy_from_user(&cParams, (void*)arg, sizeof(DISP_EXEC_COMMAND)))
            {
                DISP_ERR("DISP_IOCTL_EXEC_COMMAND Copy from user error\n");
                return -EFAULT;
            }

            if (cmdqSubmitTask(cParams.scenario,
                               cParams.priority,
                               cParams.engineFlag,
                               cParams.pFrameBaseSW,
                               cParams.blockSize))
            {
                DISP_ERR("DISP_IOCTL_EXEC_COMMAND: Execute commands failed\n");
                return -EFAULT;
            }

            break;

        case DISP_IOCTL_WRITE_REG:
            
            if(copy_from_user(&wParams, (void *)arg, sizeof(DISP_WRITE_REG )))
            {
                DISP_ERR("DISP_IOCTL_WRITE_REG, copy_from_user failed\n");
                return -EFAULT;
            }

            DISP_DBG("write  0x%x = 0x%x (0x%x)\n", wParams.reg, wParams.val, wParams.mask);
            if(wParams.reg>DISPSYS_REG_ADDR_MAX || wParams.reg<DISPSYS_REG_ADDR_MIN)
            {
                DISP_ERR("reg write, addr invalid, addr min=0x%x, max=0x%x, addr=0x%x \n", 
                    DISPSYS_REG_ADDR_MIN, 
                    DISPSYS_REG_ADDR_MAX, 
                    wParams.reg);
                return -EFAULT;
            }
            
            *(volatile unsigned int*)wParams.reg = (*(volatile unsigned int*)wParams.reg & ~wParams.mask) | (wParams.val & wParams.mask);
            //mt65xx_reg_sync_writel(wParams.reg, value);
            break;
            
        case DISP_IOCTL_READ_REG:
            if(copy_from_user(&rParams, (void *)arg, sizeof(DISP_READ_REG)))
            {
                DISP_ERR("DISP_IOCTL_READ_REG, copy_from_user failed\n");
                return -EFAULT;
            }
            if(rParams.reg>DISPSYS_REG_ADDR_MAX || rParams.reg<DISPSYS_REG_ADDR_MIN)
            {
                DISP_ERR("reg read, addr invalid, addr min=0x%x, max=0x%x, addr=0x%x \n", 
                    DISPSYS_REG_ADDR_MIN, 
                    DISPSYS_REG_ADDR_MAX, 
                    rParams.reg);
                return -EFAULT;
            }

            value = (*(volatile unsigned int*)rParams.reg) & rParams.mask;

            DISP_DBG("read 0x%x = 0x%x (0x%x)\n", rParams.reg, value, rParams.mask);
            
            if(copy_to_user(rParams.val, &value, sizeof(unsigned int)))
            {
                DISP_ERR("DISP_IOCTL_READ_REG, copy_to_user failed\n");
                return -EFAULT;            
            }
            break;

        case DISP_IOCTL_WAIT_IRQ:
            if(copy_from_user(&wait_irq_struct, (void*)arg , sizeof(wait_irq_struct)))
            {
                DISP_ERR("DISP_IOCTL_WAIT_IRQ, copy_from_user failed\n");
                return -EFAULT;
            }  
            ret = disp_wait_intr(wait_irq_struct.module, wait_irq_struct.timeout_ms);            
            break;  

        case DISP_IOCTL_DUMP_REG:
            if(copy_from_user(&module, (void*)arg , sizeof(module)))
            {
                DISP_ERR("DISP_IOCTL_DUMP_REG, copy_from_user failed\n");
                return -EFAULT;
            }  
            ret = disp_dump_reg(module);            
            break;  

        case DISP_IOCTL_LOCK_THREAD:
            printk("DISP_IOCTL_LOCK_THREAD! \n");
            value = disp_lock_cmdq_thread();  
            if (copy_to_user((void*)arg, &value , sizeof(unsigned int)))
            {
                DISP_ERR("DISP_IOCTL_LOCK_THREAD, copy_to_user failed\n");
                return -EFAULT;
            }
            break; 
            
        case DISP_IOCTL_UNLOCK_THREAD:
            if(copy_from_user(&value, (void*)arg , sizeof(value)))
            {
                    DISP_ERR("DISP_IOCTL_UNLOCK_THREAD, copy_from_user failed\n");
                    return -EFAULT;
            }  
            ret = disp_unlock_cmdq_thread(value);  
            break;

        case DISP_IOCTL_MARK_CMQ:
            if(copy_from_user(&value, (void*)arg , sizeof(value)))
            {
                    DISP_ERR("DISP_IOCTL_MARK_CMQ, copy_from_user failed\n");
                    return -EFAULT;
            }
            if(value >= CMDQ_THREAD_NUM) return -EFAULT;
//            cmq_status[value] = 1;
            break;
            
        case DISP_IOCTL_WAIT_CMQ:
            if(copy_from_user(&value, (void*)arg , sizeof(value)))
            {
                    DISP_ERR("DISP_IOCTL_WAIT_CMQ, copy_from_user failed\n");
                    return -EFAULT;
            }
            if(value >= CMDQ_THREAD_NUM) return -EFAULT;
            /*
            wait_event_interruptible_timeout(cmq_wait_queue[value], cmq_status[value], 3 * HZ);
            if(cmq_status[value] != 0)
            {
                cmq_status[value] = 0;
                return -EFAULT;
            }
        */
            break;

        case DISP_IOCTL_LOCK_RESOURCE:
            if(copy_from_user(&mutex_id, (void*)arg , sizeof(int)))
            {
                DISP_ERR("DISP_IOCTL_LOCK_RESOURCE, copy_from_user failed\n");
                return -EFAULT;
            }
            if((-1) != mutex_id)
            {
                int ret = wait_event_interruptible_timeout(
                gResourceWaitQueue, 
                (gLockedResource & (1 << mutex_id)) == 0, 
                disp_ms2jiffies(50) ); 
                
                if(ret <= 0)
                {
                    DISP_ERR("DISP_IOCTL_LOCK_RESOURCE, mutex_id 0x%x failed\n",gLockedResource);
                    return -EFAULT;
                }
                
                spin_lock(&gResourceLock);
                gLockedResource |= (1 << mutex_id);
                spin_unlock(&gResourceLock);
                
                spin_lock(&pNode->node_lock);
                pNode->u4LockedResource = gLockedResource;
                spin_unlock(&pNode->node_lock);                 
            }
            else
            {
                DISP_ERR("DISP_IOCTL_LOCK_RESOURCE, mutex_id = -1 failed\n");
                return -EFAULT;
            }
            break;

            
        case DISP_IOCTL_UNLOCK_RESOURCE:
            if(copy_from_user(&mutex_id, (void*)arg , sizeof(int)))
            {
                DISP_ERR("DISP_IOCTL_UNLOCK_RESOURCE, copy_from_user failed\n");
                return -EFAULT;
            }
            if((-1) != mutex_id)
            {
                spin_lock(&gResourceLock);
                gLockedResource &= ~(1 << mutex_id);
                spin_unlock(&gResourceLock);
                
                spin_lock(&pNode->node_lock);
                pNode->u4LockedResource = gLockedResource;
                spin_unlock(&pNode->node_lock);   

                wake_up_interruptible(&gResourceWaitQueue); 
            } 
            else
            {
                DISP_ERR("DISP_IOCTL_UNLOCK_RESOURCE, mutex_id = -1 failed\n");
                return -EFAULT;
            }            
            break;

    #if 1
        case DISP_IOCTL_LOCK_MUTEX:
        {
            wait_event_interruptible_timeout(
            gMutexWaitQueue, 
            (mutex_id = disp_lock_mutex()) != -1, 
            disp_ms2jiffies(200) );             

            if((-1) != mutex_id)
            {
                spin_lock(&pNode->node_lock);
                pNode->u4LockedMutex |= (1 << mutex_id);
                spin_unlock(&pNode->node_lock);
            }
            
            if(copy_to_user((void *)arg, &mutex_id, sizeof(int)))
            {
                DISP_ERR("disp driver : Copy to user error (mutex)\n");
                return -EFAULT;            
            }
            break;
        }
        case DISP_IOCTL_UNLOCK_MUTEX:
            if(copy_from_user(&mutex_id, (void*)arg , sizeof(int)))
            {
                DISP_ERR("DISP_IOCTL_UNLOCK_MUTEX, copy_from_user failed\n");
                return -EFAULT;
            }
            disp_unlock_mutex(mutex_id);

            if((-1) != mutex_id)
            {
                spin_lock(&pNode->node_lock);
                pNode->u4LockedMutex &= ~(1 << mutex_id);
                spin_unlock(&pNode->node_lock);
            }

            wake_up_interruptible(&gMutexWaitQueue);             

            break;
    #endif // 0

        case DISP_IOCTL_SYNC_REG:
            mb();
            break;

        case DISP_IOCTL_SET_INTR:
            DISP_DBG("DISP_IOCTL_SET_INTR! \n");
            if(copy_from_user(&value, (void*)arg , sizeof(int)))
            {
                DISP_ERR("DISP_IOCTL_SET_INTR, copy_from_user failed\n");
                return -EFAULT;
            }  

            // enable intr
            if( (value&0xffff0000) !=0)
            {
                disable_irq(value&0xff);
                printk("disable_irq %d \n", value&0xff);
            }
            else
            {
                DISP_REGISTER_IRQ(value&0xff);
                printk("enable irq: %d \n", value&0xff);
            }            
            break; 

        case DISP_IOCTL_RUN_DPF:
            DISP_DBG("DISP_IOCTL_RUN_DPF! \n");
            if(copy_from_user(&value, (void*)arg , sizeof(int)))
            {
                DISP_ERR("DISP_IOCTL_SET_INTR, copy_from_user failed, %d\n", ret);
                return -EFAULT;
            }
            
            spin_lock(&gOvlLock);

            disp_run_dp_framework = value;
    
            spin_unlock(&gOvlLock);

            if(value == 1)
            {
                while(disp_get_mutex_status() != 0)
                {
                    DISP_ERR("disp driver : wait fb release hw mutex\n");
                    msleep(3);
                }
            }
            break;

        case DISP_IOCTL_CHECK_OVL:
            DISP_DBG("DISP_IOCTL_CHECK_OVL! \n");
            value = disp_layer_enable;
            
            if(copy_to_user((void *)arg, &value, sizeof(int)))
            {
                DISP_ERR("disp driver : Copy to user error (result)\n");
                return -EFAULT;            
            }
            break;

        case DISP_IOCTL_GET_OVL:
            DISP_DBG("DISP_IOCTL_GET_OVL! \n");
            if(copy_from_user(&ovl_info, (void*)arg , sizeof(DISP_OVL_INFO)))
            {
                DISP_ERR("DISP_IOCTL_SET_INTR, copy_from_user failed, %d\n", ret);
                return -EFAULT;
            } 

            layer = ovl_info.layer;
            
            spin_lock(&gOvlLock);
            ovl_info = disp_layer_info[layer];
            spin_unlock(&gOvlLock);
            
            if(copy_to_user((void *)arg, &ovl_info, sizeof(DISP_OVL_INFO)))
            {
                DISP_ERR("disp driver : Copy to user error (result)\n");
                return -EFAULT;            
            }
            
            break;

        case DISP_IOCTL_AAL_EVENTCTL:
#if !defined(MTK_AAL_SUPPORT)
            printk("Invalid operation DISP_IOCTL_AAL_EVENTCTL since AAL is not turned on, in %s\n" , __FUNCTION__);
            return -EFAULT;
#else
            if(copy_from_user(&value, (void *)arg, sizeof(int)))
            {
                printk("disp driver : DISP_IOCTL_AAL_EVENTCTL Copy from user failed\n");
                return -EFAULT;            
            }
            disp_set_aal_alarm(value);
            disp_set_needupdate(DISP_MODULE_BLS , 1);
            ret = 0;
#endif
            break;

        case DISP_IOCTL_GET_AALSTATISTICS:
#if !defined(MTK_AAL_SUPPORT)
            printk("Invalid operation DISP_IOCTL_GET_AALSTATISTICS since AAL is not turned on, in %s\n" , __FUNCTION__);
            return -EFAULT;
#else
            // 1. Wait till new interrupt comes
            if(disp_wait_hist_update(60))
            {
                printk("disp driver : DISP_IOCTL_GET_AALSTATISTICS wait time out\n");
                return -EFAULT;
            }

            // 2. read out color engine histogram
            disp_set_hist_readlock(1);
            if(copy_to_user((void*)arg, (void *)(disp_get_hist_ptr()) , sizeof(DISP_AAL_STATISTICS)))
            {
                printk("disp driver : DISP_IOCTL_GET_AALSTATISTICS Copy to user failed\n");
                return -EFAULT;
            }
            disp_set_hist_readlock(0);
            ret = 0;
#endif
            break;

        case DISP_IOCTL_SET_AALPARAM:
#if !defined(MTK_AAL_SUPPORT)
            printk("Invalid operation : DISP_IOCTL_SET_AALPARAM since AAL is not turned on, in %s\n" , __FUNCTION__);
            return -EFAULT;
#else
//            disp_set_needupdate(DISP_MODULE_BLS , 0);

            disp_aal_lock();

            aal_param = get_aal_config();

            if(copy_from_user(aal_param , (void *)arg, sizeof(DISP_AAL_PARAM)))
            {
                printk("disp driver : DISP_IOCTL_SET_AALPARAM Copy from user failed\n");
                return -EFAULT;            
            }

            disp_aal_unlock();
#endif
            break;

        case DISP_IOCTL_SET_PQPARAM:

            DISP_RegisterExTriggerSource(CheckColorUpdateFunc , ConfColorFunc);

            GetUpdateMutex();

            pq_param = get_Color_config();
            if(copy_from_user(pq_param, (void *)arg, sizeof(DISP_PQ_PARAM)))
            {
                printk("disp driver : DISP_IOCTL_SET_PQPARAM Copy from user failed\n");
                return -EFAULT;            
            }

            ReleaseUpdateMutex();

            disp_set_needupdate(DISP_MODULE_COLOR, 1);

            break;

        case DISP_IOCTL_SET_PQINDEX:

            pq_index = get_Color_index();
            if(copy_from_user(pq_index, (void *)arg, sizeof(DISPLAY_PQ_T)))
            {
                printk("disp driver : DISP_IOCTL_SET_PQINDEX Copy from user failed\n");
                return -EFAULT;
            }

            break;    
            
        case DISP_IOCTL_GET_PQPARAM:
            
            pq_param = get_Color_config();
            if(copy_to_user((void *)arg, pq_param, sizeof(DISP_PQ_PARAM)))
            {
                printk("disp driver : DISP_IOCTL_GET_PQPARAM Copy to user failed\n");
                return -EFAULT;            
            }

            break;

        case DISP_IOCTL_SET_TDSHPINDEX:
        
            tdshp_index = get_TDSHP_index();
            if(copy_from_user(tdshp_index, (void *)arg, sizeof(DISPLAY_TDSHP_T)))
            {
                printk("disp driver : DISP_IOCTL_SET_TDSHPINDEX Copy from user failed\n");
                return -EFAULT;
            }
        
            break;           
        
        case DISP_IOCTL_GET_TDSHPINDEX:
            
                tdshp_index = get_TDSHP_index();
                if(copy_to_user((void *)arg, tdshp_index, sizeof(DISPLAY_TDSHP_T)))
                {
                    printk("disp driver : DISP_IOCTL_GET_TDSHPINDEX Copy to user failed\n");
                    return -EFAULT;            
                }
        
                break;       
        
         case DISP_IOCTL_SET_GAMMALUT:
        
            DISP_MSG("DISP_IOCTL_SET_GAMMALUT\n");
            
            gamma_index = get_gamma_index();
            if(copy_from_user(gamma_index, (void *)arg, sizeof(DISPLAY_GAMMA_T)))
            {
                printk("disp driver : DISP_IOCTL_SET_GAMMALUT Copy from user failed\n");
                return -EFAULT;
            }

            // disable BLS and suspend AAL
            GetUpdateMutex();
            bls_gamma_dirty = 1;
            aal_debug_flag = 1;
            ReleaseUpdateMutex();

            disp_set_needupdate(DISP_MODULE_COLOR, 1);

            count = 0;
            while(DISP_REG_GET(DISP_REG_BLS_EN) & 0x1) {
                msleep(1);
                count++;
                if (count > 1000) {
                    DISP_ERR("fail to disable BLS (0x%x)\n", DISP_REG_GET(DISP_REG_BLS_EN));
                    break;
                }
            }

            // update gamma lut
            // enable BLS and resume AAL
            GetUpdateMutex();
            disp_bls_update_gamma_lut();
            bls_gamma_dirty = 0;
            aal_debug_flag = 0;
            ReleaseUpdateMutex(); 

            disp_set_needupdate(DISP_MODULE_COLOR, 1);

            break;

         case DISP_IOCTL_SET_CLKON:
            if(copy_from_user(&module, (void *)arg, sizeof(DISP_MODULE_ENUM)))
            {
                printk("disp driver : DISP_IOCTL_SET_CLKON Copy from user failed\n");
                return -EFAULT;            
            }

            disp_power_on(module , &(pNode->u4Clock));
            break;

        case DISP_IOCTL_SET_CLKOFF:
            if(copy_from_user(&module, (void *)arg, sizeof(DISP_MODULE_ENUM)))
            {
                printk("disp driver : DISP_IOCTL_SET_CLKOFF Copy from user failed\n");
                return -EFAULT;            
            }

            disp_power_off(module , &(pNode->u4Clock));
            break;

        case DISP_IOCTL_MUTEX_CONTROL:
            if(copy_from_user(&value, (void *)arg, sizeof(int)))
            {
                printk("disp driver : DISP_IOCTL_MUTEX_CONTROL Copy from user failed\n");
                return -EFAULT;            
            }

            DISP_MSG("DISP_IOCTL_MUTEX_CONTROL: %d, BLS_EN = %d\n", value, DISP_REG_GET(DISP_REG_BLS_EN));

            if(value == 1)
            {
            
                // disable BLS and suspend AAL
                GetUpdateMutex();
                bls_gamma_dirty = 1;
                aal_debug_flag = 1;
                ReleaseUpdateMutex();

                disp_set_needupdate(DISP_MODULE_COLOR, 1);

                count = 0;
                while(DISP_REG_GET(DISP_REG_BLS_EN) & 0x1) {
                    msleep(1);
                    count++;
                    if (count > 1000) {
                        DISP_ERR("fail to disable BLS (0x%x)\n", DISP_REG_GET(DISP_REG_BLS_EN));
                        break;
                    }
                }
                
                ncs_tuning_mode = 1;
                GetUpdateMutex();
            }
            else if(value == 2)
            {
                // enable BLS and resume AAL
                bls_gamma_dirty = 0;
                aal_debug_flag = 0;
                ReleaseUpdateMutex();
                
                disp_set_needupdate(DISP_MODULE_COLOR, 1);
            }
            else
            {
                printk("disp driver : DISP_IOCTL_MUTEX_CONTROL invalid control\n");
                return -EFAULT;            
            }

            DISP_MSG("DISP_IOCTL_MUTEX_CONTROL done: %d, BLS_EN = %d\n", value, DISP_REG_GET(DISP_REG_BLS_EN));
            
            break;    
            
        case DISP_IOCTL_GET_LCMINDEX:
            
                lcmindex = DISP_GetLCMIndex();
                if(copy_to_user((void *)arg, &lcmindex, sizeof(unsigned long)))
                {
                    printk("disp driver : DISP_IOCTL_GET_LCMINDEX Copy to user failed\n");
                    return -EFAULT;            
                }

                break;       

            break;        
            
        case DISP_IOCTL_SET_PQ_CAM_PARAM:

            pq_cam_param = get_Color_Cam_config();
            if(copy_from_user(pq_cam_param, (void *)arg, sizeof(DISP_PQ_PARAM)))
            {
                printk("disp driver : DISP_IOCTL_SET_PQ_CAM_PARAM Copy from user failed\n");
                return -EFAULT;            
            }

            break;
            
        case DISP_IOCTL_GET_PQ_CAM_PARAM:
            
            pq_cam_param = get_Color_Cam_config();
            if(copy_to_user((void *)arg, pq_cam_param, sizeof(DISP_PQ_PARAM)))
            {
                printk("disp driver : DISP_IOCTL_GET_PQ_CAM_PARAM Copy to user failed\n");
                return -EFAULT;            
            }
            
            break;        
            
        case DISP_IOCTL_SET_PQ_GAL_PARAM:

            pq_gal_param = get_Color_Gal_config();
            if(copy_from_user(pq_gal_param, (void *)arg, sizeof(DISP_PQ_PARAM)))
            {
                printk("disp driver : DISP_IOCTL_SET_PQ_GAL_PARAM Copy from user failed\n");
                return -EFAULT;            
            }
            
            break;

        case DISP_IOCTL_GET_PQ_GAL_PARAM:
            
            pq_gal_param = get_Color_Gal_config();
            if(copy_to_user((void *)arg, pq_gal_param, sizeof(DISP_PQ_PARAM)))
            {
                printk("disp driver : DISP_IOCTL_GET_PQ_GAL_PARAM Copy to user failed\n");
                return -EFAULT;            
            }
            
            break;          

        case DISP_IOCTL_TEST_PATH:
#ifdef DDP_DBG_DDP_PATH_CONFIG
            if(copy_from_user(&value, (void*)arg , sizeof(value)))
            {
                    DISP_ERR("DISP_IOCTL_MARK_CMQ, copy_from_user failed\n");
                    return -EFAULT;
            }

            config.layer = 0;
            config.layer_en = 1;
            config.source = OVL_LAYER_SOURCE_MEM; 
            config.addr = virt_to_phys(inAddr); 
            config.inFormat = OVL_INPUT_FORMAT_RGB565; 
            config.pitch = 480;
            config.srcROI.x = 0;        // ROI
            config.srcROI.y = 0;  
            config.srcROI.width = 480;  
            config.srcROI.height = 800;  
            config.bgROI.x = config.srcROI.x;
            config.bgROI.y = config.srcROI.y;
            config.bgROI.width = config.srcROI.width;
            config.bgROI.height = config.srcROI.height;
            config.bgColor = 0xff;  // background color
            config.key = 0xff;     // color key
            config.aen = 0;             // alpha enable
            config.alpha = 0;  
            DISP_MSG("value=%d \n", value);
            if(value==0) // mem->ovl->rdma0->dpi0
            {
                config.srcModule = DISP_MODULE_OVL;
                config.outFormat = RDMA_OUTPUT_FORMAT_ARGB; 
                config.dstModule = DISP_MODULE_DPI0;
                config.dstAddr = 0;
            }
            else if(value==1) // mem->ovl-> wdma1->mem
            {
                config.srcModule = DISP_MODULE_OVL;
                config.outFormat = WDMA_OUTPUT_FORMAT_RGB888; 
                config.dstModule = DISP_MODULE_WDMA0;
                config.dstAddr = virt_to_phys(outAddr);
            }
            else if(value==2)  // mem->rdma0 -> dpi0
            {
                config.srcModule = DISP_MODULE_RDMA1;
                config.outFormat = RDMA_OUTPUT_FORMAT_ARGB; 
                config.dstModule = DISP_MODULE_DPI0;
                config.dstAddr = 0;
            }
            disp_path_config(&config);
            disp_path_enable();
#endif			
            break;            
#if 0
        case DISP_IOCTL_G_WAIT_REQUEST:
            ret = ddp_bitblt_ioctl_wait_reequest(arg);
            break;

        case DISP_IOCTL_T_INFORM_DONE:
            ret = ddp_bitblt_ioctl_inform_done(arg);
            break;
#endif
            
        default :
            DISP_ERR("Ddp drv dose not have such command : %d\n" , cmd);
            break; 
    }
    
    return ret;
}

static int disp_open(struct inode *inode, struct file *file)
{
    disp_node_struct *pNode = NULL;

    DISP_DBG("enter disp_open() process:%s\n",current->comm);

    //Allocate and initialize private data
    file->private_data = kmalloc(sizeof(disp_node_struct) , GFP_ATOMIC);
    if(NULL == file->private_data)
    {
        DISP_MSG("Not enough entry for DDP open operation\n");
        return -ENOMEM;
    }
   
    pNode = (disp_node_struct *)file->private_data;
    pNode->open_pid = current->pid;
    pNode->open_tgid = current->tgid;
    INIT_LIST_HEAD(&(pNode->testList));
    pNode->u4LockedMutex = 0;
    pNode->u4LockedResource = 0;
    pNode->u4Clock = 0;
    spin_lock_init(&pNode->node_lock);

    return 0;

}

static ssize_t disp_read(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
    return 0;
}

static int disp_release(struct inode *inode, struct file *file)
{
    disp_node_struct *pNode = NULL;
    unsigned int index = 0;
    DISP_DBG("enter disp_release() process:%s\n",current->comm);
    
    pNode = (disp_node_struct *)file->private_data;

    spin_lock(&pNode->node_lock);

    if(pNode->u4LockedMutex)
    {
        DISP_ERR("Proccess terminated[Mutex] ! :%s , mutex:%u\n" 
            , current->comm , pNode->u4LockedMutex);

        for(index = 0 ; index < ENGINE_MUTEX_NUM ; index += 1)
        {
            if((1 << index) & pNode->u4LockedMutex)
            {
                disp_unlock_mutex(index);
                DISP_DBG("unlock index = %d ,mutex_used[ %d %d %d %d ]\n",index,mutex_used[0],mutex_used[1] ,mutex_used[2],mutex_used[3]);
            }
        }
        
    } 

    if(pNode->u4LockedResource)
    {
        DISP_ERR("Proccess terminated[REsource] ! :%s , resource:%d\n" 
            , current->comm , pNode->u4LockedResource);
        spin_lock(&gResourceLock);
        gLockedResource = 0;
        spin_unlock(&gResourceLock);
    }

    if(pNode->u4Clock)
    {
        DISP_ERR("Process safely terminated [Clock] !:%s , clock:%u\n" 
            , current->comm , pNode->u4Clock);

        for(index  = 0 ; index < DISP_MODULE_MAX; index += 1)
        {
            if((1 << index) & pNode->u4Clock)
            {
                disp_power_off((DISP_MODULE_ENUM)index , &pNode->u4Clock);
            }
        }
    }

    spin_unlock(&pNode->node_lock);

    if(NULL != file->private_data)
    {
        kfree(file->private_data);
        file->private_data = NULL;
    }
    
    return 0;
}

static int disp_flush(struct file * file , fl_owner_t a_id)
{
    return 0;
}

// remap register to user space
static int disp_mmap(struct file * file, struct vm_area_struct * a_pstVMArea)
{

    a_pstVMArea->vm_page_prot = pgprot_noncached(a_pstVMArea->vm_page_prot);
    if(remap_pfn_range(a_pstVMArea , 
                 a_pstVMArea->vm_start , 
                 a_pstVMArea->vm_pgoff , 
                 (a_pstVMArea->vm_end - a_pstVMArea->vm_start) , 
                 a_pstVMArea->vm_page_prot))
    {
        DISP_MSG("MMAP failed!!\n");
        return -1;
    }


    return 0;
}


/* Kernel interface */
static struct file_operations disp_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl = disp_unlocked_ioctl,
	.open		= disp_open,
	.release	= disp_release,
	.flush		= disp_flush,
	.read       = disp_read,
	.mmap       = disp_mmap
};

static int disp_probe(struct platform_device *pdev)
{
    struct class_device;
    
	int ret;
	int i;
    struct class_device *class_dev = NULL;
    
    DISP_MSG("\ndisp driver probe...\n\n");
	ret = alloc_chrdev_region(&disp_devno, 0, 1, DISP_DEVNAME);

	if(ret)
	{
	    DISP_ERR("Error: Can't Get Major number for DISP Device\n");
	}
	else
	{
	    DISP_MSG("Get DISP Device Major number (%d)\n", disp_devno);
    }

	disp_cdev = cdev_alloc();
    disp_cdev->owner = THIS_MODULE;
	disp_cdev->ops = &disp_fops;

	ret = cdev_add(disp_cdev, disp_devno, 1);

    disp_class = class_create(THIS_MODULE, DISP_DEVNAME);
    class_dev = (struct class_device *)device_create(disp_class, NULL, disp_devno, NULL, DISP_DEVNAME);

    // initial wait queue
    for(i = 0 ; i < CMDQ_THREAD_NUM ; i++)
    {
        init_waitqueue_head(&cmq_wait_queue[i]);

        // enable CMDQ interrupt
        DISP_REG_SET(DISP_REG_CMDQ_THRx_IRQ_FLAG_EN(i),0x13); //SL TEST CMDQ time out
    }
        
    // Register IRQ                      
    DISP_REGISTER_IRQ(MT6582_DISP_COLOR_IRQ_ID);
    DISP_REGISTER_IRQ(MT6582_DISP_BLS_IRQ_ID);
    DISP_REGISTER_IRQ(MT6582_DISP_OVL_IRQ_ID);
    DISP_REGISTER_IRQ(MT6582_DISP_WDMA_IRQ_ID);
    DISP_REGISTER_IRQ(MT6582_DISP_RDMA_IRQ_ID);
    DISP_REGISTER_IRQ(MT6582_DISP_CMDQ_IRQ_ID);
    DISP_REGISTER_IRQ(MT6582_DISP_MUTEX_IRQ_ID);
    //DISP_REGISTER_IRQ(MT6582_G2D_IRQ_ID);
    
    init_waitqueue_head(&disp_irq_log_wq);
    disp_irq_log_task = kthread_create(disp_irq_log_kthread_func, NULL, "disp_config_update_kthread");
    if (IS_ERR(disp_irq_log_task)) 
    {
        DISP_ERR("DISP_InitVSYNC(): Cannot create disp_irq_log_task kthread\n");
    }
    wake_up_process(disp_irq_log_task);
   
    // init error log timer
    init_timer(&disp_irq_err_timer);
    disp_irq_err_timer.expires = jiffies + 5*HZ;
    disp_irq_err_timer.function = disp_irq_err_timer_handler;    
    add_timer(&disp_irq_err_timer);
                
	DISP_MSG("DISP Probe Done\n");
	NOT_REFERENCED(class_dev);
	return 0;
}

static int disp_remove(struct platform_device *pdev)
{
    disable_irq(MT6582_DISP_OVL_IRQ_ID);
    disable_irq(MT6582_DISP_WDMA_IRQ_ID);
    disable_irq(MT6582_DISP_RDMA_IRQ_ID);
    disable_irq(MT6582_DISP_CMDQ_IRQ_ID);
    //disable_irq(MT6582_DISP_COLOR_IRQ_ID);
    disable_irq(MT6582_DISP_BLS_IRQ_ID);
    //disable_irq(MT6582_G2D_IRQ_ID);
    return 0;
}

static void disp_shutdown(struct platform_device *pdev)
{
	/* Nothing yet */
}


/* PM suspend */
static int disp_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    printk("\n\n==== DISP suspend is called ====\n");

    return cmdqSuspendTask();
}

/* PM resume */
static int disp_resume(struct platform_device *pdev)
{
    return cmdqResumeTask();
}

#if 0
int disp_pm_restore_noirq(struct device *device)
{
    return 0; 
}

struct dev_pm_ops disp_pm_ops =
{
    .suspend       = disp_suspend,
    .resume        = disp_resume,
    .freeze        = disp_suspend,
    .thaw          = disp_resume,
    .poweroff      = disp_suspend,
    .restore       = disp_resume,
    .restore_noirq = disp_pm_restore_noirq,
};
#endif // 0


static struct platform_driver disp_driver =
{
	.probe		= disp_probe,
	.remove		= disp_remove,
	.shutdown	= disp_shutdown,
	.suspend	= disp_suspend,
	.resume		= disp_resume,
	.driver     =
	{
	    .name = DISP_DEVNAME,
	    //.pm   = &disp_pm_ops,
	},
};


#if 0
static void disp_device_release(struct device *dev)
{
	// Nothing to release? 
}

static u64 disp_dmamask = ~(u32)0;

static struct platform_device disp_device = {
	.name	 = DISP_DEVNAME,
	.id      = 0,
	.dev     = {
		.release = disp_device_release,
		.dma_mask = &disp_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources = 0,
};
#endif // 0

static int __init disp_init(void)
{
    int ret;

    spin_lock_init(&gCmdqLock);
    spin_lock_init(&gResourceLock);
    spin_lock_init(&gOvlLock);
    spin_lock_init(&gRegisterUpdateLock);
    spin_lock_init(&gPowerOperateLock);
    spin_lock_init(&g_disp_irq.irq_lock);
    spin_lock_init(&gMutexLock);

#if 0
    DISP_MSG("Register disp device\n");
	if(platform_device_register(&disp_device))
	{
        DISP_ERR("failed to register disp device\n");
        ret = -ENODEV;
        return ret;
	}
#endif // 0

    DISP_MSG("Register the disp driver\n");    
    if(platform_driver_register(&disp_driver))
    {
        DISP_ERR("failed to register disp driver\n");
        //platform_device_unregister(&disp_device);
        ret = -ENODEV;
        return ret;
    }

    ddp_debug_init();

    pRegBackup = kmalloc(DDP_BACKUP_REG_NUM*sizeof(int), GFP_KERNEL);
    ASSERT(pRegBackup!=NULL);
    *pRegBackup = DDP_UNBACKED_REG_MEM;

    cmdqInitialize();

    return 0;
}

static void __exit disp_exit(void)
{
    cmdqDeInitialize();

    cdev_del(disp_cdev);
    unregister_chrdev_region(disp_devno, 1);
	
    platform_driver_unregister(&disp_driver);
    //platform_device_unregister(&disp_device);
	
    device_destroy(disp_class, disp_devno);
    class_destroy(disp_class);

    ddp_debug_exit();

    DISP_MSG("Done\n");
}

#if 0
static int disp_set_overlay_roi(int layer, int x, int y, int w, int h, int pitch)
{
    // DISP_MSG(" disp_set_overlay_roi %d\n", layer );
    
    if(layer < 0 || layer >= DDP_OVL_LAYER_MUN) return -1;
    spin_lock(&gOvlLock);

    disp_layer_info[layer].x = x;
    disp_layer_info[layer].y = y;
    disp_layer_info[layer].w = w;
    disp_layer_info[layer].h = h;
    disp_layer_info[layer].pitch = pitch;
    
    spin_unlock(&gOvlLock);

    return 0;
}


static int disp_set_overlay_addr(int layer, unsigned int addr, DpColorFormat fmt)
{
    // DISP_MSG(" disp_set_overlay_addr %d\n", layer );
    if(layer < 0 || layer >= DDP_OVL_LAYER_MUN) return -1;
    
    spin_lock(&gOvlLock);

    disp_layer_info[layer].addr = addr;
    //if(fmt != DDP_NONE_FMT)
        disp_layer_info[layer].fmt = fmt;
    
    spin_unlock(&gOvlLock);

    return 0;
}

static int disp_set_overlay(int layer, int enable)
{
    // DISP_MSG(" disp_set_overlay %d %d\n", layer, enable );
    if(layer < 0 || layer >= DDP_OVL_LAYER_MUN) return -1;
    
    spin_lock(&gOvlLock);

    if(enable == 0)
        disp_layer_enable = disp_layer_enable & ~(1 << layer);
    else
        disp_layer_enable = disp_layer_enable | (1 << layer);

    spin_unlock(&gOvlLock);

    return 0;
}


static int disp_is_dp_framework_run()
{
    // DISP_MSG(" disp_is_dp_framework_run " );
    return disp_run_dp_framework;
}

static int disp_set_mutex_status(int enable)
{
    // DISP_MSG(" disp_set_mutex_status %d\n", enable );
    spin_lock(&gOvlLock);

    disp_mutex_status = enable;
    
    spin_unlock(&gOvlLock);
    return 0;
}
#endif

static int disp_get_mutex_status()
{
    return disp_mutex_status;
}

#if 0
int disp_dump_reg(DISP_MODULE_ENUM module)
    {
        //unsigned int size;
        //unsigned int reg_base;
        unsigned int index = 0;

        switch(module)
        {       
            case DISP_MODULE_OVL: 
                DISP_MSG("===== DISP OVL Reg Dump: ============\n");          
                DISP_MSG("(000)OVL_STA                 =0x%x \n", DISP_REG_GET(DISP_REG_OVL_STA                   ));
                DISP_MSG("(004)OVL_INTEN                   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_INTEN                 ));
                DISP_MSG("(008)OVL_INTSTA              =0x%x \n", DISP_REG_GET(DISP_REG_OVL_INTSTA                ));
                DISP_MSG("(00C)OVL_EN                  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_EN                    ));
                DISP_MSG("(010)OVL_TRIG                    =0x%x \n", DISP_REG_GET(DISP_REG_OVL_TRIG                  ));
                DISP_MSG("(014)OVL_RST                 =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RST                   ));
                DISP_MSG("(020)OVL_ROI_SIZE                =0x%x \n", DISP_REG_GET(DISP_REG_OVL_ROI_SIZE              ));
                DISP_MSG("(020)OVL_DATAPATH_CON            =0x%x \n", DISP_REG_GET(DISP_REG_OVL_DATAPATH_CON          ));
                DISP_MSG("(028)OVL_ROI_BGCLR               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_ROI_BGCLR             ));
                DISP_MSG("(02C)OVL_SRC_CON             =0x%x \n", DISP_REG_GET(DISP_REG_OVL_SRC_CON               ));
                DISP_MSG("(030)OVL_L0_CON              =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_CON                ));
                DISP_MSG("(034)OVL_L0_SRCKEY               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_SRCKEY             ));
                DISP_MSG("(038)OVL_L0_SRC_SIZE         =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_SRC_SIZE           ));
                DISP_MSG("(03C)OVL_L0_OFFSET               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_OFFSET             ));
                DISP_MSG("(040)OVL_L0_ADDR             =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_ADDR               ));
                DISP_MSG("(044)OVL_L0_PITCH                =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_PITCH              ));
                DISP_MSG("(0C0)OVL_RDMA0_CTRL          =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA0_CTRL            ));
                DISP_MSG("(0C4)OVL_RDMA0_MEM_START_TRIG    =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA0_MEM_START_TRIG  ));
                DISP_MSG("(0C8)OVL_RDMA0_MEM_GMC_SETTING   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING ));
                DISP_MSG("(0CC)OVL_RDMA0_MEM_SLOW_CON  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA0_MEM_SLOW_CON    ));
                DISP_MSG("(0D0)OVL_RDMA0_FIFO_CTRL     =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA0_FIFO_CTRL       ));
                DISP_MSG("(050)OVL_L1_CON              =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_CON                ));
                DISP_MSG("(054)OVL_L1_SRCKEY               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_SRCKEY             ));
                DISP_MSG("(058)OVL_L1_SRC_SIZE         =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_SRC_SIZE           ));
                DISP_MSG("(05C)OVL_L1_OFFSET               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_OFFSET             ));
                DISP_MSG("(060)OVL_L1_ADDR             =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_ADDR               ));
                DISP_MSG("(064)OVL_L1_PITCH                =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_PITCH              ));
                DISP_MSG("(0E0)OVL_RDMA1_CTRL          =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA1_CTRL            ));
                DISP_MSG("(0E4)OVL_RDMA1_MEM_START_TRIG    =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA1_MEM_START_TRIG  ));
                DISP_MSG("(0E8)OVL_RDMA1_MEM_GMC_SETTING   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING ));
                DISP_MSG("(0EC)OVL_RDMA1_MEM_SLOW_CON  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA1_MEM_SLOW_CON    ));
                DISP_MSG("(0F0)OVL_RDMA1_FIFO_CTRL     =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA1_FIFO_CTRL       ));
                DISP_MSG("(070)OVL_L2_CON              =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_CON                ));
                DISP_MSG("(074)OVL_L2_SRCKEY               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_SRCKEY             ));
                DISP_MSG("(078)OVL_L2_SRC_SIZE         =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_SRC_SIZE           ));
                DISP_MSG("(07C)OVL_L2_OFFSET               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_OFFSET             ));
                DISP_MSG("(080)OVL_L2_ADDR             =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_ADDR               ));
                DISP_MSG("(084)OVL_L2_PITCH                =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_PITCH              ));
                DISP_MSG("(100)OVL_RDMA2_CTRL          =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA2_CTRL            ));
                DISP_MSG("(104)OVL_RDMA2_MEM_START_TRIG    =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA2_MEM_START_TRIG  ));
                DISP_MSG("(108)OVL_RDMA2_MEM_GMC_SETTING   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING ));
                DISP_MSG("(10C)OVL_RDMA2_MEM_SLOW_CON  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA2_MEM_SLOW_CON    ));
                DISP_MSG("(110)OVL_RDMA2_FIFO_CTRL     =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA2_FIFO_CTRL       ));
                DISP_MSG("(090)OVL_L3_CON              =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_CON                ));
                DISP_MSG("(094)OVL_L3_SRCKEY               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_SRCKEY             ));
                DISP_MSG("(098)OVL_L3_SRC_SIZE         =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_SRC_SIZE           ));
                DISP_MSG("(09C)OVL_L3_OFFSET               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_OFFSET             ));
                DISP_MSG("(0A0)OVL_L3_ADDR             =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_ADDR               ));
                DISP_MSG("(0A4)OVL_L3_PITCH                =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_PITCH              ));
                DISP_MSG("(120)OVL_RDMA3_CTRL          =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA3_CTRL            ));
                DISP_MSG("(124)OVL_RDMA3_MEM_START_TRIG    =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA3_MEM_START_TRIG  ));
                DISP_MSG("(128)OVL_RDMA3_MEM_GMC_SETTING   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING ));
                DISP_MSG("(12C)OVL_RDMA3_MEM_SLOW_CON  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA3_MEM_SLOW_CON    ));
                DISP_MSG("(130)OVL_RDMA3_FIFO_CTRL     =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA3_FIFO_CTRL       ));
                DISP_MSG("(1C4)OVL_DEBUG_MON_SEL           =0x%x \n", DISP_REG_GET(DISP_REG_OVL_DEBUG_MON_SEL         ));
                DISP_MSG("(1C4)OVL_RDMA0_MEM_GMC_SETTING2 =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING2));
                DISP_MSG("(1C8)OVL_RDMA1_MEM_GMC_SETTING2 =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING2));
                DISP_MSG("(1CC)OVL_RDMA2_MEM_GMC_SETTING2 =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING2));
                DISP_MSG("(1D0)OVL_RDMA3_MEM_GMC_SETTING2 =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING2));
                DISP_MSG("(240)OVL_FLOW_CTRL_DBG           =0x%x \n", DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG         ));
                DISP_MSG("(244)OVL_ADDCON_DBG          =0x%x \n", DISP_REG_GET(DISP_REG_OVL_ADDCON_DBG            ));
                DISP_MSG("(248)OVL_OUTMUX_DBG          =0x%x \n", DISP_REG_GET(DISP_REG_OVL_OUTMUX_DBG            ));
                DISP_MSG("(24C)OVL_RDMA0_DBG               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA0_DBG             ));
                DISP_MSG("(250)OVL_RDMA1_DBG               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA1_DBG             ));
                DISP_MSG("(254)OVL_RDMA2_DBG               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA2_DBG             ));
                DISP_MSG("(258)OVL_RDMA3_DBG               =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA3_DBG             ));
                break;
                 
            case DISP_MODULE_COLOR:  
            DISP_MSG("===== DISP Color Reg Dump: ============\n");  
            DISP_MSG("(0x0F00)DISP_COLOR_START             =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_START           ));
            DISP_MSG("(0x0F04)DISP_COLOR_INTEN             =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_INTEN           ));
            DISP_MSG("(0x0F08)DISP_COLOR_INTSTA            =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_INTSTA          ));
            DISP_MSG("(0x0F0C)DISP_COLOR_OUT_SEL           =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_OUT_SEL         ));
            DISP_MSG("(0x0F10)DISP_COLOR_FRAME_DONE_DEL    =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_FRAME_DONE_DEL  ));
            DISP_MSG("(0x0F14)DISP_COLOR_CRC               =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_CRC             ));
            DISP_MSG("(0x0F18)DISP_COLOR_SW_SCRATCH        =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_SW_SCRATCH        ));
            DISP_MSG("(0x0F20)DISP_COLOR_RDY_SEL           =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_RDY_SEL           ));
            DISP_MSG("(0x0F24)DISP_COLOR_RDY_SEL_EN        =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_RDY_SEL_EN        ));
            DISP_MSG("(0x0F28)DISP_COLOR_CK_ON             =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_CK_ON             ));
            DISP_MSG("(0x0F50)DISP_COLOR_INTERNAL_IP_WIDTH =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_INTERNAL_IP_WIDTH ));
            DISP_MSG("(0x0F54)DISP_COLOR_INTERNAL_IP_HEIGHT=0x%x \n", DISP_REG_GET(DISP_REG_COLOR_INTERNAL_IP_HEIGHT));
            DISP_MSG("(0x0F60)DISP_COLOR_CM1_EN            =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_CM1_EN            ));
            break;
                     
            case DISP_MODULE_BLS:    
                DISP_MSG("===== DISP BLS Reg Dump: ============\n");
                DISP_MSG("(0x0 )BLS_EN              =0x%x \n", DISP_REG_GET(DISP_REG_BLS_EN               ));
                DISP_MSG("(0x4 )BLS_RST               =0x%x \n", DISP_REG_GET(DISP_REG_BLS_RST                ));
                DISP_MSG("(0x8 )BLS_INTEN           =0x%x \n", DISP_REG_GET(DISP_REG_BLS_INTEN              ));
                DISP_MSG("(0xC )BLS_INTSTA          =0x%x \n", DISP_REG_GET(DISP_REG_BLS_INTSTA           ));
                DISP_MSG("(0x10)BLS_BLS_SETTING     =0x%x \n", DISP_REG_GET(DISP_REG_BLS_BLS_SETTING      ));
                DISP_MSG("(0x14)BLS_FANA_SETTING      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_FANA_SETTING     ));
                DISP_MSG("(0x18)BLS_SRC_SIZE          =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SRC_SIZE         ));
                DISP_MSG("(0x20)BLS_GAIN_SETTING      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_GAIN_SETTING     ));
                DISP_MSG("(0x24)BLS_MANUAL_GAIN       =0x%x \n", DISP_REG_GET(DISP_REG_BLS_MANUAL_GAIN        ));
                DISP_MSG("(0x28)BLS_MANUAL_MAXCLR   =0x%x \n", DISP_REG_GET(DISP_REG_BLS_MANUAL_MAXCLR      ));
                DISP_MSG("(0x30)BLS_GAMMA_SETTING   =0x%x \n", DISP_REG_GET(DISP_REG_BLS_GAMMA_SETTING      ));
                DISP_MSG("(0x34)BLS_GAMMA_BOUNDARY  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_GAMMA_BOUNDARY   ));
                DISP_MSG("(0x38)BLS_LUT_UPDATE      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_LUT_UPDATE       ));
                //DISP_MSG("(0x3C)BLS_LGMA_SETTING    =0x%x \n", DISP_REG_GET(DISP_REG_BLS_LGMA_SETTING     ));
                //DISP_MSG("(0x40)BLS_LGMA_SEGMENT      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_LGMA_SEGMENT     ));
                DISP_MSG("(0x60)BLS_MAXCLR_THD      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_MAXCLR_THD       ));
                DISP_MSG("(0x64)BLS_DISTPT_THD      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_DISTPT_THD       ));
                DISP_MSG("(0x68)BLS_MAXCLR_LIMIT      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_MAXCLR_LIMIT     ));
                DISP_MSG("(0x6C)BLS_DISTPT_LIMIT      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_DISTPT_LIMIT     ));
                DISP_MSG("(0x70)BLS_AVE_SETTING       =0x%x \n", DISP_REG_GET(DISP_REG_BLS_AVE_SETTING        ));
                DISP_MSG("(0x74)BLS_AVE_LIMIT       =0x%x \n", DISP_REG_GET(DISP_REG_BLS_AVE_LIMIT          ));
                DISP_MSG("(0x78)BLS_DISTPT_SETTING  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_DISTPT_SETTING   ));
                DISP_MSG("(0x7C)BLS_HIS_CLEAR       =0x%x \n", DISP_REG_GET(DISP_REG_BLS_HIS_CLEAR          ));
                DISP_MSG("(0x80)BLS_SC_DIFF_THD     =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SC_DIFF_THD      ));
                DISP_MSG("(0x84)BLS_SC_BIN_THD      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SC_BIN_THD       ));
                DISP_MSG("(0x88)BLS_MAXCLR_GRADUAL  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_MAXCLR_GRADUAL   ));
                DISP_MSG("(0x8C)BLS_DISTPT_GRADUAL  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_DISTPT_GRADUAL   ));
                DISP_MSG("(0x90)BLS_FAST_IIR_XCOEFF =0x%x \n", DISP_REG_GET(DISP_REG_BLS_FAST_IIR_XCOEFF  ));
                DISP_MSG("(0x94)BLS_FAST_IIR_YCOEFF =0x%x \n", DISP_REG_GET(DISP_REG_BLS_FAST_IIR_YCOEFF  ));
                DISP_MSG("(0x98)BLS_SLOW_IIR_XCOEFF =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SLOW_IIR_XCOEFF  ));
                DISP_MSG("(0x9C)BLS_SLOW_IIR_YCOEFF =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SLOW_IIR_YCOEFF  ));
                DISP_MSG("(0xA0)BLS_PWM_DUTY          =0x%x \n", DISP_REG_GET(DISP_REG_BLS_PWM_DUTY         ));
                DISP_MSG("(0xA4)BLS_PWM_GRADUAL     =0x%x \n", DISP_REG_GET(DISP_REG_BLS_PWM_GRADUAL      ));
                DISP_MSG("(0xA8)BLS_PWM_CON         =0x%x \n", DISP_REG_GET(DISP_REG_BLS_PWM_CON          ));
                DISP_MSG("(0xAC)BLS_PWM_MANUAL      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_PWM_MANUAL       ));
                DISP_MSG("(0xB0)BLS_DEBUG           =0x%x \n", DISP_REG_GET(DISP_REG_BLS_DEBUG              ));
                DISP_MSG("(0xB4)BLS_PATTERN         =0x%x \n", DISP_REG_GET(DISP_REG_BLS_PATTERN          ));
                DISP_MSG("(0xB8)BLS_CHKSUM          =0x%x \n", DISP_REG_GET(DISP_REG_BLS_CHKSUM           ));
                //DISP_MSG("(0x100)BLS_HIS_BIN        =0x%x \n", DISP_REG_GET(DISP_REG_BLS_HIS_BIN          ));
                DISP_MSG("(0x200)BLS_PWM_DUTY_RD    =0x%x \n", DISP_REG_GET(DISP_REG_BLS_PWM_DUTY_RD      ));
                DISP_MSG("(0x204)BLS_FRAME_AVE_RD   =0x%x \n", DISP_REG_GET(DISP_REG_BLS_FRAME_AVE_RD     ));
                DISP_MSG("(0x208)BLS_MAXCLR_RD      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_MAXCLR_RD          ));
                DISP_MSG("(0x20C)BLS_DISTPT_RD      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_DISTPT_RD          ));
                DISP_MSG("(0x210)BLS_GAIN_RD        =0x%x \n", DISP_REG_GET(DISP_REG_BLS_GAIN_RD          ));
                DISP_MSG("(0x214)BLS_SC_RD          =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SC_RD            ));
                //DISP_MSG("(0x300)BLS_LUMINANCE      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_LUMINANCE        ));
                //DISP_MSG("(0x384)BLS_LUMINANCE_255  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_LUMINANCE_255    ));
                //DISP_MSG("(0x400)BLS_GAMMA_LUT      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_GAMMA_LUT        ));
            break;
                     
            case DISP_MODULE_WDMA0:
                DISP_MSG("===== DISP WDMA%d Reg Dump: ============\n", index);   
                DISP_MSG("(000)WDMA_INTEN      =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_INTEN        ));
                DISP_MSG("(004)WDMA_INTSTA     =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_INTSTA       ));
                DISP_MSG("(008)WDMA_EN         =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_EN           ));
                DISP_MSG("(00C)WDMA_RST            =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_RST          ));
                DISP_MSG("(010)WDMA_SMI_CON        =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_SMI_CON      ));
                DISP_MSG("(014)WDMA_CFG            =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_CFG          ));
                DISP_MSG("(018)WDMA_SRC_SIZE       =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_SRC_SIZE     ));
                DISP_MSG("(01C)WDMA_CLIP_SIZE  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_CLIP_SIZE    ));
                DISP_MSG("(020)WDMA_CLIP_COORD =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_CLIP_COORD   ));
                DISP_MSG("(024)WDMA_DST_ADDR       =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DST_ADDR     ));
                DISP_MSG("(028)WDMA_DST_W_IN_BYTE =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DST_W_IN_BYTE));
                DISP_MSG("(02C)WDMA_ALPHA      =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_ALPHA        ));
                DISP_MSG("(030)WDMA_BUF_ADDR       =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_BUF_ADDR     ));
                DISP_MSG("(034)WDMA_STA            =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_STA          ));
                DISP_MSG("(038)WDMA_BUF_CON1       =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_BUF_CON1     ));
                DISP_MSG("(03C)WDMA_BUF_CON2       =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_BUF_CON2     ));
                DISP_MSG("(040)WDMA_C00            =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_C00          ));
                DISP_MSG("(044)WDMA_C02            =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_C02          ));
                DISP_MSG("(048)WDMA_C10            =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_C10          ));
                DISP_MSG("(04C)WDMA_C12            =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_C12          ));
                DISP_MSG("(050)WDMA_C20            =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_C20          ));
                DISP_MSG("(054)WDMA_C22            =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_C22          ));
                DISP_MSG("(058)WDMA_PRE_ADD0       =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_PRE_ADD0     ));
                DISP_MSG("(05C)WDMA_PRE_ADD2       =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_PRE_ADD2     ));
                DISP_MSG("(060)WDMA_POST_ADD0  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_POST_ADD0    ));
                DISP_MSG("(064)WDMA_POST_ADD2  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_POST_ADD2    ));
                DISP_MSG("(070)WDMA_DST_U_ADDR =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DST_U_ADDR   ));
                DISP_MSG("(074)WDMA_DST_V_ADDR =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DST_V_ADDR   ));
                DISP_MSG("(078)WDMA_DST_UV_PITCH   =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DST_UV_PITCH     ));
                DISP_MSG("(090)WDMA_DITHER_CON =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DITHER_CON   ));
                DISP_MSG("(0A0)WDMA_FLOW_CTRL_DBG =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_FLOW_CTRL_DBG));
                DISP_MSG("(0A4)WDMA_EXEC_DBG       =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_EXEC_DBG     ));
                DISP_MSG("(0A8)WDMA_CLIP_DBG       =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_CLIP_DBG     ));
                break;    
               
            case DISP_MODULE_RDMA1:  
                DISP_MSG("===== DISP RDMA1 Reg Dump: ======== \n" );
                DISP_MSG("(000)RDMA_INT_ENABLE   =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_INT_ENABLE     ));
                DISP_MSG("(004)RDMA_INT_STATUS   =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_INT_STATUS     ));
                DISP_MSG("(010)RDMA_GLOBAL_CON   =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON     ));
                DISP_MSG("(014)RDMA_SIZE_CON_0   =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_0     ));
                DISP_MSG("(018)RDMA_SIZE_CON_1   =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_1     ));
                DISP_MSG("(024)RDMA_MEM_CON         =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_CON         ));
                DISP_MSG("(028)RDMA_MEM_START_ADDR  =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_START_ADDR ));
                DISP_MSG("(02C)RDMA_MEM_SRC_PITCH   =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_SRC_PITCH  ));
                DISP_MSG("(030)RDMA_MEM_GMC_SETTING_0 =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_0));
                DISP_MSG("(034)RDMA_MEM_SLOW_CON    =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_SLOW_CON   ));
                DISP_MSG("(030)RDMA_MEM_GMC_SETTING_1 =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_1));
                DISP_MSG("(040)RDMA_FIFO_CON        =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_FIFO_CON    ));
                DISP_MSG("(054)RDMA_CF_00        =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_00          ));
                DISP_MSG("(058)RDMA_CF_01        =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_01          ));
                DISP_MSG("(05C)RDMA_CF_02        =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_02          ));
                DISP_MSG("(060)RDMA_CF_10        =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_10          ));
                DISP_MSG("(064)RDMA_CF_11        =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_11          ));
                DISP_MSG("(068)RDMA_CF_12        =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_12          ));
                DISP_MSG("(06C)RDMA_CF_20        =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_20          ));
                DISP_MSG("(070)RDMA_CF_21        =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_21          ));
                DISP_MSG("(074)RDMA_CF_22        =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_22          ));
                DISP_MSG("(078)RDMA_CF_PRE_ADD0      =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_PRE_ADD0    ));
                DISP_MSG("(07C)RDMA_CF_PRE_ADD1      =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_PRE_ADD1    ));
                DISP_MSG("(080)RDMA_CF_PRE_ADD2      =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_PRE_ADD2    ));
                DISP_MSG("(084)RDMA_CF_POST_ADD0     =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_POST_ADD0   ));
                DISP_MSG("(088)RDMA_CF_POST_ADD1     =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_POST_ADD1   ));
                DISP_MSG("(08C)RDMA_CF_POST_ADD2     =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_CF_POST_ADD2   ));      
                DISP_MSG("(090)RDMA_DUMMY            =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_DUMMY          ));
                DISP_MSG("(094)RDMA_DEBUG_OUT_SEL    =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_DEBUG_OUT_SEL  ));
                break;
    
            case DISP_MODULE_DPI0:   
                DISP_MSG("===== DISP DPI0 Reg Dump: ============\n");
                break;
                     
            case DISP_MODULE_DSI_VDO:   
            case DISP_MODULE_DSI_CMD:
            break;
    
            case DISP_MODULE_CONFIG:
                DISP_MSG("===== DISP DISP_REG_MM_CONFIG Reg Dump: ============\n");
                DISP_MSG("clk tree: 0xf0000000=0x%x, 0xf0000050=0x%x\n", 
                	      *(volatile unsigned int*)(0xf0000000),
                	      *(volatile unsigned int*)(0xf0000050));
                DISP_MSG("(0x01c)CAM_MDP_MOUT_EN         =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_CAM_MDP_MOUT_EN          ));     
                DISP_MSG("(0x020)MDP_RDMA_MOUT_EN        =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MDP_RDMA_MOUT_EN         ));    
                DISP_MSG("(0x024)MDP_RSZ0_MOUT_EN        =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MDP_RSZ0_MOUT_EN         ));    
                DISP_MSG("(0x028)MDP_RSZ1_MOUT_EN        =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MDP_RSZ1_MOUT_EN         ));    
                DISP_MSG("(0x02c)MDP_TDSHP_MOUT_EN       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MDP_TDSHP_MOUT_EN        ));    
                DISP_MSG("(0x030)DISP_OVL_MOUT_EN        =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN         ));    
                DISP_MSG("(0x034)MMSYS_MOUT_RST          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MOUT_RST           ));    
                DISP_MSG("(0x038)MDP_RSZ0_SEL            =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MDP_RSZ0_SEL             ));    
                DISP_MSG("(0x03c)MDP_RSZ1_SEL            =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MDP_RSZ1_SEL             ));    
                DISP_MSG("(0x040)MDP_TDSHP_SEL           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MDP_TDSHP_SEL            ));    
                DISP_MSG("(0x044)MDP_WROT_SEL            =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MDP_WROT_SEL             ));    
                DISP_MSG("(0x048)MDP_WDMA_SEL            =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MDP_WDMA_SEL             ));    
                DISP_MSG("(0x04c)DISP_OUT_SEL            =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_DISP_OUT_SEL             ));    
                DISP_MSG("(0x100)MMSYS_CG_CON0           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0            ));    
                DISP_MSG("(0x104)MMSYS_CG_SET0           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_SET0            ));    
                DISP_MSG("(0x108)MMSYS_CG_CLR0           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CLR0            ));    
                DISP_MSG("(0x110)MMSYS_CG_CON1           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON1            ));    
                DISP_MSG("(0x114)MMSYS_CG_SET1           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_SET1            ));    
                DISP_MSG("(0x118)MMSYS_CG_CLR1           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CLR1            ));    
                DISP_MSG("(0x120)MMSYS_HW_DCM_DIS0       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_HW_DCM_DIS0        ));    
                DISP_MSG("(0x124)MMSYS_HW_DCM_DIS_SET0   =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_HW_DCM_DIS_SET0    ));    
                DISP_MSG("(0x128)MMSYS_HW_DCM_DIS_CLR0   =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_HW_DCM_DIS_CLR0    ));    
                DISP_MSG("(0x12c)MMSYS_HW_DCM_DIS1       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_HW_DCM_DIS1        ));    
                DISP_MSG("(0x130)MMSYS_HW_DCM_DIS_SET1   =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_HW_DCM_DIS_SET1    ));    
                DISP_MSG("(0x134)MMSYS_HW_DCM_DIS_CLR1   =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_HW_DCM_DIS_CLR1    ));    
                DISP_MSG("(0x138)MMSYS_SW_RST_B          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_SW_RST_B           ));    
                DISP_MSG("(0x13c)MMSYS_LCM_RST_B         =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_LCM_RST_B          ));    
                DISP_MSG("(0x800)MMSYS_MBIST_DONE        =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_DONE         ));    
                DISP_MSG("(0x804)MMSYS_MBIST_FAIL0       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_FAIL0        ));    
                DISP_MSG("(0x808)MMSYS_MBIST_FAIL1       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_FAIL1        ));    
                DISP_MSG("(0x80C)MMSYS_MBIST_HOLDB       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_HOLDB        ));    
                DISP_MSG("(0x810)MMSYS_MBIST_MODE        =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_MODE         ));    
                DISP_MSG("(0x814)MMSYS_MBIST_BSEL0       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_BSEL0        ));    
                DISP_MSG("(0x818)MMSYS_MBIST_BSEL1       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_BSEL1        ));    
                DISP_MSG("(0x81c)MMSYS_MBIST_CON         =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_CON          ));  
                DISP_MSG("(0x820)MMSYS_MEM_DELSEL0       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MEM_DELSEL0        ));    
                DISP_MSG("(0x824)MMSYS_MEM_DELSEL1       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MEM_DELSEL1        ));    
                DISP_MSG("(0x828)MMSYS_MEM_DELSEL2       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MEM_DELSEL2        ));    
                DISP_MSG("(0x82c)MMSYS_MEM_DELSEL3       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MEM_DELSEL3        ));    
                DISP_MSG("(0x830)MMSYS_DEBUG_OUT_SEL     =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_DEBUG_OUT_SEL      ));    
                DISP_MSG("(0x840)MMSYS_DUMMY             =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_DUMMY              ));    
                break;
                
            case DISP_MODULE_MUTEX:
                DISP_MSG("===== DISP DISP_REG_MUTEX_CONFIG Reg Dump: ============\n");
                DISP_MSG("(0x0  )DISP_MUTEX_INTEN        =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTEN         ));
                DISP_MSG("(0x4  )DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA       =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA        ));
                DISP_MSG("(0x8  )DISP_REG_GET(DISP_REG_CONFIG_REG_UPD_TIMEOUT    =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_REG_UPD_TIMEOUT     ));
                DISP_MSG("(0xC  )DISP_REG_GET(DISP_REG_CONFIG_REG_COMMIT         =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_REG_COMMIT          ));
                DISP_MSG("(0x20)DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_EN           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_EN           ));
                DISP_MSG("(0x24)DISP_REG_GET(DISP_REG_CONFIG_MUTEX0              =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX0              ));
                DISP_MSG("(0x28)DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_RST          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_RST          ));
                DISP_MSG("(0x2C)DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD          ));
                DISP_MSG("(0x30)DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF          ));
                DISP_MSG("(0x40)DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_EN           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_EN           ));
                DISP_MSG("(0x44)DISP_REG_GET(DISP_REG_CONFIG_MUTEX1              =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX1              ));
                DISP_MSG("(0x48)DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_RST          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_RST          ));
                DISP_MSG("(0x4C)DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_MOD          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_MOD          ));
                DISP_MSG("(0x50)DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_SOF          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_SOF          ));
                DISP_MSG("(0x60)DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_EN           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_EN           ));
                DISP_MSG("(0x64)DISP_REG_GET(DISP_REG_CONFIG_MUTEX2              =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX2              ));
                DISP_MSG("(0x68)DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_RST          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_RST          ));
                DISP_MSG("(0x6C)DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_MOD          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_MOD          ));
                DISP_MSG("(0x70)DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_SOF          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_SOF          ));
                DISP_MSG("(0x80)DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_EN           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_EN           ));
                DISP_MSG("(0x84)DISP_REG_GET(DISP_REG_CONFIG_MUTEX3              =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX3              ));
                DISP_MSG("(0x88)DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_RST          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_RST          ));
                DISP_MSG("(0x8C)DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_MOD          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_MOD          ));
                DISP_MSG("(0x90)DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_SOF          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_SOF          ));
                DISP_MSG("(0xA0)DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_EN           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_EN           ));
                DISP_MSG("(0xA4)DISP_REG_GET(DISP_REG_CONFIG_MUTEX4              =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX4              ));
                DISP_MSG("(0xA8)DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_RST          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_RST          ));
                DISP_MSG("(0xAC)DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_MOD          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_MOD          ));
                DISP_MSG("(0xB0)DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_SOF          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_SOF          ));
                DISP_MSG("(0xC0)DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_EN           =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_EN           ));
                DISP_MSG("(0xC4)DISP_REG_GET(DISP_REG_CONFIG_MUTEX5              =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX5              ));
                DISP_MSG("(0xC8)DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_RST          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_RST          ));
                DISP_MSG("(0xCC)DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_MOD          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_MOD          ));
                DISP_MSG("(0xD0)DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_SOF          =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_SOF          ));
                DISP_MSG("(0x100)DISP_REG_GET(DISP_REG_CONFIG_MUTEX_DEBUG_OUT_SEL=0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX_DEBUG_OUT_SEL ));
            break;
            
            default: DISP_MSG("error, reg_dump, unknow module=%d \n", module);
    
        }
    
        return 0;
}
#else
int disp_dump_reg(DISP_MODULE_ENUM module)
    {
        //unsigned int size;
        //unsigned int reg_base;
        unsigned int index = 0;

        switch(module)
        {       
            case DISP_MODULE_OVL: 
                DISP_MSG("==R_OVL: ==\n");          
                DISP_MSG("(000)STA   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_STA                   ));
                DISP_MSG("(004)INTE  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_INTEN                 ));
                DISP_MSG("(008)INTS  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_INTSTA                ));
                DISP_MSG("(00C)EN    =0x%x \n", DISP_REG_GET(DISP_REG_OVL_EN                    ));
                DISP_MSG("(010)TRIG  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_TRIG                  ));
                DISP_MSG("(014)RST   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RST                   ));
                DISP_MSG("(020)ROI   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_ROI_SIZE              ));
                DISP_MSG("(020)PATH_CON =0x%x \n", DISP_REG_GET(DISP_REG_OVL_DATAPATH_CON          ));
                DISP_MSG("(028)ROI_BGCLR=0x%x \n", DISP_REG_GET(DISP_REG_OVL_ROI_BGCLR             ));
                DISP_MSG("(02C)SRC_CON  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_SRC_CON               ));
                DISP_MSG("(030)0_CON    =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_CON                ));
                DISP_MSG("(034)0_SRCKEY =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_SRCKEY             ));
                DISP_MSG("(038)0_SIZE   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_SRC_SIZE           ));
                DISP_MSG("(03C)0_OFFSET =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_OFFSET             ));
                DISP_MSG("(040)0_ADDR   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_ADDR               ));
                DISP_MSG("(044)0_PITCH  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L0_PITCH              ));
                DISP_MSG("(0C0)0_CTRL   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA0_CTRL            ));
                DISP_MSG("(0D0)0_FIFO   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA0_FIFO_CTRL     ));
                DISP_MSG("(050)1_CON    =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_CON                ));
                DISP_MSG("(054)1_SRCKEY =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_SRCKEY             ));
                DISP_MSG("(058)1_SIZE   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_SRC_SIZE           ));
                DISP_MSG("(05C)1_OFFSET =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_OFFSET             ));
                DISP_MSG("(060)1_ADDR   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_ADDR               ));
                DISP_MSG("(064)1_PITCH  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L1_PITCH              ));
                DISP_MSG("(0E0)1_CTRL   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA1_CTRL            ));
                DISP_MSG("(0F0)1_FIFO   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA1_FIFO_CTRL       ));
                DISP_MSG("(070)2_CON    =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_CON                ));
                DISP_MSG("(074)2_SRCKEY =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_SRCKEY             ));
                DISP_MSG("(078)2_SIZE   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_SRC_SIZE           ));
                DISP_MSG("(07C)2_OFFSET =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_OFFSET             ));
                DISP_MSG("(080)2_ADDR   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_ADDR               ));
                DISP_MSG("(084)2_PITCH  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L2_PITCH              ));
                DISP_MSG("(100)2_CTRL   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA2_CTRL            ));
                DISP_MSG("(110)2_FIFO   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA2_FIFO_CTRL       ));
                DISP_MSG("(090)3_CON    =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_CON                ));
                DISP_MSG("(094)3_SRCKEY =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_SRCKEY             ));
                DISP_MSG("(098)3_SIZE   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_SRC_SIZE           ));
                DISP_MSG("(09C)3_OFFSET =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_OFFSET             ));
                DISP_MSG("(0A0)3_ADDR   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_ADDR               ));
                DISP_MSG("(0A4)3_PITCH  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_L3_PITCH              ));
                DISP_MSG("(120)3_CTRL   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA3_CTRL            ));
                DISP_MSG("(130)3_FIFO   =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA3_FIFO_CTRL       ));
                DISP_MSG("(1C4)MON_SEL  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_DEBUG_MON_SEL         ));
                DISP_MSG("(240)FLOW_CTRL=0x%x \n", DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG         ));
                DISP_MSG("(244)ADDCON =0x%x \n", DISP_REG_GET(DISP_REG_OVL_ADDCON_DBG            ));
                DISP_MSG("(248)OUTMUX =0x%x \n", DISP_REG_GET(DISP_REG_OVL_OUTMUX_DBG            ));
                DISP_MSG("(24C)0_DBG  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA0_DBG             ));
                DISP_MSG("(250)1_DBG  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA1_DBG             ));
                DISP_MSG("(254)2_DBG  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA2_DBG             ));
                DISP_MSG("(258)3_DBG  =0x%x \n", DISP_REG_GET(DISP_REG_OVL_RDMA3_DBG             ));
                break;
                 
            case DISP_MODULE_COLOR:  
                DISP_MSG("== R_COLOR: ==\n");  
                DISP_MSG("(F00)START =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_START           ));
                DISP_MSG("(F04)INTE  =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_INTEN           ));
                DISP_MSG("(F08)INTS  =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_INTSTA          ));
                DISP_MSG("(F0C)OUT_SEL =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_OUT_SEL         ));
                DISP_MSG("(F10)DONE_DEL =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_FRAME_DONE_DEL  ));
                DISP_MSG("(F14)CRC   =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_CRC             ));
                DISP_MSG("(F18)SW_SCRATCH =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_SW_SCRATCH        ));
                DISP_MSG("(F20)RDY_SEL =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_RDY_SEL           ));
                DISP_MSG("(F24)RDY_SEL_EN =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_RDY_SEL_EN        ));
                DISP_MSG("(F28)CK_ON   =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_CK_ON             ));
                DISP_MSG("(F50)IN_IP_W =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_INTERNAL_IP_WIDTH ));
                DISP_MSG("(F54)IN_IP_H =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_INTERNAL_IP_HEIGHT));
                DISP_MSG("(F60)CM1_EN  =0x%x \n", DISP_REG_GET(DISP_REG_COLOR_CM1_EN            ));
                break;
                         
            case DISP_MODULE_BLS:    
                DISP_MSG("== R_BLS==\n");
                DISP_MSG("(0 )EN   =0x%x \n", DISP_REG_GET(DISP_REG_BLS_EN               ));
                DISP_MSG("(4 )RST  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_RST                ));
                DISP_MSG("(8 )INTE =0x%x \n", DISP_REG_GET(DISP_REG_BLS_INTEN              ));
                DISP_MSG("(C )INTS =0x%x \n", DISP_REG_GET(DISP_REG_BLS_INTSTA           ));
                DISP_MSG("(10)BLS_SETTING   =0x%x \n", DISP_REG_GET(DISP_REG_BLS_BLS_SETTING      ));
                DISP_MSG("(14)FANA_SETTING  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_FANA_SETTING     ));
                DISP_MSG("(18)SRC_SIZE      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SRC_SIZE         ));
                DISP_MSG("(20)GAIN_SETTING  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_GAIN_SETTING     ));
                DISP_MSG("(24)MANUAL_GAIN   =0x%x \n", DISP_REG_GET(DISP_REG_BLS_MANUAL_GAIN        ));
                DISP_MSG("(28)MANUAL_MAXCLR =0x%x \n", DISP_REG_GET(DISP_REG_BLS_MANUAL_MAXCLR      ));
                DISP_MSG("(30)GMA_SET   =0x%x \n", DISP_REG_GET(DISP_REG_BLS_GAMMA_SETTING      ));
                DISP_MSG("(34)GMA_BOUND =0x%x \n", DISP_REG_GET(DISP_REG_BLS_GAMMA_BOUNDARY   ));
                DISP_MSG("(38)LUT_UPDATE  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_LUT_UPDATE       ));
                DISP_MSG("(60)MAXCLR_THD  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_MAXCLR_THD       ));
                DISP_MSG("(64)DISTPT_THD  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_DISTPT_THD       ));
                DISP_MSG("(68)MAXCLR_LIMIT=0x%x \n", DISP_REG_GET(DISP_REG_BLS_MAXCLR_LIMIT     ));
                DISP_MSG("(6C)DISTPT_LIMIT=0x%x \n", DISP_REG_GET(DISP_REG_BLS_DISTPT_LIMIT     ));
                DISP_MSG("(70)AVE_SETTING =0x%x \n", DISP_REG_GET(DISP_REG_BLS_AVE_SETTING        ));
                DISP_MSG("(74)AVE_LIMIT   =0x%x \n", DISP_REG_GET(DISP_REG_BLS_AVE_LIMIT          ));
                DISP_MSG("(78)DISTPT_SET  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_DISTPT_SETTING   ));
                DISP_MSG("(7C)HIS_CLEAR   =0x%x \n", DISP_REG_GET(DISP_REG_BLS_HIS_CLEAR          ));
                DISP_MSG("(80)SC_DIFF_THD =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SC_DIFF_THD      ));
                DISP_MSG("(84)SC_BIN_THD  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SC_BIN_THD       ));
                DISP_MSG("(88)MAXCLR_GRADUAL=0x%x \n", DISP_REG_GET(DISP_REG_BLS_MAXCLR_GRADUAL   ));
                DISP_MSG("(8C)DISTPT_GRADUAL=0x%x \n", DISP_REG_GET(DISP_REG_BLS_DISTPT_GRADUAL   ));
                DISP_MSG("(90)FAST_IIR_X  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_FAST_IIR_XCOEFF  ));
                DISP_MSG("(94)FAST_IIR_Y  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_FAST_IIR_YCOEFF  ));
                DISP_MSG("(98)SLOW_IIR_X  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SLOW_IIR_XCOEFF  ));
                DISP_MSG("(9C)SLOW_IIR_Y  =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SLOW_IIR_YCOEFF  ));
                DISP_MSG("(A0)PWM_DUTY   =0x%x \n", DISP_REG_GET(DISP_REG_BLS_PWM_DUTY         ));
                DISP_MSG("(A4)PWM_GRADUAL=0x%x \n", DISP_REG_GET(DISP_REG_BLS_PWM_GRADUAL      ));
                DISP_MSG("(A8)PWM_CON    =0x%x \n", DISP_REG_GET(DISP_REG_BLS_PWM_CON          ));
                DISP_MSG("(AC)PWM_MANUAL =0x%x \n", DISP_REG_GET(DISP_REG_BLS_PWM_MANUAL       ));
                DISP_MSG("(B0)DEBUG      =0x%x \n", DISP_REG_GET(DISP_REG_BLS_DEBUG              ));
                DISP_MSG("(B4)PATTERN    =0x%x \n", DISP_REG_GET(DISP_REG_BLS_PATTERN          ));
                DISP_MSG("(B8)CHKSUM     =0x%x \n", DISP_REG_GET(DISP_REG_BLS_CHKSUM           ));
                DISP_MSG("(200)PWM_DUTY_RD =0x%x \n", DISP_REG_GET(DISP_REG_BLS_PWM_DUTY_RD      ));
                DISP_MSG("(204)FRAME_AVE_RD=0x%x \n", DISP_REG_GET(DISP_REG_BLS_FRAME_AVE_RD     ));
                DISP_MSG("(208)MAXCLR_RD =0x%x \n", DISP_REG_GET(DISP_REG_BLS_MAXCLR_RD          ));
                DISP_MSG("(20C)DISTPT_RD =0x%x \n", DISP_REG_GET(DISP_REG_BLS_DISTPT_RD          ));
                DISP_MSG("(210)GAIN_RD   =0x%x \n", DISP_REG_GET(DISP_REG_BLS_GAIN_RD          ));
                DISP_MSG("(214)SC_RD     =0x%x \n", DISP_REG_GET(DISP_REG_BLS_SC_RD            ));                
            break;
                     
            case DISP_MODULE_WDMA0:
                DISP_MSG("== R_WDMA ==\n", index);   
                DISP_MSG("(000)INTE =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_INTEN        ));
                DISP_MSG("(004)INTS =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_INTSTA       ));
                DISP_MSG("(008)EN   =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_EN           ));
                DISP_MSG("(00C)RST  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_RST          ));
                DISP_MSG("(010)SMI_CON=0x%x \n", DISP_REG_GET(DISP_REG_WDMA_SMI_CON      ));
                DISP_MSG("(014)CFG  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_CFG          ));
                DISP_MSG("(018)SRC_SIZE =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_SRC_SIZE     ));
                DISP_MSG("(01C)CLIP  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_CLIP_SIZE    ));
                DISP_MSG("(020)CLIP_COORD =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_CLIP_COORD   ));
                DISP_MSG("(024)ADDR  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DST_ADDR     ));
                DISP_MSG("(028)DST_W_IN_BYTE =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DST_W_IN_BYTE));
                DISP_MSG("(02C)ALPHA  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_ALPHA        ));
                DISP_MSG("(030)BUF_ADDR =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_BUF_ADDR     ));
                DISP_MSG("(034)STA   =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_STA          ));
                DISP_MSG("(038)CON1  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_BUF_CON1     ));
                DISP_MSG("(03C)CON2  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_BUF_CON2     ));
                DISP_MSG("(058)PRE_ADD0 =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_PRE_ADD0     ));
                DISP_MSG("(05C)PRE_ADD2 =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_PRE_ADD2     ));
                DISP_MSG("(060)POST_ADD0=0x%x \n", DISP_REG_GET(DISP_REG_WDMA_POST_ADD0    ));
                DISP_MSG("(064)POST_ADD2=0x%x \n", DISP_REG_GET(DISP_REG_WDMA_POST_ADD2    ));
                DISP_MSG("(070)U_AD =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DST_U_ADDR   ));
                DISP_MSG("(074)V_AD =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DST_V_ADDR   ));
                DISP_MSG("(078)UV_PITCH   =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DST_UV_PITCH   ));
                DISP_MSG("(090)DITHER_CON =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_DITHER_CON   ));
                DISP_MSG("(0A0)FLOW_CTRL_DBG =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_FLOW_CTRL_DBG));
                DISP_MSG("(0A4)EXEC_DBG  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_EXEC_DBG     ));
                DISP_MSG("(0A8)CLIP_DBG  =0x%x \n", DISP_REG_GET(DISP_REG_WDMA_CLIP_DBG     ));
                break;    
               
            case DISP_MODULE_RDMA1:  
                DISP_MSG("==R_WDMA== \n" );
                DISP_MSG("(000)INTE =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_INT_ENABLE     ));
                DISP_MSG("(004)INTS =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_INT_STATUS     ));
                DISP_MSG("(010)G_CON =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON     ));
                DISP_MSG("(014)SIZE0 =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_0     ));
                DISP_MSG("(018)SIZE1 =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_1     ));
                DISP_MSG("(024)M_CON =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_CON         ));
                DISP_MSG("(028)M_START_ADDR =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_START_ADDR ));
                DISP_MSG("(02C)M_SRC_PITCH =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_SRC_PITCH  ));
                DISP_MSG("(030)M_GMC_SET_0 =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_0));
                DISP_MSG("(034)M_SLOW_CON  =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_SLOW_CON   ));
                DISP_MSG("(030)M_GMC_SET_1 =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_1));
                DISP_MSG("(040)FIFO_CON  =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_FIFO_CON    ));
                DISP_MSG("(090)DUMMY    =0x%x \n", DISP_REG_GET(DISP_REG_RDMA_DUMMY          ));
                DISP_MSG("(094)DBG_OUT_SEL=0x%x \n", DISP_REG_GET(DISP_REG_RDMA_DEBUG_OUT_SEL  ));
                break;
    
            case DISP_MODULE_DPI0:   
                DISP_MSG("== DISP DPI0 Reg Dump: ==\n");
                break;
                     
            case DISP_MODULE_DSI_VDO:   
            case DISP_MODULE_DSI_CMD:
            break;
    
            case DISP_MODULE_CONFIG:
                DISP_MSG("==R_CONFIG ==\n");
                DISP_MSG("(030)OVL_OUT =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN      ));                                               
                DISP_MSG("(04c)OUT_SEL =0x%x \n",   DISP_REG_GET(DISP_REG_CONFIG_DISP_OUT_SEL          ));    
                DISP_MSG("(100)CGCON0 =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0          ));    
                DISP_MSG("(110)CGCON1 =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON1          ));    
                DISP_MSG("(120)DCM_DIS0 =0x%x \n",   DISP_REG_GET(DISP_REG_CONFIG_MMSYS_HW_DCM_DIS0      ));    
                DISP_MSG("(12c)DCM_DIS1 =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_HW_DCM_DIS1      ));    
               DISP_MSG("(138)SW_RST_B  =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_SW_RST_B         ));    
                DISP_MSG("(13c)LCM_RST_B=0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_LCM_RST_B        ));    
                DISP_MSG("(800)MT_DONE  =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_DONE       ));    
                DISP_MSG("(804)MT_F0 =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_FAIL0      ));    
                DISP_MSG("(808)MT_F1 =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_FAIL1      ));    
                DISP_MSG("(80C)MT_HOLDB =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_HOLDB      ));    
                DISP_MSG("(810)MT_MODE =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_MODE       ));    
                DISP_MSG("(814)MT_BL0 =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_BSEL0      ));    
                DISP_MSG("(818)MT_BL1 =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_BSEL1      ));    
                DISP_MSG("(81c)MT_CON  =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MBIST_CON        ));  
                DISP_MSG("(820)M_S0 =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MEM_DELSEL0      ));    
                DISP_MSG("(824)M_S1 =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MEM_DELSEL1      ));    
                DISP_MSG("(828)M_S2 =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MEM_DELSEL2      ));    
                DISP_MSG("(82c)M_S3 =0x%x \n",      DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MEM_DELSEL3      ));    
                DISP_MSG("(830)DEBUG_OUT_SEL =0x%x \n",    DISP_REG_GET(DISP_REG_CONFIG_MMSYS_DEBUG_OUT_SEL    ));    
                DISP_MSG("(840)DUMMY  =0x%x \n",    DISP_REG_GET(DISP_REG_CONFIG_MMSYS_DUMMY            ));    
                break;
                
            case DISP_MODULE_MUTEX:
                DISP_MSG("== R_Mutex: ==\n");
                DISP_MSG("(00)INTEN  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTEN     ));
                DISP_MSG("(04)INTSTA =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA    ));
                DISP_MSG("(08)TIMEOUT =0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_REG_UPD_TIMEOUT ));
                DISP_MSG("(0C)COMMIT =0x%x \n",     DISP_REG_GET(DISP_REG_CONFIG_REG_COMMIT      ));
                DISP_MSG("(20)0_EN   =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_EN       ));
                DISP_MSG("(24)0      =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX0          ));
                DISP_MSG("(28)0_RST  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_RST      ));
                DISP_MSG("(2C)0_MOD  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD      ));
                DISP_MSG("(30)0_SOF  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF      ));
                DISP_MSG("(40)1_EN   =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_EN       ));
                DISP_MSG("(44)1      =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX1          ));
                DISP_MSG("(48)1_RST  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_RST      ));
                DISP_MSG("(4C)1_MOD  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_MOD      ));
                DISP_MSG("(50)1_SOF  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_SOF      ));
                DISP_MSG("(60)2_EN   =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_EN       ));
                DISP_MSG("(64)2      =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX2          ));
                DISP_MSG("(68)2_RST  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_RST      ));
                DISP_MSG("(6C)2_MOD  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_MOD      ));
                DISP_MSG("(70)2_SOF  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_SOF      ));
                DISP_MSG("(80)3_EN   =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_EN       ));
                DISP_MSG("(84)3      =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX3          ));
                DISP_MSG("(88)3_RST  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_RST      ));
                DISP_MSG("(8C)3_MOD  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_MOD      ));
                DISP_MSG("(90)3_SOF  =0x%x \n",          DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_SOF      ));
                DISP_MSG("(100)DEBUG_OUT_SEL=0x%x \n",   DISP_REG_GET(DISP_REG_CONFIG_MUTEX_DEBUG_OUT_SEL ));
            break;
            
            default: DISP_MSG("error, reg_dump, unknow module=%d \n", module);
    
        }
    
        return 0;
}

#endif

int disp_module_clock_on(DISP_MODULE_ENUM module, char* caller_name)
{
    return 0;
}

int disp_module_clock_off(DISP_MODULE_ENUM module, char* caller_name)
{
    return 0;
}


module_init(disp_init);
module_exit(disp_exit);
MODULE_AUTHOR("Tzu-Meng, Chung <Tzu-Meng.Chung@mediatek.com>");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");
