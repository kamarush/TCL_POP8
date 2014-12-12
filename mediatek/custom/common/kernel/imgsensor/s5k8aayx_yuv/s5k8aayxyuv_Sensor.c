/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
//#include <windows.h>
//#include <memory.h>
//#include <nkintr.h>
//#include <ceddk.h>
//#include <ceddk_exp.h>

//#include "kal_release.h"
//#include "i2c_exp.h"
//#include "gpio_exp.h"
//#include "msdk_exp.h"
//#include "msdk_sensor_exp.h"
//#include "msdk_isp_exp.h"
//#include "base_regs.h"
//#include "Sensor.h"
//#include "camera_sensor_para.h"
//#include "CameraCustomized.h"

//s_porting add
//s_porting add
//s_porting add
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/xlog.h>
#include <asm/atomic.h>
#include <asm/io.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"


#include "s5k8aayxyuv_Sensor.h"
#include "s5k8aayxyuv_Camera_Sensor_para.h"
#include "s5k8aayxyuv_CameraCustomized.h"

#define S5K8AAYX_DEBUG
#ifdef S5K8AAYX_DEBUG
//#define SENSORDB printk
#define SENSORDB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[S5K8AAYX]", fmt, ##arg)
#else
#define SENSORDB(x,...)
#endif

typedef struct
{
    UINT16  iSensorVersion;
    UINT16  iNightMode;
    UINT16  iWB;
    UINT16  iEffect;
    UINT16  iEV;
    UINT16  iBanding;
    UINT16  iMirror;
    UINT16  iFrameRate;
} S5K8AAYXStatus;
S5K8AAYXStatus S5K8AAYXCurrentStatus;

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

static int sensor_id_fail = 0;
static kal_uint32 zoom_factor = 0;


inline S5K8AAYX_read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,S5K8AAYX_WRITE_ID);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

inline int S5K8AAYX_write_cmos_sensor(u16 addr, u32 para)
{
    char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};

    iWriteRegI2C(puSendCmd , 4,S5K8AAYX_WRITE_ID);
}


/*******************************************************************************
* // Adapter for Winmo typedef
********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* // End Adapter for Winmo typedef
********************************************************************************/

#define  S5K8AAYX_LIMIT_EXPOSURE_LINES                (1253)
#define  S5K8AAYX_VIDEO_NORMALMODE_30FRAME_RATE       (30)
#define  S5K8AAYX_VIDEO_NORMALMODE_FRAME_RATE         (15)
#define  S5K8AAYX_VIDEO_NIGHTMODE_FRAME_RATE          (7.5)
#define  BANDING50_30HZ
/* Global Valuable */


kal_bool S5K8AAYX_MPEG4_encode_mode = KAL_FALSE, S5K8AAYX_MJPEG_encode_mode = KAL_FALSE;

kal_uint32 S5K8AAYX_pixel_clk_freq = 0, S5K8AAYX_sys_clk_freq = 0;          // 480 means 48MHz

kal_uint16 S5K8AAYX_CAP_dummy_pixels = 0;
kal_uint16 S5K8AAYX_CAP_dummy_lines = 0;

kal_uint16 S5K8AAYX_PV_cintr = 0;
kal_uint16 S5K8AAYX_PV_cintc = 0;
kal_uint16 S5K8AAYX_CAP_cintr = 0;
kal_uint16 S5K8AAYX_CAP_cintc = 0;

kal_bool S5K8AAYX_night_mode_enable = KAL_FALSE;


//===============old============================================
static kal_uint8 S5K8AAYX_exposure_line_h = 0, S5K8AAYX_exposure_line_l = 0,S5K8AAYX_extra_exposure_line_h = 0, S5K8AAYX_extra_exposure_line_l = 0;

static kal_bool S5K8AAYX_gPVmode = KAL_TRUE; //PV size or Full size
static kal_bool S5K8AAYX_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool S5K8AAYX_sensor_cap_state = KAL_FALSE; //Preview or Capture

static kal_uint16 S5K8AAYX_dummy_pixels=0, S5K8AAYX_dummy_lines=0;

static kal_uint16 S5K8AAYX_exposure_lines=0, S5K8AAYX_extra_exposure_lines = 0;



static kal_uint8 S5K8AAYX_Banding_setting = AE_FLICKER_MODE_50HZ;  //Jinghe modified

/****** OVT 6-18******/
static kal_uint16  S5K8AAYX_Capture_Max_Gain16= 6*16;
static kal_uint16  S5K8AAYX_Capture_Gain16=0 ;
static kal_uint16  S5K8AAYX_Capture_Shutter=0;
static kal_uint16  S5K8AAYX_Capture_Extra_Lines=0;

static kal_uint16  S5K8AAYX_PV_Dummy_Pixels =0, S5K8AAYX_Capture_Dummy_Pixels =0, S5K8AAYX_Capture_Dummy_Lines =0;
static kal_uint16  S5K8AAYX_PV_Gain16 = 0;
static kal_uint16  S5K8AAYX_PV_Shutter = 0;
static kal_uint16  S5K8AAYX_PV_Extra_Lines = 0;
kal_uint16 S5K8AAYX_sensor_gain_base=0,S5K8AAYX_FAC_SENSOR_REG=0,S5K8AAYX_iS5K8AAYX_Mode=0,S5K8AAYX_max_exposure_lines=0;
kal_uint32 S5K8AAYX_capture_pclk_in_M=520,S5K8AAYX_preview_pclk_in_M=390,S5K8AAYX_PV_dummy_pixels=0,S5K8AAYX_PV_dummy_lines=0,S5K8AAYX_isp_master_clock=0;
static kal_uint32  S5K8AAYX_sensor_pclk=720;
static kal_bool S5K8AAYX_AWB_ENABLE = KAL_TRUE;
static kal_bool S5K8AAYX_AE_ENABLE = KAL_TRUE;

//===============old============================================

kal_uint8 S5K8AAYX_sensor_write_I2C_address = S5K8AAYX_WRITE_ID;
kal_uint8 S5K8AAYX_sensor_read_I2C_address = S5K8AAYX_READ_ID;
//kal_uint16 S5K8AAYX_Sensor_ID = 0;

//HANDLE S5K8AAYXhDrvI2C;
//I2C_TRANSACTION S5K8AAYXI2CConfig;

UINT8 S5K8AAYXPixelClockDivider=0;

static DEFINE_SPINLOCK(s5k8aayx_drv_lock);
MSDK_SENSOR_CONFIG_STRUCT S5K8AAYXSensorConfigData;


/*************************************************************************
* FUNCTION
*       S5K8AAYXInitialPara
*
* DESCRIPTION
*       This function initialize the global status of  MT9V114
*
* PARAMETERS
*       None
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void S5K8AAYXInitialPara(void)
{
    S5K8AAYXCurrentStatus.iNightMode = 0xFFFF;
    S5K8AAYXCurrentStatus.iWB = AWB_MODE_AUTO;
    S5K8AAYXCurrentStatus.iEffect = MEFFECT_OFF;
    S5K8AAYXCurrentStatus.iBanding = AE_FLICKER_MODE_50HZ;
    S5K8AAYXCurrentStatus.iEV = AE_EV_COMP_n03;
    S5K8AAYXCurrentStatus.iMirror = IMAGE_NORMAL;
    S5K8AAYXCurrentStatus.iFrameRate = 25;
}


void S5K8AAYX_set_mirror(kal_uint8 image_mirror)
{
SENSORDB("Enter S5K8AAYX_set_mirror \n");

    if(S5K8AAYXCurrentStatus.iMirror == image_mirror)
      return;

    S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
    S5K8AAYX_write_cmos_sensor(0x002a, 0x01E8);

    switch (image_mirror)
    {
        case IMAGE_NORMAL:
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);           // REG_0TC_PCFG_uPrevMirror
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);           // REG_0TC_PCFG_uCaptureMirror
             break;
        case IMAGE_H_MIRROR:
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);           // REG_0TC_PCFG_uPrevMirror
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);           // REG_0TC_PCFG_uCaptureMirror
             break;
        case IMAGE_V_MIRROR:
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);           // REG_0TC_PCFG_uPrevMirror
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);           // REG_0TC_PCFG_uCaptureMirror
             break;
        case IMAGE_HV_MIRROR:
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003);           // REG_0TC_PCFG_uPrevMirror
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003);           // REG_0TC_PCFG_uCaptureMirror
             break;

        default:
             ASSERT(0);
             break;
    }

    S5K8AAYX_write_cmos_sensor(0x002A,0x01A8);
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0000); // #REG_TC_GP_ActivePrevConfig // Select preview configuration_0
    S5K8AAYX_write_cmos_sensor(0x002A,0x01AC);
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0001); // #REG_TC_GP_PrevOpenAfterChange
    S5K8AAYX_write_cmos_sensor(0x002A,0x01A6);
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_NewConfigSync // Update preview configuration
    S5K8AAYX_write_cmos_sensor(0x002A,0x01AA);
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_PrevConfigChanged
    S5K8AAYX_write_cmos_sensor(0x002A,0x019E);
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_EnablePreview // Start preview
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_EnablePreviewChanged

    S5K8AAYXCurrentStatus.iMirror = image_mirror;
}



void S5K8AAYX_set_isp_driving_current(kal_uint8 current)
{
}

/*****************************************************************************
 * FUNCTION
 *  S5K8AAYX_set_dummy
 * DESCRIPTION
 *
 * PARAMETERS
 *  pixels      [IN]
 *  lines       [IN]
 * RETURNS
 *  void
 *****************************************************************************/
void S5K8AAYX_set_dummy(kal_uint16 dummy_pixels, kal_uint16 dummy_lines)
{
    /****************************************************
      * Adjust the extra H-Blanking & V-Blanking.
      *****************************************************/
    S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
    S5K8AAYX_write_cmos_sensor(0x002A, 0x044C);

    S5K8AAYX_write_cmos_sensor(0x0F12, dummy_pixels);
    //S5K8AAYX_write_cmos_sensor(0x0F1C, dummy_pixels);   // Extra H-Blanking
    S5K8AAYX_write_cmos_sensor(0x0F12, dummy_lines);         // Extra V-Blanking
}   /* S5K8AAYX_set_dummy */


int SEN_SET_CURRENT = 1;
unsigned int D02D4_Current = 0x0;//0x02aa; 555
unsigned int D52D9_Current = 0x0;//0x02aa; 555
unsigned int unknow_gpio_Current   = 0x0000; //555
unsigned int CLK_Current   = 0x000; //555

/*****************************************************************************
 * FUNCTION
 *  S5K8AAYX_Initialize_Setting
 * DESCRIPTION
 *
 * PARAMETERS
 *  void
 * RETURNS
 *  void
 *****************************************************************************/
void S5K8AAYX_Initialize_Setting(void)
{
  SENSORDB("Enter S5K8AAYX_Initialize_Setting\n");
  S5K8AAYX_write_cmos_sensor(0xFCFC ,0xD000);
  S5K8AAYX_write_cmos_sensor(0x0010 ,0x0001); // Reset
  S5K8AAYX_write_cmos_sensor(0xFCFC ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0000 ,0x0000); // Simmian bug workaround
  S5K8AAYX_write_cmos_sensor(0xFCFC ,0xD000);
  S5K8AAYX_write_cmos_sensor(0x1030 ,0x0000); // Clear host interrupt so main will wait
  S5K8AAYX_write_cmos_sensor(0x0014 ,0x0001); // ARM go

  mdelay(50);

  //=====================================================================================
  // T&P part   // chris 20130322 
  //=====================================================================================
  S5K8AAYX_write_cmos_sensor(0x0028 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x2470);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xB510);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4910);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4810);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xF000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFA41);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4910);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4810);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xF000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFA3D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4910);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4810);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x6341);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4910);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4811);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xF000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFA36);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4910);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4811);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xF000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFA32);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4910);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4811);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xF000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFA2E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4810);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4911);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x6448);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4911);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4811);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xF000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFA27);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xBC10);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xBC08);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4718);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2870);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x8EDD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x27E8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x8725);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2788);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x26DC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xA6EF);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x26A8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xA0F1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2674);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x058F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2568);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x24F4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xAC79);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4070);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE92D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x5000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x23BC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00BE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x10BC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE351);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x33A8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x14BC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE151);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x9A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE041);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x11B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0091);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1034);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE593);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0520);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0091);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1384);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1008);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE591);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEA00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00EE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE080);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE585);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4070);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE8BD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x401F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE92D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00EB);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1002);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F86);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00EA);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2010);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE28D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0E3F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E9);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1002);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F86);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5DD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5DD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5DD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0011);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0029);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5DD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0011);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0026);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02E8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x10BA);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE351);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0022);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x12E4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5D1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE352);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001B);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5C1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1002);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE28D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0015);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEA00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5D1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5D1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3403);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE182);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC2B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2080);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE08C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE7B4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x039E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE80F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3E0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4624);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE00E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x47B4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1C2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE280);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC084);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE08C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x47B4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1DC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0493);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4624);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE00E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x47B4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1CC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC8B4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x039C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE003);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3623);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE00E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x38B4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1C2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE280);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1002);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE281);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFE7);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xBAFF);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x401F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE8BD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4010);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE92D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0250);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE310);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x123C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0DB2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1C1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4010);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE8BD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4010);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE92D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE590);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A5);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x021C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5D0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE594);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE584);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4010);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE8BD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4070);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE92D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE590);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0800);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0820);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4041);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE280);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01E8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x11B8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x51B6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE041);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0094);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1D11);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x11C8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5D1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE351);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x21B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3FB0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE353);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0003);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x31AC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x5BB2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1C3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE085);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xCBB4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1C3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE351);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1DBC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3EB4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2EB2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0193);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0092);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2811);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0194);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0092);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x11A1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1168);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02B4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1C1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4070);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE8BD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4010);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE92D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0072);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2150);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x14B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE311);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0144);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x9A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEA00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3118);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5C3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE5D3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0003);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3C1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1114);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x04B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1C2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1C1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4010);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE8BD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x41F0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE92D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE590);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC801);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC82C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE590);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1801);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1821);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4008);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE590);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x500C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE590);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3005);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0052);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x609C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x05B4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7080);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x10F4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x26B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D7);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0048);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1C4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x26B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D7);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x10F6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D5);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0043);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1C5);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x41F0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE8BD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4010);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE92D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE594);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1D0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE350);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0008);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0030);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE1A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2068);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE590);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0058);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1005);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE3A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0036);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE584);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4010);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE8BD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE594);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0034);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEB00);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE584);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFF9);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xEAFF);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3360);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x20D4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x16C8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x299C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1272);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1728);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x112C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x29A0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x122C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xF200);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xD000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2340);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0E2C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xF400);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xD000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0CDC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x06D4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4778);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x46C0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC091);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xF004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE51F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xD14C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xAC79);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0467);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2FA7);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xCB1F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x058F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xA0F1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2B43);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x8725);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x6777);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x8E49);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xC000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE59F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF1C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xE12F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x8EDD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xA4B6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  // End T&P part


  //============================================================
  // Set IO driving current
  //============================================================
  //S5K8AAYX_write_cmos_sensor(0x002A ,0x04B4);
  //S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0155); // d0~d4
  //S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0155); // d5~d9
  //S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1555); // gpio1~gpio3
  //S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0555); // HSYNC,VSYNC,PCLK,SCL,SDA
  /*if (SEN_SET_CURRENT)
  {
      S5K8AAYX_write_cmos_sensor(0x002A, 0x04B4);
      S5K8AAYX_write_cmos_sensor(0x0F12, D02D4_Current); //B4:155
      S5K8AAYX_write_cmos_sensor(0x0F12, D52D9_Current); //B6:155
      S5K8AAYX_write_cmos_sensor(0x0F12, unknow_gpio_Current); //0xB8:
      S5K8AAYX_write_cmos_sensor(0x0F12, CLK_Current); //0xBA:
  }*/


  //============================================================
  // Analog Settings
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x0028 ,0x7000);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0E38);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0476); //senHal_RegCompBiasNormSf //CDS bias
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0476); //senHal_RegCompBiasYAv //CDS bias
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0AA0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001); //setot_bUseDigitalHbin //1-Digital, 0-Analog
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0E2C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001); //senHal_bUseAnalogVerAv //2-Adding/averaging, 1-Y-Avg, 0-PLA
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0E66);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001); //senHal_RegBlstEnNorm
  S5K8AAYX_write_cmos_sensor(0x002A ,0x1250);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFF);   //senHal_Bls_nSpExpLines
  S5K8AAYX_write_cmos_sensor(0x002A ,0x1202);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0010);   //senHal_Dblr_VcoFreqMHZ

  //ADLC Filter
  S5K8AAYX_write_cmos_sensor(0x002A ,0x1288);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x020F); //gisp_dadlc_ResetFilterValue
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1C02); //gisp_dadlc_SteadyFilterValue
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0006); //gisp_dadlc_NResetIIrFrames

  //============================================================
  // AE
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0D46);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000F);

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0440);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3CF0); //lt_uMaxExp_0_
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0444);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x6590); //lt_uMaxExp_1_
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0448);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xBB80); //lt_uMaxExp_2_
  S5K8AAYX_write_cmos_sensor(0x002A ,0x044C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3880); //lt_uMaxExp_3_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0450);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3CF0); //lt_uCapMaxExp_0_
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0454);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x6590); //lt_uCapMaxExp_1_
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0458);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xBB80); //lt_uCapMaxExp_2_
  S5K8AAYX_write_cmos_sensor(0x002A ,0x045C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3880); //lt_uCapMaxExp_3_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0460);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0190); //lt_uMaxAnGain_0_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0280);  //0190 //lt_uMaxAnGain_1_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0540);  //0280 //lt_uMaxAnGain_2_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0C00);  //0A80 //lt_uMaxAnGain_3_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100); //lt_uMaxDigGain
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3000); //lt_uMaxTotGain

  S5K8AAYX_write_cmos_sensor(0x002A ,0x042E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010E); //lt_uLimitHigh
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F5); //lt_uLimitLow
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0DE0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); //ae_Fade2BlackEnable  F2B off, F2W on

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0D40);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003E); //TVAR_ae_BrAve

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0D4E); //AE_Weight
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0101);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0101);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0101);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0101);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0101);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0101);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0201);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0303);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0303);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0102);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0201);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0403);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0304);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0102);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0201);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0403);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0304);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0102);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0201);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0403);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0304);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0102);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0201);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0303);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0303);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0102);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0201);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0202);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0202);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0102);

  //============================================================
  //  Illum Type calibration
  //============================================================
  //WRITE #SARR_IllumType_0_            0078
  //WRITE #SARR_IllumType_1_            00C3
  //WRITE #SARR_IllumType_2_            00E9
  //WRITE #SARR_IllumType_3_            0128
  //WRITE #SARR_IllumType_4_            016F
  //WRITE #SARR_IllumType_5_            0195
  //WRITE #SARR_IllumType_6_            01A4
  //
  //WRITE #SARR_IllumTypeF_0_             0100
  //WRITE #SARR_IllumTypeF_1_             0100
  //WRITE #SARR_IllumTypeF_2_             0110
  //WRITE #SARR_IllumTypeF_3_             00E5
  //WRITE #SARR_IllumTypeF_4_             0100
  //WRITE #SARR_IllumTypeF_5_             00ED
  //WRITE #SARR_IllumTypeF_6_             00ED

  //*************************************/
  // 05.OTP Control                     */
  //*************************************/
  S5K8AAYX_write_cmos_sensor(0x002A ,0x3368);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // Tune_TP_bReMultGainsByNvm */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001); // Tune_TP_bUseNvmMultGain            2 7000336A SHORT
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // Tune_TP_bCorrectBlueChMismatch     2 7000336C SHORT
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // Tune_TP_BlueGainOfs88              2 7000336E SHORT
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // Tune_TP_BlueGainFactor88           2 70003370 SHORT

  //============================================================
  // Lens Shading
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x1326);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  //gisp_gos_Enable

  S5K8AAYX_write_cmos_sensor(0x002A ,0x063A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100);  // #TVAR_ash_GASAlpha[0][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E6);  // #TVAR_ash_GASAlpha[0][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E6);  // #TVAR_ash_GASAlpha[0][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B4);  // #TVAR_ash_GASAlpha[0][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B4);  // #TVAR_ash_GASAlpha[1][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E6);  // #TVAR_ash_GASAlpha[1][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E6);  // #TVAR_ash_GASAlpha[1][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D2);  // #TVAR_ash_GASAlpha[1][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B4);  // #TVAR_ash_GASAlpha[2][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E6);  // #TVAR_ash_GASAlpha[2][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E6);  // #TVAR_ash_GASAlpha[2][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D2);  // #TVAR_ash_GASAlpha[2][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B4);  // #TVAR_ash_GASAlpha[3][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E6);  // #TVAR_ash_GASAlpha[3][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E6);  // #TVAR_ash_GASAlpha[3][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00DC);  // #TVAR_ash_GASAlpha[3][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F5);  // #TVAR_ash_GASAlpha[4][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F8);  // #TVAR_ash_GASAlpha[4][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F8);  // #TVAR_ash_GASAlpha[4][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100);  // #TVAR_ash_GASAlpha[4][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0109);  // #TVAR_ash_GASAlpha[5][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100);  // #TVAR_ash_GASAlpha[5][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100);  // #TVAR_ash_GASAlpha[5][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100);  // #TVAR_ash_GASAlpha[5][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F0);  // #TVAR_ash_GASAlpha[6][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100);  // #TVAR_ash_GASAlpha[6][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100);  // #TVAR_ash_GASAlpha[6][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100);  // #TVAR_ash_GASAlpha[6][3]

  S5K8AAYX_write_cmos_sensor(0x002A ,0x067A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064);  // #TVAR_ash_GASBeta[0][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014);  // #TVAR_ash_GASBeta[0][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014);  // #TVAR_ash_GASBeta[0][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[0][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E);  // #TVAR_ash_GASBeta[1][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A);  // #TVAR_ash_GASBeta[1][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A);  // #TVAR_ash_GASBeta[1][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[1][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E);  // #TVAR_ash_GASBeta[2][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A);  // #TVAR_ash_GASBeta[2][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A);  // #TVAR_ash_GASBeta[2][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[2][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E);  // #TVAR_ash_GASBeta[3][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[3][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[3][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[3][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0046);  // #TVAR_ash_GASBeta[4][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[4][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[4][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[4][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0032);  // #TVAR_ash_GASBeta[5][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[5][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[5][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[5][3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0055);  // #TVAR_ash_GASBeta[6][0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[6][1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[6][2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #TVAR_ash_GASBeta[6][3]

  S5K8AAYX_write_cmos_sensor(0x002A ,0x06BA);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  //ash_bLumaMode

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0632);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F5); //TVAR_ash_CGrasAlphas_0_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F8); //TVAR_ash_CGrasAlphas_1_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F8); //TVAR_ash_CGrasAlphas_2_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100); //TVAR_ash_CGrasAlphas_3_

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0672);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100); //TVAR_ash_GASOutdoorAlpha_0_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100); //TVAR_ash_GASOutdoorAlpha_1_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100); //TVAR_ash_GASOutdoorAlpha_2_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100); //TVAR_ash_GASOutdoorAlpha_3_
  S5K8AAYX_write_cmos_sensor(0x002A ,0x06B2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //ash_GASOutdoorBeta_0_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //ash_GASOutdoorBeta_1_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //ash_GASOutdoorBeta_2_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //ash_GASOutdoorBeta_3_

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0624);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x009a);  //TVAR_ash_AwbAshCord_0_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00d3);  //TVAR_ash_AwbAshCord_1_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00d4);  //TVAR_ash_AwbAshCord_2_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x012c);  //TVAR_ash_AwbAshCord_3_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0162);  //TVAR_ash_AwbAshCord_4_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0190);  //TVAR_ash_AwbAshCord_5_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01a0);  //TVAR_ash_AwbAshCord_6_

  S5K8AAYX_write_cmos_sensor(0x002A ,0x06CC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0280);  //ash_uParabolicCenterX
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01E0);  //ash_uParabolicCenterY
  S5K8AAYX_write_cmos_sensor(0x002A ,0x06D0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000D);  //ash_uParabolicScalingA
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000F);  //ash_uParabolicScalingB

  S5K8AAYX_write_cmos_sensor(0x002A ,0x06C6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  //ash_bParabolicEstimation

  S5K8AAYX_write_cmos_sensor(0x002A ,0x347C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0254); //Tune_wbt_GAS_0_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01B3); //Tune_wbt_GAS_1_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0163); //Tune_wbt_GAS_2_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0120); //Tune_wbt_GAS_3_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00EE); //Tune_wbt_GAS_4_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CC); //Tune_wbt_GAS_5_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00BE); //Tune_wbt_GAS_6_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C5); //Tune_wbt_GAS_7_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E1); //Tune_wbt_GAS_8_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0113); //Tune_wbt_GAS_9_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0151); //Tune_wbt_GAS_10_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01A0); //Tune_wbt_GAS_11_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0203); //Tune_wbt_GAS_12_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01F6); //Tune_wbt_GAS_13_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x018A); //Tune_wbt_GAS_14_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x013C); //Tune_wbt_GAS_15_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F2); //Tune_wbt_GAS_16_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00BD); //Tune_wbt_GAS_17_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x009D); //Tune_wbt_GAS_18_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x008D); //Tune_wbt_GAS_19_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0095); //Tune_wbt_GAS_20_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B3); //Tune_wbt_GAS_21_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E7); //Tune_wbt_GAS_22_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0136); //Tune_wbt_GAS_23_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x018E); //Tune_wbt_GAS_24_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01EA); //Tune_wbt_GAS_25_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01B1); //Tune_wbt_GAS_26_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x014C); //Tune_wbt_GAS_27_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00FA); //Tune_wbt_GAS_28_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00AC); //Tune_wbt_GAS_29_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0078); //Tune_wbt_GAS_30_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0056); //Tune_wbt_GAS_31_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004B); //Tune_wbt_GAS_32_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0053); //Tune_wbt_GAS_33_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006C); //Tune_wbt_GAS_34_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A1); //Tune_wbt_GAS_35_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E7); //Tune_wbt_GAS_36_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x014E); //Tune_wbt_GAS_37_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01A9); //Tune_wbt_GAS_38_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x017F); //Tune_wbt_GAS_39_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0121); //Tune_wbt_GAS_40_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C7); //Tune_wbt_GAS_41_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007D); //Tune_wbt_GAS_42_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0049); //Tune_wbt_GAS_43_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002A); //Tune_wbt_GAS_44_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0021); //Tune_wbt_GAS_45_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0027); //Tune_wbt_GAS_46_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003E); //Tune_wbt_GAS_47_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006E); //Tune_wbt_GAS_48_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B4); //Tune_wbt_GAS_49_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0116); //Tune_wbt_GAS_50_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0178); //Tune_wbt_GAS_51_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0168); //Tune_wbt_GAS_52_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0105); //Tune_wbt_GAS_53_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A9); //Tune_wbt_GAS_54_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x005F); //Tune_wbt_GAS_55_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002D); //Tune_wbt_GAS_56_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0012); //Tune_wbt_GAS_57_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A); //Tune_wbt_GAS_58_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000E); //Tune_wbt_GAS_59_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); //Tune_wbt_GAS_60_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004F); //Tune_wbt_GAS_61_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0095); //Tune_wbt_GAS_62_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F6); //Tune_wbt_GAS_63_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x015A); //Tune_wbt_GAS_64_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x015C); //Tune_wbt_GAS_65_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F6); //Tune_wbt_GAS_66_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0099); //Tune_wbt_GAS_67_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0050); //Tune_wbt_GAS_68_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0022); //Tune_wbt_GAS_69_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0009); //Tune_wbt_GAS_70_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //Tune_wbt_GAS_71_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0003); //Tune_wbt_GAS_72_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0017); //Tune_wbt_GAS_73_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0043); //Tune_wbt_GAS_74_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0084); //Tune_wbt_GAS_75_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E6); //Tune_wbt_GAS_76_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x014F); //Tune_wbt_GAS_77_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x015A); //Tune_wbt_GAS_78_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F7); //Tune_wbt_GAS_79_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x009C); //Tune_wbt_GAS_80_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0054); //Tune_wbt_GAS_81_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0024); //Tune_wbt_GAS_82_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0009); //Tune_wbt_GAS_83_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //Tune_wbt_GAS_84_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004); //Tune_wbt_GAS_85_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0018); //Tune_wbt_GAS_86_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0044); //Tune_wbt_GAS_87_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0086); //Tune_wbt_GAS_88_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E6); //Tune_wbt_GAS_89_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x014E); //Tune_wbt_GAS_90_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0162); //Tune_wbt_GAS_91_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0106); //Tune_wbt_GAS_92_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00AA); //Tune_wbt_GAS_93_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0062); //Tune_wbt_GAS_94_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0030); //Tune_wbt_GAS_95_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); //Tune_wbt_GAS_96_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000B); //Tune_wbt_GAS_97_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0010); //Tune_wbt_GAS_98_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0025); //Tune_wbt_GAS_99_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0053); //Tune_wbt_GAS_100_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0095); //Tune_wbt_GAS_101_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F7); //Tune_wbt_GAS_102_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x015C); //Tune_wbt_GAS_103_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x017C); //Tune_wbt_GAS_104_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0122); //Tune_wbt_GAS_105_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CB); //Tune_wbt_GAS_106_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080); //Tune_wbt_GAS_107_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004C); //Tune_wbt_GAS_108_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0030); //Tune_wbt_GAS_109_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); //Tune_wbt_GAS_110_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002C); //Tune_wbt_GAS_111_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0043); //Tune_wbt_GAS_112_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0074); //Tune_wbt_GAS_113_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B7); //Tune_wbt_GAS_114_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x011B); //Tune_wbt_GAS_115_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x017A); //Tune_wbt_GAS_116_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01A8); //Tune_wbt_GAS_117_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x013C); //Tune_wbt_GAS_118_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F2); //Tune_wbt_GAS_119_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00AA); //Tune_wbt_GAS_120_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0077); //Tune_wbt_GAS_121_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0058); //Tune_wbt_GAS_122_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004E); //Tune_wbt_GAS_123_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0056); //Tune_wbt_GAS_124_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0070); //Tune_wbt_GAS_125_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A2); //Tune_wbt_GAS_126_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00EB); //Tune_wbt_GAS_127_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0146); //Tune_wbt_GAS_128_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x019D); //Tune_wbt_GAS_129_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01D9); //Tune_wbt_GAS_130_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x016A); //Tune_wbt_GAS_131_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0121); //Tune_wbt_GAS_132_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E5); //Tune_wbt_GAS_133_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B5); //Tune_wbt_GAS_134_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0098); //Tune_wbt_GAS_135_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x008C); //Tune_wbt_GAS_136_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0094); //Tune_wbt_GAS_137_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B1); //Tune_wbt_GAS_138_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E4); //Tune_wbt_GAS_139_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0126); //Tune_wbt_GAS_140_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x017D); //Tune_wbt_GAS_141_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01DE); //Tune_wbt_GAS_142_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01A0); //Tune_wbt_GAS_143_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00FA); //Tune_wbt_GAS_144_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00BE); //Tune_wbt_GAS_145_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0095); //Tune_wbt_GAS_146_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007E); //Tune_wbt_GAS_147_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006F); //Tune_wbt_GAS_148_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006A); //Tune_wbt_GAS_149_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006C); //Tune_wbt_GAS_150_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007C); //Tune_wbt_GAS_151_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0094); //Tune_wbt_GAS_152_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B4); //Tune_wbt_GAS_153_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E7); //Tune_wbt_GAS_154_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x014D); //Tune_wbt_GAS_155_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x013A); //Tune_wbt_GAS_156_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D4); //Tune_wbt_GAS_157_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A7); //Tune_wbt_GAS_158_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0081); //Tune_wbt_GAS_159_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0068); //Tune_wbt_GAS_160_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x005A); //Tune_wbt_GAS_161_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0053); //Tune_wbt_GAS_162_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0059); //Tune_wbt_GAS_163_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0068); //Tune_wbt_GAS_164_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0082); //Tune_wbt_GAS_165_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A7); //Tune_wbt_GAS_166_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D5); //Tune_wbt_GAS_167_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x011F); //Tune_wbt_GAS_168_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103); //Tune_wbt_GAS_169_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00AB); //Tune_wbt_GAS_170_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0084); //Tune_wbt_GAS_171_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x005D); //Tune_wbt_GAS_172_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0044); //Tune_wbt_GAS_173_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0034); //Tune_wbt_GAS_174_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0030); //Tune_wbt_GAS_175_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0035); //Tune_wbt_GAS_176_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0044); //Tune_wbt_GAS_177_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x005F); //Tune_wbt_GAS_178_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0081); //Tune_wbt_GAS_179_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B3); //Tune_wbt_GAS_180_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F3); //Tune_wbt_GAS_181_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00DC); //Tune_wbt_GAS_182_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0094); //Tune_wbt_GAS_183_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006B); //Tune_wbt_GAS_184_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0044); //Tune_wbt_GAS_185_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0029); //Tune_wbt_GAS_186_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001A); //Tune_wbt_GAS_187_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0015); //Tune_wbt_GAS_188_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001A); //Tune_wbt_GAS_189_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); //Tune_wbt_GAS_190_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0042); //Tune_wbt_GAS_191_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); //Tune_wbt_GAS_192_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0094); //Tune_wbt_GAS_193_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D0); //Tune_wbt_GAS_194_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CD); //Tune_wbt_GAS_195_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0087); //Tune_wbt_GAS_196_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x005B); //Tune_wbt_GAS_197_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0034); //Tune_wbt_GAS_198_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001A); //Tune_wbt_GAS_199_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000C); //Tune_wbt_GAS_200_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0008); //Tune_wbt_GAS_201_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000B); //Tune_wbt_GAS_202_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0018); //Tune_wbt_GAS_203_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0031); //Tune_wbt_GAS_204_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0055); //Tune_wbt_GAS_205_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0085); //Tune_wbt_GAS_206_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00BD); //Tune_wbt_GAS_207_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C6); //Tune_wbt_GAS_208_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007E); //Tune_wbt_GAS_209_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0053); //Tune_wbt_GAS_210_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002C); //Tune_wbt_GAS_211_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0013); //Tune_wbt_GAS_212_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0006); //Tune_wbt_GAS_213_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); //Tune_wbt_GAS_214_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004); //Tune_wbt_GAS_215_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0011); //Tune_wbt_GAS_216_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002A); //Tune_wbt_GAS_217_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004D); //Tune_wbt_GAS_218_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007C); //Tune_wbt_GAS_219_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B7); //Tune_wbt_GAS_220_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C6); //Tune_wbt_GAS_221_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007D); //Tune_wbt_GAS_222_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0054); //Tune_wbt_GAS_223_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002D); //Tune_wbt_GAS_224_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0013); //Tune_wbt_GAS_225_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005); //Tune_wbt_GAS_226_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001); //Tune_wbt_GAS_227_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004); //Tune_wbt_GAS_228_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0010); //Tune_wbt_GAS_229_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002A); //Tune_wbt_GAS_230_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004C); //Tune_wbt_GAS_231_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007C); //Tune_wbt_GAS_232_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B7); //Tune_wbt_GAS_233_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CB); //Tune_wbt_GAS_234_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0083); //Tune_wbt_GAS_235_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x005A); //Tune_wbt_GAS_236_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0034); //Tune_wbt_GAS_237_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001A); //Tune_wbt_GAS_238_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000B); //Tune_wbt_GAS_239_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0007); //Tune_wbt_GAS_240_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A); //Tune_wbt_GAS_241_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0017); //Tune_wbt_GAS_242_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0031); //Tune_wbt_GAS_243_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0052); //Tune_wbt_GAS_244_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0082); //Tune_wbt_GAS_245_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00BB); //Tune_wbt_GAS_246_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00DF); //Tune_wbt_GAS_247_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0092); //Tune_wbt_GAS_248_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0069); //Tune_wbt_GAS_249_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0043); //Tune_wbt_GAS_250_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002A); //Tune_wbt_GAS_251_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001B); //Tune_wbt_GAS_252_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0015); //Tune_wbt_GAS_253_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001A); //Tune_wbt_GAS_254_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); //Tune_wbt_GAS_255_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0042); //Tune_wbt_GAS_256_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0063); //Tune_wbt_GAS_257_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0092); //Tune_wbt_GAS_258_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CE); //Tune_wbt_GAS_259_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0100); //Tune_wbt_GAS_260_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A3); //Tune_wbt_GAS_261_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007B); //Tune_wbt_GAS_262_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0056); //Tune_wbt_GAS_263_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003D); //Tune_wbt_GAS_264_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0030); //Tune_wbt_GAS_265_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002C); //Tune_wbt_GAS_266_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0030); //Tune_wbt_GAS_267_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003F); //Tune_wbt_GAS_268_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0058); //Tune_wbt_GAS_269_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007A); //Tune_wbt_GAS_270_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A8); //Tune_wbt_GAS_271_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E5); //Tune_wbt_GAS_272_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0139); //Tune_wbt_GAS_273_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C6); //Tune_wbt_GAS_274_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0097); //Tune_wbt_GAS_275_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0077); //Tune_wbt_GAS_276_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0060); //Tune_wbt_GAS_277_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0053); //Tune_wbt_GAS_278_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0050); //Tune_wbt_GAS_279_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0053); //Tune_wbt_GAS_280_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); //Tune_wbt_GAS_281_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007D); //Tune_wbt_GAS_282_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x009D); //Tune_wbt_GAS_283_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CD); //Tune_wbt_GAS_284_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x011F); //Tune_wbt_GAS_285_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0192); //Tune_wbt_GAS_286_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00F2); //Tune_wbt_GAS_287_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B6); //Tune_wbt_GAS_288_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x008D); //Tune_wbt_GAS_289_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0075); //Tune_wbt_GAS_290_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0066); //Tune_wbt_GAS_291_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0060); //Tune_wbt_GAS_292_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); //Tune_wbt_GAS_293_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0072); //Tune_wbt_GAS_294_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x008A); //Tune_wbt_GAS_295_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00AB); //Tune_wbt_GAS_296_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00DD); //Tune_wbt_GAS_297_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0145); //Tune_wbt_GAS_298_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0136); //Tune_wbt_GAS_299_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D0); //Tune_wbt_GAS_300_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A5); //Tune_wbt_GAS_301_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007E); //Tune_wbt_GAS_302_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); //Tune_wbt_GAS_303_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0055); //Tune_wbt_GAS_304_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004E); //Tune_wbt_GAS_305_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0053); //Tune_wbt_GAS_306_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0061); //Tune_wbt_GAS_307_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007C); //Tune_wbt_GAS_308_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A1); //Tune_wbt_GAS_309_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CE); //Tune_wbt_GAS_310_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x011C); //Tune_wbt_GAS_311_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00FC); //Tune_wbt_GAS_312_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00AA); //Tune_wbt_GAS_313_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0082); //Tune_wbt_GAS_314_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x005A); //Tune_wbt_GAS_315_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0041); //Tune_wbt_GAS_316_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0030); //Tune_wbt_GAS_317_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002A); //Tune_wbt_GAS_318_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0030); //Tune_wbt_GAS_319_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003E); //Tune_wbt_GAS_320_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0059); //Tune_wbt_GAS_321_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007A); //Tune_wbt_GAS_322_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00AC); //Tune_wbt_GAS_323_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00EE); //Tune_wbt_GAS_324_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D8); //Tune_wbt_GAS_325_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0094); //Tune_wbt_GAS_326_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006A); //Tune_wbt_GAS_327_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0043); //Tune_wbt_GAS_328_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); //Tune_wbt_GAS_329_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0016); //Tune_wbt_GAS_330_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0012); //Tune_wbt_GAS_331_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0016); //Tune_wbt_GAS_332_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); //Tune_wbt_GAS_333_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003E); //Tune_wbt_GAS_334_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0061); //Tune_wbt_GAS_335_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0090); //Tune_wbt_GAS_336_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CE); //Tune_wbt_GAS_337_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CC); //Tune_wbt_GAS_338_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0087); //Tune_wbt_GAS_339_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x005C); //Tune_wbt_GAS_340_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0034); //Tune_wbt_GAS_341_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0019); //Tune_wbt_GAS_342_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A); //Tune_wbt_GAS_343_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005); //Tune_wbt_GAS_344_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0009); //Tune_wbt_GAS_345_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0015); //Tune_wbt_GAS_346_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002E); //Tune_wbt_GAS_347_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0052); //Tune_wbt_GAS_348_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0082); //Tune_wbt_GAS_349_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00BE); //Tune_wbt_GAS_350_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C5); //Tune_wbt_GAS_351_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007F); //Tune_wbt_GAS_352_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0054); //Tune_wbt_GAS_353_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002D); //Tune_wbt_GAS_354_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0013); //Tune_wbt_GAS_355_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004); //Tune_wbt_GAS_356_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //Tune_wbt_GAS_357_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); //Tune_wbt_GAS_358_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000E); //Tune_wbt_GAS_359_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0027); //Tune_wbt_GAS_360_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004A); //Tune_wbt_GAS_361_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007A); //Tune_wbt_GAS_362_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B5); //Tune_wbt_GAS_363_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C4); //Tune_wbt_GAS_364_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080); //Tune_wbt_GAS_365_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0056); //Tune_wbt_GAS_366_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002E); //Tune_wbt_GAS_367_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); //Tune_wbt_GAS_368_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004); //Tune_wbt_GAS_369_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //Tune_wbt_GAS_370_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); //Tune_wbt_GAS_371_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000E); //Tune_wbt_GAS_372_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0027); //Tune_wbt_GAS_373_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004A); //Tune_wbt_GAS_374_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0079); //Tune_wbt_GAS_375_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B6); //Tune_wbt_GAS_376_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CA); //Tune_wbt_GAS_377_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0086); //Tune_wbt_GAS_378_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x005C); //Tune_wbt_GAS_379_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0035); //Tune_wbt_GAS_380_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001B); //Tune_wbt_GAS_381_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A); //Tune_wbt_GAS_382_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005); //Tune_wbt_GAS_383_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0008); //Tune_wbt_GAS_384_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0015); //Tune_wbt_GAS_385_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002F); //Tune_wbt_GAS_386_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0050); //Tune_wbt_GAS_387_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007F); //Tune_wbt_GAS_388_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00BA); //Tune_wbt_GAS_389_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00DE); //Tune_wbt_GAS_390_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0094); //Tune_wbt_GAS_391_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006B); //Tune_wbt_GAS_392_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0045); //Tune_wbt_GAS_393_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002A); //Tune_wbt_GAS_394_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001A); //Tune_wbt_GAS_395_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0013); //Tune_wbt_GAS_396_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0018); //Tune_wbt_GAS_397_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0025); //Tune_wbt_GAS_398_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003F); //Tune_wbt_GAS_399_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x005F); //Tune_wbt_GAS_400_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0090); //Tune_wbt_GAS_401_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CD); //Tune_wbt_GAS_402_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0101); //Tune_wbt_GAS_403_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A6); //Tune_wbt_GAS_404_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007D); //Tune_wbt_GAS_405_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0057); //Tune_wbt_GAS_406_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003E); //Tune_wbt_GAS_407_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002F); //Tune_wbt_GAS_408_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002A); //Tune_wbt_GAS_409_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002E); //Tune_wbt_GAS_410_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003B); //Tune_wbt_GAS_411_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0055); //Tune_wbt_GAS_412_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0077); //Tune_wbt_GAS_413_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A5); //Tune_wbt_GAS_414_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E3); //Tune_wbt_GAS_415_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0139); //Tune_wbt_GAS_416_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C6); //Tune_wbt_GAS_417_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0099); //Tune_wbt_GAS_418_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0078); //Tune_wbt_GAS_419_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0060); //Tune_wbt_GAS_420_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0052); //Tune_wbt_GAS_421_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004D); //Tune_wbt_GAS_422_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0051); //Tune_wbt_GAS_423_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0060); //Tune_wbt_GAS_424_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0079); //Tune_wbt_GAS_425_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x009A); //Tune_wbt_GAS_426_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CA); //Tune_wbt_GAS_427_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0120); //Tune_wbt_GAS_428_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x016F); //Tune_wbt_GAS_429_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D5); //Tune_wbt_GAS_430_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x009D); //Tune_wbt_GAS_431_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007A); //Tune_wbt_GAS_432_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0065); //Tune_wbt_GAS_433_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0058); //Tune_wbt_GAS_434_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0054); //Tune_wbt_GAS_435_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0056); //Tune_wbt_GAS_436_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0063); //Tune_wbt_GAS_437_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007A); //Tune_wbt_GAS_438_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0095); //Tune_wbt_GAS_439_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C5); //Tune_wbt_GAS_440_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x011E); //Tune_wbt_GAS_441_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010E); //Tune_wbt_GAS_442_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B5); //Tune_wbt_GAS_443_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x008A); //Tune_wbt_GAS_444_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006B); //Tune_wbt_GAS_445_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0057); //Tune_wbt_GAS_446_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004B); //Tune_wbt_GAS_447_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0046); //Tune_wbt_GAS_448_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004A); //Tune_wbt_GAS_449_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0058); //Tune_wbt_GAS_450_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006E); //Tune_wbt_GAS_451_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0090); //Tune_wbt_GAS_452_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B7); //Tune_wbt_GAS_453_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00EE); //Tune_wbt_GAS_454_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D7); //Tune_wbt_GAS_455_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0091); //Tune_wbt_GAS_456_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006C); //Tune_wbt_GAS_457_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004C); //Tune_wbt_GAS_458_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0038); //Tune_wbt_GAS_459_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002A); //Tune_wbt_GAS_460_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0026); //Tune_wbt_GAS_461_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002A); //Tune_wbt_GAS_462_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0038); //Tune_wbt_GAS_463_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0050); //Tune_wbt_GAS_464_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006D); //Tune_wbt_GAS_465_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x009C); //Tune_wbt_GAS_466_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C7); //Tune_wbt_GAS_467_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B5); //Tune_wbt_GAS_468_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007E); //Tune_wbt_GAS_469_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0058); //Tune_wbt_GAS_470_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0038); //Tune_wbt_GAS_471_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0021); //Tune_wbt_GAS_472_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0015); //Tune_wbt_GAS_473_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0011); //Tune_wbt_GAS_474_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); //Tune_wbt_GAS_475_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0022); //Tune_wbt_GAS_476_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003A); //Tune_wbt_GAS_477_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0057); //Tune_wbt_GAS_478_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0083); //Tune_wbt_GAS_479_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00AF); //Tune_wbt_GAS_480_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A8); //Tune_wbt_GAS_481_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0071); //Tune_wbt_GAS_482_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004B); //Tune_wbt_GAS_483_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002B); //Tune_wbt_GAS_484_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); //Tune_wbt_GAS_485_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0009); //Tune_wbt_GAS_486_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005); //Tune_wbt_GAS_487_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0008); //Tune_wbt_GAS_488_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); //Tune_wbt_GAS_489_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002B); //Tune_wbt_GAS_490_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004A); //Tune_wbt_GAS_491_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0076); //Tune_wbt_GAS_492_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x009F); //Tune_wbt_GAS_493_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A1); //Tune_wbt_GAS_494_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006B); //Tune_wbt_GAS_495_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0045); //Tune_wbt_GAS_496_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0024); //Tune_wbt_GAS_497_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000F); //Tune_wbt_GAS_498_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0003); //Tune_wbt_GAS_499_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //Tune_wbt_GAS_500_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); //Tune_wbt_GAS_501_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000E); //Tune_wbt_GAS_502_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0024); //Tune_wbt_GAS_503_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0042); //Tune_wbt_GAS_504_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006E); //Tune_wbt_GAS_505_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0098); //Tune_wbt_GAS_506_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A1); //Tune_wbt_GAS_507_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006C); //Tune_wbt_GAS_508_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0046); //Tune_wbt_GAS_509_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0027); //Tune_wbt_GAS_510_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0010); //Tune_wbt_GAS_511_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004); //Tune_wbt_GAS_512_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //Tune_wbt_GAS_513_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); //Tune_wbt_GAS_514_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000E); //Tune_wbt_GAS_515_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0024); //Tune_wbt_GAS_516_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0043); //Tune_wbt_GAS_517_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006E); //Tune_wbt_GAS_518_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0099); //Tune_wbt_GAS_519_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00AA); //Tune_wbt_GAS_520_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0073); //Tune_wbt_GAS_521_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004D); //Tune_wbt_GAS_522_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002D); //Tune_wbt_GAS_523_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0016); //Tune_wbt_GAS_524_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0009); //Tune_wbt_GAS_525_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005); //Tune_wbt_GAS_526_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0008); //Tune_wbt_GAS_527_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); //Tune_wbt_GAS_528_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002C); //Tune_wbt_GAS_529_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0049); //Tune_wbt_GAS_530_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0076); //Tune_wbt_GAS_531_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x009C); //Tune_wbt_GAS_532_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00BD); //Tune_wbt_GAS_533_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007F); //Tune_wbt_GAS_534_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0058); //Tune_wbt_GAS_535_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003A); //Tune_wbt_GAS_536_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0024); //Tune_wbt_GAS_537_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0018); //Tune_wbt_GAS_538_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0012); //Tune_wbt_GAS_539_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0016); //Tune_wbt_GAS_540_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); //Tune_wbt_GAS_541_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003B); //Tune_wbt_GAS_542_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0058); //Tune_wbt_GAS_543_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0082); //Tune_wbt_GAS_544_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00AB); //Tune_wbt_GAS_545_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00DC); //Tune_wbt_GAS_546_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x008F); //Tune_wbt_GAS_547_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006A); //Tune_wbt_GAS_548_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004C); //Tune_wbt_GAS_549_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0038); //Tune_wbt_GAS_550_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002C); //Tune_wbt_GAS_551_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); //Tune_wbt_GAS_552_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x002C); //Tune_wbt_GAS_553_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0038); //Tune_wbt_GAS_554_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0050); //Tune_wbt_GAS_555_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006C); //Tune_wbt_GAS_556_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0096); //Tune_wbt_GAS_557_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C2); //Tune_wbt_GAS_558_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0117); //Tune_wbt_GAS_559_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00AF); //Tune_wbt_GAS_560_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0083); //Tune_wbt_GAS_561_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0068); //Tune_wbt_GAS_562_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0054); //Tune_wbt_GAS_563_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004A); //Tune_wbt_GAS_564_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0046); //Tune_wbt_GAS_565_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004A); //Tune_wbt_GAS_566_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0058); //Tune_wbt_GAS_567_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006D); //Tune_wbt_GAS_568_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x008A); //Tune_wbt_GAS_569_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B4); //Tune_wbt_GAS_570_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00FB); //Tune_wbt_GAS_571_

  S5K8AAYX_write_cmos_sensor(0x002A ,0x1348);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);

  //============================================================
  // AWB
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0B36);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0B3A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00EC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02CC);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0B38);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0010);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0AE6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03E1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0413);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0391);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0416);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x035D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0402);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x032D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03DD);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02EE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03B8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02AF);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x037D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0293);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0347);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x027C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x031A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0271);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02F9);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0264);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02D7);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0250);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02BF);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0238);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02A9);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x021B);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0289);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0200);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0273);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01FC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0259);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0211);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0238);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BAA);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0006);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BAE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02F3);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BAC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0B7A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03E8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0436);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x036A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0445);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02F1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x041E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0281);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03E6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0242);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03AE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0219);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x033D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01E9);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02EA);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01D4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02B4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x028C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0208);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x022E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0B70);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0B74);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01F8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02A8);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0B72);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0007);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0B40);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x029E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02C8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0281);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02C8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0266);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02AC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0251);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x028E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x023D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0275);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0228);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x025D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0228);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0243);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BC8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BCC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x018F);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BCA);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BB4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03E7);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03F8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03A7);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0352);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03D0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0322);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x039E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x032B);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x034D);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BE6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0006);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BEA);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x019E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0257);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BE8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BD2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030B);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0323);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02C3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0288);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02E5);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x026A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02A2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0C2C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x013D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x011A);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0D0E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B2);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0CFE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0FAB);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0FF5);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x10BB);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1153);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x11C5);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x122A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A9);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C0);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0CF8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x034D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0389);

  //Grid
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0CB0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0032);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFCE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);

  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0096);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0096);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0D30);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); //gridenable 0000:disable 0001:WP correction 0002:gain correction

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BFC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03D9);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x011A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0374);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0152);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03B0);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0124);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02DB);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x019C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0295);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0214);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0251);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x026A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0230);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0299);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0255);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x025B);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0C4C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0452);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0C58);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x059C);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0BF8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01AE);

  //============================================================
  // Outdoor detector
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0C86);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0C70);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF7B);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00CE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF23);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFEF3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x012C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFED7);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x014E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFEBB);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0162);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1388);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0C8A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4ACB);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0C88);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A7C);

  //============================================================
  // Pre-Gamma
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0538); //LutPreDemNoBin
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0035);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x005A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0095);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0121);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0139);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0150);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0177);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x019A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01BB);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01DC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0219);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0251);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02B3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x035F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03B1);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FF);

  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //LutPostDemNoBin
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0004);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0012);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0016);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0024);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0031);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x004E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0075);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00A8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0126);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01BE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0272);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0334);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FF);
  //============================================================
  // CCM
  //============================================================
  //Horizon
  S5K8AAYX_write_cmos_sensor(0x002A ,0x33A4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01D0); //#TVAR_wbt_pBaseCcms[0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFA1); //#TVAR_wbt_pBaseCcms[1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFA); //#TVAR_wbt_pBaseCcms[2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF6F); //#TVAR_wbt_pBaseCcms[3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0140); //#TVAR_wbt_pBaseCcms[4]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF49); //#TVAR_wbt_pBaseCcms[5]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFC1); //#TVAR_wbt_pBaseCcms[6]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001F); //#TVAR_wbt_pBaseCcms[7]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01BD); //#TVAR_wbt_pBaseCcms[8]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x013F); //#TVAR_wbt_pBaseCcms[9]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E1); //#TVAR_wbt_pBaseCcms[10]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF43); //#TVAR_wbt_pBaseCcms[11]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0191); //#TVAR_wbt_pBaseCcms[12]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFC0); //#TVAR_wbt_pBaseCcms[13]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01B7); //#TVAR_wbt_pBaseCcms[14]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF30); //#TVAR_wbt_pBaseCcms[15]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x015F); //#TVAR_wbt_pBaseCcms[16]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0106); //#TVAR_wbt_pBaseCcms[17]
  //Inca
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01D0); //#TVAR_wbt_pBaseCcms[18]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFA1); //#TVAR_wbt_pBaseCcms[19]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFA); //#TVAR_wbt_pBaseCcms[20]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF6F); //#TVAR_wbt_pBaseCcms[21]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0140); //#TVAR_wbt_pBaseCcms[22]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF49); //#TVAR_wbt_pBaseCcms[23]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFC1); //#TVAR_wbt_pBaseCcms[24]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001F); //#TVAR_wbt_pBaseCcms[25]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01BD); //#TVAR_wbt_pBaseCcms[26]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x013F); //#TVAR_wbt_pBaseCcms[27]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E1); //#TVAR_wbt_pBaseCcms[28]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF43); //#TVAR_wbt_pBaseCcms[29]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0191); //#TVAR_wbt_pBaseCcms[30]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFC0); //#TVAR_wbt_pBaseCcms[31]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01B7); //#TVAR_wbt_pBaseCcms[32]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF30); //#TVAR_wbt_pBaseCcms[33]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x015F); //#TVAR_wbt_pBaseCcms[34]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0106); //#TVAR_wbt_pBaseCcms[35]

  //WW
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01D0); //#TVAR_wbt_pBaseCcms[36]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFA1); //#TVAR_wbt_pBaseCcms[37]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFA); //#TVAR_wbt_pBaseCcms[38]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF6F); //#TVAR_wbt_pBaseCcms[39]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0140); //#TVAR_wbt_pBaseCcms[40]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF49); //#TVAR_wbt_pBaseCcms[41]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFC1); //#TVAR_wbt_pBaseCcms[42]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001F); //#TVAR_wbt_pBaseCcms[43]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01BD); //#TVAR_wbt_pBaseCcms[44]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x013F); //#TVAR_wbt_pBaseCcms[45]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E1); //#TVAR_wbt_pBaseCcms[46]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF43); //#TVAR_wbt_pBaseCcms[47]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0191); //#TVAR_wbt_pBaseCcms[48]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFC0); //#TVAR_wbt_pBaseCcms[49]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01B7); //#TVAR_wbt_pBaseCcms[50]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF30); //#TVAR_wbt_pBaseCcms[51]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x015F); //#TVAR_wbt_pBaseCcms[52]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0106); //#TVAR_wbt_pBaseCcms[53]

  //CWF
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01D0); //#TVAR_wbt_pBaseCcms[54]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFA1); //#TVAR_wbt_pBaseCcms[55]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFA); //#TVAR_wbt_pBaseCcms[56]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF6F); //#TVAR_wbt_pBaseCcms[57]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0140); //#TVAR_wbt_pBaseCcms[58]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF49); //#TVAR_wbt_pBaseCcms[59]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFC1); //#TVAR_wbt_pBaseCcms[60]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001F); //#TVAR_wbt_pBaseCcms[61]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01BD); //#TVAR_wbt_pBaseCcms[62]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x013F); //#TVAR_wbt_pBaseCcms[63]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00E1); //#TVAR_wbt_pBaseCcms[64]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF43); //#TVAR_wbt_pBaseCcms[65]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0191); //#TVAR_wbt_pBaseCcms[66]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFC0); //#TVAR_wbt_pBaseCcms[67]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01B7); //#TVAR_wbt_pBaseCcms[68]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF30); //#TVAR_wbt_pBaseCcms[69]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x015F); //#TVAR_wbt_pBaseCcms[70]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0106); //#TVAR_wbt_pBaseCcms[71]

  //D50
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01BF); //#TVAR_wbt_pBaseCcms[72]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFBF); //#TVAR_wbt_pBaseCcms[73]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFE); //#TVAR_wbt_pBaseCcms[74]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF6D); //#TVAR_wbt_pBaseCcms[75]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01B4); //#TVAR_wbt_pBaseCcms[76]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF66); //#TVAR_wbt_pBaseCcms[77]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFCA); //#TVAR_wbt_pBaseCcms[78]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFCE); //#TVAR_wbt_pBaseCcms[79]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x017B); //#TVAR_wbt_pBaseCcms[80]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0136); //#TVAR_wbt_pBaseCcms[81]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0132); //#TVAR_wbt_pBaseCcms[82]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF85); //#TVAR_wbt_pBaseCcms[83]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x018B); //#TVAR_wbt_pBaseCcms[84]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF73); //#TVAR_wbt_pBaseCcms[85]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0191); //#TVAR_wbt_pBaseCcms[86]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF3F); //#TVAR_wbt_pBaseCcms[87]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x015B); //#TVAR_wbt_pBaseCcms[88]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D0); //#TVAR_wbt_pBaseCcms[89]

  //D65
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01BF); //#TVAR_wbt_pBaseCcms[90]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFBF); //#TVAR_wbt_pBaseCcms[91]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFE); //#TVAR_wbt_pBaseCcms[92]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF6D); //#TVAR_wbt_pBaseCcms[93]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01B4); //#TVAR_wbt_pBaseCcms[94]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF66); //#TVAR_wbt_pBaseCcms[95]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFCA); //#TVAR_wbt_pBaseCcms[96]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFCE); //#TVAR_wbt_pBaseCcms[97]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x017B); //#TVAR_wbt_pBaseCcms[98]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0136); //#TVAR_wbt_pBaseCcms[99]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0132); //#TVAR_wbt_pBaseCcms[100]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF85); //#TVAR_wbt_pBaseCcms[101]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x018B); //#TVAR_wbt_pBaseCcms[102]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF73); //#TVAR_wbt_pBaseCcms[103]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0191); //#TVAR_wbt_pBaseCcms[104]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF3F); //#TVAR_wbt_pBaseCcms[105]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x015B); //#TVAR_wbt_pBaseCcms[106]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D0); //#TVAR_wbt_pBaseCcms[107]

  //Outdoor
  S5K8AAYX_write_cmos_sensor(0x002A ,0x3380);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01C8); //#TVAR_wbt_pOutdoorCcm[0]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFBF); //#TVAR_wbt_pOutdoorCcm[1]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFF); //#TVAR_wbt_pOutdoorCcm[2]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF0C); //#TVAR_wbt_pOutdoorCcm[3]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0230); //#TVAR_wbt_pOutdoorCcm[4]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFEFA); //#TVAR_wbt_pOutdoorCcm[5]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0006); //#TVAR_wbt_pOutdoorCcm[6]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFDE); //#TVAR_wbt_pOutdoorCcm[7]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0225); //#TVAR_wbt_pOutdoorCcm[8]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0124); //#TVAR_wbt_pOutdoorCcm[9]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010D); //#TVAR_wbt_pOutdoorCcm[10]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF21); //#TVAR_wbt_pOutdoorCcm[11]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01D4); //#TVAR_wbt_pOutdoorCcm[12]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF40); //#TVAR_wbt_pOutdoorCcm[13]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0187); //#TVAR_wbt_pOutdoorCcm[14]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFEB3); //#TVAR_wbt_pOutdoorCcm[15]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x014B); //#TVAR_wbt_pOutdoorCcm[16]
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007B); //#TVAR_wbt_pOutdoorCcm[17]

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0612);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x009D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D5);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0128);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0166);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0193);

  //============================================================
  // Gamma
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0498); //Indoor
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0003);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0021);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0060);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0127);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x014C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x016E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01A5);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01D3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01FB);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x021F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0260);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x029A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02F7);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x034D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0395);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03CE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FF);

  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //Outdoor
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0003);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0021);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0060);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00D3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0127);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x014C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x016E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01A5);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01D3);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01FB);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x021F);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0260);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x029A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x02F7);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x034D);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0395);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03CE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FF);

  //============================================================
  // AFIT
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x3360);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // afit_bUseNormBrForAfit */

  S5K8AAYX_write_cmos_sensor(0x002A ,0x06D4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0032); // afit_uNoiseIndInDoor_0_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0078); // afit_uNoiseIndInDoor_1_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00C8); // afit_uNoiseIndInDoor_2_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0190); // afit_uNoiseIndInDoor_3_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x028C); // afit_uNoiseIndInDoor_4_ */

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0734);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__0_  Brightness[0] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__1_  Contrast[0] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__2_  Saturation[0] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__3_  Sharp_Blur[0] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__4_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0078); // AfitBaseVals_0__5_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x012C); // AfitBaseVals_0__6_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FF); // AfitBaseVals_0__7_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_0__8_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); // AfitBaseVals_0__9_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000C); // AfitBaseVals_0__10_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0010); // AfitBaseVals_0__11_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01E6); // AfitBaseVals_0__12_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__13_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0070); // AfitBaseVals_0__14_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01FF); // AfitBaseVals_0__15_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0144); // AfitBaseVals_0__16_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000F); // AfitBaseVals_0__17_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A); // AfitBaseVals_0__18_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0073); // AfitBaseVals_0__19_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0087); // AfitBaseVals_0__20_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_0__21_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A); // AfitBaseVals_0__22_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); // AfitBaseVals_0__23_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E); // AfitBaseVals_0__24_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_0__25_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A); // AfitBaseVals_0__26_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); // AfitBaseVals_0__27_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0046); // AfitBaseVals_0__28_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2B32); // AfitBaseVals_0__29_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0601); // AfitBaseVals_0__30_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__31_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__32_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__33_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00FF); // AfitBaseVals_0__34_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x07FF); // AfitBaseVals_0__35_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFF); // AfitBaseVals_0__36_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__37_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x050D); // AfitBaseVals_0__38_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1E80); // AfitBaseVals_0__39_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__40_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1408); // 1408 AfitBaseVals_0__41_ iNearGrayDesat[0] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0214); // AfitBaseVals_0__42_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF01); // AfitBaseVals_0__43_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x180F); // AfitBaseVals_0__44_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001); // AfitBaseVals_0__45_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__46_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x8003); // AfitBaseVals_0__47_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0094); // AfitBaseVals_0__48_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0580); // AfitBaseVals_0__49_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0280); // AfitBaseVals_0__50_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0308); // AfitBaseVals_0__51_ iClustThresh_H[0] iClustMulT_H[0] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3186); // AfitBaseVals_0__52_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x707E); // AfitBaseVals_0__53_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A02); // AfitBaseVals_0__54_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080A); // AfitBaseVals_0__55_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0500); // AfitBaseVals_0__56_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x032D); // AfitBaseVals_0__57_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x324E); // AfitBaseVals_0__58_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E); // AfitBaseVals_0__59_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0200); // AfitBaseVals_0__60_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103); // AfitBaseVals_0__61_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010C); // AfitBaseVals_0__62_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x9696); // AfitBaseVals_0__63_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4646); // AfitBaseVals_0__64_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0802); // AfitBaseVals_0__65_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0802); // AfitBaseVals_0__66_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__67_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F); // AfitBaseVals_0__68_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3202); // AfitBaseVals_0__69_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F1E); // AfitBaseVals_0__70_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x020F); // AfitBaseVals_0__71_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103); // AfitBaseVals_0__72_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010C); // AfitBaseVals_0__73_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x9696); // AfitBaseVals_0__74_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x4646); // AfitBaseVals_0__75_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0802); // AfitBaseVals_0__76_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0802); // AfitBaseVals_0__77_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_0__78_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F); // AfitBaseVals_0__79_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3202); // AfitBaseVals_0__80_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F1E); // AfitBaseVals_0__81_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x020F); // AfitBaseVals_0__82_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0003); // AfitBaseVals_0__83_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__0_  Brightness[1] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__1_  Contrast[1] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__2_  Saturation[1] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__3_  Sharp_Blur[1] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__4_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x006A); // AfitBaseVals_1__5_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x012C); // AfitBaseVals_1__6_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FF); // AfitBaseVals_1__7_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_1__8_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); // AfitBaseVals_1__9_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000C); // AfitBaseVals_1__10_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0010); // AfitBaseVals_1__11_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01E6); // AfitBaseVals_1__12_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FF); // AfitBaseVals_1__13_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0070); // AfitBaseVals_1__14_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007D); // AfitBaseVals_1__15_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); // AfitBaseVals_1__16_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_1__17_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A); // AfitBaseVals_1__18_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0073); // AfitBaseVals_1__19_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0087); // AfitBaseVals_1__20_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_1__21_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A); // AfitBaseVals_1__22_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); // AfitBaseVals_1__23_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E); // AfitBaseVals_1__24_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_1__25_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000A); // AfitBaseVals_1__26_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); // AfitBaseVals_1__27_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E); // AfitBaseVals_1__28_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2B32); // AfitBaseVals_1__29_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0601); // AfitBaseVals_1__30_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__31_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__32_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__33_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00FF); // AfitBaseVals_1__34_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x07FF); // AfitBaseVals_1__35_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFF); // AfitBaseVals_1__36_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__37_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x050D); // AfitBaseVals_1__38_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1E80); // AfitBaseVals_1__39_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__40_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1403); // 1408 AfitBaseVals_1__41_ iNearGrayDesat[1] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0214); // AfitBaseVals_1__42_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF01); // AfitBaseVals_1__43_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x180F); // AfitBaseVals_1__44_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); // AfitBaseVals_1__45_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__46_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x8003); // AfitBaseVals_1__47_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080); // AfitBaseVals_1__48_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080); // AfitBaseVals_1__49_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0280); // AfitBaseVals_1__50_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0308); // AfitBaseVals_1__51_ iClustThresh_H[1] iClustMulT_H[1] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1E65); // AfitBaseVals_1__52_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1A24); // AfitBaseVals_1__53_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A03); // AfitBaseVals_1__54_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080A); // AfitBaseVals_1__55_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0500); // AfitBaseVals_1__56_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x032D); // AfitBaseVals_1__57_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x324D); // AfitBaseVals_1__58_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E); // AfitBaseVals_1__59_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0200); // AfitBaseVals_1__60_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103); // AfitBaseVals_1__61_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010C); // AfitBaseVals_1__62_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x9696); // AfitBaseVals_1__63_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2F34); // AfitBaseVals_1__64_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0504); // AfitBaseVals_1__65_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080F); // AfitBaseVals_1__66_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__67_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F); // AfitBaseVals_1__68_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3208); // AfitBaseVals_1__69_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F1E); // AfitBaseVals_1__70_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x020F); // AfitBaseVals_1__71_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103); // AfitBaseVals_1__72_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010C); // AfitBaseVals_1__73_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x9696); // AfitBaseVals_1__74_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1414); // AfitBaseVals_1__75_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0504); // AfitBaseVals_1__76_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080F); // AfitBaseVals_1__77_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_1__78_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F); // AfitBaseVals_1__79_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3208); // AfitBaseVals_1__80_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F1E); // AfitBaseVals_1__81_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x020F); // AfitBaseVals_1__82_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0003); // AfitBaseVals_1__83_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__0_  Brightness[2] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__1_  Contrast[2] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFC); // 0000 AfitBaseVals_2__2_  Saturation[2] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__3_  Sharp_Blur[2] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__4_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); // AfitBaseVals_2__5_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x012C); // AfitBaseVals_2__6_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FF); // AfitBaseVals_2__7_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_2__8_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); // AfitBaseVals_2__9_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000C); // AfitBaseVals_2__10_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0010); // AfitBaseVals_2__11_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01E6); // AfitBaseVals_2__12_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FF); // AfitBaseVals_2__13_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0070); // AfitBaseVals_2__14_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007D); // AfitBaseVals_2__15_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); // AfitBaseVals_2__16_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0032); // 0096 AfitBaseVals_2__17_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003C); // AfitBaseVals_2__18_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0073); // AfitBaseVals_2__19_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0087); // AfitBaseVals_2__20_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_2__21_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0019); // AfitBaseVals_2__22_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); // AfitBaseVals_2__23_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E); // AfitBaseVals_2__24_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_2__25_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0019); // AfitBaseVals_2__26_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); // AfitBaseVals_2__27_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E); // AfitBaseVals_2__28_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2B32); // AfitBaseVals_2__29_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0601); // AfitBaseVals_2__30_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__31_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__32_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__33_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00FF); // AfitBaseVals_2__34_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x07FF); // AfitBaseVals_2__35_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFF); // AfitBaseVals_2__36_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__37_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x050D); // AfitBaseVals_2__38_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1E80); // AfitBaseVals_2__39_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__40_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A03); // 0A08 AfitBaseVals_2__41_ iNearGrayDesat[2] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0200); // AfitBaseVals_2__42_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF01); // AfitBaseVals_2__43_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x180F); // AfitBaseVals_2__44_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); // AfitBaseVals_2__45_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__46_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x8003); // AfitBaseVals_2__47_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080); // AfitBaseVals_2__48_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080); // AfitBaseVals_2__49_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0280); // AfitBaseVals_2__50_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0208); // AfitBaseVals_2__51_ iClustThresh_H[2] iClustMulT_H[2] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1E4B); // AfitBaseVals_2__52_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1A24); // AfitBaseVals_2__53_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A05); // AfitBaseVals_2__54_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080A); // AfitBaseVals_2__55_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0500); // AfitBaseVals_2__56_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x032D); // AfitBaseVals_2__57_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x324D); // AfitBaseVals_2__58_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E); // AfitBaseVals_2__59_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0200); // AfitBaseVals_2__60_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103); // AfitBaseVals_2__61_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010C); // AfitBaseVals_2__62_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x9696); // AfitBaseVals_2__63_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1E23); // AfitBaseVals_2__64_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0505); // AfitBaseVals_2__65_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080F); // AfitBaseVals_2__66_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__67_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F); // AfitBaseVals_2__68_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3208); // AfitBaseVals_2__69_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F1E); // AfitBaseVals_2__70_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x020F); // AfitBaseVals_2__71_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103); // AfitBaseVals_2__72_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010C); // AfitBaseVals_2__73_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x9696); // AfitBaseVals_2__74_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1E23); // AfitBaseVals_2__75_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0505); // AfitBaseVals_2__76_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080F); // AfitBaseVals_2__77_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_2__78_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F); // AfitBaseVals_2__79_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3208); // AfitBaseVals_2__80_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F1E); // AfitBaseVals_2__81_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x020F); // AfitBaseVals_2__82_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0003); // AfitBaseVals_2__83_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__0_  Brightness[3] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // 0000 AfitBaseVals_3__1_  Contrast[3] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFA); // AfitBaseVals_3__2_  Saturation[3] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__3_  Sharp_Blur[3] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__4_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); // AfitBaseVals_3__5_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x012C); // AfitBaseVals_3__6_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FF); // AfitBaseVals_3__7_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_3__8_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); // AfitBaseVals_3__9_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000C); // AfitBaseVals_3__10_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0010); // AfitBaseVals_3__11_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01E6); // AfitBaseVals_3__12_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__13_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0070); // AfitBaseVals_3__14_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x007D); // AfitBaseVals_3__15_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); // AfitBaseVals_3__16_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0032); // 0096 AfitBaseVals_3__17_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003C); // AfitBaseVals_3__18_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0073); // AfitBaseVals_3__19_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x009F); // AfitBaseVals_3__20_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); // AfitBaseVals_3__21_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); // AfitBaseVals_3__22_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); // AfitBaseVals_3__23_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0037); // AfitBaseVals_3__24_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); // AfitBaseVals_3__25_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); // AfitBaseVals_3__26_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); // AfitBaseVals_3__27_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0037); // AfitBaseVals_3__28_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2B32); // AfitBaseVals_3__29_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0601); // AfitBaseVals_3__30_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__31_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__32_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__33_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00FF); // AfitBaseVals_3__34_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x07A0); // AfitBaseVals_3__35_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFF); // AfitBaseVals_3__36_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__37_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x050D); // AfitBaseVals_3__38_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1E80); // AfitBaseVals_3__39_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__40_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A03); // 0A08 AfitBaseVals_3__41_ iNearGrayDesat[3] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0200); // AfitBaseVals_3__42_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF01); // AfitBaseVals_3__43_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x180F); // AfitBaseVals_3__44_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001); // AfitBaseVals_3__45_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__46_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x8003); // AfitBaseVals_3__47_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080); // AfitBaseVals_3__48_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080); // AfitBaseVals_3__49_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0280); // AfitBaseVals_3__50_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0108); // AfitBaseVals_3__51_ iClustThresh_H[3] iClustMulT_H[3] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1E32); // AfitBaseVals_3__52_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1A24); // AfitBaseVals_3__53_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A05); // AfitBaseVals_3__54_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080A); // AfitBaseVals_3__55_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__56_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0328); // AfitBaseVals_3__57_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x324C); // AfitBaseVals_3__58_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E); // AfitBaseVals_3__59_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0200); // AfitBaseVals_3__60_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103); // AfitBaseVals_3__61_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010C); // AfitBaseVals_3__62_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x9696); // AfitBaseVals_3__63_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F0F); // AfitBaseVals_3__64_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0307); // AfitBaseVals_3__65_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080F); // AfitBaseVals_3__66_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__67_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F); // AfitBaseVals_3__68_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3208); // AfitBaseVals_3__69_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F1E); // AfitBaseVals_3__70_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x020F); // AfitBaseVals_3__71_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103); // AfitBaseVals_3__72_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010C); // AfitBaseVals_3__73_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x9696); // AfitBaseVals_3__74_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F0F); // AfitBaseVals_3__75_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0307); // AfitBaseVals_3__76_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080F); // AfitBaseVals_3__77_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_3__78_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F); // AfitBaseVals_3__79_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3208); // AfitBaseVals_3__80_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F1E); // AfitBaseVals_3__81_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x020F); // AfitBaseVals_3__82_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0003); // AfitBaseVals_3__83_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__0_  Brightness[4] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // 0000 AfitBaseVals_4__1_  Contrast[4] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFF8); // AfitBaseVals_4__2_  Saturation[4] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__3_  Sharp_Blur[4] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__4_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); // AfitBaseVals_4__5_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x012C); // AfitBaseVals_4__6_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03FF); // AfitBaseVals_4__7_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0014); // AfitBaseVals_4__8_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0064); // AfitBaseVals_4__9_  */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x000C); // AfitBaseVals_4__10_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0010); // AfitBaseVals_4__11_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x01E6); // AfitBaseVals_4__12_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__13_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0070); // AfitBaseVals_4__14_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0087); // AfitBaseVals_4__15_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0073); // AfitBaseVals_4__16_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0032); // 0096 AfitBaseVals_4__17_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x003C); // AfitBaseVals_4__18_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0073); // AfitBaseVals_4__19_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00B4); // AfitBaseVals_4__20_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); // AfitBaseVals_4__21_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); // AfitBaseVals_4__22_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); // AfitBaseVals_4__23_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0046); // AfitBaseVals_4__24_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); // AfitBaseVals_4__25_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0028); // AfitBaseVals_4__26_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0023); // AfitBaseVals_4__27_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0046); // AfitBaseVals_4__28_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2B23); // AfitBaseVals_4__29_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0601); // AfitBaseVals_4__30_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__31_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__32_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__33_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x00FF); // AfitBaseVals_4__34_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0B84); // AfitBaseVals_4__35_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFFFF); // AfitBaseVals_4__36_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__37_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x050D); // AfitBaseVals_4__38_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1E80); // AfitBaseVals_4__39_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__40_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A03); // 0A08 AfitBaseVals_4__41_ iNearGrayDesat[4] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0200); // AfitBaseVals_4__42_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFF01); // AfitBaseVals_4__43_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x180F); // AfitBaseVals_4__44_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001); // AfitBaseVals_4__45_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__46_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x8003); // AfitBaseVals_4__47_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080); // AfitBaseVals_4__48_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0080); // AfitBaseVals_4__49_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0280); // AfitBaseVals_4__50_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0108); // AfitBaseVals_4__51_ iClustThresh_H[4] iClustMulT_H[4] */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1E1E); // AfitBaseVals_4__52_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x1419); // AfitBaseVals_4__53_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0A0A); // AfitBaseVals_4__54_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0800); // AfitBaseVals_4__55_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__56_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0328); // AfitBaseVals_4__57_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x324C); // AfitBaseVals_4__58_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x001E); // AfitBaseVals_4__59_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0200); // AfitBaseVals_4__60_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103); // AfitBaseVals_4__61_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010C); // AfitBaseVals_4__62_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x6464); // AfitBaseVals_4__63_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F0F); // AfitBaseVals_4__64_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0307); // AfitBaseVals_4__65_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080F); // AfitBaseVals_4__66_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__67_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F); // AfitBaseVals_4__68_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3208); // AfitBaseVals_4__69_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F1E); // AfitBaseVals_4__70_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x020F); // AfitBaseVals_4__71_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0103); // AfitBaseVals_4__72_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x010C); // AfitBaseVals_4__73_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x6464); // AfitBaseVals_4__74_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F0F); // AfitBaseVals_4__75_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0307); // AfitBaseVals_4__76_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x080F); // AfitBaseVals_4__77_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); // AfitBaseVals_4__78_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x030F); // AfitBaseVals_4__79_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x3208); // AfitBaseVals_4__80_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0F1E); // AfitBaseVals_4__81_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x020F); // AfitBaseVals_4__82_ */
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0003); // AfitBaseVals_4__83_ */

  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x7F5E);  //ConstAfitBaseVals_0_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xFEEE);  //ConstAfitBaseVals_1_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xD9B7);  //ConstAfitBaseVals_2_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0472);  //ConstAfitBaseVals_3_
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  //ConstAfitBaseVals_4_

  S5K8AAYX_write_cmos_sensor(0x002A ,0x1278);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0xAAF0); //gisp_dadlc  Ladlc mode average
  //(0x002A ,0x0408);        chris 20130322
  //(0x0F12 ,0x067F); //REG_TC_DBG_AutoAlgEnBits all AA are on   chris 20130322
  //============================================================
  // User Control
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x018E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //Brightness
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //contrast
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0010); //Saturation
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //sharpness

  //============================================================
  // Flicker
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0408);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x065F);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x03F4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);

  //============================================================
  // PLL
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x012E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,S5K8AAYX_MCLK*1000); //input clock
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0146);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); //REG_TC_IPRM_UseNPviClocks
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_IPRM_UseNMipiClocks

  S5K8AAYX_write_cmos_sensor(0x002A ,0x014C);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2C4A); //REG_TC_IPRM_sysClocks_0 45.352MHz chris 20130322
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0152);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x57E4); //REG_TC_IPRM_MinOutRate4KHz_0 90MHz chris 20130322
  S5K8AAYX_write_cmos_sensor(0x002A ,0x014E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x57E4); //REG_TC_IPRM_MaxOutRate4KHz_0 90MHz chris 20130322
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0154);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x2981); //29FE //REG_TC_IPRM_sysClocks_1
  S5K8AAYX_write_cmos_sensor(0x002A ,0x015A);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x5208); //5302  //REG_TC_IPRM_MinOutRate4KHz_1
  S5K8AAYX_write_cmos_sensor(0x002A ,0x0156);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x53FC); //54F6 //REG_TC_IPRM_MaxOutRate4KHz_1

  S5K8AAYX_write_cmos_sensor(0x002A ,0x0164); //update PLL
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);

  //============================================================
  // Preview config0 1280*960 10~30fps
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x01BE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0500); //REG_0TC_PCFG_usWidth//1280
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03C0); //REG_0TC_PCFG_usHeight//960
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005); //REG_0TC_PCFG_Format
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0060); //0x40:OK//REG_0TC_PCFG_PVIMask
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_0TC_PCFG_OIFMask
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_0TC_PCFG_uClockInd
  S5K8AAYX_write_cmos_sensor(0x002A ,0x01D2);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_0TC_PCFG_usFrTimeType
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); //REG_0TC_PCFG_FrRateQualityType
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_0TC_PCFG_usMinFrTimeMsecMult10
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03E8); //REG_0TC_PCFG_usMaxFrTimeMsecMult10
  S5K8AAYX_write_cmos_sensor(0x002A ,0x01E8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_0TC_PCFG_uPrevMirror
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_0TC_PCFG_uPCaptureMirror

  //============================================================
  // Capture configuration 0  7.5fps~30fps
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x02AE);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_0TC_CCFG_uCaptureMode
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0500); //REG_0TC_CCFG_usWidth
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x03C0); //REG_0TC_CCFG_usHeight
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0005); //REG_0TC_CCFG_Format
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0060); //REG_0TC_CCFG_PVIMask
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_0TC_CCFG_OIFMask
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_0TC_CCFG_uClockInd
  S5K8AAYX_write_cmos_sensor(0x002A ,0x02C4);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_0TC_CCFG_usFrTimeType
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0002); //REG_0TC_CCFG_FrRateQualityType
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000); //REG_0TC_CCFG_usMinFrTimeMsecMult10
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0535); //REG_0TC_CCFG_usMaxFrTimeMsecMult10

  //============================================================
  // active preview configure
  //============================================================
  S5K8AAYX_write_cmos_sensor(0x002A ,0x01A8);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #REG_TC_GP_ActivePrevConfig
  S5K8AAYX_write_cmos_sensor(0x002A ,0x01AC);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  // #REG_TC_GP_PrevOpenAfterChange
  S5K8AAYX_write_cmos_sensor(0x002A ,0x01A6);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  // #REG_TC_GP_NewConfigSync
  S5K8AAYX_write_cmos_sensor(0x002A ,0x01AA);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  // #REG_TC_GP_PrevConfigChanged
  S5K8AAYX_write_cmos_sensor(0x002A ,0x019E);
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  // #REG_TC_GP_EnablePreview
  S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  // #REG_TC_GP_EnablePreviewChanged

  S5K8AAYX_write_cmos_sensor(0x0028 ,0xD000);
  S5K8AAYX_write_cmos_sensor(0x002A ,0x1000);  // chris 20130322
  S5K8AAYX_write_cmos_sensor(0x1000 ,0x0001);
}


/*****************************************************************************
 * FUNCTION
 *  S5K8AAYX_PV_Mode
 * DESCRIPTION
 *
 * PARAMETERS
 *  void
 * RETURNS
 *  void
 *****************************************************************************/
void S5K8AAYX_PV_Mode(void)
{
    SENSORDB("Enter S5K8AAYX_PV_Mode\n");

#if 0
    S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
    S5K8AAYX_write_cmos_sensor(0x002A, 0x01D6);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x01A0); //REG_0TC_PCFG_usMinFrTimeMsecMult10
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x03e8); //REG_0TC_PCFG_usMaxFrTimeMsecMult10 fps
    S5K8AAYX_write_cmos_sensor(0x002A, 0x01A8);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000); // REG_TC_GP_ActivePrevConfig
    S5K8AAYX_write_cmos_sensor(0x002A, 0x01AA);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); // REG_TC_GP_PreviewConfigChanged
    S5K8AAYX_write_cmos_sensor(0x002A, 0x019E);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); // REG_TC_GP_EnablePreview
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); // REG_TC_GP_EnablePreviewChanged
    S5K8AAYX_write_cmos_sensor(0x002A, 0x0164);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); // REG_TC_IPRM_InitParamsUpdated
    S5K8AAYX_write_cmos_sensor(0x0028, 0xD000);
    S5K8AAYX_write_cmos_sensor(0x002A, 0x1000);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); // Set host interrupt
#else
    S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
    S5K8AAYX_write_cmos_sensor(0x002A ,0x01A8);
    S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0000);  // #REG_TC_GP_ActivePrevConfig
    S5K8AAYX_write_cmos_sensor(0x002A ,0x01AC);
    S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  // #REG_TC_GP_PrevOpenAfterChange
    S5K8AAYX_write_cmos_sensor(0x002A ,0x01A6);
    S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  // #REG_TC_GP_NewConfigSync
    S5K8AAYX_write_cmos_sensor(0x002A ,0x01AA);
    S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  // #REG_TC_GP_PrevConfigChanged
    S5K8AAYX_write_cmos_sensor(0x002A ,0x019E);
    S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  // #REG_TC_GP_EnablePreview
    S5K8AAYX_write_cmos_sensor(0x0F12 ,0x0001);  // #REG_TC_GP_EnablePreviewChanged

    S5K8AAYX_write_cmos_sensor(0x002A, 0x0164);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); // REG_TC_IPRM_InitParamsUpdated

#endif
}



/*****************************************************************************
 * FUNCTION
 *  S5K8AAYX_CAP_Mode
 * DESCRIPTION
 *
 * PARAMETERS
 *  void
 * RETURNS
 *  void
 *****************************************************************************/
void S5K8AAYX_CAP_Mode2(void)
{
//
}


void S5K8AAYX_CAP_Mode(void)
{
    S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
    S5K8AAYX_write_cmos_sensor(0x002A, 0x01B0);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);  //config select   0 :48, 1, 20,
    S5K8AAYX_write_cmos_sensor(0x002a, 0x01A6);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
    S5K8AAYX_write_cmos_sensor(0x002a, 0x01B2);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
    S5K8AAYX_write_cmos_sensor(0x002a, 0x01A2);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
    S5K8AAYX_write_cmos_sensor(0x002a, 0x01A4);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
    mdelay(50);
    //S5K5CAGX_gPVmode = KAL_FALSE;
}


/*void S5K8AAYX_AE_AWB_Enable(kal_bool enable)
{
    S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
    S5K8AAYX_write_cmos_sensor(0x002A, 0x04D2);           // REG_TC_DBG_AutoAlgEnBits

    if (enable)
    {
        // Enable AE/AWB
        S5K8AAYX_write_cmos_sensor(0x0F12, 0x077F); // Enable aa_all, ae, awb.
    }
    else
    {
        // Disable AE/AWB
        S5K8AAYX_write_cmos_sensor(0x0F12, 0x0770); // Disable aa_all, ae, awb.
    }

}*/
static void S5K8AAYX_set_AE_mode(kal_bool AE_enable)
{
    if(AE_enable==KAL_TRUE)
    {
        S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000); // Set page
        S5K8AAYX_write_cmos_sensor(0x0028, 0x7000); // Set address

        S5K8AAYX_write_cmos_sensor(0x002A, 0x214A);
        S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
    }
    else
    {
        S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000); // Set page
        S5K8AAYX_write_cmos_sensor(0x0028, 0x7000); // Set address

        S5K8AAYX_write_cmos_sensor(0x002A, 0x214A);
        S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
    }

}


/*************************************************************************
* FUNCTION
*       S5K8AAYX_night_mode
*
* DESCRIPTION
*       This function night mode of S5K8AAYX.
*
* PARAMETERS
*       none
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void S5K8AAYX_night_mode(kal_bool enable)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                                                                             */
    /*----------------------------------------------------------------*/

    /*----------------------------------------------------------------*/
    /* Code Body                                                                                                                     */
    /*----------------------------------------------------------------*/
    kal_uint16 video_frame_len = 0;
    kal_uint16 prev_line_len = 0;

    SENSORDB("S5K8AAYX_night_mode [in] enable=%d \r\n",enable);

    if(S5K8AAYXCurrentStatus.iNightMode == enable)
        return;

    if (S5K8AAYX_sensor_cap_state == KAL_TRUE)
              return ;    //Don't need rewrite the setting when capture.
    S5K8AAYX_write_cmos_sensor(0x0028, 0x7000); // Set address

    if (enable)
    {
        if (S5K8AAYX_MPEG4_encode_mode == KAL_TRUE)
        {
            S5K8AAYX_write_cmos_sensor(0x002A,0x01D2);
            S5K8AAYX_write_cmos_sensor(0x0F12,0x0002);  //REG_0TC_PCFG_usFrTimeType
            S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  //REG_0TC_PCFG_FrRateQualityType
            S5K8AAYX_write_cmos_sensor(0x002A,0x01D6);
            S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_VID_NIT_FIX_FR_TIME); //REG_0TC_PCFG_usMinFrTimeMsecMult10
            S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_VID_NIT_FIX_FR_TIME); //REG_0TC_PCFG_usMaxFrTimeMsecMult10 fps
        }
        else
        {
            S5K8AAYX_write_cmos_sensor(0x002A,0x01D2);
            S5K8AAYX_write_cmos_sensor(0x0F12,0x0000);  //REG_0TC_PCFG_usFrTimeType
            S5K8AAYX_write_cmos_sensor(0x0F12,0x0002);  //REG_0TC_PCFG_FrRateQualityType

            S5K8AAYX_write_cmos_sensor(0x002A,0x01D6);
            S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_CAM_NIT_MIN_FR_TIME); //REG_0TC_PCFG_usMinFrTimeMsecMult10
            S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_CAM_NIT_MAX_FR_TIME);         //REG_0TC_PCFG_usMaxFrTimeMsecMult10

        }

        S5K8AAYX_write_cmos_sensor(0x002A,0x0468);
        S5K8AAYX_write_cmos_sensor(0x0F12,0x0140);  //lt_uMaxDigGain

        S5K8AAYX_night_mode_enable = KAL_TRUE;
    }
    else
    {
        if (S5K8AAYX_MPEG4_encode_mode == KAL_TRUE)
        {
            S5K8AAYX_write_cmos_sensor(0x0F12, S5K8AAYX_VID_NOM_FIX_FR_TIME); //#REG_0TC_PCFG_usMaxFrTimeMsecMult10
            S5K8AAYX_write_cmos_sensor(0x0F12, S5K8AAYX_VID_NOM_FIX_FR_TIME); //#REG_0TC_PCFG_usMinFrTimeMsecMult10
        }
        else
        {
            S5K8AAYX_write_cmos_sensor(0x002A,0x01D2);
            S5K8AAYX_write_cmos_sensor(0x0F12,0x0000);  //REG_0TC_PCFG_usFrTimeType
            S5K8AAYX_write_cmos_sensor(0x0F12,0x0002);  //REG_0TC_PCFG_FrRateQualityType

            S5K8AAYX_write_cmos_sensor(0x002A,0x01D6);
            S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_CAM_NOM_MIN_FR_TIME); //REG_0TC_PCFG_usMinFrTimeMsecMult10
            S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_CAM_NOM_MAX_FR_TIME);         //REG_0TC_PCFG_usMaxFrTimeMsecMult10
        }

        S5K8AAYX_write_cmos_sensor(0x002A,0x0468);
        S5K8AAYX_write_cmos_sensor(0x0F12,0x0100);  //lt_uMaxDigGain

        S5K8AAYX_night_mode_enable = KAL_FALSE;
    }

    // active preview configure
    //============================================================
    S5K8AAYX_write_cmos_sensor(0x002A,0x01A8);
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0000);  // #REG_TC_GP_ActivePrevConfig // Select preview configuration_0
    S5K8AAYX_write_cmos_sensor(0x002A,0x01AC);
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_PrevOpenAfterChange
    S5K8AAYX_write_cmos_sensor(0x002A,0x01A6);
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_NewConfigSync // Update preview configuration
    S5K8AAYX_write_cmos_sensor(0x002A,0x01AA);
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_PrevConfigChanged
    S5K8AAYX_write_cmos_sensor(0x002A,0x019E);
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_EnablePreview // Start preview
    S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_EnablePreviewChanged

    S5K8AAYXCurrentStatus.iNightMode = enable;

} /* S5K8AAYX_night_mode */

/*************************************************************************
* FUNCTION
*       S5K8AAYX_GetSensorID
*
* DESCRIPTION
*       This function get the sensor ID
*
* PARAMETERS
*       None
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
  kal_uint32 S5K8AAYX_GetSensorID(kal_uint32 *sensorID)
{
    kal_uint16 sensor_id = 0;
    unsigned short version = 0;
    signed int retry = 3;

    //check if sensor ID correct
    while(--retry)
    {
        S5K8AAYX_write_cmos_sensor(0xFCFC, 0x0000);
        sensor_id = S5K8AAYX_read_cmos_sensor(0x0040);
        SENSORDB("[8AA]Sensor Read S5K8AAYX ID: 0x%x \r\n", sensor_id);

        *sensorID = sensor_id;
        if (sensor_id == S5K8AAYX_SENSOR_ID)
        {
            SENSORDB("[8AA] Sensor ID: 0x%x, Read OK \r\n", sensor_id);
            //version=S5K8AAYX_read_cmos_sensor(0x0042);
            //SENSORDB("[8AA]~~~~~~~~~~~~~~~~ S5K8AAYX version: 0x%x \r\n",version);
            return ERROR_NONE;
        }
    }
    *sensorID = 0xFFFFFFFF;
    SENSORDB("[8AA] Sensor Read Fail \r\n");
    return ERROR_SENSOR_CONNECT_FAIL;
}


/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*       S5K8AAYXOpen
*
* DESCRIPTION
*       This function initialize the registers of CMOS sensor
*
* PARAMETERS
*       None
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K8AAYXOpen(void)
{
    kal_uint16 sensor_id=0;

    S5K8AAYX_GetSensorID(&sensor_id);
    S5K8AAYXInitialPara();
    S5K8AAYX_Initialize_Setting();
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*       S5K8AAYXClose
*
* DESCRIPTION
*       This function is to turn off sensor module power.
*
* PARAMETERS
*       None
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K8AAYXClose(void)
{
     return ERROR_NONE;
}        /* S5K8AAYXClose() */

/*************************************************************************
* FUNCTION
*       S5K8AAYXPreview
*
* DESCRIPTION
*       This function start the sensor preview.
*
* PARAMETERS
*       *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K8AAYXPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    SENSORDB("Enter S5K8AAYXPreview \n");
    spin_lock(&s5k8aayx_drv_lock);
    S5K8AAYX_sensor_cap_state = KAL_FALSE;
    //S5K8AAYX_PV_dummy_pixels = 0x0400;
    S5K8AAYX_PV_dummy_lines = 0;

    if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO) // MPEG4 Encode Mode
    {
        SENSORDB("S5K8AAYXPreview MSDK_SENSOR_OPERATION_MODE_VIDEO \n");
        S5K8AAYX_MPEG4_encode_mode = KAL_TRUE;
        S5K8AAYX_MJPEG_encode_mode = KAL_FALSE;
    }
    else
    {
        SENSORDB("S5K8AAYXPreview Normal \n");
        S5K8AAYX_MPEG4_encode_mode = KAL_FALSE;
        S5K8AAYX_MJPEG_encode_mode = KAL_FALSE;
    }
    spin_unlock(&s5k8aayx_drv_lock);

    S5K8AAYX_PV_Mode();

    S5K8AAYX_set_mirror(sensor_config_data->SensorImageMirror);


    image_window->GrabStartX = S5K8AAYX_IMAGE_SENSOR_PV_INSERTED_PIXELS;
    image_window->GrabStartY = S5K8AAYX_IMAGE_SENSOR_PV_INSERTED_LINES;
    image_window->ExposureWindowWidth = S5K8AAYX_IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight = S5K8AAYX_IMAGE_SENSOR_PV_HEIGHT;

    // copy sensor_config_data
    memcpy(&S5K8AAYXSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
}   /* S5K8AAYXPreview() */

UINT32 S5K8AAYXCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint32 pv_integration_time = 0;  // Uinit - us
    kal_uint32 cap_integration_time = 0;
    kal_uint16 PV_line_len = 0;
    kal_uint16 CAP_line_len = 0;
    SENSORDB("Enter S5K8AAYXCapture\n");

    S5K8AAYX_sensor_cap_state = KAL_TRUE;
    S5K8AAYX_CAP_Mode();

    image_window->GrabStartX = S5K8AAYX_IMAGE_SENSOR_FULL_INSERTED_PIXELS;
    image_window->GrabStartY = S5K8AAYX_IMAGE_SENSOR_FULL_INSERTED_LINES;
    image_window->ExposureWindowWidth = S5K8AAYX_IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = S5K8AAYX_IMAGE_SENSOR_FULL_HEIGHT;

    // copy sensor_config_data
    memcpy(&S5K8AAYXSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
}   /* S5K8AAYXCapture() */

UINT32 S5K8AAYXGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("Enter S5K8AAYXGetResolution\n");
    pSensorResolution->SensorFullWidth=S5K8AAYX_IMAGE_SENSOR_FULL_WIDTH;  //modify by yanxu
    pSensorResolution->SensorFullHeight=S5K8AAYX_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=S5K8AAYX_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight=S5K8AAYX_IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorVideoWidth=S5K8AAYX_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorVideoHeight=S5K8AAYX_IMAGE_SENSOR_PV_HEIGHT;
    return ERROR_NONE;
}   /* S5K8AAYXGetResolution() */

UINT32 S5K8AAYXGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                       MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                       MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    SENSORDB("Enter S5K8AAYXGetInfo\n");
    SENSORDB("S5K8AAYXGetInfo ScenarioId =%d \n",ScenarioId);
    switch(ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
             pSensorInfo->SensorPreviewResolutionX=S5K8AAYX_IMAGE_SENSOR_FULL_WIDTH;
             pSensorInfo->SensorPreviewResolutionY=S5K8AAYX_IMAGE_SENSOR_FULL_HEIGHT;
             pSensorInfo->SensorCameraPreviewFrameRate=15;
             break;
        default:
             pSensorInfo->SensorPreviewResolutionX=S5K8AAYX_IMAGE_SENSOR_PV_WIDTH;
             pSensorInfo->SensorPreviewResolutionY=S5K8AAYX_IMAGE_SENSOR_PV_HEIGHT;
             pSensorInfo->SensorCameraPreviewFrameRate=30;
             break;
    }
    pSensorInfo->SensorFullResolutionX=S5K8AAYX_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=S5K8AAYX_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=1;

    pSensorInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_YUYV;

    pSensorInfo->SensorClockPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_HIGH;

    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;

    #ifdef MIPI_INTERFACE
        pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_MIPI;
    #else
        pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_PARALLEL;
    #endif

    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_2MA;//ISP_DRIVING_4MA;
    pSensorInfo->CaptureDelayFrame = 3;
    pSensorInfo->PreviewDelayFrame = 3;
    pSensorInfo->VideoDelayFrame = 4;

    pSensorInfo->YUVAwbDelayFrame = 3;
    pSensorInfo->YUVEffectDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
             pSensorInfo->SensorClockFreq = S5K8AAYX_MCLK;
             pSensorInfo->SensorClockDividCount = 3;
             pSensorInfo->SensorClockRisingCount = 0;
             pSensorInfo->SensorClockFallingCount = 2;
             pSensorInfo->SensorPixelClockCount = 3;
             pSensorInfo->SensorDataLatchCount = 2;
             pSensorInfo->SensorGrabStartX = 2;
             pSensorInfo->SensorGrabStartY = 2;
             break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
             pSensorInfo->SensorClockFreq=S5K8AAYX_MCLK;
             pSensorInfo->SensorClockDividCount=   3;
             pSensorInfo->SensorClockRisingCount= 0;
             pSensorInfo->SensorClockFallingCount= 2;
             pSensorInfo->SensorPixelClockCount= 3;
             pSensorInfo->SensorDataLatchCount= 2;
             pSensorInfo->SensorGrabStartX = 2;
             pSensorInfo->SensorGrabStartY = 2;
             break;
        default:
             pSensorInfo->SensorClockFreq=S5K8AAYX_MCLK;
             pSensorInfo->SensorClockDividCount=3;
             pSensorInfo->SensorClockRisingCount=0;
             pSensorInfo->SensorClockFallingCount=2;
             pSensorInfo->SensorPixelClockCount=3;
             pSensorInfo->SensorDataLatchCount=2;
             pSensorInfo->SensorGrabStartX = 2;
             pSensorInfo->SensorGrabStartY = 2;
              break;
    }
    memcpy(pSensorConfigData, &S5K8AAYXSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
}        /* S5K8AAYXGetInfo() */


UINT32 S5K8AAYXControl(MSDK_SCENARIO_ID_ENUM ScenarioId,
                       MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                       MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    SENSORDB("Enter S5K8AAYXControl\n");
    SENSORDB(" S5K8AAYXControl ScenarioId = %d ,\n",ScenarioId);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
             SENSORDB("YSZ_S5K8AAYX_S5K8AAYXControl_preview");
             S5K8AAYXPreview(pImageWindow, pSensorConfigData);
             break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
             S5K8AAYXCapture(pImageWindow, pSensorConfigData);
             break;
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
             S5K8AAYXCapture(pImageWindow, pSensorConfigData);
             break;
        default:
           return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}   /* S5K8AAYXControl() */

/* [TC] YUV sensor */
#if 0
#if WINMO_USE
void S5K8AAYXQuery(PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo)
{
    SENSORDB("Enter S5K8AAYXQuery\n");
    MSDK_FEATURE_TYPE_RANGE_STRUCT *pFeatureRange;
    MSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT *pFeatureMultiSelection;
    switch (pSensorFeatureInfo->FeatureId)
    {
       case ISP_FEATURE_DSC_MODE:
            pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
            pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_SUPPORTED);
            pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
            pFeatureMultiSelection->TotalSelection = CAM_NO_OF_SCENE_MODE_MAX-2;
            pFeatureMultiSelection->DefaultSelection = CAM_AUTO_DSC_MODE;
            pFeatureMultiSelection->SupportedSelection =
                  (CAMERA_FEATURE_SUPPORT(CAM_AUTO_DSC_MODE)|
                  CAMERA_FEATURE_SUPPORT(CAM_NIGHTSCENE_MODE));
            break;
       case ISP_FEATURE_WHITEBALANCE:
            pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
            pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
            pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
            pFeatureMultiSelection->TotalSelection = CAM_NO_OF_WB;
            pFeatureMultiSelection->DefaultSelection = CAM_WB_AUTO;
            pFeatureMultiSelection->SupportedSelection =
                     (CAMERA_FEATURE_SUPPORT(CAM_WB_AUTO)|
                     CAMERA_FEATURE_SUPPORT(CAM_WB_CLOUD)|
                     CAMERA_FEATURE_SUPPORT(CAM_WB_DAYLIGHT)|
                     CAMERA_FEATURE_SUPPORT(CAM_WB_INCANDESCENCE)|
                     CAMERA_FEATURE_SUPPORT(CAM_WB_FLUORESCENT));
            break;
       case ISP_FEATURE_IMAGE_EFFECT:
            pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
            pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
            pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
            pFeatureMultiSelection->TotalSelection = CAM_NO_OF_EFFECT_ENC;
            pFeatureMultiSelection->DefaultSelection = CAM_EFFECT_ENC_NORMAL;
            pFeatureMultiSelection->SupportedSelection =
                     (CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_NORMAL)|
                     CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_GRAYSCALE)|
                     CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_COLORINV)|
                     CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_GRAYINV)|
                     CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIABLUE)|
                     CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SKETCH)|
                     CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_EMBOSSMENT)|
                     CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIA));
            break;
       case ISP_FEATURE_AE_METERING_MODE:
            pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
            break;
       case ISP_FEATURE_BRIGHTNESS:
            pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_RANGE;
            pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
            pFeatureRange = (PMSDK_FEATURE_TYPE_RANGE_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureRange);
            pFeatureRange->MinValue = CAM_EV_NEG_4_3;
            pFeatureRange->MaxValue = CAM_EV_POS_4_3;
            pFeatureRange->StepValue = CAMERA_FEATURE_ID_EV_STEP;
            pFeatureRange->DefaultValue = CAM_EV_ZERO;
            break;
       case ISP_FEATURE_BANDING_FREQ:
            pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
            pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
            pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
            pFeatureMultiSelection->TotalSelection = CAM_NO_OF_BANDING;
            pFeatureMultiSelection->DefaultSelection = CAM_BANDING_50HZ;
            pFeatureMultiSelection->SupportedSelection =
                     (CAMERA_FEATURE_SUPPORT(CAM_BANDING_50HZ)|
                     CAMERA_FEATURE_SUPPORT(CAM_BANDING_60HZ));
            break;
       case ISP_FEATURE_AF_OPERATION:
            pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
            break;
       case ISP_FEATURE_AF_RANGE_CONTROL:
            pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
            break;
       case ISP_FEATURE_FLASH:
            pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
            break;
       case ISP_FEATURE_VIDEO_SCENE_MODE:
            pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
            pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_VIDEO_SUPPORTED);
            pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
            pFeatureMultiSelection->TotalSelection = 2;
            pFeatureMultiSelection->DefaultSelection = CAM_VIDEO_AUTO_MODE;
            pFeatureMultiSelection->SupportedSelection =
                     (CAMERA_FEATURE_SUPPORT(CAM_VIDEO_AUTO_MODE)|
                     CAMERA_FEATURE_SUPPORT(CAM_VIDEO_NIGHT_MODE));
            break;
       case ISP_FEATURE_ISO:
            pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
            break;
       default:
            pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
            break;
    }
}
#endif
#endif


BOOL S5K8AAYX_set_param_wb(UINT16 para)
{
    SENSORDB("Enter S5K8AAYX_set_param_wb\n");
    if(S5K8AAYXCurrentStatus.iWB == para)
        return TRUE;
    SENSORDB("[Enter]S5K8AAYX set_param_wb func:para = %d\n",para);

    S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000); // Set page
    S5K8AAYX_write_cmos_sensor(0x0028, 0x7000); // Set address

    switch (para)
    {
        case AWB_MODE_AUTO:
             S5K8AAYX_write_cmos_sensor(0x002A, 0x0408); //bit[3]:AWB Auto:1 menual:0
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x067F);
             break;
        case AWB_MODE_CLOUDY_DAYLIGHT:
             //======================================================================
             //      MWB : Cloudy_D65
             //======================================================================
             S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
             S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
             S5K8AAYX_write_cmos_sensor(0x002A, 0x0408);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0677);
             S5K8AAYX_write_cmos_sensor(0x002A, 0x03DA);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0740);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0460);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
             break;
        case AWB_MODE_DAYLIGHT:
             //==============================================
             //      MWB : sun&daylight_D50
             //==============================================
             S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
             S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
             S5K8AAYX_write_cmos_sensor(0x002A, 0x0408);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0677);
             S5K8AAYX_write_cmos_sensor(0x002A, 0x03DA);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x05E0);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0530);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
             break;
        case AWB_MODE_INCANDESCENT:
             //==============================================
             //      MWB : Incand_Tungsten
             //==============================================
             S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
             S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
             S5K8AAYX_write_cmos_sensor(0x002A, 0x0408);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0677);
             S5K8AAYX_write_cmos_sensor(0x002A, 0x03DA);
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x5DC0); //Reg_sf_user_Rgain
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RgainChanged
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400); //Reg_sf_user_Ggain
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_GgainChanged
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x08B0); //Reg_sf_user_Bgain
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_BgainChanged
             break;
        case AWB_MODE_FLUORESCENT:
              //==================================================================
              //      MWB : Florescent_TL84
              //==================================================================
              S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
              S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
              S5K8AAYX_write_cmos_sensor(0x002A, 0x0408);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0677);
              S5K8AAYX_write_cmos_sensor(0x002A, 0x03DA);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0575);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0800);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
              break;
        case AWB_MODE_TUNGSTEN:
              S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
              S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
              S5K8AAYX_write_cmos_sensor(0x002A, 0x0408);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0677);
              S5K8AAYX_write_cmos_sensor(0x002A, 0x03DA);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0940);
              S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
              break;
        default:
              return KAL_FALSE;

    }
    spin_lock(&s5k8aayx_drv_lock);
    S5K8AAYXCurrentStatus.iWB = para;
    spin_unlock(&s5k8aayx_drv_lock);
    return TRUE;
}

BOOL S5K8AAYX_set_param_effect(UINT16 para)
{
    SENSORDB("Enter S5K8AAYX_set_param_effect\n");
    /*----------------------------------------------------------------*/
    /* Local Variables                                                                                                        */
    /*----------------------------------------------------------------*/
    kal_uint32 ret = KAL_TRUE;

    /*----------------------------------------------------------------*/
    /* Code Body                                                                                                                          */
    /*----------------------------------------------------------------*/

    if(S5K8AAYXCurrentStatus.iEffect == para)
        return TRUE;
    SENSORDB("[Enter]s5k8aayx set_param_effect func:para = %d\n",para);

    S5K8AAYX_write_cmos_sensor(0x0028,0x7000);
    S5K8AAYX_write_cmos_sensor(0x002A,0x019C);
    switch (para)
    {
        case MEFFECT_OFF:
             S5K8AAYX_write_cmos_sensor(0x0F12,0x0000); // Normal,
             break;
        case MEFFECT_MONO:
             S5K8AAYX_write_cmos_sensor(0x0F12,0x0001); // Monochrome (Black & White)
             break;
        case MEFFECT_SEPIA:
             S5K8AAYX_write_cmos_sensor(0x0F12,0x0004); // Sepia
             break;
        case MEFFECT_SEPIABLUE:
             S5K8AAYX_write_cmos_sensor(0x0F12,0x0005); // Aqua (Blue)
             break;
        case MEFFECT_NEGATIVE:
             S5K8AAYX_write_cmos_sensor(0x0F12,0x0002); // Negative Mono
             break;
        default:
             ret = KAL_FALSE;
    }
    spin_lock(&s5k8aayx_drv_lock);
    S5K8AAYXCurrentStatus.iEffect = para;
    spin_unlock(&s5k8aayx_drv_lock);
    return ret;
}


BOOL S5K8AAYX_set_param_banding(UINT16 para)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    SENSORDB("Enter S5K8AAYX_set_param_banding\n");
//#if (defined(S5K8AAYX_MANUAL_ANTI_FLICKER))

    if(S5K8AAYXCurrentStatus.iBanding == para)
        return TRUE;

    switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
             S5K8AAYX_write_cmos_sensor(0x0028,0x7000);
             S5K8AAYX_write_cmos_sensor(0x002a,0x0408);
             S5K8AAYX_write_cmos_sensor(0x0f12,0x065F);
             S5K8AAYX_write_cmos_sensor(0x002a,0x03F4);
             S5K8AAYX_write_cmos_sensor(0x0f12,0x0001); //REG_SF_USER_FlickerQuant 1:50hz  2:60hz
             S5K8AAYX_write_cmos_sensor(0x0f12,0x0001); //REG_SF_USER_FlickerQuantChanged active flicker
             break;
        case AE_FLICKER_MODE_60HZ:
             S5K8AAYX_write_cmos_sensor(0x0028,0x7000);
             S5K8AAYX_write_cmos_sensor(0x002a,0x0408);
             S5K8AAYX_write_cmos_sensor(0x0f12,0x065F);
             S5K8AAYX_write_cmos_sensor(0x002a,0x03F4);
             S5K8AAYX_write_cmos_sensor(0x0f12,0x0002); //REG_SF_USER_FlickerQuant 1:50hz  2:60hz
             S5K8AAYX_write_cmos_sensor(0x0f12,0x0001); //REG_SF_USER_FlickerQuantChanged active flicker
             break;
        default:
             return KAL_FALSE;
    }
    spin_lock(&s5k8aayx_drv_lock);
    S5K8AAYXCurrentStatus.iBanding = para;
    spin_unlock(&s5k8aayx_drv_lock);
    return TRUE;

//#else
    /* Auto anti-flicker method is enabled, then nothing need to do in this function.  */
//#endif /* #if (defined(S5K8AAYX_MANUAL_ANTI_FLICKER)) */
    return KAL_TRUE;
} /* S5K8AAYX_set_param_banding */



BOOL S5K8AAYX_set_param_exposure(UINT16 para)
{
    SENSORDB("Enter S5K8AAYX_set_param_exposure\n");
    if(S5K8AAYXCurrentStatus.iEV == para)
        return TRUE;

    SENSORDB("[Enter]s5k8aayx set_param_exposure func:para = %d\n",para);
    S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
    S5K8AAYX_write_cmos_sensor(0x002a, 0x019A);

    switch (para)
    {
        case AE_EV_COMP_n10:
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080); //EV -1
             break;
        case AE_EV_COMP_00:
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100); //EV 0
             break;
        case AE_EV_COMP_10:
             S5K8AAYX_write_cmos_sensor(0x0F12, 0x0200);  //EV +1
             break;
        case AE_EV_COMP_n13:
        case AE_EV_COMP_n07:
        case AE_EV_COMP_n03:
        case AE_EV_COMP_03:
        case AE_EV_COMP_07:
        case AE_EV_COMP_13:
             break;
        default:
             return FALSE;
    }
    spin_lock(&s5k8aayx_drv_lock);
    S5K8AAYXCurrentStatus.iEV = para;
    spin_unlock(&s5k8aayx_drv_lock);
    return TRUE;

}/* S5K8AAYX_set_param_exposure */


UINT32 S5K8AAYXYUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
    SENSORDB("Enter S5K8AAYXYUVSensorSetting\n");
    switch (iCmd)
    {
        case FID_SCENE_MODE:
            if (iPara == SCENE_MODE_OFF)
            {
                S5K8AAYX_night_mode(0);
            }
            else if (iPara == SCENE_MODE_NIGHTSCENE)
            {
               S5K8AAYX_night_mode(1);
            }
            break;
        case FID_AWB_MODE:
            S5K8AAYX_set_param_wb(iPara);
            break;
        case FID_COLOR_EFFECT:
            S5K8AAYX_set_param_effect(iPara);
            break;
        case FID_AE_EV:
            S5K8AAYX_set_param_exposure(iPara);
            break;
        case FID_AE_FLICKER:
            S5K8AAYX_set_param_banding(iPara);
            break;
        case FID_AE_SCENE_MODE:
            spin_lock(&s5k8aayx_drv_lock);
            if (iPara == AE_MODE_OFF) {
                S5K8AAYX_AE_ENABLE = KAL_FALSE;
            }
            else {
                S5K8AAYX_AE_ENABLE = KAL_TRUE;
            }
            spin_unlock(&s5k8aayx_drv_lock);
            S5K8AAYX_set_AE_mode(S5K8AAYX_AE_ENABLE);
            break;
        case FID_ZOOM_FACTOR:
            zoom_factor = iPara;
            break;
        default:
            break;
    }
    return ERROR_NONE;
}   /* S5K8AAYXYUVSensorSetting */


UINT32 S5K8AAYXYUVSetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("Enter S5K8AAYXYUVSetVideoMode,u2FrameRate=%d\n",u2FrameRate);
    SENSORDB("Enter S5K8AAYXYUVSetVideoMode,u2FrameRate=%d\n",u2FrameRate);
    //if(S5K8AAYXCurrentStatus.iFrameRate == u2FrameRate)
    //   return TRUE;

    S5K8AAYX_VEDIO_encode_mode = KAL_TRUE;
    S5K8AAYX_write_cmos_sensor(0x0028, 0x7000); // Set address
    if (u2FrameRate == 30)
    {
        S5K8AAYX_write_cmos_sensor(0x002A,0x01D2);
        S5K8AAYX_write_cmos_sensor(0x0F12,0x0002); //REG_0TC_PCFG_usFrTimeType
        S5K8AAYX_write_cmos_sensor(0x0F12,0x0001); //REG_0TC_PCFG_FrRateQualityType
        S5K8AAYX_write_cmos_sensor(0x002A,0x01D6);
        S5K8AAYX_write_cmos_sensor(0x0F12,0x01A0); //REG_0TC_PCFG_usMinFrTimeMsecMult10
        S5K8AAYX_write_cmos_sensor(0x0F12,0x01A0); //REG_0TC_PCFG_usMaxFrTimeMsecMult10 fps
    }
    else if (u2FrameRate == 15)
    {
        S5K8AAYX_write_cmos_sensor(0x002A,0x01D2);
        S5K8AAYX_write_cmos_sensor(0x0F12,0x0002);  //REG_0TC_PCFG_usFrTimeType
        S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  //REG_0TC_PCFG_FrRateQualityType
        S5K8AAYX_write_cmos_sensor(0x002A,0x01D6);
        S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_VID_NIT_FIX_FR_TIME); //REG_0TC_PCFG_usMinFrTimeMsecMult10
        S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_VID_NIT_FIX_FR_TIME); //REG_0TC_PCFG_usMaxFrTimeMsecMult10 fps
    }
    else
    {
        SENSORDB("Wrong Frame Rate \n");
    }

    // active preview configure
    S5K8AAYX_write_cmos_sensor(0x002A, 0x01A8);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000); // REG_TC_GP_ActivePrevConfig
    S5K8AAYX_write_cmos_sensor(0x002A, 0x01AA);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); // REG_TC_GP_PreviewConfigChanged
    S5K8AAYX_write_cmos_sensor(0x002A, 0x019E);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); // REG_TC_GP_EnablePreview
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); // REG_TC_GP_EnablePreviewChanged
    S5K8AAYX_write_cmos_sensor(0x002A, 0x0164);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); // REG_TC_IPRM_InitParamsUpdated
    S5K8AAYX_write_cmos_sensor(0x0028, 0xD000);
    S5K8AAYX_write_cmos_sensor(0x002A, 0x1000);
    S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); // Set host interrupt
    return TRUE;
}


kal_uint16 S5K8AAYXReadShutter()
{
   kal_uint16 temp;
   S5K8AAYX_write_cmos_sensor(0x002c,0x7000);
   S5K8AAYX_write_cmos_sensor(0x002e,0x16E2);

   temp=S5K8AAYX_read_cmos_sensor(0x0f12);

   return temp;
}


kal_uint16 S5K8AAYXReadGain()
{
   kal_uint16 temp;
   S5K8AAYX_write_cmos_sensor(0x002c,0x7000);
   S5K8AAYX_write_cmos_sensor(0x002e,0x20D0);
   temp=S5K8AAYX_read_cmos_sensor(0x0f12);
   return temp;
}


kal_uint16 S5K8AAYXReadAwbRGain()
{
   kal_uint16 temp;
   S5K8AAYX_write_cmos_sensor(0x002c,0x7000);
   S5K8AAYX_write_cmos_sensor(0x002e,0x20D2);
   temp=S5K8AAYX_read_cmos_sensor(0x0f12);
   return temp;
}


kal_uint16 S5K8AAYXReadAwbBGain()
{
   kal_uint16 temp;
   S5K8AAYX_write_cmos_sensor(0x002c,0x7000);
   S5K8AAYX_write_cmos_sensor(0x002e,0x20D2);
   temp=S5K8AAYX_read_cmos_sensor(0x0f12);
   return temp;
}


//#if defined(MT6575)
/*************************************************************************
* FUNCTION
*    S5K8AAYXGetEvAwbRef
*
* DESCRIPTION
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void S5K8AAYXGetEvAwbRef(UINT32 pSensorAEAWBRefStruct/*PSENSOR_AE_AWB_REF_STRUCT Ref*/)
{
    PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
    SENSORDB("S5K8AAYXGetEvAwbRef ref = 0x%x \n", Ref);

    Ref->SensorAERef.AeRefLV05Shutter = 1503;
    Ref->SensorAERef.AeRefLV05Gain = 496 * 2; /* 7.75x, 128 base */
    Ref->SensorAERef.AeRefLV13Shutter = 49;
    Ref->SensorAERef.AeRefLV13Gain = 64 * 2; /* 1x, 128 base */
    Ref->SensorAwbGainRef.AwbRefD65Rgain = 188; /* 1.46875x, 128 base */
    Ref->SensorAwbGainRef.AwbRefD65Bgain = 128; /* 1x, 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFRgain = 160; /* 1.25x, 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFBgain = 164; /* 1.28125x, 128 base */
}
/*************************************************************************
* FUNCTION
*    S5K8AAYXGetCurAeAwbInfo
*
* DESCRIPTION
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void S5K8AAYXGetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct/*PSENSOR_AE_AWB_CUR_STRUCT Info*/)
{
    PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
    SENSORDB("S5K8AAYXGetCurAeAwbInfo Info = 0x%x \n", Info);

    Info->SensorAECur.AeCurShutter = S5K8AAYXReadShutter();
    Info->SensorAECur.AeCurGain = S5K8AAYXReadGain() * 2; /* 128 base */

    Info->SensorAwbGainCur.AwbCurRgain = S5K8AAYXReadAwbRGain()<< 1; /* 128 base */

    Info->SensorAwbGainCur.AwbCurBgain = S5K8AAYXReadAwbBGain()<< 1; /* 128 base */
}
//#endif //6575

void S5K8AAYXGetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
    SENSORDB("S5K8AAYXGetAFMaxNumFocusAreas, *pFeatureReturnPara32 = %d\n",*pFeatureReturnPara32);
}


void S5K8AAYXGetAFMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
    SENSORDB("S5K8AAYXGetAFMaxNumMeteringAreas,*pFeatureReturnPara32 = %d\n",*pFeatureReturnPara32);
}


void S5K8AAYXGetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = S5K8AAYXCurrentStatus.iWB;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}


UINT32 S5K8AAYXFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                              UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    SENSORDB("Enter S5K8AAYXFeatureControl. ID=%d\n", FeatureId);
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT      *pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;
#if WINMO_USE
    PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo=(PMSDK_FEATURE_INFO_STRUCT) pFeaturePara;
#endif

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
             *pFeatureReturnPara16++ = S5K8AAYX_IMAGE_SENSOR_FULL_WIDTH;
             *pFeatureReturnPara16 = S5K8AAYX_IMAGE_SENSOR_FULL_HEIGHT;
             *pFeatureParaLen=4;
             break;
        case SENSOR_FEATURE_GET_PERIOD:
             //*pFeatureReturnPara16++ = S5K8AAYX_PV_PERIOD_PIXEL_NUMS + S5K8AAYX_PV_dummy_pixels;
             //*pFeatureReturnPara16 = S5K8AAYX_PV_PERIOD_LINE_NUMS + S5K8AAYX_PV_dummy_lines;
             //*pFeatureParaLen=4;
             break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
             *pFeatureReturnPara32 = S5K8AAYX_sensor_pclk/10;
             *pFeatureParaLen=4;
             break;
        case SENSOR_FEATURE_SET_ESHUTTER:
             break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
             S5K8AAYX_night_mode((BOOL) *pFeatureData16);
             break;
        case SENSOR_FEATURE_SET_GAIN:
        case SENSOR_FEATURE_SET_FLASHLIGHT:
             break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
             S5K8AAYX_isp_master_clock=*pFeatureData32;
             break;
        case SENSOR_FEATURE_SET_REGISTER:
             S5K8AAYX_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
             break;
        case SENSOR_FEATURE_GET_REGISTER:
             pSensorRegData->RegData = S5K8AAYX_read_cmos_sensor(pSensorRegData->RegAddr);
             break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
             memcpy(pSensorConfigData, &S5K8AAYXSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
             *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
             break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
        case SENSOR_FEATURE_GET_CCT_REGISTER:
        case SENSOR_FEATURE_SET_ENG_REGISTER:
        case SENSOR_FEATURE_GET_ENG_REGISTER:
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
        case SENSOR_FEATURE_GET_GROUP_INFO:
        case SENSOR_FEATURE_GET_ITEM_INFO:
        case SENSOR_FEATURE_SET_ITEM_INFO:
        case SENSOR_FEATURE_GET_ENG_INFO:
             break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
             *pFeatureReturnPara32++ = 0;
             *pFeatureParaLen = 4;
             break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
             // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
             // if EEPROM does not exist in camera module.
             *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
             *pFeatureParaLen=4;
             break;
        case SENSOR_FEATURE_SET_YUV_CMD:
             //S5K8AAYXYUVSensorSetting((MSDK_ISP_FEATURE_ENUM)*pFeatureData16, *(pFeatureData16+1));
             S5K8AAYXYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
             break;
             //break;
        #if WINMO_USE
             case SENSOR_FEATURE_QUERY:
                  S5K8AAYXQuery(pSensorFeatureInfo);
                  *pFeatureParaLen = sizeof(MSDK_FEATURE_INFO_STRUCT);
                  break;
             case SENSOR_FEATURE_SET_YUV_CAPTURE_RAW_SUPPORT:
                  /* update yuv capture raw support flag by *pFeatureData16 */
                  break;
        #endif

        case SENSOR_FEATURE_SET_VIDEO_MODE:
             //SENSORDB("S5K8AAYX SENSOR_FEATURE_SET_VIDEO_MODE\r\n");
             S5K8AAYXYUVSetVideoMode(*pFeatureData16);
             break;
        case SENSOR_FEATURE_GET_EV_AWB_REF:
             S5K8AAYXGetEvAwbRef(pFeatureData32);
             break;
        case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
             S5K8AAYXGetCurAeAwbInfo(pFeatureData32);
             break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
             S5K8AAYX_GetSensorID(pFeatureData32);
             break;

        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
             S5K8AAYXGetAFMaxNumFocusAreas(pFeatureReturnPara32);
             *pFeatureParaLen=4;
             break;
        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
             S5K8AAYXGetAFMaxNumMeteringAreas(pFeatureReturnPara32);
             *pFeatureParaLen=4;
             break;
        case SENSOR_FEATURE_GET_EXIF_INFO:
             SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO\n");
             SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32);
             S5K8AAYXGetExifInfo(*pFeatureData32);
             break;
       case SENSOR_FEATURE_SET_TEST_PATTERN:
        //1 TODO
        SENSORDB("S5K8AA SENSOR_FEATURE_SET_TEST_PATTERN: FAIL: NOT Support\n");
        break;

        default:
             SENSORDB("Enter S5K8AAYXFeatureControl. default. return\n");
             break;
    }
    return ERROR_NONE;
}

SENSOR_FUNCTION_STRUCT SensorFuncS5K8AAYX=
{
    S5K8AAYXOpen,
    S5K8AAYXGetInfo,
    S5K8AAYXGetResolution,
    S5K8AAYXFeatureControl,
    S5K8AAYXControl,
    S5K8AAYXClose
};

UINT32 S5K8AAYX_PVI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    SENSORDB("Enter S5K8AAYX_MIPI_YUV_SensorInit\n");
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncS5K8AAYX;
    return ERROR_NONE;
}   /* SensorInit() */

