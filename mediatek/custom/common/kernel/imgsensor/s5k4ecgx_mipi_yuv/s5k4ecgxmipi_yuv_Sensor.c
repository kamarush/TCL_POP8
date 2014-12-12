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
 * 01 04 2012 hao.wang
 * [ALPS00109603] getsensorid func check in
 * .
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#if !defined(MTK_NATIVE_3D_SUPPORT) //2D
  #define S5K4ECGX_MIPIYUV_MAIN_2_USE_HW_I2C
#else //MTK_NATIVE_3D_SUPPORT
  #define S5K4ECGX_MIPIYUV_MAIN_2_USE_HW_I2C

  #ifdef S5K4ECGX_MIPI_MAIN_2_USE_HW_I2C
    #define S5K4ECGX_MIPI_SUPPORT_N3D
  #endif
#endif

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
//Daniel
//#include <linux/slab.h>
#include <linux/xlog.h>
#include <asm/atomic.h>
//#include <asm/uaccess.h> //copy from user
//#include <linux/miscdevice.h>
//#include <mach/mt6516_pll.h>
#include <asm/io.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"


#include "s5k4ecgxmipi_yuv_Sensor.h"
#include "s5k4ecgxmipi_yuv_Camera_Sensor_para.h"
#include "s5k4ecgxmipi_yuv_CameraCustomized.h"

#define PRE_CLK 80

#define S5K4ECGX_MIPI_DEBUG
#ifdef S5K4ECGX_MIPI_DEBUG
//#define SENSORDB printk
#define SENSORDB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[S5K4ECGXMIPI]", fmt, ##arg)
#else
#define SENSORDB(x,...)
#endif
#define Sleep(ms) mdelay(ms)



#define S5K4ECGX_MIPI_AF_Enable 1


//Sophie Add: Need to check
static DEFINE_SPINLOCK(s5k4ecgx_mipi_drv_lock);
static kal_uint32  S5K4ECGX_MIPI_sensor_pclk = 390 * 1000000;
MSDK_SCENARIO_ID_ENUM S5K4ECGXCurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT S5K4ECGX_PreviewWin[4] = {0};

// global
static MSDK_SENSOR_CONFIG_STRUCT S5K4ECGXSensorConfigData;
//
kal_uint8 S5K4ECGX_MIPI_sensor_write_I2C_address = S5K4ECGX_WRITE_ID;
kal_uint8 S5K4ECGX_MIPI_sensor_read_I2C_address = S5K4ECGX_READ_ID;
kal_uint8 S5K4ECGX_MIPI_sensor_socket = DUAL_CAMERA_NONE_SENSOR;
struct S5K4ECGX_MIPI_sensor_struct S5K4ECGX_Driver;
unsigned int S5K4ECGX_Preview_enabled = 0;
static unsigned int  S5K4ECGX_RV_DefaultMode = 0;


extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iMultiWriteReg(u8 *pData, u16 lens, u16 i2cId);

static kal_uint16 S5K4ECGX_write_cmos_sensor_wID(kal_uint32 addr, kal_uint32 para, kal_uint32 id)
{
   char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
   iWriteRegI2C(puSendCmd , 4,id);
   //SENSORDB("[Write]:addr=0x%x, para=0x%x, ID=0x%x\r\n", addr, para, id);
}

static kal_uint16 S5K4ECGX_read_cmos_sensor_wID(kal_uint32 addr, kal_uint32 id)
{
   kal_uint16 get_byte=0;
   char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
   iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,id);
   return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

static kal_uint16 S5K4ECGX_read_cmos_sensor(kal_uint32 addr)
{
   kal_uint16 get_byte=0;
   char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
   iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,S5K4ECGX_MIPI_sensor_write_I2C_address);
   return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}
static kal_uint16 S5K4ECGX_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
   char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
   iWriteRegI2C(puSendCmd , 4,S5K4ECGX_MIPI_sensor_write_I2C_address);
   //SENSORDB("[Write]:id=0x%x, addr=0x%x, para=0x%x\r\n", S5K4ECGX_MIPI_sensor_write_I2C_address, addr, para);
}



int SEN_RUN_TEST_PATTERN = 0;
UINT32 S5K4ECGX_MIPI_SetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[S5K4ECGX_MIPI_SetTestPatternMode] Fail: Not Support. Test pattern enable:%d\n", bEnable);
    //output color bar
    S5K4ECGX_write_cmos_sensor(0x0028,0xd000);
    S5K4ECGX_write_cmos_sensor(0x002A,0x3100);
    S5K4ECGX_write_cmos_sensor(0x0F12,0x0002);
    S5K4ECGX_write_cmos_sensor(0xFCFC,0xd000);
    S5K4ECGX_write_cmos_sensor(0x002C,0x7000);
    return ERROR_NONE;
}


/***********************************************************
**    AF Control Start
***********************************************************/
//S5K4ECGX_AF_STATE_ENUM   S5K4ECGX_MIPI_AFState = S5K4ECGX_AF_STATE_UNINIT;
//S5K4ECGX_AF_MODE_ENUM   S5K4ECGX_MIPI_AF_Mode = S5K4ECGX_AF_MODE_SINGLE;
//unsigned int            S5K4ECGX_MIPI_ASK_AE_LOCK = 0; //0: false, 1: True
//S5K4ECGX_AE_STATE_ENUM  S5K4ECGX_MIPI_AE_STATE = S5K4ECGX_AE_STATE_UNLOCK;

//S5K4ECGX_MIPI_AF_WIN_T  S5K4ECGX_MIPI_AF_Windows;
//unsigned int            S5K4ECGX_MIPI_AF_onAutoMode = 1; //when the window is only a point, keep sensor on auto focus mode
//unsigned int            S5K4ECGX_MIPI_AE_Windows[4]; //x0,y0,x1,y1



static S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AE_Lock(void)
{
    S5K4ECGX_AE_STATE_ENUM ae_state;

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    ae_state = S5K4ECGX_Driver.aeState;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    if (S5K4ECGX_AE_STATE_UNLOCK == ae_state)
    {
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002E,0x2C5E);
        S5K4ECGX_write_cmos_sensor(0x0F12,0x0000);
    }

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.aeState = S5K4ECGX_AE_STATE_LOCK;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);
    return S5K4ECGX_AAA_AF_STATUS_OK;
}


static S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AE_UnLock(void)
{
    S5K4ECGX_AE_STATE_ENUM ae_state;
#if 0
    spin_lock(&s5k4ecgx_mipi_drv_lock);
    ae_state = S5K4ECGX_Driver.aeState;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    if (S5K4ECGX_AE_STATE_LOCK == ae_state)
    {
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002E,0x2C5E);
        S5K4ECGX_write_cmos_sensor(0x0F12,0x0001);
    }
#endif

	S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
	S5K4ECGX_write_cmos_sensor(0x002E,0x2C5E);
	S5K4ECGX_write_cmos_sensor(0x0F12,0x0001);

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.aeState = S5K4ECGX_AE_STATE_UNLOCK;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);
    return S5K4ECGX_AAA_AF_STATUS_OK;
}



static void S5K4ECGX_MIPI_AE_Enable(kal_bool AE_enable)
{
    kal_uint16 Status_3A=0;

    if(S5K4ECGX_Driver.aeEnable == AE_enable)
        return;

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.aeEnable = AE_enable;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);;

    while(Status_3A==0)
    {
         S5K4ECGX_write_cmos_sensor(0xFCFC,0xd000);
         S5K4ECGX_write_cmos_sensor(0x002C,0x7000);
         S5K4ECGX_write_cmos_sensor(0x002E,0x04E6); //REG_TC_DBG_AutoAlgEnBits
         Status_3A=S5K4ECGX_read_cmos_sensor(0x0F12);
         Sleep(2);
    }

    if(AE_enable)
    {
        //Enable Auto Exposure
        Status_3A = (Status_3A | 0x0002);
        S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
        S5K4ECGX_write_cmos_sensor(0x002a, 0x04E6);
        S5K4ECGX_write_cmos_sensor(0x0f12, Status_3A);
    }
    else
    {
        //Disable Auto Exposure
        Status_3A = (Status_3A & 0xFFFD);
        S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
        S5K4ECGX_write_cmos_sensor(0x002a, 0x04E6);
        S5K4ECGX_write_cmos_sensor(0x0f12, Status_3A);
    }
    return;
}

#if 0
static void S5K4ECGX_MIPI_AE_set_mode(kal_bool AE_enable)
{
    if(AE_enable)
    {
    //UnLock AE
    S5K4ECGX_AF_STATE_ENUM af_state;
    spin_lock(&s5k4ecgx_mipi_drv_lock);
    //af_state = S5K4ECGX_Driver.afState;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    if ((S5K4ECGX_AF_STATE_1ST_SEARCHING != af_state) &&
      (S5K4ECGX_AF_STATE_2ND_SEARCHING != af_state))
    {
      S5K4ECGX_MIPI_AE_UnLock();
    }
    }
    else
    {
    //Lock AE
    S5K4ECGX_MIPI_AE_Lock();
    }
    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.userAskAeLock = AE_enable ? 0:1;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);
}
#endif

void S5K4ECGX_MIPI_AE_Rollback_Weight_Table(void)
{
    //==================================================================================
    // 13.AE Weight (Normal)
    //==================================================================================
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1492);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);   //ae_WeightTbl_16[0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[1]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[2]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //ae_WeightTbl_16[3]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[4]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0201);   //ae_WeightTbl_16[5]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0102);   //ae_WeightTbl_16[6]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[7]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[8]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0202);   //ae_WeightTbl_16[9]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0202);   //ae_WeightTbl_16[10]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[11]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0201);   //ae_WeightTbl_16[12]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0302);   //ae_WeightTbl_16[13]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0203);   //ae_WeightTbl_16[14]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0102);   //ae_WeightTbl_16[15]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0201);   //ae_WeightTbl_16[16]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0302);   //ae_WeightTbl_16[17]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0203);   //ae_WeightTbl_16[18]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0102);   //ae_WeightTbl_16[19]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0201);   //ae_WeightTbl_16[20]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0202);   //ae_WeightTbl_16[21]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0202);   //ae_WeightTbl_16[22]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0102);   //ae_WeightTbl_16[23]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[24]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0202);   //ae_WeightTbl_16[25]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0202);   //ae_WeightTbl_16[26]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[27]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[28]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[29]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[30]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101);   //ae_WeightTbl_16[31]
    return;
}


static void
S5K4ECGX_MIPI_AE_Set_Window2HW(unsigned int prevW, unsigned int prevH)
{
    unsigned int x0, y0, x1, y1;
    unsigned char *tablePtr = 0;
    unsigned int  stepX, stepY;

    x0 = S5K4ECGX_Driver.aeWindows[0];
    y0 = S5K4ECGX_Driver.aeWindows[1];
    x1 = S5K4ECGX_Driver.aeWindows[2];
    y1 = S5K4ECGX_Driver.aeWindows[3];

    x0 = (x0 + x1) / 2;
    y0 = (y0 + y1) / 2;

    stepX = prevW / 8;
    stepY = prevH / 8;

    x0 /= stepX;
    y0 /= stepY;


    {
       unsigned int i;
       S5K4ECGX_write_cmos_sensor(0x002A ,0x2E08);
       for (i = 0; i < 32; i++)
       {
           S5K4ECGX_write_cmos_sensor(0x0F12, 0x0);
       }

       S5K4ECGX_write_cmos_sensor(0x002A ,0x2E08 + (y0 * 8) + (x0/2));
       S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0505);
    }
}


static S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AE_Set_Window(
    unsigned int zone_addr,
    unsigned int prevW,
    unsigned int prevH)
{
    unsigned int x0, y0, x1, y1, width, height;
    unsigned int* ptr = (unsigned int*)zone_addr;
    unsigned int srcW_maxW;// = S5K4ECGX_MIPI_AF_CALLER_WINDOW_WIDTH; //source window's max width
    unsigned int srcW_maxH;// = S5K4ECGX_MIPI_AF_CALLER_WINDOW_HEIGHT;  //source window's max height

    x0 = *ptr       ;
    y0 = *(ptr + 1) ;
    x1 = *(ptr + 2) ;
    y1 = *(ptr + 3) ;
    width = *(ptr + 4);
    height = *(ptr + 5);
    srcW_maxW = width;
    srcW_maxH = height;

    //SENSORDB("S5K4ECGX ~~~~S5K4ECGX_MIPI_AE_Set_Window AP's setting: (%d,%d)~(%d,%d).size:(%d,%d)\n",x0, y0, x1, y1, width, height);

    if (x0 >= srcW_maxW)
    {
        x0 = srcW_maxW - 1;
    }

    if (x1 >= srcW_maxW)
    {
        x1 = srcW_maxW - 1;
    }

    if (y0 >= srcW_maxH)
    {
        y0 = srcW_maxH - 1;
    }

    if (y1 >= srcW_maxH)
    {
        y1 = srcW_maxH - 1;
    }

    x0 = x0 * (prevW / srcW_maxW);
    y0 = y0 * (prevH / srcW_maxH);
    x1 = x1 * (prevW / srcW_maxW);
    y1 = y1 * (prevH / srcW_maxH);

    S5K4ECGX_Driver.aeWindows[0] = x0;
    S5K4ECGX_Driver.aeWindows[1] = y0;
    S5K4ECGX_Driver.aeWindows[2] = x1;
    S5K4ECGX_Driver.aeWindows[3] = y1;

    //S5K4ECGX_MIPI_AE_Set_Window2HW(prevW, prevH);
    //SENSORDB("S5K4ECGX ~~~~S5K4ECGX_MIPI_AE_Set_Window: (%d,%d)~(%d,%d)\n",x0, y0, x1, y1);

  return S5K4ECGX_AAA_AF_STATUS_OK;
}



static void
S5K4ECGX_MIPI_AF_Get_Max_Num_Focus_Areas(unsigned int *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 1;
    //SENSORDB("S5K4ECGX *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
}

static void
S5K4ECGX_MIPI_AE_Get_Max_Num_Metering_Areas(unsigned int *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 1;
    //SENSORDB("S5K4ECGX_MIPI_AE_Get_Max_Num_Metering_Areas *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
}

static void
S5K4ECGX_MIPI_AF_Get_Inf(unsigned int *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
}

static void
S5K4ECGX_MIPI_AF_Get_Macro(unsigned int *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 255;
}



static void 
S5K4ECGX_MIPI_AF_rollbackWinSet(void)
{
	S5K4ECGX_MIPI_AF_WIN_T	*AfWindows = &S5K4ECGX_Driver.orignalAfWindows;

    S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A,0x029C);
    S5K4ECGX_write_cmos_sensor(0x0F12,AfWindows->inWx);
    S5K4ECGX_write_cmos_sensor(0x002A,0x029E);
    S5K4ECGX_write_cmos_sensor(0x0F12,AfWindows->inWy);

    S5K4ECGX_write_cmos_sensor(0x002A,0x0294);
    S5K4ECGX_write_cmos_sensor(0x0F12,AfWindows->outWx);
    S5K4ECGX_write_cmos_sensor(0x002A,0x0296);
    S5K4ECGX_write_cmos_sensor(0x0F12,AfWindows->outWy);

    //Update AF Window
    S5K4ECGX_write_cmos_sensor(0x002A,0x02A4);
    S5K4ECGX_write_cmos_sensor(0x0F12,0x0001);
}



S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AF_Init(void)
{
    unsigned int backupAFWindDone = 1;

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    // have done in init_function
    if (S5K4ECGX_AF_STATE_UNINIT == S5K4ECGX_Driver.afState)
    {
        S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_IDLE;
        backupAFWindDone = 0;
    }
    S5K4ECGX_Driver.afMode = S5K4ECGX_AF_MODE_RSVD;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    if (!backupAFWindDone)
    {
        S5K4ECGX_MIPI_AF_WIN_T  *AfWindows = &S5K4ECGX_Driver.orignalAfWindows;
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A,0x029C);
        AfWindows->inWx = S5K4ECGX_read_cmos_sensor(0x0F12);
        AfWindows->inWy = S5K4ECGX_read_cmos_sensor(0x0F12);
    		   
        S5K4ECGX_write_cmos_sensor(0x002A,0x0294);
        AfWindows->outWx = S5K4ECGX_read_cmos_sensor(0x0F12);
        AfWindows->outWy = S5K4ECGX_read_cmos_sensor(0x0F12);
    }

    return S5K4ECGX_AAA_AF_STATUS_OK;
}



static S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AF_Set_Window(
    unsigned int zone_addr,
    unsigned int prevW,
    unsigned int prevH)
{
    unsigned int x0, y0, x1, y1, FD_XS, FD_YS;
    unsigned int* ptr = (unsigned int*)zone_addr;
    unsigned int srcW_maxW = S5K4ECGX_MIPI_AF_CALLER_WINDOW_WIDTH;
    unsigned int srcW_maxH = S5K4ECGX_MIPI_AF_CALLER_WINDOW_HEIGHT;
    unsigned int af_win_idx = 1;

    x0 = *ptr       ;
    y0 = *(ptr + 1) ;
    x1 = *(ptr + 2) ;
    y1 = *(ptr + 3) ;
    FD_XS = *(ptr + 4);
    FD_YS = *(ptr + 5);
    SENSORDB("S5K4ECGX ~~~~S5K4ECGX_MIPI_AF_Set_Window AP's setting: (%d,%d)~(%d,%d)\n",x0, y0, x1, y1);

    //S5K4ECGX_MIPI_AE_Set_Window(zone_addr, prevW, prevH);

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.afStateOnOriginalSet = 0;
    if ((x0 == x1) && (y0 == y1))
    {
        S5K4ECGX_Driver.afStateOnOriginalSet = 1;
        spin_unlock(&s5k4ecgx_mipi_drv_lock);
        SENSORDB("S5K4ECGX ~~~~Keep on AutoFocus Mode.\n");
        return S5K4ECGX_AAA_AF_STATUS_OK;;
    }
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    if (x0 >= srcW_maxW)
    {
        x0 = srcW_maxW - 1;
    }

    if (x1 >= srcW_maxW)
    {
        x1 = srcW_maxW - 1;
    }

    if (y0 >= srcW_maxH)
    {
        y0 = srcW_maxH - 1;
    }

    if (y1 >= srcW_maxH)
    {
        y1 = srcW_maxH - 1;
    }

    x0 = (x0 + x1) / 2;
    y0 = (y0 + y1) / 2;

    //Map 320x240 coordinate to preview size window
    x0 = x0 * (prevW / srcW_maxW);
    y0 = y0 * (prevH / srcW_maxH);

    {
        unsigned int inWw0, inWh0, outWw0, outWh0;
        unsigned int inWx1, inWy1, inWw1, inWh1;     // x, y, width, height
        unsigned int outWx1, outWy1, outWw1, outWh1; // x, y, width, height

        //Calculate Inner & Outer Window Size
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002E,0x02A0);
        inWw0 = S5K4ECGX_read_cmos_sensor(0x0F12);
        inWw1 = inWw0 * prevW / 1024;

        S5K4ECGX_write_cmos_sensor(0x002E,0x02A2);
        inWh0 = S5K4ECGX_read_cmos_sensor(0x0F12);
        inWh1 = inWh0 * prevH / 1024;

        S5K4ECGX_write_cmos_sensor(0x002E,0x0298);
        outWw0 = S5K4ECGX_read_cmos_sensor(0x0F12);
        outWw1 = outWw0 * prevW / 1024;

        S5K4ECGX_write_cmos_sensor(0x002E,0x029A);
        outWh0 = S5K4ECGX_read_cmos_sensor(0x0F12);
        outWh1 = outWh0 * prevH / 1024;

        //Set X axis
        if (x0 <= inWw1/2 )
        {
            inWx1 = 0;
            outWx1 = 0;
        }
        else if (x0 <= (outWw1 / 2))
        {
            inWx1 = x0 - (inWw1 / 2);
            outWx1 = 0;
        }
        else if (x0 >= ((prevW-1) - (inWw1 / 2)))
        {
            inWx1  = (prevW-1) - inWw1;
            outWx1 = (prevW-1) - outWw1;
        }
        else if (x0 >= ((prevW-1) - (outWw1 / 2)))
        {
            inWx1  = x0 - (inWw1/2);
            outWx1 = (prevW-1) - outWw1;
        }
        else
        {
            inWx1  = x0 - (inWw1/2);
            outWx1 = x0 - (outWw1/2);
        }


        //Set Y axis
        if (y0 <= (inWh1/2))
        {
            inWy1 = 0;
            outWy1 = 0;
        }
        else if (y0 <= (outWh1/2))
        {
            inWy1 = y0 - (inWh1/2);
            outWy1 = 0;
        }
        else if (y0 >= ((prevH-1) - (inWh1/2)))
        {
            inWy1  = (prevH-1) - inWh1;
            outWy1 = (prevH-1) - outWh1;
        }
        else if (y0 >= ((prevH-1) - (outWh1/2)))
        {
            inWy1  = y0 - (inWh1/2);
            outWy1 = (prevH-1) - outWh1;
        }
        else
        {
            inWy1  = y0 - (inWh1/2);
            outWy1 = y0 - (outWh1/2);
        }

        //restore
        spin_lock(&s5k4ecgx_mipi_drv_lock);
        S5K4ECGX_Driver.afWindows.inWx = inWx1 * 1024 / (prevW);
        S5K4ECGX_Driver.afWindows.inWy = inWy1 * 1024 / (prevH);
        S5K4ECGX_Driver.afWindows.inWw = inWw1;
        S5K4ECGX_Driver.afWindows.inWh = inWh1;
        S5K4ECGX_Driver.afWindows.outWx = outWx1 * 1024 / (prevW);
        S5K4ECGX_Driver.afWindows.outWy = outWy1 * 1024 / (prevH);
        S5K4ECGX_Driver.afWindows.outWw = outWw1;
        S5K4ECGX_Driver.afWindows.outWh = outWh1;
        spin_unlock(&s5k4ecgx_mipi_drv_lock);
    }

    SENSORDB("S5K4ECGX ~~~~S5K4ECGX_MIPI_AF_Set_Window: inXY:(%d,%d), inWH: (%d,%d), outXY:(%d,%d), outWH: (%d,%d)\n",
    S5K4ECGX_Driver.afWindows.inWx, S5K4ECGX_Driver.afWindows.inWy,
    S5K4ECGX_Driver.afWindows.inWw, S5K4ECGX_Driver.afWindows.inWh,
    S5K4ECGX_Driver.afWindows.outWx, S5K4ECGX_Driver.afWindows.outWy,
    S5K4ECGX_Driver.afWindows.outWw, S5K4ECGX_Driver.afWindows.outWh);

    return S5K4ECGX_AAA_AF_STATUS_OK;
}



S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AF_Set_Window2HW(void)
{
    unsigned int inWx1, inWy1, inWw1, inWh1; // x, y, width, height
    unsigned int outWx1, outWy1, outWw1, outWh1; // x, y, width, height

#if defined(S5K4ECGX_MIPI_AF_Enable)

    if (S5K4ECGX_Driver.afStateOnOriginalSet)
        return S5K4ECGX_AAA_AF_STATUS_OK;

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    inWx1 = S5K4ECGX_Driver.afWindows.inWx;
    inWy1 = S5K4ECGX_Driver.afWindows.inWy;
    inWw1 = S5K4ECGX_Driver.afWindows.inWw;
    inWh1 = S5K4ECGX_Driver.afWindows.inWh;
    outWx1 = S5K4ECGX_Driver.afWindows.outWx;
    outWy1 = S5K4ECGX_Driver.afWindows.outWy;
    outWw1 = S5K4ECGX_Driver.afWindows.outWw;
    outWh1 = S5K4ECGX_Driver.afWindows.outWh;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    //S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
    S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A,0x029C);
    S5K4ECGX_write_cmos_sensor(0x0F12,inWx1);
    S5K4ECGX_write_cmos_sensor(0x002A,0x029E);
    S5K4ECGX_write_cmos_sensor(0x0F12,inWy1);

    S5K4ECGX_write_cmos_sensor(0x002A,0x0294);
    S5K4ECGX_write_cmos_sensor(0x0F12,outWx1);
    S5K4ECGX_write_cmos_sensor(0x002A,0x0296);
    S5K4ECGX_write_cmos_sensor(0x0F12,outWy1);

    //For the size part, FW will update automatically


    //Update AF Window
    S5K4ECGX_write_cmos_sensor(0x002A,0x02A4);
    S5K4ECGX_write_cmos_sensor(0x0F12,0x0001);

#endif //S5K4ECGX_MIPI_AF_EnableTouch

    return S5K4ECGX_AAA_AF_STATUS_OK;
}



S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AF_Stop(void)
{
#if defined(S5K4ECGX_MIPI_AF_Enable)

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_IDLE;

    ////Lock AE
    if ((!S5K4ECGX_Driver.userAskAeLock) &&
        (S5K4ECGX_AF_MODE_SINGLE == S5K4ECGX_Driver.afMode))
    {
        spin_unlock(&s5k4ecgx_mipi_drv_lock);
        S5K4ECGX_MIPI_AE_UnLock();
    }
    else
    {
        spin_unlock(&s5k4ecgx_mipi_drv_lock);
    }
    SENSORDB("S5K4ECGX ~~~~S5K4ECGX_MIPI_AF_Stop---\n");
#endif

    return S5K4ECGX_AAA_AF_STATUS_OK;
}



static void
S5K4ECGX_MIPI_AF_ShowLensPosition(void)
{
    unsigned int lens_cur_pos, lens_best_pos, lens_prev_best_pos;
    unsigned int lens_tlb_idx;
#if defined(S5K4ECGX_MIPI_AF_Enable)

    //S5K4ECGX_write_cmos_sensor(0x002C,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002E,0x15E8);
    lens_tlb_idx = S5K4ECGX_read_cmos_sensor(0x0F12);

    S5K4ECGX_write_cmos_sensor(0x002E,0x2EE8);
    lens_cur_pos = S5K4ECGX_read_cmos_sensor(0x0F12);

    S5K4ECGX_write_cmos_sensor(0x002E,0x2EEA);
    lens_best_pos = S5K4ECGX_read_cmos_sensor(0x0F12);

    S5K4ECGX_write_cmos_sensor(0x002E,0x2EEC);
    lens_prev_best_pos = S5K4ECGX_read_cmos_sensor(0x0F12);

    SENSORDB("S5K4ECGX ~~~~AF_GetLensPosition: lens_tlb_idx=%d; cur_pos=%x, best_pos=%x, prev_best_pos=%x\n", lens_tlb_idx, lens_cur_pos, lens_best_pos, lens_prev_best_pos);
#endif

    return;
}



static S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AF_CancelFocus(void)
{
    signed int loop_iter = 200;
    unsigned int af_status;

    //S5K4ECGX_write_cmos_sensor(0x002A,0x028C);
    //S5K4ECGX_write_cmos_sensor(0x0F12,0x0002); //aborting
    mdelay(30);


#if 0
    while (loop_iter--)
    {
        S5K4ECGX_write_cmos_sensor(0x002A,0x0290);
        af_status = S5K4ECGX_read_cmos_sensor(0x0F12);
        if (0x0 == af_status)
        {
            break; //finish: No error
        }
        else if (0x1 == af_status)
        {
            loop_iter = -1;  //error happen
            break;
        }
        mdelay(1);
       //SENSORDB("[4EC] S5K4ECGX_MIPI_AF_ContinuousShot_Start 1st Search polling\n");
    }
#endif

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.afMode = S5K4ECGX_AF_MODE_RSVD;
    S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_DONE;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);
    SENSORDB("\nS5K4ECGX ~~~~AF_CancelFocus\n");

    return S5K4ECGX_AAA_AF_STATUS_OK;
}



S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AF_SingleShot_Start(void)
{
    unsigned int af_status;
    signed int loop_iter = 200;

#if defined(S5K4ECGX_MIPI_AF_Enable)

    SENSORDB("\nS5K4ECGX ~~~~S5K4ECGX_MIPI_AF_SingleShot_Start+\n");

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    if ((S5K4ECGX_AF_MODE_CONTINUOUS == S5K4ECGX_Driver.afMode))
    {
        SENSORDB("\nS5K4ECGX ~~~~AF_SingleShot_Start:Cancel CAF\n");
        S5K4ECGX_MIPI_AF_CancelFocus();
    }
    S5K4ECGX_Driver.afMode = S5K4ECGX_AF_MODE_SINGLE;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    //S5K4ECGX_MIPI_AF_ShowLensPosition();

    S5K4ECGX_MIPI_AF_Set_Window2HW(); //with delay in this function
    if (!S5K4ECGX_Driver.afStateOnOriginalSet)
        mdelay(40); // delay 1 frame

    S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_1ST_SEARCHING;

    ////Lock AE
    S5K4ECGX_MIPI_AE_Lock();

    ////Single AF start
    //S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A,0x028C);
    S5K4ECGX_write_cmos_sensor(0x0F12,0x0005);
    mdelay(80); //delay 2 frames


    //wait until 1st search done
    while (loop_iter--)
    {
         S5K4ECGX_write_cmos_sensor(0x002E,0x2EEE);
         af_status = S5K4ECGX_read_cmos_sensor(0x0F12);
         if (0x2 == af_status)
         {
            break; //finish
         }
         //else if (0x2 < af_status)
         //{
         //   loop_iter = -1; //error happen
         //   break;
         //}
         mdelay(1);
         //SENSORDB("[Enter]S5K4ECGX_MIPI_AF_SingleShot 1st_search_sta=%d\n", af_status);
    }
    //S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_1ST_SEARCH_DONE;

    if (!loop_iter)
    {
        //S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_ERROR;
        SENSORDB("\nS5K4ECGX ~~~~SingleFocus Fail: 1st_search_sta=%d\n", af_status);
        return S5K4ECGX_AAA_AF_STATUS_1ST_SEARCH_FAIL;
    }
    SENSORDB("\nS5K4ECGX ~~~~SingleShot_Start 1st Search Done\n");


    S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_2ND_SEARCHING;
#if 0

    //wait until 2nd search done
    loop_iter = 100;
    while (loop_iter--)
    {
        S5K4ECGX_write_cmos_sensor(0x002A,0x2207);
        af_status = S5K4ECGX_read_cmos_sensor(0x0F12) >> 8;
        if (0x0 == af_status)
        {
            break;
        }
        mdelay(1);
        //SENSORDB("[Enter]S5K4ECGX_MIPI_AF_SingleShot 2nd_search_sta=%d\n", af_status);
    }

    if (!loop_iter)
    {
        S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_ERROR;
        SENSORDB("S5K4ECGX ~~~~AF_SingleShot Fail: 2nd_search_sta=%d\n", af_status);
        return S5K4ECGX_AAA_AF_STATUS_2ND_SEARCH_FAIL;
    }
#endif

#endif

    //S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_DONE;
    //SENSORDB("S5K4ECGX ~~~~AF_SingleShot_Start 2nd Search Done\n");
    //move len to 2nd search
    SENSORDB("\nS5K4ECGX ~~~~S5K4ECGX_MIPI_AF_SingleShot_Start-\n");
    return S5K4ECGX_AAA_AF_STATUS_OK;
}


S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AF_ContinuousShot_Start(void)
{
    // 30fps

    unsigned int af_status;
    signed int loop_iter = 200;
    SENSORDB("\nS5K4ECGX ~~~~AF_ContinuousShot_Start+\n");

#if defined(S5K4ECGX_MIPI_AF_Enable)

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    if (S5K4ECGX_AF_MODE_CONTINUOUS == S5K4ECGX_Driver.afMode)
    {
        spin_unlock(&s5k4ecgx_mipi_drv_lock);
        SENSORDB("\nS5K4ECGX ~~~~Already at Continuous Mode.\n");
        return S5K4ECGX_AAA_AF_STATUS_OK;
    }
    /*if ((S5K4ECGX_AF_MODE_SINGLE == S5K4ECGX_Driver.afMode) &&
          (S5K4ECGX_AF_STATE_2ND_SEARCHING == S5K4ECGX_Driver.afState))
    {
        //the previous single AF is not completed.
        SENSORDB("S5K4ECGX ~~~~Error: the previous single AF is not completedxxxxxxx\n");
    }*/
    S5K4ECGX_Driver.afMode = S5K4ECGX_AF_MODE_CONTINUOUS;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    //Rollback AE/AF setting for CAF mode starting
	S5K4ECGX_MIPI_AE_Rollback_Weight_Table();
	S5K4ECGX_MIPI_AF_rollbackWinSet();

    //S5K4ECGX_MIPI_AF_Set_Window2HW(); //with delay in this function
    //S5K4ECGX_write_cmos_sensor(0x002A,0x028E);
    //S5K4ECGX_write_cmos_sensor(0x0F12,0x0010);
    //SENSORDB("S5K4ECGX ~~~~AF_ContinuousShot_Start: 1st Sleep\n");
    //if (!S5K4ECGX_Driver.afStateOnOriginalSet)
    mdelay(35); //1 frames

    S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_1ST_SEARCHING;
    ////Continus AF start
    //S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A,0x028C);
    S5K4ECGX_write_cmos_sensor(0x0F12,0x0006); //contitous mode
    //SENSORDB("S5K4ECGX ~~~~AF_ContinuousShot_Start: 2nd Sleep\n");


#if 0
    //mdelay(30); //1frames
#else
    //wait until 1st search done
    mdelay(30); //2frames
    /*while (loop_iter--)
    {
        S5K4ECGX_write_cmos_sensor(0x002E,0x2EEE);
        af_status = S5K4ECGX_read_cmos_sensor(0x0F12);
        if (0x2 == af_status)
        {
            break; //finish
        }
        else if (0x2 < af_status)
        {
            loop_iter = -1;  //error happen
            break;
        }
        mdelay(1);
        //SENSORDB("[4EC] S5K4ECGX_MIPI_AF_ContinuousShot_Start 1st Search polling\n");
    }

    if (!loop_iter)
    {
        //S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_ERROR;
        SENSORDB("S5K4ECGX ~~~~AF_ContinuousShot 1st_search_sta=%d\n", af_status);
        return S5K4ECGX_AAA_AF_STATUS_1ST_SEARCH_FAIL;
    }
    SENSORDB("S5K4ECGX ~~~~AF_ContinuousShot_Start 1st Search Done\n");
    mdelay(40); //1 frames // move to getinfo?
    */

    S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_DONE;

#endif

#endif
    //move len to 2nd search
    SENSORDB("\nS5K4ECGX ~~~~AF_ContinuousShot_Start-\n");
    return S5K4ECGX_AAA_AF_STATUS_OK;
}



static S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AF_Move_to(unsigned int a_u2MovePosition)//??how many bits for ov3640??
{
    return S5K4ECGX_AAA_AF_STATUS_OK;
}



static S5K4ECGX_AAA_STATUS_ENUM
S5K4ECGX_MIPI_AF_Get_Status(unsigned int *pFeatureReturnPara32)
{
    S5K4ECGX_AF_STATE_ENUM af_state;
    S5K4ECGX_AF_MODE_ENUM af_Mode;
    unsigned int af_1stSearch_status, af_2ndSearch_status;

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    //af_state = S5K4ECGX_Driver.afState;
    af_Mode  = S5K4ECGX_Driver.afMode;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);
    //SENSORDB("S5K4ECGX ~~~~S5K4ECGX_MIPI_AF_Get_Status+\r\n");


    //SENSORDB("S5K4ECGX ~~~~1ST_Search Status: %d\n", af_1stSearch_status);
    //SENSORDB("S5K4ECGX ~~~~AF_Get_Status: afMode: %d\n", af_Mode);
    if (S5K4ECGX_AF_MODE_CONTINUOUS == af_Mode)
    {
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002E,0x2EEE);
        af_1stSearch_status = S5K4ECGX_read_cmos_sensor(0x0F12) & 0xFFFF;

        switch (af_1stSearch_status)
        {
            case 0:
                *pFeatureReturnPara32 = SENSOR_AF_IDLE;
                //S5K4ECGX_MIPI_AF_ContinuousShot_Start(); ////something wrong here....
                SENSORDB("\nS5K4ECGX ~~~~CAF: SENSOR_AF_IDLE\n");
                break;
            case 1:
                *pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
                SENSORDB("\nS5K4ECGX ~~~~CAF: SENSOR_AF_FOCUSING\n");
                break;
            case 2:
               *pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
                //mdelay(10); //let lans move to correct position
                SENSORDB("\nS5K4ECGX ~~~~CAF: SENSOR_AF_FOCUSED\n");
                break;
            case 3:
                *pFeatureReturnPara32 = SENSOR_AF_FOCUSED; //Low Confidence
                SENSORDB("\nS5K4ECGX ~~~~CAF: SENSOR_AF_LOW_CONFIDENCE\n");
                break;
            case 4:
                *pFeatureReturnPara32 = SENSOR_AF_FOCUSED; //Cancelled
                SENSORDB("\nS5K4ECGX ~~~~CAF: SENSOR_AF_CANCELLED\n");
                break;
            case 6:
            case 7:
                *pFeatureReturnPara32 = SENSOR_AF_FOCUSING; //Restart
                SENSORDB("\nS5K4ECGX ~~~~CAF: SENSOR_AF_RESTART\n");
                break;
            case 8:
                *pFeatureReturnPara32 = SENSOR_AF_FOCUSING; //Reset
                SENSORDB("\nS5K4ECGX ~~~~CAF: SENSOR_AF_RESET---\n");
                break;
            default:
                *pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
                SENSORDB("\nS5K4ECGX ~~~~CAF: SENSOR_AF_ERROR:%x!!!!\n", af_1stSearch_status);
                break;
        }

        if (*pFeatureReturnPara32 == SENSOR_AF_FOCUSED)
        {
             //SENSORDB("S5K4ECGX ~~~~CAF: Back to S5K4ECGX_AF_STATE_DONE!\n");
             S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_DONE;
        }
        return S5K4ECGX_AAA_AF_STATUS_OK;
    }


    if (S5K4ECGX_AF_MODE_SINGLE == af_Mode)
    {
#if 0
        if (S5K4ECGX_AF_STATE_1ST_SEARCHING == S5K4ECGX_Driver.afState)
        {
            switch (af_1stSearch_status)
            {
                case 0:
                    *pFeatureReturnPara32 = SENSOR_AF_IDLE;
                    SENSORDB("S5K4ECGX ~~~~SAF: SENSOR_AF_IDLE\n");
                    break;
                case 1:
                case 6:
                case 7:
                    *pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
                    SENSORDB("S5K4ECGX ~~~~SAF: 1st SENSOR_AF_FOCUSING\n");
                    break;
                case 2:
                case 3: // the 1st search is done
                    *pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
                    S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_2ND_SEARCHING;
                    SENSORDB("S5K4ECGX ~~~~SAF: 1st SENSOR_AF_FOCUSED, run 2nd focus\n");
                    break;
                case 4: // canceld
                    *pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
                    S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_DONE;
                    SENSORDB("S5K4ECGX ~~~~SAF: SENSOR_AF_CANCELLED\n");
                default:
                    *pFeatureReturnPara32 = SENSOR_AF_ERROR;
                    SENSORDB("S5K4ECGX ~~~~SAF: SENSOR_AF_ERROR\n");
                    break;
            }
            return S5K4ECGX_AAA_AF_STATUS_OK;
        }
#endif

        S5K4ECGX_write_cmos_sensor(0x002E,0x2207);
        af_2ndSearch_status = S5K4ECGX_read_cmos_sensor(0x0F12) >> 8;

        if (S5K4ECGX_Driver.afState == S5K4ECGX_AF_STATE_2ND_SEARCHING)
        {
            if (af_2ndSearch_status == 0)
            {
                *pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
                S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_DONE;
                SENSORDB("\nS5K4ECGX ~~~~SAF: 2ND_SEARCHING SENSOR_AF_FOCUSED Done\n");
                mdelay(10); //let lans move to correct position
            }
            else
            {
                *pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
                SENSORDB("\nS5K4ECGX ~~~~SAF: 2ND_SEARCHING SENSOR_AF_FOCUSING\n");
            }
        }
        else if (S5K4ECGX_Driver.afState == S5K4ECGX_AF_STATE_DONE)
        {
            *pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
            SENSORDB("\nS5K4ECGX ~~~~SAF: 2ND_SEARCHING SENSOR_AF_FOCUSED Done\n");
        }
        else
        {
            SENSORDB("\nS5K4ECGX ~~~~SAF: State Error: %d\n", S5K4ECGX_Driver.afState);
        }
    return S5K4ECGX_AAA_AF_STATUS_OK;
  }


  //SENSORDB("S5K4ECGX ~~~~SAF: Shall not goto here...State: %d\n", S5K4ECGX_Driver.afState);
  //SENSORDB("S5K4ECGX ~~~~AF_Get_Status: IDLE\n");
  *pFeatureReturnPara32 = SENSOR_AF_IDLE;
   return S5K4ECGX_AAA_AF_STATUS_OK;
}



static void S5K4ECGX_MIPI_set_AF_infinite(kal_bool is_AF_OFF)
{
    if(is_AF_OFF){
      S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
      S5K4ECGX_write_cmos_sensor(0x002a, 0x028E);
      S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);
      Sleep(100);
      S5K4ECGX_write_cmos_sensor(0x002a, 0x028C);
      S5K4ECGX_write_cmos_sensor(0x0F12, 0x0004);
    } else {
      S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
      S5K4ECGX_write_cmos_sensor(0x002a, 0x028C);
      S5K4ECGX_write_cmos_sensor(0x0F12, 0x0003);
    }
}


static void
S5K4ECGX_MIPI_AF_UnitTest(void)
{
#if 0
    unsigned int af_touch_pos[6] = {200, 150, 104, 104, 320, 240};
    unsigned int prevW = 1280;
    unsigned int prevH = 960;
    S5K4ECGX_AAA_STATUS_ENUM status;

    SENSORDB("S5K4ECGX ~~~~AF_UnitTest: Before AF================================\n");

    S5K4ECGX_MIPI_AF_Init();
    S5K4ECGX_MIPI_AF_ShowLensPosition();

    S5K4ECGX_MIPI_AF_Set_Window(af_touch_pos, prevW, prevH);

    //mdelay(70); //delay x0ms to wait preview stable

    status = S5K4ECGX_MIPI_AF_SingleShot_Start();
    if (S5K4ECGX_AAA_AF_STATUS_OK != status)
    {
        SENSORDB("S5K4ECGX ~~~~AF_UnitTest: SingleShot Fail ooooooooooooooooo\n");
        S5K4ECGX_MIPI_AF_Stop();
        return;
    }
    S5K4ECGX_MIPI_AF_ShowLensPosition();
    S5K4ECGX_MIPI_AF_Stop();

    SENSORDB("S5K4ECGX ~~~~AF_UnitTest: After AF================================\n");
#endif
}


/***********************************************************
**    AF Control End
***********************************************************/

unsigned int MIPI_CLK0_SYS_OP_RATE = 0x4074;
unsigned int MIPI_CLK0_MIN = 0x59D8;
unsigned int MIPI_CLK0_MAX = 0x59D8;
unsigned int MIPI_CLK1_SYS_OP_RATE = 0x4F1A; //for 15fps capture
//unsigned int MIPI_CLK1_SYS_OP_RATE = 0x4074;   //for 12fps capture

unsigned int MIPI_CLK1_MIN = 0x59D8;
unsigned int MIPI_CLK1_MAX = 0x59D8;
unsigned int MIPI_CAP_CLK_IDX = 1;

static void S5K4ECGX_MIPI_Init_Setting(void)
{
    SENSORDB("[4EC] Sensor Init...\n");
    // FOR 4EC EVT1.1
    // ARM Initiation
    //$MIPI[Width:1280,Height:960,Format:YUV422,Lane:2,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:2,DataRate:600]

    unsigned int MIPI_FRM_TIME_MIN = 0;
    unsigned int MIPI_FRM_TIME_MAX = 0;
    unsigned int OUTPUT_FMT = 0;

    MIPI_FRM_TIME_MIN = 0x014D;//0x014A; //0x029A;  //vendor:0x5535;
    MIPI_FRM_TIME_MAX = 0x029A;//0x03E8; //0x0450;  //vendor:0x5535;

    OUTPUT_FMT = 5; //YUV
#if defined(__JPEG_OUTPUT_ENABLE__)
    OUTPUT_FMT = 9; //JPEG
#endif
    S5K4ECGX_Driver.aeState = S5K4ECGX_AE_STATE_UNLOCK;
    S5K4ECGX_Driver.userAskAeLock = 0;//FALSE
    S5K4ECGX_Driver.afMode = S5K4ECGX_AF_MODE_RSVD;
    S5K4ECGX_Driver.afState = S5K4ECGX_AF_STATE_UNINIT;
    S5K4ECGX_Driver.afStateOnOriginalSet = 1;//True

    S5K4ECGX_MIPI_sensor_pclk = MIPI_CLK0_MAX * 4000 * 2; //(reg_val * 4k * 2)
    S5K4ECGX_write_cmos_sensor(0xFCFC, 0xD000);
    S5K4ECGX_write_cmos_sensor(0x0010, 0x0001);  //S/W Reset
    S5K4ECGX_write_cmos_sensor(0x1030, 0x0000);  //contint_host_int
    S5K4ECGX_write_cmos_sensor(0x0014, 0x0001);  //sw_load_complete - Release CORE (Arm) from reset state
    mdelay(50);

    //==================================================================================
    //02.ETC Setting
    //==================================================================================
    S5K4ECGX_write_cmos_sensor(0x0028, 0xD000);
    S5K4ECGX_write_cmos_sensor(0x002A, 0x1082);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //cregs_d0_d4_cd10 //D4[9:8], D3[7:6], D2[5:4], D1[3:2], D0[1:0]
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //cregs_d5_d9_cd10 //D9[9:8], D8[7:6], D7[5:4], D6[3:2], D5[1:0]
    S5K4ECGX_write_cmos_sensor(0x002A, 0x1088);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //cregs_clks_output_cd10 //SDA[11:10], SCL[9:8], PCLK[7:6], VSYNC[3:2], HSYNC[1:0]


    //==================================================================================
    // 03.Analog Setting & ASP Control
    //==================================================================================
    //This register is for FACTORY ONLY.
    //If you change it without prior notification
    //YOU are RESPONSIBLE for the FAILURE that will happen in the future
    S5K4ECGX_write_cmos_sensor(0x002A ,0x007A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x002A ,0xE406);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0092);
    S5K4ECGX_write_cmos_sensor(0x002A ,0xE410);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3804); //[15:8]fadlc_filter_co_b, [7:0]fadlc_filter_co_a
    S5K4ECGX_write_cmos_sensor(0x002A ,0xE41A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010);
    S5K4ECGX_write_cmos_sensor(0x002A ,0xE420);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0003); //adlc_fadlc_filter_refresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //adlc_filter_level_diff_threshold
    S5K4ECGX_write_cmos_sensor(0x002A ,0xE42E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0004); //dithered l-ADLC(4bit)
    S5K4ECGX_write_cmos_sensor(0x002A ,0xF400);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x5A3C); //[15:8]stx_width, [7:0]dstx_width
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0023); //[14]binning_test [13]gain_mode [11:12]row_id [10]cfpn_test [9]pd_pix [8]teg_en, [7]adc_res, [6]smp_en, [5]ldb_en, [4]ld_en, [3]clp_en [2]srx_en, [1]dshut_en, [0]dcds_en
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8080); //CDS option
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03AF); //[11:6]rst_mx, [5:0]sig_mx
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000A); //Avg mode
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xAA54); //x1~x1.49:No MS, x1.5~x3.99:MS2, x4~x16:MS4
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040); //RMP option [6]1: RES gain
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x464E); //[14]msoff_en, [13:8]off_rst, [7:0]adc_sat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0240); //bist_sig_width_e
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0240); //bist_sig_width_o
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040); //[9]dbs_bist_en, [8:0]bist_rst_width
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1000); //[15]aac_en, [14]GCLK_DIV2_EN, [13:10]dl_cont [9:8]dbs_mode, [7:0]dbs_option
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x55FF); //bias [15:12]pix, [11:8]pix_bst [7:4]comp2, [3:0]comp1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD000); //[15:8]clp_lvl, [7:0]ref_option, [5]pix_bst_en
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //[7:0]monit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0202); //[15:8]dbr_tune_tgsl, [7:0]dbr_tune_pix
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0401); //[15:8]dbr_tune_ntg, [7:0]dbr_tune_rg
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0022); //[15:8]reg_option, [7:4]rosc_tune_ncp, [3:0]rosc_tune_cp
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0088); //PD [8]inrush_ctrl, [7]fblv, [6]reg_ntg, [5]reg_tgsl, [4]reg_rg, [3]reg_pix, [2]ncp_rosc, [1]cp_rosc, [0]cp
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x009F); //[9]capa_ctrl_en, [8:7]fb_lv, [6:5]dbr_clk_sel, [4:0]cp_capa
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //[15:0]blst_en_cintr
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1800); //[11]blst_en, [10]rfpn_test, [9]sl_off, [8]tx_off, [7:0]rdv_option
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0088); //[15:0]pmg_reg_tune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //[15:1]analog_dummy, [0]pd_reg_test
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2428); //[13:11]srx_gap1, [10:8]srx_gap0, [7:0]stx_gap
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //[0]atx_option
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03EE); //aig_avg_half
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //[0]hvs_test_reg
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //[0]dbus_bist_auto
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //[7:0]dbr_option
    S5K4ECGX_write_cmos_sensor(0x002A ,0xF552);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0708); //[7:0]lat_st, [15:8]lat_width
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x080C); //[7:0]hold_st, [15:8]hold_width

    //=================================================================================
    // 04.Trap and Patch  Driver IC DW9714  //update by Chris 20130326
    //=================================================================================
    // Start of Patch data
    S5K4ECGX_write_cmos_sensor(0x0028  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A  ,0x3AF8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB570);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4B39);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4939);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x483A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2200);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC008);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4939);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4839);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2401);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFBD4);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4938);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4839);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2502);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0022);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFBCE);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4837);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0261);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2100);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8041);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4936);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4836);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6041);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4936);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4837);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2403);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x002A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFBC0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4832);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4935);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x30C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x63C1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4930);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4834);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3980);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6408);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4833);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4934);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6388);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4934);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4834);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0022);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2504);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFBAF);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4933);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4833);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2405);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x002A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF881);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x491F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4830);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0022);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2506);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x39B6);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1D80);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF879);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x482D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x492D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2407);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x002A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x300C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF872);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4829);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x492B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0022);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2508);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3010);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF86B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4929);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4829);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2409);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x002A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFB8D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4928);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4828);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0022);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x250A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFB87);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4927);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4827);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x240B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x002A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFB81);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4926);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4826);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0022);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x250C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFB7B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4925);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4825);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x240D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x002A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFB75);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4924);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4824);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0022);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFB70);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC70);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC08);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4718);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x018F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4EC2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x037F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1F90);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3C81);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE38B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3CB9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC3B1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4780);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3D17);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0080);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3D53);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB49D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3DFF);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3DB3);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFFFF);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x00FF);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x17E0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3F7B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x053D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0A89);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6CD2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0A9A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x02D2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3FC9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9E65);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x403D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7C49);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x40B1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7C63);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x40CD);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8F01);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x416F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7F3F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x41FD);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x98C5);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB570);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x000C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0015);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0029);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFB2A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x49F8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x00A8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x500C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC70);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC08);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4718);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6808);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6849);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0409);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C09);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4AF3);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8992);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2A00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD00D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2300);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1A89);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x000B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0419);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C09);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x23FF);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x33C1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1810);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4298);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD800);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0003);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0418);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4AEB);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8150);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8191);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4770);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB5F3);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0004);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB081);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9802);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6800);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0600);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0E00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2201);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0015);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0021);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3910);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x408A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x40A5);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4FE4);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0016);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2C10);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xDA03);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8839);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x43A9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8039);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE002);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8879);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x43B1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8079);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFAF6);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2C10);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xDA03);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8839);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4329);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8039);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE002);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8879);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4331);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8079);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x49DA);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8809);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2900);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD102);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFAEF);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9902);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6008);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBCFE);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC08);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4718);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB538);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9C04);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0015);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x002A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFAEA);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4AD1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8811);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2900);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD00F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8820);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4281);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD20C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8861);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8853);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4299);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD200);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1E40);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8020);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8851);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8061);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4368);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1840);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6060);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC38);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC08);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4718);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB5F8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0004);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6808);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2201);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0015);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0021);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3910);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x408A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x40A5);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4FBE);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0016);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2C10);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xDA03);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8839);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x43A9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8039);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE002);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8879);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x43B1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8079);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFAC3);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2C10);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xDA03);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8838);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4328);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8038);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE002);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8878);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4330);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8078);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x48B6);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8800);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD507);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4BB5);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7819);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4AB5);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7810);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7018);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7011);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x49B4);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8188);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBCF8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC08);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4718);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB538);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x48B2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4669);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFAAE);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x48B1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x49B0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x69C2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x31A8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2A00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD008);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x61C4);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x684A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6242);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6282);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x466B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x881A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6302);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x885A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6342);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6A02);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2A00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD00A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6204);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6849);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6281);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x466B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8819);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6301);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8859);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6341);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x49A5);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88C9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x63C1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA96);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE7A6);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB5F0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB08B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x20FF);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1C40);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x49A1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x89CC);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4E9E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6AB1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4284);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD101);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x489F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6081);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6A70);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0200);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA8D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4A96);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8A11);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9109);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2101);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0349);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4288);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD200);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4A92);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8211);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4D97);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8829);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9108);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4A8B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2303);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3222);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1F91);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA7E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8028);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x488E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4987);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6BC2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6AC0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4282);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD201);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8CC8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8028);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88E8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9007);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2240);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4310);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x80E8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0041);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x194B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x001E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3680);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8BB2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xAF04);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x527A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4A7D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x188A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8897);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x83B7);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x33A0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x891F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xAE01);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5277);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8A11);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8119);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1C40);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2806);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD3E9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA5F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA65);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4F79);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x37A8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2800);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD10A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1FE0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x38FD);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1CC0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD105);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4874);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8829);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3818);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6840);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4348);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6078);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4972);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6878);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6B89);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4288);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD300);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0008);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6078);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0041);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xAA04);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5A53);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x194A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x269C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x52B3);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xAB01);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5A59);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x32A0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8111);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1C40);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2806);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD3F0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4965);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9809);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8208);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9808);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8028);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9807);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x80E8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1FE0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x38FD);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD13B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4D64);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x89E8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1FC1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x39FF);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD136);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4C5F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8AE0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA34);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0006);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8B20);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA38);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6AA1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6878);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1809);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0200);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA0B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0022);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3246);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0011);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x310A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2305);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA08);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x66E8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6B23);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0002);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0031);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0018);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA29);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x466B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8518);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6EEA);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6B60);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9900);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA22);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x466B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8558);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0029);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x980A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3170);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA23);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0028);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3060);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8A02);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4946);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3128);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x808A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8A42);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x80CA);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8A80);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8108);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB00B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBCF0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC08);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4718);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB570);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4D46);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4846);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8881);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4846);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8041);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2101);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA12);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4842);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3820);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8BC0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFA15);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4B42);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x220D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0712);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x18A8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8806);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x00E1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x18C9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x81CE);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8846);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x818E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8886);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x824E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8208);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3508);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x042D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C2D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1C64);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0424);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C24);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2C07);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD3EC);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE658);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB510);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4834);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4C34);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8060);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8020);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4831);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3820);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8BC0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF9F2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88E0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4A31);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2800);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD003);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4930);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8849);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2900);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD009);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x03C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8050);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x80D0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8090);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8110);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC10);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC08);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4718);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8050);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8920);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x80D0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8960);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8090);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x89A1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0409);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1409);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8111);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x89E3);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8A24);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2B00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD104);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x17C3);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0F5B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1818);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x10C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8090);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD1E6);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x17C8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0F40);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1840);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x10C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8110);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE7E0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB510);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x000C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4919);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2204);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6820);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5E8A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0140);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1A80);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0280);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8849);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF9C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6020);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE7D2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x38D4);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x17D0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1100);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x171A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4780);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2FCA);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2FC5);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2FC6);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2ED8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2BD0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x17E0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2DE8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x37E0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x210C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1484);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xA006);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0724);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xA000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2270);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2558);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x146C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB510);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x000C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4979);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2208);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6820);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5E8A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0140);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1A80);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0280);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88C9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF986);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6020);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE798);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB5FE);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x000C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6825);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6866);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x68A0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x68E7);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1BA8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x42B5);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xDA00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1B70);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x496D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x486E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x884A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8843);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x435A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2304);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5ECB);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0A92);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x18D2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x02D2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C12);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88CB);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8880);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4343);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0A98);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2308);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5ECB);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x18C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x02C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0411);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1409);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1A08);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4962);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x39E0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6148);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9801);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3040);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7880);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2800);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD103);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9801);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0029);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF959);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8839);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9800);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4281);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD814);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8879);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9800);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4281);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD20C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9801);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0029);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF955);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9801);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0029);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF951);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9801);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0029);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF94D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE003);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9801);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0029);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF948);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9801);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0032);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0039);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF94B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6020);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE5D0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB57C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x484C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xA901);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0004);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF8CF);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x466B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88D9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8898);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4B47);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3346);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1E9A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF943);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4846);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4944);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3812);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3140);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8A42);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x888B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x18D2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8242);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8AC2);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88C9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1851);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x82C1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0020);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4669);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF8B7);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x483F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x214D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8301);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2196);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8381);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x211D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3020);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF931);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF937);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x483A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4C3A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6E00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x60E0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x466B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8818);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8859);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0025);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1A40);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3540);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x61A8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4831);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9900);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3060);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF92F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x466B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8819);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1DE0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x30F9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8741);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8859);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8781);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x71A0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x74A8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC7C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xBC08);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4718);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB5F8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0005);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x6808);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x684A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0412);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C12);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x688E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x68CC);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4922);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x884B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4343);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0A98);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2304);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5ECB);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x18C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x02C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88CB);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4353);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0A9A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2308);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5ECB);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x18D1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x02C9);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0C09);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2701);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x003A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x40AA);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9200);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x002A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3A10);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4097);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2D10);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xDA06);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4A1B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9B00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8812);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x439A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4B19);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x801A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE003);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4B18);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x885A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x43BA);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x805A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0023);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0032);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF8D7);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2D10);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xDA05);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4913);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x9A00);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8808);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4310);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8008);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE003);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4810);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8841);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4339);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8041);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4D0D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x3580);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88AA);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5E30);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2100);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF8E3);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8030);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x88AA);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x5E20);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2100);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF8DC);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8020);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE587);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2558);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2AB8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x145E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2698);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2BB8);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2998);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1100);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xD000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x1789);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x16F1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC3B1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC36D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF6D7);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xB49D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7EDF);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x448D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF004);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE51F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x29EC);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2EF1);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xEE03);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xA58B);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7C49);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7C63);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2DB7);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xEB3D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF061);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF0EF);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xF004);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE51F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x2824);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8EDD);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8DCB);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x8E17);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x98C5);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7C7D);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7E31);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7EAB);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x4778);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x46C0);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xC000);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE59F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xFF1C);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0xE12F);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x7501);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0000);
    // End of Patch Data(Last : 700046CEh)
    // Total Size 3032 (0x0BD8)
    // Addr : 3AF8 , Size : 3030(BD6h)

    // TNP_USER_MBCV_CONTROL
    // TNP_4EC_MBR_TUNE
    // TNP_4EC_FORBIDDEN_TUNE
    // TNP_AF_FINESEARCH_DRIVEBACK
    // TNP_FLASH_ALG
    // TNP_GAS_ALPHA_OTP
    // TNP_AWB_MODUL_COMP
    // TNP_AWB_INIT_QUEUE
    // TNP_AWB_GRID_LOWBR
    // TNP_AWB_GRID_MODULECOMP
    // TNP_AFD_MOTO
    // TNP_ADLC_TUNE
    // TNP_1FRAME_AE
    // TNP_TG_OFF_CFG_CHG_IN_SPOOF_MODE


    //==================================================================================
    // 05.OTP Control
    //==================================================================================
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0722);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0100); ///skl_OTP_usWaitTime This register should be positioned in fornt of D0001000
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0726);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //skl_bUseOTPfunc This is OTP on/off function
    S5K4ECGX_write_cmos_sensor(0x002A, 0x08D6);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //ash_bUseOTPData
    S5K4ECGX_write_cmos_sensor(0x002A, 0x146E);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //awbb_otp_disable
    S5K4ECGX_write_cmos_sensor(0x002A, 0x08DC);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //ash_bUseGasAlphaOTP
    //OTP on
    //002A    0722
    //0F12    0100 ///skl_OTP_usWaitTime This register should be positioned in fornt of D0001000
    //002A    0726
    //0F12    0001 //skl_bUseOTPfunc This is OTP on/off function
    //002A    08D6
    //0F12    0001 //ash_bUseOTPData
    //002A    146E
    //0F12    0000 //awbb_otp_disable
    //002A    08DC
    //0F12    0000 //ash_bUseGasAlphaOTP

    S5K4ECGX_write_cmos_sensor(0x0028, 0xD000);
    S5K4ECGX_write_cmos_sensor(0x002A, 0x1000);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001);

    //==================================================================================
    // 06.Gas_Anti Shading
    //==================================================================================
    // Refer Mon_AWB_RotGain
    S5K4ECGX_write_cmos_sensor(0x0028 ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x08B4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //wbt_bUseOutdoorASH
    S5K4ECGX_write_cmos_sensor(0x002A ,0x08BC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //TVAR_ash_AwbAshCord_0_ 2300K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00DF); //TVAR_ash_AwbAshCord_1_ 2750K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //TVAR_ash_AwbAshCord_2_ 3300K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0125); //TVAR_ash_AwbAshCord_3_ 4150K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x015F); //TVAR_ash_AwbAshCord_4_ 5250K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x017C); //TVAR_ash_AwbAshCord_5_ 6400K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0194); //TVAR_ash_AwbAshCord_6_ 7500K
    S5K4ECGX_write_cmos_sensor(0x002A ,0x08F6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_0__0_ R  // 2300K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_0__1_ GR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_0__2_ GB
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_0__3_ B
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_1__0_ R  // 2750K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_1__1_ GR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_1__2_ GB
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_1__3_ B
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_2__0_ R  // 3300K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_2__1_ GR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_2__2_ GB
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_2__3_ B
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_3__0_ R  // 4150K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_3__1_ GR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_3__2_ GB
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_3__3_ B
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_4__0_ R  // 5250K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_4__1_ GR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_4__2_ GB
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_4__3_ B
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4300); //TVAR_ash_GASAlpha_5__0_ R  // 6400K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_5__1_ GR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_5__2_ GB
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_5__3_ B
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4300); //TVAR_ash_GASAlpha_6__0_ R  // 7500K
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_6__1_ GR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_6__2_ GB
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000); //TVAR_ash_GASAlpha_6__3_ B
    //Outdoor GAS Alpha
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4500);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4000);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x08F4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //ash_bUseGasAlpha


    //GAS High table   If OTP is used, GAS Setting Should be deleted. //
    //BENI 1.1 module 101018//
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0D26);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F0F);

    // TVAR_ash_pGAS_low
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0DB6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x88A2);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xEF5B);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF576);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2242);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xEC90);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFCB2);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD726);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF77C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1CCB);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xDB4D);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0948);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x13C2);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A14);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x017A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xE9B4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x190D);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x16E5);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xCAB2);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x18CD);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A84);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x097E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF076);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xE849);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2CFC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xE460);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xEE89);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0693);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x06B4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF16E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x12B6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F99);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F3B);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xE728);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x19BB);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x058E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xDA99);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x952B);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xE6F0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0163);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1376);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFC0E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF3A2);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xCE5D);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFA86);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x11D3);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xEB02);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFE43);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x17ED);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1320);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0156);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF4FF);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0ACA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x162B);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD2D8);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F4F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0178);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0AD1);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xEDE5);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFBA5);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1A69);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF30F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFC58);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF92D);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x131C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xE607);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1564);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02A8);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x08B5);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF04C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x15D0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFAD0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xEB70);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8564);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xE967);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFFF);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x16A8);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xEFD6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01AF);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD7AD);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01A2);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A4E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF1CE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFA95);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x143F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1046);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF6A1);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF7BB);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0E8D);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x11A3);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xDB43);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1459);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0FFA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0731);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xEC67);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF7CA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1682);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xDF77);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xEEA5);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF71);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x08FF);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF8FA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x138E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x16FE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0BA0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF297);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1717);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF5BB);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xE6B7);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x87A3);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xECB4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF8A1);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1D23);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF35F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF7C7);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD9ED);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF792);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1E98);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD734);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0BA1);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x14E3);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0BB9);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0279);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xDEC5);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2EDC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x010A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD36F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1A6A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03F6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1AE5);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD3FB);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFFA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x26A0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xDF98);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF8DC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF675);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x168E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xEFC9);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A42);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x11D3);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x08BE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xEF30);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1785);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFBF7);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xE573);
    //==================================================================================
    // 07. Analog Setting 2
    //==================================================================================
    //This register is for FACTORY ONLY.
    //If you change it without prior notification
    //YOU are RESPONSIBLE for the FAILURE that will happen in the future
    //For subsampling Size
    S5K4ECGX_write_cmos_sensor(0x002A ,0x18BC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0004);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05B6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05BA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0007);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05BA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x024E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05B6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05BA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x024F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0075);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00CF);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0075);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00D6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0004);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00F0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x029E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05B2);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F8);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0228);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0208);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0238);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0218);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0238);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0009);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00DE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05C0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00DF);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00E4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F8);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01FD);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05B6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05BB);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F8);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0077);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x007E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x024F);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x025E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    // For Capture
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0004);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x09D1);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x09D5);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0008);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x09D5);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02AA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0326);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02AA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x09D1);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02AA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x09D5);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02AA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0327);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0008);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0084);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0008);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x008D);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0008);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02AA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00AA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02AA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03AD);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x09CD);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02AE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02DE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02BE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02EE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02CE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02EE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0009);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0095);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x09DB);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0096);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x009B);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02AE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02B3);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x09D1);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x09D6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02AE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0009);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0327);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0336);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1AF8);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x5A3C); //senHal_TuneStr_AngTuneData1_2_D000F400 register at subsampling
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1896);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0002); //senHal_SamplingType 0002 03EE: PLA setting
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //senHal_SamplingMode 0 : 2 PLA / 1 : 4PLA
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //senHal_PLAOption  [0] VPLA enable  [1] HPLA enable
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1B00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF428);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFFF);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x189E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0FB0); //senHal_ExpMinPixels
    S5K4ECGX_write_cmos_sensor(0x002A ,0x18AC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060);   //senHal_uAddColsBin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //senHal_uAddColsNoBin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x07DC); //senHal_uMinColsBin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05C0); //senHal_uMinColsNoBin
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1AEA);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8080); //senHal_SubF404Tune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //senHal_FullF404Tune
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1AE0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //senHal_bSenAAC
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1A72);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //senHal_bSRX SRX off
    S5K4ECGX_write_cmos_sensor(0x002A ,0x18A2);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0004); //senHal_NExpLinesCheckFine extend Forbidden area line
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1A6A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x009A); //senHal_usForbiddenRightOfs extend right Forbidden area line
    S5K4ECGX_write_cmos_sensor(0x002A ,0x385E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x024C); //Mon_Sen_uExpPixelsOfs
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0EE6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //setot_bUseDigitalHbin
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1B2A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0300); //70001B2A //senHal_TuneStr2_usAngTuneGainTh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00D6); //70001B2C //senHal_TuneStr2_AngTuneF4CA_0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x008D); //70001B2E //senHal_TuneStr2_AngTuneF4CA_1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00CF); //70001B30 //senHal_TuneStr2_AngTuneF4C2_0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0084); //70001B32 //senHal_TuneStr2_AngTuneF4C2_1_
    //==================================================================================
    // 08.AF Setting
    //==================================================================================

    //AF interface setting
    S5K4ECGX_write_cmos_sensor(0x002A, 0x01FC);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_IPRM_LedGpio, for Flash control
    //s002A1720
    //s0F120100 //afd_usFlags, Low voltage AF enable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0003); //REG_TC_IPRM_CM_Init_AfModeType, VCM IIC
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_IPRM_CM_Init_PwmConfig1
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0204);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0061); //REG_TC_IPRM_CM_Init_GpioConfig1, AF Enable GPIO 6
    S5K4ECGX_write_cmos_sensor(0x002A ,0x020C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2F0C); //REG_TC_IPRM_CM_Init_Mi2cBit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0190); //REG_TC_IPRM_CM_Init_Mi2cRateKhz, IIC Speed

    //AF Window Settings
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0294);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //REG_TC_AF_FstWinStartX
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00E3); //REG_TC_AF_FstWinStartY
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0200); //REG_TC_AF_FstWinSizeX
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0238); //REG_TC_AF_FstWinSizeY
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x018C); //REG_TC_AF_ScndWinStartX
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0166); //REG_TC_AF_ScndWinStartY
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00E6); //REG_TC_AF_ScndWinSizeX
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0132); //REG_TC_AF_ScndWinSizeY
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //REG_TC_AF_WinSizesUpdated
    //2nd search setting
    S5K4ECGX_write_cmos_sensor(0x002A ,0x070E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //skl_af_StatOvlpExpFactor
    S5K4ECGX_write_cmos_sensor(0x002A ,0x071E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //skl_af_bAfStatOff
    S5K4ECGX_write_cmos_sensor(0x002A ,0x163C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //af_search_usAeStable
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1648);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x9002); //af_search_usSingleAfFlags
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1652);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0002); //af_search_usFinePeakCount
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //af_search_usFineMaxScale
    S5K4ECGX_write_cmos_sensor(0x002A ,0x15E0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0403); //af_pos_usFineStepNumSize
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1656);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //af_search_usCapturePolicy
    //Peak Threshold
    S5K4ECGX_write_cmos_sensor(0x002A ,0x164C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0003); //af_search_usMinPeakSamples
    S5K4ECGX_write_cmos_sensor(0x002A ,0x163E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //af_search_usPeakThr
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //af_search_usPeakThrLow
    S5K4ECGX_write_cmos_sensor(0x002A ,0x47A8);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //TNP, Macro Threshold register
    //Home Pos
    S5K4ECGX_write_cmos_sensor(0x002A ,0x15D4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //af_pos_usHomePos
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD000); //af_pos_usLowConfPos
    //AF statistics
    S5K4ECGX_write_cmos_sensor(0x002A ,0x169A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF95); //af_search_usConfCheckOrder_1_
    S5K4ECGX_write_cmos_sensor(0x002A ,0x166A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0280); //af_search_usConfThr_4_
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1676);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03A0); //af_search_usConfThr_10_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0320); //af_search_usConfThr_11_
    S5K4ECGX_write_cmos_sensor(0x002A ,0x16BC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0030); //af_stat_usMinStatVal
    S5K4ECGX_write_cmos_sensor(0x002A ,0x16E0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //af_scene_usSceneLowNormBrThr
    S5K4ECGX_write_cmos_sensor(0x002A ,0x16D4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //af_stat_usBpfThresh


    //AF Lens Position Table Settings
    S5K4ECGX_write_cmos_sensor(0x002A ,0x15E8);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //af_pos_usTableLastInd
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0018); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0020); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0028); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0030); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0038); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0048); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0050); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0058); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0068); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0070); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0090); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00A0); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00B0); //af_pos_usTable
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //af_pos_usTable
    //VCM AF driver with PWM/I2C
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1722);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8000); //afd_usParam[0] I2C power down command
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0006); //afd_usParam[1] Position Right Shift
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3FF0); //afd_usParam[2] I2C Data Mask
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03E8); //afd_usParam[3] PWM Period
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //afd_usParam[4] PWM Divider
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0020); //afd_usParam[5] SlowMotion Delay 4. reduce lens collision noise.
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //afd_usParam[6] SlowMotion Threshold
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0008); //afd_usParam[7] Signal Shaping
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040); //afd_usParam[8] Signal Shaping level
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //afd_usParam[9] Signal Shaping level
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //afd_usParam[10] Signal Shaping level
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00E0); //afd_usParam[11] Signal Shaping level
    S5K4ECGX_write_cmos_sensor(0x002A ,0x028C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0003); //REG_TC_AF_AfCmd
    //==================================================================================
    // 09.AWB-BASIC setting
    //==================================================================================

    // AWB init Start point
    S5K4ECGX_write_cmos_sensor(0x002A ,0x145E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0580); //awbb_GainsInit_0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0428); //awbb_GainsInit_1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x07B0); //awbb_GainsInit_2_
    // AWB Convergence Speed
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0008); //awbb_WpFilterMinThr
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0190); //awbb_WpFilterMaxThr
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00A0); //awbb_WpFilterCoef
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0004); //awbb_WpFilterSize
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0002); //awbb_GridEnable
    S5K4ECGX_write_cmos_sensor(0x002A ,0x144E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_RGainOff
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_BGainOff
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GGainOff
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C2); //awbb_Alpha_Comp_Mode
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0002); //awbb_Rpl_InvalidOutDoor
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //awbb_UseGrThrCorr
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0074); //awbb_Use_Filters
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //awbb_CorrectMinNumPatches
    // White Locus
    S5K4ECGX_write_cmos_sensor(0x002A ,0x11F0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x012C); //awbb_IntcR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0121); //awbb_IntcB
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02DF); //awbb_GLocusR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0314); //awbb_GLocusB
    S5K4ECGX_write_cmos_sensor(0x002A ,0x120E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_MovingScale10
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05FD); //awbb_GamutWidthThr1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x036B); //awbb_GamutHeightThr1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0020); //awbb_GamutWidthThr2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x001A); //awbb_GamutHeightThr2
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1278);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFEF7); //awbb_SCDetectionMap_SEC_StartR_B
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0021); //awbb_SCDetectionMap_SEC_StepR_B
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x07D0); //awbb_SCDetectionMap_SEC_SunnyNB
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x07D0); //awbb_SCDetectionMap_SEC_StepNB
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01C8); //awbb_SCDetectionMap_SEC_LowTempR_B
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0096); //awbb_SCDetectionMap_SEC_SunnyNBZone
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0004); //awbb_SCDetectionMap_SEC_LowTempR_BZone
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1224);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0032); //awbb_LowBr
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x001E); //awbb_LowBr_NBzone
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00E2); //awbb_YThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //awbb_YThreshLow_Norm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0002); //awbb_YThreshLow_Low
    S5K4ECGX_write_cmos_sensor(0x002A ,0x2BA4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0004); //Mon_AWB_ByPassMode
    S5K4ECGX_write_cmos_sensor(0x002A ,0x11FC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000C); //awbb_MinNumOfFinalPatches
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1208);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0020); //awbb_MinNumOfChromaclassifpatches
    // Indoor Zone
    S5K4ECGX_write_cmos_sensor(0x002A ,0x101C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0360); //awbb_IndoorGrZones_m_BGrid_0__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x036C); //awbb_IndoorGrZones_m_BGrid_0__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0320); //awbb_IndoorGrZones_m_BGrid_1__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x038A); //awbb_IndoorGrZones_m_BGrid_1__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02E8); //awbb_IndoorGrZones_m_BGrid_2__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0380); //awbb_IndoorGrZones_m_BGrid_2__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02BE); //awbb_IndoorGrZones_m_BGrid_3__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x035A); //awbb_IndoorGrZones_m_BGrid_3__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0298); //awbb_IndoorGrZones_m_BGrid_4__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0334); //awbb_IndoorGrZones_m_BGrid_4__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0272); //awbb_IndoorGrZones_m_BGrid_5__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x030E); //awbb_IndoorGrZones_m_BGrid_5__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x024C); //awbb_IndoorGrZones_m_BGrid_6__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02EA); //awbb_IndoorGrZones_m_BGrid_6__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0230); //awbb_IndoorGrZones_m_BGrid_7__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02CC); //awbb_IndoorGrZones_m_BGrid_7__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0214); //awbb_IndoorGrZones_m_BGrid_8__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02B0); //awbb_IndoorGrZones_m_BGrid_8__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F8); //awbb_IndoorGrZones_m_BGrid_9__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0294); //awbb_IndoorGrZones_m_BGrid_9__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01DC); //awbb_IndoorGrZones_m_BGrid_10__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0278); //awbb_IndoorGrZones_m_BGrid_10__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01C0); //awbb_IndoorGrZones_m_BGrid_11__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0264); //awbb_IndoorGrZones_m_BGrid_11__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01AA); //awbb_IndoorGrZones_m_BGrid_12__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0250); //awbb_IndoorGrZones_m_BGrid_12__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0196); //awbb_IndoorGrZones_m_BGrid_13__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x023C); //awbb_IndoorGrZones_m_BGrid_13__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0180); //awbb_IndoorGrZones_m_BGrid_14__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0228); //awbb_IndoorGrZones_m_BGrid_14__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x016C); //awbb_IndoorGrZones_m_BGrid_15__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0214); //awbb_IndoorGrZones_m_BGrid_15__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0168); //awbb_IndoorGrZones_m_BGrid_16__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0200); //awbb_IndoorGrZones_m_BGrid_16__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0172); //awbb_IndoorGrZones_m_BGrid_17__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01EC); //awbb_IndoorGrZones_m_BGrid_17__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x019A); //awbb_IndoorGrZones_m_BGrid_18__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01D8); //awbb_IndoorGrZones_m_BGrid_18__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_IndoorGrZones_m_BGrid_19__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_IndoorGrZones_m_BGrid_19__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0005); //awbb_IndoorGrZones_m_GridStep
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1070);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0013); //awbb_IndoorGrZones_ZInfo_m_GridSz
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1074);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00EC); //awbb_IndoorGrZones_m_Boffs
    // Outdoor Zone
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1078);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0232); //awbb_OutdoorGrZones_m_BGrid_0__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x025A); //awbb_OutdoorGrZones_m_BGrid_0__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x021E); //awbb_OutdoorGrZones_m_BGrid_1__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0274); //awbb_OutdoorGrZones_m_BGrid_1__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x020E); //awbb_OutdoorGrZones_m_BGrid_2__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x028E); //awbb_OutdoorGrZones_m_BGrid_2__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0200); //awbb_OutdoorGrZones_m_BGrid_3__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0290); //awbb_OutdoorGrZones_m_BGrid_3__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4); //awbb_OutdoorGrZones_m_BGrid_4__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0286); //awbb_OutdoorGrZones_m_BGrid_4__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01E8); //awbb_OutdoorGrZones_m_BGrid_5__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x027E); //awbb_OutdoorGrZones_m_BGrid_5__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01DE); //awbb_OutdoorGrZones_m_BGrid_6__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0274); //awbb_OutdoorGrZones_m_BGrid_6__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01D2); //awbb_OutdoorGrZones_m_BGrid_7__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0268); //awbb_OutdoorGrZones_m_BGrid_7__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01D0); //awbb_OutdoorGrZones_m_BGrid_8__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x025E); //awbb_OutdoorGrZones_m_BGrid_8__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01D6); //awbb_OutdoorGrZones_m_BGrid_9__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0252); //awbb_OutdoorGrZones_m_BGrid_9__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01E2); //awbb_OutdoorGrZones_m_BGrid_10__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0248); //awbb_OutdoorGrZones_m_BGrid_10__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4); //awbb_OutdoorGrZones_m_BGrid_11__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x021A); //awbb_OutdoorGrZones_m_BGrid_11__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0004); //awbb_OutdoorGrZones_m_GridStep
    S5K4ECGX_write_cmos_sensor(0x002A ,0x10AC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000C); //awbb_OutdoorGrZones_ZInfo_m_GridSz
    S5K4ECGX_write_cmos_sensor(0x002A ,0x10B0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01DA); //awbb_OutdoorGrZones_m_Boffs
    // Low Brightness Zone
    S5K4ECGX_write_cmos_sensor(0x002A ,0x10B4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0348); //awbb_LowBrGrZones_m_BGrid_0__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03B6); //awbb_LowBrGrZones_m_BGrid_0__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02B8); //awbb_LowBrGrZones_m_BGrid_1__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03B6); //awbb_LowBrGrZones_m_BGrid_1__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0258); //awbb_LowBrGrZones_m_BGrid_2__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x038E); //awbb_LowBrGrZones_m_BGrid_2__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0212); //awbb_LowBrGrZones_m_BGrid_3__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0348); //awbb_LowBrGrZones_m_BGrid_3__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01CC); //awbb_LowBrGrZones_m_BGrid_4__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x030C); //awbb_LowBrGrZones_m_BGrid_4__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01A2); //awbb_LowBrGrZones_m_BGrid_5__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02D2); //awbb_LowBrGrZones_m_BGrid_5__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0170); //awbb_LowBrGrZones_m_BGrid_6__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02A6); //awbb_LowBrGrZones_m_BGrid_6__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x014C); //awbb_LowBrGrZones_m_BGrid_7__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0280); //awbb_LowBrGrZones_m_BGrid_7__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0128); //awbb_LowBrGrZones_m_BGrid_8__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x025C); //awbb_LowBrGrZones_m_BGrid_8__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0146); //awbb_LowBrGrZones_m_BGrid_9__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0236); //awbb_LowBrGrZones_m_BGrid_9__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0164); //awbb_LowBrGrZones_m_BGrid_10__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0212); //awbb_LowBrGrZones_m_BGrid_10__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_LowBrGrZones_m_BGrid_11__m_left
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_LowBrGrZones_m_BGrid_11__m_right
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0006); //awbb_LowBrGrZones_m_GridStep
    S5K4ECGX_write_cmos_sensor(0x002A ,0x10E8);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //awbb_LowBrGrZones_ZInfo_m_GridSz
    S5K4ECGX_write_cmos_sensor(0x002A ,0x10EC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00D2); //awbb_LowBrGrZones_m_Boffs

    // Low Temp. Zone
    S5K4ECGX_write_cmos_sensor(0x002A ,0x10F0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x039A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_CrclLowT_R_c
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00FE);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_CrclLowT_B_c
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2284);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_CrclLowT_Rad_c

    //AWB - GridCorrection
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1434);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02C1); //awbb_GridConst_1_0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x033A); //awbb_GridConst_1_1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x038A); //awbb_GridConst_1_2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x101A); //awbb_GridConst_2_0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1075); //awbb_GridConst_2_1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x113D); //awbb_GridConst_2_2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x113F); //awbb_GridConst_2_3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x11AF); //awbb_GridConst_2_4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x11F0); //awbb_GridConst_2_5_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00B2); //awbb_GridCoeff_R_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00B8); //awbb_GridCoeff_B_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00CA); //awbb_GridCoeff_R_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x009D); //awbb_GridCoeff_B_2

    // Indoor Grid Offset
    S5K4ECGX_write_cmos_sensor(0x002A ,0x13A4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE0); //awbb_GridCorr_R_0__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE0); //awbb_GridCorr_R_0__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE0); //awbb_GridCorr_R_0__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFA0); //awbb_GridCorr_R_0__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFEE); //awbb_GridCorr_R_0__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0096); //awbb_GridCorr_R_0__5_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE0); //awbb_GridCorr_R_1__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE0); //awbb_GridCorr_R_1__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE0); //awbb_GridCorr_R_1__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFA0); //awbb_GridCorr_R_1__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFEE); //awbb_GridCorr_R_1__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0096); //awbb_GridCorr_R_1__5_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE0); //awbb_GridCorr_R_2__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE0); //awbb_GridCorr_R_2__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE0); //awbb_GridCorr_R_2__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFA0); //awbb_GridCorr_R_2__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFEE); //awbb_GridCorr_R_2__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0096); //awbb_GridCorr_R_2__5_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_B_0__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_B_0__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_B_0__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF38); //awbb_GridCorr_B_0__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFEF2); //awbb_GridCorr_B_0__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFE5C); //awbb_GridCorr_B_0__5_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_B_1__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_B_1__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_B_1__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF38); //awbb_GridCorr_B_1__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFEF2); //awbb_GridCorr_B_1__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFE5C); //awbb_GridCorr_B_1__5_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_B_2__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_B_2__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_B_2__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF38); //awbb_GridCorr_B_2__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFEF2); //awbb_GridCorr_B_2__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFE5C); //awbb_GridCorr_B_2__5_

    // Outdoor Grid Offset
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_R_Out_0__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_R_Out_0__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_R_Out_0__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_R_Out_0__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_R_Out_0__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_R_Out_0__5_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_R_Out_1__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_R_Out_1__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_R_Out_1__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_R_Out_1__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_R_Out_1__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_R_Out_1__5_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFC0); //awbb_GridCorr_R_Out_2__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_R_Out_2__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_R_Out_2__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_R_Out_2__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_R_Out_2__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_R_Out_2__5_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //awbb_GridCorr_B_Out_0__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_B_Out_0__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_B_Out_0__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_B_Out_0__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_B_Out_0__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_B_Out_0__5_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //awbb_GridCorr_B_Out_1__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_B_Out_1__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_B_Out_1__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_B_Out_1__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_B_Out_1__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_B_Out_1__5_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //awbb_GridCorr_B_Out_2__0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_B_Out_2__1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_B_Out_2__2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD0); //awbb_GridCorr_B_Out_2__3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_B_Out_2__4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //awbb_GridCorr_B_Out_2__5_


    //==================================================================================
    // 11.Auto Flicker Detection
    //==================================================================================
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0F30);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //AFC_D_ConvAccelerPower
    //Auto Flicker (60Mhz start)
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0F2A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);  //AFC_Default60Hz 0001:60Hz 0000h:50Hz
    S5K4ECGX_write_cmos_sensor(0x002A ,0x04E6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x077F);  //REG_TC_DBG

    //==================================================================================
    // 12.AE Setting
    //==================================================================================
    //AE Target
    S5K4ECGX_write_cmos_sensor(0x002A, 0x1484);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x003C);   //TVAR_ae_BrAve
    //ae_StatMode bit[3] BLC has to be bypassed to prevent AE weight change especially backlight scene
    S5K4ECGX_write_cmos_sensor(0x002A ,0x148A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000F);   //ae_StatMode
    S5K4ECGX_write_cmos_sensor(0x002A ,0x058C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3520);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //lt_uMaxExp1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD4C0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //lt_uMaxExp2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3520);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //lt_uCapMaxExp1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD4C0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //lt_uCapMaxExp2
    S5K4ECGX_write_cmos_sensor(0x002A ,0x059C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0470);   //lt_uMaxAnGain1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0C00);   //lt_uMaxAnGain2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);   //lt_uMaxDigGain
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1000);   //lt_uMaxTotGain
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0544);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0111);   //lt_uLimitHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00EF);   //lt_uLimitLow
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0608);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //lt_ExpGain_uSubsamplingmode
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //lt_ExpGain_uNonSubsampling
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0800);   //lt_ExpGain_ExpCurveGainMaxStr
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);   //0100   //lt_ExpGain_ExpCurveGainMaxStr_0__uMaxDigGain
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //0001
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000   //lt_ExpGain_ExpCurveGainMaxStr_0__ulExpIn_0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A3C);   //0A3C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0D05);   //0D05
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4008);   //4008
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x7000);   //7400  //?? //700Lux
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x9C00);   //C000  //?? //9C00->9F->A5 //400Lux
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xAD00);   //AD00
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //0001
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xF1D4);   //F1D4
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0002);   //0002
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xDC00);   //DC00
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0005);   //0005
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xDC00);   //DC00
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0005);   //0005         //
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0638);   //0638
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //0001
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000   //lt_ExpGain_ExpCurveGainMaxStr_0__ulExpOut_0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A3C);   //0A3C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0D05);   //0D05
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3408);   //3408
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3408);   //3408
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x6810);   //6810
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8214);   //8214
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xC350);   //C350
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD4C0);   //C350
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //0000
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xD4C0);   //C350
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //0000
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0660);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0650);   //lt_ExpGain_ExpCurveGainMaxStr_1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);   //lt_ExpGain_ExpCurveGainMaxStr_1__uMaxDigGain
    S5K4ECGX_write_cmos_sensor(0x002A ,0x06B8);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x452C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000A);   //0005   //lt_uMaxLei
    S5K4ECGX_write_cmos_sensor(0x002A ,0x05D0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);   //lt_mbr_Peak_behind

    //==================================================================================
    // 13.AE Weight (Normal)
    //==================================================================================
    S5K4ECGX_MIPI_AE_Rollback_Weight_Table();

    //==================================================================================
    // 14.Flash Setting
    //==================================================================================
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0484);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0002);     //capture flash on
    S5K4ECGX_write_cmos_sensor(0x002A ,0x183A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);     //one frame AE
    S5K4ECGX_write_cmos_sensor(0x002A ,0x17F6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x023C);     //AWB R point
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0248);     //AWB B point
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1840);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);     // Fls AE tune start
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);     // fls_afl_FlsAFIn  Rin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0120);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0180);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0200);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0400);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0800);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);     // fls_afl_FlsAFOut  Rout
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00A0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0090);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0070);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0045);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0030);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1884);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);     // fls_afl_FlsNBOut  flash NB default
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1826);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);     // fls_afl_FlashWP_Weight  flash NB default
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000A);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0030);     // fls_afl_FlashWP_Weight  flash NB default
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0048);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0050);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x4784);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00A0);     // TNP_Regs_FlsWeightRIn  weight tune start in
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00D0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0200);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0300);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0088);     // TNP_Regs_FlsWeightROut  weight tune start out
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00B0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0200);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0300);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0120);     //Fls  BRIn
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0150);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0200);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x003C);     // Fls  BROut
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x003B);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0026);     //brightness



    //==================================================================================
    // 15.CCM Setting
    //==================================================================================
    S5K4ECGX_write_cmos_sensor(0x002A ,0x08A6);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //SARR_AwbCcmCord[0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //SARR_AwbCcmCord[1]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0125); //SARR_AwbCcmCord[2]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x015F); //SARR_AwbCcmCord[3]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x017C); //SARR_AwbCcmCord[4]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0194); //SARR_AwbCcmCord[5]
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0898);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4800); //TVAR_wbt_pBaseCcms
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x08A0);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x48D8); //TVAR_wbt_pOutdoorCcm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x7000);
    //Horizon
    S5K4ECGX_write_cmos_sensor(0x002A ,0x4800);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0208); //TVAR_wbt_pBaseCcms[0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFB5); //TVAR_wbt_pBaseCcms[1]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE8); //TVAR_wbt_pBaseCcms[2]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF20); //TVAR_wbt_pBaseCcms[3]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01BF); //TVAR_wbt_pBaseCcms[4]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF53); //TVAR_wbt_pBaseCcms[5]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0022); //TVAR_wbt_pBaseCcms[6]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFEA); //TVAR_wbt_pBaseCcms[7]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01C2); //TVAR_wbt_pBaseCcms[8]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C6); //TVAR_wbt_pBaseCcms[9]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0095); //TVAR_wbt_pBaseCcms[10]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFEFD); //TVAR_wbt_pBaseCcms[11]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0206); //TVAR_wbt_pBaseCcms[12]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF7F); //TVAR_wbt_pBaseCcms[13]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0191); //TVAR_wbt_pBaseCcms[14]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF06); //TVAR_wbt_pBaseCcms[15]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01BA); //TVAR_wbt_pBaseCcms[16]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0108); //TVAR_wbt_pBaseCcms[17]

    // INCA A
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0208); //TVAR_wbt_pBaseCcms[18]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFB5); //TVAR_wbt_pBaseCcms[19]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE8); //TVAR_wbt_pBaseCcms[20]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF20); //TVAR_wbt_pBaseCcms[21]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01BF); //TVAR_wbt_pBaseCcms[22]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF53); //TVAR_wbt_pBaseCcms[23]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0022); //TVAR_wbt_pBaseCcms[24]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFEA); //TVAR_wbt_pBaseCcms[25]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01C2); //TVAR_wbt_pBaseCcms[26]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C6); //TVAR_wbt_pBaseCcms[27]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0095); //TVAR_wbt_pBaseCcms[28]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFEFD); //TVAR_wbt_pBaseCcms[29]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0206); //TVAR_wbt_pBaseCcms[30]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF7F); //TVAR_wbt_pBaseCcms[31]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0191); //TVAR_wbt_pBaseCcms[32]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF06); //TVAR_wbt_pBaseCcms[33]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01BA); //TVAR_wbt_pBaseCcms[34]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0108); //TVAR_wbt_pBaseCcms[35]
    //Warm White
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0208); //TVAR_wbt_pBaseCcms[36]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFB5); //TVAR_wbt_pBaseCcms[37]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE8); //TVAR_wbt_pBaseCcms[38]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF20); //TVAR_wbt_pBaseCcms[39]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01BF); //TVAR_wbt_pBaseCcms[40]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF53); //TVAR_wbt_pBaseCcms[41]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0022); //TVAR_wbt_pBaseCcms[42]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFEA); //TVAR_wbt_pBaseCcms[43]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01C2); //TVAR_wbt_pBaseCcms[44]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C6); //TVAR_wbt_pBaseCcms[45]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0095); //TVAR_wbt_pBaseCcms[46]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFEFD); //TVAR_wbt_pBaseCcms[47]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0206); //TVAR_wbt_pBaseCcms[48]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF7F); //TVAR_wbt_pBaseCcms[49]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0191); //TVAR_wbt_pBaseCcms[50]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF06); //TVAR_wbt_pBaseCcms[51]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01BA); //TVAR_wbt_pBaseCcms[52]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0108); //TVAR_wbt_pBaseCcms[53]
    //Cool White
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0204); //TVAR_wbt_pBaseCcms[54]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFB2); //TVAR_wbt_pBaseCcms[55]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFF5); //TVAR_wbt_pBaseCcms[56]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFEF1); //TVAR_wbt_pBaseCcms[57]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x014E); //TVAR_wbt_pBaseCcms[58]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF18); //TVAR_wbt_pBaseCcms[59]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE6); //TVAR_wbt_pBaseCcms[60]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFDD); //TVAR_wbt_pBaseCcms[61]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01B2); //TVAR_wbt_pBaseCcms[62]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00F2); //TVAR_wbt_pBaseCcms[63]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00CA); //TVAR_wbt_pBaseCcms[64]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF48); //TVAR_wbt_pBaseCcms[65]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0151); //TVAR_wbt_pBaseCcms[66]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF50); //TVAR_wbt_pBaseCcms[67]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0147); //TVAR_wbt_pBaseCcms[68]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF75); //TVAR_wbt_pBaseCcms[69]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0187); //TVAR_wbt_pBaseCcms[70]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01BF); //TVAR_wbt_pBaseCcms[71]
    //D50
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0204); //TVAR_wbt_pBaseCcms[72]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFB2); //TVAR_wbt_pBaseCcms[73]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFF5); //TVAR_wbt_pBaseCcms[74]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFEF1); //TVAR_wbt_pBaseCcms[75]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x014E); //TVAR_wbt_pBaseCcms[76]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF18); //TVAR_wbt_pBaseCcms[77]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE6); //TVAR_wbt_pBaseCcms[78]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFDD); //TVAR_wbt_pBaseCcms[79]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01B2); //TVAR_wbt_pBaseCcms[80]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00F2); //TVAR_wbt_pBaseCcms[81]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00CA); //TVAR_wbt_pBaseCcms[82]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF48); //TVAR_wbt_pBaseCcms[83]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0151); //TVAR_wbt_pBaseCcms[84]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF50); //TVAR_wbt_pBaseCcms[85]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0147); //TVAR_wbt_pBaseCcms[86]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF75); //TVAR_wbt_pBaseCcms[87]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0187); //TVAR_wbt_pBaseCcms[88]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01BF); //TVAR_wbt_pBaseCcms[89]
    //D65
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0204); //TVAR_wbt_pBaseCcms[90]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFB2); //TVAR_wbt_pBaseCcms[91]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFF5); //TVAR_wbt_pBaseCcms[92]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFEF1); //TVAR_wbt_pBaseCcms[93]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x014E); //TVAR_wbt_pBaseCcms[94]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF18); //TVAR_wbt_pBaseCcms[95]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFE6); //TVAR_wbt_pBaseCcms[96]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFDD); //TVAR_wbt_pBaseCcms[97]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01B2); //TVAR_wbt_pBaseCcms[98]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00F2); //TVAR_wbt_pBaseCcms[99]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00CA); //TVAR_wbt_pBaseCcms[100]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF48); //TVAR_wbt_pBaseCcms[101]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0151); //TVAR_wbt_pBaseCcms[102]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF50); //TVAR_wbt_pBaseCcms[103]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0147); //TVAR_wbt_pBaseCcms[104]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF75); //TVAR_wbt_pBaseCcms[105]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0187); //TVAR_wbt_pBaseCcms[106]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01BF); //TVAR_wbt_pBaseCcms[107]
    //Outdoor
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01E5); //TVAR_wbt_pOutdoorCcm[0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFA4); //TVAR_wbt_pOutdoorCcm[1]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFDC); //TVAR_wbt_pOutdoorCcm[2]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFE90); //TVAR_wbt_pOutdoorCcm[3]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x013F); //TVAR_wbt_pOutdoorCcm[4]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF1B); //TVAR_wbt_pOutdoorCcm[5]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFD2); //TVAR_wbt_pOutdoorCcm[6]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFDF); //TVAR_wbt_pOutdoorCcm[7]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0236); //TVAR_wbt_pOutdoorCcm[8]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00EC); //TVAR_wbt_pOutdoorCcm[9]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00F8); //TVAR_wbt_pOutdoorCcm[10]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF34); //TVAR_wbt_pOutdoorCcm[11]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01CE); //TVAR_wbt_pOutdoorCcm[12]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF83); //TVAR_wbt_pOutdoorCcm[13]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0195); //TVAR_wbt_pOutdoorCcm[14]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFEF3); //TVAR_wbt_pOutdoorCcm[15]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0126); //TVAR_wbt_pOutdoorCcm[16]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0162); //TVAR_wbt_pOutdoorCcm[17]

    //==================================================================================
    // 16.GAMMA
    //==================================================================================
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0734);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //saRR_usDualGammaLutRGBIndoor[0][0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000A); //saRR_usDualGammaLutRGBIndoor[0][1]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0016); //saRR_usDualGammaLutRGBIndoor[0][2]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0030); //saRR_usDualGammaLutRGBIndoor[0][3]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0066); //saRR_usDualGammaLutRGBIndoor[0][4]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00D5); //saRR_usDualGammaLutRGBIndoor[0][5]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0138); //saRR_usDualGammaLutRGBIndoor[0][6]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0163); //saRR_usDualGammaLutRGBIndoor[0][7]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0189); //saRR_usDualGammaLutRGBIndoor[0][8]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01C6); //saRR_usDualGammaLutRGBIndoor[0][9]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F8); //saRR_usDualGammaLutRGBIndoor[0][10]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0222); //saRR_usDualGammaLutRGBIndoor[0][11]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0247); //saRR_usDualGammaLutRGBIndoor[0][12]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0282); //saRR_usDualGammaLutRGBIndoor[0][13]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02B5); //saRR_usDualGammaLutRGBIndoor[0][14]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x030F); //saRR_usDualGammaLutRGBIndoor[0][15]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x035F); //saRR_usDualGammaLutRGBIndoor[0][16]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03A2); //saRR_usDualGammaLutRGBIndoor[0][17]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03D8); //saRR_usDualGammaLutRGBIndoor[0][18]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03FF); //saRR_usDualGammaLutRGBIndoor[0][19]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //saRR_usDualGammaLutRGBIndoor[1][0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000A); //saRR_usDualGammaLutRGBIndoor[1][1]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0016); //saRR_usDualGammaLutRGBIndoor[1][2]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0030); //saRR_usDualGammaLutRGBIndoor[1][3]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0066); //saRR_usDualGammaLutRGBIndoor[1][4]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00D5); //saRR_usDualGammaLutRGBIndoor[1][5]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0138); //saRR_usDualGammaLutRGBIndoor[1][6]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0163); //saRR_usDualGammaLutRGBIndoor[1][7]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0189); //saRR_usDualGammaLutRGBIndoor[1][8]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01C6); //saRR_usDualGammaLutRGBIndoor[1][9]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F8); //saRR_usDualGammaLutRGBIndoor[1][10]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0222); //saRR_usDualGammaLutRGBIndoor[1][11]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0247); //saRR_usDualGammaLutRGBIndoor[1][12]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0282); //saRR_usDualGammaLutRGBIndoor[1][13]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02B5); //saRR_usDualGammaLutRGBIndoor[1][14]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x030F); //saRR_usDualGammaLutRGBIndoor[1][15]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x035F); //saRR_usDualGammaLutRGBIndoor[1][16]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03A2); //saRR_usDualGammaLutRGBIndoor[1][17]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03D8); //saRR_usDualGammaLutRGBIndoor[1][18]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03FF); //saRR_usDualGammaLutRGBIndoor[1][19]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //saRR_usDualGammaLutRGBIndoor[2][0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000A); //saRR_usDualGammaLutRGBIndoor[2][1]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0016); //saRR_usDualGammaLutRGBIndoor[2][2]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0030); //saRR_usDualGammaLutRGBIndoor[2][3]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0066); //saRR_usDualGammaLutRGBIndoor[2][4]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00D5); //saRR_usDualGammaLutRGBIndoor[2][5]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0138); //saRR_usDualGammaLutRGBIndoor[2][6]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0163); //saRR_usDualGammaLutRGBIndoor[2][7]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0189); //saRR_usDualGammaLutRGBIndoor[2][8]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01C6); //saRR_usDualGammaLutRGBIndoor[2][9]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F8); //saRR_usDualGammaLutRGBIndoor[2][10]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0222); //saRR_usDualGammaLutRGBIndoor[2][11]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0247); //saRR_usDualGammaLutRGBIndoor[2][12]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0282); //saRR_usDualGammaLutRGBIndoor[2][13]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02B5); //saRR_usDualGammaLutRGBIndoor[2][14]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x030F); //saRR_usDualGammaLutRGBIndoor[2][15]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x035F); //saRR_usDualGammaLutRGBIndoor[2][16]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03A2); //saRR_usDualGammaLutRGBIndoor[2][17]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03D8); //saRR_usDualGammaLutRGBIndoor[2][18]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03FF); //saRR_usDualGammaLutRGBIndoor[2][19]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //saRR_usDualGammaLutRGBOutdoor[0][0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //saRR_usDualGammaLutRGBOutdoor[0][1]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0019); //saRR_usDualGammaLutRGBOutdoor[0][2]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0036); //saRR_usDualGammaLutRGBOutdoor[0][3]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x006F); //saRR_usDualGammaLutRGBOutdoor[0][4]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00D8); //saRR_usDualGammaLutRGBOutdoor[0][5]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0135); //saRR_usDualGammaLutRGBOutdoor[0][6]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x015F); //saRR_usDualGammaLutRGBOutdoor[0][7]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0185); //saRR_usDualGammaLutRGBOutdoor[0][8]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01C1); //saRR_usDualGammaLutRGBOutdoor[0][9]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F3); //saRR_usDualGammaLutRGBOutdoor[0][10]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0220); //saRR_usDualGammaLutRGBOutdoor[0][11]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x024A); //saRR_usDualGammaLutRGBOutdoor[0][12]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0291); //saRR_usDualGammaLutRGBOutdoor[0][13]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02D0); //saRR_usDualGammaLutRGBOutdoor[0][14]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x032A); //saRR_usDualGammaLutRGBOutdoor[0][15]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x036A); //saRR_usDualGammaLutRGBOutdoor[0][16]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x039F); //saRR_usDualGammaLutRGBOutdoor[0][17]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03CC); //saRR_usDualGammaLutRGBOutdoor[0][18]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03F9); //saRR_usDualGammaLutRGBOutdoor[0][19]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //saRR_usDualGammaLutRGBOutdoor[1][0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //saRR_usDualGammaLutRGBOutdoor[1][1]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0019); //saRR_usDualGammaLutRGBOutdoor[1][2]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0036); //saRR_usDualGammaLutRGBOutdoor[1][3]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x006F); //saRR_usDualGammaLutRGBOutdoor[1][4]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00D8); //saRR_usDualGammaLutRGBOutdoor[1][5]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0135); //saRR_usDualGammaLutRGBOutdoor[1][6]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x015F); //saRR_usDualGammaLutRGBOutdoor[1][7]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0185); //saRR_usDualGammaLutRGBOutdoor[1][8]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01C1); //saRR_usDualGammaLutRGBOutdoor[1][9]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F3); //saRR_usDualGammaLutRGBOutdoor[1][10]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0220); //saRR_usDualGammaLutRGBOutdoor[1][11]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x024A); //saRR_usDualGammaLutRGBOutdoor[1][12]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0291); //saRR_usDualGammaLutRGBOutdoor[1][13]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02D0); //saRR_usDualGammaLutRGBOutdoor[1][14]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x032A); //saRR_usDualGammaLutRGBOutdoor[1][15]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x036A); //saRR_usDualGammaLutRGBOutdoor[1][16]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x039F); //saRR_usDualGammaLutRGBOutdoor[1][17]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03CC); //saRR_usDualGammaLutRGBOutdoor[1][18]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03F9); //saRR_usDualGammaLutRGBOutdoor[1][19]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //saRR_usDualGammaLutRGBOutdoor[2][0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //saRR_usDualGammaLutRGBOutdoor[2][1]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0019); //saRR_usDualGammaLutRGBOutdoor[2][2]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0036); //saRR_usDualGammaLutRGBOutdoor[2][3]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x006F); //saRR_usDualGammaLutRGBOutdoor[2][4]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00D8); //saRR_usDualGammaLutRGBOutdoor[2][5]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0135); //saRR_usDualGammaLutRGBOutdoor[2][6]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x015F); //saRR_usDualGammaLutRGBOutdoor[2][7]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0185); //saRR_usDualGammaLutRGBOutdoor[2][8]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01C1); //saRR_usDualGammaLutRGBOutdoor[2][9]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F3); //saRR_usDualGammaLutRGBOutdoor[2][10]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0220); //saRR_usDualGammaLutRGBOutdoor[2][11]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x024A); //saRR_usDualGammaLutRGBOutdoor[2][12]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0291); //saRR_usDualGammaLutRGBOutdoor[2][13]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x02D0); //saRR_usDualGammaLutRGBOutdoor[2][14]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x032A); //saRR_usDualGammaLutRGBOutdoor[2][15]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x036A); //saRR_usDualGammaLutRGBOutdoor[2][16]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x039F); //saRR_usDualGammaLutRGBOutdoor[2][17]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03CC); //saRR_usDualGammaLutRGBOutdoor[2][18]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x03F9); //saRR_usDualGammaLutRGBOutdoor[2][19]

    //==================================================================================
    // 17.AFIT
    //==================================================================================
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0944);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0050); //afit_uNoiseIndInDoor
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00B0); //afit_uNoiseIndInDoor
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0196); //afit_uNoiseIndInDoor
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0245); //afit_uNoiseIndInDoor
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0300); //afit_uNoiseIndInDoor
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0938);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); // on/off AFIT by NB option
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0014); //SARR_uNormBrInDoor
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00D2); //SARR_uNormBrInDoor
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0384); //SARR_uNormBrInDoor
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x07D0); //SARR_uNormBrInDoor
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1388); //SARR_uNormBrInDoor
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0976);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0070); //afit_usGamutTh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0005); //afit_usNeargrayOffset
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //afit_bUseSenBpr
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01CC); //afit_usBprThr_0_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01CC); //afit_usBprThr_1_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01CC); //afit_usBprThr_2_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01CC); //afit_usBprThr_3_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01CC); //afit_usBprThr_4_
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0180); //afit_NIContrastAFITValue
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0196); //afit_NIContrastTh
    S5K4ECGX_write_cmos_sensor(0x002A ,0x098C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //7000098C//AFIT16_BRIGHTNESS
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //7000098E//AFIT16_CONTRAST
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000990//AFIT16_SATURATION
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000992//AFIT16_SHARP_BLUR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000994//AFIT16_GLAMOUR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //70000996//AFIT16_bnr_edge_high
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0064); //70000998//AFIT16_postdmsc_iLowBright
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0384); //7000099A//AFIT16_postdmsc_iHighBright
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x005F); //7000099C//AFIT16_postdmsc_iLowSat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4); //7000099E//AFIT16_postdmsc_iHighSat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0070); //700009A0//AFIT16_postdmsc_iTune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040); //700009A2//AFIT16_yuvemix_mNegRanges_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00A0); //700009A4//AFIT16_yuvemix_mNegRanges_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //700009A6//AFIT16_yuvemix_mNegRanges_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //700009A8//AFIT16_yuvemix_mPosRanges_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040); //700009AA//AFIT16_yuvemix_mPosRanges_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00A0); //700009AC//AFIT16_yuvemix_mPosRanges_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1430); //700009AE//AFIT8_bnr_edge_low  [7:0] AFIT8_bnr_repl_thresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0201); //700009B0//AFIT8_bnr_repl_force  [7:0] AFIT8_bnr_iHotThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0204); //700009B2//AFIT8_bnr_iHotThreshLow   [7:0] AFIT8_bnr_iColdThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3604); //700009B4//AFIT8_bnr_iColdThreshLow   [7:0] AFIT8_bnr_DispTH_Low
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x032A); //700009B6//AFIT8_bnr_DispTH_High   [7:0] AFIT8_bnr_DISP_Limit_Low
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0403); //700009B8//AFIT8_bnr_DISP_Limit_High   [7:0] AFIT8_bnr_iDistSigmaMin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1B06); //700009BA//AFIT8_bnr_iDistSigmaMax   [7:0] AFIT8_bnr_iDiffSigmaLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x6015); //700009BC//AFIT8_bnr_iDiffSigmaHigh   [7:0] AFIT8_bnr_iNormalizedSTD_TH
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //700009BE//AFIT8_bnr_iNormalizedSTD_Limit [7:0] AFIT8_bnr_iDirNRTune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x6080); //700009C0//AFIT8_bnr_iDirMinThres [7:0] AFIT8_bnr_iDirFltDiffThresHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4080); //700009C2//AFIT8_bnr_iDirFltDiffThresLow   [7:0] AFIT8_bnr_iDirSmoothPowerHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0640); //700009C4//AFIT8_bnr_iDirSmoothPowerLow   [7:0] AFIT8_bnr_iLowMaxSlopeAllowed
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0306); //700009C6//AFIT8_bnr_iHighMaxSlopeAllowed [7:0] AFIT8_bnr_iLowSlopeThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2003); //700009C8//AFIT8_bnr_iHighSlopeThresh [7:0] AFIT8_bnr_iSlopenessTH
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF01); //700009CA//AFIT8_bnr_iSlopeBlurStrength   [7:0] AFIT8_bnr_iSlopenessLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //700009CC//AFIT8_bnr_AddNoisePower1   [7:0] AFIT8_bnr_AddNoisePower2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0400); //700009CE//AFIT8_bnr_iRadialTune   [7:0] AFIT8_bnr_iRadialPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x365A); //700009D0//AFIT8_bnr_iRadialLimit [7:0] AFIT8_ee_iFSMagThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x102A); //700009D2//AFIT8_ee_iFSMagThHigh   [7:0] AFIT8_ee_iFSVarThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //700009D4//AFIT8_ee_iFSVarThHigh   [7:0] AFIT8_ee_iFSThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0600); //700009D6//AFIT8_ee_iFSThHigh [7:0] AFIT8_ee_iFSmagPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x5A0F); //700009D8//AFIT8_ee_iFSVarCountTh [7:0] AFIT8_ee_iRadialLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0505); //700009DA//AFIT8_ee_iRadialPower   [7:0] AFIT8_ee_iSmoothEdgeSlope
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1802); //700009DC//AFIT8_ee_iROADThres   [7:0] AFIT8_ee_iROADMaxNR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //700009DE//AFIT8_ee_iROADSubMaxNR [7:0] AFIT8_ee_iROADSubThres
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2006); //700009E0//AFIT8_ee_iROADNeiThres [7:0] AFIT8_ee_iROADNeiMaxNR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3028); //700009E2//AFIT8_ee_iSmoothEdgeThres   [7:0] AFIT8_ee_iMSharpen
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0418); //700009E4//AFIT8_ee_iWSharpen [7:0] AFIT8_ee_iMShThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101); //700009E6//AFIT8_ee_iWShThresh   [7:0] AFIT8_ee_iReduceNegative
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0800); //700009E8//AFIT8_ee_iEmbossCentAdd   [7:0] AFIT8_ee_iShDespeckle
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1804); //700009EA//AFIT8_ee_iReduceEdgeThresh [7:0] AFIT8_dmsc_iEnhThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4008); //700009EC//AFIT8_dmsc_iDesatThresh   [7:0] AFIT8_dmsc_iDemBlurHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0540); //700009EE//AFIT8_dmsc_iDemBlurLow [7:0] AFIT8_dmsc_iDemBlurRange
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8006); //700009F0//AFIT8_dmsc_iDecisionThresh [7:0] AFIT8_dmsc_iCentGrad
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0020); //700009F2//AFIT8_dmsc_iMonochrom   [7:0] AFIT8_dmsc_iGBDenoiseVal
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //700009F4//AFIT8_dmsc_iGRDenoiseVal   [7:0] AFIT8_dmsc_iEdgeDesatThrHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1800); //700009F6//AFIT8_dmsc_iEdgeDesatThrLow   [7:0] AFIT8_dmsc_iEdgeDesat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //700009F8//AFIT8_dmsc_iNearGrayDesat   [7:0] AFIT8_dmsc_iEdgeDesatLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1E10); //700009FA//AFIT8_postdmsc_iBCoeff [7:0] AFIT8_postdmsc_iGCoeff
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //700009FC//AFIT8_postdmsc_iWideMult   [7:0] AFIT8_yuvemix_mNegSlopes_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0607); //700009FE//AFIT8_yuvemix_mNegSlopes_1 [7:0] AFIT8_yuvemix_mNegSlopes_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0005); //70000A00//AFIT8_yuvemix_mNegSlopes_3 [7:0] AFIT8_yuvemix_mPosSlopes_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0607); //70000A02//AFIT8_yuvemix_mPosSlopes_1 [7:0] AFIT8_yuvemix_mPosSlopes_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0405); //70000A04//AFIT8_yuvemix_mPosSlopes_3 [7:0] AFIT8_yuviirnr_iXSupportY
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0205); //70000A06//AFIT8_yuviirnr_iXSupportUV [7:0] AFIT8_yuviirnr_iLowYNorm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0304); //70000A08//AFIT8_yuviirnr_iHighYNorm   [7:0] AFIT8_yuviirnr_iLowUVNorm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0409); //70000A0A//AFIT8_yuviirnr_iHighUVNorm [7:0] AFIT8_yuviirnr_iYNormShift
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0306); //70000A0C//AFIT8_yuviirnr_iUVNormShift   [7:0] AFIT8_yuviirnr_iVertLength_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0407); //70000A0E//AFIT8_yuviirnr_iVertLength_UV   [7:0] AFIT8_yuviirnr_iDiffThreshL_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1C04); //70000A10//AFIT8_yuviirnr_iDiffThreshH_Y   [7:0] AFIT8_yuviirnr_iDiffThreshL_UV
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0214); //70000A12//AFIT8_yuviirnr_iDiffThreshH_UV [7:0] AFIT8_yuviirnr_iMaxThreshL_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1002); //70000A14//AFIT8_yuviirnr_iMaxThreshH_Y   [7:0] AFIT8_yuviirnr_iMaxThreshL_UV
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0610); //70000A16//AFIT8_yuviirnr_iMaxThreshH_UV   [7:0] AFIT8_yuviirnr_iYNRStrengthL
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1A02); //70000A18//AFIT8_yuviirnr_iYNRStrengthH   [7:0] AFIT8_yuviirnr_iUVNRStrengthL
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4A18); //70000A1A//AFIT8_yuviirnr_iUVNRStrengthH   [7:0] AFIT8_byr_gras_iShadingPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //70000A1C//AFIT8_RGBGamma2_iLinearity [7:0] AFIT8_RGBGamma2_iDarkReduce
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0348); //70000A1E//AFIT8_ccm_oscar_iSaturation   [7:0] AFIT8_RGB2YUV_iYOffset
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0180); //70000A20//AFIT8_RGB2YUV_iRGBGain [7:0] AFIT8_bnr_nClustLevel_H
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A0A); //70000A22//AFIT8_bnr_iClustMulT_H [7:0] AFIT8_bnr_iClustMulT_C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101); //70000A24//AFIT8_bnr_iClustThresh_H   [7:0] AFIT8_bnr_iClustThresh_C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2A36); //70000A26//AFIT8_bnr_iDenThreshLow   [7:0] AFIT8_bnr_iDenThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x6024); //70000A28//AFIT8_ee_iLowSharpPower   [7:0] AFIT8_ee_iHighSharpPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2A36); //70000A2A//AFIT8_ee_iLowShDenoise [7:0] AFIT8_ee_iHighShDenoise
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFFF); //70000A2C//AFIT8_ee_iLowSharpClamp   [7:0] AFIT8_ee_iHighSharpClamp
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0808); //70000A2E//AFIT8_ee_iReduceEdgeMinMult   [7:0] AFIT8_ee_iReduceEdgeSlope
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A01); //70000A30//AFIT8_bnr_nClustLevel_H_Bin   [7:0] AFIT8_bnr_iClustMulT_H_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x010A); //70000A32//AFIT8_bnr_iClustMulT_C_Bin [7:0] AFIT8_bnr_iClustThresh_H_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3601); //70000A34//AFIT8_bnr_iClustThresh_C_Bin   [7:0] AFIT8_bnr_iDenThreshLow_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x242A); //70000A36//AFIT8_bnr_iDenThreshHigh_Bin   [7:0] AFIT8_ee_iLowSharpPower_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3660); //70000A38//AFIT8_ee_iHighSharpPower_Bin   [7:0] AFIT8_ee_iLowShDenoise_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF2A); //70000A3A//AFIT8_ee_iHighShDenoise_Bin   [7:0] AFIT8_ee_iLowSharpClamp_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x08FF); //70000A3C//AFIT8_ee_iHighSharpClamp_Bin   [7:0] AFIT8_ee_iReduceEdgeMinMult_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0008); //70000A3E//AFIT8_ee_iReduceEdgeSlope_Bin [7:0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //70000A40//AFITB_bnr_nClustLevel_C    [0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000A42//AFIT16_BRIGHTNESS
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000A44//AFIT16_CONTRAST
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000A46//AFIT16_SATURATION
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000A48//AFIT16_SHARP_BLUR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000A4A//AFIT16_GLAMOUR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //70000A4C//AFIT16_bnr_edge_high
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0064); //70000A4E//AFIT16_postdmsc_iLowBright
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0384); //70000A50//AFIT16_postdmsc_iHighBright
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0051); //70000A52//AFIT16_postdmsc_iLowSat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4); //70000A54//AFIT16_postdmsc_iHighSat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0070); //70000A56//AFIT16_postdmsc_iTune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040); //70000A58//AFIT16_yuvemix_mNegRanges_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00A0); //70000A5A//AFIT16_yuvemix_mNegRanges_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //70000A5C//AFIT16_yuvemix_mNegRanges_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //70000A5E//AFIT16_yuvemix_mPosRanges_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //70000A60//AFIT16_yuvemix_mPosRanges_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //70000A62//AFIT16_yuvemix_mPosRanges_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1430); //70000A64//AFIT8_bnr_edge_low  [7:0] AFIT8_bnr_repl_thresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0201); //70000A66//AFIT8_bnr_repl_force  [7:0] AFIT8_bnr_iHotThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0204); //70000A68//AFIT8_bnr_iHotThreshLow   [7:0] AFIT8_bnr_iColdThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2404); //70000A6A//AFIT8_bnr_iColdThreshLow   [7:0] AFIT8_bnr_DispTH_Low
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x031B); //70000A6C//AFIT8_bnr_DispTH_High   [7:0] AFIT8_bnr_DISP_Limit_Low
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0103); //70000A6E//AFIT8_bnr_DISP_Limit_High   [7:0] AFIT8_bnr_iDistSigmaMin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1205); //70000A70//AFIT8_bnr_iDistSigmaMax   [7:0] AFIT8_bnr_iDiffSigmaLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x400D); //70000A72//AFIT8_bnr_iDiffSigmaHigh   [7:0] AFIT8_bnr_iNormalizedSTD_TH
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //70000A74//AFIT8_bnr_iNormalizedSTD_Limit [7:0] AFIT8_bnr_iDirNRTune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2080); //70000A76//AFIT8_bnr_iDirMinThres [7:0] AFIT8_bnr_iDirFltDiffThresHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3040); //70000A78//AFIT8_bnr_iDirFltDiffThresLow   [7:0] AFIT8_bnr_iDirSmoothPowerHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0630); //70000A7A//AFIT8_bnr_iDirSmoothPowerLow   [7:0] AFIT8_bnr_iLowMaxSlopeAllowed
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0306); //70000A7C//AFIT8_bnr_iHighMaxSlopeAllowed [7:0] AFIT8_bnr_iLowSlopeThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2003); //70000A7E//AFIT8_bnr_iHighSlopeThresh [7:0] AFIT8_bnr_iSlopenessTH
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF01); //70000A80//AFIT8_bnr_iSlopeBlurStrength   [7:0] AFIT8_bnr_iSlopenessLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0404); //70000A82//AFIT8_bnr_AddNoisePower1   [7:0] AFIT8_bnr_AddNoisePower2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0300); //70000A84//AFIT8_bnr_iRadialTune   [7:0] AFIT8_bnr_iRadialPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x245A); //70000A86//AFIT8_bnr_iRadialLimit [7:0] AFIT8_ee_iFSMagThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1018); //70000A88//AFIT8_ee_iFSMagThHigh   [7:0] AFIT8_ee_iFSVarThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //70000A8A//AFIT8_ee_iFSVarThHigh   [7:0] AFIT8_ee_iFSThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0B00); //70000A8C//AFIT8_ee_iFSThHigh [7:0] AFIT8_ee_iFSmagPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x5A0F); //70000A8E//AFIT8_ee_iFSVarCountTh [7:0] AFIT8_ee_iRadialLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0505); //70000A90//AFIT8_ee_iRadialPower   [7:0] AFIT8_ee_iSmoothEdgeSlope
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1802); //70000A92//AFIT8_ee_iROADThres   [7:0] AFIT8_ee_iROADMaxNR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000A94//AFIT8_ee_iROADSubMaxNR [7:0] AFIT8_ee_iROADSubThres
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2006); //70000A96//AFIT8_ee_iROADNeiThres [7:0] AFIT8_ee_iROADNeiMaxNR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3428); //70000A98//AFIT8_ee_iSmoothEdgeThres   [7:0] AFIT8_ee_iMSharpen
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x041C); //70000A9A//AFIT8_ee_iWSharpen [7:0] AFIT8_ee_iMShThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101); //70000A9C//AFIT8_ee_iWShThresh   [7:0] AFIT8_ee_iReduceNegative
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0800); //70000A9E//AFIT8_ee_iEmbossCentAdd   [7:0] AFIT8_ee_iShDespeckle
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1004); //70000AA0//AFIT8_ee_iReduceEdgeThresh [7:0] AFIT8_dmsc_iEnhThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4008); //70000AA2//AFIT8_dmsc_iDesatThresh   [7:0] AFIT8_dmsc_iDemBlurHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0540); //70000AA4//AFIT8_dmsc_iDemBlurLow [7:0] AFIT8_dmsc_iDemBlurRange
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8006); //70000AA6//AFIT8_dmsc_iDecisionThresh [7:0] AFIT8_dmsc_iCentGrad
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0020); //70000AA8//AFIT8_dmsc_iMonochrom   [7:0] AFIT8_dmsc_iGBDenoiseVal
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000AAA//AFIT8_dmsc_iGRDenoiseVal   [7:0] AFIT8_dmsc_iEdgeDesatThrHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1800); //70000AAC//AFIT8_dmsc_iEdgeDesatThrLow   [7:0] AFIT8_dmsc_iEdgeDesat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000AAE//AFIT8_dmsc_iNearGrayDesat   [7:0] AFIT8_dmsc_iEdgeDesatLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1E10); //70000AB0//AFIT8_postdmsc_iBCoeff [7:0] AFIT8_postdmsc_iGCoeff
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //70000AB2//AFIT8_postdmsc_iWideMult   [7:0] AFIT8_yuvemix_mNegSlopes_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0607); //70000AB4//AFIT8_yuvemix_mNegSlopes_1 [7:0] AFIT8_yuvemix_mNegSlopes_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0005); //70000AB6//AFIT8_yuvemix_mNegSlopes_3 [7:0] AFIT8_yuvemix_mPosSlopes_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0607); //70000AB8//AFIT8_yuvemix_mPosSlopes_1 [7:0] AFIT8_yuvemix_mPosSlopes_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0405); //70000ABA//AFIT8_yuvemix_mPosSlopes_3 [7:0] AFIT8_yuviirnr_iXSupportY
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0205); //70000ABC//AFIT8_yuviirnr_iXSupportUV [7:0] AFIT8_yuviirnr_iLowYNorm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0304); //70000ABE//AFIT8_yuviirnr_iHighYNorm   [7:0] AFIT8_yuviirnr_iLowUVNorm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0409); //70000AC0//AFIT8_yuviirnr_iHighUVNorm [7:0] AFIT8_yuviirnr_iYNormShift
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0306); //70000AC2//AFIT8_yuviirnr_iUVNormShift   [7:0] AFIT8_yuviirnr_iVertLength_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0407); //70000AC4//AFIT8_yuviirnr_iVertLength_UV   [7:0] AFIT8_yuviirnr_iDiffThreshL_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1F04); //70000AC6//AFIT8_yuviirnr_iDiffThreshH_Y   [7:0] AFIT8_yuviirnr_iDiffThreshL_UV
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0218); //70000AC8//AFIT8_yuviirnr_iDiffThreshH_UV [7:0] AFIT8_yuviirnr_iMaxThreshL_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1102); //70000ACA//AFIT8_yuviirnr_iMaxThreshH_Y   [7:0] AFIT8_yuviirnr_iMaxThreshL_UV
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0611); //70000ACC//AFIT8_yuviirnr_iMaxThreshH_UV   [7:0] AFIT8_yuviirnr_iYNRStrengthL
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1A02); //70000ACE//AFIT8_yuviirnr_iYNRStrengthH   [7:0] AFIT8_yuviirnr_iUVNRStrengthL
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8018); //70000AD0//AFIT8_yuviirnr_iUVNRStrengthH   [7:0] AFIT8_byr_gras_iShadingPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //70000AD2//AFIT8_RGBGamma2_iLinearity [7:0] AFIT8_RGBGamma2_iDarkReduce
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0380); //70000AD4//AFIT8_ccm_oscar_iSaturation   [7:0] AFIT8_RGB2YUV_iYOffset
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0180); //70000AD6//AFIT8_RGB2YUV_iRGBGain [7:0] AFIT8_bnr_nClustLevel_H
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A0A); //70000AD8//AFIT8_bnr_iClustMulT_H [7:0] AFIT8_bnr_iClustMulT_C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101); //70000ADA//AFIT8_bnr_iClustThresh_H   [7:0] AFIT8_bnr_iClustThresh_C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1B24); //70000ADC//AFIT8_bnr_iDenThreshLow   [7:0] AFIT8_bnr_iDenThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x6024); //70000ADE//AFIT8_ee_iLowSharpPower   [7:0] AFIT8_ee_iHighSharpPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1D22); //70000AE0//AFIT8_ee_iLowShDenoise [7:0] AFIT8_ee_iHighShDenoise
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFFF); //70000AE2//AFIT8_ee_iLowSharpClamp   [7:0] AFIT8_ee_iHighSharpClamp
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0808); //70000AE4//AFIT8_ee_iReduceEdgeMinMult   [7:0] AFIT8_ee_iReduceEdgeSlope
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A01); //70000AE6//AFIT8_bnr_nClustLevel_H_Bin   [7:0] AFIT8_bnr_iClustMulT_H_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x010A); //70000AE8//AFIT8_bnr_iClustMulT_C_Bin [7:0] AFIT8_bnr_iClustThresh_H_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2401); //70000AEA//AFIT8_bnr_iClustThresh_C_Bin   [7:0] AFIT8_bnr_iDenThreshLow_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x241B); //70000AEC//AFIT8_bnr_iDenThreshHigh_Bin   [7:0] AFIT8_ee_iLowSharpPower_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1E60); //70000AEE//AFIT8_ee_iHighSharpPower_Bin   [7:0] AFIT8_ee_iLowShDenoise_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF18); //70000AF0//AFIT8_ee_iHighShDenoise_Bin   [7:0] AFIT8_ee_iLowSharpClamp_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x08FF); //70000AF2//AFIT8_ee_iHighSharpClamp_Bin   [7:0] AFIT8_ee_iReduceEdgeMinMult_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0008); //70000AF4//AFIT8_ee_iReduceEdgeSlope_Bin [7:0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //70000AF6//AFITB_bnr_nClustLevel_C    [0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000AF8//AFIT16_BRIGHTNESS
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000AFA//AFIT16_CONTRAST
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000AFC//AFIT16_SATURATION
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000AFE//AFIT16_SHARP_BLUR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000B00//AFIT16_GLAMOUR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //70000B02//AFIT16_bnr_edge_high
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0064); //70000B04//AFIT16_postdmsc_iLowBright
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0384); //70000B06//AFIT16_postdmsc_iHighBright
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0043); //70000B08//AFIT16_postdmsc_iLowSat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4); //70000B0A//AFIT16_postdmsc_iHighSat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0070); //70000B0C//AFIT16_postdmsc_iTune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040); //70000B0E//AFIT16_yuvemix_mNegRanges_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00A0); //70000B10//AFIT16_yuvemix_mNegRanges_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //70000B12//AFIT16_yuvemix_mNegRanges_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //70000B14//AFIT16_yuvemix_mPosRanges_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //70000B16//AFIT16_yuvemix_mPosRanges_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //70000B18//AFIT16_yuvemix_mPosRanges_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1430); //70000B1A//AFIT8_bnr_edge_low  [7:0] AFIT8_bnr_repl_thresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0201); //70000B1C//AFIT8_bnr_repl_force  [7:0] AFIT8_bnr_iHotThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0204); //70000B1E//AFIT8_bnr_iHotThreshLow   [7:0] AFIT8_bnr_iColdThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1B04); //70000B20//AFIT8_bnr_iColdThreshLow   [7:0] AFIT8_bnr_DispTH_Low
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0312); //70000B22//AFIT8_bnr_DispTH_High   [7:0] AFIT8_bnr_DISP_Limit_Low
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0003); //70000B24//AFIT8_bnr_DISP_Limit_High   [7:0] AFIT8_bnr_iDistSigmaMin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0C03); //70000B26//AFIT8_bnr_iDistSigmaMax   [7:0] AFIT8_bnr_iDiffSigmaLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2806); //70000B28//AFIT8_bnr_iDiffSigmaHigh   [7:0] AFIT8_bnr_iNormalizedSTD_TH
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //70000B2A//AFIT8_bnr_iNormalizedSTD_Limit [7:0] AFIT8_bnr_iDirNRTune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1580); //70000B2C//AFIT8_bnr_iDirMinThres [7:0] AFIT8_bnr_iDirFltDiffThresHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2020); //70000B2E//AFIT8_bnr_iDirFltDiffThresLow   [7:0] AFIT8_bnr_iDirSmoothPowerHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0620); //70000B30//AFIT8_bnr_iDirSmoothPowerLow   [7:0] AFIT8_bnr_iLowMaxSlopeAllowed
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0306); //70000B32//AFIT8_bnr_iHighMaxSlopeAllowed [7:0] AFIT8_bnr_iLowSlopeThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2003); //70000B34//AFIT8_bnr_iHighSlopeThresh [7:0] AFIT8_bnr_iSlopenessTH
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF01); //70000B36//AFIT8_bnr_iSlopeBlurStrength   [7:0] AFIT8_bnr_iSlopenessLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0404); //70000B38//AFIT8_bnr_AddNoisePower1   [7:0] AFIT8_bnr_AddNoisePower2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0300); //70000B3A//AFIT8_bnr_iRadialTune   [7:0] AFIT8_bnr_iRadialPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x145A); //70000B3C//AFIT8_bnr_iRadialLimit [7:0] AFIT8_ee_iFSMagThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1010); //70000B3E//AFIT8_ee_iFSMagThHigh   [7:0] AFIT8_ee_iFSVarThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //70000B40//AFIT8_ee_iFSVarThHigh   [7:0] AFIT8_ee_iFSThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0E00); //70000B42//AFIT8_ee_iFSThHigh [7:0] AFIT8_ee_iFSmagPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x5A0F); //70000B44//AFIT8_ee_iFSVarCountTh [7:0] AFIT8_ee_iRadialLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0504); //70000B46//AFIT8_ee_iRadialPower   [7:0] AFIT8_ee_iSmoothEdgeSlope
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1802); //70000B48//AFIT8_ee_iROADThres   [7:0] AFIT8_ee_iROADMaxNR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000B4A//AFIT8_ee_iROADSubMaxNR [7:0] AFIT8_ee_iROADSubThres
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2006); //70000B4C//AFIT8_ee_iROADNeiThres [7:0] AFIT8_ee_iROADNeiMaxNR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3828); //70000B4E//AFIT8_ee_iSmoothEdgeThres   [7:0] AFIT8_ee_iMSharpen
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0428); //70000B50//AFIT8_ee_iWSharpen [7:0] AFIT8_ee_iMShThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101); //70000B52//AFIT8_ee_iWShThresh   [7:0] AFIT8_ee_iReduceNegative
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8000); //70000B54//AFIT8_ee_iEmbossCentAdd   [7:0] AFIT8_ee_iShDespeckle
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A04); //70000B56//AFIT8_ee_iReduceEdgeThresh [7:0] AFIT8_dmsc_iEnhThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4008); //70000B58//AFIT8_dmsc_iDesatThresh   [7:0] AFIT8_dmsc_iDemBlurHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0540); //70000B5A//AFIT8_dmsc_iDemBlurLow [7:0] AFIT8_dmsc_iDemBlurRange
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8006); //70000B5C//AFIT8_dmsc_iDecisionThresh [7:0] AFIT8_dmsc_iCentGrad
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0020); //70000B5E//AFIT8_dmsc_iMonochrom   [7:0] AFIT8_dmsc_iGBDenoiseVal
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000B60//AFIT8_dmsc_iGRDenoiseVal   [7:0] AFIT8_dmsc_iEdgeDesatThrHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1800); //70000B62//AFIT8_dmsc_iEdgeDesatThrLow   [7:0] AFIT8_dmsc_iEdgeDesat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000B64//AFIT8_dmsc_iNearGrayDesat   [7:0] AFIT8_dmsc_iEdgeDesatLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1E10); //70000B66//AFIT8_postdmsc_iBCoeff [7:0] AFIT8_postdmsc_iGCoeff
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //70000B68//AFIT8_postdmsc_iWideMult   [7:0] AFIT8_yuvemix_mNegSlopes_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0607); //70000B6A//AFIT8_yuvemix_mNegSlopes_1 [7:0] AFIT8_yuvemix_mNegSlopes_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0005); //70000B6C//AFIT8_yuvemix_mNegSlopes_3 [7:0] AFIT8_yuvemix_mPosSlopes_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0607); //70000B6E//AFIT8_yuvemix_mPosSlopes_1 [7:0] AFIT8_yuvemix_mPosSlopes_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0405); //70000B70//AFIT8_yuvemix_mPosSlopes_3 [7:0] AFIT8_yuviirnr_iXSupportY
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0207); //70000B72//AFIT8_yuviirnr_iXSupportUV [7:0] AFIT8_yuviirnr_iLowYNorm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0304); //70000B74//AFIT8_yuviirnr_iHighYNorm   [7:0] AFIT8_yuviirnr_iLowUVNorm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0409); //70000B76//AFIT8_yuviirnr_iHighUVNorm [7:0] AFIT8_yuviirnr_iYNormShift
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0306); //70000B78//AFIT8_yuviirnr_iUVNormShift   [7:0] AFIT8_yuviirnr_iVertLength_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0407); //70000B7A//AFIT8_yuviirnr_iVertLength_UV   [7:0] AFIT8_yuviirnr_iDiffThreshL_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2404); //70000B7C//AFIT8_yuviirnr_iDiffThreshH_Y   [7:0] AFIT8_yuviirnr_iDiffThreshL_UV
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0221); //70000B7E//AFIT8_yuviirnr_iDiffThreshH_UV [7:0] AFIT8_yuviirnr_iMaxThreshL_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1202); //70000B80//AFIT8_yuviirnr_iMaxThreshH_Y   [7:0] AFIT8_yuviirnr_iMaxThreshL_UV
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0613); //70000B82//AFIT8_yuviirnr_iMaxThreshH_UV   [7:0] AFIT8_yuviirnr_iYNRStrengthL
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1A02); //70000B84//AFIT8_yuviirnr_iYNRStrengthH   [7:0] AFIT8_yuviirnr_iUVNRStrengthL
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8018); //70000B86//AFIT8_yuviirnr_iUVNRStrengthH   [7:0] AFIT8_byr_gras_iShadingPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //70000B88//AFIT8_RGBGamma2_iLinearity [7:0] AFIT8_RGBGamma2_iDarkReduce
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //70000B8A//AFIT8_ccm_oscar_iSaturation   [7:0] AFIT8_RGB2YUV_iYOffset
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0180); //70000B8C//AFIT8_RGB2YUV_iRGBGain [7:0] AFIT8_bnr_nClustLevel_H
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A0A); //70000B8E//AFIT8_bnr_iClustMulT_H [7:0] AFIT8_bnr_iClustMulT_C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101); //70000B90//AFIT8_bnr_iClustThresh_H   [7:0] AFIT8_bnr_iClustThresh_C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x141D); //70000B92//AFIT8_bnr_iDenThreshLow   [7:0] AFIT8_bnr_iDenThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x6024); //70000B94//AFIT8_ee_iLowSharpPower   [7:0] AFIT8_ee_iHighSharpPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0C0C); //70000B96//AFIT8_ee_iLowShDenoise [7:0] AFIT8_ee_iHighShDenoise
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFFF); //70000B98//AFIT8_ee_iLowSharpClamp   [7:0] AFIT8_ee_iHighSharpClamp
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0808); //70000B9A//AFIT8_ee_iReduceEdgeMinMult   [7:0] AFIT8_ee_iReduceEdgeSlope
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A01); //70000B9C//AFIT8_bnr_nClustLevel_H_Bin   [7:0] AFIT8_bnr_iClustMulT_H_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x010A); //70000B9E//AFIT8_bnr_iClustMulT_C_Bin [7:0] AFIT8_bnr_iClustThresh_H_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1B01); //70000BA0//AFIT8_bnr_iClustThresh_C_Bin   [7:0] AFIT8_bnr_iDenThreshLow_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2412); //70000BA2//AFIT8_bnr_iDenThreshHigh_Bin   [7:0] AFIT8_ee_iLowSharpPower_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0C60); //70000BA4//AFIT8_ee_iHighSharpPower_Bin   [7:0] AFIT8_ee_iLowShDenoise_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF0C); //70000BA6//AFIT8_ee_iHighShDenoise_Bin   [7:0] AFIT8_ee_iLowSharpClamp_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x08FF); //70000BA8//AFIT8_ee_iHighSharpClamp_Bin   [7:0] AFIT8_ee_iReduceEdgeMinMult_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0008); //70000BAA//AFIT8_ee_iReduceEdgeSlope_Bin [7:0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //70000BAC//AFITB_bnr_nClustLevel_C    [0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000BAE//AFIT16_BRIGHTNESS
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000BB0//AFIT16_CONTRAST
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000BB2//AFIT16_SATURATION
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000BB4//AFIT16_SHARP_BLUR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000BB6//AFIT16_GLAMOUR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //70000BB8//AFIT16_bnr_edge_high
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0064); //70000BBA//AFIT16_postdmsc_iLowBright
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0384); //70000BBC//AFIT16_postdmsc_iHighBright
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0032); //70000BBE//AFIT16_postdmsc_iLowSat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4); //70000BC0//AFIT16_postdmsc_iHighSat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0070); //70000BC2//AFIT16_postdmsc_iTune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040); //70000BC4//AFIT16_yuvemix_mNegRanges_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00A0); //70000BC6//AFIT16_yuvemix_mNegRanges_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //70000BC8//AFIT16_yuvemix_mNegRanges_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //70000BCA//AFIT16_yuvemix_mPosRanges_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //70000BCC//AFIT16_yuvemix_mPosRanges_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //70000BCE//AFIT16_yuvemix_mPosRanges_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1430); //70000BD0//AFIT8_bnr_edge_low  [7:0] AFIT8_bnr_repl_thresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0201); //70000BD2//AFIT8_bnr_repl_force  [7:0] AFIT8_bnr_iHotThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0204); //70000BD4//AFIT8_bnr_iHotThreshLow   [7:0] AFIT8_bnr_iColdThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1504); //70000BD6//AFIT8_bnr_iColdThreshLow   [7:0] AFIT8_bnr_DispTH_Low
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x030F); //70000BD8//AFIT8_bnr_DispTH_High   [7:0] AFIT8_bnr_DISP_Limit_Low
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0003); //70000BDA//AFIT8_bnr_DISP_Limit_High   [7:0] AFIT8_bnr_iDistSigmaMin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0902); //70000BDC//AFIT8_bnr_iDistSigmaMax   [7:0] AFIT8_bnr_iDiffSigmaLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2004); //70000BDE//AFIT8_bnr_iDiffSigmaHigh   [7:0] AFIT8_bnr_iNormalizedSTD_TH
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0050); //70000BE0//AFIT8_bnr_iNormalizedSTD_Limit [7:0] AFIT8_bnr_iDirNRTune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1140); //70000BE2//AFIT8_bnr_iDirMinThres [7:0] AFIT8_bnr_iDirFltDiffThresHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x201C); //70000BE4//AFIT8_bnr_iDirFltDiffThresLow   [7:0] AFIT8_bnr_iDirSmoothPowerHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0620); //70000BE6//AFIT8_bnr_iDirSmoothPowerLow   [7:0] AFIT8_bnr_iLowMaxSlopeAllowed
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0306); //70000BE8//AFIT8_bnr_iHighMaxSlopeAllowed [7:0] AFIT8_bnr_iLowSlopeThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2003); //70000BEA//AFIT8_bnr_iHighSlopeThresh [7:0] AFIT8_bnr_iSlopenessTH
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF01); //70000BEC//AFIT8_bnr_iSlopeBlurStrength   [7:0] AFIT8_bnr_iSlopenessLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0404); //70000BEE//AFIT8_bnr_AddNoisePower1   [7:0] AFIT8_bnr_AddNoisePower2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0300); //70000BF0//AFIT8_bnr_iRadialTune   [7:0] AFIT8_bnr_iRadialPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x145A); //70000BF2//AFIT8_bnr_iRadialLimit [7:0] AFIT8_ee_iFSMagThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1010); //70000BF4//AFIT8_ee_iFSMagThHigh   [7:0] AFIT8_ee_iFSVarThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //70000BF6//AFIT8_ee_iFSVarThHigh   [7:0] AFIT8_ee_iFSThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1000); //70000BF8//AFIT8_ee_iFSThHigh [7:0] AFIT8_ee_iFSmagPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x5A0F); //70000BFA//AFIT8_ee_iFSVarCountTh [7:0] AFIT8_ee_iRadialLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0503); //70000BFC//AFIT8_ee_iRadialPower   [7:0] AFIT8_ee_iSmoothEdgeSlope
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1802); //70000BFE//AFIT8_ee_iROADThres   [7:0] AFIT8_ee_iROADMaxNR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000C00//AFIT8_ee_iROADSubMaxNR [7:0] AFIT8_ee_iROADSubThres
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2006); //70000C02//AFIT8_ee_iROADNeiThres [7:0] AFIT8_ee_iROADNeiMaxNR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x3C28); //70000C04//AFIT8_ee_iSmoothEdgeThres   [7:0] AFIT8_ee_iMSharpen
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x042C); //70000C06//AFIT8_ee_iWSharpen [7:0] AFIT8_ee_iMShThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101); //70000C08//AFIT8_ee_iWShThresh   [7:0] AFIT8_ee_iReduceNegative
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF00); //70000C0A//AFIT8_ee_iEmbossCentAdd   [7:0] AFIT8_ee_iShDespeckle
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0904); //70000C0C//AFIT8_ee_iReduceEdgeThresh [7:0] AFIT8_dmsc_iEnhThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4008); //70000C0E//AFIT8_dmsc_iDesatThresh   [7:0] AFIT8_dmsc_iDemBlurHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0540); //70000C10//AFIT8_dmsc_iDemBlurLow [7:0] AFIT8_dmsc_iDemBlurRange
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8006); //70000C12//AFIT8_dmsc_iDecisionThresh [7:0] AFIT8_dmsc_iCentGrad
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0020); //70000C14//AFIT8_dmsc_iMonochrom   [7:0] AFIT8_dmsc_iGBDenoiseVal
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000C16//AFIT8_dmsc_iGRDenoiseVal   [7:0] AFIT8_dmsc_iEdgeDesatThrHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1800); //70000C18//AFIT8_dmsc_iEdgeDesatThrLow   [7:0] AFIT8_dmsc_iEdgeDesat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000C1A//AFIT8_dmsc_iNearGrayDesat   [7:0] AFIT8_dmsc_iEdgeDesatLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1E10); //70000C1C//AFIT8_postdmsc_iBCoeff [7:0] AFIT8_postdmsc_iGCoeff
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //70000C1E//AFIT8_postdmsc_iWideMult   [7:0] AFIT8_yuvemix_mNegSlopes_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0607); //70000C20//AFIT8_yuvemix_mNegSlopes_1 [7:0] AFIT8_yuvemix_mNegSlopes_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0005); //70000C22//AFIT8_yuvemix_mNegSlopes_3 [7:0] AFIT8_yuvemix_mPosSlopes_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0607); //70000C24//AFIT8_yuvemix_mPosSlopes_1 [7:0] AFIT8_yuvemix_mPosSlopes_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0405); //70000C26//AFIT8_yuvemix_mPosSlopes_3 [7:0] AFIT8_yuviirnr_iXSupportY
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0206); //70000C28//AFIT8_yuviirnr_iXSupportUV [7:0] AFIT8_yuviirnr_iLowYNorm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0304); //70000C2A//AFIT8_yuviirnr_iHighYNorm   [7:0] AFIT8_yuviirnr_iLowUVNorm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0409); //70000C2C//AFIT8_yuviirnr_iHighUVNorm [7:0] AFIT8_yuviirnr_iYNormShift
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0305); //70000C2E//AFIT8_yuviirnr_iUVNormShift   [7:0] AFIT8_yuviirnr_iVertLength_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0406); //70000C30//AFIT8_yuviirnr_iVertLength_UV   [7:0] AFIT8_yuviirnr_iDiffThreshL_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2804); //70000C32//AFIT8_yuviirnr_iDiffThreshH_Y   [7:0] AFIT8_yuviirnr_iDiffThreshL_UV
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0228); //70000C34//AFIT8_yuviirnr_iDiffThreshH_UV [7:0] AFIT8_yuviirnr_iMaxThreshL_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1402); //70000C36//AFIT8_yuviirnr_iMaxThreshH_Y   [7:0] AFIT8_yuviirnr_iMaxThreshL_UV
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0618); //70000C38//AFIT8_yuviirnr_iMaxThreshH_UV   [7:0] AFIT8_yuviirnr_iYNRStrengthL
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1A02); //70000C3A//AFIT8_yuviirnr_iYNRStrengthH   [7:0] AFIT8_yuviirnr_iUVNRStrengthL
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8018); //70000C3C//AFIT8_yuviirnr_iUVNRStrengthH   [7:0] AFIT8_byr_gras_iShadingPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //70000C3E//AFIT8_RGBGamma2_iLinearity [7:0] AFIT8_RGBGamma2_iDarkReduce
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //70000C40//AFIT8_ccm_oscar_iSaturation   [7:0] AFIT8_RGB2YUV_iYOffset
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0180); //70000C42//AFIT8_RGB2YUV_iRGBGain [7:0] AFIT8_bnr_nClustLevel_H
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A0A); //70000C44//AFIT8_bnr_iClustMulT_H [7:0] AFIT8_bnr_iClustMulT_C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101); //70000C46//AFIT8_bnr_iClustThresh_H   [7:0] AFIT8_bnr_iClustThresh_C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1117); //70000C48//AFIT8_bnr_iDenThreshLow   [7:0] AFIT8_bnr_iDenThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x6024); //70000C4A//AFIT8_ee_iLowSharpPower   [7:0] AFIT8_ee_iHighSharpPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A0A); //70000C4C//AFIT8_ee_iLowShDenoise [7:0] AFIT8_ee_iHighShDenoise
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFFF); //70000C4E//AFIT8_ee_iLowSharpClamp   [7:0] AFIT8_ee_iHighSharpClamp
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0808); //70000C50//AFIT8_ee_iReduceEdgeMinMult   [7:0] AFIT8_ee_iReduceEdgeSlope
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A01); //70000C52//AFIT8_bnr_nClustLevel_H_Bin   [7:0] AFIT8_bnr_iClustMulT_H_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x010A); //70000C54//AFIT8_bnr_iClustMulT_C_Bin [7:0] AFIT8_bnr_iClustThresh_H_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1501); //70000C56//AFIT8_bnr_iClustThresh_C_Bin   [7:0] AFIT8_bnr_iDenThreshLow_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x240F); //70000C58//AFIT8_bnr_iDenThreshHigh_Bin   [7:0] AFIT8_ee_iLowSharpPower_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A60); //70000C5A//AFIT8_ee_iHighSharpPower_Bin   [7:0] AFIT8_ee_iLowShDenoise_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF0A); //70000C5C//AFIT8_ee_iHighShDenoise_Bin   [7:0] AFIT8_ee_iLowSharpClamp_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x08FF); //70000C5E//AFIT8_ee_iHighSharpClamp_Bin   [7:0] AFIT8_ee_iReduceEdgeMinMult_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0008); //70000C60//AFIT8_ee_iReduceEdgeSlope_Bin [7:0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //70000C62//AFITB_bnr_nClustLevel_C    [0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000C64//AFIT16_BRIGHTNESS
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000C66//AFIT16_CONTRAST
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000C68//AFIT16_SATURATION
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000C6A//AFIT16_SHARP_BLUR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000C6C//AFIT16_GLAMOUR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00C0); //70000C6E//AFIT16_bnr_edge_high
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0064); //70000C70//AFIT16_postdmsc_iLowBright
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0384); //70000C72//AFIT16_postdmsc_iHighBright
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0032); //70000C74//AFIT16_postdmsc_iLowSat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x01F4); //70000C76//AFIT16_postdmsc_iHighSat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0070); //70000C78//AFIT16_postdmsc_iTune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040); //70000C7A//AFIT16_yuvemix_mNegRanges_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00A0); //70000C7C//AFIT16_yuvemix_mNegRanges_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //70000C7E//AFIT16_yuvemix_mNegRanges_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //70000C80//AFIT16_yuvemix_mPosRanges_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //70000C82//AFIT16_yuvemix_mPosRanges_1
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100); //70000C84//AFIT16_yuvemix_mPosRanges_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1430); //70000C86//AFIT8_bnr_edge_low  [7:0] AFIT8_bnr_repl_thresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0201); //70000C88//AFIT8_bnr_repl_force  [7:0] AFIT8_bnr_iHotThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0204); //70000C8A//AFIT8_bnr_iHotThreshLow   [7:0] AFIT8_bnr_iColdThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F04); //70000C8C//AFIT8_bnr_iColdThreshLow   [7:0] AFIT8_bnr_DispTH_Low
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x030C); //70000C8E//AFIT8_bnr_DispTH_High   [7:0] AFIT8_bnr_DISP_Limit_Low
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0003); //70000C90//AFIT8_bnr_DISP_Limit_High   [7:0] AFIT8_bnr_iDistSigmaMin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0602); //70000C92//AFIT8_bnr_iDistSigmaMax   [7:0] AFIT8_bnr_iDiffSigmaLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1803); //70000C94//AFIT8_bnr_iDiffSigmaHigh   [7:0] AFIT8_bnr_iNormalizedSTD_TH
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0040); //70000C96//AFIT8_bnr_iNormalizedSTD_Limit [7:0] AFIT8_bnr_iDirNRTune
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0E20); //70000C98//AFIT8_bnr_iDirMinThres [7:0] AFIT8_bnr_iDirFltDiffThresHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2018); //70000C9A//AFIT8_bnr_iDirFltDiffThresLow   [7:0] AFIT8_bnr_iDirSmoothPowerHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0620); //70000C9C//AFIT8_bnr_iDirSmoothPowerLow   [7:0] AFIT8_bnr_iLowMaxSlopeAllowed
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0306); //70000C9E//AFIT8_bnr_iHighMaxSlopeAllowed [7:0] AFIT8_bnr_iLowSlopeThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2003); //70000CA0//AFIT8_bnr_iHighSlopeThresh [7:0] AFIT8_bnr_iSlopenessTH
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF01); //70000CA2//AFIT8_bnr_iSlopeBlurStrength   [7:0] AFIT8_bnr_iSlopenessLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0404); //70000CA4//AFIT8_bnr_AddNoisePower1   [7:0] AFIT8_bnr_AddNoisePower2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0200); //70000CA6//AFIT8_bnr_iRadialTune   [7:0] AFIT8_bnr_iRadialPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x145A); //70000CA8//AFIT8_bnr_iRadialLimit [7:0] AFIT8_ee_iFSMagThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1010); //70000CAA//AFIT8_ee_iFSMagThHigh   [7:0] AFIT8_ee_iFSVarThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //70000CAC//AFIT8_ee_iFSVarThHigh   [7:0] AFIT8_ee_iFSThLow
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1200); //70000CAE//AFIT8_ee_iFSThHigh [7:0] AFIT8_ee_iFSmagPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x5A0F); //70000CB0//AFIT8_ee_iFSVarCountTh [7:0] AFIT8_ee_iRadialLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0502); //70000CB2//AFIT8_ee_iRadialPower   [7:0] AFIT8_ee_iSmoothEdgeSlope
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1802); //70000CB4//AFIT8_ee_iROADThres   [7:0] AFIT8_ee_iROADMaxNR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000CB6//AFIT8_ee_iROADSubMaxNR [7:0] AFIT8_ee_iROADSubThres
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2006); //70000CB8//AFIT8_ee_iROADNeiThres [7:0] AFIT8_ee_iROADNeiMaxNR
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4028); //70000CBA//AFIT8_ee_iSmoothEdgeThres   [7:0] AFIT8_ee_iMSharpen
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0430); //70000CBC//AFIT8_ee_iWSharpen [7:0] AFIT8_ee_iMShThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101); //70000CBE//AFIT8_ee_iWShThresh   [7:0] AFIT8_ee_iReduceNegative
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF00); //70000CC0//AFIT8_ee_iEmbossCentAdd   [7:0] AFIT8_ee_iShDespeckle
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0804); //70000CC2//AFIT8_ee_iReduceEdgeThresh [7:0] AFIT8_dmsc_iEnhThresh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x4008); //70000CC4//AFIT8_dmsc_iDesatThresh   [7:0] AFIT8_dmsc_iDemBlurHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0540); //70000CC6//AFIT8_dmsc_iDemBlurLow [7:0] AFIT8_dmsc_iDemBlurRange
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8006); //70000CC8//AFIT8_dmsc_iDecisionThresh [7:0] AFIT8_dmsc_iCentGrad
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0020); //70000CCA//AFIT8_dmsc_iMonochrom   [7:0] AFIT8_dmsc_iGBDenoiseVal
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000CCC//AFIT8_dmsc_iGRDenoiseVal   [7:0] AFIT8_dmsc_iEdgeDesatThrHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1800); //70000CCE//AFIT8_dmsc_iEdgeDesatThrLow   [7:0] AFIT8_dmsc_iEdgeDesat
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000CD0//AFIT8_dmsc_iNearGrayDesat   [7:0] AFIT8_dmsc_iEdgeDesatLimit
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1E10); //70000CD2//AFIT8_postdmsc_iBCoeff [7:0] AFIT8_postdmsc_iGCoeff
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000B); //70000CD4//AFIT8_postdmsc_iWideMult   [7:0] AFIT8_yuvemix_mNegSlopes_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0607); //70000CD6//AFIT8_yuvemix_mNegSlopes_1 [7:0] AFIT8_yuvemix_mNegSlopes_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0005); //70000CD8//AFIT8_yuvemix_mNegSlopes_3 [7:0] AFIT8_yuvemix_mPosSlopes_0
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0607); //70000CDA//AFIT8_yuvemix_mPosSlopes_1 [7:0] AFIT8_yuvemix_mPosSlopes_2
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0405); //70000CDC//AFIT8_yuvemix_mPosSlopes_3 [7:0] AFIT8_yuviirnr_iXSupportY
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0205); //70000CDE//AFIT8_yuviirnr_iXSupportUV [7:0] AFIT8_yuviirnr_iLowYNorm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0304); //70000CE0//AFIT8_yuviirnr_iHighYNorm   [7:0] AFIT8_yuviirnr_iLowUVNorm
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0409); //70000CE2//AFIT8_yuviirnr_iHighUVNorm [7:0] AFIT8_yuviirnr_iYNormShift
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0306); //70000CE4//AFIT8_yuviirnr_iUVNormShift   [7:0] AFIT8_yuviirnr_iVertLength_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0407); //70000CE6//AFIT8_yuviirnr_iVertLength_UV   [7:0] AFIT8_yuviirnr_iDiffThreshL_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x2C04); //70000CE8//AFIT8_yuviirnr_iDiffThreshH_Y   [7:0] AFIT8_yuviirnr_iDiffThreshL_UV
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x022C); //70000CEA//AFIT8_yuviirnr_iDiffThreshH_UV [7:0] AFIT8_yuviirnr_iMaxThreshL_Y
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1402); //70000CEC//AFIT8_yuviirnr_iMaxThreshH_Y   [7:0] AFIT8_yuviirnr_iMaxThreshL_UV
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0618); //70000CEE//AFIT8_yuviirnr_iMaxThreshH_UV   [7:0] AFIT8_yuviirnr_iYNRStrengthL
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x1A02); //70000CF0//AFIT8_yuviirnr_iYNRStrengthH   [7:0] AFIT8_yuviirnr_iUVNRStrengthL
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x8018); //70000CF2//AFIT8_yuviirnr_iUVNRStrengthH   [7:0] AFIT8_byr_gras_iShadingPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //70000CF4//AFIT8_RGBGamma2_iLinearity [7:0] AFIT8_RGBGamma2_iDarkReduce
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0080); //70000CF6//AFIT8_ccm_oscar_iSaturation   [7:0] AFIT8_RGB2YUV_iYOffset
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0180); //70000CF8//AFIT8_RGB2YUV_iRGBGain [7:0] AFIT8_bnr_nClustLevel_H
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A0A); //70000CFA//AFIT8_bnr_iClustMulT_H [7:0] AFIT8_bnr_iClustMulT_C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0101); //70000CFC//AFIT8_bnr_iClustThresh_H   [7:0] AFIT8_bnr_iClustThresh_C
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0C0F); //70000CFE//AFIT8_bnr_iDenThreshLow   [7:0] AFIT8_bnr_iDenThreshHigh
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x6024); //70000D00//AFIT8_ee_iLowSharpPower   [7:0] AFIT8_ee_iHighSharpPower
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0808); //70000D02//AFIT8_ee_iLowShDenoise [7:0] AFIT8_ee_iHighShDenoise
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFFFF); //70000D04//AFIT8_ee_iLowSharpClamp   [7:0] AFIT8_ee_iHighSharpClamp
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0808); //70000D06//AFIT8_ee_iReduceEdgeMinMult   [7:0] AFIT8_ee_iReduceEdgeSlope
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A01); //70000D08//AFIT8_bnr_nClustLevel_H_Bin   [7:0] AFIT8_bnr_iClustMulT_H_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x010A); //70000D0A//AFIT8_bnr_iClustMulT_C_Bin [7:0] AFIT8_bnr_iClustThresh_H_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0F01); //70000D0C//AFIT8_bnr_iClustThresh_C_Bin   [7:0] AFIT8_bnr_iDenThreshLow_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x240C); //70000D0E//AFIT8_bnr_iDenThreshHigh_Bin   [7:0] AFIT8_ee_iLowSharpPower_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0860); //70000D10//AFIT8_ee_iHighSharpPower_Bin   [7:0] AFIT8_ee_iLowShDenoise_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFF08); //70000D12//AFIT8_ee_iHighShDenoise_Bin   [7:0] AFIT8_ee_iLowSharpClamp_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x08FF); //70000D14//AFIT8_ee_iHighSharpClamp_Bin   [7:0] AFIT8_ee_iReduceEdgeMinMult_Bin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0008); //70000D16//AFIT8_ee_iReduceEdgeSlope_Bin [7:0]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //70000D18 AFITB_bnr_nClustLevel_C    [0]   bWideWide[1]
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x23CE); //70000D19//ConstAfitBaseVals
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFDC8); //70000D1A//ConstAfitBaseVals
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x112E); //70000D1B//ConstAfitBaseVals
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x93A5); //70000D1C//ConstAfitBaseVals
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0xFE67); //70000D1D//ConstAfitBaseVals
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //70000D1E//ConstAfitBaseVals
    //==================================================================================
    // 18.JPEG Thumnail Setting
    //==================================================================================

    //s002A0478
    //s0F12005F //REG_TC_BRC_usPrevQuality
    //s0F12005F //REG_TC_BRC_usCaptureQuality
    //s0F120001 //REG_TC_THUMB_Thumb_bActive
    //s0F120280 //REG_TC_THUMB_Thumb_uWidth
    //s0F1201E0 //REG_TC_THUMB_Thumb_uHeight
    //s0F120005 //REG_TC_THUMB_Thumb_Format
    //s002A17DC
    //s0F120054 //jpeg_ManualMBCV
    //s002A1AE4
    //s0F12001C //senHal_bExtraAddLine
    //s002A0284
    //s0F120001 //REG_TC_GP_bBypassScalerJpg
    //s002A028A
    //s0F120000 //REG_TC_GP_bUse1FrameCaptureMode
    //s002A1CC2 //DRx_uDRxWeight for AutoCont function
    //s0F120100
    //s0F120100
    //s0F120100
    //s0F120100

    S5K4ECGX_write_cmos_sensor(0x002A, 0x0EE2);      //System Setting
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0200);      //0x5DC0

    //==================================================================================
    // 10.Clock Setting
    //==================================================================================
    //For MCLK=24MHz, PCLK=5DC0
    S5K4ECGX_write_cmos_sensor(0x002A, 0x01F8);      //System Setting
    S5K4ECGX_write_cmos_sensor(0x0F12, (S5K4ECGX_MCLK*1000)); //0x5DC0
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0212);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //0x212:REG_TC_IPRM_UseNPviClocks
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0002);   //0x214:REG_TC_IPRM_UseNMipiClocks
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0002);   //0x216:REG_TC_IPRM_NumberOfMipiLanes

    S5K4ECGX_write_cmos_sensor(0x002A, 0x021A);
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK0_SYS_OP_RATE);  //0x4F1A//0x3A98//0x32c8);
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK0_MAX); //REG_TC_IPRM_MinOutRate4KHz_0 PCLK Min : 81Mhz
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK0_MIN); //REG_TC_IPRM_MaxOutRate4KHz_0 PCLK Max : 81Mhz

    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK1_SYS_OP_RATE);  //0x32c8//0x4F1A;
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK1_MIN); //REG_TC_IPRM_MinOutRate4KHz_1 PCLK Min : 81Mhz
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK1_MAX); //REG_TC_IPRM_MaxOutRate4KHz_1 PCLK Max : 81Mhz

    //==================================================================================
    // 19.Input Size Setting
    //==================================================================================
    //Input Size
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0250);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00); //REG_TC_GP_PrevReqInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0780); //REG_TC_GP_PrevReqInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //REG_TC_GP_PrevInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000C); //REG_TC_GP_PrevInputHeightOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00); //REG_TC_GP_CapReqInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0780); //REG_TC_GP_CapReqInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //REG_TC_GP_CapInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000C); //REG_TC_GP_CapInputHeightOfs
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0494);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00); //REG_TC_PZOOM_ZoomInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0780); //REG_TC_PZOOM_ZoomInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_PZOOM_ZoomInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_PZOOM_ZoomInputHeightOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00); //REG_TC_CZOOM_ZoomInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0780); //REG_TC_CZOOM_ZoomInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_CZOOM_ZoomInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_CZOOM_ZoomInputHeightOfs
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0262);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //REG_TC_GP_bUseReqInputInPre
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //REG_TC_GP_bUseReqInputInCap


    //Preview config[0]: normal mode
    //91MHz, 1280x960, Dynamic night mode 30~7.5fps
    S5K4ECGX_write_cmos_sensor(0x002A, 0x02A6);   //Configuration Setting//Normal mode(VGA preview 30~15fps)
    S5K4ECGX_write_cmos_sensor(0x0F12, S5K4ECGX_IMAGE_SENSOR_PV_WIDTH_DRV);   //REG_0TC_PCFG_usWidth: 1280
    S5K4ECGX_write_cmos_sensor(0x0F12, S5K4ECGX_IMAGE_SENSOR_PV_HEIGHT_DRV);   //REG_0TC_PCFG_usHeight: 960
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0005);   //REG_0TC_PCFG_Format 5 YUV 7 Raw 9 JPG
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK0_MAX);  //REG_0TC_PCFG_usMaxOut4KHzRate PCLK Min : xxMhz
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK0_MIN);  //REG_0TC_PCFG_usMinOut4KHzRate PCLK Max : xxMhz
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0100);   //REG_0TC_PCFG_OutClkPerPix88   0x100: 256
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0300);   //REG_0TC_PCFG_uBpp88
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0002);   //REG_0TC_PCFG_PVIMask
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_0TC_PCFG_OIFMask
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x01E0);   //REG_0TC_PCFG_usJpegPacketSize
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_0TC_PCFG_usJpegTotalPackets
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_0TC_PCFG_uClockInd
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_0TC_PCFG_usFrTimeType
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001);   //REG_0TC_PCFG_FrRateQualityType
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0535); //REG_0TC_PCFG_usMaxFrTimeMsecMult10
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x014A); //REG_0TC_PCFG_usMinFrTimeMsecMult10
    S5K4ECGX_write_cmos_sensor(0x002A, 0x02D0);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_0TC_PCFG_uPrevMirror
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_0TC_PCFG_uCaptureMirror
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_0TC_PCFG_uRotation
    S5K4ECGX_PreviewWin[0].GrabStartX = 0;
    S5K4ECGX_PreviewWin[0].GrabStartY = 0;
    S5K4ECGX_PreviewWin[0].ExposureWindowWidth = S5K4ECGX_IMAGE_SENSOR_PV_WIDTH_DRV;
    S5K4ECGX_PreviewWin[0].ExposureWindowHeight = S5K4ECGX_IMAGE_SENSOR_PV_HEIGHT_DRV;


    //Preview config[1]: video mode
    //91MHz, 1280x720, Dynamic 30~7.5fps
    S5K4ECGX_write_cmos_sensor(0x002A, 0x02D6);   //Night mode(VGA preview 30~4fps)
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0500);   //REG_1TC_PCFG_usWidth
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x02D0);   //REG_1TC_PCFG_usHeight
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0005);   //REG_1TC_PCFG_Format 5 YUV 7 Raw 9 JPG
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK0_MAX);  //REG_1TC_PCFG_usMaxOut4KHzRate PCLK Min : xxMhz
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK0_MIN);  //REG_1TC_PCFG_usMinOut4KHzRate PCLK Max : xxMhz
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0100);   //REG_1TC_PCFG_OutClkPerPix88
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0300);   //REG_1TC_PCFG_uBpp88
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0002);   //REG_1TC_PCFG_PVIMask
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_1TC_PCFG_OIFMask
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x01E0);   //REG_1TC_PCFG_usJpegPacketSize
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_1TC_PCFG_usJpegTotalPackets
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_1TC_PCFG_uClockInd
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_1TC_PCFG_usFrTimeType
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001);   //REG_1TC_PCFG_FrRateQualityType
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0535);   //REG_1TC_PCFG_usMaxFrTimeMsecMult10
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x014d);   //REG_1TC_PCFG_usMinFrTimeMsecMult10
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0300);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_1TC_PCFG_uPrevMirror
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_1TC_PCFG_uCaptureMirror
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_1TC_PCFG_uRotation
    S5K4ECGX_PreviewWin[1].GrabStartX = 0;
    S5K4ECGX_PreviewWin[1].GrabStartY = 0;
    S5K4ECGX_PreviewWin[1].ExposureWindowWidth = 0x0500;
    S5K4ECGX_PreviewWin[1].ExposureWindowHeight = 0x02D0;


    //Preview config[2]:night mode
    //91MHz, 1280x960, 30 ~ 5fps
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0306);   //Night mode(VGA preview 30~4fps)
    S5K4ECGX_write_cmos_sensor(0x0F12, S5K4ECGX_IMAGE_SENSOR_PV_WIDTH_DRV);   //REG_2TC_PCFG_usWidth
    S5K4ECGX_write_cmos_sensor(0x0F12, S5K4ECGX_IMAGE_SENSOR_PV_HEIGHT_DRV);   //REG_2TC_PCFG_usHeight
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0005);   //REG_2TC_PCFG_Format 5 YUV 7 Raw 9 JPG
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK0_MAX);  //REG_2TC_PCFG_usMaxOut4KHzRate PCLK Min : xxMhz
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK0_MIN);  //REG_2TC_PCFG_usMinOut4KHzRate PCLK Max : xxMhz
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0100);   //REG_2TC_PCFG_OutClkPerPix88
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0300);   //REG_2TC_PCFG_uBpp88
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0002);   //REG_2TC_PCFG_PVIMask
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_2TC_PCFG_OIFMask
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x01E0);   //REG_2TC_PCFG_usJpegPacketSize
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_2TC_PCFG_usJpegTotalPackets
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_2TC_PCFG_uClockInd
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_2TC_PCFG_usFrTimeType
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001);   //REG_2TC_PCFG_FrRateQualityType
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x07D0);   //REG_2TC_PCFG_usMaxFrTimeMsecMult10
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x014A);   //REG_2TC_PCFG_usMinFrTimeMsecMult10
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0330);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_2TC_PCFG_uPrevMirror
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_2TC_PCFG_uCaptureMirror
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_2TC_PCFG_uRotation
    S5K4ECGX_PreviewWin[2].GrabStartX = 0;
    S5K4ECGX_PreviewWin[2].GrabStartY = 0;
    S5K4ECGX_PreviewWin[2].ExposureWindowWidth = S5K4ECGX_IMAGE_SENSOR_PV_WIDTH_DRV;
    S5K4ECGX_PreviewWin[2].ExposureWindowHeight = S5K4ECGX_IMAGE_SENSOR_PV_HEIGHT_DRV;


    //Preview config[3] 2560x1920; 15 ~ 7.5fps
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0336);
    S5K4ECGX_write_cmos_sensor(0x0F12, S5K4ECGX_IMAGE_SENSOR_FULL_WIDTH_DRV); //REG_3TC_PCFG_usWidth
    S5K4ECGX_write_cmos_sensor(0x0F12, S5K4ECGX_IMAGE_SENSOR_FULL_HEIGHT_DRV); //REG_3TC_PCFG_usHeight
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0005);   //REG_3TC_PCFG_Format 5 YUV 7 Raw 9 JPG
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK0_MAX);  //REG_3TC_PCFG_usMaxOut4KHzRate PCLK Min : xxMhz
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK0_MIN);  //REG_3TC_PCFG_usMinOut4KHzRate PCLK Max : xxMhz
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0100); //REG_3TC_PCFG_OutClkPerPix88
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0300); //REG_3TC_PCFG_uBpp88
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0002); //REG_3TC_PCFG_PVIMask   //YUYV
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_3TC_PCFG_OIFMask
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x01E0); //REG_3TC_PCFG_usJpegPacketSize
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_3TC_PCFG_usJpegTotalPackets
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_3TC_PCFG_uClockInd
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_3TC_PCFG_usFrTimeType
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0002); //REG_3TC_PCFG_FrRateQualityType
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0535); //REG_3TC_PCFG_usMaxFrTimeMsecMult10  0x03E8
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0250); //REG_3TC_PCFG_usMinFrTimeMsecMult10  0x029A
    //Full size: frame time: 0x3e8; 0x600 --> OK, but fps is too low
    S5K4ECGX_PreviewWin[3].GrabStartX = 0;
    S5K4ECGX_PreviewWin[3].GrabStartY = 0;
    S5K4ECGX_PreviewWin[3].ExposureWindowWidth = S5K4ECGX_IMAGE_SENSOR_FULL_WIDTH_DRV;
    S5K4ECGX_PreviewWin[3].ExposureWindowHeight = S5K4ECGX_IMAGE_SENSOR_FULL_HEIGHT_DRV;

    S5K4ECGX_write_cmos_sensor(0x002A, 0x0360);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_3TC_PCFG_uPrevMirror
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_3TC_PCFG_uCaptureMirror
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_3TC_PCFG_uRotation



    //Capture config[0]
    //S5K4ECGX_write_cmos_sensor(0x002A, 0x0396);  //Normal mode Capture(7.5fps)
    //S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001);   //REG_0TC_CCFG_uCaptureMode//[Sophie Add]
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0398);
    S5K4ECGX_write_cmos_sensor(0x0F12, S5K4ECGX_IMAGE_SENSOR_FULL_WIDTH_DRV);   //REG_0TC_CCFG_usWidth (5M)
    S5K4ECGX_write_cmos_sensor(0x0F12, S5K4ECGX_IMAGE_SENSOR_FULL_HEIGHT_DRV);   //REG_0TC_CCFG_usHeight
    S5K4ECGX_write_cmos_sensor(0x0F12, OUTPUT_FMT);   //REG_0TC_PCFG_Format 5 YUV 7 Raw 9 JPG
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK1_MAX);  //REG_0TC_CCFG_usMaxOut4KHzRate PCLK Min : xxMhz
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CLK1_MIN);  //REG_0TC_CCFG_usMinOut4KHzRate PCLK Max : xxMhz
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0100);   //REG_0TC_CCFG_OutClkPerPix88
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0300);   //REG_0TC_CCFG_uBpp88
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0002);   //REG_0TC_CCFG_PVIMask //[Sophie Add]
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0070);   //REG_0TC_CCFG_OIFMask
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0810);   //REG_0TC_CCFG_usJpegPacketSize
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0900);   //REG_0TC_CCFG_usJpegTotalPackets
    S5K4ECGX_write_cmos_sensor(0x0F12, MIPI_CAP_CLK_IDX);   //REG_0TC_CCFG_uClockInd
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_0TC_CCFG_usFrTimeType
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0002);   //REG_0TC_CCFG_FrRateQualityType
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0535);   //REG_0TC_CCFG_usMaxFrTimeMsecMult10 0x029A
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);   //REG_0TC_CCFG_usMinFrTimeMsecMult10


    S5K4ECGX_write_cmos_sensor(0x002A ,0x0250);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00);   //REG_TC_GP_PrevReqInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0780);   //REG_TC_GP_PrevReqInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010);   //REG_TC_GP_PrevInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000C);   //REG_TC_GP_PrevInputHeightOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00);   //REG_TC_GP_CapReqInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0780);   //REG_TC_GP_CapReqInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010);   //REG_TC_GP_CapInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000C);   //REG_TC_GP_CapInputHeightOfs
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0494);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0780);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0780);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0262);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x1CC2);   //DRx_uDRxWeight for AutoCont function
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x022C);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001);   //REG_TC_IPRM_InitParamsUpdated

    //==================================================================================
    // 21.Select Cofigration Display
    //==================================================================================
    //PREVIEW
    //Select preview 0
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0266);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000 + S5K4ECGX_RV_DefaultMode); //REG_TC_GP_ActivePrevConfig
    S5K4ECGX_write_cmos_sensor(0x002A, 0x026A);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_PrevOpenAfterChange
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0268);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_PrevConfigChanged
    S5K4ECGX_write_cmos_sensor(0x002A, 0x026E);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_TC_GP_ActiveCapConfig
    S5K4ECGX_write_cmos_sensor(0x002A, 0x026A);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_CapOpenAfterChange
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0270);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_CapConfigChanged
    S5K4ECGX_write_cmos_sensor(0x002A, 0x024E);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_NewConfigSync
    S5K4ECGX_write_cmos_sensor(0x002A, 0x023E);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_EnablePreview
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_EnablePreviewChanged

   //===================================================================================
   // 22. ESD Check
   //===================================================================================
   //S5K4ECGX_write_cmos_sensor(0x002A, 0x01A8);
   //S5K4ECGX_write_cmos_sensor(0x0F12, 0xAAAA);

   //===================================================================================
   // 23. Brightness min/Max
   //===================================================================================
   //S5K4ECGX_write_cmos_sensor(0x0028 ,0x147C);
   //S5K4ECGX_write_cmos_sensor(0x002A ,0x01AA);
   //S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0180);   //bp_uMaxBrightnessFactor
   //S5K4ECGX_write_cmos_sensor(0x0028 ,0x1482);
   //S5K4ECGX_write_cmos_sensor(0x002A ,0x01AC);
   //S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0180);   //bp_uMinBrightnessFactor
   return;
}



static void S5K4ECGX_MIPI_enb_preview(void)
{
   SENSORDB("[4EC] Enable preview...\n");
   S5K4ECGX_write_cmos_sensor(0x002A, 0x023E);
   S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Enable Preview output
   S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Sync FW with Enable preview
   SENSORDB("[4EC] Enable preview done...\n");
}



/*************************************************************************
* FUNCTION
*    S5K4ECGXReadShutter
*
* DESCRIPTION
*    This function Read Shutter from sensor
*
* PARAMETERS
*    Shutter: integration time
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint16 S5K4ECGX_MIPI_ReadShutter(void)
{
    kal_uint32 Shutter=0,Integration_Time=0;

    S5K4ECGX_write_cmos_sensor(0x002C, 0x7000);
    S5K4ECGX_write_cmos_sensor(0x002E, 0x2c28);

    //Integration_Time = ((coarse_time * line_length_pck) + fine_time +const) * pclk[sec]
    Integration_Time = S5K4ECGX_read_cmos_sensor(0x0F12);
    Integration_Time = Integration_Time + 65536 * S5K4ECGX_read_cmos_sensor(0x0F12);

    //Shutter=PRE_CLK*1000000*Integration_Time/((400*1000)*1920);
    Shutter = PRE_CLK * Integration_Time / 768;
    return Shutter;
}


void S5K4ECGX_MIPI_SetShutter(kal_uint32 iShutter)
{
#if 0
    if(S5K4ECGX_Driver.shutter == iShutter)
        return;
    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.shutter = iShutter;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);
    S5K3H7Y_write_shutter(iShutter);
#endif
}


/*************************************************************************
* FUNCTION
*    S5K4ECGXReadGain
*
* DESCRIPTION
*    This function get gain from sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    Gain: base on 0x40
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint32 S5K4ECGX_MIPI_ReadGain(void)
{
    kal_uint32 Reg ;

    S5K4ECGX_write_cmos_sensor(0x002C, 0x7000);
    S5K4ECGX_write_cmos_sensor(0x002E, 0x2bc4);
    Reg=S5K4ECGX_read_cmos_sensor(0x0F12);

    //spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.sensorGain = Reg;
    //spin_unlock(&s5k4ecgx_mipi_drv_lock);

    Reg = Reg / 2;
    if( Reg < 1)
    {
       Reg=1;
    }

    return Reg; //
}

static void S5K4ECGX_MIPI_SetGain(kal_uint32 iGain)
{
#if 0
    kal_uint32 Reg ;

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.sensorGain = iGain;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    S5K4ECGX_write_cmos_sensor(0x002C,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002E,0x2bc4);
    Reg =S5K4ECGX_read_cmos_sensor(0x0F12);

    Reg =Reg / 2;
    if(Reg < 1)
    {
      Reg = 1;
    }

    return Reg; //
#endif
}


static kal_bool S5K4ECGX_MIPI_FlashTriggerCheck(void)
{
    kal_bool NeedTrigger = KAL_FALSE;
    kal_uint32 Shutter = 0;
    kal_uint32 Gain = 0;
    kal_uint32 Yaverage = 0;
    kal_uint32 Yup=0;
    kal_uint32 Ydown=0;

    ///TODO: check the average Y and threshhold to determine if need to trigger flashlight
    //1: read shutter
    //2: read gain
    //3: determine if trigger flash
    Shutter = S5K4ECGX_MIPI_ReadShutter();
    Gain = S5K4ECGX_MIPI_ReadGain();
    //Yaverage = CamReadCmosSensor(0x56A1);
    //Yup = CamReadCmosSensor(0x3A1B);
    //Ydown = CamReadCmosSensor(0x3A1E);

    if (Shutter > 6250)
    {
        NeedTrigger = KAL_TRUE;
    }
    if (Gain > 5 * 64) /* 5x */
    {
        NeedTrigger = KAL_TRUE;
    }
    if (Gain * Shutter > 3 * 64 * 6250) /* 3x */
    {
        NeedTrigger = KAL_TRUE;
    }

    return NeedTrigger;
}



/*************************************************************************
* FUNCTION
*    S5K4ECGXGetEvAwbRef
*
* DESCRIPTION
*    This function get sensor Ev/Awb (EV05/EV13) for auto scene detect
*
* PARAMETERS
*    Ref
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void S5K4ECGX_MIPI_GetEvAwbRef(PSENSOR_AE_AWB_REF_STRUCT Ref)  //???
{
    Ref->SensorAERef.AeRefLV05Shutter = 3816; //0xc6c
    Ref->SensorAERef.AeRefLV05Gain = 896; /* 4.1x, 128 base */
    Ref->SensorAERef.AeRefLV13Shutter = 99;   //0x88
    Ref->SensorAERef.AeRefLV13Gain = 1 * 128; /* 2x, 128 base */
    Ref->SensorAwbGainRef.AwbRefD65Rgain = 210; //0xc4/* 1.58x, 128 base */
    Ref->SensorAwbGainRef.AwbRefD65Bgain = 149; //0xa6/* 1.23x, 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFRgain = 179; //0xb9/* 1.4453125x, 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFBgain = 267; //0xf1/* 1.8828125x, 128 base */
}
/*************************************************************************
* FUNCTION
*    S5K4ECGXGetCurAeAwbInfo
*
* DESCRIPTION
*    This function get sensor cur Ae/Awb for auto scene detect
*
* PARAMETERS
*    Info
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void S5K4ECGX_MIPI_GetCurAeAwbInfo(PSENSOR_AE_AWB_CUR_STRUCT Info)
{
    Info->SensorAECur.AeCurShutter = S5K4ECGX_MIPI_ReadShutter();
    Info->SensorAECur.AeCurGain = S5K4ECGX_MIPI_ReadGain(); /* 128 base */
    S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);

    Info->SensorAwbGainCur.AwbCurRgain = S5K4ECGX_read_cmos_sensor(0x2bd0)/8; //   (sensorGain/1024)*128//
    Info->SensorAwbGainCur.AwbCurBgain = S5K4ECGX_read_cmos_sensor(0x2bd4)/8; /* 128 base */
}



UINT32 S5K4ECGX_MIPI_GetSensorID(UINT32 *sensorID)
{
   int  retry = 3;

   SENSORDB("[Camera & Sensor] S5K4ECGXGetSensorID+ \n");
   // check if sensor ID correct
   do {
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
        S5K4ECGX_write_cmos_sensor(0x002C,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002E,0x01A4);//id register
        *sensorID = S5K4ECGX_read_cmos_sensor(0x0F12);
        SENSORDB("[Camera & Sensor] S5K4EC Read Sensor ID = 0x%04x\n", *sensorID);

        if (*sensorID == S5K4ECGX_SENSOR_ID)
        {
            SENSORDB("[Camera & Sensor] Find S5K4EC- ID = 0x%04x\n", *sensorID);
            break;
        }
        SENSORDB("[Camera & Sensor] S5K4EC Read Sensor ID Fail = 0x%04x\n", *sensorID);
        retry--;
   } while (retry > 0);

   if (*sensorID != S5K4ECGX_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        SENSORDB("[Camera & Sensor] S5K4EC Read Sensor ID Fail = 0x%04x\n", *sensorID);
        return ERROR_SENSOR_CONNECT_FAIL;
   }

   SENSORDB("[Camera & Sensor] S5K4ECGXGetSensorID- \n");

   return ERROR_NONE;
} /* S5K4ECGXGetSensorID() */




/*************************************************************************
* FUNCTION
* S5K4ECOpen
*
* DESCRIPTION
* This function initialize the registers of CMOS sensor
*
* PARAMETERS
* None
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 S5K4ECGX_MIPI_Open(void)
{
    kal_uint16 sensor_id=0;

    SENSORDB("[Enter]:S5K4ECGX_MIPI_Open:\r\n");
    S5K4ECGX_MIPI_GetSensorID(&sensor_id);
    if (0xFFFFFFFF == sensor_id)
    {
        SENSORDB("[Camera & Sensor] S5K4EC Read Sensor ID Fail\n");
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    S5K4ECGX_MIPI_Init_Setting();
    S5K4ECGX_MIPI_enb_preview();
    S5K4ECGX_Driver.shutter = 0x4EA;
    S5K4ECGX_Driver.sensorGain = 0x1f;
    S5K4ECGX_Driver.Dummy_Pixels = 0;
    S5K4ECGX_Driver.Dummy_Lines = 0;


    return ERROR_NONE;
} /* S5K4ECGXOpen() */




/*************************************************************************
* FUNCTION
* S5K4ECGXClose
*
* DESCRIPTION
* This function is to turn off sensor module power.
*
* PARAMETERS
* None
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K4ECGX_MIPI_Close(void)
{

    return ERROR_NONE;
} /* S5K4ECGXClose() */



#if 0
static void S5K4ECGX_MIPI_Preview_Mode_Setting(kal_uint8 preview_mode)
{
    SENSORDB("[Enter]:S5K4ECGX Preview mode: mode=%d\r\n", preview_mode);

    if(SEN_RUN_TEST_PATTERN)
    {
        S5K4ECGX_MIPI_SetTestPatternMode(1);
    }

    if (preview_mode > 3)
    {
        preview_mode = 2;
    }

    //Select preview 2
    S5K4ECGX_write_cmos_sensor(0xFCFC, 0xD000);
    S5K4ECGX_write_cmos_sensor(0x002C, 0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0266);
    S5K4ECGX_write_cmos_sensor(0x0F12, preview_mode); //REG_TC_GP_ActivePrevConfig

    S5K4ECGX_write_cmos_sensor(0x002A, 0x026A);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_PrevOpenAfterChange
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0268);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_PrevConfigChanged
    S5K4ECGX_write_cmos_sensor(0x002A, 0x026E);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_TC_GP_ActiveCapConfig
    S5K4ECGX_write_cmos_sensor(0x002A, 0x026A);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_CapOpenAfterChange
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0270);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_CapConfigChanged
    S5K4ECGX_write_cmos_sensor(0x002A, 0x024E);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_NewConfigSync
    S5K4ECGX_write_cmos_sensor(0x002A, 0x023E);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_EnablePreview
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_EnablePreviewChanged

    SENSORDB("[Exit]:S5K4ECGX Preview mode:\r\n");
}


static void S5K4ECGX_MIPI_Capture_Mode_Setting(kal_uint8 capture_mode)
{
    kal_uint32 af_error_state = 0xff;
    SENSORDB("[Enter]:S5K4ECGX Enter capture mode\r\n");

    while(af_error_state!=0)
    {
       S5K4ECGX_write_cmos_sensor(0xFCFC,0xd000);
       S5K4ECGX_write_cmos_sensor(0x002C,0x7000);
       S5K4ECGX_write_cmos_sensor(0x002E,0x0290);
       af_error_state = S5K4ECGX_read_cmos_sensor(0x0F12); //Index number of active capture configuration //Normal capture//
       Sleep(10);
    }
    SENSORDB("[Exit]:S5K4ECGX Leave af state check\r\n");

    //test config if active
    kal_uint32 capture_state=0xff;

    if(SEN_RUN_TEST_PATTERN)
    {
        S5K4ECGX_MIPI_SetTestPatternMode(1);
    }

    while(capture_state!=0)
    {
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xd000);
        S5K4ECGX_write_cmos_sensor(0x002C,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002E,0x0272); //capture state
        capture_state=S5K4ECGX_read_cmos_sensor(0x0F12); //Index number of active capture configuration //Normal capture//
        SENSORDB("[Enter]:S5K4ECGX 1808capture state=%d\r\n", capture_state);
        {
            S5K4ECGX_write_cmos_sensor(0x002a, 0x026e);
            S5K4ECGX_write_cmos_sensor(0x0f12, 0x0000 + capture_mode);
            S5K4ECGX_write_cmos_sensor(0x002A, 0x0242);
            S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001);
            S5K4ECGX_write_cmos_sensor(0x002A, 0x024E);
            S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001);
            S5K4ECGX_write_cmos_sensor(0x002A, 0x0244);
            S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001);
            S5K4ECGX_write_cmos_sensor(0x002A, 0x0272);
            S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000);
        }
        Sleep(10);
    }
    SENSORDB("[Exit]:S5K4ECGX Capture mode:\r\n");
}

#else

unsigned int s5k4ec_cap_enable = 0;
static void S5K4ECGX_MIPI_Preview_Mode_Setting(kal_uint8 preview_mode)
{
    SENSORDB("[Enter]:S5K4ECGX Preview_Mode_Setting: mode=%d\r\n", preview_mode);
    unsigned int cap_en = 0;
    if(SEN_RUN_TEST_PATTERN)
    {
        S5K4ECGX_MIPI_SetTestPatternMode(1);
    }

    if (preview_mode > 3)
    {
        preview_mode = 3;
    }

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    cap_en = s5k4ec_cap_enable;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    if (cap_en)
    {
        // stop Capture
        SENSORDB("[Enter]:S5K4ECGX Preview_Mode_Setting: Stop Capture\r\n");
        S5K4ECGX_write_cmos_sensor(0x0028 ,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A ,0x023E);
        S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_GP_EnablePreview
        S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //REG_TC_GP_EnablePreviewChanged
        S5K4ECGX_write_cmos_sensor(0x002A ,0x0242);
        S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_GP_EnableCapture
        S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //REG_TC_GP_EnableCaptureChanged
        spin_lock(&s5k4ecgx_mipi_drv_lock);
        s5k4ec_cap_enable = 0;
        spin_unlock(&s5k4ecgx_mipi_drv_lock);
        mdelay(200);
    }

    //FCFCD000
    //S5K4ECGX_write_cmos_sensor(0xFCFC, 0xD000);
    SENSORDB("[Enter]:S5K4ECGX Preview_Mode_Setting: ReconfigPreview Size\r\n");
    S5K4ECGX_write_cmos_sensor(0x0028 ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x18AC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //senHal_uAddColsBin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //senHal_uAddColsNoBin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x07DC); //senHal_uMinColsBin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05C0); //senHal_uMinColsNoBin

    S5K4ECGX_write_cmos_sensor(0x002A ,0x0250);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00); //REG_TC_GP_CapReqInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0780); //REG_TC_GP_CapReqInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //REG_TC_GP_CapInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x000C); //REG_TC_GP_CapInputHeightOfs

    S5K4ECGX_write_cmos_sensor(0x002A ,0x0494);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00); //REG_TC_PZOOM_ZoomInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0780); //REG_TC_PZOOM_ZoomInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_PZOOM_ZoomInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_PZOOM_ZoomInputHeightOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00); //REG_TC_CZOOM_ZoomInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0780); //REG_TC_CZOOM_ZoomInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_CZOOM_ZoomInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_CZOOM_ZoomInputHeightOfs

    S5K4ECGX_write_cmos_sensor(0x002A ,0x0262);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); // #REG_TC_GP_bUseReqInputInPre
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //REG_TC_GP_bUseReqInputInCap

    S5K4ECGX_write_cmos_sensor(0x002A ,0x0A1E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0028); // AfitBaseVals_0__73_ 0040   Why??
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0AD4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x003C); // AfitBaseVals_1__73_ 0060   Why??


    //Select preview
    S5K4ECGX_write_cmos_sensor(0x002A  ,0x0266);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,preview_mode);  //REG_TC_GP_ActivePrevConfig
    S5K4ECGX_write_cmos_sensor(0x002A  ,0x026A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);  //REG_TC_GP_PrevOpenAfterChange
    S5K4ECGX_write_cmos_sensor(0x002A  ,0x0268);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);  //REG_TC_GP_PrevConfigChanged
    S5K4ECGX_write_cmos_sensor(0x002A  ,0x024E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);  //REG_TC_GP_NewConfigSync


    //start preview
    SENSORDB("[Enter]:S5K4ECGX Preview_Mode_Setting: Start Preview\r\n");
    S5K4ECGX_write_cmos_sensor(0x0028  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A  ,0x023E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);  //REG_TC_GP_EnablePreview
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);  //REG_TC_GP_EnablePreviewChanged
    return;
}



UINT32 S5K4ECGX_MIPI_StopPreview(void)
{
  unsigned int status = 1;
  unsigned int prev_en = 1;

  //spin_lock(&s5k4ecgx_mipi_drv_lock);
    //prev_en = S5K4ECGX_Preview_enabled;
  //spin_unlock(&s5k4ecgx_mipi_drv_lock);

    //if (prev_en)
    {
      SENSORDB("[Exit]:S5K4ECGX StopPreview + ><><><><><><><><><><\r\n");

      S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
      S5K4ECGX_write_cmos_sensor(0x002A, 0x023E);
      S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_TC_GP_EnablePreview
      S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_EnablePreviewChanged
      S5K4ECGX_write_cmos_sensor(0x002A, 0x0242);
      S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_TC_GP_EnableCapture
      S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_EnableCaptureChanged

      /*S5K4ECGX_write_cmos_sensor(0x002C, 0x7000);
      S5K4ECGX_write_cmos_sensor(0x002E, 0x0240);
      while(status!=0)
      {
        status = S5K4ECGX_read_cmos_sensor(0x0F12); //REG_TC_GP_EnablePreview ; xxxx==0000
        Sleep(5);
      }*/
      mdelay(200);
      //S5K4ECGX_Preview_enabled= 0;
      SENSORDB("[Exit]:S5K4ECGX StopPreview - ><><><><><><><><><><\r\n");
    }
}



static void S5K4ECGX_MIPI_Capture_Mode_Setting(kal_uint8 capture_mode)
{
    unsigned int status = 0;

    SENSORDB("[Exit]:S5K4ECGX Capture_Mode_Setting+\r\n");


    S5K4ECGX_MIPI_StopPreview();

    //<CAMTUNING_INIT>

    //capture
    //static const u32 s5k4ecgx_5M_Capture[] = {
    SENSORDB("[Exit]:S5K4ECGX Capture_Mode_Setting: Reconfig\r\n");

    S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0258);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0A00); //REG_TC_GP_CapReqInputWidth //2560
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0780); //REG_TC_GP_CapReqInputHeight //1920
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0010); //REG_TC_GP_CapInputWidthOfs //(2592-2560)/2
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x000C); //REG_TC_GP_CapInputHeightOfs //(1944-1920)/2
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0264);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_bUseReqInputInCap

    S5K4ECGX_write_cmos_sensor(0x002A, 0x049C);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0A00); //REG_TC_PZOOM_CapZoomReqInputWidth //2560
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0780); //REG_TC_PZOOM_CapZoomReqInputHeight //1920
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_TC_PZOOM_CapZoomReqInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0000); //REG_TC_PZOOM_CapZoomReqInputHeightOfs

    //* Kilsung.Hur 20121209 no need for YUV.
    S5K4ECGX_write_cmos_sensor(0x002A, 0x047C);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_THUMB_Thumb_bActive
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0280); //REG_TC_THUMB_Thumb_uWidth //640
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x01E0); //REG_TC_THUMB_Thumb_uHeight //480

    //mipi format
    S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
    //S5K4ECGX_write_cmos_sensor(0x002a, 0x026e);
    //S5K4ECGX_write_cmos_sensor(0x0f12, 0x0000 + capture_mode); //REG_TC_GP_ActiveCapConfig

    SENSORDB("[Exit]:S5K4ECGX Capture_Mode_Setting: Start Preview\r\n");
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0242);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_EnableCapture
    S5K4ECGX_write_cmos_sensor(0x002A, 0x024E);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_NewConfigSync
    S5K4ECGX_write_cmos_sensor(0x002A, 0x0244);
    S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //REG_TC_GP_EnableCaptureChanged

    /*S5K4ECGX_write_cmos_sensor(0x002C, 0x7000);
    S5K4ECGX_write_cmos_sensor(0x002E, 0x0530);
    //read
    while(status!=1)
    {
      status = S5K4ECGX_read_cmos_sensor(0x0F12); //REG_TC_GP_EnablePreview ; xx==01
        Sleep(5);
    }*/
    mdelay(5);
    //S5K4ECGX_Preview_enabled = 1;

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    s5k4ec_cap_enable = 1;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);
    SENSORDB("[Exit]:S5K4ECGX Capture_Mode_Setting-\r\n");
}

#endif



static void S5K4ECGX_MIPI_HVMirror(kal_uint8 image_mirror)
{
    /********************************************************
    Preview:Mirror: 0x02d0 bit[0],Flip :    0x02d0 bit[1]
    Capture:Mirror: 0x02d2 bit[0],Flip :    0x02d2 bit[1]
    *********************************************************/

    SENSORDB("[Enter]:S5K4ECGX Mirror\r\n");

    //if(S5K4ECYX_MIPICurrentStatus.iMirror == image_mirror)
    //    return;

    S5K4ECGX_write_cmos_sensor(0xFCFC,0xd000);
    S5K4ECGX_write_cmos_sensor(0x0028,0x7000);


    switch (image_mirror)
    {
        case IMAGE_NORMAL:
        default:
            S5K4ECGX_write_cmos_sensor(0x002A,  0x02D0);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0000);  //#REG_0TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0000);  //#REG_0TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0300);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0000);  //#REG_1TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0000);  //#REG_1TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0330);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0000);  //#REG_2TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0000);  //#REG_2TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0360);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0000);  //#REG_3TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0000);  //#REG_3TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0390);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0000);  //#REG_4TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0000);  //#REG_4TC_PCFG_uCaptureMirror
            break;

        case IMAGE_H_MIRROR:
            S5K4ECGX_write_cmos_sensor(0x002A,  0x02D0);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0001);  //#REG_0TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0001);  //#REG_0TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0300);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0001);  //#REG_1TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0001);  //#REG_1TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0330);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0001);  //#REG_2TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0001);  //#REG_2TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0360);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0001);  //#REG_3TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0001);  //#REG_3TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0390);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0001);  //#REG_4TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0001);  //#REG_4TC_PCFG_uCaptureMirror
            break;

        case IMAGE_V_MIRROR:
            S5K4ECGX_write_cmos_sensor(0x002A,  0x02D0);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0002);  //#REG_0TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0002);  //#REG_0TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0300);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0002);  //#REG_1TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0002);  //#REG_1TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0330);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0002);  //#REG_2TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0002);  //#REG_2TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0360);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0002);  //#REG_3TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0002);  //#REG_3TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0390);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0002);  //#REG_4TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0002);  //#REG_4TC_PCFG_uCaptureMirror
            break;

        case IMAGE_HV_MIRROR:
            S5K4ECGX_write_cmos_sensor(0x002A,  0x02D0);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0003);  //#REG_0TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0003);  //#REG_0TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0300);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0003);  //#REG_1TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0003);  //#REG_1TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0330);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0003);  //#REG_2TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0003);  //#REG_2TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0360);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0003);  //#REG_3TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0003);  //#REG_3TC_PCFG_uCaptureMirror

            S5K4ECGX_write_cmos_sensor(0x002A,  0x0390);
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0003);  //#REG_4TC_PCFG_uPrevMirror
            S5K4ECGX_write_cmos_sensor(0x0F12,  0x0003);  //#REG_4TC_PCFG_uCaptureMirror
            break;
    }
    //spin_lock(&s5k4ecgx_mipi_drv_lock);
    //S5K4ECYX_MIPICurrentStatus.iMirror = image_mirror;
    //spin_unlock(&s5k4ecgx_mipi_drv_lock);
}


void S5K4ECGX_MIPI_NightMode(kal_bool enable)
{
    SENSORDB("[Enter]S5K4ECGX night mode func:enable = %d\n",enable);
    if(enable)
    {
        S5K4ECGX_MIPI_Preview_Mode_Setting(2); ////MODE0=5-30FPS
    }
    else
    {
        S5K4ECGX_MIPI_Preview_Mode_Setting(S5K4ECGX_RV_DefaultMode); ////MODE0=10-30FPS
    }
}

/*************************************************************************
* FUNCTION
* S5K4ECGXPreview
*
* DESCRIPTION
* This function start the sensor preview.
*
* PARAMETERS
*  *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static UINT32 S5K4ECGX_MIPI_Preview(
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    SENSORDB("[Enter]:S5K4ECGX preview func:");
    //S5K8AAYX_MIPI_sensor_cap_state = KAL_FALSE;

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.sensorMode = SENSOR_MODE_PREVIEW; // Need set preview setting after capture mode
    S5K4ECGX_Driver.Period_PixelNum = S5K4ECGX_IMAGE_SENSOR_PV_WIDTH_DRV + S5K4ECGX_Driver.Dummy_Pixels;
    S5K4ECGX_Driver.Period_LineNum = S5K4ECGX_IMAGE_SENSOR_PV_HEIGHT_DRV + S5K4ECGX_Driver.Dummy_Lines;
    S5K4ECGX_Driver.Preview_Width = S5K4ECGX_IMAGE_SENSOR_PV_WIDTH_DRV;
    S5K4ECGX_Driver.Preview_Height = S5K4ECGX_IMAGE_SENSOR_PV_HEIGHT_DRV;
    //S5K4ECGX_Driver.imgMirror = sensor_config_data->SensorImageMirror;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);


    //S5K4ECGX_MIPI_write_shutter(S5K4ECGX_Driver.shutter);
    //S5K4ECGX_MIPI_SetGain(S5K4ECGX_Driver.sensorGain);
    //S5K4ECGX_MIPI_HVMirror(sensor_config_data->SensorImageMirror);
    S5K4ECGX_MIPI_Preview_Mode_Setting(S5K4ECGX_RV_DefaultMode); ////MODE0=10-30FPS

    image_window->GrabStartX = S5K4ECGX_PV_X_START;
    image_window->GrabStartY = S5K4ECGX_PV_Y_START;
    image_window->ExposureWindowWidth = S5K4ECGX_IMAGE_SENSOR_PV_WIDTH - 2 * S5K4ECGX_PV_X_START;
    image_window->ExposureWindowHeight = S5K4ECGX_IMAGE_SENSOR_PV_HEIGHT - 2 * S5K4ECGX_PV_Y_START;

    memcpy(&S5K4ECGXSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    SENSORDB("[Exit]:S5K4ECGX preview func\n");
    return ERROR_NONE;
} /* S5K4ECGXPreview */



UINT32 S5K4ECGX_MIPI_Capture(
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint32 shutter = S5K4ECGX_Driver.shutter;
    kal_uint32 gain = 0;

    if(SENSOR_MODE_CAPTURE== S5K4ECGX_Driver.sensorMode)
    {
        SENSORDB("S5K4ECGXCapture BusrtShot!!!\n");
        return ERROR_NONE;
    }

    //Record Preview shutter & gain
    //shutter = S5K4ECGX_MIPI_ReadShutter();
    //gain = S5K4ECGX_MIPI_ReadGain();

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.shutter = shutter;
    S5K4ECGX_Driver.sensorGain = gain;
    //s5k3h7y.imgMirror = sensor_config_data->SensorImageMirror;
    //S5K4ECGX_Driver.Period_PixelNum = S5K4ECGX_IMAGE_SENSOR_PV_WIDTH_DRV + S5K4ECGX_Driver.DummyPixels;
    //S5K4ECGX_Driver.Period_LineNum = S5K4ECGX_IMAGE_SENSOR_PV_HEIGHT_DRV + S5K4ECGX_Driver.DummyLines;
    //S5K4ECGX_Driver.pvGain =s5k3h7y.sensorGain;
    S5K4ECGX_Driver.Camco_mode = S5K4ECGX_CAM_CAPTURE; //to be removed?
    S5K4ECGX_Driver.StartX = S5K4ECGX_FULL_X_START;//1; //to be removed?
    S5K4ECGX_Driver.StartY = S5K4ECGX_FULL_Y_START;//1; //to be removed?
    spin_unlock(&s5k4ecgx_mipi_drv_lock);;

    //when entry capture mode,will auto close ae,awb .
    SENSORDB("[Enter]:S5K4ECGX_MIPI_Capture targetwidth=%d\r\n", image_window->ImageTargetWidth);
    //S5K4ECGX_MIPI_AF_ShowLensPosition();

    S5K4ECGX_MIPI_Capture_Mode_Setting(0);
    //S5K3H7YSetFlipMirror(sensor_config_data->SensorImageMirror);

    Sleep(4);
    spin_lock(&s5k4ecgx_mipi_drv_lock);
    //S5K4ECGX_Driver.iGrabWidth = S5K4ECGX_IMAGE_SENSOR_FULL_WIDTH_DRV - 16;
    //S5K4ECGX_Driver.iGrabheight = S5K4ECGX_IMAGE_SENSOR_FULL_HEIGHT_DRV - 12;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);
#if 0
    image_window->GrabStartX = S5K4ECGX_Driver.StartX;
    image_window->GrabStartY = S5K4ECGX_Driver.StartY;
    image_window->ExposureWindowWidth = S5K4ECGX_Driver.iGrabWidth;
    image_window->ExposureWindowHeight = S5K4ECGX_Driver.iGrabheight;
#else
    S5K4ECGX_Driver.iGrabWidth = S5K4ECGX_IMAGE_SENSOR_FULL_WIDTH_DRV -2 * S5K4ECGX_FULL_X_START;
    S5K4ECGX_Driver.iGrabheight = S5K4ECGX_IMAGE_SENSOR_FULL_HEIGHT_DRV -2 * S5K4ECGX_FULL_Y_START;
    image_window->GrabStartX = S5K4ECGX_FULL_X_START;
    image_window->GrabStartY = S5K4ECGX_FULL_Y_START;
    image_window->ExposureWindowWidth = S5K4ECGX_Driver.iGrabWidth;
    image_window->ExposureWindowHeight = S5K4ECGX_Driver.iGrabheight;
#endif

}


UINT32 S5K4ECGX_MIPI_GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("[Enter]:S5K4ECGX get Resolution func\n");

    pSensorResolution->SensorFullWidth=S5K4ECGX_IMAGE_SENSOR_FULL_WIDTH_DRV - 2*S5K4ECGX_FULL_X_START;
    pSensorResolution->SensorFullHeight=S5K4ECGX_IMAGE_SENSOR_FULL_HEIGHT_DRV - 2*S5K4ECGX_FULL_Y_START;
    switch(S5K4ECGXCurrentScenarioId)
    {
       case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorResolution->SensorPreviewWidth   = S5K4ECGX_IMAGE_SENSOR_FULL_WIDTH_DRV - 2*S5K4ECGX_FULL_X_START;
            pSensorResolution->SensorPreviewHeight  = S5K4ECGX_IMAGE_SENSOR_FULL_HEIGHT_DRV - 2*S5K4ECGX_FULL_Y_START;
            break;
       default:
            pSensorResolution->SensorPreviewWidth   = S5K4ECGX_IMAGE_SENSOR_PV_WIDTH_DRV - 2*S5K4ECGX_PV_X_START;
            pSensorResolution->SensorPreviewHeight  = S5K4ECGX_IMAGE_SENSOR_PV_HEIGHT_DRV - 2*S5K4ECGX_PV_Y_START;
            pSensorResolution->SensorVideoWidth  = pSensorResolution->SensorPreviewWidth;
            pSensorResolution->SensorVideoHeight = pSensorResolution->SensorPreviewHeight;
            break;
    }

    pSensorResolution->Sensor3DFullWidth = 0;
    pSensorResolution->Sensor3DFullHeight= 0;
    pSensorResolution->Sensor3DPreviewWidth = 0;
    pSensorResolution->Sensor3DPreviewHeight = 0;
    SENSORDB("[Exit]:S5K4ECGX get Resolution func\n");
    return ERROR_NONE;
} /* NSXC301HS5K4ECGXGetResolution() */


UINT32 S5K4ECGX_MIPI_GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
    MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    SENSORDB("[Enter]:S5K4ECGX getInfo func: ScenarioId = %d\n",ScenarioId);

    switch(ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
          pSensorInfo->SensorPreviewResolutionX = S5K4ECGX_IMAGE_SENSOR_FULL_WIDTH_DRV;
          pSensorInfo->SensorPreviewResolutionY = S5K4ECGX_IMAGE_SENSOR_FULL_HEIGHT_DRV;
          pSensorInfo->SensorCameraPreviewFrameRate=15;
          break;
        default:
          pSensorInfo->SensorPreviewResolutionX = S5K4ECGX_IMAGE_SENSOR_PV_WIDTH_DRV;
          pSensorInfo->SensorPreviewResolutionY = S5K4ECGX_IMAGE_SENSOR_PV_HEIGHT_DRV;
          pSensorInfo->SensorCameraPreviewFrameRate=30;
          break;
    }

    pSensorInfo->SensorFullResolutionX = S5K4ECGX_IMAGE_SENSOR_FULL_WIDTH_DRV;
    pSensorInfo->SensorFullResolutionY = S5K4ECGX_IMAGE_SENSOR_FULL_HEIGHT_DRV;
    //pSensorInfo->SensorCameraPreviewFrameRate= 30; //12
    pSensorInfo->SensorVideoFrameRate = 30;
    pSensorInfo->SensorStillCaptureFrameRate = 5;
    pSensorInfo->SensorWebCamCaptureFrameRate = 15;
    pSensorInfo->SensorResetActiveHigh=FALSE;//low is to reset
    pSensorInfo->SensorResetDelayCount=4;  //4ms

    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;

    pSensorInfo->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;

 #ifdef MIPI_INTERFACE
      pSensorInfo->SensorInterruptDelayLines = 2;
      pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_MIPI;
 #else
      pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_PARALLEL;
 #endif

    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_4MA;
    pSensorInfo->CaptureDelayFrame = 1;
    pSensorInfo->PreviewDelayFrame = 2;
    pSensorInfo->VideoDelayFrame = 0;

    //Sophie Add for 72
    pSensorInfo->YUVAwbDelayFrame = 3;
    pSensorInfo->YUVEffectDelayFrame = 2;

    //Sophie: Maigh need to remove the assignments?
    //pSensorInfo->SensorDriver3D = SENSOR_3D_NOT_SUPPORT;
    //SENSORDB("[Enter]:2005still frame=%d\r\n", pSensorInfo->SensorStillCaptureFrameRate);

//Sophie Add:
#ifdef MIPI_INTERFACE  //copy from 8AA
    pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
    pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 4;
    pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    pSensorInfo->SensorWidthSampling = 0;   // 0 is default 1x
    pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
    pSensorInfo->SensorPacketECCOrder = 1;
#endif

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
             pSensorInfo->SensorClockFreq = S5K4ECGX_MCLK;
             pSensorInfo->SensorClockDividCount = 5;
             pSensorInfo->SensorClockRisingCount = 0;
             pSensorInfo->SensorClockFallingCount = 2;
             pSensorInfo->SensorPixelClockCount = 3;
             pSensorInfo->SensorDataLatchCount = 2;
             pSensorInfo->SensorGrabStartX = S5K4ECGX_PV_X_START;
             pSensorInfo->SensorGrabStartY = S5K4ECGX_PV_Y_START;
             break;

        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
             pSensorInfo->SensorClockFreq = S5K4ECGX_MCLK;
             pSensorInfo->SensorClockDividCount = 5;
             pSensorInfo->SensorClockRisingCount = 0;
             pSensorInfo->SensorClockFallingCount = 2;
             pSensorInfo->SensorPixelClockCount = 3;
             pSensorInfo->SensorDataLatchCount = 2;
             pSensorInfo->SensorGrabStartX = S5K4ECGX_FULL_X_START;
             pSensorInfo->SensorGrabStartY = S5K4ECGX_FULL_Y_START;
             break;

        default:
             pSensorInfo->SensorClockFreq = S5K4ECGX_MCLK;
             pSensorInfo->SensorClockDividCount = 5;
             pSensorInfo->SensorClockRisingCount = 0;
             pSensorInfo->SensorClockFallingCount = 2;
             pSensorInfo->SensorPixelClockCount =3;
             pSensorInfo->SensorDataLatchCount =2;
             pSensorInfo->SensorGrabStartX = S5K4ECGX_PV_X_START;
             pSensorInfo->SensorGrabStartY = S5K4ECGX_PV_Y_START;
             break;
    }

    SENSORDB("[Exit]:S5K4ECGX getInfo func\n");
    return ERROR_NONE;
}  /* NSXC301HS5K4ECGXGetInfo() */


/*************************************************************************
* FUNCTION
* S5K4ECGX_set_param_effect
*
* DESCRIPTION
* effect setting.
*
* PARAMETERS
* none
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL S5K4ECGX_MIPI_set_param_effect(UINT16 para)
{

   SENSORDB("[Enter]S5K4ECGX set_param_effect func:para = %d\n",para);
   switch (para)
   {
      case MEFFECT_OFF:
          S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
          S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
          S5K4ECGX_write_cmos_sensor(0x002A,0x023c);
          S5K4ECGX_write_cmos_sensor(0x0F12,0x0000);  //REG_TC_GP_SpecialEffects
          break;
      case MEFFECT_NEGATIVE:
          S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
          S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
          S5K4ECGX_write_cmos_sensor(0x002A,0x023c);
          S5K4ECGX_write_cmos_sensor(0x0F12,0x0002);  //REG_TC_GP_SpecialEffects
          break;
      case MEFFECT_SEPIA:
          S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
          S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
          S5K4ECGX_write_cmos_sensor(0x002A,0x023c);
          S5K4ECGX_write_cmos_sensor(0x0F12,0x0004);  //REG_TC_GP_SpecialEffects
          break;
      case MEFFECT_SEPIABLUE:
          S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
          S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
          S5K4ECGX_write_cmos_sensor(0x002A,0x023c);
          S5K4ECGX_write_cmos_sensor(0x0F12,0x0007);  //REG_TC_GP_SpecialEffects
          break;
      case MEFFECT_SEPIAGREEN:
          S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
          S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
          S5K4ECGX_write_cmos_sensor(0x002A,0x023c);
          S5K4ECGX_write_cmos_sensor(0x0F12,0x0008);  //REG_TC_GP_SpecialEffects
          break;
      case MEFFECT_MONO:
          S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
          S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
          S5K4ECGX_write_cmos_sensor(0x002A,0x023c);
          S5K4ECGX_write_cmos_sensor(0x0F12,0x0001);  //REG_TC_GP_SpecialEffects
          break;
      default:
          return KAL_FALSE;
   }
   return KAL_TRUE;
} /* S5K4ECGX_set_param_effect */


UINT32 S5K4ECGX_MIPI_Control(
    MSDK_SCENARIO_ID_ENUM ScenarioId,
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
   SENSORDB("[Enter]:S5K4ECGX_MIPI_Control  func:ScenarioId = 0x%04x\n",ScenarioId);

   spin_lock(&s5k4ecgx_mipi_drv_lock);
   S5K4ECGXCurrentScenarioId = ScenarioId;
   spin_unlock(&s5k4ecgx_mipi_drv_lock);

   switch (ScenarioId)
   {
      case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
      case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
         //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
         S5K4ECGX_MIPI_Preview(pImageWindow, pSensorConfigData);
         break;
      case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
         //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
         S5K4ECGX_MIPI_Capture(pImageWindow, pSensorConfigData);
         break;
      case MSDK_SCENARIO_ID_CAMERA_ZSD:
         S5K4ECGX_MIPI_Capture(pImageWindow, pSensorConfigData);
         break;
      default:
         break;
   }

   SENSORDB("[Exit]:S5K4ECGX_MIPI_Control  func\n");
   return ERROR_NONE;
} /* S5K4ECGXControl() */


/*************************************************************************
* FUNCTION
* S5K4ECGX_set_param_wb
*
* DESCRIPTION
* wb setting.
*
* PARAMETERS
* none
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/

BOOL S5K4ECGX_MIPI_set_param_wb(UINT16 para)
{

    //This sensor need more time to balance AWB,
    //we suggest higher fps or drop some frame to avoid garbage color when preview initial
    SENSORDB("[Enter]S5K4ECGX set_param_wb func:para = %d\n",para);
    kal_uint16 Status_3A=0;
    while(Status_3A==0)
    {
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xd000);
        S5K4ECGX_write_cmos_sensor(0x002C,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002E,0x04E6);
        Status_3A=S5K4ECGX_read_cmos_sensor(0x0F12); //Index number of active capture configuration //Normal capture//
        Sleep(10);
    }

    switch (para)
    {
      case AWB_MODE_AUTO:
      {
          Status_3A = (Status_3A | 0x8); // Enable AWB
          S5K4ECGX_write_cmos_sensor(0xFCFC, 0xD000);
          S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
          S5K4ECGX_write_cmos_sensor(0x002a, 0x04e6);//
          S5K4ECGX_write_cmos_sensor(0x0F12, Status_3A);//
          //S5K4ECGX_write_cmos_sensor(0x0F12, 0x077F);//
      }
      break;
      case AWB_MODE_CLOUDY_DAYLIGHT:
      {
         Status_3A = (Status_3A & 0xFFF7); // Disable AWB
         S5K4ECGX_write_cmos_sensor(0xFCFC, 0xD000);
         S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
         S5K4ECGX_write_cmos_sensor(0x002A, 0x04E6);
         S5K4ECGX_write_cmos_sensor(0x0F12, Status_3A);
         //S5K4ECGX_write_cmos_sensor(0x0F12, 0x0777);
         S5K4ECGX_write_cmos_sensor(0x002A, 0x04BA);
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0740); //Reg_sf_user_Rgain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RgainChanged
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x03D0); //0400Reg_sf_user_Ggain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_GgainChanged
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x04D0); //0460Reg_sf_user_Bgain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_BgainChanged
      }
      break;
      case AWB_MODE_DAYLIGHT:
      {
         Status_3A = (Status_3A & 0xFFF7); // Disable AWB
         S5K4ECGX_write_cmos_sensor(0xFCFC, 0xD000);
         S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
         S5K4ECGX_write_cmos_sensor(0x002A, 0x04E6);
         S5K4ECGX_write_cmos_sensor(0x0F12, Status_3A);
         //S5K4ECGX_write_cmos_sensor(0x0F12, 0x0777);
         S5K4ECGX_write_cmos_sensor(0x002A, 0x04BA);
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x06c5); //05E0Reg_sf_user_Rgain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RgainChanged
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0400); //0400Reg_sf_user_Ggain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_GgainChanged
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x04d3); //0530Reg_sf_user_Bgain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_BgainChanged
      }
      break;
      case AWB_MODE_INCANDESCENT:
      {
         Status_3A = (Status_3A & 0xFFF7); // Disable AWB
         S5K4ECGX_write_cmos_sensor(0xFCFC, 0xD000);
         S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
         S5K4ECGX_write_cmos_sensor(0x002A, 0x04E6);
         S5K4ECGX_write_cmos_sensor(0x0F12, Status_3A);
         //S5K4ECGX_write_cmos_sensor(0x0F12, 0x0777);
         S5K4ECGX_write_cmos_sensor(0x002A, 0x04BA);
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0401); //0575Reg_sf_user_Rgain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RgainChanged
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0400); //0400Reg_sf_user_Ggain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_GgainChanged
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0957); //0800Reg_sf_user_Bgain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_BgainChanged
      }
      break;
      case AWB_MODE_FLUORESCENT:
      {
         Status_3A = (Status_3A & 0xFFF7); // Disable AWB
         S5K4ECGX_write_cmos_sensor(0xFCFC, 0xD000);
         S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
         S5K4ECGX_write_cmos_sensor(0x002A, 0x04E6);
         S5K4ECGX_write_cmos_sensor(0x0F12, Status_3A);
         //S5K4ECGX_write_cmos_sensor(0x0F12, 0x0777);
         S5K4ECGX_write_cmos_sensor(0x002A, 0x04BA);
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x05a1); //0400Reg_sf_user_Rgain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RgainChanged
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0400); //0400Reg_sf_user_Ggain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_GgainChanged
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x08e7); //Reg_sf_user_Bgain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_BgainChanged
      }
      break;
      case AWB_MODE_TUNGSTEN:
      {
         Status_3A = (Status_3A & 0xFFF7); // Disable AWB
         S5K4ECGX_write_cmos_sensor(0xFCFC, 0xD000);
         S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
         S5K4ECGX_write_cmos_sensor(0x002A, 0x04E6);
         S5K4ECGX_write_cmos_sensor(0x0F12, Status_3A);
         //S5K4ECGX_write_cmos_sensor(0x0F12, 0x0777);
         S5K4ECGX_write_cmos_sensor(0x002A, 0x04BA);
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0200); //0400Reg_sf_user_Rgain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RgainChanged
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0200); //0400Reg_sf_user_Ggain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_GgainChanged
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x04A0); //Reg_sf_user_Bgain
         S5K4ECGX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_BgainChanged
      }
      break;
     default:
        return FALSE;
    }
    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECYX_MIPICurrentStatus.iWB = para;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    SENSORDB("S5K4ECGX Status_3A = 0x%x\n",Status_3A);
    return TRUE;
} /* S5K4ECGX_set_param_wb */


/*************************************************************************
* FUNCTION
* S5K4ECGX_set_param_banding
*
* DESCRIPTION
* banding setting.
*
* PARAMETERS
* none
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL S5K4ECGX_MIPI_set_param_banding(UINT16 para)
{
   SENSORDB("[Enter]S5K4ECGX set_param_banding func:para = %d\n",para);
   kal_uint16 Status_3A=0;

   if(S5K4ECGX_Driver.Banding == para)
       return TRUE;

   spin_lock(&s5k4ecgx_mipi_drv_lock);
   S5K4ECGX_Driver.Banding = para;
   spin_unlock(&s5k4ecgx_mipi_drv_lock);;

   while(Status_3A==0)
   {
      S5K4ECGX_write_cmos_sensor(0xFCFC,0xd000);
      S5K4ECGX_write_cmos_sensor(0x002C,0x7000);
      S5K4ECGX_write_cmos_sensor(0x002E,0x04E6); //REG_TC_DBG_AutoAlgEnBits
      Status_3A=S5K4ECGX_read_cmos_sensor(0x0F12);
      Sleep(10);
   }

   switch (para)
   {
       case AE_FLICKER_MODE_50HZ:
       {
          Status_3A = (Status_3A & 0xFFDF); // disable auto-flicker first
          S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
          S5K4ECGX_write_cmos_sensor(0x002a, 0x04e6);
          S5K4ECGX_write_cmos_sensor(0x0f12, Status_3A);
          //S5K4ECGX_write_cmos_sensor(0x0f12, 0x075f);
          S5K4ECGX_write_cmos_sensor(0x002a, 0x04d6);
          S5K4ECGX_write_cmos_sensor(0x0f12, 0x0001); //enable 50MHz
          S5K4ECGX_write_cmos_sensor(0x0f12, 0x0001); //update flicker info to FW
       }
       break;

       case AE_FLICKER_MODE_60HZ:
       {
          Status_3A = (Status_3A & 0xFFDF); // disable auto-flicker
          S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
          S5K4ECGX_write_cmos_sensor(0x002a, 0x04e6);
          S5K4ECGX_write_cmos_sensor(0x0f12, Status_3A);
          S5K4ECGX_write_cmos_sensor(0x002a, 0x04d6);
          S5K4ECGX_write_cmos_sensor(0x0f12, 0x0002); //enable 60MHz
          S5K4ECGX_write_cmos_sensor(0x0f12, 0x0001); //update flicker info to FW
       }
       break;

       default:
       {
          //auto flicker
          Status_3A = (Status_3A | 0x5); // disable auto-flicker
          S5K4ECGX_write_cmos_sensor(0x0028, 0x7000);
          S5K4ECGX_write_cmos_sensor(0x002a, 0x04e6);
          S5K4ECGX_write_cmos_sensor(0x0f12, Status_3A);
          break;
       }
   }
   SENSORDB("S5K4ECGXparam_banding Status_3A = 0x%x\n",Status_3A);
   return KAL_TRUE;
} /* S5K4ECGX_set_param_banding */


/*************************************************************************
* FUNCTION
* S5K4ECGX_set_param_exposure
*
* DESCRIPTION
* exposure setting.
*
* PARAMETERS
* none
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL S5K4ECGX_MIPI_set_param_exposure(UINT16 para)
{
   kal_uint16 base_target = 0;

   SENSORDB("[Enter]S5K4ECGX set_param_exposure func:para = %d\n",para);
   switch (para)
   {
      case AE_EV_COMP_13:  //+4 EV
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A,0x023A);  //UserExposureVal88
        S5K4ECGX_write_cmos_sensor(0x0F12,0x0200);  //TVAR_ae_BrAve
        break;
      case AE_EV_COMP_10:  //+3 EV
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A,0x023A);  //UserExposureVal88
        S5K4ECGX_write_cmos_sensor(0x0F12,0x0190);  //TVAR_ae_BrAve
        break;
      case AE_EV_COMP_07:  //+2 EV
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A,0x023A);  //UserExposureVal88
        S5K4ECGX_write_cmos_sensor(0x0F12,0x0170);  //TVAR_ae_BrAve
        break;
      case AE_EV_COMP_03:  // +1 EV
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A,0x023A);  //UserExposureVal88
        S5K4ECGX_write_cmos_sensor(0x0F12,0x0145);  //TVAR_ae_BrAve
        break;
      case AE_EV_COMP_00:  // +0 EV
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A,0x023A);  //UserExposureVal88
        S5K4ECGX_write_cmos_sensor(0x0F12,0x0100);  //TVAR_ae_BrAve
        break;
      case AE_EV_COMP_n03:  // -1 EV
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A,0x023A);  //UserExposureVal88
        S5K4ECGX_write_cmos_sensor(0x0F12,0x00C0);  //TVAR_ae_BrAve
        break;
      case AE_EV_COMP_n07:  // -2 EV
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A,0x023A);  //UserExposureVal88
        S5K4ECGX_write_cmos_sensor(0x0F12,0x0080);  //TVAR_ae_BrAve
        break;
      case AE_EV_COMP_n10:   //-3 EV
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A,0x023A);  //UserExposureVal88
        S5K4ECGX_write_cmos_sensor(0x0F12,0x0060);  //TVAR_ae_BrAve
        break;
      case AE_EV_COMP_n13:  // -4 EV
        S5K4ECGX_write_cmos_sensor(0xFCFC,0xD000);
        S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A,0x023A);  //UserExposureVal88
        S5K4ECGX_write_cmos_sensor(0x0F12,0x0040);  //TVAR_ae_BrAve
        break;
      default:
        return FALSE;
   }
   return TRUE;

} /* S5K4ECGX_set_param_exposure */

void S5K4ECGX_MIPI_GetAEAWBLock(UINT32 *pAElockRet32,UINT32 *pAWBlockRet32)
{
    *pAElockRet32 = 1;
    *pAWBlockRet32 = 1;
    SENSORDB("S5K4ECGX_MIPI_IGetAEAWBLock,AE=%d ,AWB=%d\n,",*pAElockRet32,*pAWBlockRet32);
}



UINT32 S5K4ECGX_MIPI_SensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
    //SENSORDB("[Enter]S5K4ECGX_MIPISensorSetting func:cmd = %d\n",iCmd);

    switch (iCmd)
    {
        case FID_SCENE_MODE:     //auto mode or night mode
            SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_SCENE_MODE\n");
            if (iPara == SCENE_MODE_OFF)//auto mode
            {
               S5K4ECGX_MIPI_NightMode(FALSE);
            }
            else if (iPara == SCENE_MODE_NIGHTSCENE)//night mode
            {
               S5K4ECGX_MIPI_NightMode(TRUE);
            }
            break;
        case FID_AWB_MODE:
            SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_AWB_MODE\n");
            S5K4ECGX_MIPI_set_param_wb(iPara);
            break;
        case FID_COLOR_EFFECT:
            SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_COLOR_EFFECT\n");
            S5K4ECGX_MIPI_set_param_effect(iPara);
            break;
        case FID_AE_EV:
            SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_AE_EV\n");
            S5K4ECGX_MIPI_set_param_exposure(iPara);
            break;
        case FID_AE_FLICKER:
            SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_AE_FLICKER\n");
            S5K4ECGX_MIPI_set_param_banding(iPara);
            break;
        case FID_AE_SCENE_MODE:
            SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_AE_SCENE_MODE\n");
            if (iPara == AE_MODE_OFF) {
               S5K4ECGX_MIPI_AE_Enable(KAL_FALSE);
            }
            else {
               S5K4ECGX_MIPI_AE_Enable(KAL_TRUE);
            }
            break;
        case FID_ZOOM_FACTOR:
            SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_ZOOM_FACTOR\n");
            break;
        default:
            SENSORDB("S5K4ECGX_MIPISensorSetting func:default, FID=%d\n", iCmd);
            break;
    }
    return TRUE;
}   /* S5K4ECGX_MIPISensorSetting */

UINT32 S5K4ECGX_MIPI_SetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("[Enter]S5K4ECGX S5K4ECGX_MIPI_SetVideoMode:FrameRate= %d\n",u2FrameRate);

#if 0
    unsigned int cap_en = 0;

    if(u2FrameRate==0)
    {
      SENSORDB("Disable Video Mode or dynimac fps\n");
      return ERROR_NONE;
    }

    spin_lock(&s5k4ecgx_mipi_drv_lock);
    S5K4ECGX_Driver.Preview_Width = S5K4ECGX_IMAGE_SENSOR_VDO_WIDTH_DRV;
    S5K4ECGX_Driver.Preview_Height = S5K4ECGX_IMAGE_SENSOR_VDO_HEIGHT_DRV;
    cap_en = s5k4ec_cap_enable;
    spin_unlock(&s5k4ecgx_mipi_drv_lock);

    if (cap_en)
    {
       // stop Capture
        SENSORDB("[Enter]:S5K4ECGX_MIPI_SetVideoMode: Stop Capture\r\n");
        S5K4ECGX_write_cmos_sensor(0x0028 ,0x7000);
        S5K4ECGX_write_cmos_sensor(0x002A ,0x023E);
        S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_GP_EnablePreview
        S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //REG_TC_GP_EnablePreviewChanged
        S5K4ECGX_write_cmos_sensor(0x002A ,0x0242);
        S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_GP_EnableCapture
        S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //REG_TC_GP_EnableCaptureChanged
        spin_lock(&s5k4ecgx_mipi_drv_lock);
        s5k4ec_cap_enable = 0;
        spin_unlock(&s5k4ecgx_mipi_drv_lock);
        mdelay(200);
    }

    //FCFCD000
    //S5K4ECGX_write_cmos_sensor(0xFCFC, 0xD000);
    SENSORDB("[Enter]:S5K4ECGX Preview_Mode_Setting: ReconfigPreview Size\r\n");
    S5K4ECGX_write_cmos_sensor(0x0028 ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A ,0x18AC);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //senHal_uAddColsBin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0060); //senHal_uAddColsNoBin
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x07DC); //senHal_uMinColsBin
    //S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05C0); //senHal_uMinColsNoBin

    S5K4ECGX_write_cmos_sensor(0x002A ,0x0250);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00); //REG_TC_GP_PrevReqInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05A0); //REG_TC_GP_PrevReqInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //REG_TC_GP_PrevInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00F0); //REG_TC_GP_PrevInputHeightOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00); //REG_TC_GP_CapReqInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05A0); //REG_TC_GP_CapReqInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0010); //REG_TC_GP_CapInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x00F0); //REG_TC_GP_CapInputHeightOfs

    S5K4ECGX_write_cmos_sensor(0x002A ,0x0494);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00); //REG_TC_PZOOM_ZoomInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05A0); //REG_TC_PZOOM_ZoomInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_PZOOM_ZoomInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_PZOOM_ZoomInputHeightOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0A00); //REG_TC_CZOOM_ZoomInputWidth
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x05A0); //REG_TC_CZOOM_ZoomInputHeight
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_CZOOM_ZoomInputWidthOfs
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0000); //REG_TC_CZOOM_ZoomInputHeightOfs

    S5K4ECGX_write_cmos_sensor(0x002A ,0x0262);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); // #REG_TC_GP_bUseReqInputInPre
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0001); //REG_TC_GP_bUseReqInputInCap

    S5K4ECGX_write_cmos_sensor(0x002A ,0x0A1E);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x0028); // AfitBaseVals_0__73_ 0040   Why??
    S5K4ECGX_write_cmos_sensor(0x002A ,0x0AD4);
    S5K4ECGX_write_cmos_sensor(0x0F12 ,0x003C); // AfitBaseVals_1__73_ 0060   Why??


    //Select preview 1
    S5K4ECGX_write_cmos_sensor(0x002A  ,0x0266);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);  //REG_TC_GP_ActivePrevConfig
    S5K4ECGX_write_cmos_sensor(0x002A  ,0x026A);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);  //REG_TC_GP_PrevOpenAfterChange
    S5K4ECGX_write_cmos_sensor(0x002A  ,0x0268);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);  //REG_TC_GP_PrevConfigChanged
    S5K4ECGX_write_cmos_sensor(0x002A  ,0x024E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);  //REG_TC_GP_NewConfigSync


    ////$MIPI[Width:1280,Height:720,Format:YUV422,Lane:2,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:2,DataRate:600]
    //start preview
    SENSORDB("[Enter]:S5K4ECGX Preview_Mode_Setting: Start Preview\r\n");
    S5K4ECGX_write_cmos_sensor(0x0028  ,0x7000);
    S5K4ECGX_write_cmos_sensor(0x002A  ,0x023E);
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);  //REG_TC_GP_EnablePreview
    S5K4ECGX_write_cmos_sensor(0x0F12  ,0x0001);  //REG_TC_GP_EnablePreviewChanged
#endif

    return TRUE;
}


void S5K4ECGX_MIPI_GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = S5K4ECYX_MIPICurrentStatus.iWB;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}



UINT32 S5K4ECGX_MIPI_SetMaxFramerateByScenario(
  MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate)
{
    kal_uint32 pclk;
    kal_int16 dummyLine;
    kal_uint16 lineLength,frameHeight;

#if 0
    SENSORDB("S5K4ECGX_MIPI_SetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
    switch (scenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
             pclk = S5K4ECGX_MIPI_sensor_pclk;
             lineLength = S5K3H7Y_PV_PERIOD_PIXEL_NUMS;
             frameHeight = (10 * pclk)/frameRate/lineLength;
             dummyLine = frameHeight - S5K3H7Y_PV_PERIOD_LINE_NUMS;
             s5k3h7y.sensorMode = SENSOR_MODE_PREVIEW;
             S5K3H7Y_SetDummy(0, dummyLine);
             break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
             pclk = S5K4ECGX_MIPI_sensor_pclk;
             lineLength = S5K3H7Y_PV_PERIOD_PIXEL_NUMS;
             frameHeight = (10 * pclk)/frameRate/lineLength;
             dummyLine = frameHeight - S5K3H7Y_PV_PERIOD_LINE_NUMS;
             s5k3h7y.sensorMode = SENSOR_MODE_PREVIEW;
             S5K3H7Y_SetDummy(0, dummyLine);
             break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
             pclk = S5K4ECGX_MIPI_sensor_pclk;
             lineLength = S5K3H7Y_FULL_PERIOD_PIXEL_NUMS;
             frameHeight = (10 * pclk)/frameRate/lineLength;
             dummyLine = frameHeight - S5K3H7Y_FULL_PERIOD_LINE_NUMS;
             s5k3h7y.sensorMode = SENSOR_MODE_CAPTURE;
             S5K3H7Y_SetDummy(0, dummyLine);
             break;
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
             break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
             break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
             break;
        default:
             break;
    }
#endif
  return ERROR_NONE;
}


UINT32 S5K4ECGX_MIPI_GetDefaultFramerateByScenario(
  MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate)
{
    switch (scenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
             *pframeRate = 300;
             break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
             *pframeRate = 300;
             break;
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
             *pframeRate = 300;
             break;
        default:
          break;
    }

  return ERROR_NONE;
}



UINT32 S5K4ECGX_MIPI_FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
               UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 u2Temp = 0;
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT *pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;
    //SENSORDB("S5K4ECGX_MIPI_FeatureControl+++ ID=%d\n", FeatureId);

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_RESOLUTION\n");
             *pFeatureReturnPara16++ = S5K4ECGX_IMAGE_SENSOR_FULL_WIDTH_DRV;
             *pFeatureReturnPara16 = S5K4ECGX_IMAGE_SENSOR_FULL_HEIGHT_DRV;
             *pFeatureParaLen=4;
             break;

        case SENSOR_FEATURE_GET_PERIOD:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_PERIOD\n");
             *pFeatureReturnPara16++ = S5K4ECGX_Driver.Period_PixelNum;
             *pFeatureReturnPara16 = S5K4ECGX_Driver.Period_LineNum;
             *pFeatureParaLen=4;
             break;

        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ\n");
             *pFeatureReturnPara32 = S5K4ECGX_MIPI_sensor_pclk;
             *pFeatureParaLen=4;
             break;

        case SENSOR_FEATURE_SET_ESHUTTER:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_SET_ESHUTTER\n");
             //Sophie Add: not support
             //S5K4ECGX_MIPI_SetShutter(*pFeatureData16);
             break;

        case SENSOR_FEATURE_SET_NIGHTMODE:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_SET_NIGHTMODE\n");
             S5K4ECGX_MIPI_NightMode((BOOL) *pFeatureData16);
             break;

        case SENSOR_FEATURE_SET_GAIN:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_SET_GAIN\n");
             //Sophie Add: not support
             //S5K4ECGX_MIPI_SetGain(*pFeatureData16);
             break;

        case SENSOR_FEATURE_SET_FLASHLIGHT:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_SET_FLASHLIGHT\n");
             break;

        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
             // SENSORDB("S5K4ECGX SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ\n");
             //spin_lock(&s5k4ecgx_mipi_drv_lock);
             //S5K4ECGX_MIPI_isp_master_clock = *pFeatureData32;
             //spin_unlock(&s5k4ecgx_mipi_drv_lock);
             //SENSORDB("S5K4ECGX SET_ISP_MASTER_CLOCK_FREQ: %d\n", (*pFeatureData32));
             break;

        case SENSOR_FEATURE_SET_REGISTER:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_SET_REGISTER\n");
             S5K4ECGX_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
             break;

        case SENSOR_FEATURE_GET_REGISTER:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_REGISTER\n");
             pSensorRegData->RegData = S5K4ECGX_read_cmos_sensor(pSensorRegData->RegAddr);
             break;

        case SENSOR_FEATURE_GET_CONFIG_PARA:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_CONFIG_PARA\n");
             memcpy(pSensorConfigData, &S5K4ECGXSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
             *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
             break;

        case SENSOR_FEATURE_SET_CCT_REGISTER: //phase out?
        case SENSOR_FEATURE_GET_CCT_REGISTER: //phase out?
        case SENSOR_FEATURE_SET_ENG_REGISTER: //phase out?
        case SENSOR_FEATURE_GET_ENG_REGISTER: //phase out?
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT: //phase out?
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:  //phase out?
        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA: //phase out?
        case SENSOR_FEATURE_GET_GROUP_INFO:
        case SENSOR_FEATURE_GET_ITEM_INFO:
        case SENSOR_FEATURE_SET_ITEM_INFO:
        case SENSOR_FEATURE_GET_ENG_INFO:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_ENG_INFO\n");
             break;

        case SENSOR_FEATURE_GET_GROUP_COUNT:
             *pFeatureReturnPara32++=0;
             *pFeatureParaLen=4;
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_GROUP_COUNT\n");
             break;

        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_LENS_DRIVER_ID\n");
             // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
             // if EEPROM does not exist in camera module.
             *pFeatureReturnPara32= LENS_DRIVER_ID_DO_NOT_CARE;
             *pFeatureParaLen=4;
             break;

        case SENSOR_FEATURE_SET_YUV_CMD:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_SET_YUV_CMD ID:%d\n", *pFeatureData32);
             S5K4ECGX_MIPI_SensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
             break;

        case SENSOR_FEATURE_SET_VIDEO_MODE:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_SET_VIDEO_MODE\n");
             S5K4ECGX_MIPI_SetVideoMode(*pFeatureData16);
             break;

        case SENSOR_FEATURE_CHECK_SENSOR_ID:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_CHECK_SENSOR_ID\n");
             S5K4ECGX_MIPI_GetSensorID(pFeatureReturnPara32);
             break;

        case SENSOR_FEATURE_GET_EV_AWB_REF:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_EV_AWB_REF\n");
             S5K4ECGX_MIPI_GetEvAwbRef(*pFeatureData32);
             break;

        case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
             SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN\n");
             S5K4ECGX_MIPI_GetCurAeAwbInfo(*pFeatureData32);
             break;

  #ifdef MIPI_INTERFACE
        case SENSOR_FEATURE_GET_EXIF_INFO:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_EXIF_INFO\n");
             //SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32);
             S5K4ECGX_MIPI_GetExifInfo(*pFeatureData32);
             break;
  #endif

        case SENSOR_FEATURE_SET_SLAVE_I2C_ID:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_SET_SLAVE_I2C_ID:[%d]\n",*pFeatureData32);
             S5K4ECGX_MIPI_sensor_socket = *pFeatureData32;
             break;

        case SENSOR_FEATURE_SET_TEST_PATTERN:
             //1 TODO
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_SET_TEST_PATTERN: FAIL: NOT Support\n");
             S5K4ECGX_MIPI_SetTestPatternMode((BOOL)*pFeatureData16);
             break;

        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
             //1 TODO
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO: FAIL: NOT Support\n");
             S5K4ECGX_MIPI_SetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
             break;

        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
             //SENSORDB("S5K4ECGX SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO\n");
             S5K4ECGX_MIPI_GetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
             break;

        //below is AF control
        case SENSOR_FEATURE_INITIALIZE_AF:
             //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_INITIALIZE_AF\n");
             S5K4ECGX_MIPI_AF_Init();
             break;

        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
             //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_MOVE_FOCUS_LENS\n");
             S5K4ECGX_MIPI_AF_Move_to(pFeatureReturnPara32); // not implement yet.
             break;

        case SENSOR_FEATURE_GET_AF_STATUS:
            S5K4ECGX_MIPI_AF_Get_Status(pFeatureReturnPara32);
            //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_GET_AF_STATUS=%d\n", *pFeatureReturnPara32);
            *pFeatureParaLen = 4;
             break;

        case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
             //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_SINGLE_FOCUS_MODE\n");
             S5K4ECGX_MIPI_AF_SingleShot_Start();
             break;

        case SENSOR_FEATURE_CONSTANT_AF:
             //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_CONSTANT_AF\n");
             S5K4ECGX_MIPI_AF_ContinuousShot_Start();
             break;

        case SENSOR_FEATURE_CANCEL_AF:
             //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_CANCEL_AF\n");
             S5K4ECGX_MIPI_AF_CancelFocus();
             break;

        case SENSOR_FEATURE_GET_AF_INF:
             //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_GET_AF_INF\n");
             S5K4ECGX_MIPI_AF_Get_Inf(pFeatureReturnPara32);
             *pFeatureParaLen=4;
             break;

        case SENSOR_FEATURE_GET_AF_MACRO:
             S5K4ECGX_MIPI_AF_Get_Macro(pFeatureReturnPara32);
             //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_GET_AF_MACRO=%d\n", *pFeatureReturnPara32);
             *pFeatureParaLen=4;
             break;

        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
             S5K4ECGX_MIPI_AF_Get_Max_Num_Focus_Areas(pFeatureReturnPara32);
             //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS=%d\n", *pFeatureReturnPara32);
             *pFeatureParaLen=4;
             break;

        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
             S5K4ECGX_MIPI_AE_Get_Max_Num_Metering_Areas(pFeatureReturnPara32);
             //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS=%d\n", *pFeatureReturnPara32);
             *pFeatureParaLen=4;
             break;
        case SENSOR_FEATURE_SET_AF_WINDOW:
             //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_SET_AF_WINDOW\n");
             S5K4ECGX_MIPI_AF_Set_Window(*pFeatureReturnPara32, S5K4ECGX_Driver.Preview_Width, S5K4ECGX_Driver.Preview_Height);
             break;

        case SENSOR_FEATURE_SET_AE_WINDOW:
             //SENSORDB("S5K4ECGX ~~~~SENSOR_FEATURE_SET_AE_WINDOW\n");
             S5K4ECGX_MIPI_AE_Set_Window(*pFeatureReturnPara32, S5K4ECGX_Driver.Preview_Width, S5K4ECGX_Driver.Preview_Height);
             break;

        case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
             //SENSORDB("S5K4ECGX ~~~SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO\n");
             S5K4ECGX_MIPI_GetAEAWBLock((*pFeatureData32),*(pFeatureData32+1));
             break;

        default:
             SENSORDB("S5K4ECGX FeatureControl default\n");
             break;
    }

    //SENSORDB("S5K4ECGX_MIPI_FeatureControl---\n");
    return ERROR_NONE;
} /* S5K4ECGXFeatureControl() */


SENSOR_FUNCTION_STRUCT  SensorFuncS5K4ECGX_MIPI=
{
    S5K4ECGX_MIPI_Open,             // get sensor id, set initial setting to sesnor
    S5K4ECGX_MIPI_GetInfo,          // get sensor capbility,
    S5K4ECGX_MIPI_GetResolution,    // get sensor capure/preview resolution
    S5K4ECGX_MIPI_FeatureControl,   // set shutter/gain, set/read register
    S5K4ECGX_MIPI_Control,          // change mode to preview/capture/video
    S5K4ECGX_MIPI_Close             // close, do nothing currently
};

UINT32 S5K4ECGX_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
  /* To Do : Check Sensor status here */
  if (pfFunc!=NULL)
     *pfFunc=&SensorFuncS5K4ECGX_MIPI;

  return ERROR_NONE;
} /* SensorInit() */

