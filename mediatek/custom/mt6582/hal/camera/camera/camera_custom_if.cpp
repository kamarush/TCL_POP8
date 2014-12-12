

/*================================================================================================*/
/*   Author : Yuan Xiang																	   */
/*   Role :	Idle Screen 																	   */
/*   Reference documents : TN Idle Screen														   */
/*================================================================================================*/
/* Comments :																					   */
/* 	file	: 										   */
/* 	Labels	:																				   */
/*================================================================================================*/
/* Modifications	(month/day/year)															   */
/*================================================================================================*/
/* date	| author	   |FeatureID					|modification							   */
/*=========|==============|============================|==========================================*/
/*02/27/13 |Yuan Xiang    |PR409038-Yuanxiang-001 | No “Maker”and “Model”in image details screen*/
/*=========|==============|============================|==========================================*/


#include <stdlib.h>
#include <stdio.h>
#include "camera_custom_if.h"
//PR409038-yuanxiang-begin
#define LOG_TAG "custm_SetExif"
#include <utils/Log.h>
#include <cutils/properties.h>
//PR409038-yuanxiang-end

namespace NSCamCustom
{


#define EN_CUSTOM_EXIF_INFO
//PR409038-yuanxiang-begin
//MINT32 custom_SetExif(void **ppCustomExifTag)
//{
//#ifdef EN_CUSTOM_EXIF_INFO
//#define CUSTOM_EXIF_STRING_MAKE  "custom make"
//#define CUSTOM_EXIF_STRING_MODEL "custom model"
//#define CUSTOM_EXIF_STRING_SOFTWARE "custom software"
//static customExifInfo_t exifTag = {CUSTOM_EXIF_STRING_MAKE,CUSTOM_EXIF_STRING_MODEL,CUSTOM_EXIF_STRING_SOFTWARE};
MINT32 custom_SetExif(void **ppCustomExifTag)
{
#ifdef EN_CUSTOM_EXIF_INFO
    char model[32];
    char manufacturer[32];
    property_get("ro.product.display.model", model, "default");
    ALOGI("custom_SetExif model= %s",model);
    property_get("ro.product.manufacturer", manufacturer, "default");
    ALOGI("custom_SetExif manufacturer= %s",manufacturer);
    //#define CUSTOM_EXIF_STRING_MAKE  "custom make"
    //#define CUSTOM_EXIF_STRING_MODEL "custom model"
    static customExifInfo_t exifTag = { 0 };
    for (int i = 0; i < 32; i++) {
        if (model[i] != '\0' && i < strlen(model)) {
            exifTag.strModel[i] = (unsigned char) model[i];
        } else {
            exifTag.strModel[i] = '\0';
        }
        if (manufacturer[i] != '\0' && i < strlen(manufacturer)) {
            exifTag.strMake[i] = (unsigned char) manufacturer[i];
        } else {
            exifTag.strMake[i] = '\0';
        }
    }
//PR409038-yuanxiang-end
    if (0 != ppCustomExifTag) {
        *ppCustomExifTag = (void*)&exifTag;
    }
    return 0;
#else
    return -1;
#endif
}
//
customExif_t const&
getCustomExif()
{
    static customExif_t inst = {
        bEnCustom       :   false,  // default value: false.
        u4ExpProgram    :   0,      // default value: 0.    '0' means not defined, '1' manual control, '2' program normal
    };
    return inst;
}
//
MINT32 get_atv_disp_delay(MINT32 mode)
{
    return ((ATV_MODE_NTSC == mode)?ATV_MODE_NTSC_DELAY:((ATV_MODE_PAL == mode)?ATV_MODE_PAL_DELAY:0));
}

MINT32 get_atv_input_data()
{
    return ATV_INPUT_DATA_FORMAT;
}


#define FLASHLIGHT_CALI_LED_GAIN_PRV_TO_CAP_10X 10
MUINT32 custom_GetFlashlightGain10X(void)
{   
    // x10 , 1 mean 0.1x gain    
    //10 means no difference. use torch mode for preflash and cpaflash
    //> 10 means capture flashlight is lighter than preflash light. < 10 is opposite condition.    
    return (MUINT32)FLASHLIGHT_CALI_LED_GAIN_PRV_TO_CAP_10X;
}

MUINT32 custom_BurstFlashlightGain10X(void)
{
    return (MUINT32)FLASHLIGHT_CALI_LED_GAIN_PRV_TO_CAP_10X;
}
#define FLASHLIGHT_YUV_THRESHOlD 3.0
double custom_GetYuvFlashlightThreshold(void)
{    
    return (double)FLASHLIGHT_YUV_THRESHOlD;
}

#define FLASHLIGHT_YUV_CONVERGENCE_FRAME 7
MINT32 custom_GetYuvFlashlightFrameCnt(void)
{    
    return (int)FLASHLIGHT_YUV_CONVERGENCE_FRAME;
}

#define FLASHLIGHT_YUV_NORMAL_LEVEL 12
MINT32 custom_GetYuvFlashlightDuty(void)
{    
    return (int)FLASHLIGHT_YUV_NORMAL_LEVEL;
}

#define FLASHLIGHT_YUV_MAIN_HI_LEVEL 12
MINT32 custom_GetYuvFlashlightHighCurrentDuty(void)
{
    // if FLASHLIGHT_CALI_LED_GAIN_PRV_TO_CAP_10X > 10 (high current mode),
    // it means capture flashlight is lighter than preflash light.
    // In this case, you need to specify the level for capture flash accordingly.
    return (int)FLASHLIGHT_YUV_MAIN_HI_LEVEL;
}

#define FLASHLIGHT_YUV_MAIN_HI_TIMEOUT 500
MINT32 custom_GetYuvFlashlightHighCurrentTimeout(void)
{
    // if FLASHLIGHT_CALI_LED_GAIN_PRV_TO_CAP_10X > 10 (high current mode),
    // it means capture flashlight is lighter than preflash light.
    // In this case, you may need to set the timeout in ms in case of LED burning out.
    return (int)FLASHLIGHT_YUV_MAIN_HI_TIMEOUT;
}


#define FLASHLIGHT_YUV_STEP 7
MINT32 custom_GetYuvFlashlightStep(void)
{    
    return (int)FLASHLIGHT_YUV_STEP;
}

#define FLASHLIGHT_YUV_AF_LAMP 0
MINT32 custom_GetYuvAfLampSupport(void)
{
    // 0: indicates no AF lamp when touch AF
    // 1: indicates AF lamp support for touch AF
    return (int)FLASHLIGHT_YUV_AF_LAMP;
}

};  //NSCamCustom

