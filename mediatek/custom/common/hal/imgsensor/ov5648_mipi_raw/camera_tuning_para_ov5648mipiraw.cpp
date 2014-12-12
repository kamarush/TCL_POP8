#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_ov5648mipiraw.h"
#include "camera_info_ov5648mipiraw.h"
#include "camera_custom_AEPlinetable.h"
const NVRAM_CAMERA_ISP_PARAM_STRUCT CAMERA_ISP_DEFAULT_VALUE =
{{
    //Version
    Version: NVRAM_CAMERA_PARA_FILE_VERSION,
    //SensorId
    SensorId: SENSOR_ID,
    ISPComm:{
        {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    },
    ISPPca:{
        #include INCLUDE_FILENAME_ISP_PCA_PARAM
    },
    ISPRegs:{
        #include INCLUDE_FILENAME_ISP_REGS_PARAM
        },
    ISPMfbMixer:{{
        {//00: MFB mixer for ISO 100
            0x00000000, 0x00000000
        },
        {//01: MFB mixer for ISO 200
            0x00000000, 0x00000000
        },
        {//02: MFB mixer for ISO 400
            0x00000000, 0x00000000
        },
        {//03: MFB mixer for ISO 800
            0x00000000, 0x00000000
        },
        {//04: MFB mixer for ISO 1600
            0x00000000, 0x00000000
        },
        {//05: MFB mixer for ISO 2400
            0x00000000, 0x00000000
        },
        {//06: MFB mixer for ISO 3200
            0x00000000, 0x00000000
        }
    }},
    ISPCcmPoly22:{
        69000,    // i4R_AVG
        13801,    // i4R_STD
        82367,    // i4B_AVG
        23841,    // i4B_STD
        {  // i4P00[9]
            4826667, -1426667, -840000, -823333, 3586667, -203333, -30000, -1756667, 4343333
        },
        {  // i4P10[9]
            914675, -1052563, 137888, -43918, -301048, 344966, -56186, 577315, -519166
        },
        {  // i4P01[9]
            576395, -573656, -2739, -136836, -155225, 292061, -43917, -169407, 220464
        },
        {  // i4P20[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P02[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    }
}};

const NVRAM_CAMERA_3A_STRUCT CAMERA_3A_NVRAM_DEFAULT_VALUE =
{
    NVRAM_CAMERA_3A_FILE_VERSION, // u4Version
    SENSOR_ID, // SensorId

    // AE NVRAM
    {
        // rDevicesInfo
        {
            1136,    // u4MinGain, 1024 base = 1x
            10240,    // u4MaxGain, 16x
            51,    // u4MiniISOGain, ISOxx  
            64,    // u4GainStepUnit, 1x/8 
            35,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            35,    // u4VideoExpUnit  
            30,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            35,    // u4CapExpUnit 
            15,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            24,    // u4LensFno, Fno = 2.8
            350    // u4FocusLength_100x
        },
        // rHistConfig
        {
            2,    // u4HistHighThres
            40,    // u4HistLowThres
            2,    // u4MostBrightRatio
            1,    // u4MostDarkRatio
            160,    // u4CentralHighBound
            20,    // u4CentralLowBound
            {240, 230, 220, 210, 200},    // u4OverExpThres[AE_CCT_STRENGTH_NUM] 
            {86, 108, 128, 148, 170},    // u4HistStretchThres[AE_CCT_STRENGTH_NUM] 
            {18, 22, 26, 30, 34}    // u4BlackLightThres[AE_CCT_STRENGTH_NUM] 
        },
        // rCCTConfig
        {
            TRUE,    // bEnableBlackLight
            TRUE,    // bEnableHistStretch
            FALSE,    // bEnableAntiOverExposure
            TRUE,    // bEnableTimeLPF
            FALSE,    // bEnableCaptureThres
            FALSE,    // bEnableVideoThres
            FALSE,    // bEnableStrobeThres
            55,    // u4AETarget
            0,    // u4StrobeAETarget
            50,    // u4InitIndex
            4,    // u4BackLightWeight
            32,    // u4HistStretchWeight
            4,    // u4AntiOverExpWeight
            2,    // u4BlackLightStrengthIndex
            2,    // u4HistStretchStrengthIndex
            2,    // u4AntiOverExpStrengthIndex
            2,    // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8},    // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM] 
            90,    // u4InDoorEV = 9.0, 10 base 
            -10,    // i4BVOffset delta BV = value/10 
            32,//64,    // u4PreviewFlareOffset
            32,//64,    // u4CaptureFlareOffset
            5,    // u4CaptureFlareThres
            4,    // u4VideoFlareOffset
            4,    // u4VideoFlareThres
            64,    // u4StrobeFlareOffset
            2,    // u4StrobeFlareThres
            8,    // u4PrvMaxFlareThres
            0,    // u4PrvMinFlareThres
            8,    // u4VideoMaxFlareThres
            0,    // u4VideoMinFlareThres
            18,    // u4FlatnessThres    // 10 base for flatness condition.
            75    // u4FlatnessStrength
        }
    },
    // AWB NVRAM
    {
        // AWB calibration data
        {
            // rUnitGain (unit gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rGoldenGain (golden sample gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rTuningUnitGain (Tuning sample unit gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rD65Gain (D65 WB gain: 1.0 = 512)
            {
                775,    // i4R
                512,    // i4G
                580    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                17,    // i4X
                -377    // i4Y
            },
            // Horizon
            {
                -436,    // i4X
                -272    // i4Y
            },
            // A
            {
                -312,    // i4X
                -268    // i4Y
            },
            // TL84
            {
                -167,    // i4X
                -300    // i4Y
            },
            // CWF
            {
                -95,    // i4X
                -400    // i4Y
            },
            // DNP
            {
                -3,    // i4X
                -266    // i4Y
            },
            // D65
            {
                107,    // i4X
                -199    // i4Y
            },
            // DF
            {
                32,    // i4X
                -325    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                -48,    // i4X
                -374    // i4Y
            },
            // Horizon
            {
                -476,    // i4X
                -193    // i4Y
            },
            // A
            {
                -353,    // i4X
                -210    // i4Y
            },
            // TL84
            {
                -216,    // i4X
                -267    // i4Y
            },
            // CWF
            {
                -162,    // i4X
                -377    // i4Y
            },
            // DNP
            {
                -49,    // i4X
                -261    // i4Y
            },
            // D65
            {
                71,    // i4X
                -214    // i4Y
            },
            // DF
            {
                -24,    // i4X
                -325    // i4Y
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                873,    // i4R
                512,    // i4G
                833    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                639,    // i4G
                1666    // i4B
            },
            // A 
            {
                512,    // i4R
                544,    // i4G
                1193    // i4B
            },
            // TL84 
            {
                613,    // i4R
                512,    // i4G
                963    // i4B
            },
            // CWF 
            {
                774,    // i4R
                512,    // i4G
                1002    // i4B
            },
            // DNP 
            {
                731,    // i4R
                512,    // i4G
                736    // i4B
            },
            // D65 
            {
                775,    // i4R
                512,    // i4G
                580    // i4B
            },
            // DF 
            {
                830,    // i4R
                512,    // i4G
                761    // i4B
            }
        },
        // Rotation matrix parameter
        {
            10,    // i4RotationAngle
            252,    // i4Cos
            44    // i4Sin
        },
        // Daylight locus parameter
        {
            -180,    // i4SlopeNumerator
            128    // i4SlopeDenominator
        },
        // AWB light area
        {
            // Strobe:FIXME
            {
            0,    // i4RightBound
            0,    // i4LeftBound
            0,    // i4UpperBound
            0    // i4LowerBound
            },
            // Tungsten
            {
            -266,    // i4RightBound
            -916,    // i4LeftBound
            -151,    // i4UpperBound
            -251    // i4LowerBound
            },
            // Warm fluorescent
            {
            -266,    // i4RightBound
            -916,    // i4LeftBound
            -251,    // i4UpperBound
            -371    // i4LowerBound
            },
            // Fluorescent
            {
            -99,    // i4RightBound
            -266,    // i4LeftBound
            -142,    // i4UpperBound
            -322    // i4LowerBound
            },
            // CWF
            {
            -99,    // i4RightBound
            -266,    // i4LeftBound
            -322,    // i4UpperBound
            -427    // i4LowerBound
            },
            // Daylight
            {
            96,    // i4RightBound
            -99,    // i4LeftBound
            -134,    // i4UpperBound
            -294    // i4LowerBound
            },
            // Shade
            {
            456,    // i4RightBound
            96,    // i4LeftBound
            -134,    // i4UpperBound
            -294    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            120,    // i4RightBound
            -99,    // i4LeftBound
            -294,    // i4UpperBound
            -400    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            456,    // i4RightBound
            -916,    // i4LeftBound
            -109,    // i4UpperBound
            -427    // i4LowerBound
            },
            // Daylight
            {
            121,    // i4RightBound
            -99,    // i4LeftBound
            -134,    // i4UpperBound
            -294    // i4LowerBound
            },
            // Cloudy daylight
            {
            221,    // i4RightBound
            46,    // i4LeftBound
            -134,    // i4UpperBound
            -294    // i4LowerBound
            },
            // Shade
            {
            321,    // i4RightBound
            46,    // i4LeftBound
            -134,    // i4UpperBound
            -294    // i4LowerBound
            },
            // Twilight
            {
            -99,    // i4RightBound
            -259,    // i4LeftBound
            -134,    // i4UpperBound
            -294    // i4LowerBound
            },
            // Fluorescent
            {
            121,    // i4RightBound
            -316,    // i4LeftBound
            -164,    // i4UpperBound
            -427    // i4LowerBound
            },
            // Warm fluorescent
            {
            -253,    // i4RightBound
            -453,    // i4LeftBound
            -164,    // i4UpperBound
            -427    // i4LowerBound
            },
            // Incandescent
            {
            -253,    // i4RightBound
            -453,    // i4LeftBound
            -134,    // i4UpperBound
            -294    // i4LowerBound
            },
            // Gray World
            {
            5000,    // i4RightBound
            -5000,    // i4LeftBound
            5000,    // i4UpperBound
            -5000    // i4LowerBound
            }
        },
        // PWB default gain	
        {
            // Daylight
            {
            725,    // i4R
            512,    // i4G
            637    // i4B
            },
            // Cloudy daylight
            {
            830,    // i4R
            512,    // i4G
            526    // i4B
            },
            // Shade
            {
            877,    // i4R
            512,    // i4G
            486    // i4B
            },
            // Twilight
            {
            588,    // i4R
            512,    // i4G
            858    // i4B
            },
            // Fluorescent
            {
            731,    // i4R
            512,    // i4G
            826    // i4B
            },
            // Warm fluorescent
            {
            552,    // i4R
            512,    // i4G
            1233    // i4B
            },
            // Incandescent
            {
            485,    // i4R
            512,    // i4G
            1127    // i4B
            },
            // Gray World
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        // AWB preference color	
        {
            // Tungsten
            {
            0,    // i4SliderValue
            6668    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            5662    // i4OffsetThr
            },
            // Shade
            {
            0,    // i4SliderValue
            1346    // i4OffsetThr
            },
            // Daylight WB gain
            {
            687,    // i4R
            512,    // i4G
            548    // i4B
            },
            // Preference gain: strobe
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: tungsten
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: warm fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: CWF
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: daylight
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: shade
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: daylight fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        {// CCT estimation
            {// CCT
                2300,    // i4CCT[0]
                2850,    // i4CCT[1]
                4100,    // i4CCT[2]
                5100,    // i4CCT[3]
                6500    // i4CCT[4]
            },
            {// Rotated X coordinate
                -547,    // i4RotatedXCoordinate[0]
                -424,    // i4RotatedXCoordinate[1]
                -287,    // i4RotatedXCoordinate[2]
                -120,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
            }
        }
    },
    {0}
};

#include INCLUDE_FILENAME_ISP_LSC_PARAM
//};  //  namespace


typedef NSFeature::RAWSensorInfo<SENSOR_ID> SensorInfoSingleton_T;


namespace NSFeature {
template <>
UINT32
SensorInfoSingleton_T::
impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const
{
    UINT32 dataSize[CAMERA_DATA_TYPE_NUM] = {sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT),
                                             sizeof(NVRAM_CAMERA_3A_STRUCT),
                                             sizeof(NVRAM_CAMERA_SHADING_STRUCT),
                                             sizeof(NVRAM_LENS_PARA_STRUCT),
                                             sizeof(AE_PLINETABLE_T)};

    if (CameraDataType > CAMERA_DATA_AE_PLINETABLE || NULL == pDataBuf || (size < dataSize[CameraDataType]))
    {
        return 1;
    }

    switch(CameraDataType)
    {
        case CAMERA_NVRAM_DATA_ISP:
            memcpy(pDataBuf,&CAMERA_ISP_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_3A:
            memcpy(pDataBuf,&CAMERA_3A_NVRAM_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_3A_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_SHADING:
            memcpy(pDataBuf,&CAMERA_SHADING_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_SHADING_STRUCT));
            break;
        case CAMERA_DATA_AE_PLINETABLE:
            memcpy(pDataBuf,&g_PlineTableMapping,sizeof(AE_PLINETABLE_T));
            break;
        default:
            break;
    }
    return 0;
}}; // NSFeature


