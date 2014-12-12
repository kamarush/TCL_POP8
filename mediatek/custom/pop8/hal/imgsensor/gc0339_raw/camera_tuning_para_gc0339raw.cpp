#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_gc0339raw.h"
#include "camera_info_gc0339raw.h"
#include "camera_custom_AEPlinetable.h"
//#include "camera_custom_tsf_tbl.h"
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
        70300,    // i4R_AVG
        8906,    // i4R_STD
        92267,    // i4B_AVG
        24788,    // i4B_STD
        {  // i4P00[9]
            5383333, -2373333, -453333, -790000, 3470000, -120000, 120000, -1833333, 4270000
        },
        {  // i4P10[9]
            639683, -691536, 50691, 25718, -78211, 52494, 39217, -43419, 3040
        },
        {  // i4P01[9]
            534236, -502884, -26376, -173683, -14681, 188364, -40808, -131880, 177664
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
            1232,    // u4MinGain, 1024 base = 1x
            4096,    // u4MaxGain, 16x
            52,    // u4MiniISOGain, ISOxx  
            128,    // u4GainStepUnit, 1x/8 
            80000,    // u4PreExpUnit 
            25,    // u4PreMaxFrameRate
            80000,    // u4VideoExpUnit  
            25,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            80000,    // u4CapExpUnit 
            25,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            28,    // u4LensFno, Fno = 2.8
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
            68,    // u4AETarget
            0,    // u4StrobeAETarget
            50,    // u4InitIndex
            8,    // u4BackLightWeight
            32,    // u4HistStretchWeight
            4,    // u4AntiOverExpWeight
            2,    // u4BlackLightStrengthIndex
            2,    // u4HistStretchStrengthIndex
            2,    // u4AntiOverExpStrengthIndex
            2,    // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8},    // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM] 
            90,    // u4InDoorEV = 9.0, 10 base 
            -10,    // i4BVOffset delta BV = value/10 
            84,    // u4PreviewFlareOffset
            124,    // u4CaptureFlareOffset
            5,    // u4CaptureFlareThres
            84,    // u4VideoFlareOffset
            5,    // u4VideoFlareThres
            32,    // u4StrobeFlareOffset
            2,    // u4StrobeFlareThres
            50,    // u4PrvMaxFlareThres
            0,    // u4PrvMinFlareThres
            50,    // u4VideoMaxFlareThres
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
                663,    // i4R
                560,    // i4G
                512    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                0,    // i4X
                0    // i4Y
            },
            // Horizon
            {
                -471,    // i4X
                10    // i4Y
            },
            // A
            {
                -338,    // i4X
                -14    // i4Y
            },
            // TL84
            {
                -182,    // i4X
                -36    // i4Y
            },
            // CWF
            {
                -164,    // i4X
                -113    // i4Y
            },
            // DNP
            {
                20,    // i4X
                -58    // i4Y
            },
            // D65
            {
                95,    // i4X
                -28    // i4Y
            },
            // DF
            {
                0,    // i4X
                0    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                0,    // i4X
                0    // i4Y
            },
            // Horizon
            {
                -471,    // i4X
                17    // i4Y
            },
            // A
            {
                -338,    // i4X
                -9    // i4Y
            },
            // TL84
            {
                -183,    // i4X
                -33    // i4Y
            },
            // CWF
            {
                -166,    // i4X
                -110    // i4Y
            },
            // DNP
            {
                19,    // i4X
                -58    // i4Y
            },
            // D65
            {
                95,    // i4X
                -29    // i4Y
            },
            // DF
            {
                0,    // i4X
                0    // i4Y
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                512,    // i4R
                512,    // i4G
                512    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                981,    // i4G
                1830    // i4B
            },
            // A 
            {
                512,    // i4R
                794,    // i4G
                1279    // i4B
            },
            // TL84 
            {
                512,    // i4R
                624,    // i4G
                838    // i4B
            },
            // CWF 
            {
                512,    // i4R
                549,    // i4G
                798    // i4B
            },
            // DNP 
            {
                569,    // i4R
                512,    // i4G
                539    // i4B
            },
            // D65 
            {
                663,    // i4R
                560,    // i4G
                512    // i4B
            },
            // DF 
            {
                512,    // i4R
                512,    // i4G
                512    // i4B
            }
        },
        // Rotation matrix parameter
        {
            1,    // i4RotationAngle
            256,    // i4Cos
            4    // i4Sin
        },
        // Daylight locus parameter
        {
            -116,    // i4SlopeNumerator
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
            -210,    // i4RightBound
            -883,    // i4LeftBound
            54,    // i4UpperBound
            -46    // i4LowerBound
            },
            // Warm fluorescent
            {
            -210,    // i4RightBound
            -883,    // i4LeftBound
            -46,    // i4UpperBound
            -166    // i4LowerBound
            },
            // Fluorescent
            {
            -31,    // i4RightBound
            -210,    // i4LeftBound
            53,    // i4UpperBound
            -71    // i4LowerBound
            },
            // CWF
            {
            -31,    // i4RightBound
            -210,    // i4LeftBound
            -71,    // i4UpperBound
            -160    // i4LowerBound
            },
            // Daylight
            {
            120,    // i4RightBound
            -31,    // i4LeftBound
            51,    // i4UpperBound
            -150    // i4LowerBound
            },
            // Shade
            {
            480,    // i4RightBound
            120,    // i4LeftBound
            51,    // i4UpperBound
            -150    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            0,    // i4RightBound
            0,    // i4LeftBound
            0,    // i4UpperBound
            0    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            480,    // i4RightBound
            -883,    // i4LeftBound
            54,    // i4UpperBound
            -166    // i4LowerBound
            },
            // Daylight
            {
            145,    // i4RightBound
            -31,    // i4LeftBound
            51,    // i4UpperBound
            -150    // i4LowerBound
            },
            // Cloudy daylight
            {
            245,    // i4RightBound
            70,    // i4LeftBound
            51,    // i4UpperBound
            -150    // i4LowerBound
            },
            // Shade
            {
            345,    // i4RightBound
            70,    // i4LeftBound
            51,    // i4UpperBound
            -150    // i4LowerBound
            },
            // Twilight
            {
            -31,    // i4RightBound
            -191,    // i4LeftBound
            51,    // i4UpperBound
            -150    // i4LowerBound
            },
            // Fluorescent
            {
            145,    // i4RightBound
            -283,    // i4LeftBound
            21,    // i4UpperBound
            -160    // i4LowerBound
            },
            // Warm fluorescent
            {
            -238,    // i4RightBound
            -438,    // i4LeftBound
            21,    // i4UpperBound
            -160    // i4LowerBound
            },
            // Incandescent
            {
            -238,    // i4RightBound
            -438,    // i4LeftBound
            51,    // i4UpperBound
            -150    // i4LowerBound
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
            591,    // i4R
            512,    // i4G
            506    // i4B
            },
            // Cloudy daylight
            {
            676,    // i4R
            512,    // i4G
            440    // i4B
            },
            // Shade
            {
            723,    // i4R
            512,    // i4G
            411    // i4B
            },
            // Twilight
            {
            473,    // i4R
            512,    // i4G
            637    // i4B
            },
            // Fluorescent
            {
            514,    // i4R
            512,    // i4G
            618    // i4B
            },
            // Warm fluorescent
            {
            359,    // i4R
            512,    // i4G
            894    // i4B
            },
            // Incandescent
            {
            349,    // i4R
            512,    // i4G
            870    // i4B
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
            7068    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            5159    // i4OffsetThr
            },
            // Shade
            {
            0,    // i4SliderValue
            1341    // i4OffsetThr
            },
            // Daylight WB gain
            {
            546,    // i4R
            512,    // i4G
            518    // i4B
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
                -566,    // i4RotatedXCoordinate[0]
                -433,    // i4RotatedXCoordinate[1]
                -278,    // i4RotatedXCoordinate[2]
                -76,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
            }
        }
    },
    {0}
};

#include INCLUDE_FILENAME_ISP_LSC_PARAM
//};  //  namespace

const CAMERA_TSF_TBL_STRUCT CAMERA_TSF_DEFAULT_VALUE =
{
    #include "camera_tsf_para_gc0339raw.h"
    #include "camera_tsf_data_gc0339raw.h"
};


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
                                             sizeof(AE_PLINETABLE_T),
                                             0,
                                             sizeof(CAMERA_TSF_TBL_STRUCT)};

	ALOGE("%s gc0339 pop8 %d\n",__func__, CameraDataType);
    if (CameraDataType > CAMERA_DATA_TSF_TABLE || NULL == pDataBuf || (size < dataSize[CameraDataType]))
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
        case CAMERA_DATA_TSF_TABLE:
            memcpy(pDataBuf,&CAMERA_TSF_DEFAULT_VALUE,sizeof(CAMERA_TSF_TBL_STRUCT));
            break;
        default:
            break;
    }
    return 0;
}}; // NSFeature


