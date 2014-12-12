/**
 * @file
 *   val_types_public.h
 *
 * @par Project:
 *   Video
 *
 * @par Description:
 *   Video Abstraction Layer Type Definitions for external use
 *
 * @par Author:
 *   Jackal Chen (mtk02532)
 *
 * @par $Revision: #1 $
 * @par $Modtime:$
 * @par $Log:$
 *
 */

#ifndef _VAL_TYPES_PUBLIC_H_
#define _VAL_TYPES_PUBLIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#define VCODEC_THREAD_MAX_NUM 16

/*=============================================================================
 *                              Type definition
 *===========================================================================*/

typedef void                VAL_VOID_T;         ///< void type definition
typedef char                VAL_BOOL_T;         ///< char type definition
typedef char                VAL_CHAR_T;         ///< char type definition
typedef signed char         VAL_INT8_T;         ///< signed char type definition
typedef signed short        VAL_INT16_T;        ///< signed short type definition
typedef signed long         VAL_INT32_T;        ///< signed int type definition
typedef unsigned char       VAL_UCHAR_T;        ///< unsigned char type definition
typedef unsigned char       VAL_UINT8_T;        ///< unsigned char type definition
typedef unsigned short      VAL_UINT16_T;       ///< unsigned short definition
typedef unsigned long       VAL_UINT32_T;       ///< unsigned int type definition
typedef unsigned long long  VAL_UINT64_T;       ///< unsigned long long type definition
typedef long long           VAL_INT64_T;        ///< long long type definition
typedef unsigned int        VAL_HANDLE_T;       ///< handle type definition

#define VAL_NULL        (0)     ///< VAL_NULL = 0
#define VAL_TRUE        (1)     ///< VAL_TRUE = 1
#define VAL_FALSE       (0)     ///< VAL_FALSE = 0

#define VAL_RESOLUTION_CHANGED       (2)


/**
 * @par Enumeration
 *   VAL_MEM_CODEC_T
 * @par Description
 *   This is the item in VAL_MEMORY_T for video encoder or video decoder
 */
typedef enum _VAL_MEM_CODEC_T
{
    VAL_MEM_CODEC_FOR_VENC = 0,                 ///< Memory foe Video Encoder
    VAL_MEM_CODEC_FOR_VDEC,                     ///< Memory for Video Decoder
    VAL_MEM_CODEC_MAX = 0xFFFFFFFF              ///< Max Value
} VAL_MEM_CODEC_T;


/**
 * @par Enumeration
 *   VAL_CHIP_NAME_T
 * @par Description
 *   This is the item for query chip name for HAL interface
 */
typedef enum _VAL_CHIP_NAME_T
{
    VAL_CHIP_NAME_MT6516 = 0,                   ///< MT6516
    VAL_CHIP_NAME_MT6572,                       ///< MT6572
    VAL_CHIP_NAME_MT6573,                       ///< MT6573
    VAL_CHIP_NAME_MT6575,                       ///< MT6575
    VAL_CHIP_NAME_MT6577,                       ///< MT6577
    VAL_CHIP_NAME_MT6589,                       ///< MT6589
    VAL_CHIP_NAME_MT6582,                       ///< MT6582
    VAL_CHIP_NAME_MT8135,                       ///< MT8135
    VAL_CHIP_NAME_ROME,                         ///< ROME
    VAL_CHIP_NAME_MAX = 0xFFFFFFFF              ///< Max Value
} VAL_CHIP_NAME_T;


/**
 * @par Enumeration
 *   VAL_DRIVER_TYPE_T
 * @par Description
 *   This is the item in VAL_OBJECT_T for open driver type and
 *                    in VAL_CLOCK_T for clock setting and
 *                    in VAL_ISR_T for irq line setting
 */
typedef enum _VAL_DRIVER_TYPE_T
{
    VAL_DRIVER_TYPE_NONE = 0,                   ///< None
    VAL_DRIVER_TYPE_MP4_ENC,                    ///< MP4 encoder
    VAL_DRIVER_TYPE_MP4_DEC,                    ///< MP4 decoder
    VAL_DRIVER_TYPE_H263_ENC,                   ///< H.263 encoder
    VAL_DRIVER_TYPE_H263_DEC,                   ///< H.263 decoder
    VAL_DRIVER_TYPE_H264_ENC,                   ///< H.264 encoder
    VAL_DRIVER_TYPE_H264_DEC,                   ///< H.264 decoder
    VAL_DRIVER_TYPE_SORENSON_SPARK_DEC,         ///< Sorenson Spark decoder
    VAL_DRIVER_TYPE_VC1_SP_DEC,                 ///< VC-1 simple profile decoder
    VAL_DRIVER_TYPE_RV9_DEC,                    ///< RV9 decoder
    VAL_DRIVER_TYPE_MP1_MP2_DEC,                ///< MPEG1/2 decoder
    VAL_DRIVER_TYPE_XVID_DEC,                   ///< Xvid decoder
    VAL_DRIVER_TYPE_DIVX4_DIVX5_DEC,            ///< Divx4/5 decoder
    VAL_DRIVER_TYPE_VC1_MP_WMV9_DEC,            ///< VC-1 main profile (WMV9) decoder
    VAL_DRIVER_TYPE_RV8_DEC,                    ///< RV8 decoder
    VAL_DRIVER_TYPE_WMV7_DEC,                   ///< WMV7 decoder
    VAL_DRIVER_TYPE_WMV8_DEC,                   ///< WMV8 decoder
    VAL_DRIVER_TYPE_AVS_DEC,                    ///< AVS decoder
    VAL_DRIVER_TYPE_DIVX_3_11_DEC,              ///< Divx3.11 decoder
    VAL_DRIVER_TYPE_H264_DEC_MAIN,              ///< H.264 main profile decoder (due to different packet) == 20
    VAL_DRIVER_TYPE_H264_DEC_MAIN_CABAC,        ///< H.264 main profile decoder for CABAC type but packet is the same, just for reload.
    VAL_DRIVER_TYPE_VP8_DEC,                    ///< VP8 decoder
    VAL_DRIVER_TYPE_VP8_ENC,                    ///< VP8 encoder
    VAL_DRIVER_TYPE_MP2_DEC,                    ///< MPEG2 decoder
    VAL_DRIVER_TYPE_VC1_ADV_DEC,                ///< VC1 advance decoder
    VAL_DRIVER_TYPE_VC1_DEC,                    ///< VC1 simple/main/advance decoder
    VAL_DRIVER_TYPE_JPEG_ENC,                   ///< JPEG encoder
    VAL_DRIVER_TYPE_HEVC_ENC,                        ///< HEVC encoder
    VAL_DRIVER_TYPE_HEVC_DEC,                        ///< HEVC decoder
    VAL_DRIVER_TYPE_MAX = 0xFFFFFFFF            ///< Max driver type
} VAL_DRIVER_TYPE_T;


/**
 * @par Enumeration
 *   VAL_RESULT_T
 * @par Description
 *   This is the return status of each OSAL function
 */
typedef enum _VAL_RESULT_T
{
    VAL_RESULT_NO_ERROR = 0,                    ///< The function work successfully
    VAL_RESULT_INVALID_DRIVER,                  ///< Error due to invalid driver
    VAL_RESULT_INVALID_PARAMETER,               ///< Error due to invalid parameter
    VAL_RESULT_INVALID_MEMORY,                  ///< Error due to invalid memory
    VAL_RESULT_INVALID_ISR,                     ///< Error due to invalid isr request
    VAL_RESULT_ISR_TIMEOUT,                     ///< Error due to invalid isr request
    VAL_RESULT_UNKNOWN_ERROR,                   ///< Unknown error
    VAL_RESULT_RESTARTSYS,                      ///< Restart sys
    VAL_RESULT_MAX = 0xFFFFFFFF                 ///< Max result
} VAL_RESULT_T;


/**
 * @par Enumeration
 *   VAL_MEM_ALIGN_T
 * @par Description
 *   This is the item in VAL_MEMORY_T for allocation memory byte alignment
 */
typedef enum _VAL_MEM_ALIGN_T
{
    VAL_MEM_ALIGN_1 = 1,                        ///< 1 byte alignment
    VAL_MEM_ALIGN_2 = (1<<1),                   ///< 2 byte alignment
    VAL_MEM_ALIGN_4 = (1<<2),                   ///< 4 byte alignment
    VAL_MEM_ALIGN_8 = (1<<3),                   ///< 8 byte alignment
    VAL_MEM_ALIGN_16 = (1<<4),                  ///< 16 byte alignment
    VAL_MEM_ALIGN_32 = (1<<5),                  ///< 32 byte alignment
    VAL_MEM_ALIGN_64 = (1<<6),                  ///< 64 byte alignment
    VAL_MEM_ALIGN_128 = (1<<7),                 ///< 128 byte alignment
    VAL_MEM_ALIGN_256 = (1<<8),                 ///< 256 byte alignment
    VAL_MEM_ALIGN_512 = (1<<9),                 ///< 512 byte alignment
    VAL_MEM_ALIGN_1K = (1<<10),                 ///< 1K byte alignment
    VAL_MEM_ALIGN_2K = (1<<11),                 ///< 2K byte alignment
    VAL_MEM_ALIGN_4K = (1<<12),                 ///< 4K byte alignment
    VAL_MEM_ALIGN_8K = (1<<13),                 ///< 8K byte alignment
    VAL_MEM_ALIGN_MAX = 0xFFFFFFFF              ///< Max memory byte alignment
} VAL_MEM_ALIGN_T;


/**
 * @par Enumeration
 *   VAL_MEM_TYPE_T
 * @par Description
 *   This is the item in VAL_MEMORY_T for allocation memory type
 */
typedef enum _VAL_MEM_TYPE_T
{
    VAL_MEM_TYPE_FOR_SW = 0,                    ///< External memory foe SW
    VAL_MEM_TYPE_FOR_HW_CACHEABLE,              ///< External memory for HW Cacheable
    VAL_MEM_TYPE_FOR_HW_CACHEABLE_MCI,          ///< External memory for HW Cacheable, with MCI port config
    VAL_MEM_TYPE_FOR_HW_NONCACHEABLE,           ///< External memory for HW Non-Cacheable
    VAL_MEM_TYPE_MAX = 0xFFFFFFFF               ///< Max memory type
} VAL_MEM_TYPE_T;


/**
 * @par Structure
 *  VAL_MEM_ADDR_T
 * @par Description
 *  This is a structure for memory address, includes virtual, physical address, and size.
 */
typedef struct _VAL_MEM_ADDR_T
{
    VAL_UINT32_T    u4VA;                       ///< [IN/OUT] virtual address
    VAL_UINT32_T    u4PA;                       ///< [IN/OUT] physical address
    VAL_UINT32_T    u4Size;                     ///< [IN/OUT] size
} VAL_MEM_ADDR_T;


typedef struct
{
    VAL_UINT32_T    u4VCodecThreadNum;
    VAL_UINT32_T    u4VCodecThreadID[VCODEC_THREAD_MAX_NUM];
} VAL_VCODEC_THREAD_ID_T;


typedef struct
{
    unsigned long long  _cpu_idle_time;         // [out]
    unsigned long long  _thread_cpu_time;       // [out]
    unsigned long long  _sched_clock;           // [out]
    unsigned int        _inst_count;
} VAL_VCODEC_CPU_LOADING_INFO_T;


typedef struct
{
    int limited_freq;                           // [in]
    int limited_cpu;                            // [in]
    int enable;                                 // [in]
} VAL_VCODEC_CPU_OPP_LIMIT_T;


typedef struct
{
    VAL_MEM_CODEC_T eMemCodec;
    VAL_UINT32_T    cache_coherent;
    VAL_UINT32_T    security;
} VAL_VCODEC_M4U_BUFFER_CONFIG_T;


/**
 * @par Structure
 *  VAL_MEMORY_T
 * @par Description
 *  This is a parameter for eVideoMemAlloc(), eVideoMemFree()
 */
typedef struct _VAL_MEMORY_T
{
    VAL_MEM_TYPE_T  eMemType;                   ///< [IN]     The allocation memory type
    VAL_UINT32_T    u4MemSize;                  ///< [IN]     The size of memory allocation
    VAL_VOID_T      *pvMemVa;                   ///< [IN/OUT] The memory virtual address
    VAL_VOID_T      *pvMemPa;                   ///< [IN/OUT] The memory physical address
    VAL_MEM_ALIGN_T eAlignment;                 ///< [IN]     The memory byte alignment setting
    VAL_VOID_T      *pvAlignMemVa;              ///< [IN/OUT] The align memory virtual address
    VAL_VOID_T      *pvAlignMemPa;              ///< [IN/OUT] The align memory physical address
    VAL_MEM_CODEC_T eMemCodec;                  ///< [IN]     The memory codec for VENC or VDEC
    VAL_VOID_T      *pvReserved;                ///< [IN/OUT] The reserved parameter
    VAL_UINT32_T    u4ReservedSize;             ///< [IN]     The size of reserved parameter structure
#ifdef __EARLY_PORTING__
    VAL_VOID_T      *pvReservedPmem;            ///< [IN/OUT] The reserved parameter
#endif    
} VAL_MEMORY_T;


typedef struct __VAL_RECORD_SIZE_T
{
    VAL_UINT32_T    u4FrmWidth;
    VAL_UINT32_T    u4FrmHeight;
    VAL_UINT32_T    u4BufWidth;
    VAL_UINT32_T    u4BufHeight;
} VAL_RECORD_SIZE_T;


#ifdef __cplusplus
}
#endif

#endif // #ifndef _VAL_TYPES_PUBLIC_H_
