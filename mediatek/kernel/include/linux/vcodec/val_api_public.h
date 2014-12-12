/**
 * @file
 *   val_api_public.h
 *
 * @par Project:
 *   Video
 *
 * @par Description:
 *   Video Abstraction Layer API for external use
 *
 * @par Author:
 *   Jackal Chen (mtk02532)
 *
 * @par $Revision: #1 $
 * @par $Modtime:$
 * @par $Log:$
 *
 */

#ifndef _VAL_API_PUBLIC_H_
#define _VAL_API_PUBLIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "val_types_public.h"

VAL_UINT32_T eVideoInitMVA(VAL_VOID_T** a_pvHandle);
VAL_UINT32_T eVideoAllocMVA(VAL_VOID_T* a_pvHandle, VAL_UINT32_T a_u4Va, VAL_UINT32_T* ap_u4Pa, VAL_UINT32_T a_u4Size, VAL_VCODEC_M4U_BUFFER_CONFIG_T *a_pvM4uConfig);
VAL_UINT32_T eVideoFreeMVA(VAL_VOID_T* a_pvHandle, VAL_UINT32_T a_u4Va, VAL_UINT32_T a_u4Pa, VAL_UINT32_T a_u4Size, VAL_VCODEC_M4U_BUFFER_CONFIG_T *a_pvM4uConfig);
VAL_UINT32_T eVideoDeInitMVA(VAL_VOID_T* a_pvHandle);

VAL_INT32_T eVideoGetM4UModuleID(VAL_UINT32_T u4MemType);

#ifdef __cplusplus
}
#endif

#endif // #ifndef _VAL_API_PUBLIC_H_
