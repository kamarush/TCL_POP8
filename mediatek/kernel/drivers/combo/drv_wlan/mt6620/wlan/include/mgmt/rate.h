




#ifndef _RATE_H
#define _RATE_H








/*----------------------------------------------------------------------------*/
/* Routines in rate.c                                                         */
/*----------------------------------------------------------------------------*/
VOID
rateGetRateSetFromIEs (
    IN P_IE_SUPPORTED_RATE_T prIeSupportedRate,
    IN P_IE_EXT_SUPPORTED_RATE_T prIeExtSupportedRate,
    OUT PUINT_16 pu2OperationalRateSet,
    OUT PUINT_16 pu2BSSBasicRateSet,
    OUT PBOOLEAN pfgIsUnknownBSSBasicRate
    );

VOID
rateGetDataRatesFromRateSet (
    IN UINT_16 u2OperationalRateSet,
    IN UINT_16 u2BSSBasicRateSet,
    OUT PUINT_8 pucDataRates,
    OUT PUINT_8 pucDataRatesLen
    );

BOOLEAN
rateGetHighestRateIndexFromRateSet (
    IN UINT_16 u2RateSet,
    OUT PUINT_8 pucHighestRateIndex
    );

BOOLEAN
rateGetLowestRateIndexFromRateSet (
    IN UINT_16 u2RateSet,
    OUT PUINT_8 pucLowestRateIndex
    );


#endif /* _RATE_H */

