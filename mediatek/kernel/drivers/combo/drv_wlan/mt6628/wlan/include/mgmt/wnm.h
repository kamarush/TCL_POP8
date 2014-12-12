




#ifndef _WNM_H
#define _WNM_H





typedef struct _TIMINGMSMT_PARAM_T {
	  BOOLEAN     fgInitiator;
	  UINT_8      ucTrigger;	  
    UINT_8      ucDialogToken;              /* Dialog Token */
    UINT_8      ucFollowUpDialogToken;      /* Follow Up Dialog Token */
    UINT_32     u4ToD;                      /* Timestamp of Departure [10ns] */
    UINT_32     u4ToA;                      /* Timestamp of Arrival [10ns] */    
} TIMINGMSMT_PARAM_T, *P_TIMINGMSMT_PARAM_T;





VOID
wnmWNMAction (
    IN P_ADAPTER_T                  prAdapter,
    IN P_SW_RFB_T                   prSwRfb
    );

VOID
wnmReportTimingMeas (
    IN P_ADAPTER_T         prAdapter,
    IN UINT_8              ucStaRecIndex,
    IN UINT_32             u4ToD,
    IN UINT_32             u4ToA
    );

#define WNM_UNIT_TEST 1

#if WNM_UNIT_TEST
VOID wnmTimingMeasUnitTest1(P_ADAPTER_T prAdapter, UINT_8 ucStaRecIndex);
#endif


#endif /* _WNM_H */

