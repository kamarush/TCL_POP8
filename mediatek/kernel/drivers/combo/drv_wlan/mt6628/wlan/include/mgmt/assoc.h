




#ifndef _ASSOC_H
#define _ASSOC_H








/*----------------------------------------------------------------------------*/
/* Routines in assoc.c                                                        */
/*----------------------------------------------------------------------------*/
WLAN_STATUS
assocSendReAssocReqFrame (
    IN P_ADAPTER_T prAdapter,
    IN P_STA_RECORD_T prStaRec
    );

WLAN_STATUS
assocCheckTxReAssocReqFrame (
    IN P_ADAPTER_T      prAdapter,
    IN P_MSDU_INFO_T    prMsduInfo
    );

WLAN_STATUS
assocCheckTxReAssocRespFrame(
    IN P_ADAPTER_T prAdapter,
    IN P_MSDU_INFO_T prMsduInfo
    );

WLAN_STATUS
assocCheckRxReAssocRspFrameStatus (
    IN P_ADAPTER_T  prAdapter,
    IN P_SW_RFB_T   prSwRfb,
    OUT PUINT_16    pu2StatusCode
    );

WLAN_STATUS
assocSendDisAssocFrame (
    IN P_ADAPTER_T    prAdapter,
    IN P_STA_RECORD_T prStaRec,
    IN UINT_16        u2ReasonCode
    );

WLAN_STATUS
assocProcessRxDisassocFrame (
    IN P_ADAPTER_T  prAdapter,
    IN P_SW_RFB_T prSwRfb,
    IN UINT_8 aucBSSID[],
    OUT PUINT_16 pu2ReasonCode
    );

WLAN_STATUS
assocProcessRxAssocReqFrame (
    IN P_ADAPTER_T  prAdapter,
    IN P_SW_RFB_T prSwRfb,
    OUT PUINT_16 pu2StatusCode
    );

WLAN_STATUS
assocSendReAssocRespFrame (
    IN P_ADAPTER_T      prAdapter,
    IN P_STA_RECORD_T   prStaRec
    );


#endif /* _ASSOC_H */

