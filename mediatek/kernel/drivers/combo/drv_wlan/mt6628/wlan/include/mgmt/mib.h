




#ifndef _MIB_H
#define _MIB_H




/* Entry in SMT AuthenticationAlgorithms Table: dot11AuthenticationAlgorithmsEntry */
typedef struct _DOT11_AUTHENTICATION_ALGORITHMS_ENTRY {
    BOOLEAN     dot11AuthenticationAlgorithmsEnable;    /* dot11AuthenticationAlgorithmsEntry 3 */
} DOT11_AUTHENTICATION_ALGORITHMS_ENTRY, *P_DOT11_AUTHENTICATION_ALGORITHMS_ENTRY;

/* Entry in SMT dot11RSNAConfigPairwiseCiphersTalbe Table: dot11RSNAConfigPairwiseCiphersEntry */
typedef struct _DOT11_RSNA_CONFIG_PAIRWISE_CIPHERS_ENTRY
{
    UINT_32     dot11RSNAConfigPairwiseCipher;          /* dot11RSNAConfigPairwiseCiphersEntry 2 */
    BOOLEAN     dot11RSNAConfigPairwiseCipherEnabled;   /* dot11RSNAConfigPairwiseCiphersEntry 3 */
} DOT11_RSNA_CONFIG_PAIRWISE_CIPHERS_ENTRY, *P_DOT11_RSNA_CONFIG_PAIRWISE_CIPHERS_ENTRY;

/* Entry in SMT dot11RSNAConfigAuthenticationSuitesTalbe Table: dot11RSNAConfigAuthenticationSuitesEntry */
typedef struct _DOT11_RSNA_CONFIG_AUTHENTICATION_SUITES_ENTRY
{
    UINT_32     dot11RSNAConfigAuthenticationSuite;         /* dot11RSNAConfigAuthenticationSuitesEntry 2 */
    BOOLEAN     dot11RSNAConfigAuthenticationSuiteEnabled;  /* dot11RSNAConfigAuthenticationSuitesEntry 3 */
} DOT11_RSNA_CONFIG_AUTHENTICATION_SUITES_ENTRY, *P_DOT11_RSNA_CONFIG_AUTHENTICATION_SUITES_ENTRY;

/* ----- IEEE 802.11 MIB Major sections ----- */
typedef struct _IEEE_802_11_MIB_T {
    /* dot11PrivacyTable                            (dot11smt 5) */
    UINT_8      dot11WEPDefaultKeyID;                   /* dot11PrivacyEntry 2 */
    BOOLEAN     dot11TranmitKeyAvailable;
    UINT_32     dot11WEPICVErrorCount;                  /* dot11PrivacyEntry 5 */
    UINT_32     dot11WEPExcludedCount;                  /* dot11PrivacyEntry 6 */

    /* dot11RSNAConfigTable                         (dot11smt 8) */
    UINT_32     dot11RSNAConfigGroupCipher;             /* dot11RSNAConfigEntry 4 */

    /* dot11RSNAConfigPairwiseCiphersTable          (dot11smt 9) */
    DOT11_RSNA_CONFIG_PAIRWISE_CIPHERS_ENTRY dot11RSNAConfigPairwiseCiphersTable[MAX_NUM_SUPPORTED_CIPHER_SUITES];

    /* dot11RSNAConfigAuthenticationSuitesTable     (dot11smt 10) */
    DOT11_RSNA_CONFIG_AUTHENTICATION_SUITES_ENTRY dot11RSNAConfigAuthenticationSuitesTable[MAX_NUM_SUPPORTED_AKM_SUITES];

#if 0 //SUPPORT_WAPI
    BOOLEAN            fgWapiKeyInstalled;
    PARAM_WPI_KEY_T    rWapiPairwiseKey[2];
    BOOLEAN            fgPairwiseKeyUsed[2];
    UINT_8             ucWpiActivedPWKey; /* Must be 0 or 1, by wapi spec */
    PARAM_WPI_KEY_T    rWapiGroupKey[2];
    BOOLEAN            fgGroupKeyUsed[2];
#endif
} IEEE_802_11_MIB_T, *P_IEEE_802_11_MIB_T;

/* ------------------ IEEE 802.11 non HT PHY characteristics ---------------- */
typedef const struct _NON_HT_PHY_ATTRIBUTE_T {
    UINT_16 u2SupportedRateSet;

    BOOLEAN fgIsShortPreambleOptionImplemented;

    BOOLEAN fgIsShortSlotTimeOptionImplemented;

} NON_HT_PHY_ATTRIBUTE_T, *P_NON_HT_PHY_ATTRIBUTE_T;

typedef const struct _NON_HT_ADHOC_MODE_ATTRIBUTE_T {

    ENUM_PHY_TYPE_INDEX_T ePhyTypeIndex;

    UINT_16 u2BSSBasicRateSet;

} NON_HT_ADHOC_MODE_ATTRIBUTE_T, *P_NON_HT_ADHOC_MODE_ATTRIBUTE_T;

typedef NON_HT_ADHOC_MODE_ATTRIBUTE_T NON_HT_AP_MODE_ATTRIBUTE_T;

extern NON_HT_PHY_ATTRIBUTE_T rNonHTPhyAttributes[];
extern NON_HT_ADHOC_MODE_ATTRIBUTE_T rNonHTAdHocModeAttributes[];
extern NON_HT_AP_MODE_ATTRIBUTE_T rNonHTApModeAttributes[];





#endif /* _MIB_H */

