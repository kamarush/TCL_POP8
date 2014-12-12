



#ifndef _NIC_INIT_CMD_EVENT_H
#define _NIC_INIT_CMD_EVENT_H



#include "gl_typedef.h"

#define INIT_CMD_STATUS_SUCCESS                 0
#define INIT_CMD_STATUS_REJECTED_INVALID_PARAMS 1
#define INIT_CMD_STATUS_REJECTED_CRC_ERROR      2
#define INIT_CMD_STATUS_REJECTED_DECRYPT_FAIL   3
#define INIT_CMD_STATUS_UNKNOWN                 4

#define EVENT_HDR_SIZE          OFFSET_OF(WIFI_EVENT_T, aucBuffer[0])

typedef enum _ENUM_INIT_CMD_ID {
    INIT_CMD_ID_DOWNLOAD_BUF = 1,
    INIT_CMD_ID_WIFI_START,
    INIT_CMD_ID_ACCESS_REG,
    INIT_CMD_ID_QUERY_PENDING_ERROR
} ENUM_INIT_CMD_ID, *P_ENUM_INIT_CMD_ID;

typedef enum _ENUM_INIT_EVENT_ID {
    INIT_EVENT_ID_CMD_RESULT = 1,
    INIT_EVENT_ID_ACCESS_REG,
    INIT_EVENT_ID_PENDING_ERROR
} ENUM_INIT_EVENT_ID, *P_ENUM_INIT_EVENT_ID;

typedef UINT_8 CMD_STATUS;

// commands
typedef struct _INIT_WIFI_CMD_T {
    UINT_8      ucCID;
    UINT_8      ucSeqNum;
    UINT_16     u2Reserved;
    UINT_8      aucBuffer[0];
} INIT_WIFI_CMD_T, *P_INIT_WIFI_CMD_T;

typedef struct _INIT_HIF_TX_HEADER_T {
    UINT_16     u2TxByteCount;
    UINT_8      ucEtherTypeOffset;
    UINT_8      ucCSflags;
    INIT_WIFI_CMD_T rInitWifiCmd;
} INIT_HIF_TX_HEADER_T, *P_INIT_HIF_TX_HEADER_T;

#define DOWNLOAD_BUF_ENCRYPTION_MODE    BIT(0)
#define DOWNLOAD_BUF_NO_CRC_CHECKING    BIT(30)
#define DOWNLOAD_BUF_ACK_OPTION         BIT(31)
typedef struct _INIT_CMD_DOWNLOAD_BUF {
    UINT_32     u4Address;
    UINT_32     u4Length;
    UINT_32     u4CRC32;
    UINT_32     u4DataMode;
    UINT_8      aucBuffer[0];
} INIT_CMD_DOWNLOAD_BUF, *P_INIT_CMD_DOWNLOAD_BUF;

typedef struct _INIT_CMD_WIFI_START {
    UINT_32     u4Override;
    UINT_32     u4Address;
} INIT_CMD_WIFI_START, *P_INIT_CMD_WIFI_START;

typedef struct _INIT_CMD_ACCESS_REG {
    UINT_8      ucSetQuery;
    UINT_8      aucReserved[3];
    UINT_32     u4Address;
    UINT_32     u4Data;
} INIT_CMD_ACCESS_REG, *P_INIT_CMD_ACCESS_REG;

// Events
typedef struct _INIT_WIFI_EVENT_T {
    UINT_16     u2RxByteCount;
    UINT_8      ucEID;
    UINT_8      ucSeqNum;
    UINT_8      aucBuffer[0];
} INIT_WIFI_EVENT_T, *P_INIT_WIFI_EVENT_T;

typedef struct _INIT_HIF_RX_HEADER_T {
    INIT_WIFI_EVENT_T rInitWifiEvent;
} INIT_HIF_RX_HEADER_T, *P_INIT_HIF_RX_HEADER_T;

typedef struct _INIT_EVENT_CMD_RESULT {
    UINT_8      ucStatus;   // 0: success 
                            // 1: rejected by invalid param 
                            // 2: rejected by incorrect CRC
                            // 3: rejected by decryption failure
                            // 4: unknown CMD
    UINT_8      aucReserved[3];
} INIT_EVENT_CMD_RESULT, *P_INIT_EVENT_CMD_RESULT, INIT_EVENT_PENDING_ERROR, *P_INIT_EVENT_PENDING_ERROR;

typedef struct _INIT_EVENT_ACCESS_REG {
    UINT_32     u4Address;
    UINT_32     u4Data;
} INIT_EVENT_ACCESS_REG, *P_INIT_EVENT_ACCESS_REG;






#endif /* _NIC_INIT_CMD_EVENT_H */

