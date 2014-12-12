




#ifndef _VERSION_H
#define _VERSION_H



#ifndef NIC_AUTHOR
#define NIC_AUTHOR      "NIC_AUTHOR"
#endif
#ifndef NIC_DESC
#define NIC_DESC        "NIC_DESC"
#endif

#ifndef NIC_NAME
    #if defined(MT6620)
        #define NIC_NAME            "MT6620"
        #define NIC_DEVICE_ID       "MT6620"
		#define NIC_DEVICE_ID_LOW   "mt6620"
    #elif defined(MT5931)
        #define NIC_NAME            "MT5931"
        #define NIC_DEVICE_ID       "MT5931"
		#define NIC_DEVICE_ID_LOW   "mt5931"
    #elif defined(MT6628)
        #define NIC_NAME            "MT6582"
        #define NIC_DEVICE_ID       "MT6582"
		#define NIC_DEVICE_ID_LOW   "mt6582"
    #endif
#endif

/* NIC driver information */
#define NIC_VENDOR                      "MediaTek Inc."
#define NIC_VENDOR_OUI                  {0x00, 0x0C, 0xE7}

#if defined(MT6620)
    #define NIC_PRODUCT_NAME                "MediaTek Inc. MT6620 Wireless LAN Adapter"
    #define NIC_DRIVER_NAME                 "MediaTek Inc. MT6620 Wireless LAN Adapter Driver"
#elif defined(MT5931)
    #define NIC_PRODUCT_NAME                "MediaTek Inc. MT5931 Wireless LAN Adapter"
    #define NIC_DRIVER_NAME                 "MediaTek Inc. MT5931 Wireless LAN Adapter Driver"
#elif defined(MT6628)
//    #define NIC_PRODUCT_NAME                "MediaTek Inc. MT6628 Wireless LAN Adapter"
//    #define NIC_DRIVER_NAME                 "MediaTek Inc. MT6628 Wireless LAN Adapter Driver"
    #define NIC_PRODUCT_NAME                "MediaTek Inc. MT6582 Wireless LAN Adapter"
    #define NIC_DRIVER_NAME                 "MediaTek Inc. MT6582 Wireless LAN Adapter Driver"
#endif

/* Define our driver version */
#define NIC_DRIVER_MAJOR_VERSION        2
#define NIC_DRIVER_MINOR_VERSION        0
#define NIC_DRIVER_VERSION              2,0,1,1
#define NIC_DRIVER_VERSION_STRING       "2.0.1.1"









#endif /* _VERSION_H */

