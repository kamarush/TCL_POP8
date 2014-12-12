
#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE         
#define TPD_I2C_NUMBER           0
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100

#define TPD_DELAY                (2*HZ/100)

//#define TPD_CALIBRATION_MATRIX  {962,0,0,0,1600,0,0,0};
//600x1024
//#define TPD_CALIBRATION_MATRIX_ROTATION_NORMAL  {0, -4096, 4190208, 1364, 0, 0, 0, 0};
//#define TPD_CALIBRATION_MATRIX_ROTATION_FACTORY {0, -2398, 2453504, 2329, 0, 0, 0, 0};
//768x1024
#define TPD_CALIBRATION_MATRIX_ROTATION_NORMAL  {0, 4096, 0, -4096, 0, 2453504, 0, 0};
#define TPD_CALIBRATION_MATRIX_ROTATION_FACTORY {0, 5463, 0, -3071, 0, 3141632, 0, 0};

#define TPD_POWER_SOURCE		MT6323_POWER_LDO_VGP2 //for mt8389

#define TPD_HAVE_CALIBRATION
//#define TPD_HAVE_TREMBLE_ELIMINATION

//#define TPD_HAVE_BUTTON

#if defined(CTP_BUTTON_CONFIG_FOR_JBAOL)	
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           2
	#define TPD_KEYS                { KEY_MENU, KEY_BACK}
	#define TPD_KEYS_DIM            {{80,850,160,TPD_BUTTON_HEIGH},{400,850,160,TPD_BUTTON_HEIGH}}
#elif defined(CTP_FT5406_BUTTON_CONFIG_FOR_E1808_JBL)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           2
	#define TPD_KEYS                { KEY_MENU, KEY_BACK}
	#define TPD_KEYS_DIM            {{60,850,100,TPD_BUTTON_HEIGH},{420,850,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_LY)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           2
	#define TPD_KEYS                { KEY_MENU, KEY_BACK}
	#define TPD_KEYS_DIM            {{80,850,100,TPD_BUTTON_HEIGH},{400,850,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_JH1)	
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                { KEY_MENU,KEY_HOME,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,850,100,TPD_BUTTON_HEIGH},{240,850,100,TPD_BUTTON_HEIGH},{400,850,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_GDS1)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           2
	#define TPD_KEYS                { KEY_MENU, KEY_BACK}
	#define TPD_KEYS_DIM            {{10,850,20,TPD_BUTTON_HEIGH},{470,850,20,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1808_HJY_A7)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           4
	#define TPD_KEYS                {KEY_HOME,KEY_MENU,KEY_BACK,KEY_SEARCH}
	#define TPD_KEYS_DIM            {{30,850,60,TPD_BUTTON_HEIGH},{170,850,60,TPD_BUTTON_HEIGH},{320,850,60,TPD_BUTTON_HEIGH},{450,850,60,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_HJY_A6)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{10,850,20,100},{60,850,20,100}}

#elif defined(CTP_BUTTON_CONFIG_FOR_GQ_YD)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEYS                {KEY_MENU, KEY_BACK}
	#define TPD_KEYS_DIM            {{10,850,20,TPD_BUTTON_HEIGH},{60,850,20,TPD_BUTTON_HEIGH}}
	#define TPD_KEY_COUNT           2

#elif defined(CTP_BUTTON_CONFIG_FOR_HLX)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           4
	#define TPD_KEYS                {KEY_HOME,KEY_MENU,KEY_BACK,KEY_SEARCH}
	#define TPD_KEYS_DIM            {{60,850,100,TPD_BUTTON_HEIGH},{180,850,100,TPD_BUTTON_HEIGH},{300,850,100,TPD_BUTTON_HEIGH},{420,850,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_NBX1)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEYS                {KEY_MENU, KEY_BACK}
	#define TPD_KEYS_DIM            {{80,850,100,TPD_BUTTON_HEIGH},{400,850,100,TPD_BUTTON_HEIGH}}
	#define TPD_KEY_COUNT           2

#elif defined(CTP_BUTTON_CONFIG_FOR_JDT)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEYS                {KEY_BACK,KEY_HOME,KEY_MENU}
	#define TPD_KEYS_DIM            {{50,850,100,TPD_BUTTON_HEIGH},{250,850,100,TPD_BUTTON_HEIGH},{430,850,100,TPD_BUTTON_HEIGH}}
	#define TPD_KEY_COUNT           3

#elif defined(CTP_BUTTON_CONFIG_FOR_JHGG)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEYS                {KEY_MENU,KEY_HOME,KEY_BACK,KEY_SEARCH}
	#define TPD_KEYS_DIM            {{50,850,100,TPD_BUTTON_HEIGH},{170,850,120,TPD_BUTTON_HEIGH},{300,850,100,TPD_BUTTON_HEIGH},{430,850,100,TPD_BUTTON_HEIGH}}
	#define TPD_KEY_COUNT           4

#elif defined(CTP_BUTTON_CONFIG_FOR_JBS_YS)	
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{60,850,100,TPD_BUTTON_HEIGH},{420,850,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_ZY_YK)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEYS                {KEY_MENU, KEY_BACK}
	#define TPD_KEYS_DIM            {{80,850,100,TPD_BUTTON_HEIGH},{400,850,100,TPD_BUTTON_HEIGH}}
	#define TPD_KEY_COUNT           2

#elif defined(CTP_BUTTON_CONFIG_FOR_GXQ_G5)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEYS                {KEY_SEND,KEY_MENU,KEY_HOME,KEY_BACK,KEY_SEARCH}
	#define TPD_KEYS_DIM            {{50,850,90,TPD_BUTTON_HEIGH},{135,850,90,TPD_BUTTON_HEIGH},{240,850,90,TPD_BUTTON_HEIGH},{335,850,90,TPD_BUTTON_HEIGH},{430,850,100,TPD_BUTTON_HEIGH}}
	#define TPD_KEY_COUNT           5

#elif defined(CTP_BUTTON_CONFIG_FOR_ND1BAT)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEYS                {KEY_MENU,KEY_HOME,KEY_BACK,KEY_SEARCH}
	#define TPD_KEYS_DIM            {{60,850,60,TPD_BUTTON_HEIGH},{180,850,60,TPD_BUTTON_HEIGH},{310,850,60,TPD_BUTTON_HEIGH},{435,850,60,TPD_BUTTON_HEIGH}}
	#define TPD_KEY_COUNT           4

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_ND_MJK)
	#define CONFIG_FOR_FWVGA
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                {KEY_MENU,KEY_HOME,KEY_BACK}
	#define TPD_KEYS_DIM            {{100,920,100,TPD_BUTTON_HEIGH},{230,920,100,TPD_BUTTON_HEIGH},{380,920,100,TPD_BUTTON_HEIGH}}
#elif defined(CTP_BUTTON_CONFIG_FOR_XGTX1)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           2
	#define TPD_KEYS                { KEY_MENU, KEY_BACK}
	#define TPD_KEYS_DIM            {{80,850,100,TPD_BUTTON_HEIGH},{400,850,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_GWDZ_JL)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           2
	#define TPD_KEYS                { KEY_MENU, KEY_BACK}
	#define TPD_KEYS_DIM            {{340,859,1,TPD_BUTTON_HEIGH},{400,850,80,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_ZXD_XJD)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           4
	#define TPD_KEYS                {KEY_MENU,KEY_HOME,KEY_BACK,KEY_SEARCH}
	#define TPD_KEYS_DIM            {{80,850,60,TPD_BUTTON_HEIGH},{180,850,60,TPD_BUTTON_HEIGH},{280,850,60,TPD_BUTTON_HEIGH},{380,850,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_ZXD_HGDZ)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            4
	#define TPD_KEYS                {KEY_MENU,KEY_HOME,KEY_BACK,KEY_SEARCH}
	#define TPD_KEYS_DIM            {{120,850,40,80},{170,850,40,80},{220,850,40,80},{270,850,40,80}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_ZLH)
	#define TPD_RES_X                320
	#define TPD_RES_Y                480
	#define TPD_BUTTON_HEIGH        (60)
	#define TPD_KEY_COUNT           2
	#define TPD_KEYS                {KEY_MENU, KEY_BACK}
	#define TPD_KEYS_DIM            {{50,530,80,TPD_BUTTON_HEIGH},{260,530,80,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_JXLC_BT)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                {KEY_BACK,KEY_HOME,KEY_MENU}
	#define TPD_KEYS_DIM            {{50,850,100,TPD_BUTTON_HEIGH},{250,850,100,TPD_BUTTON_HEIGH},{430,850,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_YM_MD)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           4
	#define TPD_KEYS                {KEY_HOME,KEY_MENU,KEY_BACK,KEY_SEARCH}
	#define TPD_KEYS_DIM            {{60,850,120,TPD_BUTTON_HEIGH},{180,850,120,TPD_BUTTON_HEIGH},{300,850,120,TPD_BUTTON_HEIGH},{420,850,120,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_YM_DM)
	#define TPD_BUTTON_HEIGH        (60)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{60,830,80,TPD_BUTTON_HEIGH},{415,850,80,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_DF_XJD)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           4
	#define TPD_KEYS                {KEY_SEARCH,KEY_MENU,KEY_HOME,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,850,100,TPD_BUTTON_HEIGH},{190,850,80,TPD_BUTTON_HEIGH},{280,850,100,TPD_BUTTON_HEIGH},{400,850,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_YM_MD2)
	#define TPD_BUTTON_HEIGH        (60)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{60,830,80,TPD_BUTTON_HEIGH},{420,850,80,TPD_BUTTON_HEIGH}}
       
#elif defined(CTP_BUTTON_CONFIG_FOR_E1811_JBAOL_CM)
	#define CONFIG_FOR_QHD 
	#define TPD_BUTTON_HEIGH        (60)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                {KEY_BACK,KEY_HOME,KEY_MENU}
	#define TPD_KEYS_DIM            {{81,999,80,TPD_BUTTON_HEIGH},{297,999,80,TPD_BUTTON_HEIGH},{459,999,80,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1811_ZLH_CM)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (60)
	#define TPD_KEY_COUNT            3
	#define TPD_KEYS                {KEY_BACK,KEY_HOME,KEY_MENU,}
	#define TPD_KEYS_DIM            {{80,1030,80,TPD_BUTTON_HEIGH},{300,1030,80,TPD_BUTTON_HEIGH},{450,1030,80,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_QXH_SD)
	#define TPD_BUTTON_HEIGH        (60)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{100,850,100,TPD_BUTTON_HEIGH},{380,850,100,TPD_BUTTON_HEIGH}}
       
#elif defined(CTP_BUTTON_CONFIG_FOR_E1811_JWT_XL)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            4
	#define TPD_KEYS                {KEY_MENU,KEY_HOME,KEY_BACK,KEY_SEARCH}
	#define TPD_KEYS_DIM            {{80,850,80,TPD_BUTTON_HEIGH},{190,850,80,TPD_BUTTON_HEIGH},{280,850,80,TPD_BUTTON_HEIGH},{410,850,80,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_ZL_DZT)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            4
	#define TPD_KEYS                {KEY_MENU,KEY_HOME,KEY_SEARCH,KEY_BACK}
	#define TPD_KEYS_DIM            {{60,850,100,TPD_BUTTON_HEIGH},{180,850,100,TPD_BUTTON_HEIGH},{300,850,100,TPD_BUTTON_HEIGH},{420,850,100,TPD_BUTTON_HEIGH}}       

#elif defined(CTP_BUTTON_CONFIG_FOR_E1809C_JBAIL_JMS)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,850,100,TPD_BUTTON_HEIGH},{400,850,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_FM_MD)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                { KEY_MENU, KEY_HOME,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,1000,160,TPD_BUTTON_HEIGH},{240,1000,160,TPD_BUTTON_HEIGH},{400,1000,160,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_MG)
	#define CONFIG_FOR_QHD	
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            4
	#define TPD_KEYS                {KEY_HOME,KEY_MENU,KEY_BACK,KEY_SEARCH}
	#define TPD_KEYS_DIM            {{70,1020,100,TPD_BUTTON_HEIGH},{225,1020,100,TPD_BUTTON_HEIGH},{360,1020,100,TPD_BUTTON_HEIGH},{480,1020,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1811_JBAOL_CM2)  
	#define CONFIG_FOR_QHD 
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,1000,100,100},{460,1000,100,100}}  

#elif defined(CTP_BUTTON_CONFIG_FOR_E1811_ZGT_JF)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,938,100,100},{400,938,100,100}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1811_ME_DM)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,1050,80,100},{460,1050,80,100}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1811_FWVGA_LY_MD)
	#define CONFIG_FOR_FWVGA
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                { KEY_MENU, KEY_HOME,KEY_BACK}
	#define TPD_KEYS_DIM            {{100,900,20,TPD_BUTTON_HEIGH},{50,900,20,TPD_BUTTON_HEIGH},{10,900,20,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1811_XL_QC)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                { KEY_MENU, KEY_HOME,KEY_BACK}
	#define TPD_KEYS_DIM            {{450,1030,100,TPD_BUTTON_HEIGH},{250,1030,100,TPD_BUTTON_HEIGH},{50,1030,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1811_HT_HZ)
	#define TPD_BUTTON_HEIGH        (60)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{20,890,40,TPD_BUTTON_HEIGH},{60,890,40,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_ZGT_CM)
	#define CONFIG_FOR_FWVGA
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,940,100,TPD_BUTTON_HEIGH},{400,940,100,TPD_BUTTON_HEIGH}}
#elif defined(CTP_BUTTON_CONFIG_FOR_E1811_GQ_YJ)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                { KEY_MENU, KEY_HOME,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,850,80,TPD_BUTTON_HEIGH},{240,850,80,TPD_BUTTON_HEIGH},{400,850,80,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_YCD)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{70,1000,100,80},{460,1000,100,80}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_MG2)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            4
	#define TPD_KEYS                {KEY_HOME,KEY_MENU,KEY_BACK,KEY_SEARCH}
	#define TPD_KEYS_DIM            {{80,1040,100,TPD_BUTTON_HEIGH},{240,1040,100,TPD_BUTTON_HEIGH},{400,1040,100,TPD_BUTTON_HEIGH},{560,1040,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1811_HJY_JL)
	#define CONFIG_FOR_FWVGA	
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                { KEY_MENU, KEY_HOME,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,950,80,TPD_BUTTON_HEIGH},{240,950,80,TPD_BUTTON_HEIGH},{400,950,80,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_HT_QC)
	#define CONFIG_FOR_FWVGA
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{180,920,40,80},{300,920,40,80}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1908_JBAIL_JMS)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,850,100,TPD_BUTTON_HEIGH},{400,850,100,TPD_BUTTON_HEIGH}}
       
#elif defined(CTP_BUTTON_CONFIG_FOR_E1901_HYDY_QD)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{70,1010,100,TPD_BUTTON_HEIGH},{480,1010,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1901_AD_HXD)
	#define CONFIG_FOR_FWVGA
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{70,920,100,TPD_BUTTON_HEIGH},{420,920,100,TPD_BUTTON_HEIGH}}
       
#elif defined(CTP_BUTTON_CONFIG_FOR_E1901_GQ_CM)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,1030,100,TPD_BUTTON_HEIGH},{470,1030,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_MG_SXGD)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{200,1010,100,100},{400,1010,100,100}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_OK_HL)
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{50,850,80,80},{450,850,80,80}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1901_HYDY_CM)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,1030,100,TPD_BUTTON_HEIGH},{470,1030,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1901_JBAIL_JMS)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT            2
	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,1030,100,TPD_BUTTON_HEIGH},{470,1030,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1901_GQ_JMS)
	#define CONFIG_FOR_QHD
        #define TPD_BUTTON_HEIGH        (100)
        #define TPD_KEY_COUNT            2
        #define TPD_KEYS                {KEY_MENU,KEY_BACK}
        #define TPD_KEYS_DIM            {{80,1010,100,TPD_BUTTON_HEIGH},{460,1010,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1910_SMT)
	#define CONFIG_FOR_WSVGA
	#define TPD_BUTTON_HEIGH        (120)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                {KEY_HOME, KEY_MENU, KEY_BACK}
	#define TPD_KEYS_DIM            {{160,1050,100,TPD_BUTTON_HEIGH},{300,1050,100,TPD_BUTTON_HEIGH},{440,1050,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1910_JHZT_WGJ)
	#define CONFIG_FOR_WSVGA
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                { KEY_BACK, KEY_HOME,KEY_MENU}
	#define TPD_KEYS_DIM            {{600,1060,60,60},{525,1060,60,60},{475,1060,60,60}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_OK_HLTQ)
       #define TPD_BUTTON_HEIGH        (100)
       #define TPD_KEY_COUNT            2
       #define TPD_KEYS                {KEY_MENU,KEY_BACK}
       #define TPD_KEYS_DIM            {{60,850,80,TPD_BUTTON_HEIGH},{450,850,80,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_JHGG_LH)
	#define CONFIG_FOR_FWVGA	
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                {KEY_BACK,KEY_HOME,KEY_MENU}
	#define TPD_KEYS_DIM            {{80,920,100,TPD_BUTTON_HEIGH},{250,920,100,TPD_BUTTON_HEIGH},{430,920,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_LY_MD)
	#define CONFIG_FOR_FWVGA
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           2
	#define TPD_KEYS                {KEY_MENU, KEY_BACK,}
	#define TPD_KEYS_DIM            {{70,1020,100,TPD_BUTTON_HEIGH},{460,1020,20,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1911_ND_JSD)
	#define CONFIG_FOR_QHD
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                { KEY_MENU, KEY_HOME,KEY_BACK}
	#define TPD_KEYS_DIM            {{100,1030,100,TPD_BUTTON_HEIGH},{250,1030,100,TPD_BUTTON_HEIGH},{420,1030,100,TPD_BUTTON_HEIGH}}

#elif defined(CTP_BUTTON_CONFIG_FOR_E1901_TZWX_CM)
	#define CONFIG_FOR_QHD
       	#define TPD_BUTTON_HEIGH        (100)
       	#define TPD_KEY_COUNT            2
       	#define TPD_KEYS                {KEY_MENU,KEY_BACK}
       	#define TPD_KEYS_DIM            {{110,1030,100,TPD_BUTTON_HEIGH},{430,1030,100,TPD_BUTTON_HEIGH}}

#else
	#define TPD_BUTTON_HEIGH        (100)
	#define TPD_KEY_COUNT           3
	#define TPD_KEYS                { KEY_MENU, KEY_HOME,KEY_BACK}
	#define TPD_KEYS_DIM            {{80,850,160,TPD_BUTTON_HEIGH},{240,850,160,TPD_BUTTON_HEIGH},{400,850,160,TPD_BUTTON_HEIGH}}
#endif

/* for different ctp resolution */
#if defined(CONFIG_FOR_QHD)
	#define TPD_RES_X                540
	#define TPD_RES_Y                960

#elif defined(CONFIG_FOR_FWVGA)
	#define TPD_RES_X                480
	#define TPD_RES_Y                854
#elif defined(CONFIG_FOR_WSVGA)
	#define TPD_RES_X                600
	#define TPD_RES_Y                1024
#else 
	#define TPD_RES_X                1280//768//480
	#define TPD_RES_Y                800//800
#endif

#endif /* TOUCHPANEL_H__ */

