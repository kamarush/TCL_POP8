




#ifndef CUSTOM_CFG_FILE_LID_H
#define CUSTOM_CFG_FILE_LID_H


//#include "../../../../../common/inc/cfg_file_lid.h"
//#include "../../../common/cgen/inc/CFG_File_LID.h"
#include "CFG_file_lid.h"

/* the definition of file LID */
typedef enum
{
    AP_CFG_RDCL_FILE_AUDIO_LID=AP_CFG_CUSTOM_BEGIN_LID,	//AP_CFG_CUSTOM_BEGIN_LID: this lid must not be changed, it is reserved for system.
    AP_CFG_CUSTOM_FILE_GPS_LID,
    AP_CFG_RDCL_FILE_AUDIO_COMPFLT_LID,
    AP_CFG_RDCL_FILE_AUDIO_EFFECT_LID,
    AP_CFG_RDEB_FILE_WIFI_LID,
    AP_CFG_RDEB_WIFI_CUSTOM_LID,
    AP_CFG_RDCL_FILE_AUDIO_PARAM_MED_LID,
    AP_CFG_RDCL_FILE_AUDIO_VOLUME_CUSTOM_LID,
    AP_CFG_RDCL_FILE_DUAL_MIC_CUSTOM_LID,
    AP_CFG_RDCL_FILE_AUDIO_WB_PARAM_LID,
    AP_CFG_REEB_PRODUCT_INFO_LID,
//	  AP_CFG_RDCL_FILE_META_LID,
//    AP_CFG_CUSTOM_FILE_CUSTOM1_LID,
//    AP_CFG_CUSTOM_FILE_CUSTOM2_LID,
    AP_CFG_RDCL_FILE_HEADPHONE_COMPFLT_LID,
    AP_CFG_RDCL_FILE_AUDIO_GAIN_TABLE_LID,
    AP_CFG_RDCL_FILE_AUDIO_VER1_VOLUME_CUSTOM_LID,
    AP_CFG_RDCL_FILE_AUDIO_HD_REC_PAR_LID,
    AP_CFG_RDCL_FILE_AUDIO_HD_REC_SCENE_LID,
    AP_CFG_RDCL_FILE_AUDIO_HD_REC_48K_PAR_LID,
    AP_CFG_RDCL_FILE_AUDIO_BUFFER_DC_CALIBRATION_PAR_LID,
    AP_CFG_CUSTOM_FILE_MAX_LID,
} CUSTOM_CFG_FILE_LID;


/* verno of data items */
/* audio file version */
#define AP_CFG_RDCL_FILE_AUDIO_LID_VERNO			"001"

/* GPS file version */
#define AP_CFG_CUSTOM_FILE_GPS_LID_VERNO			"000"

/* audio acf file version */
#define AP_CFG_RDCL_FILE_AUDIO_COMPFLT_LID_VERNO	"001"

/* audio hcf file version */
#define AP_CFG_RDCL_FILE_HEADPHONE_COMPFLT_LID_VERNO	"001"

/* audio effect file version */
#define AP_CFG_RDCL_FILE_AUDIO_EFFECT_LID_VERNO	"001"

/* audio med file version */
#define AP_CFG_RDCL_FILE_AUDIO_PARAM_MED_LID_VERNO  "001"

/* audio volume custom file version */
#define AP_CFG_RDCL_FILE_AUDIO_VOLUME_CUSTOM_LID_VERNO  "001"
#define AP_CFG_RDCL_FILE_AUDIO_VER1_VOLUME_CUSTOM_LID_VERNO  "001"

/* dual mic custom file version */
#define AP_CFG_RDCL_FILE_DUAL_MIC_CUSTOM_LID_VERNO  "002"

/* audio wb specch param custom file version */
#define AP_CFG_RDCL_FILE_AUDIO_WB_PARAM_LID_VERNO "001"

/* audio gain table custom file version */
#define AP_CFG_RDCL_FILE_AUDIO_GAIN_TABLE_LID_VERNO "001"

/* audio hd record par custom file version*/
#define AP_CFG_RDCL_FILE_AUDIO_HD_REC_PAR_LID_VERNO "001"
#define AP_CFG_RDCL_FILE_AUDIO_HD_REC_SCENE_LID_VERNO "001"
#define AP_CFG_RDCL_FILE_AUDIO_HD_REC_48K_PAR_LID_VERNO "001"

/* audio buffer dc calibration custom file version*/
#define AP_CFG_RDCL_FILE_AUDIO_BUFFER_DC_CALIBRATION_PAR_LID_VERNO "000"

/* META log and com port config file version */
#define AP_CFG_RDCL_FILE_META_LID_VERNO			    "000"

/* custom2 file version */
#define AP_CFG_CUSTOM_FILE_CUSTOM1_LID_VERNO			"000"
/* custom2 file version */
#define AP_CFG_CUSTOM_FILE_CUSTOM2_LID_VERNO			"000"

/* WIFI file version */
#define AP_CFG_RDEB_FILE_WIFI_LID_VERNO				"000"
/* WIFI MAC addr file version */
#define AP_CFG_RDCL_FILE_WIFI_ADDR_LID_VERNO		"000"
#define AP_CFG_RDEB_WIFI_CUSTOM_LID_VERNO				"000"
#define AP_CFG_REEB_PRODUCT_INFO_LID_VERNO      "000"

#endif /* CFG_FILE_LID_H */
