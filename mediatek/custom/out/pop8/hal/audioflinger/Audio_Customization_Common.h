
#ifndef AUDIO_CUSTOMIZATION_COMMON_H
#define AUDIO_CUSTOMIZATION_COMMON_H

#define DEVICE_MAX_VOLUME           (12)
#define DEVICE_VOICE_MAX_VOLUME     (12)
#define DEVICE_MIN_VOLUME           (-32)
#define DEVICE_VOICE_MIN_VOLUME     (-32)
#define DEVICE_VOLUME_RANGE     (64)
#define DEVICE_VOLUME_STEP (256)

#define BOOT_ANIMATION_VOLUME       (0.25)

#define USE_REFMIC_IN_LOUDSPK       (0)

#define ENABLE_AUDIO_COMPENSATION_FILTER

#define ENABLE_AUDIO_DRC_SPEAKER


#define ENABLE_HEADPHONE_COMPENSATION_FILTER
#define HEADPHONE_COMPENSATION_FLT_MODE (4)


#define ENABLE_AUDIO_SW_STEREO_TO_MONO


#define ENABLE_HIGH_SAMPLERATE_RECORD


//#define FORCE_CAMERA_SHUTTER_SOUND_AUDIBLE


//#define ENABLE_STEREO_SPEAKER


//#define ALL_USING_VOICEBUFFER_INCALL


//#define AUDIO_HQA_SUPPORT

//#define ENABLE_CAMERA_SOUND_FORCED_SET

#define AUDIO_DROP_FRAME_COUNT_NORMAL 5
#define AUDIO_DROP_FRAME_COUNT_RECORD 5
#define AUDIO_DROP_FRAME_COUNT_CTS 5

#define CHANNEL_STATUS_COPY_BIT (1)

#define CHANNEL_STATUS_CATEGORY_CODE  (0x00)


#define DEFAULT_HDRecordEnhanceParas \
	0, 479, 16388, 36892, 37124, 8192,  768, 0,  4048, 2245, 611, 0, 0, 0, 0, 8192


#define MagiLoudness_TE_mode (0x0)


#define DEFAULT_HDRecordCompenFilter \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0
#endif
