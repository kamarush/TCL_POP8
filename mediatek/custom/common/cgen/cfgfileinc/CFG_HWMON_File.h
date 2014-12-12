



#ifndef _CFG_HWMON_FILE_H
#define _CFG_HWMON_FILE_H
#define C_HWMON_ACC_AXES    3
/*-----------------------------------------------------------------------------*/
typedef struct
{
    int offset[C_HWMON_ACC_AXES];
} NVRAM_HWMON_ACC_STRUCT;
/*-----------------------------------------------------------------------------*/
#define CFG_FILE_HWMON_ACC_REC_SIZE    sizeof(NVRAM_HWMON_ACC_STRUCT)
#define CFG_FILE_HWMON_ACC_REC_TOTAL   1
/*-----------------------------------------------------------------------------*/

#define C_HWMON_GYRO_AXES    3
/*-----------------------------------------------------------------------------*/
typedef struct
{
    int offset[C_HWMON_GYRO_AXES];
} NVRAM_HWMON_GYRO_STRUCT;
/*-----------------------------------------------------------------------------*/
#define CFG_FILE_HWMON_GYRO_REC_SIZE    sizeof(NVRAM_HWMON_GYRO_STRUCT)
#define CFG_FILE_HWMON_GYRO_REC_TOTAL   1
/*-----------------------------------------------------------------------------*/
#define C_HWMON_ALSPS_AXES    3
/*-----------------------------------------------------------------------------*/
typedef struct
{
    int ps_cali[C_HWMON_ALSPS_AXES];
} NVRAM_HWMON_PS_STRUCT;
/*-----------------------------------------------------------------------------*/
#define CFG_FILE_HWMON_PS_REC_SIZE    sizeof(NVRAM_HWMON_PS_STRUCT)
#define CFG_FILE_HWMON_PS_REC_TOTAL   1
/*-----------------------------------------------------------------------------*/

#endif


