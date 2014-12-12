#ifndef LIS33DE_H
#define LIS33DE_H
	 
#include <linux/ioctl.h>

#define LIS33DE_I2C_SLAVE_ADDR		0x38//0x38<-> SD0=GND;0x3A<-> SD0=High
	 
	 /* LIS33DE Register Map  (Please refer to LIS33DE Specifications) */
#define LIS33DE_REG_CTL_REG1		0x20
#define LIS33DE_REG_CTL_REG2		0x21
#define LIS33DE_REG_CTL_REG3		0x22
#define LIS33DE_REG_DATAX0		    0x28
#define LIS33DE_REG_OUT_X		    0x29
#define LIS33DE_REG_OUT_Y		    0x2B
#define LIS33DE_REG_OUT_Z		    0x2D


#define LIS33DE_FIXED_DEVID			0xE5
	 
//#define LIS33DE_BW_200HZ			0x0C
#define LIS33DE_BW_400HZ			0x80 //400 or 100 on other choise //changed
//#define LIS33DE_BW_50HZ				0x0A

#define	LIS33DE_FULLRANG_LSB		0XFF
	 
#define LIS33DE_MEASURE_MODE		0x40	//changed 
#define LIS33DE_DATA_READY			0x07    //changed
	 
//#define LIS33DE_FULL_RES			0x08
//#define LIS33DE_RANGE_2G			0x00
//#define LIS33DE_RANGE_4G			0x01
#define LIS33DE_RANGE_8G			0x20 //8g or 2g no ohter choise//changed
//#define LIS33DE_RANGE_16G			0x03
#define LIS33DE_SELF_TEST           0x10 //changed
	 
#define LIS33DE_STREAM_MODE			0x80
#define LIS33DE_SAMPLES_15			0x0F
	 
#define LIS33DE_FS_8G_LSB_G			64
#define LIS33DE_FS_4G_LSB_G			128
#define LIS33DE_FS_2G_LSB_G			256
	 
#define LIS33DE_LEFT_JUSTIFY		0x04
#define LIS33DE_RIGHT_JUSTIFY		0x00
	 
	 
#define LIS33DE_SUCCESS						0
#define LIS33DE_ERR_I2C						-1
#define LIS33DE_ERR_STATUS					-3
#define LIS33DE_ERR_SETUP_FAILURE			-4
#define LIS33DE_ERR_GETGSENSORDATA			-5
#define LIS33DE_ERR_IDENTIFICATION			-6
	 
	 
	 
#define LIS33DE_BUFSIZE				256
	 
#endif

