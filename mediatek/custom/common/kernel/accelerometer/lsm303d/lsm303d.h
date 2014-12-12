

#ifndef LSM303D
#define LSM303D
#include <linux/ioctl.h>

#define	I2C_AUTO_INCREMENT	(0x80)



#define	LSM303D_REG_CTL0			0x1F	  
#define LSM303D_REG_DEVID			0x0F
#define	LSM303D_REG_BW_RATE			0x20
#define LSM303D_REG_DATA_FORMAT		0x21
#define LSM303D_REG_DATA_FILTER		0x21
#define LSM303D_REG_DATAX0			0x28


#define LSM303D_REG_OFSX            0XFF




#define LSM303D_FIXED_DEVID			0x49

/* Accelerometer Sensor Full Scale */
#define	LSM303D_ACC_FS_MASK	(0x18)
#define LSM303D_ACC_FS_2G 	(0x00)	/* Full scale 2g */
#define LSM303D_ACC_FS_4G 	(0x08)	/* Full scale 4g */
#define LSM303D_ACC_FS_8G 	(0x10)	/* Full scale 8g */
#define LSM303D_ACC_FS_16G	(0x18)	/* Full scale 16g */

/* Sensitivity */
#define SENSITIVITY_ACC_2G	(16*1024)	/**	60ug/LSB	*/
#define SENSITIVITY_ACC_4G	(8*1024)	/**	120ug/LSB	*/
#define SENSITIVITY_ACC_8G	(4*1024)	/**	240ug/LSB	*/
#define SENSITIVITY_ACC_16G	(2*1024)	/**	480ug/LSB	*/


//#define LSM303D_BW_200HZ			0x0C
//#define LSM303D_BW_100HZ			0x0B
//#define LSM303D_BW_50HZ				0x0A
/* ODR */
#define ODR_ACC_MASK		(0XF0)	/* Mask for odr change on acc */
#define LSM303D_ACC_ODR_OFF	(0x00)  /* Power down */
#define LSM303D_ACC_ODR3_125 (0x10)  /* 3.25Hz output data rate */
#define LSM303D_ACC_ODR6_25	(0x20)  /* 6.25Hz output data rate */
#define LSM303D_ACC_ODR12_5	(0x30)  /* 12.5Hz output data rate */
#define LSM303D_ACC_ODR25	(0x40)  /* 25Hz output data rate */
#define LSM303D_ACC_ODR50	(0x50)  /* 50Hz output data rate */
#define LSM303D_ACC_ODR100	(0x60)  /* 100Hz output data rate */
#define LSM303D_ACC_ODR200	(0x70)  /* 200Hz output data rate */
#define LSM303D_ACC_ODR400	(0x80)  /* 400Hz output data rate */
#define LSM303D_ACC_ODR800	(0x90)  /* 800Hz output data rate */
#define LSM303D_ACC_ODR1600	(0xA0)  /* 1600Hz output data rate */
#define LSM303D_ACC_ODR_ENABLE (0x07)


/* Accelerometer Filter */
#define LSM303D_ACC_FILTER_MASK	(0xC0)
#define FILTER_773		773
#define FILTER_362		362
#define FILTER_194		194
#define FILTER_50		50


#define LSM303D_SUCCESS						0
#define LSM303D_ERR_I2C						-1
#define LSM303D_ERR_STATUS					-3
#define LSM303D_ERR_SETUP_FAILURE			-4
#define LSM303D_ERR_GETGSENSORDATA			-5
#define LSM303D_ERR_IDENTIFICATION			-6
	  
	  
	  
#define LSM303D_BUFSIZE				256
	  
#endif



 /////////////////////////

	 
	 /* ADXL345 Register Map  (Please refer to ADXL345 Specifications) */


