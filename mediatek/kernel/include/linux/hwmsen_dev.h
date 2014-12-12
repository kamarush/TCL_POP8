#ifndef __HWMSEN_DEV_H__ 
#define __HWMSEN_DEV_H__

#include <linux/types.h> 
#include <linux/hwmsensor.h>


/*define sensor operator type---------------------------------------------------*/
#define SENSOR_DELAY	0X01
#define	SENSOR_ENABLE	0X02
#define	SENSOR_GET_DATA	0X04


#define SENSOR_STATUS_UNRELIABLE        0
#define SENSOR_STATUS_ACCURACY_LOW      1
#define SENSOR_STATUS_ACCURACY_MEDIUM   2
#define SENSOR_STATUS_ACCURACY_HIGH     3

#define GRAVITY_EARTH_1000           9807	// about (9.80665f)*1000


struct hwmsen_object {
    void *self;
	int polling;
	int (*sensor_operate)(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout);
};

struct sensor_init_info
{
    char *name;
	int (*init)(void);
	int (*uninit)(void);
	struct platform_driver* platform_diver_addr;
};


/*----------------------------------------------------------------------------*/
extern int hwmsen_attach(int sensor, struct hwmsen_object *obj);
extern int hwmsen_detach(int sensor);
extern int hwmsen_get_interrupt_data(int sensor, hwm_sensor_data *data);

#if defined(MTK_AUTO_DETECT_ACCELEROMETER)//
extern int hwmsen_gsensor_add(struct sensor_init_info* obj) ;
#endif

#if defined(MTK_AUTO_DETECT_MAGNETOMETER)
extern int hwmsen_msensor_add(struct sensor_init_info* obj);
#endif
/*----------------------------------------------------------------------------*/
#endif 
