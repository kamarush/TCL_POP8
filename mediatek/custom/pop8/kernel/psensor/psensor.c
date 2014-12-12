/*
psensor driver
*/
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/switch.h>

#include "psensor.h"

struct switch_dev *psensor_dev=NULL;

extern struct psensor_driver_t iqs128_driver;
extern struct psensor_driver_t sx9500_driver;

struct psensor_driver_t *psensor_driver_list[]={
	&iqs128_driver,
	&sx9500_driver,
};

int psensor_driver_num=sizeof(psensor_driver_list)/sizeof(int);

void send_psensor_uevent(enum PSENSOR_STATUS val)
{
	char *envp[2];
	char psensor[20];

	if(psensor_dev==NULL)
		return;
	psensor_dev->state=val;
	snprintf(psensor, sizeof(psensor), "SWITCH_STATE=%d", psensor_dev->state);
	envp[0]=psensor;
	envp[1]=NULL;

	if(!psensor_dev->state)
		kobject_uevent_env(&psensor_dev->dev->kobj, KOBJ_ADD, envp);
	else
		kobject_uevent_env(&psensor_dev->dev->kobj, KOBJ_REMOVE, envp);
}

static int __init psensor_init(void)
{
	int ret=0,i=0;

	printk("<2>""--------------enter %s-----------------\n",__func__);
	psensor_dev=kzalloc(sizeof(struct switch_dev), GFP_KERNEL);
	if (psensor_dev == NULL){
		printk("!!!!!!!!psensor dev alloc fail\n");
		return -1;
	}

	psensor_dev->name="psensor";
	psensor_dev->state=PSENSOR_STATUS_FAR;

	ret = switch_dev_register(psensor_dev);
	if (ret < 0){
		printk("register switch dev fail,ret=%d\n",ret);
		return-2;
	}

	send_psensor_uevent(PSENSOR_STATUS_FAR);

	for(i=0;i<psensor_driver_num;i++){
		if(psensor_driver_list[i] && psensor_driver_list[i]->init){
			ret=psensor_driver_list[i]->init();
			printk("<2>""psensor %s loaded,ret=%d\n",psensor_driver_list[i]->name,ret);
		}
	}

	printk("<2>""-------------- %s finish-----------------\n",__func__);
	return 0;
}

static void __exit psensor_exit(void)
{
	printk("%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);
}

module_init(psensor_init);
module_exit(psensor_exit);
MODULE_LICENSE("GPL");
