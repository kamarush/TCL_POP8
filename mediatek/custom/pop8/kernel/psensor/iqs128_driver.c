
#include <mach/eint.h>
#include <cust_eint.h>
#include <mach/mt_gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/sched.h>
#include <linux/rtpm_prio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <mach/mt_pm_ldo.h>
#include "cust_gpio_usage.h"
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <mach/upmu_common.h>
#include <linux/switch.h> 
#include "psensor.h"

#define IQS128_EINT_PIN         (GPIO117 | 0x80000000)
#define IQS128_EINT_PIN_M_GPIO  GPIO_MODE_00
#define IQS128_EINT_PIN_M_EINT  IQS128_EINT_PIN_M_GPIO
#define IQS128_EINT_NUM   117
#define IQS128_EINT_DEBOUNCE_CN      0
#define IQS128_EINT_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE

struct work_struct *iqs128_work;
static struct workqueue_struct *iqs128_wq;
static int iqs128_int_value=1;

static void do_iqs128_work(struct work_struct *work)
{
	send_psensor_uevent(iqs128_int_value);
}

static void iqs128_int_handler(void)
{
//	printk("<2>""---%s,%d\n",__func__,__LINE__);
	iqs128_int_value=mt_get_gpio_in(IQS128_EINT_PIN);
	queue_work(iqs128_wq, iqs128_work);
	mt_eint_set_polarity(IQS128_EINT_NUM,iqs128_int_value? MT_EINT_POL_NEG : MT_EINT_POL_POS);
}

static int iqs128_init(void)
{
	printk("<2>""---enter %s----\n",__func__);
	upmu_set_rg_vibr_vosel(0x7); //modify VIBR TO 3.3V
	upmu_set_rg_vibr_en(1);

	mt_set_gpio_mode(IQS128_EINT_PIN, IQS128_EINT_PIN_M_EINT);
	mt_set_gpio_dir(IQS128_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(IQS128_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(IQS128_EINT_PIN, GPIO_PULL_UP);
	msleep(50);

	iqs128_work = kzalloc(sizeof(typeof(*iqs128_work)), GFP_KERNEL);
	if (!iqs128_work) {
		printk("create work queue error, line: %d\n", __LINE__);
		return -1;
	}

	INIT_WORK(iqs128_work, do_iqs128_work);

	iqs128_wq = create_singlethread_workqueue("iqs128_wq");
	if (!iqs128_wq) {
		kfree(iqs128_work);
		printk("create thread error, line: %d\n", __LINE__);
		return -1;
	}

	mt_eint_set_hw_debounce(IQS128_EINT_NUM,IQS128_EINT_DEBOUNCE_CN );
	mt_eint_registration(IQS128_EINT_NUM, mt_get_gpio_in(IQS128_EINT_PIN)? EINTF_TRIGGER_FALLING : EINTF_TRIGGER_RISING, iqs128_int_handler, 1);

	mt_eint_unmask(IQS128_EINT_NUM);
	msleep(50);
	return 0;
}

struct psensor_driver_t iqs128_driver={
	.name="iqs128",
	.init=iqs128_init,
};
