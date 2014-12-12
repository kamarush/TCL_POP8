
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/mt_gpio.h>
#include <cust_eint.h>
#include <mach/eint.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#include <linux/cdev.h>
#include <linux/switch.h>
#include <linux/tcl_config.h>
#include "psensor.h"
#include "sx9500.h"

#define DRIVER_NAME "sx9500"

#define SX9500_CONTROL_BY_GPIO 1

#if TCL_PROTO_PCB
#define SX9500_TXEN_PIN         (GPIO46 | 0x80000000)
#define SX9500_NRST_PIN         (GPIO43 | 0x80000000)
#endif

#define SX9500_EINT_PIN         (GPIO5 | 0x80000000)
#define SX9500_EINT_PIN_M_GPIO  GPIO_MODE_00
#define SX9500_EINT_PIN_M_EINT  SX9500_EINT_PIN_M_GPIO
#define SX9500_EINT_NUM   5
#define SX9500_EINT_DEBOUNCE_CN      0
#define SX9500_EINT_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE

typedef struct sx9500_st sx9500_t, *psx9500_t;
struct sx9500_st
{
	void *bus; /* either i2c_client or spi_client */
  /* Function Pointers */
	int (*init)(psx9500_t this); /* (re)initialize device */
  /* since we are trying to avoid knowing registers, create a pointer to a
   * common read register which would be to read what the interrupt source
   * is from 
   */
	int (*refreshStatus)(psx9500_t this); /* read register status */
	int (*get_nirq_low)(void); /* get whether nirq is low (platform data) */
	struct delayed_work dworker; /* work struct for worker function */
};

static psx9500_t g_sx9500=NULL;

/*! \fn static int write_register(psx86XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct 
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx9500_t this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;
	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;
	if (this && this->bus) {
		i2c = this->bus;

		returnValue = i2c_master_send(i2c,buffer,2);
		dev_dbg(&i2c->dev,"write_register Address: 0x%x Value: 0x%x Return: %d\n",
				address,value,returnValue);
	}
	return returnValue;
}


/*! \fn static int read_register(psx86XX_t this, u8 address, u8 *value) 
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct 
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to 
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(psx9500_t this, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;
	if (this && value && this->bus) {
		i2c = this->bus;
		returnValue = i2c_smbus_read_byte_data(i2c,address);
		dev_dbg(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",address,returnValue);
		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		} else {
			return returnValue;
		}
	}
	return -ENOMEM;
}

/*! \fn static int read_regStat(psx86XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s) 
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct 
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx9500_t this)
{
	u8 data = 0;
	int ret=0;

	if (!this)
		return -1;

	ret=read_register(this,SX9500_IRQSTAT_REG,&data);

	if(ret==0)
		return (data & 0x00FF);

	return ret;
}

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct 
 */
static void hw_init(psx9500_t this)
{
	psx9500_platform_data_t pdata = 0;
  	struct i2c_client *client = 0;
	int i = 0;
	u8 read=0;
	/* configure device */
	if(!this){
		printk("<2>""%s: i2c client is err!!!\n",__func__);
		return;
	}

	client=this->bus;
	if(!client){
		printk("<2>""%s: platform data is err!!!\n",__func__);
		return;
	}

	pdata=client->dev.platform_data;
	if (pdata){
		while ( i < pdata->i2c_reg_num) {
		/* Write all registers/values contained in i2c_reg */
			printk("<2>""Going to Write Reg: 0x%x Value: 0x%x\n",pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
			//      msleep(3);        
			write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
			read_register(this,pdata->pi2c_reg[i].reg,&read);
			
			printk("<2>""read Reg: 0x%x Value: 0x%x\n",pdata->pi2c_reg[i].reg,read);
			i++;
		}
	} else
		printk("<2>""ERROR! platform data 0x%p\n");
}
/*********************************************************************/

/*********************************************************************/
/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct 
* \return Value return value from the write register
 */
static int manual_offset_calibration(psx9500_t this)
{
	s32 returnValue = 0;
	returnValue = write_register(this,SX9500_IRQSTAT_REG,0xFF);
	return returnValue;
}

static int sx9500_initialize(psx9500_t this)
{
	if (!this) {
		printk("<2>""%s: can't get data\n",__func__);
		return -1;
	}

	/* prepare reset by disabling any irq handling */
	mt_eint_mask(SX9500_EINT_NUM);	
	/* perform a reset */
	write_register(this,SX9500_SOFTRESET_REG,SX9500_SOFTRESET);
	/* wait until the reset has finished by monitoring NIRQ */
	/* just sleep for awhile instead of using a loop with reading irq status */
	msleep(300);
	hw_init(this);
	msleep(100); /* make sure everything is running */
	manual_offset_calibration(this);
	/* re-enable interrupt handling */
	mt_eint_unmask(SX9500_EINT_NUM);	
	/* make sure no interrupts are pending since enabling irq will only
	 * work on next falling edge */
	read_regStat(this);
	return 0;
}



static int check_sx9500_i2c(psx9500_t this)
{
	u8 value;
	int ret;

	ret=read_register(this,0x06,&value);
	if(ret){
		printk("<2>""read addr 0x06 err(%d)\n",ret);
		return ret;
	}else
		printk("<2>""read addr 0x06,val=%x\n",value);
			
	ret=write_register(this,0x06,value);
	if(ret<0){
		printk("<2>""write addr 0x06 err(%d)\n",ret);
		return ret;
	}else
		printk("<2>""write addr 0x06,val=%x,ret =%d\n",0xa,ret);
	
	ret=read_register(this,0x06,&value);
	if(ret)
		printk("<2>""read addr 0x06 err(%d)\n",ret);
	else
		printk("<2>""read addr 0x06,val=%x\n",value);
	
	return ret;
}


static ssize_t int_state_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	
  	u8 reg_value = 0;
	psx9500_t this = dev_get_drvdata(dev);

	check_sx9500_i2c(this);	
	return sprintf(buf, "%d\n", mt_get_gpio_in(SX9500_EINT_PIN));
}

static ssize_t int_state_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	return count;
}

static ssize_t sx9500_reg_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{	
  	u8 reg_value = 0;
	psx9500_t this = dev_get_drvdata(dev);

	check_sx9500_i2c(this);	
	return sprintf(buf, "%d\n", mt_get_gpio_in(SX9500_EINT_PIN));
}

#if 1
static int str_to_hex(char *c,int start,int count)
{
        int i,j=0,v=0,len;

        if(!c) return 0;
        len=strlen(c);

	if((count+start)>len)
		return 0;

        for(i=start;i<start+len;i++){
                if((c[i]>='0') && (c[i]<='9')){
                        v=(v<<4)+(c[i]-'0');
                        continue;
                }
                if((c[i]>='a') && (c[i]<='f')){
                        v=(v<<4)+(10+(c[i]-'a'));
                        continue;
                }

                if((c[i]>='A') && (c[i]<='F')){
                        v=(v<<4)+10+(c[i]-'A');
                        continue;
                }

                break;
        }
        return v;
}
#endif

static ssize_t sx9500_reg_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#if 1 
	psx9500_t this = dev_get_drvdata(dev);
	u8 addr,val;

	if(count<2){
		printk("<2>""format err!!\n");
		return count;
	}

	addr=str_to_hex(buf,0,2);

	if(count<5){
		read_register(this,addr,&val);
		printk("<2>""addr:0x%x = 0x%x\n",addr,val);
		return count;
	}

	val=str_to_hex(buf,3,2);
	write_register(this,addr,val);
	msleep(100);
	read_register(this,addr,&val);

	printk("<2>""after write,addr:0x%x = 0x%x\n",addr,val);
#endif		
	return count;
}

static DEVICE_ATTR(int_state, 0666, int_state_show,int_state_store);
static DEVICE_ATTR(sx9500_reg, 0666, sx9500_reg_show,sx9500_reg_store);
static struct attribute *sx9500_attributes[] = {
	&dev_attr_int_state.attr,
	&dev_attr_sx9500_reg.attr,
	NULL,
};
static struct attribute_group sx9500_attr_group = {
	.attrs = sx9500_attributes,
};


static void sx9500_int_handle(void)
{
	if(g_sx9500==NULL){
		printk("<2>""g_sx9500==NULL!!!!!\n");
		return;
	}
	schedule_delayed_work(&(g_sx9500->dworker),msecs_to_jiffies(1));
}

static void sx9500_worker_func(struct work_struct *work)
{
	int status;
	u8 v;

	psx9500_t this = 0;

	printk("<2>""======enter %s=======\n",__func__);
	if(!work){
		printk(KERN_ERR "sx86XX_worker_func, NULL work_struct\n");
		return;
	}

	this = container_of(work,sx9500_t,dworker.work);
	if (!this) {
      		printk(KERN_ERR "sx86XX_worker_func, NULL sx86XX_t\n");
		return;
	}
	
	status = this->refreshStatus(this);
	read_register(this, SX9500_TCHCMPSTAT_REG, &v);

	if(v&0x20){
		printk("<2>""---------------------psensor touched--------------------\n");
		send_psensor_uevent(PSENSOR_STATUS_NEAR);	
	}else{
		printk("<2>""------------------psensor realased----------------\n");
		send_psensor_uevent(PSENSOR_STATUS_FAR);	
	}

	printk("<2>""%s: status =%x, reg1=%x\n",__func__,status,v);
	
}

static int sx9500_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	psx9500_platform_data_t pdata = client->dev.platform_data;
	psx9500_t this = 0;
	//u8 value;
	//int ret;

	printk("<2>""============== enter %s ===============\n",__func__);

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;
	
  	this = kzalloc(sizeof(sx9500_t), GFP_KERNEL);
	if(!this){
		printk("<2>""alloc sx86XX_t fail\n");
		return -EIO;
	}

	this->refreshStatus = read_regStat;	
	this->get_nirq_low = pdata->get_is_nirq_low;
    	this->init = sx9500_initialize;
	this->bus = client;
	i2c_set_clientdata(client, this);

	/* initialize worker function */
	INIT_DELAYED_WORK(&this->dworker, sx9500_worker_func);

	printk("<2>""%s: start config interrupt\n",__func__);
	if(pdata->init_platform_hw)
		pdata->init_platform_hw();

	if(check_sx9500_i2c(this)){
		printk("<2>""i2c read/write err(%d),maybe sx9500 not exit\n");
		return -1;
	}

      	sysfs_create_group(&client->dev.kobj, &sx9500_attr_group);
	
	if (this->init)
    		this->init(this);

	g_sx9500=this;

	printk("<2>""----register psensor interrupt------%d\n",mt_get_gpio_in(SX9500_EINT_PIN));
	mt_eint_set_hw_debounce(SX9500_EINT_NUM, SX9500_EINT_DEBOUNCE_CN);
	mt_eint_registration(SX9500_EINT_NUM, EINTF_TRIGGER_FALLING, sx9500_int_handle, 1);
	mt_eint_unmask(SX9500_EINT_NUM);

	return 0;
}

int psensor_gpio_config(void)
{	
	printk("<2>""============== enter %s ===============\n",__func__);

#if TCL_PROTO_PCB
	mt_set_gpio_mode(SX9500_TXEN_PIN, SX9500_EINT_PIN_M_GPIO);
	mt_set_gpio_dir(SX9500_TXEN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(SX9500_TXEN_PIN, GPIO_OUT_ONE);

	mt_set_gpio_mode(SX9500_NRST_PIN, SX9500_EINT_PIN_M_GPIO);
	mt_set_gpio_dir(SX9500_NRST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(SX9500_NRST_PIN, GPIO_OUT_ONE);
#endif
	mt_set_gpio_mode(SX9500_EINT_PIN, SX9500_EINT_PIN_M_EINT);
	mt_set_gpio_dir(SX9500_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(SX9500_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(SX9500_EINT_PIN, GPIO_PULL_UP);
	msleep(50);
	return 0;
}


static int sx9500_nirq_is_low(void)
{
	return !mt_get_gpio_in(SX9500_EINT_PIN);
}

static int sx9500_remove(struct i2c_client *client)
{
	printk("<2>""============== enter %s ===============\n",__func__);
}

static sx9500_platform_data_t sx9500_config = {
	.init_platform_hw = psensor_gpio_config,
	.pi2c_reg = sx9500_i2c_reg_setup,
	.i2c_reg_num = ARRAY_SIZE(sx9500_i2c_reg_setup),
	.get_is_nirq_low=sx9500_nirq_is_low,
//  .pbuttonInformation = &smtcButtonInformation,
};

static struct i2c_board_info __initdata smtc_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(DRIVER_NAME, 0x2b),
		.flags         = I2C_CLIENT_WAKE,
		.irq           = SX9500_EINT_PIN,
		.platform_data = &sx9500_config,
	},
};

static struct i2c_device_id sx9500_idtable[] = {
	{ DRIVER_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sx9500_idtable);

static struct i2c_driver sx9500_i2c_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = DRIVER_NAME
	},
	.id_table = sx9500_idtable,
	.probe	  = sx9500_probe,
	.remove	  = __devexit_p(sx9500_remove),
//	.suspend  = sx9500_suspend,
//	.resume   = sx9500_resume,
};

static int sx9500_init(void)
{
	printk("<2>""%s:-------------------\n",__func__);
	i2c_register_board_info(2, smtc_i2c_boardinfo, 1);
	return i2c_add_driver(&sx9500_i2c_driver);
}

struct psensor_driver_t sx9500_driver={
	.name="sx9500",
	.init=sx9500_init,
};
