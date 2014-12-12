/*
 *drivers/input/touchscreen/ft5x06_ex_fun.c
 *
 *FocalTech ft5x0x expand function for debug.
 *
 *Copyright (c) 2010  Focal tech Ltd.
 *
 *This software is licensed under the terms of the GNU General Public
 *License version 2, as published by the Free Software Foundation, and
 *may be copied, distributed, and modified under those terms.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *Note:the error code of EIO is the general error in this file.
 */

//Begin,[xiaoguang.xiong][2013.06.20][PR 475675]
//add this file to add read raw data function.



#include "ft5x06_ex_fun.h"
#include "tpd_custom_ft5406.h"
#include <linux/mount.h>
#include <linux/netdevice.h>
#include "tpd.h"

extern struct i2c_client *i2c_client;
extern u8 *I2CDMABuf_va;
extern volatile u32 I2CDMABuf_pa;
struct Upgrade_Info {
	u16 delay_aa;		/*delay of write FT_UPGRADE_AA */
	u16 delay_55;		/*delay of write FT_UPGRADE_55 */
	u8 upgrade_id_1;	/*upgrade id 1 */
	u8 upgrade_id_2;	/*upgrade id 2 */
	u16 delay_readid;	/*delay of read id */
	u16 delay_earse_flash; /*delay of earse flash*/
};

int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret,i;
	
	//client->timing = 100;	
	if(writelen!=0)
	{
		//DMA Write		
		{
			for(i = 0 ; i < writelen; i++)
			{
				I2CDMABuf_va[i] = writebuf[i];
			}
			client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;	
			
			if((ret=i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
				dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
			//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
		
		}
	}
	//DMA Read 
	if(readlen!=0)
	{		
		{
			client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
			
			ret = i2c_master_recv(client, (unsigned char *)I2CDMABuf_pa, readlen);

			for(i = 0; i < readlen; i++) {
	            		readbuf[i] = I2CDMABuf_va[i];
	        	}
		}
	}

	return ret;
}


/*write data by i2c*/
//#if 0
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	int i = 0;
	//printk("ft6x06_i2c_Write  %d  %x  %x %x %x",writelen,writebuf,I2CDMABuf_va,I2CDMABuf_pa,client);
	
  //client->addr = client->addr & I2C_MASK_FLAG;//test
   //client->ext_flag |= I2C_DIRECTION_FLAG; //test
   //client->timing = 100;	//test
	{
		for(i = 0 ; i < writelen; i++)
		{
			I2CDMABuf_va[i] = writebuf[i];
		}
		client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;	
		if((ret=i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
		{
			printk("focal ft6x06_i2c_Write error\n");
			dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
		}
		//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
		return ret;
	} 


}


int ft5x0x_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return ft5x0x_i2c_Write(client, buf, sizeof(buf));
}


int ft5x0x_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return ft5x0x_i2c_Read(client, &regaddr, 1, regvalue, 1);
}


static struct mutex g_device_mutex;

static int ft5x0x_read_rawdata(struct i2c_client *client, u16 rawdata[][FTS_RX_MAX],
			u8 tx, u8 rx)
{
	u8 i = 0, j = 0, k = 0;
	int err = 0;
	u8 regvalue = 0x00;
	u8 regaddr = 0x00;
	u16 dataval = 0x0000;
	u8 writebuf[2] = {0};
	u8 read_buffer[FTS_RX_MAX * 2];
	/*scan*/
        err = read_reg(FTS_DEVICE_MODE_REG, &regvalue);
	if (err < 0) {
		return err;
	} else {
		regvalue |= 0x80;
                err = write_reg(FTS_DEVICE_MODE_REG, regvalue);
		if (err < 0) {
			return err;
		} else {
			for(i=0; i<20; i++)
			{
				msleep(8);
                                err = read_reg(FTS_DEVICE_MODE_REG,&regvalue);
				if (err < 0) {
					return err;
				} else {
					if (0 == (regvalue >> 7))
						break;
				}
			}
		}
	}

	/*get rawdata*/
	//dev_dbg(&client->dev, "%s() - Reading raw data...\n", __func__);
	for(i=0; i<tx; i++)
	{
		memset(read_buffer, 0x00, (FTS_RX_MAX * 2));
		writebuf[0] = FTS_RAW_READ_REG;
		writebuf[1] = i;

                write_reg(writebuf[0], writebuf[1]);
		/* Read the data for this row */
		regaddr = FTS_RAW_BEGIN_REG;

                k = 0;
		for (j = 0; j < rx*2; j += 2)
        	{
                        read_reg(FTS_RAW_BEGIN_REG+j,read_buffer+j);
                        read_reg(FTS_RAW_BEGIN_REG+j+1,read_buffer+(j+1));
			dataval  = read_buffer[j];
			dataval  = (dataval << 8);
			dataval |= read_buffer[j+1];
			rawdata[i][k] = dataval;
			k++;
        	}
	}


	return 0;
}
/*raw data show*/
static ssize_t ft5x0x_rawdata_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
        char *s = buf;
        u16 before_rawdata[FTS_TX_MAX][FTS_RX_MAX]={0};
	u8 rx = 0, tx = 0;
	u8 i = 0, j = 0;
	int err = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	mutex_lock(&g_device_mutex);
	/*entry factory*/
        err = write_reg(FTS_DEVICE_MODE_REG, FTS_FACTORYMODE_VALUE);  //goto factory mode
	if (err < 0) {
                s +=sprintf(s, "%s:rawdata show error!\n",__func__);
		goto RAW_ERROR;
	}

	/*get rx and tx num*/
        err = read_reg(FTS_TXNUM_REG, &tx);
	if (err < 0) {
                s +=sprintf(s, "%s:get tx error!\n",__func__);
		goto RAW_ERROR;
	}

        err = read_reg(FTS_RXNUM_REG, &rx);
	if (err < 0) {
                s +=sprintf(s, "%s:get rx error!\n",__func__);
		goto RAW_ERROR;
	}

        s +=sprintf(s, "tp channel: tx is %u ,rx is %u\n", tx, rx);

	/*get rawdata*/
	err = ft5x0x_read_rawdata(client, before_rawdata, tx, rx);
	if (err < 0) {
                s +=sprintf(s, "%s:rawdata show error!\n", __func__);
		goto RAW_ERROR;
	}
   
        else {
		for (i=0; i<tx; i++)
                {
                    s +=sprintf(s, "the begin of the %uth tx: ",i);
		    for (j=0; j<rx; j++) 
                        {
                            s +=sprintf(s, "%u ", before_rawdata[i][j]);
			}
                    s +=sprintf(s, "%s.\n","the end");
     
		}
                for (i=0; i<tx; i++)
                {
                    if(i<(tx-1))
                    {
                        for(j=0; j<rx; j++)
                        {
                            if(before_rawdata[i][j]>RAW_DATA_MAX_VALUE||before_rawdata[i][j]<RAW_DATA_MIN_VALUE)
                            {
                                 s +=sprintf(s, "there is bad spot in the %uth tx.\n", i);
                                 break;
                            }
                        }
                    }
                    else
                    {
                        if((before_rawdata[i][BACK_KEY_PLACE]>RAW_DATA_MAX_VALUE||before_rawdata[i][BACK_KEY_PLACE]<RAW_DATA_MIN_VALUE)||(before_rawdata[i][HOME_KEY_PLACE]>RAW_DATA_MAX_VALUE||before_rawdata[i][HOME_KEY_PLACE]<RAW_DATA_MIN_VALUE)||(before_rawdata[i][MENU_KEY_PLACE]>RAW_DATA_MAX_VALUE||before_rawdata[i][MENU_KEY_PLACE]<RAW_DATA_MIN_VALUE))
                            s +=sprintf(s, "there is bad spot in the %uth tx.\n", i);
                    }
                }
	}


RAW_ERROR:
	/*enter work mode*/
        err = write_reg(FTS_DEVICE_MODE_REG, FTS_WORKMODE_VALUE); //return to normal mode

	msleep(100);
	mutex_unlock(&g_device_mutex);
        return (s-buf);
}

static ssize_t ft5x0x_rawdata_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

//Begin,[xiaoguang.xiong][2013.07.09][PR 485950]
//add FW online_upgrade function.

/*
*get upgrade information depend on the ic type
*/
static void fts_get_upgrade_info(struct Upgrade_Info *upgrade_info)
{
	switch (DEVICE_IC_TYPE) {
	case IC_FT5X06:
		upgrade_info->delay_55 = FT5X06_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5X06_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5X06_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5X06_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5X06_UPGRADE_READID_DELAY;
		//upgrade_info->delay_earse_flash = FT5X06_UPGRADE_EARSE_DELAY;
                upgrade_info->delay_earse_flash = FT_UPGRADE_EARSE_DELAY;
		break;
	case IC_FT5606:
		upgrade_info->delay_55 = FT5606_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5606_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5606_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5606_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5606_UPGRADE_READID_DELAY;
		//upgrade_info->delay_earse_flash = FT5606_UPGRADE_EARSE_DELAY;
                upgrade_info->delay_earse_flash = FT_UPGRADE_EARSE_DELAY;
		break;
	case IC_FT5316:
		upgrade_info->delay_55 = FT5316_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5316_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5316_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5316_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5316_UPGRADE_READID_DELAY;
		//upgrade_info->delay_earse_flash = FT5316_UPGRADE_EARSE_DELAY;
                upgrade_info->delay_earse_flash = FT_UPGRADE_EARSE_DELAY;
		break;
#if 0
	case IC_FT6208:
		upgrade_info->delay_55 = FT6208_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT6208_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT6208_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT6208_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT6208_UPGRADE_READID_DELAY;
		upgrade_info->delay_earse_flash = FT6208_UPGRADE_EARSE_DELAY;
		break;
#endif
	default:
		break;
	}
}
void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++)
	{
		for (j = 0; j < 1000; j++)
		{
			 udelay(1);
		}
	}
}


int fts_ctpm_sys_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	struct Upgrade_Info upgradeinfo;

	fts_get_upgrade_info(&upgradeinfo);

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, 0);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
		mdelay(10);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		mdelay(50);

		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		do {
			i++;
			i_ret = ft5x0x_i2c_Write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && i < 5);

		/*********Step 3:check READ-ID***********************/
		msleep(upgradeinfo.delay_readid);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		ft5x0x_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == upgradeinfo.upgrade_id_1 
			&& reg_val[1] == upgradeinfo.upgrade_id_2) {
				TPD_DMESG("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
				break;
		} else {
				dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	auc_i2c_write_buf[0] = 0xcd;

	ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);

	/*Step 4:erase app and panel paramenter area*/
	//DBG("Step 4:erase app and panel paramenter area\n");
        TPD_DMESG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(upgradeinfo.delay_earse_flash);
	/*erase panel parameter area */
	auc_i2c_write_buf[0] = 0x63;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	//DBG("Step 5:write firmware(FW) to ctpm flash\n");
        TPD_DMESG("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		ft5x0x_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		msleep(FTS_PACKET_LENGTH / 6 + 1);
		//DBG("write bytes:0x%04x\n", (j+1) * FTS_PACKET_LENGTH);
		//delay_qt_ms(FTS_PACKET_LENGTH / 6 + 1);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		ft5x0x_i2c_Write(client, packet_buf, temp + 6);
		msleep(20);
	}


	/*send the last six byte */
	for (i = 0; i < 6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = 1;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		packet_buf[6] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6];
		ft5x0x_i2c_Write(client, packet_buf, 7);
		msleep(20);
	}


	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	//DBG("Step 6: read out checksum\n");
        TPD_DMESG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	//DBG("Step 7: reset the new FW\n");
        TPD_DMESG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */

	return 0;


}

/*
*get firmware size

@firmware_name:firmware name
*note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int ft5x0x_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s", firmware_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}



/*
*read firmware buf for .bin file.

@firmware_name: fireware name
@firmware_buf: data buf of fireware

note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int ft5x0x_ReadFirmware(char *firmware_name,
			       unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}





/*
upgrade with *.bin file
*/

int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client,
				       char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret;
	int fwsize = ft5x0x_GetFirmwareSize(firmware_name);
	printk("fw upgrade online, %s, %d \n",__FUNCTION__,__LINE__);

	printk("fw upgrade online, i2c_client = %x addr = %x\n", client,client->addr);
	if (fwsize <= 0) {
		dev_err(&client->dev, "%s ERROR:Get firmware size failed\n",
					__func__);
		return -EIO;
	}

	if (fwsize < 8 || fwsize > 32 * 1024) {
		//dev_dbg(&client->dev, "%s:FW length error\n", __func__);
		return -EIO;
	}

	/*=========FW upgrade========================*/
	pbt_buf = kmalloc(fwsize + 1, GFP_ATOMIC);

	if (ft5x0x_ReadFirmware(firmware_name, pbt_buf)) {
		dev_err(&client->dev, "%s() - ERROR: request_firmware failed\n",
					__func__);
		kfree(pbt_buf);
		return -EIO;
	}
	printk("fw upgrade online, %s, %d \n",__FUNCTION__,__LINE__);
	if ((pbt_buf[fwsize - 8] ^ pbt_buf[fwsize - 6]) == 0xFF
		&& (pbt_buf[fwsize - 7] ^ pbt_buf[fwsize - 5]) == 0xFF
		&& (pbt_buf[fwsize - 3] ^ pbt_buf[fwsize - 4]) == 0xFF) {
		/*call the upgrade function */	

		i_ret = fts_ctpm_sys_fw_upgrade(client, pbt_buf, fwsize);
		if (i_ret != 0)
			//dev_dbg(&client->dev, "%s() - ERROR:[FTS] upgrade failed..\n",
						//__func__);
                        udelay(1);
		else {
#ifdef AUTO_CLB
			fts_ctpm_auto_clb(client);	/*start auto CLB*/
#endif
		 }
		kfree(pbt_buf);
	} else {
		//dev_dbg(&client->dev, "%s:FW format error\n", __func__);
		kfree(pbt_buf);
		return -EIO;
	}

	return i_ret;
}


static ssize_t ft5x0x_fwupgradeapp_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}


/*upgrade from app.bin*/
static ssize_t ft5x0x_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = i2c_client;
		//container_of(dev, struct i2c_client, dev);

	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count - 1] = '\0';

	mutex_lock(&g_device_mutex);
	disable_irq(client->irq);

	fts_ctpm_fw_upgrade_with_app_file(client, fwname);

	enable_irq(client->irq);
	mutex_unlock(&g_device_mutex);

	return count;
}


/*sysfs */

/*show a frame rawdata
*example:cat ftsrawdatashow
*/
static DEVICE_ATTR(ftsrawdatashow, S_IRUGO | S_IWUSR, ft5x0x_rawdata_show,
			ft5x0x_rawdata_store);

/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO | S_IWUSR, ft5x0x_fwupgradeapp_show,
			ft5x0x_fwupgradeapp_store);

/*add your attr in here*/
static struct attribute *ft5x0x_attributes[] = {
	&dev_attr_ftsrawdatashow.attr,
        &dev_attr_ftsfwupgradeapp.attr,
	NULL
};

static struct attribute_group ft5x0x_attribute_group = {
	.attrs = ft5x0x_attributes
};



//add sysfs
struct kobject *TP_ctrl_kobj;

int TP_sysfs_init(void)
{ 
	TP_ctrl_kobj = kobject_create_and_add("tp-information", NULL);
	if (!TP_ctrl_kobj)
		return -ENOMEM;

        mutex_init(&g_device_mutex);

	return sysfs_create_group(TP_ctrl_kobj, &ft5x0x_attribute_group);
}
//remove sysfs
void TP_sysfs_exit(void)
{
	sysfs_remove_group(TP_ctrl_kobj, &ft5x0x_attribute_group);

        mutex_destroy(&g_device_mutex);

	kobject_put(TP_ctrl_kobj);
}


