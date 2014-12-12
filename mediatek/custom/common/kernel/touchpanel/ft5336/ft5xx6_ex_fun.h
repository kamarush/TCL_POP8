//Begin,[xiaoguang.xiong][2013.06.20][PR 475675]
//add this file to add read rawdata function.

#ifndef __LINUX_FT5X06_EX_FUN_H__
#define __LINUX_FT5X06_EX_FUN_H__

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>

#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>

#define IC_FT5X06	0
#define IC_FT5606	1
#define IC_FT5316	2
#define IC_FT6208	3

#define FT_UPGRADE_AA	0xAA
#define FT_UPGRADE_55 	0x55
#define FT_UPGRADE_EARSE_DELAY		2000

/*upgrade config of FT5606*/
#define FT5606_UPGRADE_AA_DELAY 		50
#define FT5606_UPGRADE_55_DELAY 		10
#define FT5606_UPGRADE_ID_1			0x79
#define FT5606_UPGRADE_ID_2			0x06
#define FT5606_UPGRADE_READID_DELAY 	100

/*upgrade config of FT5316*/
#define FT5316_UPGRADE_AA_DELAY 		50
#define FT5316_UPGRADE_55_DELAY 		40
#define FT5316_UPGRADE_ID_1			0x79
#define FT5316_UPGRADE_ID_2			0x07
#define FT5316_UPGRADE_READID_DELAY 	1

/*upgrade config of FT5x06(x=2,3,4)*/
#define FT5X06_UPGRADE_AA_DELAY 		50
#define FT5X06_UPGRADE_55_DELAY 		30
#define FT5X06_UPGRADE_ID_1			0x79
#define FT5X06_UPGRADE_ID_2			0x03
#define FT5X06_UPGRADE_READID_DELAY 	1


#define DEVICE_IC_TYPE	IC_FT5X06

#define FTS_PACKET_LENGTH        128
#define FTS_SETTING_BUF_LEN        128

#define FTS_UPGRADE_LOOP	3

#define FTS_TX_MAX				40
#define FTS_RX_MAX				40
#define FTS_DEVICE_MODE_REG	0x00
#define FTS_TXNUM_REG			0x03
#define FTS_RXNUM_REG			0x04
#define FTS_RAW_READ_REG		0x01
#define FTS_RAW_BEGIN_REG		0x10
#define FTS_VOLTAGE_REG		0x05

#define FTS_FACTORYMODE_VALUE		0x40
#define FTS_WORKMODE_VALUE		0x00

#define RAW_DATA_MAX_VALUE              10000
#define RAW_DATA_MIN_VALUE              6800
#define BACK_KEY_PLACE                  2
#define HOME_KEY_PLACE                  6
#define MENU_KEY_PLACE                  10

//#define    AUTO_CLB


/*create sysfs for debug*/
int TP_sysfs_init(void);
void TP_sysfs_exit(void);

/*
*ft5x0x_write_reg- write register
*@client: handle of i2c
*@regaddr: register address
*@regvalue: register value

*
*/

int write_reg(u8 addr, u8 para);
int read_reg(u8 addr, unsigned char *pdata);
u8 cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num);
//int  fts_ctpm_fw_upgrade(unsigned char* pbt_buf, long unsigned int dw_lenth);

#endif
