
#ifndef __MT6626_FM_LINK_H__
#define __MT6626_FM_LINK_H__

#include <linux/wait.h>
#include "fm_link.h"

#define RX_BUF_SIZE 128
#define TX_BUF_SIZE 1024

#define SW_RETRY_CNT            (2)
#define SW_RETRY_CNT_MAX        (5)
#define SW_WAIT_TIMEOUT_MAX     (100)

// FM operation timeout define for error handle
#define TEST_TIMEOUT            (3)
#define FSPI_EN_TIMEOUT         (3)
#define FSPI_MUXSEL_TIMEOUT     (3)
#define FSPI_RD_TIMEOUT         (3)
#define FSPI_WR_TIMEOUT         (3)
#define I2C_RD_TIMEOUT          (3)
#define I2C_WR_TIMEOUT          (3)
#define EN_TIMEOUT              (5)
#define RST_TIMEOUT             (3)
#define TUNE_TIMEOUT            (3)
#define SEEK_TIMEOUT            (10)
#define SCAN_TIMEOUT            (15) //usualy scan will cost 10 seconds 
#define RDS_RX_EN_TIMEOUT       (3)
#define RDS_DATA_TIMEOUT        (100)
#define RAMPDOWN_TIMEOUT        (3)
#define MCUCLK_TIMEOUT          (3)
#define MODEMCLK_TIMEOUT        (3)
#define RDS_TX_TIMEOUT          (3)
#define PATCH_TIMEOUT           (5)
#define COEFF_TIMEOUT           (5)
#define HWCOEFF_TIMEOUT         (5)
#define ROM_TIMEOUT             (5)

struct fm_link_event {
    //data
    volatile fm_u32 flag;
    wait_queue_head_t wq;
    struct fm_res_ctx result; // seek/scan/read/RDS
    //methods
    fm_u32(*set)(struct fm_link_event* thiz, fm_u32 mask);
    fm_u32(*clr)(struct fm_link_event* thiz, fm_u32 mask);
    long(*check)(struct fm_link_event* thiz, fm_u32 mask, long timeout);
    fm_u32(*get)(struct fm_link_event* thiz);
    fm_u32(*rst)(struct fm_link_event* thiz);
};

#endif
