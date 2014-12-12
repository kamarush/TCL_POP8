#if 0
#ifndef __MT6628_FM_LINK_H__
#define __MT6628_FM_LINK_H__

#include <linux/wait.h>
#include "fm_link.h"
#include "fm_utils.h"

#define RX_BUF_SIZE 128
#define TX_BUF_SIZE 1024

#define SW_RETRY_CNT            (1)
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
#define EN_TIMEOUT              (3)
#define RST_TIMEOUT             (3)
#define TUNE_TIMEOUT            (3)
#define SM_TUNE_TIMEOUT         (6)
#define SEEK_TIMEOUT            (15)
#define SCAN_TIMEOUT            (15) //usualy scan will cost 10 seconds 
#define RDS_RX_EN_TIMEOUT       (3)
#define RDS_DATA_TIMEOUT        (100)
#define RAMPDOWN_TIMEOUT        (3)
#define MCUCLK_TIMEOUT          (3)
#define MODEMCLK_TIMEOUT        (3)
#define RDS_TX_TIMEOUT          (3)
#define PATCH_TIMEOUT           (3)
#define COEFF_TIMEOUT           (3)
#define HWCOEFF_TIMEOUT         (3)
#define ROM_TIMEOUT             (3)

struct fm_link_event {
    struct fm_flag_event *ln_event;
    struct fm_res_ctx result; // seek/scan/read/RDS
};
#endif
#endif
