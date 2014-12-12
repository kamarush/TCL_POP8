
#ifndef __MT_SEC_HAL_H__
#define __MT_SEC_HAL_H__

typedef enum{
    HACC_USER1 = 0,
    HACC_USER2,
    HACC_USER3
} HACC_USER;

int masp_hal_get_uuid(unsigned int *uuid);
int masp_hal_sbc_enabled(void);
int masp_hal_get_sbc_checksum(unsigned int *pChecksum);
unsigned char masp_hal_secure_algo_init(void);
unsigned char masp_hal_secure_algo_deinit(void);
void masp_hal_secure_algo(unsigned char Direction, unsigned int ContentAddr, unsigned int ContentLen, unsigned char *CustomSeed, unsigned char *ResText);
unsigned int masp_hal_sp_hacc_init (unsigned char *sec_seed, unsigned int size);
unsigned int masp_hal_sp_hacc_blk_sz (void);
unsigned char* masp_hal_sp_hacc_enc(unsigned char *buf, unsigned int size, unsigned char bAC, HACC_USER user, unsigned char bDoLock);
unsigned char* masp_hal_sp_hacc_dec(unsigned char *buf, unsigned int size, unsigned char bAC, HACC_USER user, unsigned char bDoLock);

#endif/* !__MT_SEC_HAL_H__ */
