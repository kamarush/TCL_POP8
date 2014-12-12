



#ifndef _CFG_SIM_FILE_H
#define _CFG_SIM_FILE_H

typedef struct
{	
	unsigned char sim2_ctl_flag;//0: AP side control, 1: MD side control;                                           
}ap_nvram_sim_config_struct;

#define CFG_FILE_SIM_CONFIG_SIZE    sizeof(ap_nvram_sim_config_struct)
#define CFG_FILE_SIM_CONFIG_TOTAL   1

#endif// _CFG_SIM_FILE_H


