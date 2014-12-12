
#include "emi.h"

#define NUM_EMI_RECORD (1)

int num_of_emi_records = NUM_EMI_RECORD ;

EMI_SETTINGS emi_settings[] =
{
     
	//MT29PZZZ8D4WKFEW_18W_6D4
	{
		0x0,		/* sub_version */
		0x0202,		/* TYPE */
		9,		/* EMMC ID/FW ID checking length */
		1,		/* FW length */
		{0xFE,0x01,0x4E,0x50,0x30,0x58,0x58,0x58,0x58,0x0,0x0,0x0,0x0,0x0,0x0,0x0},		/* NAND_EMMC_ID */
		{0x10,0x0,0x0,0x0,0x0,0x0,0x0,0x0},		/* FW_ID */
		0x0002A3AE,		/* EMI_CONA_VAL */
		0xAA00AA00,		/* DRAMC_DRVCTL0_VAL */
		0xAA00AA00,		/* DRAMC_DRVCTL1_VAL */
		0x44584493,		/* DRAMC_ACTIM_VAL */
		0x01000000,		/* DRAMC_GDDR3CTL1_VAL */
		0xF0048683,		/* DRAMC_CONF1_VAL */
		0xA00632D1,		/* DRAMC_DDR2CTL_VAL */
		0xBF080401,		/* DRAMC_TEST2_3_VAL */
		0x0340633F,		/* DRAMC_CONF2_VAL */
		0x51642342,		/* DRAMC_PD_CTRL_VAL */
		0x00000000,		/* DRAMC_PADCTL3_VAL */
		0xEEEEEEEE,		/* DRAMC_DQODLY_VAL */
		0x00000000,		/* DRAMC_ADDR_OUTPUT_DLY */
		0x00000000,		/* DRAMC_CLK_OUTPUT_DLY */
		0x01000510,		/* DRAMC_ACTIM1_VAL*/
		0x07800000,		/* DRAMC_MISCTL0_VAL*/
		0x04002600,		/* DRAMC_ACTIM05T_VAL*/
		{0x20000000,0x20000000,0,0},		/* DRAM RANK SIZE */
		{0,0,0,0,0,0,0,0,0,0},		/* reserved 10 */
		0x00C30001,		/* LPDDR2_MODE_REG1 */
		0x00060002,		/* LPDDR2_MODE_REG2 */
		0x00020003,		/* LPDDR2_MODE_REG3 */
		0x000000FF,		/* LPDDR2_MODE_REG5 */
		0x00FF000A,		/* LPDDR2_MODE_REG10 */
		0x0000003F,		/* LPDDR2_MODE_REG63 */
	} ,
};