

enum PSENSOR_STATUS{
	PSENSOR_STATUS_NEAR=0,
	PSENSOR_STATUS_FAR=1,
};

struct psensor_driver_t{
	char *name;
	int (*init)(void);
};


void send_psensor_uevent(enum PSENSOR_STATUS val);
