include $(MTK_PATH_BUILD)/common.mk
#Fix-me: modified for early porting
#$(call all-subdir-src-or-makefile)
obj-y   += uart/
obj-y   += gpio/
obj-y   += pwm/
ifneq	($(MTK_GPU_SUPPORT),no)
obj-m   += gpu/
endif
obj-y   += dual_ccci/
obj-y   += btcvsd/
obj-y   += pmic_wrap/
obj-y   += power/
obj-y   += power_gs/
obj-y   += m4u/
obj-y   += smi/
obj-y   += usb20/
obj-y   += mmc-host/
obj-y   += jpeg/
obj-y   += i2c/
obj-y   += spi/
obj-y   += keypad/
obj-y   += video/
obj-y   += dispsys/
obj-y   += rtc/
obj-y   += wdt/
obj-y   += sound/
obj-m   += devinfo/
obj-m   += devapc/
#ifneq ($(MTK_PROJECT), mt6582fpga_ca7_ldvt)

#endif
#ifneq ($(CONFIG_EARLY_LINUX_PORTING),y)
#obj-y   += nand/
#obj-m   += mdrvb/
obj-y   += auxadc/
#obj-y   += touchpanel/
#obj-y   += spi/
#obj-y   += ccci/
obj-y   += ldvt/
#obj-y   += mmc-host/
obj-y   += videocodec/
#obj-y   += rtc/
#
#obj-y   += jpeg/
#obj-y   += gdma/
obj-y   += masp/
#endif
#
obj-y   += accdet/
obj-y   += vibrator/
obj-y   += leds/
obj-y   += btif/
obj-y   += thermal/
obj-y   += cpu_stress/
obj-y   += pmt/
ifeq ($(HAVE_CMMB_FEATURE),yes)
obj-m += cmmbdrv/
endif

obj-y	+= $(call all-subdirs-with-makefile,met)
