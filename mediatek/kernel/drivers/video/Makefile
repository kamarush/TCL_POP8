# drivers/video/mtk/Makefile
#
#

obj-$(CONFIG_MTK_FB) += disp_drv.o                                 \
         mtkfb_console.o                            \
         disp_assert_layer.o                        \
         mtkfb.o                                    \
		 mtkfb_vsync.o								\
         debug.o                                    \
         font_8x16.o                                \
         mtkfb_proc.o

ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/video/  \
             -I$(MTK_PATH_SOURCE)/drivers/video/    \
			 -I$(MTK_PATH_PLATFORM)/drivers/dispsys/ \
			 -I$(MTK_PATH_PLATFORM)/core/include/mach/ \
			 -I$(MTK_PATH_SOURCE)/../hardware/dpframework/inc
ifeq ($(MTK_INTERNAL_HDMI_SUPPORT), yes)
ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/hdmi/
endif  
ifeq ($(MTK_INTERNAL_MHL_SUPPORT), yes)
ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/mhltx/
endif   

ifeq ($(MTK_DITHERING_SUPPORT), yes)
ccflags-y += -DDITHERING_SUPPORT
endif

ifeq ($(MTK_GPU_SUPPORT), yes)
ccflags-y += -DHWGPU_SUPPORT
endif

ifeq ($(MTK_DAL_SUPPORT), yes)
ccflags-y += -DMTKFB_DISP_ASSERT_LAYER_ENABLE
endif


