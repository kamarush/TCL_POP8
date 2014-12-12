ifeq ($(MTK_INTERNAL), yes)
ifeq ($(MTK_USE_RESERVED_EXT_MEM), yes)
KBUILD_CFLAGS += -DMTK_USE_RESERVED_EXT_MEM
endif
endif

obj-$(CONFIG_KGDB_KDB) += kdb_enhance.o
obj-y += mt_storage_logger.o
obj-y += mt_io_logger.o
obj-$(CONFIG_MT65XX_TRACER) += trace_mt65xx_mon.o
obj-$(CONFIG_MT_PRIO_TRACER) += sched/
obj-$(CONFIG_MTK_MEMCFG) += mtk_memcfg.o
obj-y += mtk_meminfo.o
obj-y += system.o

obj-y += mt_cache_v7.o
obj-$(CONFIG_SMP) += fiq_smp_call.o
# Hibernation
obj-$(CONFIG_MTK_HIBERNATION) += hibcore/
