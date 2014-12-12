#include <linux/init.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <asm/localtimer.h>
#include <asm/fiq_glue.h>
#include <mach/mt_reg_base.h>
#include <mach/smp.h>
#include <mach/sync_write.h>
#include <mach/hotplug.h>
#ifdef CONFIG_HOTPLUG_WITH_POWER_CTRL
#include <mach/mt_spm_mtcmos.h>
#endif
#include <mach/mt_spm_idle.h>
#include <mach/wd_api.h>


#define SLAVE1_MAGIC_REG (SRAMROM_BASE+0x38)
#define SLAVE2_MAGIC_REG (SRAMROM_BASE+0x3C)
#define SLAVE3_MAGIC_REG (SRAMROM_BASE+0x40)
#define SLAVE1_MAGIC_NUM 0x534C4131
#define SLAVE2_MAGIC_NUM 0x4C415332
#define SLAVE3_MAGIC_NUM 0x41534C33
#define SLAVE_JUMP_REG  (SRAMROM_BASE+0x34)
extern void mt_secondary_startup(void);
extern void irq_raise_softirq(const struct cpumask *mask, unsigned int irq);
extern void mt_gic_secondary_init(void);


extern unsigned int irq_total_secondary_cpus;
static unsigned int is_secondary_cpu_first_boot;
static DEFINE_SPINLOCK(boot_lock);
/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen".
 */
volatile int pen_release = -1;


/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void __cpuinit write_pen_release(int val)
{
    pen_release = val;
    smp_wmb();
    __cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
    outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}


int L2CTLR_get_core_count(void)
{
    unsigned int cores = 0;
    extern u32 get_devinfo_with_index(u32 index);
    u32 idx = 3;
    u32 value = 0;
    value = get_devinfo_with_index(idx);

    value = (value >> 24) & 0xF;
    if (value == 0x0)
        cores = 4;
    else
        cores = 2;

    return cores;
}

void __cpuinit platform_secondary_init(unsigned int cpu)
{
    struct wd_api *wd_api = NULL;

    printk(KERN_INFO "Slave cpu init\n");
    HOTPLUG_INFO("platform_secondary_init, cpu: %d\n", cpu);

    mt_gic_secondary_init();

    /*
     * let the primary processor know we're out of the
     * pen, then head off into the C entry point
     */
    write_pen_release(-1);

    get_wd_api(&wd_api);
    if (wd_api)
        wd_api->wd_cpu_hot_plug_on_notify(cpu);

    fiq_glue_resume();

#ifdef SPM_MCDI_FUNC
    spm_hot_plug_in_before(cpu);
#endif

    /*
     * Synchronise with the boot thread.
     */
    spin_lock(&boot_lock);
    spin_unlock(&boot_lock);
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
    unsigned long timeout;

    printk(KERN_CRIT "Boot slave CPU\n");

    atomic_inc(&hotplug_cpu_count);

    /*
     * Set synchronisation state between this boot processor
     * and the secondary one
     */
    spin_lock(&boot_lock);

    HOTPLUG_INFO("boot_secondary, cpu: %d\n", cpu);
    /*
     * The secondary processor is waiting to be released from
     * the holding pen - release it, then wait for it to flag
     * that it has been released by resetting pen_release.
     *
     * Note that "pen_release" is the hardware CPU ID, whereas
     * "cpu" is Linux's internal ID.
     */
    /*
     * This is really belt and braces; we hold unintended secondary
     * CPUs in the holding pen until we're ready for them.  However,
     * since we haven't sent them a soft interrupt, they shouldn't
     * be there.
     */
    write_pen_release(cpu);

    switch(cpu)
    {
        case 1:
            if (is_secondary_cpu_first_boot)
            {
                --is_secondary_cpu_first_boot;
                mt65xx_reg_sync_writel(SLAVE1_MAGIC_NUM, SLAVE1_MAGIC_REG);
                HOTPLUG_INFO("SLAVE1_MAGIC_NUM:%x\n", SLAVE1_MAGIC_NUM);
            
            }
        #ifdef CONFIG_HOTPLUG_WITH_POWER_CTRL
            else
            {
                mt65xx_reg_sync_writel(virt_to_phys(mt_secondary_startup), BOOT_ADDR);
                spm_mtcmos_ctrl_cpu1(STA_POWER_ON);
            }
        #endif
            break;
        case 2:
            if (is_secondary_cpu_first_boot)
            {
                --is_secondary_cpu_first_boot;
                mt65xx_reg_sync_writel(SLAVE2_MAGIC_NUM, SLAVE2_MAGIC_REG);
                HOTPLUG_INFO("SLAVE2_MAGIC_NUM:%x\n", SLAVE2_MAGIC_NUM);
            }
        #ifdef CONFIG_HOTPLUG_WITH_POWER_CTRL
            else
            {
                mt65xx_reg_sync_writel(virt_to_phys(mt_secondary_startup), BOOT_ADDR);
                spm_mtcmos_ctrl_cpu2(STA_POWER_ON);
            }
        #endif
            break;
        case 3:
            if (is_secondary_cpu_first_boot)
            {
                --is_secondary_cpu_first_boot;
                mt65xx_reg_sync_writel(SLAVE3_MAGIC_NUM, SLAVE3_MAGIC_REG);
                HOTPLUG_INFO("SLAVE3_MAGIC_NUM:%x\n", SLAVE3_MAGIC_NUM);
            }
        #ifdef CONFIG_HOTPLUG_WITH_POWER_CTRL
            else
            {
                mt65xx_reg_sync_writel(virt_to_phys(mt_secondary_startup), BOOT_ADDR);
                spm_mtcmos_ctrl_cpu3(STA_POWER_ON);
            }
        #endif
            break;
        default:
            break;

    }

    smp_cross_call(cpumask_of(cpu));

    timeout = jiffies + (1 * HZ);
    while (time_before(jiffies, timeout)) {
        smp_rmb();
        if (pen_release == -1)
            break;

        udelay(10);
    }
    
    /*
     * Now the secondary core is starting up let it run its
     * calibrations, then wait for it to finish
     */
    spin_unlock(&boot_lock);

    if (pen_release == -1)
    {
        return 0;
    }
    else
    {
        mt65xx_reg_sync_writel(cpu + 8, 0xf0200080);
        printk(KERN_EMERG "CPU%u, debug event: 0x%08x, debug monitor: 0x%08x\n", cpu, *(volatile u32 *)(0xf0200080), *(volatile u32 *)(0xf0200084));
        on_each_cpu((smp_call_func_t)dump_stack, NULL, 0);
        return -ENOSYS;
    }
}

void __init smp_init_cpus(void)
{
    unsigned int i, ncores;

    ncores = L2CTLR_get_core_count();
    if (ncores > NR_CPUS) {
        printk(KERN_WARNING
               "L2CTLR core count (%d) > NR_CPUS (%d)\n", ncores, NR_CPUS);
        printk(KERN_WARNING
               "set nr_cores to NR_CPUS (%d)\n", NR_CPUS);
        ncores = NR_CPUS;
    }
    
    if (ncores == 2)
    {
        // We only have 2 cores, turn off power of the other cores.
        // Otherwise we can't enter suspend.
        spm_mtcmos_ctrl_cpu2(STA_POWER_DOWN);
        spm_mtcmos_ctrl_cpu3(STA_POWER_DOWN);
    }

    for (i = 0; i < ncores; i++)
        set_cpu_possible(i, true);

    irq_total_secondary_cpus = num_possible_cpus() - 1;
    is_secondary_cpu_first_boot = num_possible_cpus() - 1;

    set_smp_cross_call(irq_raise_softirq);
    
}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
    int i;

    for (i = 0; i < max_cpus; i++)
        set_cpu_present(i, true);


    /* write the address of slave startup into the system-wide flags register */
    mt65xx_reg_sync_writel(virt_to_phys(mt_secondary_startup), SLAVE_JUMP_REG);
  
}
