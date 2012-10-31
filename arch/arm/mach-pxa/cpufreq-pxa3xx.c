/*
 * linux/arch/arm/mach-pxa/cpufreq-pxa3xx.c
 *
 * Copyright (C) 2008 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/semaphore.h>

#include <asm-generic/errno.h>

#include <mach/pxa3xx-regs.h>
#include <mach/pxa3xx-dvm.h>

#include "generic.h"

#define HSS_104M	(0)
#define HSS_156M	(1)
#define HSS_208M	(2)
#define HSS_312M	(3)

#define SMCFS_78M	(0)
#define SMCFS_104M	(2)
#define SMCFS_208M	(5)

#define SFLFS_104M	(0)
#define SFLFS_156M	(1)
#define SFLFS_208M	(2)
#define SFLFS_312M	(3)

#define XSPCLK_156M	(0)
#define XSPCLK_NONE	(3)

#define DMCFS_26M	(0)
#define DMCFS_260M	(3)

struct pxa3xx_freq_info {
	unsigned int cpufreq_mhz;
	unsigned int core_xl : 5;
	unsigned int core_xn : 3;
	unsigned int hss : 2;
	unsigned int dmcfs : 2;
	unsigned int smcfs : 3;
	unsigned int sflfs : 2;
	unsigned int df_clkdiv : 3;

	int	vcc_core;	/* in mV */
	int	vcc_sram;	/* in mV */
};

#define OP(cpufreq, _xl, _xn, _hss, _dmc, _smc, _sfl, _dfi, vcore, vsram) \
{									\
	.cpufreq_mhz	= cpufreq,					\
	.core_xl	= _xl,						\
	.core_xn	= _xn,						\
	.hss		= HSS_##_hss##M,				\
	.dmcfs		= DMCFS_##_dmc##M,				\
	.smcfs		= SMCFS_##_smc##M,				\
	.sflfs		= SFLFS_##_sfl##M,				\
	.df_clkdiv	= _dfi,						\
	.vcc_core	= vcore,					\
	.vcc_sram	= vsram,					\
}

static struct pxa3xx_freq_info pxa300_freqs[] = {
	/*  CPU XL XN  HSS DMEM SMEM SRAM DFI VCC_CORE VCC_SRAM */
	OP(104,  8, 1, 104, 260,  78, 104, 3, 1100, 1100), /* 104MHz */
	OP(208, 16, 1, 104, 260, 104, 156, 2, 1100, 1100), /* 208MHz */
	OP(416, 16, 2, 156, 260, 104, 208, 2, 1100, 1100), /* 416MHz */
	OP(624, 24, 2, 208, 260, 208, 312, 3, 1375, 1375), /* 624MHz */
};

static struct pxa3xx_freq_info pxa320_freqs[] = {
	/*  CPU XL XN  HSS DMEM SMEM SRAM DFI VCC_CORE VCC_SRAM */
	OP(104,  8, 1, 104, 260,  78, 104, 3, 1100, 1100), /* 104MHz */
	OP(208, 16, 1, 104, 260, 104, 156, 2, 1100, 1100), /* 208MHz */
	OP(416, 16, 2, 156, 260, 104, 208, 2, 1100, 1100), /* 416MHz */
	OP(624, 24, 2, 208, 260, 208, 312, 3, 1375, 1375), /* 624MHz */
	OP(806, 31, 2, 208, 260, 208, 312, 3, 1400, 1400), /* 806MHz */
};

static unsigned int pxa3xx_freqs_num;
static struct pxa3xx_freq_info *pxa3xx_freqs;
static struct cpufreq_frequency_table *pxa3xx_freqs_table;

/* VCC_CORE & VCC_SRAM regulators */
static struct regulator *pxa3xx_reg_vcore;
static struct regulator *pxa3xx_reg_vsram;

/* Actual operating point set by cpufreq */
static struct pxa3xx_freq_info *pxa3xx_actual_freq;

/* Protects "frequency change" resource */
static DEFINE_SEMAPHORE(pxa3xx_dvm_sema);
/* Enable DVM on cpufreq transitions */
static unsigned int pxa3xx_allow_cpufreq_dvm;

/* Required voltage tolerance (in uV): +-10mV */
#define PXA3XX_DVM_TOL		10000
/* "Shutdown" voltage levels */
#define PXA3XX_DVM_VCORE_SHDN	1400000
#define PXA3XX_DVM_VSRAM_SHDN	1400000
/* "Suspend" voltage levels */
#define PXA3XX_DVM_VCORE_SUSP	1400000
#define PXA3XX_DVM_VSRAM_SUSP	1400000

static int pxa3xx_dvm_set_vcore (int volt_uV)
{
	int ret;

	if(!pxa3xx_reg_vcore)
		return 0;

	pr_debug("DVM: VCORE req %duV", volt_uV);
	ret = regulator_set_voltage(pxa3xx_reg_vcore,
				    volt_uV - PXA3XX_DVM_TOL,
				    volt_uV + PXA3XX_DVM_TOL);
	if(ret)
		pr_err("\nDVM: unable to set VCORE: %d (%duV)\n", ret, volt_uV);

	return ret;
}

static int pxa3xx_dvm_set_vsram (int volt_uV)
{
	int ret;

	if(!pxa3xx_reg_vsram)
		return 0;

	pr_debug("DVM: VSRAM req %duV\n", volt_uV);
	ret = regulator_set_voltage(pxa3xx_reg_vsram,
				    volt_uV - PXA3XX_DVM_TOL,
				    volt_uV + PXA3XX_DVM_TOL);
	if(ret)
		pr_err("DVM: unable to set VSRAM: %d (%duV)\n", ret, volt_uV);

	return ret;
}

static inline int _vcomp(int volt_req, int volt_curr)
{
	if(volt_curr > volt_req + PXA3XX_DVM_TOL)
		return 1;
	else if (volt_curr < volt_req - PXA3XX_DVM_TOL)
		return -1;
	return 0;
}

static int pxa3xx_cpufreq_dvm(struct pxa3xx_freq_info *freq, int stage)
{
	int ret = 0;
	int vcore, vsram, req_vcore, req_vsram; /* in uV */
	/*
	 * Check for freq == 0, because someday supplied pxa3xx_actual_freq
	 * might be NULL
	 */
	if(!pxa3xx_allow_cpufreq_dvm || !pxa3xx_reg_vcore || !freq)
		return 0;

	req_vcore = freq->vcc_core * 1000;
	req_vsram = freq->vcc_sram * 1000;

	vcore = regulator_get_voltage(pxa3xx_reg_vcore);
	if((_vcomp(req_vcore, vcore) > 0 && stage == CPUFREQ_POSTCHANGE) ||
	   (_vcomp(req_vcore, vcore) < 0 && stage == CPUFREQ_PRECHANGE))
			ret = pxa3xx_dvm_set_vcore(req_vcore);
	if(ret || !pxa3xx_reg_vsram)
		return ret;

	vsram = regulator_get_voltage(pxa3xx_reg_vsram);
	if((_vcomp(req_vsram, vsram) > 0 && stage == CPUFREQ_POSTCHANGE) ||
	   (_vcomp(req_vsram, vsram) < 0 && stage == CPUFREQ_PRECHANGE))
			ret = pxa3xx_dvm_set_vsram(req_vsram);

	return ret;
}

/* cpufreq driver */

static int setup_freqs_table(struct cpufreq_policy *policy,
			     struct pxa3xx_freq_info *freqs, int num)
{
	struct cpufreq_frequency_table *table;
	int i;

	table = kzalloc((num + 1) * sizeof(*table), GFP_KERNEL);
	if (table == NULL)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		table[i].index = i;
		table[i].frequency = freqs[i].cpufreq_mhz * 1000;
	}
	table[num].index = i;
	table[num].frequency = CPUFREQ_TABLE_END;

	pxa3xx_freqs = freqs;
	pxa3xx_freqs_num = num;
	pxa3xx_freqs_table = table;

	return cpufreq_frequency_table_cpuinfo(policy, table);
}

static void __update_core_freq(struct pxa3xx_freq_info *info)
{
	uint32_t mask = ACCR_XN_MASK | ACCR_XL_MASK;
	uint32_t accr = ACCR;
	uint32_t xclkcfg;

	accr &= ~(ACCR_XN_MASK | ACCR_XL_MASK | ACCR_XSPCLK_MASK);
	accr |= ACCR_XN(info->core_xn) | ACCR_XL(info->core_xl);

	/* No clock until core PLL is re-locked */
	accr |= ACCR_XSPCLK(XSPCLK_NONE);

	xclkcfg = (info->core_xn == 2) ? 0x3 : 0x2;	/* turbo bit */

	ACCR = accr;
	__asm__("mcr p14, 0, %0, c6, c0, 0\n" : : "r"(xclkcfg));

	while ((ACSR & mask) != (accr & mask))
		cpu_relax();
}

static void __update_bus_freq(struct pxa3xx_freq_info *info)
{
	uint32_t mask;
	uint32_t accr = ACCR;

	mask = ACCR_SMCFS_MASK | ACCR_SFLFS_MASK | ACCR_HSS_MASK |
		ACCR_DMCFS_MASK;

	accr &= ~mask;
	accr |= ACCR_SMCFS(info->smcfs) | ACCR_SFLFS(info->sflfs) |
		ACCR_HSS(info->hss) | ACCR_DMCFS(info->dmcfs);

	ACCR = accr;

	while ((ACSR & mask) != (accr & mask))
		cpu_relax();
}

static int pxa3xx_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, pxa3xx_freqs_table);
}

static unsigned int pxa3xx_cpufreq_get(unsigned int cpu)
{
	return pxa3xx_get_clk_frequency_khz(0);
}

static int pxa3xx_cpufreq_set(struct cpufreq_policy *policy,
			      unsigned int target_freq,
			      unsigned int relation)
{
	struct pxa3xx_freq_info *next;
	struct cpufreq_freqs freqs;
	unsigned long flags;
	int idx, ret;

	if (policy->cpu != 0)
		return -EINVAL;

	/* Lookup the next frequency */
	if (cpufreq_frequency_table_target(policy, pxa3xx_freqs_table,
				target_freq, relation, &idx))
		return -EINVAL;

	next = &pxa3xx_freqs[idx];

	freqs.old = policy->cur;
	freqs.new = next->cpufreq_mhz * 1000;
	freqs.cpu = policy->cpu;

	pr_debug("CPU frequency from %d MHz to %d MHz%s\n",
			freqs.old / 1000, freqs.new / 1000,
			(freqs.old == freqs.new) ? " (skipped)" : "");

	if (freqs.old == target_freq)
		return 0;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	down(&pxa3xx_dvm_sema);
	/* Pre-change voltage scaling */
	if((ret = pxa3xx_cpufreq_dvm(next, CPUFREQ_PRECHANGE))) {
		up(&pxa3xx_dvm_sema);
		return ret;
	}

	local_irq_save(flags);
	__update_core_freq(next);
	__update_bus_freq(next);
	local_irq_restore(flags);

	pxa3xx_actual_freq = next;
	/* Post-change voltage scaling */
	pxa3xx_cpufreq_dvm(next, CPUFREQ_POSTCHANGE);
	up(&pxa3xx_dvm_sema);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

static int pxa3xx_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret = -EINVAL;

	/* set default policy and cpuinfo */
	policy->cpuinfo.min_freq = 104000;
	policy->cpuinfo.max_freq = (cpu_is_pxa320()) ? 806000 : 624000;
	policy->cpuinfo.transition_latency = 1000; /* FIXME: 1 ms, assumed */
	policy->max = pxa3xx_get_clk_frequency_khz(0);
	policy->cur = policy->min = policy->max;

	if (cpu_is_pxa300() || cpu_is_pxa310())
		ret = setup_freqs_table(policy, ARRAY_AND_SIZE(pxa300_freqs));

	if (cpu_is_pxa320())
		ret = setup_freqs_table(policy, ARRAY_AND_SIZE(pxa320_freqs));
	if (ret) {
		pr_err("failed to setup frequency table\n");
		return ret;
	}

	/* Assume that top frequency is set */
	pxa3xx_actual_freq = &pxa3xx_freqs[pxa3xx_freqs_num - 1];
	pr_info("CPUFREQ support for PXA3xx initialized\n");
	return 0;
}

static struct cpufreq_driver pxa3xx_cpufreq_driver = {
	.verify		= pxa3xx_cpufreq_verify,
	.target		= pxa3xx_cpufreq_set,
	.init		= pxa3xx_cpufreq_init,
	.get		= pxa3xx_cpufreq_get,
	.name		= "pxa3xx-cpufreq",
};

/* pxa3xx-dvm platform device */

static int pxa3xx_dvm_probe (struct platform_device *pdev)
{
	struct pxa3xx_dvm_pdata *pdata = pdev->dev.platform_data;
	int ret;

	if(pdev->id != -1) {
		dev_err(&pdev->dev, "device id must be -1\n");
		return -EINVAL;
	}
	if(!pdata) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	pxa3xx_reg_vcore = regulator_get(&pdev->dev, "VCC_CORE");
	if(!pxa3xx_reg_vcore) {
		dev_err(&pdev->dev, "unable to get VCC_CORE regulator\n");
		return -ENXIO;
	}

	pxa3xx_reg_vsram = regulator_get(&pdev->dev, "VCC_SRAM");

	while(pdata->pm_state_logic) {
		if((ret = regulator_enable(pxa3xx_reg_vcore))) {
			dev_err(&pdev->dev, "unable to enable VCC_CORE\n");
			goto err;
		}
		if(!pxa3xx_reg_vsram)
			break;
		if((ret = regulator_enable(pxa3xx_reg_vsram))) {
			dev_err(&pdev->dev, "unable to enable VCC_SRAM\n");
			goto err;
		}
		break;
	}

	down(&pxa3xx_dvm_sema);
	pxa3xx_allow_cpufreq_dvm = pdata->freq_transition_logic;
	pxa3xx_cpufreq_dvm(pxa3xx_actual_freq, CPUFREQ_PRECHANGE);
	pxa3xx_cpufreq_dvm(pxa3xx_actual_freq, CPUFREQ_POSTCHANGE);
	up(&pxa3xx_dvm_sema);

	dev_info(&pdev->dev, "PXA3xx voltage sequencer enabled\n");

	return 0;
err:
	while(pdata->pm_state_logic) {
		regulator_disable(pxa3xx_reg_vcore);
		if(!pxa3xx_reg_vsram)
			break;
		regulator_disable(pxa3xx_reg_vsram);
		break;
	}
	regulator_put(pxa3xx_reg_vcore);
	if(pxa3xx_reg_vsram)
		regulator_put(pxa3xx_reg_vsram);
	return ret;
}

static int pxa3xx_dvm_remove(struct platform_device *pdev)
{
	struct pxa3xx_dvm_pdata *pdata = pdev->dev.platform_data;

	down(&pxa3xx_dvm_sema);

	pxa3xx_dvm_set_vcore(PXA3XX_DVM_VCORE_SHDN);
	if(pdata->pm_state_logic)
		regulator_disable(pxa3xx_reg_vcore);
	regulator_put(pxa3xx_reg_vcore);
	pxa3xx_reg_vcore = NULL;

	if(!pxa3xx_reg_vsram)
		goto out;
	pxa3xx_dvm_set_vsram(PXA3XX_DVM_VSRAM_SHDN);
	if(pdata->pm_state_logic)
		regulator_disable(pxa3xx_reg_vsram);
	regulator_put(pxa3xx_reg_vsram);
	pxa3xx_reg_vsram = NULL;

out:
	pxa3xx_allow_cpufreq_dvm = 0;
	up(&pxa3xx_dvm_sema);
	return 0;
}

static void pxa3xx_dvm_pm_shutdown(struct platform_device *pdev)
{
	struct pxa3xx_dvm_pdata *pdata = pdev->dev.platform_data;

	down(&pxa3xx_dvm_sema);

	pxa3xx_dvm_set_vcore(PXA3XX_DVM_VCORE_SHDN);
	if(pxa3xx_reg_vsram)
		pxa3xx_dvm_set_vsram(PXA3XX_DVM_VSRAM_SHDN);

	if(!pdata->pm_state_logic)
		goto out;
	if(regulator_disable(pxa3xx_reg_vcore))
		dev_err(&pdev->dev, "unable to disable VCC_CORE\n");
	if(pxa3xx_reg_vsram && regulator_disable(pxa3xx_reg_vsram))
		dev_err(&pdev->dev, "unable to disable VCC_SRAM\n");

out:
	pxa3xx_allow_cpufreq_dvm = 0;
	up(&pxa3xx_dvm_sema);
}

#ifdef CONFIG_PM
static int pxa3xx_dvm_pm_resume(struct platform_device *pdev)
{
	struct pxa3xx_dvm_pdata *pdata = pdev->dev.platform_data;
	int ret = 0;

	down(&pxa3xx_dvm_sema);
	/* Enable reg_vcore */
	if(pdata->pm_state_logic && regulator_enable(pxa3xx_reg_vcore))
		dev_warn(&pdev->dev, "unable to enable VCC_CORE: %d\n", ret);
	/* Apply correct voltages */
	if(pxa3xx_cpufreq_dvm(pxa3xx_actual_freq, CPUFREQ_PRECHANGE) ||
	   pxa3xx_cpufreq_dvm(pxa3xx_actual_freq, CPUFREQ_POSTCHANGE))
		dev_warn(&pdev->dev, "unable to restore voltages: %d\n", ret);

	pxa3xx_allow_cpufreq_dvm = pdata->freq_transition_logic;
	up(&pxa3xx_dvm_sema);
	return 0;
}

static int pxa3xx_dvm_pm_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct pxa3xx_dvm_pdata *pdata = pdev->dev.platform_data;
	int ret;

	down(&pxa3xx_dvm_sema);

	if((ret = pxa3xx_dvm_set_vcore(PXA3XX_DVM_VCORE_SUSP)))
		goto fallback;
	if((ret = pxa3xx_dvm_set_vsram(PXA3XX_DVM_VCORE_SUSP)))
		goto fallback;

	if(!pdata->pm_state_logic)
		goto out;

	if(pxa3xx_reg_vsram && !regulator_is_enabled(pxa3xx_reg_vsram))
		if((ret = regulator_enable(pxa3xx_reg_vsram))) {
			dev_err(&pdev->dev, "unable to enable VCC_SRAM\n");
			goto fallback;
		}
	if((ret = regulator_disable(pxa3xx_reg_vcore))) {
		dev_err(&pdev->dev, "unable to disable VCC_CORE\n");
		goto fallback;
	}

out:
	pxa3xx_allow_cpufreq_dvm = 0;
	up(&pxa3xx_dvm_sema);
	return 0;
fallback:
	dev_warn(&pdev->dev, "abort suspend\n");
	/* Restore voltages */
	pxa3xx_cpufreq_dvm(pxa3xx_actual_freq, CPUFREQ_PRECHANGE);
	pxa3xx_cpufreq_dvm(pxa3xx_actual_freq, CPUFREQ_POSTCHANGE);
	up(&pxa3xx_dvm_sema);
	return ret;
}
#endif

static struct platform_driver pxa3xx_dvm = {
	.probe = pxa3xx_dvm_probe,
	.remove = pxa3xx_dvm_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "pxa3xx-dvm",
	},
#ifdef CONFIG_PM
	.suspend = pxa3xx_dvm_pm_suspend,
	.resume = pxa3xx_dvm_pm_resume,
#endif
	.shutdown = pxa3xx_dvm_pm_shutdown,
};

static int __init cpufreq_init(void)
{
	if (cpu_is_pxa3xx()) {
		int ret;
		ret = cpufreq_register_driver(&pxa3xx_cpufreq_driver);
		if(ret)
			return ret;
		return platform_driver_register(&pxa3xx_dvm);
	}

	return 0;
}
module_init(cpufreq_init);

static void __exit cpufreq_exit(void)
{
	platform_driver_unregister(&pxa3xx_dvm);
	cpufreq_unregister_driver(&pxa3xx_cpufreq_driver);
}
module_exit(cpufreq_exit);

MODULE_DESCRIPTION("CPU frequency scaling driver for PXA3xx");
MODULE_LICENSE("GPL");
