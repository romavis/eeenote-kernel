/*
 * arch/arm/mach-pxa/include/mach/pxa3xx-dvm.h
 *
 * PXA3xx software dynamic voltage management
 *
 * Copytight (C) 2012 Roman Dobrodiy
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation; version 2 ONLY.
 *
 */

#ifndef ASMARM_ARCH_PXA3XX_DVM_H
#define ASMARM_ARCH_PXA3XX_DVM_H

/*
 * pxa3xx-dvm platform data (read description for more information)
 *
 * freq_transition_logic - enable voltage sequencing on cpufreq transitions
 * pm_state_logic - enable regulator state control(enable/disable) on PM
 * 	transitions. Doesn't affect voltage control.
 */
struct pxa3xx_dvm_pdata {
	unsigned freq_transition_logic :1;
	unsigned pm_state_logic :1;
};

#endif /* ASMARM_ARCH_PXA3XX_DVM_H */
