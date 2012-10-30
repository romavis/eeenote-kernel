#ifndef ASMARM_ARCH_MMC_MULTI_H
#define ASMARM_ARCH_MMC_MULTI_H

#include <linux/interrupt.h>

struct device;

/*
 * pxamci-multi driver is developed primarily for ASUS EeeNote device,
 *  but it can be used on other platforms with crazy sd/mmc cards switching ;)
 * Obviously, there is no support for SDIO IRQ in such system...
 * 
 * You must supply:
 * -card selection routine ("select" callback), which accepts selected 
 *  slot index as parameter and returns 0 on success
 * -valid ocr_mask
 * -slot params :)
 *
 * Driver supports switching cards' power via single power rail, connected
 *  to all cards, via GPIO and/or via "setpower" callback.
 *  Card powerdowns and powerups are balanced internally.
 *
 * "init" and "exit" callbacks are called when driver is loaded/unloaded
 * 
 * For configuration example, look into etablet.c
 */
struct pxamci_multi_platform_data {
	unsigned int ocr_mask;		/* available voltages */
	unsigned long detect_delay_ms;	/* card detection delay */
	
	int (*init)(struct device *, irq_handler_t , void *);
	void (*exit)(struct device *, void *);
	
	int gpio_power;			/* gpio powering up MMC bus */
	bool gpio_power_invert;		/* gpio power is inverted */
	int (*setpower)(struct device *, unsigned int);
	
	struct pxamci_multi_platform_slot_info* slot_info;
	unsigned int slot_count;
	
	int (*select)(struct device*, unsigned int);
};

struct pxamci_multi_platform_slot_info {
	int (*get_ro)(struct device *, unsigned int);
	
	int gpio_card_detect;		/* gpio detecting card insertion */	
	int gpio_card_ro;		/* gpio detecting read only toggle */
	bool gpio_card_ro_invert;	/* gpio ro is inverted */
	
	bool removable;			/* card IS removable */
};

#endif
