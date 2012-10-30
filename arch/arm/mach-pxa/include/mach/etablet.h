/*
 * arch/arm/mach-pxa/include/mach/etablet.h
 *
 * Support for the ASUS EeeNote EA-800 platform
 *
 * Copyright (C) 2012 Roman Dobrodiy
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation; version 2 ONLY.
 *
 */
#ifndef ASMARM_ARCH_ETABLET_H
#define ASMARM_ARCH_ETABLET_H

/*
 * Define some GPIOs here ;)
 * AL - active low, AH - active high
 */

#define ETABLET_GPIO_CHRG_PG		0	/* VBUS sensing - AH */
#define ETABLET_GPIO_HP_DETECT		1	/* Jack inserion - AL */
#define ETABLET_GPIO_CHRG_BAT_ALERT	2	/* Battery alert (??) */

#define	ETABLET_GPIO_MMC1_SLOT1_CMD	8	/* External SD CMD line */
#define	ETABLET_GPIO_MMC1_SLOT0_CMD	15	/* Internal SD CMD line */
#define ETABLET_GPIO_MMC1_SLOT1_CD	53	/* External SD detection - AL */

#define ETABLET_GPIO_LDS_IRQ		76	/* Sensor board IRQ */
#define	ETABLET_GPIO_LCD_BACKLIGHT	77	/* LCD fake backlight (??) */
#define ETABLET_GPIO_CHRG_CHRG		79	/* "Battery charging" signal */
#define ETABLET_GPIO_KEY_SSW		80	/* UNK (LDS6128 GPIO ?) */

#define ETABLET_GPIO_WACOM_PDCT		81	/* Wacom pen detect - AH */
#define ETABLET_GPIO_WACOM_SLEEP	82	/* Wacom sleep - AH (??) */

#define	ETABLET_GPIO_LCD_BLANK		83	/* LCD blanking - AL */
#define	ETABLET_GPIO_WLAN_RESET		86	/* WLAN chip reset - AL */
#define ETABLET_GPIO_OTG_ID		106	/* OTG ID pin (??) */
#define ETABLET_GPIO_I2C_BRIDGE		119	/* PCA9515 enable - AH */
#define ETABLET_GPIO_SPKPWR		125	/* Amp control - AL */
#define	ETABLET_GPIO_KEY_SUSPEND	127	/* Power button - AL */

#endif /* ASMARM_ARCH_ETABLET_H */
