#ifndef __ASM_ARCH_ZETABLET_H
#define __ASM_ARCH_ZETABLET_H

/*
 * Define some GPIOs here ;)
 */

#define ETABLET_GPIO_CHRG_PG		0	/* VBUS sensing */
#define ETABLET_GPIO_HP_DETECT		1	/* Jack inserion (invert) */
#define	ETABLET_GPIO_MMC1_SLOT1_CMD	8	/* External SD selection */
#define	ETABLET_GPIO_MMC1_SLOT0_CMD	15	/* Internal SD selection */
#define ETABLET_GPIO_MMC1_SLOT1_CD	53	/* External SD detection */
#define ETABLET_GPIO_LDS_IRQ		76	/* Sensor board IRQ */
#define	ETABLET_GPIO_LCD_BACKLIGHT	77	/* LCD fake backlight (??) */
#define ETABLET_GPIO_CHRG_CHRG		79	/* "Battery charging" signal */
#define ETABLET_GPIO_KEY_SSW		80	/* UNK (maybe LDS GPIO ?) */
#define	ETABLET_GPIO_LCD_BLANK		83	/* LCD blank */
#define	ETABLET_GPIO_WLAN_RESET		86	/* WLAN chip reset (inv) */
#define ETABLET_GPIO_SPKPWR		125	/* Amp control (invert) */
#define	ETABLET_GPIO_KEY_SUSPEND	127	/* Power button (invert) */

#endif /* __ASM_ARCH_ZETABLET_H */
