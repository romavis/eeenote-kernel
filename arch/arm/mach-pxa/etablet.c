/*
 * linux/arch/arm/mach-pxa/etablet.c
 *
 * Support for the ASUS EeeNote EA-800 platform
 * 
 *
 *
 * 2012-01-03: Roman Dobrodiy <ztcoils@gmail.com>
 *             port to latest (3.1) kernel
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation; version 2 ONLY.
 * 
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/syscore_ops.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/sizes.h>

#include <mach/pxa300.h>
#include <mach/audio.h>
#include <mach/pxafb.h>
#include <mach/pxa27x-udc.h>
#include <mach/udc.h>
#include <mach/mmc-multi.h>
#include <mach/mmc.h>
#include <mach/ohci.h>

#include <mach/etablet.h>

#include <linux/gpio_keys.h>
#include <linux/usb/gpio_vbus.h>
//#include <linux/i2c.h>
#include <linux/i2c/pxa-i2c.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max8660.h>
#include <linux/power/bq27x00_battery.h>
#include <linux/power/gpio-charger.h>
#include <linux/lds61xx.h>

#include "devices.h"
#include "generic.h"

/*
 * Machine specs:
 * 
 * PXA303 processor
 * XGA chimei_tentative TFT LCD (requires PCDDIV to be set!)
 * MAX8660 power management IC on PWR-I2C
 * 
 * Wacom touchscreen on FFUART, with GPIO lines
 * 
 * BTUART is debug port
 * 
 * RT5611 AC97 audio codec
 * 
 * BQ27541 gauge on I2C (inside battery pack)
 * BQ24072 charger with GPIO signals
 * 	GPIO79 - charging 
 * 	GPIO0_2 - power_good (detect VBUS)
 * 
 * CIF camera
 * 
 * WIFI on MMC2 SDIO (so, only SPI mode) with GPIO reset
 * 
 * MMC1 inner microSD + external microSD
 * MMC2 - WIFI
 * 
 * GPIO key "Suspend"
 * LDS6128 sensor keypad on I2C, with IRQ on pin 76
 * Suspend LED controlled by LDS6128
 */ 

/*
 * MFP configs
 */

/* 
 * LCD
 */
static mfp_cfg_t lcd_mfp_cfg[] __initdata = {
	/* Data */
	GPIO54_LCD_LDD_0,
	GPIO55_LCD_LDD_1,
	GPIO56_LCD_LDD_2,
	GPIO57_LCD_LDD_3,
	GPIO58_LCD_LDD_4,
	GPIO59_LCD_LDD_5,
	GPIO60_LCD_LDD_6,
	GPIO61_LCD_LDD_7,
	GPIO62_LCD_LDD_8,
	GPIO63_LCD_LDD_9,
	GPIO64_LCD_LDD_10,
	GPIO65_LCD_LDD_11,
	GPIO66_LCD_LDD_12,
	GPIO67_LCD_LDD_13,
	GPIO68_LCD_LDD_14,
	GPIO69_LCD_LDD_15,
	GPIO70_LCD_LDD_16,
	GPIO71_GPIO,				// ??
	
	/* Clocks */
	GPIO72_LCD_FCLK,
	GPIO73_LCD_LCLK,
	MFP_CFG_DRV(GPIO74, AF1, DS04X),	//PCLK high current
	GPIO75_LCD_BIAS,
	GPIO76_GPIO,
	
	GPIO83_GPIO | MFP_LPM_DRIVE_LOW,	//Blank
	MFP_CFG_DRV(GPIO77, AF0, DS13X)		//Backlight
};
	
/*
 * UARTs
 */
static mfp_cfg_t uart_mfp_cfg[] __initdata = {
	/* FFUART */
	GPIO30_UART1_RXD,
	GPIO31_UART1_TXD,
	GPIO32_UART1_CTS,
	GPIO33_UART1_DCD,
	GPIO34_UART1_DSR,
	GPIO35_UART1_RI,
	GPIO36_UART1_DTR,
	GPIO37_UART1_RTS,

	/* BTUART */
	GPIO111_UART2_RTS,
	GPIO112_UART2_RXD | MFP_LPM_EDGE_BOTH,
	GPIO113_UART2_TXD,
	GPIO114_UART2_CTS,
	
	/* STUART */
	GPIO109_UART3_TXD,
	GPIO110_UART3_RXD,
};

/*
 * AC97 - ALC5611 codec
 */
static mfp_cfg_t ac97_mfp_cfg[] __initdata = {
	GPIO23_AC97_nACRESET 	| MFP_LPM_PULL_LOW,
	
	GPIO24_AC97_SYSCLK,
	//GPIO24_GPIO 		| MFP_LPM_PULL_LOW,
	GPIO25_AC97_SDATA_IN_0,
	GPIO27_AC97_SDATA_OUT,
	GPIO28_AC97_SYNC,
	GPIO29_AC97_BITCLK,
	
	/* Amp control and HP insertion detect */
	GPIO125_GPIO		| MFP_LPM_PULL_HIGH,
	GPIO1_2_GPIO,
};

/*
 * CIF Camera
 */
static mfp_cfg_t cif_mfp_cfg[] __initdata = {
	/* QCI proto */
	GPIO39_CI_DD_0,
	GPIO40_CI_DD_1,
	GPIO41_CI_DD_2,
	GPIO42_CI_DD_3,
	GPIO43_CI_DD_4,
	GPIO44_CI_DD_5,
	GPIO45_CI_DD_6,
	GPIO46_CI_DD_7,
	GPIO47_CI_DD_8,
	GPIO48_CI_DD_9,
	GPIO49_CI_MCLK,
	GPIO50_CI_PCLK,
	GPIO51_CI_HSYNC,
	GPIO52_CI_VSYNC
};

/*
 * Generic MFP
 */
static mfp_cfg_t generic_mfp_cfg[] __initdata = {
	GPIO76_GPIO,			//LDS IRQ pin
	GPIO127_GPIO |\
		MFP_LPM_EDGE_FALL | MFP_LPM_PULL_HIGH, //Suspend button
	GPIO106_USB_P2_7, 		//OTG ID pin
	GPIO86_GPIO,			//WLAN reset
	GPIO79_GPIO,			//Charger "charging" pin
	GPIO0_2_GPIO |\
		MFP_LPM_EDGE_BOTH | MFP_LPM_FLOAT, //Charger "power ok" pin
	/* Standard I2C */
	GPIO21_I2C_SCL,
	GPIO22_I2C_SDA,
};

/*
 * MMC generic
 */
static mfp_cfg_t mmc_mfp_cfg[] __initdata = {
	/* MMC1 */
	GPIO53_GPIO,		//External mSD card detect
	GPIO3_MMC1_DAT0,
	GPIO4_MMC1_DAT1,
	GPIO5_MMC1_DAT2,
	GPIO6_MMC1_DAT3,
	GPIO7_MMC1_CLK,
	
	/* MMC2 */
	GPIO9_MMC2_DAT0,
	GPIO10_MMC2_DAT1,
	GPIO11_MMC2_DAT2,
	GPIO12_MMC2_DAT3,
	GPIO13_MMC2_CLK,
	GPIO14_MMC2_CMD,
};

/*
 * Framebuffer
 */
#if defined(CONFIG_FB_PXA) || defined(CONFIG_FB_PXA_MODULE)
static void zetablet_lcd_power(int on, struct fb_var_screeninfo *screen)
{
	/* MFP for MFP_BACKLIGHT_PWM is supposed to be configured */
	gpio_request(ETABLET_GPIO_LCD_BACKLIGHT	, "LCD Backlight");
	gpio_request(ETABLET_GPIO_LCD_BLANK	, "LCD Blank");
	if(on){
		gpio_direction_output(ETABLET_GPIO_LCD_BACKLIGHT, 1);
		mdelay(5);
		gpio_direction_output(ETABLET_GPIO_LCD_BLANK, 1);
	}
	else{
		gpio_direction_output(ETABLET_GPIO_LCD_BLANK, 0);
		mdelay(5);
		gpio_direction_output(ETABLET_GPIO_LCD_BACKLIGHT, 0);
	}
}

static struct pxafb_mode_info chimei_tentative_mode = {
	.pixclock	= 19230,
	.xres		= 768,
	.yres		= 1024,
	.bpp		= 16,
	//.bpp		= 8,
	.hsync_len	= 64,
	.left_margin	= 94,
	.right_margin	= 94,
	.vsync_len	= 2,
	.upper_margin	= 4,
	.lower_margin	= 4,
	.sync		= 0,
};

static struct pxafb_mach_info chimei_tentative_info = {
	.num_modes      	= 1,
	.cmap_inverse		= 1,
	
	.lccr0			= LCCR0_Act,
	.lccr3			= LCCR3_OutEnH | LCCR3_PixRsEdg,
	.lccr4			= LCCR4_PCDDIV,
	
	.pxafb_backlight_power	= NULL,
	.pxafb_lcd_power	= zetablet_lcd_power,
	
	.modes			= &chimei_tentative_mode
};

static void __init zetablet_init_lcd(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(lcd_mfp_cfg));
	/*
	 * Just use our chimey XGA LCD without any checks
	 */
	pxa_set_fb_info(NULL, &chimei_tentative_info);
}
#else
static inline void zetablet_init_lcd(void) {}
#endif


/*
 * MMC stuff
 */
static mfp_cfg_t mmc_sel_0_mfp_cfg[] = {
	GPIO8_GPIO | MFP_LPM_PULL_HIGH,
	GPIO15_MMC1_CMD,
};

static mfp_cfg_t mmc_sel_1_mfp_cfg[] = {
	GPIO8_MMC1_CMD,
	GPIO15_GPIO | MFP_LPM_PULL_HIGH,
};

static mfp_cfg_t mmc_sel_nothing_mfp_cfg[] = {
	GPIO8_GPIO  | MFP_LPM_PULL_HIGH,
	GPIO15_GPIO | MFP_LPM_PULL_HIGH,
};

#if defined(CONFIG_MMC)
static int zetablet_mci_multi_select_slot(struct device* dev, unsigned int id)
{
	if (id==1) {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(mmc_sel_1_mfp_cfg));
		gpio_direction_output(ETABLET_GPIO_MMC1_SLOT0_CMD, 1);
		return 0;
	} else if(id==0) {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(mmc_sel_0_mfp_cfg));
		gpio_direction_output(ETABLET_GPIO_MMC1_SLOT1_CMD, 1);
		return 0;
	}
	
	pxa3xx_mfp_config(ARRAY_AND_SIZE(mmc_sel_nothing_mfp_cfg));
	gpio_direction_output(ETABLET_GPIO_MMC1_SLOT1_CMD, 1);
	gpio_direction_output(ETABLET_GPIO_MMC1_SLOT0_CMD, 1);
	return -EINVAL;
}

static struct pxamci_multi_platform_slot_info zetablet_mci_multi_slot_info[] = {
	[0] = {
		.gpio_card_detect = -1,
		.gpio_card_ro = -1,
		.removable = 0,
	},
	[1] = {
		.gpio_card_detect = ETABLET_GPIO_MMC1_SLOT1_CD,
		.gpio_card_ro = -1,
		.removable = 1,
	},
};

static struct pxamci_multi_platform_data zetablet_mci_multi_pdata = {
	.detect_delay_ms	= 200,
	.ocr_mask		= MMC_VDD_32_33|MMC_VDD_33_34,
	.gpio_power		= -1,
	.select			= zetablet_mci_multi_select_slot,
	.slot_info		= zetablet_mci_multi_slot_info,
	.slot_count		= ARRAY_SIZE(zetablet_mci_multi_slot_info)
};

static struct resource pxamci_multi_resources[] = {
	[0] = {
		.start	= 0x41100000,
		.end	= 0x41100fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_MMC,
		.end	= IRQ_MMC,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= 21,
		.end	= 21,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		.start	= 22,
		.end	= 22,
		.flags	= IORESOURCE_DMA,
	},
};

static u64 pxamci_multi_dmamask = 0xffffffffUL;

struct platform_device zetablet_device_pxamci_multi = {
	.name		= "pxamci-multi",
	.id		= 0,
	.dev		= {
			    .dma_mask = &pxamci_multi_dmamask,
			    .coherent_dma_mask = 0xffffffff,
			    .platform_data = &zetablet_mci_multi_pdata,
	},
	.num_resources	= ARRAY_SIZE(pxamci_multi_resources),
	.resource	= pxamci_multi_resources,
};

static struct pxamci_platform_data zetablet_mci2_pdata = {
	.detect_delay_ms= 200,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.gpio_card_detect = -1,
	.gpio_card_ro	= -1,
	.gpio_power	= ETABLET_GPIO_WLAN_RESET,
	.gpio_power_invert = 0,
};

static void __init zetablet_init_mmc(void)
{
	struct clk *mci_clk;
	struct clk_lookup *alias_clk_lu;
	
	/* Configure MFP */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(mmc_mfp_cfg));
	pxa3xx_mfp_config(ARRAY_AND_SIZE(mmc_sel_nothing_mfp_cfg));
	gpio_request(ETABLET_GPIO_MMC1_SLOT0_CMD, "MMC Slot 0 CMD");
	gpio_request(ETABLET_GPIO_MMC1_SLOT1_CMD, "MMC Slot 1 CMD");

	/* Reset WLAN */
	gpio_request(ETABLET_GPIO_WLAN_RESET, "WLAN Reset");
	gpio_direction_output(ETABLET_GPIO_WLAN_RESET, 0);
	mdelay(20);
	gpio_direction_output(ETABLET_GPIO_WLAN_RESET, 1);
	gpio_free(ETABLET_GPIO_WLAN_RESET);
	
	/* Create alias of first MCI controller's clock for 'multi' driver */
	mci_clk = clk_get_sys("pxa2xx-mci.0", NULL);  
        alias_clk_lu = clkdev_alloc(mci_clk, NULL, "pxamci-multi.0");
        clk_put(mci_clk);
        clkdev_add(alias_clk_lu);
	
	/* Register devices */
	platform_device_register(&zetablet_device_pxamci_multi);
	pxa3xx_set_mci2_info(&zetablet_mci2_pdata);
}
#else
static inline void __init zetablet_init_mmc(void)
{
	/* Put wifi to sleep, we can't use it without MMC */
	gpio_request(ETABLET_GPIO_WLAN_RESET, "WLAN Reset");
	gpio_direction_output(ETABLET_GPIO_WLAN_RESET, 0);
	
	/* Deselect slots */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(mmc_sel_nothing_mfp_cfg));
	gpio_request(ETABLET_GPIO_MMC1_SLOT0_CMD, "MMC Slot 0 CMD");
	gpio_request(ETABLET_GPIO_MMC1_SLOT1_CMD, "MMC Slot 1 CMD");
	gpio_direction_output(ETABLET_GPIO_MMC1_SLOT1_CMD, 1);
	gpio_direction_output(ETABLET_GPIO_MMC1_SLOT0_CMD, 1);
}
#endif

/*
 * GPIO buttons
 */
static struct gpio_keys_button zetablet_gpio_keys_buttons[] = {
	{
	  .code = KEY_SUSPEND,
	  .gpio = ETABLET_GPIO_KEY_SUSPEND,
	  .active_low = 1,
	  .desc = "gpio_key_suspend",
	  .wakeup = 1
	},
};

static struct gpio_keys_platform_data zetablet_gpio_keys_pdata = {
	.buttons = zetablet_gpio_keys_buttons,
	.nbuttons = ARRAY_SIZE(zetablet_gpio_keys_buttons),
};

static struct platform_device zetablet_device_gpio_keys = {
	.name = "gpio-keys",
	.dev  = {
		.platform_data = &zetablet_gpio_keys_pdata,
	},
	.id   = -1,
};

/*
 * OHCI
 */
static struct pxaohci_platform_data zetablet_ohci_pdata = {
	.port_mode	= PMM_PERPORT_MODE,
	.flags		= ENABLE_PORT1 | ENABLE_PORT2 |
			  POWER_CONTROL_LOW | POWER_SENSE_LOW,
};


static void __init zetablet_init_ohci(void)
{
	pxa_set_ohci_info(&zetablet_ohci_pdata);
}


/*
 * USB Device Controller
 */
static struct gpio_vbus_mach_info zetablet_gpiovbus_pdata = {
	.gpio_vbus = ETABLET_GPIO_CHRG_PG,
	.gpio_pullup = -1,
};

static struct platform_device zetablet_device_gpiovbus = {
	.name		= "gpio-vbus",
	.id		= -1,
	.dev		=  {
		.platform_data	= &zetablet_gpiovbus_pdata,
	}
};

/*
 * Process UDC connect/disconnect from host by toggling internal pullup
 * However, we shouldn't touch udc regs upon clock is running
 */
static void zetablet_udc_pullup(int cmd)
{
	struct clk *udc_clk;
	
	printk(KERN_INFO "udc pullup: setting %s\n", 
	       (cmd == PXA2XX_UDC_CMD_CONNECT) ? "on" : "off");
	
	udc_clk = clk_get_sys("pxa27x-udc", NULL);
	if(IS_ERR(udc_clk)){
		printk(KERN_ERR "udc pullup: can't get udc clock\n");
		return;
	}
	clk_enable(udc_clk);
	
	if(UP2OCR & UP2OCR_HXS){
		printk(KERN_ERR "udc pullup: host active, nothing to do\n");
		goto out;
	}
	
	if(cmd == PXA2XX_UDC_CMD_CONNECT)
		UP2OCR = UP2OCR_HXOE | UP2OCR_DPPUE;
	else
		UP2OCR = UP2OCR_HXOE;
	
	printk(KERN_INFO "udc pullup: UP2OCR after : %08x\n", UP2OCR);
	
out:
	clk_disable(udc_clk);
	clk_put(udc_clk);
}

static struct pxa2xx_udc_mach_info zetablet_udc_pdata = {
	.udc_command = zetablet_udc_pullup,
	.gpio_pullup = -1,
};

static void __init zetablet_init_udc(void)
{
	/* 
	 * Reset EMCE bit and disable UDC
	 * Driver won't do this and'll fail at probing
	 */
	UDCCR = UDCCR_EMCE;
	/* 
	 * Configure PXA USB Port2 to work as device
	 * SEOS = 0 (disable single-ended port)
	 * HXOE = 1 (enable transceiver)
	 * HXS = 0 (connect transceiver to UDC)
	 */
	UP2OCR = UP2OCR_HXOE;
	
	/* Register simple 'gpio_vbus' for cable plug-in detection */
	platform_device_register(&zetablet_device_gpiovbus);
	pxa_set_udc_info(&zetablet_udc_pdata);
}

/*
 * MAX8660 PMIC support
 */
static struct regulator_consumer_supply max8660_v6_consumers[] = {
	{
		.supply = "V6",
	}
};

static struct regulator_init_data max8660_v6_info = {
	.constraints = {
		.name = "V6",
		.min_uV = 1800000,
		.max_uV = 3300000,
		.boot_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(max8660_v6_consumers),
	.consumer_supplies = max8660_v6_consumers,
};

static struct max8660_subdev_data max8660_subdevs[] = {
	{ 
		.name = "V6", 
		.id = MAX8660_V6,
		.platform_data = &max8660_v6_info
	},
};

static struct max8660_platform_data max8660_pdata = {
	.subdevs = max8660_subdevs,
	.num_subdevs = ARRAY_SIZE(max8660_subdevs),
	.en34_is_high = 0,
};

/*
 * PWR I2C devices
 */
static struct i2c_pxa_platform_data zetablet_pwri2c_pdata = {
	.fast_mode = 1,
};

static struct i2c_board_info zetablet_pwri2c_devices[] = {
	{
		I2C_BOARD_INFO("max8660", 0x34),
		.platform_data = &max8660_pdata,
	},
};

/*
 * LDS6128 sensor keypad
 */
static struct lds61xx_channel lds61xx_channels[] = {
	{
		.id = 0,
		.type = LDS_CHANNEL_TOUCH,
		.enabled = 1,
		.init_threshold = 18,
	},
	{
		.id = 1,
		.type = LDS_CHANNEL_TOUCH,
		.enabled = 1,
		.init_threshold = 23,
	},
	{
		.id = 2,
		.type = LDS_CHANNEL_TOUCH,
		.enabled = 1,
		.init_threshold = 17,
	},
	{
		.id = 3,
		.type = LDS_CHANNEL_TOUCH,
		.enabled = 1,
		.init_threshold = 16,
	},
	{
		.id = 4,
		.type = LDS_CHANNEL_TOUCH,
		.enabled = 1,
		.init_threshold = 19,
	},
	{
		.id = 5,
		.type = LDS_CHANNEL_TOUCH,
		.enabled = 1,
		.init_threshold = 26,
	},
	{
		.id = 10,
		.type = LDS_CHANNEL_TOUCH,
		.enabled = 1,
		.init_threshold = 34,
	},
	{
		.id = 13,
		.type = LDS_CHANNEL_TOUCH,
		.enabled = 1,
		.init_threshold = 26,
	},
	{
		.id = 15,
		.type = LDS_CHANNEL_LED,
		.enabled = 1,
		.init_assignement = 10,
		.current_min = 0,
		.current_max = 18,
		.init_effect = LDS_LED_EFFECT_DIMMING,
		.init_period1 = 0x1,
		.init_period3 = 0x1,
	},
};

static struct lds61xx_pdata lds61xx_pdata = {
	.ic_type = LDS_IC_6128,
	.channel_count = ARRAY_SIZE(lds61xx_channels),
	.channels = lds61xx_channels,
	.gpio = ETABLET_GPIO_LDS_IRQ,
	/* Initial touch and calibration params */
	.init_debounce = 4,
	.init_undebounce = 3,
	.init_hysteresis = 3,
	.init_ambient_enable = 1, /* Enable automatic ambient calibration */
	.init_calib_auto_step = 1, /* Adaptive SELC step */
	.init_recalib_delay = 157, /* ~10 sec */
	.init_stuck_delay = 1000, /* 64 sec */
	.init_ambient_pos_limit = 17,
	.init_ambient_neg_limit = 17,
};


/*
 * I2C devices
 */
static struct i2c_pxa_platform_data zetablet_i2c_pdata = {
	.fast_mode = 1,
};

static struct i2c_board_info zetablet_i2c_devices[] = {
	{
		I2C_BOARD_INFO("bq27500", 0x55),
	},
	{
		I2C_BOARD_INFO("lds61xx", 0x2C),
		.platform_data = &lds61xx_pdata,
	},
};

/*
 * USB Charger stuff  (via gpio-charger)
 */
static char* zetablet_gpiochrg_batt = "bq27500-0";

static struct gpio_charger_platform_data zetablet_gpiochrg_pdata = {
	.name = "usb_charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.gpio = ETABLET_GPIO_CHRG_CHRG,
	.supplied_to = &zetablet_gpiochrg_batt,
	.num_supplicants = 1,
};

static struct platform_device zetablet_device_charger = {
	.name		= "gpio-charger",
	.id		= -1,
	.dev		= {
		.platform_data = &zetablet_gpiochrg_pdata,
	},
};

/*
 * RT5611 (WM9713) stuff
 */
static struct platform_device zetablet_device_audio = {
	.name		= "etablet-audio",
	.id		= -1,
};

/*
 * Syscore ops for suspend/resume hacking
 */
static int zetablet_sys_suspend(void)
{
	/* Enable wakeups here */
	printk(KERN_INFO "ETABLET suspending\n");
	enable_irq_wake(IRQ_MMC3);
	enable_irq_wake(IRQ_BTUART);
	return 0;
}


static void zetablet_sys_resume(void)
{
	/* Disable wakeups */
	printk(KERN_INFO "ETABLET resuming\n");
	disable_irq_wake(IRQ_MMC3);
	disable_irq_wake(IRQ_BTUART);
}

static struct syscore_ops zetablet_syscore_ops = {
	.suspend	= zetablet_sys_suspend,
	.resume		= zetablet_sys_resume,
};

/*
 * Power-off proc
 * We use S3D4C4 as the power-down mode
 */
static void zetablet_power_off(void)
{
	unsigned int pwrmode;
	pwrmode = PXA3xx_PM_S3D4C4;
	
	printk(KERN_INFO "ETABLET going to S3D4C4\n");
	asm volatile("mcr	p14, 0, %[state], c7, c0, 0"
			:: [state]"r" (pwrmode));
}

/*
 * Machine init proc
 */
static void __init zetablet_init(void)
{
	struct clk *clk_pout, *clk_ac97;
	/* Generic MFP configuration */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(generic_mfp_cfg));
	
	/* UARTs init */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(uart_mfp_cfg));
	pxa_set_ffuart_info(NULL);
	pxa_set_btuart_info(NULL);
	pxa_set_stuart_info(NULL);
	
	/* Charger */
	//platform_device_register(&zetablet_gpiochrg);
	
	/* I2C */
	pxa_set_i2c_info(&zetablet_i2c_pdata);
	pxa3xx_set_i2c_power_info(&zetablet_pwri2c_pdata);
	
	i2c_register_board_info(0, ARRAY_AND_SIZE(zetablet_i2c_devices));
	i2c_register_board_info(1, ARRAY_AND_SIZE(zetablet_pwri2c_devices));
	
	/* Camera */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(cif_mfp_cfg));
	
	/* AC97 */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(ac97_mfp_cfg));
	
	/* Enable CLK_POUT */
	
	//clk_pout = clk_get_sys(NULL, "CLK_POUT");
	//if(IS_ERR(clk_pout)){
	//	printk(KERN_ERR "CLK_POUT unavailable\n");
	//}
	//clk_enable(clk_pout);
	
	//clk_ac97 = clk_get_sys(NULL, "AC97CLK");
	//if(IS_ERR(clk_ac97)){
	//	printk(KERN_ERR "CLK_AC97 unavailable\n");
	//}
	//clk_enable(clk_ac97);
	
	pxa_set_ac97_info(NULL);
	platform_device_register(&zetablet_device_audio);
	
	/* Other periph init */
	platform_device_register(&zetablet_device_gpio_keys);
	zetablet_init_lcd();
	zetablet_init_mmc();
	//zetablet_init_ohci();
	zetablet_init_udc();
	
	/* Power management */
	register_syscore_ops(&zetablet_syscore_ops);
	pm_power_off = zetablet_power_off;
}
/*
 * Custom IO mapping
 * Override default pxa3xx_map_io, because it doesn't map DDR controller MMIO
 * We need that for suspend/standby modes
 * BUG?
 */
static struct map_desc zetablet_io_desc[] __initdata = {
	{
		.virtual = (unsigned long ) DMEMC_VIRT,
		.pfn = __phys_to_pfn(DMEMC_PHYS),
		.length = DMEMC_SIZE,
		.type = MT_DEVICE,
	},
	{
		.virtual = (unsigned long ) SMEMC_VIRT,
		.pfn = __phys_to_pfn(PXA3XX_SMEMC_PHYS),
		.length = SMEMC_SIZE,
		.type = MT_DEVICE,
	},
};

void __init zetablet_map_io(void)
{
	pxa_map_io();
	iotable_init(ARRAY_AND_SIZE(zetablet_io_desc));
	pxa3xx_get_clk_frequency_khz(1);
}

MACHINE_START(ETABLET, "ASUS EeeNote")
	.atag_offset		= 0x100,
	.map_io			= zetablet_map_io,
	.nr_irqs		= PXA_NR_IRQS,
	.init_irq		= pxa3xx_init_irq,
	.handle_irq		= pxa3xx_handle_irq,
	.timer			= &pxa_timer,
	.init_machine		= zetablet_init,
	.restart		= pxa_restart,
MACHINE_END
