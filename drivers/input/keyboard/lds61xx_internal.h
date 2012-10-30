/* Header with internal LDS61xx definitions */

#ifndef _INPUT_LDS61XX_
#define _INPUT_LDS61XX_

/* 
 * All LDS61xx ICs are cutted versions of LDS6120/LDS6100
 * So, all we need is to map IC's channels to those of LDS6120
 * And implement routines only for LDS6120
 */
#define LDS_MAX_CHANNELS 20

/* Paged touch parameters page numbers */
#define LDS_PAGE_INITSELC 0
#define LDS_PAGE_TCHTHRES 1
#define LDS_PAGE_AMBTHRES 2
#define LDS_PAGE_NOISENEG 3
#define LDS_PAGE_NOISEPOS 4

/* Manufacturer ID */
#define LDS_ID_VALID 0xF2

struct lds61xx_params
{
	/* Generic stuff */
	char name[8];
	int has_leds;
	/* External-to-internal (LDS6120 core) channel mappings */
	int internal_chan[LDS_MAX_CHANNELS];
};

/* 
 * Note: usually you should pass correct lds61xx_runtime
 * Otherwise, driver will use IC defaults
 */
struct lds61xx_runtime
{
	/* Touch config */
	int touch_mode;
	int touch_relative_st;	/* Relative strongest touch enable */
	int touch_hyst;		/* Hysteresis (in cap. units) */
	int touch_debounce;	/* Consecutive scans to recognize touch */
	int touch_undebounce;	/* Consecutive scans to remove touch */
	int touch_strongest_hyst; /* Strongest touch hysteresis */
	int touch_strongest_time; /* Replacement time (number of scans) */
	
	/* Calibration related */
	int calib_timeout;	/* Calibration timeout, 8 bit */
	int calib_dec_limit;	/* How quickly ambient recalibration triggers */
	int calib_inc_limit;	/* 8 bit */
	int calib_recal_delay;	/* 11 bit. Time before ambient recalib. */
	int calib_stuck_limit; /* 11 bit. Time before forced recalib. */
	
	/* LED config */
	int led_manual;		/* When set, LEDs are controlled by software */
	int led_gang;		/* All LEDs are lit together upon touch */
	int led_latency;	/* LED lightup delay (= latency * 5ms) */
	int led_dim_on;		/* Fade out LED while turning it on */
	int led_dim_off;	/* Fade in LED while turning it off */
	int led_period2;	/* Timer2 period = (led_period2 * 5ms) */
};

/*
 * IC definitions
 * Indexes correspond to LDS_IC_*
 */
static struct lds61xx_params lds61xx_ic[] = {
	[0] = {
		/* 20 channels, 10 DCM, 0 LEDs */
		.name = "LDS6100",
		.has_leds = 0,
		.internal_chan = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,\
				10, 11, 12, 13, 14, 15, 16, 17, 18, 19},
	},
	[1] = {
		/* 8 channels, 4 DCM, 0 LEDs */
		.name = "LDS6104",
		.has_leds = 0,
		.internal_chan = {1, 2, 3, 8, 11, 12, 13, 18, -1, -1,\
				-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	},
	[2] = {
		/* 13 channels, 5 DCM, 0 LEDs */
		.name = "LDS6107",
		.has_leds = 0,
		.internal_chan = {1, 2, 3, 4, 8, 10, 11, 12, 13, 14,\
				15, 18, 19, -1, -1, -1, -1, -1, -1, -1},
	},
	[3] = {
		/* 16 channels, 8 DCM, 0 LEDs */
		.name = "LDS6108",
		.has_leds = 0,
		.internal_chan = {0, 1, 2, 3, 4, 5, 8, 9, 10, 11,\
				12, 13, 14, 15, 18, 19, -1, -1, -1, -1},
	},
	[4] = {
		/* 20 channels, 10 DCM, 10 LEDs */
		.name = "LDS6120",
		.has_leds = 1,
		.internal_chan = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,\
				10, 11, 12, 13, 14, 15, 16, 17, 18, 19},
	},
	[5] = {
		/* 8 channels, 4 DCM, 4 LEDs */
		.name = "LDS6124",
		.has_leds = 1,
		.internal_chan = {1, 2, 3, 8, 11, 12, 13, 18, -1, -1,\
				-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	},
	[6] = {
		/* 11 channels, 5 DCM, 6 LEDs */
		.name = "LDS6126",
		.has_leds = 1,
		.internal_chan = {1, 2, 3, 4, 8, 11, 12, 13, 14, 15,\
				18, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	},
	[7] = {
		/* 16 channels, 8 DCM, 8 LEDs */
		.name = "LDS6128",
		.has_leds = 1,
		.internal_chan = {0, 1, 2, 3, 4, 5, 8, 9, 10, 11,\
				12, 13, 14, 15, 18, 19, -1, -1, -1, -1},
	},
};

/* Some bits */
/* Interrupt config */
#define LDS_INT_READRESET 3
#define LDS_INT_INTERNAL  1
#define LDS_INT_FIXED	  0
#define LDS_INT_ACTIVE_HIGH  (1<<15)
#define LDS_INT_OPENDRAIN (1<<3)


#define LDS_BIT_TOUCHEN	  (1<<15)
#define LDS_BIT_RELSTEN	  (1<<15)
#define LDS_BIT_LED_MAN	  (1<<15)
#define LDS_BIT_LED_GANG  (1<<14)
#define LDS_BIT_LED_DOFF  (1<<15)
#define LDS_BIT_LED_DON	  (1<<14)
#define LDS_BIT_READY	  (1<<3)
#define LDS_BIT_AMBDIS	  (1<<13)

#define LDS_SELC_DEFAULT  0x2

/* Registers */
/* Registers are 16-bit wide, address is also 16 bit */

/* Reset */
#define LDS_COLDRESET	0x000
#define LDS_SOFTRESET	0x001

/* Power */
#define LDS_POWERMODE	0x002
#define LDS_SLEEPWAIT	0x003
#define LDS_IDLETIME	0x055
#define LDS_SLEEPCONF	0x056

/* Interrupt */
#define LDS_INTCONF	0x008
#define LDS_INTEN	0x043	/* Base */

/* GPIO */
#define LDS_GPIO	0x009

/* DCM */
#define LDS_DCM		0x00A

/* Identifier */
#define LDS_ID		0x01F

/* Touch config */
#define LDS_TOUCHMODE	0x040
#define LDS_TOUCHEN	0x041	/* Base */

/* Touch parameters */
#define LDS_MEMPAGE	0x05F
#define LDS_PARAMBASE	0x060	/* Base */
#define LDS_PARAM(x)	(LDS_PARAMBASE + (x))

/* Touch status */
#define LDS_TOUCHSTATUS	0x045
#define LDS_CAPVAL_BASE	0x080	/* Base */
#define LDS_CAPVAL(x)	(LDS_CAPVAL_BASE + ((x)<<2))

/* Calibration */
#define LDS_CALIB_CONFIG  0x04E
#define LDS_CALIB_TIMEOUT 0x050
#define LDS_CALIB_AMBIENT 0x051
#define LDS_CALIB_RECAL	  0x052
#define LDS_CALIB_STUCK	  0x053

/* SELC */
#define LDS_SELC_BASE	0x081
#define LDS_SELC(x)	(LDS_SELC_BASE + ((x)<<2))

/* LEDs */
#define LDS_LED_MANUAL	0x03E
#define LDS_LED_EN	0x03F
#define LDS_LED_LATENCY	0x02E
#define LDS_LED_WAVEFORM 0x02F

#define LDS_LED_CONFBASE 0x020
#define LDS_LED_EFFBASE	0x030

#define LDS_LED_CONF(x)	(LDS_LED_CONFBASE + (x) - 10)
#define LDS_LED_EFFECT(x) (LDS_LED_EFFBASE + (x) - 10)

/* Slider */
#define LDS_SLIDER_POS	0x04B
#define LDS_SLIDER_EN	0x074

/* Mixed regs */
#define LDS_DEB_STR	0x057
#define LDS_HYSTERESIS	0x075
#define LDS_UNDEB_STR	0x076

/* Guard channel */
#define LDS_GUARD_EN	0x07C	/* Base */
#define LDS_GUARD_MASK	0x07E	/* Base */

/* Noise immunity */
#define LDS_NI		0x077

#endif
