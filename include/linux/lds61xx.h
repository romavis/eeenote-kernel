/* Header with LDS61xx definitions used for platform_data */
#ifndef _LINUX_LDS61XX_
#define _LINUX_LDS61XX_

/* LDS61xx family ICs */
/* w/o LEDs */
#define LDS_IC_6100	0
#define LDS_IC_6104	1
#define LDS_IC_6107	2
#define LDS_IC_6108	3
/* with LEDs */
#define LDS_IC_6120	4
#define LDS_IC_6124	5
#define LDS_IC_6126	6
#define LDS_IC_6128	7

/* Platform data */
#define LDS_SUSPEND_FULLPOWER	0
#define LDS_SUSPEND_LOWPOWER	1
#define LDS_SUSPEND_SHUTDOWN	2

#define LDS_TOUCH_NORMAL	0
#define LDS_TOUCH_STRONGEST	1
#define LDS_TOUCH_TWOSTRONGEST	2

#define LDS_CHANNEL_UNUSED	0
#define LDS_CHANNEL_TOUCH	1
#define LDS_CHANNEL_DCM		2
#define LDS_CHANNEL_LED		3

#define LDS_LED_EFFECT_DIMMING	0
#define LDS_LED_EFFECT_PULSATE	1
#define LDS_LED_EFFECT_FLASH	2

/*
 * Channel desctiption in platform data
 * Mandatory parameters: id, type
 * 
 * Note:
 * current_max and current_min are required for LEDs to work at all
 * keycode is required for touch channels
 * 
 * initial parameters are unnecessary but you are recommended to set them
 * in order for touch/LEDs to work at boot time
 */
struct lds61xx_channel
{
	int id;		/* Channel index */
	int type;	/* Channel type */
	int enabled;	/* Enable/disable channel (but configure it) */
	
	/* Touch channel */
	unsigned int keycode; /* Keycode assigned to channel */
	int init_threshold; /* Startup threshold (in cap. units). 11 bit*/

	/* LED channel */
	int current_max; /* Max current (in units of 0.25 mA) */
	int current_min; /* Min current (in units of 0.25 mA) */
	int init_assignement; /* No. of channel assigned to LED */
	
	int init_effect; /* LED effect type */
	/* Time = value * 5ms * (current sweep) */
	int init_period1;
	int init_period3;
};

/*
 * LDS61xx platform data
 * Mandatory: ic_type, gpio, channels, channel_count
 * Recommended: debounce
 * 
 * Note:
 * Debounce/Undebounce (4 bit) set number of conecutive scans reqired to
 * 	recognize touch/untouch. Each scan time = 2ms*Active_Sensors_Count
 * Hysteresis (8 bit) set negative hysteresis region for touches.
 * 	Hysteresis is set in the same units as threshold.
 * Ambient disable - disables automatical ambient calibration.
 * 
 * Recalibration delay (11 bit) - time before small changes in capacitance
 * 	trigger recalibration. Small value leads to calibrating out
 * 	slowly approaching fingers.
 * 
 * Stuck touch delay (11 bit) - time before touch is counted as "stuck"
 * 	and calibrated out. You should prefer larger value here ;)
 * 
 * Stuck delay and recalibration delays are calculated as:
 * 	Value = Time / (4*Active_Sensors_Count*2ms) - 1.
 * 
 * Positive/Negative ambient calibration trigger limits set capacitance region
 * 	around baseline inside which ambient calibration is not triggered.
 * 	Outside that region ambient calibration will be triggered after
 * 	recalibration_delay time passes.
 * 	Note: it can't be zero!
 * 
 * Auto calibration step (SELC) enables adaptive SELC step,
 * 	otherwise it is set to 2 (default).
 * 
 * If ambient calibration is enabled, please set all calibration parameters
 * 	for the correct operation!
 */
struct lds61xx_pdata
{
	int ic_type;	/* Type of an IC */
	
	int suspend_state; /* What to do on suspend */
	int lowpower_sleep; /* Time to sleep between scans (ms) */
	/* IRQ GPIO */
	int gpio;
	int gpio_inverted;
	/* Touch */	
	int init_debounce;
	int init_undebounce;
	int init_hysteresis;
	
	int init_ambient_enable;
	int init_recalib_delay;
	int init_stuck_delay;
	int init_calib_auto_step;
	int init_ambient_pos_limit;
	int init_ambient_neg_limit;
	
	/* Channels */
	int channel_count;
	struct lds61xx_channel* channels;
};

#endif
