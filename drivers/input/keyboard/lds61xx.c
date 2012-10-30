/*
 * lds61xx.c - LDS61xx sensor keypad driver
 *
 * Copyright (C) 2012 Roman Dobrodiy
 *
 * Based on max7359_keypad.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Datasheet is available upon request :(
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/input.h>
#include <linux/gpio.h>

#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/lds61xx.h>
#include "lds61xx_internal.h"

#define LDS_RDY_TIMEOUT 15000

struct lds61xx {
	struct lds61xx_pdata *pdata;
	struct lds61xx_params *ic_params;

	struct i2c_client *client;
	struct device *dev;
	
	int irq;
	
	struct lds61xx_channel channels[LDS_MAX_CHANNELS];
};

/*
 * LDS61xx register access over I2C
 */
static int lds61xx_write_reg(struct i2c_client *client, int reg, int data)
{
	char buffer[4];
	int ret;
	/* Prepare packet for device */
	buffer[0] = (reg >> 8) & 0xFF;
	buffer[1] = reg & 0xFF;
	buffer[2] = (data >> 8) & 0xFF;
	buffer[3] = data & 0xFF;
	ret = i2c_master_send(client, buffer, 4);
	if (ret != 4)
		goto err;
	return 0;
	
err:
	dev_err(&client->dev, "reg write error: addr 0x%x, val 0x%x, err %d\n",\
		reg, data, ret);
	return ret;
}

static int lds61xx_read_reg(struct i2c_client *client, int reg, int *data)
{
	char buffer[2];
	int ret;
	
	/* Write reg address */
	buffer[0] = (reg >> 8) & 0xFF;
	buffer[1] = reg & 0xFF;
	ret = i2c_master_send(client, buffer, 2);
	if(ret != 2)
		goto err;

	ret = i2c_master_recv(client, buffer, 2);
	if(ret != 2)
		goto err;
	
	*data = (buffer[0] << 8) | buffer[1];
	return 0;
	
err:
	dev_err(&client->dev, "reg read error: addr 0x%x, err %d\n", reg, ret);
	return -EIO;
}

/*
 * Sysfs lots of crap (attributes) :)
 * 
 * Note: channels are listed in such an order as they are in platform_data!
 * 
 * chan_id [ro] - lists channel numbers in use (for mapping with other attrs.)
 * chan_type [ro] - lists channel types ("touch", "led", "dcm")
 * chan_enabled [rw] - enables/disables individual channels
 * chan_keycode [ro] - lists keycodes mapped to channels
 * 	non-touch are listed as "none"
 * chan_threshold [rw] - touch channels thresholds
 * 	non-touch are listed as "none", writing to them ignored
 */
/*
static DEVICE_ATTR(chan_id, S_IRUGO,\
			lds61xx_sysfs_chan_id_show, NULL);
static DEVICE_ATTR(chan_type, S_IRUGO,\
			lds61xx_sysfs_chan_type_show, NULL);
static DEVICE_ATTR(chan_enabled, S_IWUSR | S_IRUGO,\
			lds61xx_sysfs_chan_enabled_show,
			lds61xx_sysfs_chan_enabled_store);
static DEVICE_ATTR(chan_keycode, S_IRUGO,\
			lds61xx_sysfs_chan_keycode_show, NULL);
static DEVICE_ATTR(chan_threshold, S_IWUSR | S_IRUGO,\
			lds61xx_sysfs_chan_threshold_show,
			lds61xx_sysfs_chan_threshold_store);
*/

/*
 * Routine writes per-channel enable/disable according to lds->channels
 * It doesn't touch any channel-related configuration
 */
static int lds61xx_enable_disable(struct lds61xx *lds)
{
	int ret;
	int i;
	int en_sen, en_led, en_dcm;
	en_sen = en_led = en_dcm = 0;
	
	for(i=0; i<LDS_MAX_CHANNELS; ++i)
		if(lds->channels[i].type == LDS_CHANNEL_TOUCH &&\
			lds->channels[i].enabled)
			en_sen |= (1<<i);
		else if(lds->channels[i].type == LDS_CHANNEL_DCM &&\
			lds->channels[i].enabled)
			en_dcm |= (1<<i);
		else if(lds->channels[i].type == LDS_CHANNEL_LED &&\
			lds->channels[i].enabled)
			en_led |= (1<<i);
	
	ret = lds61xx_write_reg(lds->client, LDS_TOUCHEN, en_sen & 0x3FF);
	if(ret)
		return ret;
	ret = lds61xx_write_reg(lds->client, LDS_TOUCHEN + 1, en_sen >> 10);
	if(ret)
		return ret;
	ret = lds61xx_write_reg(lds->client, LDS_INTEN, en_sen & 0x3FF);
	if(ret)
		return ret;
	ret = lds61xx_write_reg(lds->client, LDS_INTEN + 1, en_sen >> 10);
	if(ret)
		return ret;
	
	if(lds->ic_params->has_leds) {
		en_led >>= 10;
		ret = lds61xx_write_reg(lds->client, LDS_LED_EN, en_led & 0x3FF);
		if(ret)
			return ret;
	}
	
	ret = lds61xx_write_reg(lds->client, LDS_DCM, en_dcm & 0x3FF);
	if(ret)
		return ret;
	
	return 0;
}

/*
 * Enable/disable touch subsystem
 */
static int lds61xx_touch_control(struct lds61xx *lds, int enable)
{
	int ret;
	int data;
	
	ret = lds61xx_read_reg(lds->client, LDS_TOUCHMODE, &data);
	if(ret)
		return ret;
	
	if(enable)
		data |= LDS_BIT_TOUCHEN;
	else
		data &= ~LDS_BIT_TOUCHEN;
	
	ret = lds61xx_write_reg(lds->client, LDS_TOUCHMODE, data);
	
	return ret;
}

/*
 * Issue "Cold reset" and then perform initialization
 * During cold reset, IC losses all configuration
 */
static int lds61xx_reset(struct lds61xx *lds)
{
	int ret;
	int i;
	int data;
	
	/* Cold reset */
	ret = lds61xx_write_reg(lds->client, LDS_COLDRESET, 0x0);
	if(ret)
		return ret;
	
	/* Disable touch subsystem */
	ret = lds61xx_touch_control(lds, 0);
	if(ret)
		return ret;
	
	/* Write interrupt config */
	ret = lds61xx_write_reg(lds->client, LDS_INTCONF, LDS_INT_READRESET |\
		(lds->pdata->gpio_inverted ? 0x0 : LDS_INT_ACTIVE_HIGH));
	if(ret)
		return ret;
	
	/* Write initial touch thresholds */
	ret = lds61xx_write_reg(lds->client, LDS_MEMPAGE, LDS_PAGE_TCHTHRES);
	if(ret)
		return ret;
	for(i=0; i<LDS_MAX_CHANNELS; ++i)
		if(lds->channels[i].type == LDS_CHANNEL_TOUCH){
			ret = lds61xx_write_reg(lds->client, LDS_PARAM(i),\
				lds->channels[i].init_threshold & 0x7FF);
			if(ret) return ret;
		}	
	/* Enable/disable ambient calibration, SELC_STEP = default */
	ret = lds61xx_write_reg(lds->client, LDS_CALIB_CONFIG,\
		(lds->pdata->init_ambient_enable ? 0x0 : LDS_BIT_AMBDIS) |\
		(lds->pdata->init_calib_auto_step ? 0x0 : LDS_SELC_DEFAULT));
	if(ret)
		return ret;
	/* Touch stability parameters */
	ret = lds61xx_write_reg(lds->client, LDS_DEB_STR,\
		((lds->pdata->init_debounce & 0xF) << 12));
	if(ret)
		return ret;
	ret = lds61xx_write_reg(lds->client, LDS_UNDEB_STR,\
		(lds->pdata->init_undebounce & 0xF));
	if(ret)
		return ret;
	ret = lds61xx_write_reg(lds->client, LDS_HYSTERESIS,\
		(lds->pdata->init_hysteresis & 0xFF));
	if(ret)
		return ret;
	/* Recalibration parameters */
	ret = lds61xx_write_reg(lds->client, LDS_CALIB_RECAL,\
		(lds->pdata->init_recalib_delay & 0x7FF));
	if(ret)
		return ret;
	ret = lds61xx_write_reg(lds->client, LDS_CALIB_STUCK,\
		(lds->pdata->init_stuck_delay & 0x7FF));
	if(ret)
		return ret;
	/* Write CNT_DEC_LIMIT, CNT_INC_LIMIT */
	ret = lds61xx_write_reg(lds->client, LDS_CALIB_AMBIENT,\
		(lds->pdata->init_ambient_pos_limit & 0xFF) |\
		((lds->pdata->init_ambient_neg_limit & 0xFF)) << 8);
	if(ret)
		return ret;
	
	
	/* Write static and initial LED parameters if applicable */
	if(lds->ic_params->has_leds)
	for(i=10; i<LDS_MAX_CHANNELS; ++i)
		if(lds->channels[i].type == LDS_CHANNEL_LED) {
			int assigned_ch = lds->ic_params->internal_chan[\
					lds->channels[i].init_assignement];
			if(assigned_ch < 0) {
				dev_err(lds->dev,\
					"led %d incorrect assignement\n",\
					lds->channels[i].id);
				continue;
			}
			data = (assigned_ch & 0x1F) |\
				((lds->channels[i].current_min & 0x1F) << 6) |\
				((lds->channels[i].current_max & 0x1F) << 11);
			ret = lds61xx_write_reg(lds->client, LDS_LED_CONF(i),\
						data);
			if(ret)
				return ret;
			
			data = ((lds->channels[i].init_effect & 0x3) << 14) |\
				((lds->channels[i].init_period1 & 0x3F) << 6) |\
				(lds->channels[i].init_period3 & 0x3F);
			ret = lds61xx_write_reg(lds->client, LDS_LED_EFFECT(i),\
						data);
			if(ret)
				return ret;
		}

	/* Write touch/LED/DCM enable bits */
	ret = lds61xx_enable_disable(lds);
	if(ret)
		return ret;

	/* Enable touch subsystem */
	ret = lds61xx_touch_control(lds, 1);
	if(ret)
		return ret;
	
	/* Soft reset - recalibrate */
	ret = lds61xx_write_reg(lds->client, LDS_SOFTRESET, 0);
	
	return ret;
}

static irqreturn_t lds61xx_isr(int irq, void *devid)
{
	struct lds61xx *lds = devid;
	int ret;
	int i, mask;
	int data, tmp;
	
	lds61xx_read_reg(lds->client, LDS_TOUCHSTATUS, &tmp);
	data = tmp & 0x3FF;
	lds61xx_read_reg(lds->client, LDS_TOUCHSTATUS + 1, &tmp);
	data |= (tmp & 0x3FF) << 10;
	printk(KERN_INFO "lds61xx: touched:: ");
	
	for(i=0, mask=1; i<LDS_MAX_CHANNELS; ++i) {
		if(data & mask)
			printk("%d ", lds->channels[i].id);
		mask <<= 1;
	}
	printk("\n");
	return IRQ_HANDLED;
}

static int __devinit lds61xx_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct lds61xx_pdata *pdata = dev->platform_data;
	struct lds61xx *lds;
	
	int ret;
	int data;
	int i;
	int chan;
	
	/* Probing for LDS61xx */
	if(!pdata) {
		dev_err(dev, "no platform_data supplied\n");
		return -EINVAL;
	}
	
	/* Wait for READY bit */
	for(i=0; i<LDS_RDY_TIMEOUT && !(data & LDS_BIT_READY); ++i) {
		ret = lds61xx_read_reg(client, LDS_TOUCHMODE, &data);
		if(ret)
			return ret;
	}
	if(!(data & LDS_BIT_READY)) {
		dev_err(dev, "READY bit timeout\n");
		return -ENXIO;
	}
	ret = lds61xx_read_reg(client, LDS_ID, &data);
	if(ret)
		return ret;
	printk(KERN_INFO "lds61xx %s: read MID 0x%X\n", dev_name(dev), data);
	if(data!=LDS_ID_VALID) {
		dev_warn(dev, "invalid chip ID read, forcing\n");
		/* return -EINVAL; */
	}
	printk(KERN_INFO "lds61xx %s: found LDS61xx IC\n", dev_name(dev));
	
	lds = (struct lds61xx*) kzalloc(sizeof(struct lds61xx), GFP_KERNEL);
	if(!lds) {
		dev_err(dev, "can't allocate memory\n");
		return -ENOMEM;
	}
	
	lds->client = client;
	lds->pdata = pdata;
	lds->dev = dev;
	i2c_set_clientdata(client, lds);
	
	/* Request interrupt */
	if(gpio_is_valid(pdata->gpio)) {
		ret = gpio_request(pdata->gpio, "lds61xx irq");
		if(ret) {
			dev_err(dev, "unable to request gpio\n");
			goto out_alloc;
		}
		ret = gpio_direction_input(pdata->gpio);
		if(ret) {
			dev_err(dev, "unable to set gpio direction\n");
			goto out_gpio;
		}
		lds->irq = gpio_to_irq(pdata->gpio);
		ret = request_threaded_irq(lds->irq, NULL,\
			lds61xx_isr,\
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "lds61xx irq", lds);
		if(ret) {
			dev_err(dev, "unable to request irq\n");
			goto out_gpio;
		}
	} else {
		dev_err(dev, "invalid GPIO\n");
		ret = -EINVAL;
		goto out_alloc;
	}
	
	/* Fill in lds61xx struct */
	lds->ic_params = lds61xx_ic + pdata->ic_type;
	for(i=0; i<pdata->channel_count; ++i) {
		chan = lds->ic_params->internal_chan[pdata->channels[i].id];
		if(chan<0) {
			dev_err(dev, "channel id %d incorrect\n",\
			pdata->channels[i].id);
			ret = -EINVAL;
			goto out_irq;
		}
		if(pdata->channels[i].type == LDS_CHANNEL_LED &&\
			!lds->ic_params->has_leds) {
			dev_err(dev, "IC doesn't support LEDs (channel %d)\n",\
				pdata->channels[i].id);
			ret = -EINVAL;
			goto out_irq;
		}
		
		lds->channels[chan] = pdata->channels[i];
	}
	
	if((ret = lds61xx_reset(lds))) {
		dev_err(dev, "Unable to reset IC\n");
		goto out_irq;
	}
	
	dev_info(lds->dev, "probed ok\n");
	return 0;
	
out_irq:
	free_irq(lds->irq, lds);
out_gpio:
	gpio_free(pdata->gpio);
out_alloc:
	kfree(lds);
	return ret;
}

static int __devexit lds61xx_remove(struct i2c_client *client)
{
	struct lds61xx *lds = i2c_get_clientdata(client);

	return 0;
}

#ifdef CONFIG_PM
static int lds61xx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	return 0;
}

static int lds61xx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(lds61xx_pm, lds61xx_suspend, lds61xx_resume);

static const struct i2c_device_id lds61xx_ids[] = {
	{ "lds61xx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lds61xx_ids);

static struct i2c_driver lds61xx_i2c_driver = {
	.driver = {
		.name = "lds61xx",
		.pm   = &lds61xx_pm,
	},
	.probe		= lds61xx_probe,
	.remove		= __devexit_p(lds61xx_remove),
	.id_table	= lds61xx_ids,
};

static int __init lds61xx_init(void)
{
	return i2c_add_driver(&lds61xx_i2c_driver);
}
module_init(lds61xx_init);

static void __exit lds61xx_exit(void)
{
	i2c_del_driver(&lds61xx_i2c_driver);
}
module_exit(lds61xx_exit);

MODULE_AUTHOR("Roman Dobrodiy <ztcoils@gmail.com>");
MODULE_DESCRIPTION("LDS61xx touch keypad controller");
MODULE_LICENSE("GPL v2");
