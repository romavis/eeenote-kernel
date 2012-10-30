/*
 * etablet.c  --  SoC audio for EeeNote EA-800
 *
 * Based on e750_wm9705.c
 * 
 * Copyright 2012 (c) Dobrodiy Roman <ztcoils@gmail.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation; version 2 ONLY.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <mach/audio.h>
#include <mach/etablet.h>

#include <asm/mach-types.h>

#include "../codecs/rt5611.h"
#include "pxa2xx-ac97.h"

static int etablet_spk_amp_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	if (event & SND_SOC_DAPM_PRE_PMU)
		gpio_set_value(ETABLET_GPIO_SPKPWR, 0);
	else if (event & SND_SOC_DAPM_POST_PMD)
		gpio_set_value(ETABLET_GPIO_SPKPWR, 1);
	return 0;
}

static const struct snd_soc_dapm_widget etablet_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
	SND_SOC_DAPM_PGA_E("Speaker Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
			etablet_spk_amp_event, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route etablet_audio_map[] = {
	{"Headphone Jack", NULL, "HPL"},
	{"Headphone Jack", NULL, "HPR"},
	
	{"Speaker Amp", NULL, "SPKR"},
	{"Speaker Amp", NULL, "SPKRN"},
	{"Speaker", NULL, "Speaker Amp"},

	{"MIC2", NULL, "Mic 1 Bias"},
	{"MIC2N", NULL, "Mic 1 Bias"},
	{"Mic 1 Bias", NULL, "Mic"},
};

static int etablet_ac97_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_nc_pin(dapm, "MONO");
	snd_soc_dapm_nc_pin(dapm, "SPKL");
	snd_soc_dapm_nc_pin(dapm, "SPKLN");
	
	snd_soc_dapm_nc_pin(dapm, "LINEL");
	snd_soc_dapm_nc_pin(dapm, "LINER");
	snd_soc_dapm_nc_pin(dapm, "PHONEIN");
	snd_soc_dapm_nc_pin(dapm, "PHONEINN");
	snd_soc_dapm_nc_pin(dapm, "MIC1");
	snd_soc_dapm_nc_pin(dapm, "MIC1N");
	
	snd_soc_dapm_enable_pin(dapm, "Mic");
	snd_soc_dapm_enable_pin(dapm, "Speaker");
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	
	snd_soc_dapm_sync(dapm);
	return 0;
}

static struct snd_soc_dai_link etablet_dai[] = {
	{
		.name = "AC97",
		.stream_name = "AC97 HiFi",
		.cpu_dai_name = "pxa2xx-ac97",
		.codec_dai_name = "rt5611-hifi",
		.platform_name = "pxa-pcm-audio",
		.codec_name = "rt5611-codec",
		.init = etablet_ac97_init,
	},
};

static struct snd_soc_card etablet_card = {
	.name = "EeeNote",
	.owner = THIS_MODULE,
	.dai_link = etablet_dai,
	.num_links = ARRAY_SIZE(etablet_dai),
	
	.dapm_widgets = etablet_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(etablet_dapm_widgets),
	.dapm_routes = etablet_audio_map,
	.num_dapm_routes = ARRAY_SIZE(etablet_audio_map),
};

static struct gpio etablet_audio_gpios[] = {
	{ ETABLET_GPIO_SPKPWR, GPIOF_OUT_INIT_HIGH, "Speaker amp" },
	{ ETABLET_GPIO_HP_DETECT, GPIOF_IN, "Jack detection" },
};

static struct platform_device *etablet_snd_device;

static int __devinit etablet_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &etablet_card;
	int ret;
	
	printk(KERN_INFO "ETABLET audio probing\n");
	ret = gpio_request_array(etablet_audio_gpios,
				 ARRAY_SIZE(etablet_audio_gpios));
	if (ret) {
		dev_err(&pdev->dev, "unable to request GPIOs");
		return ret;
	}
	
	etablet_snd_device = platform_device_alloc("rt5611-codec", -1);
	if (!etablet_snd_device) {
		ret = -ENOMEM;
		goto out_gpio;
	}
	
	ret = platform_device_add(etablet_snd_device);
	if (ret) {
		dev_err(&pdev->dev, "platform_device_add() failed");
		goto out_pdevice;
	}
	
	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed");
		goto out_pdevice;
	}
	return 0;
out_pdevice:
	platform_device_put(etablet_snd_device);
out_gpio:
	gpio_free_array(etablet_audio_gpios, ARRAY_SIZE(etablet_audio_gpios));
	return ret;
}

static int __devexit etablet_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	gpio_free_array(etablet_audio_gpios, ARRAY_SIZE(etablet_audio_gpios));
	snd_soc_unregister_card(card);
	platform_device_put(etablet_snd_device);
	return 0;
}

static struct platform_driver etablet_driver = {
	.driver		= {
		.name	= "etablet-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= etablet_probe,
	.remove		= __devexit_p(etablet_remove),
};

module_platform_driver(etablet_driver);

/* Module information */
MODULE_AUTHOR("Dobrodiy Roman <ztcoils@gmail.com>");
MODULE_DESCRIPTION("ALSA SoC driver for EeeNote");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:etablet-audio");
