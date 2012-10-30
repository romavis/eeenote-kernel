/*
 * rt5610.c  --  RT5610 ALSA Soc Audio driver
 *
 * Copyright 2008 Realtek Microelectronics
 *
 * Author: flove <flove@realtek.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
 
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "rt5611.h"

#define RT5611_VERSION "0.01"
unsigned int gpio_val_bak=-1;
unsigned int audio_jack_state=-1;

struct rt5611_priv {
	u32 pll_in; /* PLL input frequency */
	u32 pll_out; /* PLL output frequency */
};

static unsigned int ac97_read(struct snd_soc_codec *codec,
	unsigned int reg);
static int ac97_write(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int val);
static int ac97_write_mask(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int val, unsigned int mask);
/*
 * RT5611 register cache
 * Reg 0x3c bit 15 is used by touch driver.
 */
static const u16 rt5611_reg[] = {
	0x59b4, 0x8080, 0x8080, 0x0000, // 6
	0xc880, 0xe808, 0xeC0C, 0x0808, // e
	0xe0e0, 0xf58b, 0x7f7f, 0x0000, // 16
	0xe800, 0x0000, 0x0000, 0x0000, // 1e
	0x0000, 0x0000, 0x0000, 0xef00, // 26
	0x0000, 0x0000, 0xbb80, 0x0000, // 2e
	0x0000, 0xbb80, 0x0000, 0x0000, // 36
	0x0000, 0x0000, 0x0000, 0x0000, // 3e
	0x0428, 0x0000, 0x0000, 0x0000, // 46
	0x0000, 0x0000, 0x2e3e, 0x2e3e, // 4e
	0x0000, 0x0000, 0x003a, 0x0000, // 56
	0x0cff, 0x0000, 0x0000, 0x0000, // 5e
	0x0000, 0x0000, 0x2130, 0x0010, // 66
	0x0053, 0x0000, 0x0000, 0x0000, // 6e
	0x0000, 0x0000, 0x008c, 0x3f00, // 76
	0x0000, 0x0000, 0x10ec, 0x1003, // 7e
	0x0000, 0x0000, 0x0000 // virtual hp & mic mixers
};

/* virtual HP mixers regs */
#define HPL_MIXER	0x80
#define HPR_MIXER	0x82
#define MICB_MUX	0x82

static const char *rt5611_spkl_pga[] = {"Off","HPL mixer","SPK mixer","Mono Mixer"};
static const char *rt5611_spkr_pga[] = {"Off","HPR mixer","SPK mixer","Mono Mixer"};
static const char *rt5611_hpl_pga[]  = {"Off","HPL mixer"};
static const char *rt5611_hpr_pga[]  = {"Off","HPR mixer"};
static const char *rt5611_mono_pga[] = {"Off","HP mixer","SPK mixer","Mono Mixer"};
static const char *rt5611_mic_boost_select[] = {"+0db","+20db","+30db","+40db"};
static const char *rt5611_amp_type_select[] = {"Class AB","Class D"};

static const struct soc_enum rt5611_enum[] = {
SOC_ENUM_SINGLE(RT5611_OUTPUT_MIXER_CTRL, 14, 4, rt5611_spkl_pga),
SOC_ENUM_SINGLE(RT5611_OUTPUT_MIXER_CTRL, 11, 4, rt5611_spkr_pga),
SOC_ENUM_SINGLE(RT5611_OUTPUT_MIXER_CTRL, 9, 2, rt5611_hpl_pga),
SOC_ENUM_SINGLE(RT5611_OUTPUT_MIXER_CTRL, 8, 2, rt5611_hpr_pga),
SOC_ENUM_SINGLE(RT5611_OUTPUT_MIXER_CTRL, 6, 4, rt5611_mono_pga), 
SOC_ENUM_SINGLE(RT5611_MIC_CTRL, 10,4, rt5611_mic_boost_select),
SOC_ENUM_SINGLE(RT5611_MIC_CTRL, 8,4, rt5611_mic_boost_select),
SOC_ENUM_SINGLE(RT5611_OUTPUT_MIXER_CTRL, 13, 2, rt5611_amp_type_select),
};

/* Logarithmic volume scales */
/* Output atten scale: -46.5dB to 0 step +1.5dB (invert) */
static const DECLARE_TLV_DB_SCALE(rt5611_outp_tlv, -4650, 150, 0);
/* Input preamp + DAC scale: -34.5dB to +12dB step +1.5dB (invert) */
static const DECLARE_TLV_DB_SCALE(rt5611_inp_tlv, -3450, 150, 0);
/* ADC Gain scale: -16.5dB to +30dB step +1.5dB */
static const DECLARE_TLV_DB_SCALE(rt5611_adc_tlv, -1650, 150, 0);

/* Linear scales for output amp gain */
static const DECLARE_TLV_DB_LINEAR(rt5611_hp_tlv, 0, 352);
static const DECLARE_TLV_DB_LINEAR(rt5611_cls_d_tlv, 0, 486);
static const DECLARE_TLV_DB_LINEAR(rt5611_cls_ab_tlv, 0, 700);


static const struct snd_kcontrol_new rt5611_snd_ac97_controls[] = {
/* Output path volumes */
SOC_DOUBLE_TLV("Speaker Playback Volume", RT5611_SPK_OUT_VOL, 8, 0, 31, 1, rt5611_outp_tlv),
SOC_DOUBLE_TLV("Headphone Playback Volume", RT5611_HP_OUT_VOL, 8, 0, 31, 1, rt5611_outp_tlv),
SOC_SINGLE_TLV("Mono Playback Volume", 	RT5611_PHONEIN_MONO_OUT_VOL, 0, 31, 1, rt5611_outp_tlv),
/* Output switches */
SOC_DOUBLE("Speaker Playback Switch", 	RT5611_SPK_OUT_VOL, 15, 7, 1, 1),
SOC_DOUBLE("Headphone Playback Switch", RT5611_HP_OUT_VOL,15, 7, 1, 1),
SOC_SINGLE("Mono Playback Switch", 	RT5611_PHONEIN_MONO_OUT_VOL, 7, 1, 1),
/* DAC & ADC volumes */
SOC_DOUBLE_TLV("PCM Playback Volume", 	RT5611_STEREO_DAC_VOL, 8, 0, 31, 1, rt5611_inp_tlv),
SOC_DOUBLE_TLV("Capture Volume", 	RT5611_ADC_REC_GAIN, 7, 0, 31, 0, rt5611_adc_tlv),
/* Input path volumes */
SOC_DOUBLE_TLV("Line In Volume", 	RT5611_LINE_IN_VOL, 8, 0, 31, 1, rt5611_inp_tlv),
SOC_SINGLE_TLV("Mic 1 Volume", 		RT5611_MIC_VOL, 8, 31, 1, rt5611_inp_tlv),
SOC_SINGLE_TLV("Mic 2 Volume", 		RT5611_MIC_VOL, 0, 31, 1, rt5611_inp_tlv),
SOC_SINGLE_TLV("Phone In Volume", 	RT5611_PHONEIN_MONO_OUT_VOL, 8, 31, 1, rt5611_inp_tlv),
/* Various things ;) */
SOC_ENUM("Mic 1 Boost", 		rt5611_enum[5]),
SOC_ENUM("Mic 2 Boost", 		rt5611_enum[6]),
SOC_ENUM("Speaker AMP Type", 		rt5611_enum[7]),
SOC_SINGLE_TLV("Headphones gain", 	RT5611_GEN_CTRL_REG1, 8, 2, 0, rt5611_hp_tlv),
SOC_SINGLE_TLV("Class D gain", 		RT5611_GEN_CTRL_REG1, 6, 3, 1, rt5611_cls_d_tlv),
SOC_SINGLE_TLV("Class AB gain", 	RT5611_GEN_CTRL_REG1, 3, 5, 1, rt5611_cls_ab_tlv),
};

/* We have to create a fake left and right HP mixers because
 * the codec only has a single control that is shared by both channels.
 * This makes it impossible to determine the audio path using the current
 * register map, thus we add a new (virtual) register to help determine the
 * audio route within the device.
 */
static int mixer_event (struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	u16 l, r, lineIn, mic, phone, pcm;

	l = ac97_read(w->codec, HPL_MIXER);
	r = ac97_read(w->codec, HPR_MIXER);
	lineIn = ac97_read(w->codec, RT5611_LINE_IN_VOL);
	mic = ac97_read(w->codec, RT5611_MIC_ROUTING_CTRL);
	phone = ac97_read(w->codec,RT5611_PHONEIN_MONO_OUT_VOL);
	pcm = ac97_read(w->codec, RT5611_STEREO_DAC_VOL);

	if (event & SND_SOC_DAPM_PRE_REG)
		return 0;

	if (l & 0x2 || r & 0x2)
		ac97_write(w->codec, RT5611_STEREO_DAC_VOL, pcm & 0x7fff);
	else
		ac97_write(w->codec, RT5611_STEREO_DAC_VOL, pcm | 0x8000);

	if (l & 0x4 || r & 0x4) /* Mic2 */
		ac97_write(w->codec, RT5611_MIC_ROUTING_CTRL, mic &= 0xff7f);
	else
		ac97_write(w->codec, RT5611_MIC_ROUTING_CTRL, mic |= 0x0080);
	
	if (l & 0x8 || r & 0x8) /* Mic1 */
		ac97_write(w->codec, RT5611_MIC_ROUTING_CTRL, mic & 0x7fff);
	else
		ac97_write(w->codec, RT5611_MIC_ROUTING_CTRL, mic | 0x8000);

	if (l & 0x10 || r & 0x10)
		ac97_write(w->codec, RT5611_PHONEIN_MONO_OUT_VOL, phone & 0x7fff);
	else
		ac97_write(w->codec, RT5611_PHONEIN_MONO_OUT_VOL, phone | 0x8000);

	if (l & 0x20 || r & 0x20)
		ac97_write(w->codec, RT5611_LINE_IN_VOL, lineIn & 0x7fff);
	else
		ac97_write(w->codec, RT5611_LINE_IN_VOL, lineIn | 0x8000);

	return 0;
}

/* Left Headphone Mixers */
static const struct snd_kcontrol_new rt5611_hpl_mixer_controls[] = {
SOC_DAPM_SINGLE("LineIn Playback Switch", HPL_MIXER, 5, 1, 0),
SOC_DAPM_SINGLE("PhoneIn Playback Switch", HPL_MIXER, 4, 1, 0),
SOC_DAPM_SINGLE("Mic1 Playback Switch", HPL_MIXER, 3, 1, 0),
SOC_DAPM_SINGLE("Mic2 Playback Switch", HPL_MIXER, 2, 1, 0),
SOC_DAPM_SINGLE("PCM Playback Switch", HPL_MIXER, 1, 1, 0),
SOC_DAPM_SINGLE("RecordL Playback Switch", RT5611_ADC_REC_GAIN, 15, 1,1),
};


/* Right Headphone Mixers */
static const struct snd_kcontrol_new rt5611_hpr_mixer_controls[] = {
SOC_DAPM_SINGLE("LineIn Playback Switch", HPR_MIXER, 5, 1, 0),
SOC_DAPM_SINGLE("PhoneIn Playback Switch", HPR_MIXER, 4, 1, 0),
SOC_DAPM_SINGLE("Mic1 Playback Switch", HPR_MIXER, 3, 1, 0),
SOC_DAPM_SINGLE("Mic2 Playback Switch", HPR_MIXER, 2, 1, 0),
SOC_DAPM_SINGLE("PCM Playback Switch", HPR_MIXER, 1, 1, 0),
SOC_DAPM_SINGLE("RecordR Playback Switch", RT5611_ADC_REC_GAIN, 14, 1,1),
};

/* Capture Mixers */
static const struct snd_kcontrol_new rt5611_captureL_mixer_controls[] = {
SOC_DAPM_SINGLE("Mic1 Capture Switch", RT5611_ADC_REC_MIXER, 14, 1, 1),
SOC_DAPM_SINGLE("Mic2 Capture Switch", RT5611_ADC_REC_MIXER, 13, 1, 1),
SOC_DAPM_SINGLE("LineInL Capture Switch",RT5611_ADC_REC_MIXER,12, 1, 1),
SOC_DAPM_SINGLE("Phone Capture Switch", RT5611_ADC_REC_MIXER, 11, 1, 1),
SOC_DAPM_SINGLE("HPMixerL Capture Switch", RT5611_ADC_REC_MIXER,10, 1, 1),
SOC_DAPM_SINGLE("SPKMixer Capture Switch",RT5611_ADC_REC_MIXER,9, 1, 1),
SOC_DAPM_SINGLE("MonoMixer Capture Switch",RT5611_ADC_REC_MIXER,8, 1, 1),
};


static const struct snd_kcontrol_new rt5611_captureR_mixer_controls[] = {
SOC_DAPM_SINGLE("Mic1 Capture Switch", RT5611_ADC_REC_MIXER, 6, 1, 1),
SOC_DAPM_SINGLE("Mic2 Capture Switch", RT5611_ADC_REC_MIXER, 5, 1, 1),
SOC_DAPM_SINGLE("LineInR Capture Switch",RT5611_ADC_REC_MIXER,4, 1, 1),
SOC_DAPM_SINGLE("Phone Capture Switch", RT5611_ADC_REC_MIXER, 3, 1, 1),
SOC_DAPM_SINGLE("HPMixerR Capture Switch", RT5611_ADC_REC_MIXER,2, 1, 1),
SOC_DAPM_SINGLE("SPKMixer Capture Switch",RT5611_ADC_REC_MIXER,1, 1, 1),
SOC_DAPM_SINGLE("MonoMixer Capture Switch",RT5611_ADC_REC_MIXER,0, 1, 1),
};


/* Speaker Mixer */
static const struct snd_kcontrol_new rt5611_speaker_mixer_controls[] = {
SOC_DAPM_SINGLE("LineIn Playback Switch", RT5611_LINE_IN_VOL, 14, 1, 1),
SOC_DAPM_SINGLE("PhoneIn Playback Switch", RT5611_PHONEIN_MONO_OUT_VOL, 14, 1, 1),
SOC_DAPM_SINGLE("Mic1 Playback Switch", RT5611_MIC_ROUTING_CTRL, 14, 1, 1),
SOC_DAPM_SINGLE("Mic2 Playback Switch", RT5611_MIC_ROUTING_CTRL, 6, 1, 1),
SOC_DAPM_SINGLE("PCM Playback Switch", RT5611_STEREO_DAC_VOL, 14, 1, 1),
};


/* Mono Mixer */
static const struct snd_kcontrol_new rt5611_mono_mixer_controls[] = {
SOC_DAPM_SINGLE("LineIn Playback Switch", RT5611_LINE_IN_VOL, 13, 1, 1),
SOC_DAPM_SINGLE("Mic1 Playback Switch", RT5611_MIC_ROUTING_CTRL, 13, 1, 1),
SOC_DAPM_SINGLE("Mic2 Playback Switch", RT5611_MIC_ROUTING_CTRL, 5, 1, 1),
SOC_DAPM_SINGLE("PCM Playback Switch", RT5611_STEREO_DAC_VOL, 13, 1, 1),
SOC_DAPM_SINGLE("RecL Playback Switch", RT5611_ADC_REC_GAIN, 13, 1, 1),
SOC_DAPM_SINGLE("RecR Playback Switch", RT5611_ADC_REC_GAIN, 12, 1, 1),
SOC_DAPM_SINGLE("RecordL Playback Switch", RT5611_ADC_REC_GAIN, 13, 1,1),
SOC_DAPM_SINGLE("RecordR Playback Switch", RT5611_ADC_REC_GAIN, 12, 1,1),
};

/* Mono Output mux */
static const struct snd_kcontrol_new rt5611_mono_mux_controls =
SOC_DAPM_ENUM("Route", rt5611_enum[4]);


/* Speaker Left Output Mux */
static const struct snd_kcontrol_new rt5611_hp_spkl_mux_controls =
SOC_DAPM_ENUM("Route", rt5611_enum[0]);


/* Speaker Right Output Mux */
static const struct snd_kcontrol_new rt5611_hp_spkr_mux_controls =
SOC_DAPM_ENUM("Route", rt5611_enum[1]);


/* Headphone Left Output Mux */
static const struct snd_kcontrol_new rt5611_hpl_out_mux_controls =
SOC_DAPM_ENUM("Route", rt5611_enum[2]);


/* Headphone Right Output Mux */
static const struct snd_kcontrol_new rt5611_hpr_out_mux_controls =
SOC_DAPM_ENUM("Route", rt5611_enum[3]);


static const struct snd_soc_dapm_widget rt5611_dapm_widgets[] = {
/* Output muxes */
SND_SOC_DAPM_MUX("Mono Out Mux", SND_SOC_NOPM, 0, 0,
	&rt5611_mono_mux_controls),
SND_SOC_DAPM_MUX("Left Speaker Out Mux", SND_SOC_NOPM, 0, 0,
	&rt5611_hp_spkl_mux_controls),
SND_SOC_DAPM_MUX("Right Speaker Out Mux", SND_SOC_NOPM, 0, 0,
	&rt5611_hp_spkr_mux_controls),
SND_SOC_DAPM_MUX("Left Headphone Out Mux", SND_SOC_NOPM, 0, 0,
	&rt5611_hpl_out_mux_controls),
SND_SOC_DAPM_MUX("Right Headphone Out Mux", SND_SOC_NOPM, 0, 0,
	&rt5611_hpr_out_mux_controls),
/* DAC */
SND_SOC_DAPM_DAC("Left DAC", "Left Playback", RT5611_PWR_MANAG_ADD2, 9, 0),
SND_SOC_DAPM_DAC("Right DAC", "Right Playback", RT5611_PWR_MANAG_ADD2, 8, 0),	
/* ADC */
SND_SOC_DAPM_ADC("Left ADC", "Left Capture", RT5611_PWR_MANAG_ADD2, 7, 0),
SND_SOC_DAPM_ADC("Right ADC", "Right Capture", RT5611_PWR_MANAG_ADD2, 6, 0),
/* Output mixers with their power control */
SND_SOC_DAPM_MIXER_E("Left HP Mixer",RT5611_PWR_MANAG_ADD2, 5, 0,
	rt5611_hpl_mixer_controls, ARRAY_SIZE(rt5611_hpl_mixer_controls),
	mixer_event, SND_SOC_DAPM_POST_REG),
SND_SOC_DAPM_MIXER_E("Right HP Mixer",RT5611_PWR_MANAG_ADD2, 4, 0,
	rt5611_hpr_mixer_controls, ARRAY_SIZE(rt5611_hpr_mixer_controls),
	mixer_event, SND_SOC_DAPM_POST_REG),
SND_SOC_DAPM_MIXER("Speaker Mixer", RT5611_PWR_MANAG_ADD2,3,0,
	rt5611_speaker_mixer_controls,
	ARRAY_SIZE(rt5611_speaker_mixer_controls)),
SND_SOC_DAPM_MIXER("Mono Mixer", RT5611_PWR_MANAG_ADD2, 2, 0,
	rt5611_mono_mixer_controls, ARRAY_SIZE(rt5611_mono_mixer_controls)),
SND_SOC_DAPM_MIXER("Left Record Mixer", RT5611_PWR_MANAG_ADD2,1,0,
	rt5611_captureL_mixer_controls,
	ARRAY_SIZE(rt5611_captureL_mixer_controls)),
SND_SOC_DAPM_MIXER("Right Record Mixer", RT5611_PWR_MANAG_ADD2,0,0,
	rt5611_captureR_mixer_controls,
	ARRAY_SIZE(rt5611_captureR_mixer_controls)),
/* Virtuals */
SND_SOC_DAPM_MIXER("AC97 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Line Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("HP Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
/* Output amplifiers */
SND_SOC_DAPM_PGA("Mono Out", RT5611_PWR_MANAG_ADD3, 14, 0, NULL, 0),
SND_SOC_DAPM_PGA("Left Speaker Inv", RT5611_PWR_MANAG_ADD3, 13, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Speaker Inv", RT5611_PWR_MANAG_ADD3, 12, 0, NULL, 0),
SND_SOC_DAPM_PGA("Left Headphone", RT5611_PWR_MANAG_ADD3, 11, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Headphone", RT5611_PWR_MANAG_ADD3, 10, 0, NULL, 0),
SND_SOC_DAPM_PGA("Left Speaker", RT5611_PWR_MANAG_ADD3, 9, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Speaker", RT5611_PWR_MANAG_ADD3, 8, 0, NULL, 0),
/* Input paths */
SND_SOC_DAPM_PGA("Left Line In PGA", RT5611_PWR_MANAG_ADD3, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Line In PGA", RT5611_PWR_MANAG_ADD3, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("Phone In PGA", RT5611_PWR_MANAG_ADD3, 5, 0, NULL, 0),
SND_SOC_DAPM_PGA("Phone In Pre Amp", RT5611_PWR_MANAG_ADD3, 4, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mic 1 PGA", RT5611_PWR_MANAG_ADD3, 3, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mic 2 PGA", RT5611_PWR_MANAG_ADD3, 2, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mic 1 Pre Amp", RT5611_PWR_MANAG_ADD3, 1, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mic 2 Pre Amp", RT5611_PWR_MANAG_ADD3, 0, 0, NULL, 0),

SND_SOC_DAPM_PGA("Mic 1 Diff Input", RT5611_MIC_ROUTING_CTRL, 12, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mic 2 Diff Input", RT5611_MIC_ROUTING_CTRL, 4, 0, NULL, 0),
SND_SOC_DAPM_PGA("Phone In Diff Input", RT5611_PHONEIN_MONO_OUT_VOL, 13, 0, NULL, 0),

SND_SOC_DAPM_MICBIAS("Mic 1 Bias", RT5611_PWR_MANAG_ADD1, 3, 0),
SND_SOC_DAPM_MICBIAS("Mic 2 Bias", RT5611_PWR_MANAG_ADD1, 2, 0),

/* Codec pins */
SND_SOC_DAPM_OUTPUT("MONO"),

SND_SOC_DAPM_OUTPUT("HPL"),
SND_SOC_DAPM_OUTPUT("HPR"),

SND_SOC_DAPM_OUTPUT("SPKL"),
SND_SOC_DAPM_OUTPUT("SPKR"),
SND_SOC_DAPM_OUTPUT("SPKLN"),
SND_SOC_DAPM_OUTPUT("SPKRN"),

SND_SOC_DAPM_INPUT("LINEL"),
SND_SOC_DAPM_INPUT("LINER"),
SND_SOC_DAPM_INPUT("PHONEIN"),
SND_SOC_DAPM_INPUT("PHONEINN"),
SND_SOC_DAPM_INPUT("MIC1"),
SND_SOC_DAPM_INPUT("MIC1N"),
SND_SOC_DAPM_INPUT("MIC2"),
SND_SOC_DAPM_INPUT("MIC2N"),
};


static const struct snd_soc_dapm_route rt5611_audio_map[] = {
	/* left HP mixer */
	{"Left HP Mixer", "LineIn Playback Switch", "Left Line In PGA"},
	{"Left HP Mixer", "PhoneIn Playback Switch","Phone In PGA"},
	{"Left HP Mixer", "Mic1 Playback Switch","Mic 1 PGA"},
	{"Left HP Mixer", "Mic2 Playback Switch","Mic 2 PGA"},
	{"Left HP Mixer", "PCM Playback Switch","Left DAC"},
	{"Left HP Mixer", "RecordL Playback Switch","Left Record Mixer"},
	
	/* right HP mixer */
	{"Right HP Mixer", "LineIn Playback Switch", "Right Line In PGA"},
	{"Right HP Mixer", "PhoneIn Playback Switch","Phone In PGA"},
	{"Right HP Mixer", "Mic1 Playback Switch","Mic 1 PGA"},
	{"Right HP Mixer", "Mic2 Playback Switch","Mic 2 PGA"},
	{"Right HP Mixer", "PCM Playback Switch","Right DAC"},
	{"Right HP Mixer", "RecordR Playback Switch","Right Record Mixer"},
	
	/* virtual mixers - mix stereo input for mono SPK and MONO mixers */
	{"AC97 Mixer", NULL, "Left DAC"},
	{"AC97 Mixer", NULL, "Right DAC"},
	{"Line Mixer", NULL, "Right Line In PGA"},
	{"Line Mixer", NULL, "Left Line In PGA"},
	
	/* Mixes stereo HP mixer output for mono outputs */
	{"HP Mixer", NULL, "Left HP Mixer"},
	{"HP Mixer", NULL, "Right HP Mixer"},
	
	/* speaker mixer */
	{"Speaker Mixer", "LineIn Playback Switch","Line Mixer"},
	{"Speaker Mixer", "PhoneIn Playback Switch","Phone In PGA"},
	{"Speaker Mixer", "Mic1 Playback Switch","Mic 1 PGA"},
	{"Speaker Mixer", "Mic2 Playback Switch","Mic 2 PGA"},
	{"Speaker Mixer", "PCM Playback Switch","AC97 Mixer"},

	/* mono mixer */
	{"Mono Mixer", "LineIn Playback Switch","Line Mixer"},
	{"Mono Mixer", "Mic1 Playback Switch","Mic 1 PGA"},
	{"Mono Mixer", "Mic2 Playback Switch","Mic 2 PGA"},
	{"Mono Mixer", "PCM Playback Switch","AC97 Mixer"},
	{"Mono Mixer", "RecordL Playback Switch","Left Record Mixer"},
	{"Mono Mixer", "RecordR Playback Switch","Right Record Mixer"},
	
	/*Left record mixer */
	{"Left Record Mixer", "Mic1 Capture Switch","Mic 1 Pre Amp"},
	{"Left Record Mixer", "Mic2 Capture Switch","Mic 2 Pre Amp"},
	{"Left Record Mixer", "LineInL Capture Switch","LINEL"},
	{"Left Record Mixer", "Phone Capture Switch","Phone In Pre Amp"},
	{"Left Record Mixer", "HPMixerL Capture Switch","Left HP Mixer"},
	{"Left Record Mixer", "SPKMixer Capture Switch","Speaker Mixer"},
	{"Left Record Mixer", "MonoMixer Capture Switch","Mono Mixer"},
	
	/*Right record mixer */
	{"Right Record Mixer", "Mic1 Capture Switch","Mic 1 Pre Amp"},
	{"Right Record Mixer", "Mic2 Capture Switch","Mic 2 Pre Amp"},
	{"Right Record Mixer", "LineInR Capture Switch","LINER"},
	{"Right Record Mixer", "Phone Capture Switch","Phone In Pre Amp"},
	{"Right Record Mixer", "HPMixerR Capture Switch","Right HP Mixer"},
	{"Right Record Mixer", "SPKMixer Capture Switch","Speaker Mixer"},
	{"Right Record Mixer", "MonoMixer Capture Switch","Mono Mixer"},	

	/* headphone left mux */
	{"Left Headphone Out Mux", "HPL mixer", "Left HP Mixer"},

	/* headphone right mux */
	{"Right Headphone Out Mux", "HPR mixer", "Right HP Mixer"},

	/* speaker left mux */
	{"Left Speaker Out Mux", "HPL mixer", "Left HP Mixer"},
	{"Left Speaker Out Mux", "SPK mixer", "Speaker Mixer"},
	{"Left Speaker Out Mux", "Mono Mixer", "Mono Mixer"},

	/* speaker right mux */
	{"Right Speaker Out Mux", "HPR mixer", "Right HP Mixer"},
	{"Right Speaker Out Mux", "SPK mixer", "Speaker Mixer"},
	{"Right Speaker Out Mux", "Mono Mixer", "Mono Mixer"},

	/* mono mux */
	{"Mono Out Mux", "HP mixer", "HP Mixer"},
	{"Mono Out Mux", "SPK mixer", "Speaker Mixer"},
	{"Mono Out Mux", "Mono Mixer", "Mono Mixer"},
	
	/* output pga */
	{"Left Headphone", NULL, "Left Headphone Out Mux"},
	{"Right Headphone", NULL, "Right Headphone Out Mux"},
	{"HPL", NULL, "Left Headphone"},
	{"HPR", NULL, "Right Headphone"},
	{"Left Speaker", NULL, "Left Speaker Out Mux"},
	{"Right Speaker", NULL, "Right Speaker Out Mux"},
	{"Left Speaker Inv", NULL, "Left Speaker"},
	{"Right Speaker Inv", NULL, "Right Speaker"},
	{"SPKL", NULL, "Left Speaker"},
	{"SPKR", NULL, "Right Speaker"},
	{"SPKLN", NULL, "Left Speaker Inv"},
	{"SPKRN", NULL, "Right Speaker Inv"},
	
	{"Mono Out", NULL, "Mono Out Mux"},
	{"MONO", NULL, "Mono Out"},

	/* input pga */
	{"Left Line In PGA", NULL, "LINEL"},
	{"Right Line In PGA", NULL, "LINER"},
	
	{"Phone In Diff Input", NULL, "PHONEINN"},
	{"Phone In Pre Amp", NULL, "Phone In Diff Input"},
	{"Phone In Pre Amp", NULL, "PHONEIN"},
	{"Phone In PGA", NULL, "Phone In Pre Amp"},
	
	{"Mic 1 Diff Input", NULL, "MIC1N"},
	{"Mic 2 Diff Input", NULL, "MIC2N"},
	{"Mic 1 Pre Amp", NULL, "Mic 1 Diff Input"},
	{"Mic 2 Pre Amp", NULL, "Mic 2 Diff Input"},
	{"Mic 1 Pre Amp", NULL, "MIC1"},
	{"Mic 2 Pre Amp", NULL, "MIC2"},
	{"Mic 1 PGA", NULL, "Mic 1 Pre Amp"},
	{"Mic 2 PGA", NULL, "Mic 2 Pre Amp"},

	/* left ADC */
	{"Left ADC", NULL, "Left Record Mixer"},

	/* right ADC */
	{"Right ADC", NULL, "Right Record Mixer"},
};


static unsigned int ac97_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;

	//if (reg == AC97_RESET || reg == AC97_GPIO_STATUS ||
	//	reg == AC97_VENDOR_ID1 || reg == AC97_VENDOR_ID2 ||
	//	reg == AC97_CD)
	if (reg < 0x7c)
		return soc_ac97_ops.read(codec->ac97, reg);
	else {
		reg = reg >> 1;

		if (reg >= (ARRAY_SIZE(rt5611_reg)))
			return -EIO;

		return cache[reg];
	}
}

static int ac97_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val)
{
	u16 *cache = codec->reg_cache;
	unsigned int rd;
	WARN_ON(reg==0x0);
	if (reg < 0x7c)
	{
		soc_ac97_ops.write(codec->ac97, reg, val);
		rd = soc_ac97_ops.read(codec->ac97, reg);
	}
	reg = reg >> 1;
	if (reg <= (ARRAY_SIZE(rt5611_reg)))
		cache[reg] = val;

	return 0;
}

static int ac97_write_mask(struct snd_soc_codec *codec,unsigned int reg,unsigned int val,unsigned int mask)
{
	unsigned int CodecData;

	CodecData=ac97_read(codec,reg);
	CodecData&=~mask;
	CodecData|=(val & mask);
	
	return ac97_write(codec,reg,CodecData);
}


/* PLL divisors */
struct _pll_div {
	u32 pll_in;
	u32 pll_out;
	u16 regvalue;
};

static const struct _pll_div rt5611_pll_div[] = {		
	{2048000,  24576000, 0x2ea0},
	{3686400,  24576000, 0xee27},	
	{12000000, 24576000, 0x2915},   
	{13000000, 24576000, 0x772e},
	{13100000, 24576000, 0x0d20},
	{24576000, 24576000, 0x0620},
};

static int rt5611_set_pll(struct snd_soc_codec *codec,
	int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	unsigned int i;
		
	if (!freq_in || !freq_out) 
		return 0;
	
	for (i = 0; i < ARRAY_SIZE(rt5611_pll_div); ++i) {
		if (rt5611_pll_div[i].pll_in == freq_in && rt5611_pll_div[i].pll_out == freq_out)
		{			 	
		 	printk("%s %d regval:%x \n",__FUNCTION__,__LINE__,rt5611_pll_div[i].regvalue);
		 	ac97_write(codec,RT5611_PLL_CTRL,rt5611_pll_div[i].regvalue);			 							 				 	
			break;
		}
	}
	//ac97_write_mask(codec,RT5611_GEN_CTRL_REG1,GP2_PLL_PRE_DIV_2,GP2_PLL_PRE_DIV_2);	
	
	//codec clock source from PLL output		
	ac97_write_mask(codec,RT5611_GEN_CTRL_REG1,GP_CLK_FROM_PLL,GP_CLK_FROM_PLL);	
	//enable PLL power	
	ac97_write_mask(codec,RT5611_PWR_MANAG_ADD2,PWR_PLL,PWR_PLL);						
	//Power off ACLink	
	ac97_write_mask(codec,RT5611_PD_CTRL_STAT,RT_PWR_PR4,RT_PWR_PR4);	 		
	//need ac97 controller to do warm reset	
	soc_ac97_ops.warm_reset(codec->ac97);
	/* wait 10ms AC97 link frames for the link to stabilise */
	schedule_timeout_interruptible(msecs_to_jiffies(10));	

	return 0;	
}	

static int rt5611_set_dai_pll(struct snd_soc_dai *codec_dai, int pll_id,
		int source, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	rt5611_set_pll(codec,pll_id,freq_in,freq_out);

	return 0;
}

static int ac97_hifi_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int reg;
	u16 vra;

	vra = ac97_read(codec, RT5611_TONE_CTRL);
	ac97_write(codec, RT5611_TONE_CTRL, vra | 0x1);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg = RT5611_STEREO_DAC_RATE;
	else
		reg = RT5611_STEREO_ADC_RATE;

	return ac97_write(codec, reg, runtime->rate);
}


#define RT5611_RATES (SNDRV_PCM_RATE_8000 |\
		      SNDRV_PCM_RATE_11025 |\
		      SNDRV_PCM_RATE_22050 |\
		      SNDRV_PCM_RATE_44100 |\
		      SNDRV_PCM_RATE_48000)

#define RT5611_PCM_FORMATS SNDRV_PCM_FMTBIT_S16_LE

static const struct snd_soc_dai_ops rt5611_dai_ops_hifi = {
	.prepare = ac97_hifi_prepare,
	.set_pll = rt5611_set_dai_pll,
};

static struct snd_soc_dai_driver rt5611_dai[] = {
	{
	.name = "rt5611-hifi",
	.ac97_control = 1,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = RT5611_RATES,
		.formats = RT5611_PCM_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = RT5611_RATES,
		.formats = RT5611_PCM_FORMATS,
	},
	.ops = &rt5611_dai_ops_hifi,
	},
};

int rt5611_reset(struct snd_soc_codec *codec, int try_warm)
{
	if (try_warm && soc_ac97_ops.warm_reset) {
		soc_ac97_ops.warm_reset(codec->ac97);
		if (ac97_read(codec, 0) == rt5611_reg[0])
			return 1;
	}

	soc_ac97_ops.reset(codec->ac97);
	if (soc_ac97_ops.warm_reset)
		soc_ac97_ops.warm_reset(codec->ac97);
	if (ac97_read(codec, 0) != rt5611_reg[0])
		return -EIO;
	return 0;
}
EXPORT_SYMBOL_GPL(rt5611_reset);

static int rt5611_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	u16 reg;

	printk(KERN_INFO "RT5611 bias %d %s\n", level,\
		(level == SND_SOC_BIAS_ON) ? "on" :\
		(level == SND_SOC_BIAS_PREPARE) ? "prepare" :\
		(level == SND_SOC_BIAS_STANDBY) ? "standby" :\
		(level == SND_SOC_BIAS_OFF) ? "off" :\
		"unk");
	switch (level) {
	case SND_SOC_BIAS_ON:
		/* enable thermal shutdown */
		ac97_write_mask(codec, RT5611_PWR_MANAG_ADD2, EN_THREMAL_SHUTDOWN, EN_THREMAL_SHUTDOWN);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		/* enable master bias and vmid */
		ac97_write_mask(codec, RT5611_PWR_MANAG_ADD1, PWR_MAIN_BIAS | PWR_DAC_REF, PWR_MAIN_BIAS | PWR_DAC_REF);
		ac97_write_mask(codec, RT5611_PWR_MANAG_ADD2, PWR_MIXER_VREF, PWR_MIXER_VREF);
		ac97_write(codec, RT5611_PD_CTRL_STAT, 0x0000);
		break;
	case SND_SOC_BIAS_OFF:
		/* disable everything including AC link */
		ac97_write(codec, RT5611_PWR_MANAG_ADD3, 0x0000);
		ac97_write(codec, RT5611_PWR_MANAG_ADD1, 0x0000);
		ac97_write(codec, RT5611_PWR_MANAG_ADD2, 0x0000);
		ac97_write(codec, RT5611_PD_CTRL_STAT, 0xFF00);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

static int rt5611_soc_suspend(struct snd_soc_codec *codec)
{
	rt5611_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int rt5611_soc_resume(struct snd_soc_codec *codec)
{
	struct rt5611_priv *rt5611 = snd_soc_codec_get_drvdata(codec);
	
	int i, ret;
	u16 *cache = codec->reg_cache;

	if ((ret = rt5611_reset(codec, 1)) < 0){
		printk(KERN_ERR "could not reset AC97 codec\n");
		return ret;
	}

	rt5611_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	
	/* do we need to re-start the PLL ? */
	if (rt5611->pll_out)
		rt5611_set_pll(codec, 0, rt5611->pll_in, rt5611->pll_out);

	/* only synchronise the codec if warm reset failed */
	if (ret == 0) {
		for (i = 2; i < ARRAY_SIZE(rt5611_reg) << 1; i+=2) {
			if (i > 0x66)
				continue;
			soc_ac97_ops.write(codec->ac97, i, cache[i>>1]);
		}
	}

	return ret;
}

static int rt5611_soc_probe(struct snd_soc_codec *codec)
{
	struct rt5611_priv *rt5611;
	int ret = 0;

	printk(KERN_INFO "RT5611 SoC Audio Codec %s\n", RT5611_VERSION);
	rt5611 = kzalloc(sizeof(struct rt5611_priv), GFP_KERNEL);
	if(!rt5611)
		return -ENOMEM;
	snd_soc_codec_set_drvdata(codec, rt5611);

	ret = snd_soc_new_ac97_codec(codec, &soc_ac97_ops, 0);
	if (ret < 0)
		goto codec_err;

	rt5611_reset(codec, 0);
	ret = rt5611_reset(codec, 1);
	if (ret < 0) {
		printk(KERN_ERR "Failed to reset RT5611: AC97 link error\n");
		goto reset_err;
	}

	rt5611_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	snd_soc_add_codec_controls(codec,rt5611_snd_ac97_controls,
				ARRAY_SIZE(rt5611_snd_ac97_controls));
	
	return 0;

reset_err:
	snd_soc_free_ac97_codec(codec);
codec_err:
	kfree(rt5611);
	return ret;
}

static int rt5611_soc_remove(struct snd_soc_codec *codec)
{
	struct rt5611_priv *rt5611 = snd_soc_codec_get_drvdata(codec);
	snd_soc_free_ac97_codec(codec);
	kfree(rt5611);
	return 0;
}


static struct snd_soc_codec_driver soc_codec_dev_rt5611 = {
	.probe = 	rt5611_soc_probe,
	.remove = 	rt5611_soc_remove,
	.suspend =	rt5611_soc_suspend,
	.resume = 	rt5611_soc_resume,
	.read =		ac97_read,
	.write =	ac97_write,
	.set_bias_level = rt5611_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(rt5611_reg),
	.reg_word_size = sizeof(u16),
	.reg_cache_step = 2,
	.reg_cache_default = rt5611_reg,
	.dapm_widgets = rt5611_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rt5611_dapm_widgets),
	.dapm_routes = rt5611_audio_map,
	.num_dapm_routes = ARRAY_SIZE(rt5611_audio_map),
};

static __devinit int rt5611_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_rt5611, rt5611_dai, ARRAY_SIZE(rt5611_dai));
}

static int __devexit rt5611_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver rt5611_codec_driver = {
	.driver = {
			.name = "rt5611-codec",
			.owner = THIS_MODULE,
	},

	.probe = rt5611_probe,
	.remove = __devexit_p(rt5611_remove),
};

module_platform_driver(rt5611_codec_driver);

MODULE_DESCRIPTION("ASoC RT5611 driver");
MODULE_AUTHOR("flove");
MODULE_LICENSE("GPL");
