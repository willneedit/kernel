/*
 * Tegra machine ASoC driver for boards using a MAX90809 CODEC.
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Based on code copyright/by:
 *
 * Copyright (C) 2010-2012 - NVIDIA, Inc.
 * Copyright (C) 2011 The AC100 Kernel Team <ac100@lists.lauchpad.net>
 * (c) 2009, 2010 Nvidia Graphics Pvt. Ltd.
 * Copyright 2007 Wolfson Microelectronics PLC.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "card_info.h"
#include "rk_pcm.h"
#include "rk_i2s.h"


#define DRV_NAME "rockchip-snd-max98090"


static int rockchip_max98090_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int pll_out = 0, dai_fmt = rtd->dai_link->dai_fmt;
	int ret;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_fmt);
	if (ret < 0) {
		printk("%s():failed to set the format for codec side\n", __FUNCTION__);
		return ret;
	}

	switch(params_rate(params)) {
		case 8000:
		case 16000:
		case 24000:
		case 32000:
		case 48000:
			pll_out = 12288000;
			break;
		case 11025:
		case 22050:
		case 44100:
			pll_out = 11289600;
			break;
		default:
			return -EINVAL;
			break;
	}

	snd_soc_dai_set_sysclk(cpu_dai, 0, pll_out, 0);
	snd_soc_dai_set_sysclk(codec_dai, 0, pll_out, 0);
 
	return 0;
}

static struct snd_soc_ops rockchip_max98090_ops = {
	.hw_params = rockchip_max98090_hw_params,
};

static struct snd_soc_jack rockchip_hp_jack;
static struct snd_soc_jack_pin rockchip_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Speakers",
		.mask = SND_JACK_HEADPHONE,
		.invert = 1,
	},
};

static struct snd_soc_jack_gpio rockchip_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
};

static struct snd_soc_jack rockchip_mic_jack;
static struct snd_soc_jack_pin rockchip_mic_jack_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
};

static struct snd_soc_jack_gpio rockchip_mic_jack_gpio = {
	.name = "mic detect",
	.report = SND_JACK_MICROPHONE,
};

static const struct snd_soc_dapm_route rockchip_audio_map[] = {
	{"Mic Jack", NULL, "MICBIAS"},
	{"MIC2", NULL, "Mic Jack"},
	{"Headphone Jack", NULL, "HPL"},
	{"Headphone Jack", NULL, "HPR"},
	{"Speakers", NULL, "SPKL"},
	{"Speakers", NULL, "SPKR"},
};

static const struct snd_soc_dapm_widget rockchip_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_SPK("Speakers", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static const struct snd_kcontrol_new rockchip_dapm_controls[] = {
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
	SOC_DAPM_PIN_SWITCH("Speakers"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
};

static int rockchip_max98090_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct device_node *dn = card->dev->of_node;

	if (dn) {
		enum of_gpio_flags flags;

		rockchip_mic_jack_gpio.gpio = of_get_named_gpio_flags(
				dn, "rockchip,mic-det-gpios", 0, &flags);
		rockchip_mic_jack_gpio.invert = !!(flags & OF_GPIO_ACTIVE_LOW);

		rockchip_hp_jack_gpio.gpio = of_get_named_gpio_flags(
				dn, "rockchip,hp-det-gpios", 0, &flags);
		rockchip_hp_jack_gpio.invert = !!(flags & OF_GPIO_ACTIVE_LOW);
	}

	if (gpio_is_valid(rockchip_mic_jack_gpio.gpio)) {
		snd_soc_jack_new(codec, "Mic Jack", SND_JACK_MICROPHONE,
				 &rockchip_mic_jack);
		snd_soc_jack_add_pins(&rockchip_mic_jack,
				      ARRAY_SIZE(rockchip_mic_jack_pins),
				      rockchip_mic_jack_pins);
		snd_soc_jack_add_gpios(&rockchip_mic_jack, 1,
				       &rockchip_mic_jack_gpio);
	}

	if (gpio_is_valid(rockchip_hp_jack_gpio.gpio)) {
		snd_soc_jack_new(codec, "Headphone Jack",
				 SND_JACK_HEADPHONE, &rockchip_hp_jack);
		snd_soc_jack_add_pins(&rockchip_hp_jack,
				      ARRAY_SIZE(rockchip_hp_jack_pins),
				      rockchip_hp_jack_pins);
		snd_soc_jack_add_gpios(&rockchip_hp_jack, 1,
				       &rockchip_hp_jack_gpio);
	}

	/* Microphone BIAS is needed to power the analog mic.
	 * MICBIAS2 is connected to analog mic (MIC3, which is in turn
	 * connected to MIC2 via 'External MIC') on the max98095 codec.
	 * Microphone BIAS is controlled by MICBIAS
	 * on the max98090 / max98091 codec.
	*/
	snd_soc_dapm_force_enable_pin(dapm, "MICBIAS");

	snd_soc_dapm_sync(dapm);

	return 0;
}

static struct snd_soc_dai_link rockchip_max98090_dai = {
	.name = "max98090",
	.stream_name = "max98090 PCM",
	.codec_dai_name = "HiFi",
	.ops = &rockchip_max98090_ops,
	.init = rockchip_max98090_init,
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
};

static struct snd_soc_card snd_soc_rockchip_max98090 = {
	.name = "rockchip-max98090",
	.owner = THIS_MODULE,
	.dai_link = &rockchip_max98090_dai,
	.num_links = 1,
	.controls = rockchip_dapm_controls,
	.num_controls = ARRAY_SIZE(rockchip_dapm_controls),
	.dapm_widgets = rockchip_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rockchip_dapm_widgets),
	.dapm_routes = rockchip_audio_map,
	.num_dapm_routes = ARRAY_SIZE(rockchip_audio_map),
};

static int rockchip_max98090_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &snd_soc_rockchip_max98090;

	card->dev = &pdev->dev;

	ret = rockchip_of_get_sound_card_info(card);
	if (ret) {
		printk("%s() get sound card info failed:%d\n", __FUNCTION__, ret);
		return ret;
	}

	ret = snd_soc_register_card(card);
	if (ret)
		printk("%s() register card failed:%d\n", __FUNCTION__, ret);

	return ret;
}

static int rockchip_max98090_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id rockchip_max98090_of_match[] = {
	{ .compatible = "rockchip-max98090", },
	{},
};

static struct platform_driver rockchip_max98090_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = rockchip_max98090_of_match,
	},
	.probe = rockchip_max98090_probe,
	.remove = rockchip_max98090_remove,
};
module_platform_driver(rockchip_max98090_driver);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra max98090 machine ASoC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, rockchip_max98090_of_match);
