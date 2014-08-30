/*
 * Rockchip machine ASoC driver for boards using a MAX90809 CODEC.
 *
 * Copyright (c) 2014, ROCKCHIP CORPORATION.  All rights reserved.
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

#include "rockchip_i2s.h"

#define DRV_NAME "rockchip-snd-max98090"

struct rockchip_max98090 {
	int gpio_hp_det;
	int gpio_mic_det;
};

static int rockchip_max98090_asoc_hw_params(struct snd_pcm_substream *substream,
					    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	int mclk, ret;
	unsigned int dai_fmt = rtd->dai_link->dai_fmt;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_fmt);
	if (ret < 0) {
		dev_err(card->dev, "codec_dai format not set, %d\n", ret);
		return ret;
	}

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_fmt);
	if (ret < 0) {
		dev_err(card->dev, "cpu_dai format not set, %d\n", ret);
		return ret;
	}

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		return -EINVAL;
	}
	mclk = 12000000;
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(card->dev, "cpu_dai clock not set, %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(card->dev, "codec_dai clock not set, %d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops rockchip_max98090_ops = {
	.hw_params = rockchip_max98090_asoc_hw_params,
};

static struct snd_soc_jack rockchip_max98090_hp_jack;
static struct snd_soc_jack_pin rockchip_max98090_hp_jack_pins[] = {
	{
		.pin = "Headphones",
		.mask = SND_JACK_HEADPHONE,
	},
};

static struct snd_soc_jack_gpio rockchip_max98090_hp_jack_gpio = {
	.name = "Headphone detection",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
	.invert = 1,
};

static struct snd_soc_jack rockchip_max98090_mic_jack;
static struct snd_soc_jack_pin rockchip_max98090_mic_jack_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
};

static struct snd_soc_jack_gpio rockchip_max98090_mic_jack_gpio = {
	.name = "mic detect",
	.report = SND_JACK_MICROPHONE,
	.debounce_time = 150,
	.invert = 0,
};

static const struct snd_soc_dapm_widget rockchip_max98090_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Speakers", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static const struct snd_kcontrol_new rockchip_max98090_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speakers"),
};

static int rockchip_max98090_asoc_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct device_node *dn = card->dev->of_node;

	if (dn) {
		enum of_gpio_flags flags;

		rockchip_max98090_mic_jack_gpio.gpio = of_get_named_gpio_flags(
				dn, "rockchip,mic-det-gpios", 0, &flags);
		rockchip_max98090_mic_jack_gpio.invert = !!(flags & OF_GPIO_ACTIVE_LOW);

		rockchip_max98090_hp_jack_gpio.gpio = of_get_named_gpio_flags(
				dn, "rockchip,hp-det-gpios", 0, &flags);
		rockchip_max98090_hp_jack_gpio.invert = !!(flags & OF_GPIO_ACTIVE_LOW);
	}

	if (gpio_is_valid(rockchip_max98090_mic_jack_gpio.gpio)) {
		snd_soc_jack_new(codec, "Mic Jack", SND_JACK_MICROPHONE,
				 &rockchip_max98090_mic_jack);
		snd_soc_jack_add_pins(&rockchip_max98090_mic_jack,
				      ARRAY_SIZE(rockchip_max98090_mic_jack_pins),
				      rockchip_max98090_mic_jack_pins);
		snd_soc_jack_add_gpios(&rockchip_max98090_mic_jack, 1,
				       &rockchip_max98090_mic_jack_gpio);
	}

	if (gpio_is_valid(rockchip_max98090_hp_jack_gpio.gpio)) {
		snd_soc_jack_new(codec, "Headphone Jack",
				 SND_JACK_HEADPHONE, &rockchip_max98090_hp_jack);
		snd_soc_jack_add_pins(&rockchip_max98090_hp_jack,
				      ARRAY_SIZE(rockchip_max98090_hp_jack_pins),
				      rockchip_max98090_hp_jack_pins);
		snd_soc_jack_add_gpios(&rockchip_max98090_hp_jack, 1,
				       &rockchip_max98090_hp_jack_gpio);
	}

	return 0;
}

static struct snd_soc_dai_link rockchip_max98090_dai = {
	.name = "max98090",
	.stream_name = "max98090 PCM",
	.codec_dai_name = "HiFi",
	.init = rockchip_max98090_asoc_init,
	.ops = &rockchip_max98090_ops,
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		   SND_SOC_DAIFMT_CBM_CFM,
};

static struct snd_soc_card snd_soc_rockchip_max98090 = {
	.name = "ROCKCHIP-I2S",
	.owner = THIS_MODULE,
	.dai_link = &rockchip_max98090_dai,
	.num_links = 1,
	.controls = rockchip_max98090_controls,
	.num_controls = ARRAY_SIZE(rockchip_max98090_controls),
	.dapm_widgets = rockchip_max98090_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rockchip_max98090_dapm_widgets),
	.fully_routed = true,
};

static int rockchip_max98090_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_rockchip_max98090;
	struct rockchip_max98090 *machine;
	int ret;

	machine = devm_kzalloc(&pdev->dev, sizeof(*machine), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate rockchip_max98090\n");
		return -ENOMEM;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	machine->gpio_hp_det = of_get_named_gpio(np, "rockchip,hp-det-gpios", 0);
	if (machine->gpio_hp_det == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	ret = snd_soc_of_parse_card_name(card, "rockchip,model");
	if (ret)
		goto err;

	ret = snd_soc_of_parse_audio_routing(card, "rockchip,audio-routing");
	if (ret)
		goto err;

	rockchip_max98090_dai.codec_of_node = of_parse_phandle(np,
			"rockchip,audio-codec", 0);
	if (!rockchip_max98090_dai.codec_of_node) {
		dev_err(&pdev->dev,
			"Property 'rockchip,audio-codec' missing or invalid\n");
		ret = -EINVAL;
		goto err;
	}

	rockchip_max98090_dai.cpu_of_node = of_parse_phandle(np,
			"rockchip,i2s-controller", 0);
	if (!rockchip_max98090_dai.cpu_of_node) {
		dev_err(&pdev->dev,
			"Property 'rockchip,i2s-controller' missing or invalid\n");
		ret = -EINVAL;
		goto err;
	}

	rockchip_max98090_dai.platform_of_node = rockchip_max98090_dai.cpu_of_node;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err;
	}

	return 0;

err:
	return ret;
}

static int rockchip_max98090_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_jack_free_gpios(&rockchip_max98090_hp_jack, 1,
				&rockchip_max98090_hp_jack_gpio);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id rockchip_max98090_of_match[] = {
	{ .compatible = "rockchip,rockchip-audio-max98090", },
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

MODULE_AUTHOR("jianqun <xjq@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip max98090 machine ASoC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, rockchip_max98090_of_match);
