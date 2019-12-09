/*
 * sound/soc/pxa/hpipaq214-audio.c  --  SoC audio for HP IPAQ 214.
 *
 * Mostly based on sound/soc/pxa/tosa.c.
 *
 * Authors: Oliver Ford <ipaqcode-oliford-co-uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/audio.h>

#include "../codecs/wm9713.h"
#include "pxa2xx-pcm.h"
#include "pxa2xx-ac97.h"


static struct snd_soc_card hpipaq214;
//static struct workqueue_struct *jack_detect_workqueue;
static struct snd_soc_codec *theCodec; //FIXME: Yea, that's bad, I know.

static int hpipaq214_spk_func;		/* These are our internal copy of the kcontrol */
static int hpipaq214_hp_func;		/* state of the two UI switches */

static int hpipaq214_hp_jack_on;	/* These are our internal copy of the DAPM widget states*/
static int hpipaq214_hp_24pin_on;	/* so we know when not to disable IPAQ214_GPIO_HP_EN */

/* gpio 7_2 must be off for headset to work _properly_,
 * it must be on for hedaphones to work _properly_
 * gpio 98 must be on for either to work at all 
 * 
 * Here, the jack/24-pin DAPM objects can theoretically 
 * be both on. If so, the jack will be selected.
 *
 * TODO: Maybe that should use a MUX DAPM object?
 */
//#define IPAQ214_GPIO_HP_DETECT		95
#define IPAQ214_GPIO_SPK_EN		96
#define	IPAQ214_GPIO_HP_EN		98
#define	IPAQ214_GPIO_HP_JACK		7

#define IPAQ214_HP_AUTO		0
#define IPAQ214_HP_JACK		1
#define IPAQ214_HP_24PIN	2
#define IPAQ214_HP_OFF		3

#define IPAQ214_SPK_AUTO	0
#define IPAQ214_SPK_ON		1
#define IPAQ214_SPK_OFF		2

static void hpipaq214_ext_control(struct snd_soc_codec *codec)
{
	int hpPresent = 0; //gpio_get_value(IPAQ214_GPIO_HP_DETECT) ? 1 : 0;

	/* Speaker */
	if ( hpipaq214_spk_func == IPAQ214_SPK_ON ||
			(hpipaq214_spk_func == IPAQ214_SPK_AUTO && !hpPresent) )
		snd_soc_dapm_enable_pin(codec, "iPAQ Speaker");
	else
		snd_soc_dapm_disable_pin(codec, "iPAQ Speaker");

	/* Headphones on jack or 24-pin */
	if ( hpipaq214_hp_func == IPAQ214_HP_JACK ||
			(hpipaq214_hp_func == IPAQ214_HP_AUTO && hpPresent) )

		snd_soc_dapm_enable_pin(codec, "iPAQ Headphone Jack");
	else
		snd_soc_dapm_disable_pin(codec, "iPAQ Headphone Jack");


	if ( hpipaq214_hp_func == IPAQ214_HP_24PIN ||
			(hpipaq214_hp_func == IPAQ214_HP_AUTO && !hpPresent) )

		snd_soc_dapm_enable_pin(codec, "iPAQ Headset on 24-pin");
	else
		snd_soc_dapm_disable_pin(codec, "iPAQ Headset on 24-pin");
		
	snd_soc_dapm_sync(codec);
}

static int hpipaq214_startup(struct snd_pcm_substream *substream){	
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->card->codec;

	hpipaq214_ext_control(codec);

	return 0;
}

static struct snd_soc_ops hpipaq214_ops = {
 	.startup = hpipaq214_startup,
};

static int hpipaq214_get_hp(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = hpipaq214_hp_func;
	return 0;
}

static int hpipaq214_set_hp(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (hpipaq214_hp_func == ucontrol->value.integer.value[0])
		return 0;

	hpipaq214_hp_func = ucontrol->value.integer.value[0];
	hpipaq214_ext_control(codec);
	return 1;
}

static int hpipaq214_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = hpipaq214_spk_func;
	return 0;
}

static int hpipaq214_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (hpipaq214_spk_func == ucontrol->value.integer.value[0])
		return 0;

	hpipaq214_spk_func = ucontrol->value.integer.value[0];
	hpipaq214_ext_control(codec);
	return 1;
}

/* Headphone jack detection */
/*static void hpipaq214_jack_detect_work(struct work_struct *work){

 	if (hpipaq214_spk_func == IPAQ214_SPK_AUTO || 
			hpipaq214_hp_func == IPAQ214_HP_AUTO)
 		hpipaq214_ext_control(theCodec);
}*/

/*static irqreturn_t hpipaq214_hp_detect_irq(int irq, void *codec)
{
	
	static int initialised = 0;
	static struct work_struct task;
	
	if (initialised == 0) {
		INIT_WORK(&task, hpipaq214_jack_detect_work);
		initialised = 1;
	} else {
		PREPARE_WORK(&task, hpipaq214_jack_detect_work);
	}
	
	//queue_work(jack_detect_workqueue, &task);

	return IRQ_HANDLED;
}*/

/* hpipaq214 dapm event handlers */
static int hpipaq214_hp_jack_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	hpipaq214_hp_jack_on = SND_SOC_DAPM_EVENT_ON(event) ? 1 : 0;

	gpio_set_value(IPAQ214_GPIO_HP_EN, hpipaq214_hp_jack_on || hpipaq214_hp_24pin_on);
	gpio_set_value(IPAQ214_GPIO_HP_JACK, hpipaq214_hp_jack_on); //force select to jack if its on

	return 0;
}

static int hpipaq214_hp_24pin_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	hpipaq214_hp_24pin_on = SND_SOC_DAPM_EVENT_ON(event) ? 1 : 0;

	gpio_set_value(IPAQ214_GPIO_HP_EN, hpipaq214_hp_jack_on || hpipaq214_hp_24pin_on);
	gpio_set_value(IPAQ214_GPIO_HP_JACK, hpipaq214_hp_jack_on); //force select to jack if its on
	
	return 0;
}

static int hpipaq214_spk_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	gpio_set_value(IPAQ214_GPIO_SPK_EN, SND_SOC_DAPM_EVENT_ON(event) ? 1 : 0);

	return 0;
}

/* hpipaq214 machine dapm widgets */
static const struct snd_soc_dapm_widget hpipaq214_dapm_widgets[] = {
	SND_SOC_DAPM_HP("iPAQ Headphone Jack", hpipaq214_hp_jack_event),
	SND_SOC_DAPM_HP("iPAQ Headset on 24-pin", hpipaq214_hp_24pin_event),
	SND_SOC_DAPM_SPK("iPAQ Speaker", hpipaq214_spk_event),
	//SND_SOC_DAPM_MIC("Mic (Internal)", NULL),
};

/* hpipaq214 audio map */
static const struct snd_soc_dapm_route audio_map[] = {

	/* headphone connected to HPL, HPR */
	{"iPAQ Headphone Jack", NULL, "HPL"},
	{"iPAQ Headphone Jack", NULL, "HPR"},

	/* and also the 24-pin connecotr */
	{"iPAQ Headset on 24-pin", NULL, "HPL"},
	{"iPAQ Headset on 24-pin", NULL, "HPR"},
	
	/* ext speaker connected to SPKL, OUT4 */
	{"iPAQ Speaker", NULL, "SPKL"},
	{"iPAQ Speaker", NULL, "OUT4"},

	/* SOMETHING is physically connected to these, don't know what though */
	/*{"????", NULL, "SPKR"},
	{"????", NULL, "OUT3"},*/


	/* internal mic is connected to mic1, mic2 differential - with bias */
	//{"MIC1", NULL, "Mic Bias"},
	//{"MIC2", NULL, "Mic Bias"},
	//{"Mic Bias", NULL, "Mic (Internal)"},

	/* headset is connected to HPOUTR, and LINEINR with bias */
	//{"LINEINR", NULL, "Mic Bias"},
	//{"Mic Bias", NULL, "Headset Jack"},
};

static const char *hp_function[] = {"Auto", "Jack", "24-pin", "Off"};
static const char *spk_function[] = {"Auto", "On", "Off"};
static const struct soc_enum hpipaq214_enum[] = {
	SOC_ENUM_SINGLE_EXT(4, hp_function),
	SOC_ENUM_SINGLE_EXT(3, spk_function),
};

static const struct snd_kcontrol_new hpipaq214_controls[] = {
	SOC_ENUM_EXT("iPAQ Headphone Output", hpipaq214_enum[0], hpipaq214_get_hp,
		hpipaq214_set_hp),
	SOC_ENUM_EXT("iPAQ Speaker", hpipaq214_enum[1], hpipaq214_get_spk,
		hpipaq214_set_spk),
};

static int hpipaq214_ac97_init(struct snd_soc_codec *codec)
{
	int i, err, hp_det_irq;

	 //theses are actually connected to something, but I don't know what.
	snd_soc_dapm_nc_pin(codec, "SPKR");
	snd_soc_dapm_nc_pin(codec, "OUT3");
	snd_soc_dapm_nc_pin(codec, "MONO");

	/* add hpipaq214 specific controls */
	for (i = 0; i < ARRAY_SIZE(hpipaq214_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&hpipaq214_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}

	/* add hpipaq214 specific widgets */
	snd_soc_dapm_new_controls(codec, hpipaq214_dapm_widgets,
				  ARRAY_SIZE(hpipaq214_dapm_widgets));

	/* init our internal state copies */
	hpipaq214_hp_jack_on = 1;
	hpipaq214_hp_24pin_on = 0;

	/* set up hpipaq214 specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	snd_soc_dapm_enable_pin(codec, "iPAQ Speaker");
	snd_soc_dapm_sync(codec);

	// hp_det_irq = gpio_to_irq(IPAQ214_GPIO_HP_DETECT);
	
	theCodec = codec;
	//jack_detect_workqueue = create_workqueue("Jack Detect");
	//FIXME: Free this IRQ somewhere!!
	//if( request_irq(hp_det_irq, hpipaq214_hp_detect_irq,
	//		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	//		"HP Insert Detect", codec) )
	//	printk(KERN_ERR "Unable to get HP jack detect IRQ.");

	return 0;
}

static struct snd_soc_dai_link hpipaq214_dai[] = {
{
	.name = "AC97",
	.stream_name = "AC97 HiFi",
	.cpu_dai = &pxa_ac97_dai[PXA2XX_DAI_AC97_HIFI],
	.codec_dai = &wm9713_dai[WM9713_DAI_AC97_HIFI],
	.init = hpipaq214_ac97_init,
	.ops = &hpipaq214_ops,
},
// { Don't want this yet
// 	.name = "AC97 Aux",
// 	.stream_name = "AC97 Aux",
// 	.cpu_dai = &pxa_ac97_dai[PXA2XX_DAI_AC97_AUX],
// 	.codec_dai = &wm9713_dai[WM9713_DAI_AC97_AUX],
// 	.ops = &hpipaq214_ops,
// },
};

static struct snd_soc_card hpipaq214 = {
	.name = "hpipaq214",
	.platform = &pxa2xx_soc_platform,
	.dai_link = hpipaq214_dai,
	.num_links = ARRAY_SIZE(hpipaq214_dai),
};

static struct snd_soc_device hpipaq214_snd_devdata = {
	.card = &hpipaq214,
	.codec_dev = &soc_codec_dev_wm9713,
};

static struct platform_device *hpipaq214_snd_device;

static int __init hpipaq214_init(void)
{
	int ret;

	if (!machine_is_hpipaq214())
		return -ENODEV;

	//if (gpio_request(IPAQ214_GPIO_HP_DETECT, "HP Insert Detect"))
	//	printk(KERN_ERR "Unable to register 'HP Insert Detect' gpio (%i)\n",
	//		IPAQ214_GPIO_HP_DETECT);
	if (gpio_request(IPAQ214_GPIO_SPK_EN, "Speaker Enable"))
		printk(KERN_ERR "Unable to register 'Speaker Enable' gpio (%i)\n",
			IPAQ214_GPIO_SPK_EN);
	if (gpio_request(IPAQ214_GPIO_HP_EN, "Headphone/set Enable"))
		printk(KERN_ERR "Unable to register 'Headphone/set Enable' gpio (%i)\n",
			IPAQ214_GPIO_HP_EN);
	if (gpio_request(IPAQ214_GPIO_HP_JACK, "Headphone jack/24-pin Select"))
		printk(KERN_ERR "Unable to register 'Headphone jack/24-pin Select' gpio (%i)\n",
			IPAQ214_GPIO_HP_JACK);

	//gpio_direction_input(IPAQ214_GPIO_HP_DETECT);
	gpio_direction_output(IPAQ214_GPIO_SPK_EN, 0);
	gpio_direction_output(IPAQ214_GPIO_HP_EN, 0);
	gpio_direction_output(IPAQ214_GPIO_HP_JACK, 0);

	hpipaq214_snd_device = platform_device_alloc("soc-audio", -1);
	if (!hpipaq214_snd_device){
		ret = -ENOMEM;
		goto err_alloc;
	}

	platform_set_drvdata(hpipaq214_snd_device, &hpipaq214_snd_devdata);
	hpipaq214_snd_devdata.dev = &hpipaq214_snd_device->dev;
	ret = platform_device_add(hpipaq214_snd_device);

	if (!ret)
		return 0;

	platform_device_put(hpipaq214_snd_device);

err_alloc:
	//gpio_free(IPAQ214_GPIO_HP_DETECT);
	gpio_free(IPAQ214_GPIO_SPK_EN);
	gpio_free(IPAQ214_GPIO_HP_EN);
	gpio_free(IPAQ214_GPIO_HP_JACK);

	return ret;
}

static void __exit hpipaq214_exit(void)
{
	platform_device_unregister(hpipaq214_snd_device);
	//gpio_free(IPAQ214_GPIO_HP_DETECT);
	gpio_free(IPAQ214_GPIO_SPK_EN);
	gpio_free(IPAQ214_GPIO_HP_EN);
	gpio_free(IPAQ214_GPIO_HP_JACK);
	//FIXME: free_irq( hp_det_irq, ??? );
}

module_init(hpipaq214_init);
module_exit(hpipaq214_exit);

/* Module information */
MODULE_AUTHOR("Oliver Ford");
MODULE_DESCRIPTION("ALSA SoC HP iPAQ 214");
MODULE_LICENSE("GPL");
