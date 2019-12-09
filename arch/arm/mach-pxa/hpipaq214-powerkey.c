/*
 * Driver for the power key of the iPAQ 214
 *
 * Copyright 2012 Alvin Wong <alvinhochun-at-gmail-com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>

#include <mach/pxa3xx-regs.h>

#define HPIPAQ214_POWERKEY_IRQ		IRQ_WAKEUP0
#define HPIPAQ214_POWERKEY_KEYCODE	KEY_POWER
#define HPIPAQ214_POWERKEY_STATE	( PECR & 0x00000001 )

static irqreturn_t hpipaq214_powerkey_irq(int irq, void *dev_id)
{
	struct input_dev *input = dev_id;

	BUG_ON(irq != HPIPAQ214_POWERKEY_IRQ);

	input_event(input, EV_KEY, HPIPAQ214_POWERKEY_KEYCODE, HPIPAQ214_POWERKEY_STATE);
	input_sync(input);

	return IRQ_HANDLED;
}

static int __devinit hpipaq214_powerkey_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int error;
	int wakeup = 0;

	input = input_allocate_device();
	if (!input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	input->name = pdev->name;
	input->phys = "hpipaq214-powerkey/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	error = request_irq(HPIPAQ214_POWERKEY_IRQ,
		            hpipaq214_powerkey_irq,
		            IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		            "hpipaq214_powerkey",
		            input);
	if (error) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			HPIPAQ214_POWERKEY_IRQ, error);
		goto fail2;
	}

	input_set_capability(input, EV_KEY, HPIPAQ214_POWERKEY_KEYCODE);

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail2;
	}

	/* get current state of power key */
	input_event(input, EV_KEY, HPIPAQ214_POWERKEY_KEYCODE, HPIPAQ214_POWERKEY_STATE);
	input_sync(input);

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail2:
	platform_set_drvdata(pdev, input);
 fail1:
	input_free_device(input);

	return error;
}

static int __devexit hpipaq214_powerkey_remove(struct platform_device *pdev)
{
	struct input_dev *input = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);

	free_irq(HPIPAQ214_POWERKEY_IRQ, input);

	return 0;
}


#ifdef CONFIG_PM
static int hpipaq214_powerkey_suspend(struct device *dev)
{
	return 0;
}

static int hpipaq214_powerkey_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops hpipaq214_powerkey_pm_ops = {
	.suspend	= hpipaq214_powerkey_suspend,
	.resume		= hpipaq214_powerkey_resume,
};
#endif

static struct platform_driver hpipaq214_powerkey_device_driver = {
	.probe		= hpipaq214_powerkey_probe,
	.remove		= __devexit_p(hpipaq214_powerkey_remove),
	.driver		= {
		.name	= "hpipaq214-powerkey",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &hpipaq214_powerkey_pm_ops,
#endif
	}
};

static int __init hpipaq214_powerkey_init(void){
	return platform_driver_register(&hpipaq214_powerkey_device_driver);
}

static void __exit hpipaq214_powerkey_exit(void)
{
	platform_driver_unregister(&hpipaq214_powerkey_device_driver);
}

module_init(hpipaq214_powerkey_init);
module_exit(hpipaq214_powerkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alvin Wong <alvinhochun@gmail.com>");
MODULE_DESCRIPTION("Power key driver for iPAQ 214");
MODULE_ALIAS("platform:hpipaq214-powerkey");
