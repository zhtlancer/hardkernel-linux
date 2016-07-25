/*
 * ODROID GPIO raw level control driver (/dev/gpioctrl)
 *
 * Copyright (C) 2017, Hardkernel Co,.Ltd
 * Author: Charles Park <charles.park@hardkernel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
/*---------------------------------------------------------------------------*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/compat.h>

/*---------------------------------------------------------------------------*/
#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
#include <linux/amlogic/iomap.h>
#endif

/*---------------------------------------------------------------------------*/
#define	ODROIDC2_GPIO_BASE	0xC8834000
#define	GPIO_MEM_SIZE		(1024 * 4)

static void __iomem		*gpio_base;
static struct miscdevice	*pmisc;

struct gpioctrl_iocreg	{
	__u32	reg_offset;
	__u32	reg_data;
	__u32	bit_mask;
	__u32	bit_data;
};

#define GPIOCTRL_IOCGREG	_IOR('g', 1, struct gpioctrl_iocreg)
#define GPIOCTRL_IOCWREG	_IOW('g', 2, struct gpioctrl_iocreg)


/*---------------------------------------------------------------------------*/
static inline void misc_gpioctrl_set(struct gpioctrl_iocreg *iocreg)
{
	iocreg->reg_data = readl(gpio_base + (iocreg->reg_offset << 2));

	if (iocreg->bit_data)
		iocreg->reg_data |= (iocreg->bit_mask);
	else
		iocreg->reg_data &= (~iocreg->bit_mask);

	writel(iocreg->reg_data, gpio_base + (iocreg->reg_offset << 2));
}

/*---------------------------------------------------------------------------*/
static inline void misc_gpioctrl_get(struct gpioctrl_iocreg *iocreg)
{
	iocreg->reg_data = readl(gpio_base + (iocreg->reg_offset << 2));
	iocreg->bit_data = (iocreg->reg_data & iocreg->bit_mask) ? 1 : 0;
}

/*---------------------------------------------------------------------------*/
static int misc_gpioctrl_open(struct inode *inode, struct file *file)
{
	return 0;
}
/*---------------------------------------------------------------------------*/
static long misc_gpioctrl_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct gpioctrl_iocreg	*iocreg = (struct gpioctrl_iocreg *)arg;

	switch (cmd) {
		/* Get GPIO Register */
	case	GPIOCTRL_IOCGREG:
		misc_gpioctrl_get(iocreg);
	break;
		/* Set GPIO Register */
	case	GPIOCTRL_IOCWREG:
		misc_gpioctrl_set(iocreg);
	break;
	default:
		pr_err("%s : unknown ioctl message!!\n", __func__);
	break;
	}
	return 0;
}

/*---------------------------------------------------------------------------*/
#if defined(CONFIG_COMPAT)
static long misc_gpioctrl_compat_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	return	misc_gpioctrl_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif
/*---------------------------------------------------------------------------*/
static const struct file_operations	gpioctrl_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= misc_gpioctrl_open,
	.unlocked_ioctl	= misc_gpioctrl_ioctl,
#if defined(CONFIG_COMPAT)
	.compat_ioctl	= misc_gpioctrl_compat_ioctl,
#endif
};

/*---------------------------------------------------------------------------*/
static int odroid_gpioctrl_suspend(struct platform_device *dev,
					pm_message_t state)
{
	return 0;
}

/*---------------------------------------------------------------------------*/
static int odroid_gpioctrl_resume(struct platform_device *dev)
{
	return 0;
}

/*---------------------------------------------------------------------------*/
static  int odroid_gpioctrl_remove(struct platform_device *pdev)
{
	if (pmisc)
		misc_deregister(pmisc);

	return 0;
}

/*---------------------------------------------------------------------------*/
static int odroid_gpioctrl_probe(struct platform_device *pdev)
{
	int rc = 0;

	pmisc = devm_kzalloc(&pdev->dev, sizeof(struct miscdevice),
					GFP_KERNEL);
	if (!pmisc) {
		pr_err("%s : odroid-gpioctrl misc struct malloc error!\n",
			__func__);
		return -ENOMEM;
	}

	pmisc->minor = MISC_DYNAMIC_MINOR;
	pmisc->name = "gpioctrl";
	pmisc->fops = &gpioctrl_misc_fops;

	rc = misc_register(pmisc);
	if (rc < 0)	{
		pr_err("%s : odroid-gpioctrl misc register fail!\n", __func__);
		return rc;
	}

	gpio_base = ioremap(ODROIDC2_GPIO_BASE, GPIO_MEM_SIZE);

	if (gpio_base == NULL)	{
		pr_err("%s : ioremap error! 0x%08X, size = 0x%08X\n",
			__func__, ODROIDC2_GPIO_BASE, GPIO_MEM_SIZE);
		return -1;
	}

	dev_info(&pdev->dev, "%s : success!\n", __func__);
	return rc;
}

/*---------------------------------------------------------------------------*/
#if defined(CONFIG_OF)
static const struct of_device_id odroid_gpioctrl_dt[] = {
	{ .compatible = "odroid-gpioctrl" },
	{ },
};
MODULE_DEVICE_TABLE(of, odroid_gpioctrl_dt);
#endif

/*---------------------------------------------------------------------------*/
static struct platform_driver odroid_gpioctrl_driver = {
	.driver = {
		.name = "odroid-gpioctrl",
		.owner = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(odroid_gpioctrl_dt),
#endif
	},
	.probe = odroid_gpioctrl_probe,
	.remove = odroid_gpioctrl_remove,
	.suspend = odroid_gpioctrl_suspend,
	.resume = odroid_gpioctrl_resume,
};

/*---------------------------------------------------------------------------*/
static int __init odroid_gpioctrl_init(void)
{
	return platform_driver_register(&odroid_gpioctrl_driver);
}

/*---------------------------------------------------------------------------*/
static void __exit odroid_gpioctrl_exit(void)
{
	platform_driver_unregister(&odroid_gpioctrl_driver);
}

/*---------------------------------------------------------------------------*/
module_init(odroid_gpioctrl_init);
module_exit(odroid_gpioctrl_exit);

/*---------------------------------------------------------------------------*/
MODULE_AUTHOR("Hardkernel Co.,Ltd");
MODULE_DESCRIPTION("GPIO Control driver for ODROID hardware");
MODULE_LICENSE("GPL");

/*---------------------------------------------------------------------------*/
