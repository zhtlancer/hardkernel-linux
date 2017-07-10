/*----------------------------------------------------------------------------*/
/*
 * Copyright (C) 2014, Hardkernel Co,.Ltd
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
/*----------------------------------------------------------------------------*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

/*----------------------------------------------------------------------------*/
#if defined(CONFIG_MACH_MESON8B_ODROIDC)

#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/of.h>

#define	AMLGPIO_IRQ_BASE	INT_GPIO_0

#endif

#if defined(CONFIG_ARCH_MESON64_ODROIDC2)

#include <linux/amlogic/pinctrl_amlogic.h>
#include <linux/amlogic/aml_gpio_consumer.h>

#define	AMLGPIO_IRQ_BASE	96

#endif

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*
	ODROID-ENCODER H/W & Config data Description

	* if ODROID-C1 or ODROID-C2 then basicalliy there are 5 interrupts used.
	  To get the state of the encoder phase, assign two irqs per port.
	  One irq for Push-Button.
	  if you have not enough interrupt, you can use ENCODER_USE_HALF flag
	  for reduce interrupt.
	  if ENCODER_USE_HALF is 1 then there are 3 interrupt used.
	  (sometimes not stable).

	40 Pin Header Used:
		16 - ENCODER PORT A
		18 - ENCODER PORT B
		22 - ENCODER PUSH Button

	* Hardware Layout (ACTIVE LEVEL : PORT_LEVEL, PUSH_BT_LEVEL = 1) *

	+ (Vcc 3V)
	|
	|    *Encoder Module : PEL12T-4225S-S1024
	|   ---------------------------------------------
	|---| Encoder Port common       Encoder Port A   |-- 40 Pin Header(16)
	|   |                           Encoder Port B   |-- 40 Pin Header(18)
	|   |                                            |
	|---| Encoder Push bt common    Encoder Push BT  |-- 40 Pin Header(22)
	    ----------------------------------------------
	* If PULL_UPDN_EN is 1 then don't need external pull-down resister.


	* Hardware Layout (ACTIVE LEVEL : PORT_LEVEL, PUSH_BT_LEVEL = 0) *

	     *Encoder Module : PEL12T-4225S-S1024
	    ---------------------------------------------
	|---| Encoder Port common       Encoder Port A   |-- 40 Pin Header(16)
	|   |                           Encoder Port B   |-- 40 Pin Header(18)
	|   |                                            |
	|---| Encoder Push bt common    Encoder Push BT  |-- 40 Pin Header(22)
	|   ----------------------------------------------
	|
	+ (GND 0V)
	* If PULL_UPDN_EN is 1 then don't need external pull-up resister.
*/
/*----------------------------------------------------------------------------*/
/* Will use only one rising or falling trigger irq for encoder port */
/* sometimes not stable. */
/* #define	ENCODER_USE_HALF_IRQ */

/* Default port data setup for driver test */
/* #define	ODROID_ENCODER_DEBUG */
/* #define	ODROID_ENCODER_DEBUG_MSG */
/*----------------------------------------------------------------------------*/
#if defined(ODROID_ENCODER_DEBUG)

/* Encoder Keycode define */
#define	ENCODEER_CCW_KEYCODE		KEY_VOLUMEDOWN
#define	ENCODEER_CW_KEYCODE		KEY_VOLUMEUP
#define	ENCODEER_PUSH_BT_KEYCODE	KEY_MUTE

/*----------------------------------------------------------------------------*/
#define	PORT_LEVEL	1	/* Active level */
#define	PUSH_BT_LEVEL	1	/* Active level */
#define	PULL_UPDN_EN	1	/* internal pull up/down enable */

/*----------------------------------------------------------------------------*/
#if defined(CONFIG_MACH_MESON8B_ODROIDC)

#define	GPIO_PORT_A	102	/* GPIOX.5 */
#define	GPIO_PORT_B	104	/* GPIOX.7 */
#define	GPIO_PUSH_BT	103	/* GPIOX.6 */

#endif

#if defined(CONFIG_ARCH_MESON64_ODROIDC2)

#define	GPIO_PORT_A	233	/* GPIOX.5 */
#define	GPIO_PORT_B	236	/* GPIOX.8 */
#define	GPIO_PUSH_BT	231	/* GPIOX.3 */

#endif

#if defined(CONFIG_MACH_ODROIDXU3)

#define	GPIO_PORT_A	23	/* GPX1.7 */
#define	GPIO_PORT_B	19	/* GPX1.3 */
#define	GPIO_PUSH_BT	24	/* GPX2.0 */

#endif

#endif	/* ODROID_ENCODER_DEBUG */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define	DRIVER_NAME	"odroid-encoder"

#define	ENCODER_NONE	0
/* Clockwise */
#define	ENCODER_CW	1
/* CounterClockwise */
#define	ENCODER_CCW	2
/* encoder rotation detect */
#define	ENCODER_ARMED	3

/*----------------------------------------------------------------------------*/
struct port_info {
	/* Control gpio number */
	u32	gpio;
	/* active level : 0 = low, 1 = high */
	u32	active;
	/* internal pull up/down resister enable */
	u32	pullen;
};

struct encoder_config {
	/* encoder gpio port description */
	struct port_info port_a;
	struct port_info port_b;
	struct port_info push_bt;

	/* Encoder dir : Counter Clockwise keycode */
	u32 ccw_keycode;
	/* Encoder dir : Clockwise keycode */
	u32 cw_keycode;
	/* Push button keycode */
	u32 push_bt_keycode;
};

#define	ENCODER_IOCSREG	_IOW('e', 1, struct encoder_config)
#define	ENCODER_IOCGREG	_IOR('e', 2, int)

/*----------------------------------------------------------------------------*/
enum DRIVER_STATE {
	DRV_ERR_NONE = 0,
	DRV_ERR_PORT_A,
	DRV_ERR_PORT_B,
	DRV_ERR_PUSH_BT,
	DRV_ERR_PORT_A_IRQ,
	DRV_ERR_PORT_B_IRQ,
	DRV_ERR_PUSH_BT_IRQ,
};

#define	DRV_ERROR_STATE(x, y)	(x |= (1 << y))

struct encoder {
	struct miscdevice	misc;
	struct input_dev	*input;
	struct platform_device	*pdev;

	struct work_struct	work;
	struct workqueue_struct	*work_q;

	/* enum DRIVER_STATE : bit field */
	u32	cfg_state;
	struct encoder_config	cfg;

	/* encoder direction save */
	u32	state;
	u32	direction;
	u32	armed;

	int	(*irq_setup_func)(void *);
	void	(*irq_free_func)(int, void *);
	int	(*pullupdn_func)(void *, const char *, void *);
};

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#if defined(ENCODER_USE_HALF_IRQ)
static void encoder_work_handler(struct work_struct *work)
{
	struct encoder *encoder =
		container_of(work, struct encoder, work);
	int keycode;

	switch (encoder->state) {
	case	ENCODER_NONE:
		switch (encoder->direction) {
		case	ENCODER_CW:
			keycode = encoder->cfg.cw_keycode;
			break;
		case	ENCODER_CCW:
			keycode = encoder->cfg.ccw_keycode;
			break;
		default:
			keycode = 0;
			break;
		}
		if (keycode) {
			input_report_key(encoder->input,
					keycode, 1);
			input_sync(encoder->input);

			input_report_key(encoder->input,
					keycode, 0);
			input_sync(encoder->input);
		}
		encoder->direction = ENCODER_NONE;
		break;
	case	ENCODER_CW:
	case	ENCODER_CCW:
	case	ENCODER_ARMED:
			encoder->direction = encoder->state;
		break;
	}
}

#else	/* ENCODER_USE_HALF_IRQ */

static void encoder_work_handler(struct work_struct *work)
{
	struct encoder *encoder =
		container_of(work, struct encoder, work);
	int keycode;

	switch (encoder->state) {
	case	ENCODER_NONE:
		if (encoder->armed) {
			switch (encoder->direction) {
			case	ENCODER_CW:
				keycode = encoder->cfg.cw_keycode;
				break;
			case	ENCODER_CCW:
				keycode = encoder->cfg.ccw_keycode;
				break;
			default:
				keycode = 0;
				break;
			}
			if (keycode) {
				input_report_key(encoder->input,
						keycode, 1);
				input_sync(encoder->input);
				input_report_key(encoder->input,
						keycode, 0);
				input_sync(encoder->input);
			}
			encoder->direction = ENCODER_NONE;
		}
		encoder->armed = 0;
		encoder->direction = 0;
		break;
	case	ENCODER_CW:
	case	ENCODER_CCW:
		if (encoder->armed)
			encoder->direction = encoder->state;
		break;
	case	ENCODER_ARMED:
		encoder->armed = 1;
		break;
	}
}

#endif	/* ENCODER_USE_HALF_IRQ */
/*----------------------------------------------------------------------------*/
static int encoder_port_state(struct encoder *encoder)
{
	int state, a_gpio, b_gpio;

	a_gpio = !!gpio_get_value(encoder->cfg.port_a.gpio);
	b_gpio = !!gpio_get_value(encoder->cfg.port_b.gpio);

	if (!encoder->cfg.port_a.active)
		a_gpio = a_gpio ? 0 : 1;

	if (!encoder->cfg.port_b.active)
		b_gpio = b_gpio ? 0 : 1;

	state  = a_gpio ? 0x01 : 0x00;
	state |= b_gpio ? 0x02 : 0x00;

#if defined(ODROID_ENCODER_DEBUG_MSG)
	pr_err("%s : a_gpio = %d, b_gpio = %d, state = %d\n",
		__func__, a_gpio, b_gpio, state);
#endif
	return	state;
}

/*----------------------------------------------------------------------------*/
static irqreturn_t encoder_port_irq(int irq, void *handle)
{
	struct encoder *encoder = handle;
	int cur_state;

	cur_state = encoder_port_state(encoder);

	if (encoder->state != cur_state) {
#if defined(ODROID_ENCODER_DEBUG_MSG)
		pr_err("%s : cur_state = %d\n", __func__, cur_state);
#endif
		encoder->state = cur_state;
		queue_work(encoder->work_q, &encoder->work);
	}

	return	IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
static irqreturn_t encoder_push_bt_irq(int irq, void *handle)
{
	struct encoder *encoder = handle;
	static int state = -1;

	if (state == -1) {
		state = !!gpio_get_value(encoder->cfg.push_bt.gpio);
		return	IRQ_HANDLED;
	}

	state = !!gpio_get_value(encoder->cfg.push_bt.gpio);

	if (state != encoder->cfg.push_bt.active) {
		input_report_key(encoder->input,
			encoder->cfg.push_bt_keycode, 1);
		input_sync(encoder->input);
		input_report_key(encoder->input,
			encoder->cfg.push_bt_keycode, 0);
		input_sync(encoder->input);
	}

	return	IRQ_HANDLED;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#if defined(CONFIG_ARCH_MESON64_ODROIDC2) \
	|| defined(CONFIG_MACH_MESON8B_ODROIDC)
/*----------------------------------------------------------------------------*/
static void amlogic_irq_free(int gpio, void *handle)
{
	struct encoder *encoder = handle;
	int irq_banks[2], free_irq_num;

	if (gpio) {
#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
		free_irq_num = gpio_to_irq(gpio);
#else
		free_irq_num = gpio;
#endif
		meson_free_irq(free_irq_num, &irq_banks[0]);
		/* rising irq bank */
		if (irq_banks[0] != -1)
			free_irq(irq_banks[0] + AMLGPIO_IRQ_BASE, encoder);
		/* falling irq bank */
		if (irq_banks[1] != -1)
			free_irq(irq_banks[1] + AMLGPIO_IRQ_BASE, encoder);
	}
}
/*----------------------------------------------------------------------------*/
static int _amlogic_irq_setup(struct encoder *encoder,
				struct port_info *port,
				int irq_flags,
				irqreturn_t (*irq_func)(int, void *))
{
	struct device *dev = &encoder->pdev->dev;
	int ret = 0, irq_bank, aml_irq, aml_irq_flags;

	if (!port->gpio) {
		dev_err(dev, "%s : gpio value is 0!\n", __func__);
		return -EINVAL;
	}

	irq_bank = meson_fix_irqbank(0);
	if (irq_bank < 0) {
		dev_err(dev, "Could not find irq bank!\n");
		return -EINVAL;
	}
	aml_irq = AML_GPIO_IRQ(irq_bank, FILTER_NUM7, irq_flags);

	ret = gpio_for_irq(port->gpio, aml_irq);
	if (ret) {
		dev_err(dev, "AML_GPIO_IRQ setup fail(gpio%d),(flags=0x%x)!\n",
				port->gpio, aml_irq);
		return -EINVAL;
	} else {
		aml_irq_flags = IRQF_DISABLED | IRQF_ONESHOT;

		ret = request_irq(AMLGPIO_IRQ_BASE + irq_bank,
					irq_func,
					aml_irq_flags,
					DRIVER_NAME,
					encoder);
		if (ret) {
			dev_err(dev, "request_irq fail! irq = %d\n",
				AMLGPIO_IRQ_BASE + irq_bank);
			return -EINVAL;
		}
	}
	return	0;
}

/*----------------------------------------------------------------------------*/
#if defined(ENCODER_USE_HALF_IRQ)
static int amlogic_irq_setup(void *handle)
{
	struct encoder *encoder = handle;
	int err = 0, irq_flags;

	/* Encoder Port A Interrupts */
	if (encoder->cfg.port_a.active) {
		err = _amlogic_irq_setup(encoder, &encoder->cfg.port_a,
				GPIO_IRQ_FALLING, encoder_port_irq);
		if (err < 0) {
			DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_A_IRQ);
			goto err_irq_port_a;
		}
	} else {
		err = _amlogic_irq_setup(encoder, &encoder->cfg.port_a,
				GPIO_IRQ_RISING, encoder_port_irq);
		if (err < 0) {
			DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_A_IRQ);
			goto err_irq_port_a;
		}
	}

	/* Encoder Port B Interrupts */
	if (encoder->cfg.port_b.active) {
		err = _amlogic_irq_setup(encoder, &encoder->cfg.port_b,
				GPIO_IRQ_FALLING, encoder_port_irq);
		if (err < 0) {
			DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_B_IRQ);
			goto err_irq_port_b;
		}
	} else {
		err = _amlogic_irq_setup(encoder, &encoder->cfg.port_b,
				GPIO_IRQ_RISING, encoder_port_irq);
		if (err < 0) {
			DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_B_IRQ);
			goto err_irq_port_b;
		}
	}

	/* Encoder Push button Interrupt */
	if (encoder->cfg.push_bt.gpio) {
		irq_flags = encoder->cfg.push_bt.active ?
				GPIO_IRQ_FALLING : GPIO_IRQ_RISING;
		err = _amlogic_irq_setup(encoder, &encoder->cfg.push_bt,
				irq_flags, encoder_push_bt_irq);
		if (err < 0) {
			DRV_ERROR_STATE(encoder->cfg_state,
						DRV_ERR_PUSH_BT_IRQ);
			goto err_irq_push_bt;
		}
	}
	return	0;

err_irq_push_bt:
	if (encoder->irq_free_func)
		encoder->irq_free_func(encoder->cfg.push_bt.gpio, encoder);
err_irq_port_b:
	if (encoder->irq_free_func)
		encoder->irq_free_func(encoder->cfg.port_b.gpio, encoder);
err_irq_port_a:
	if (encoder->irq_free_func)
		encoder->irq_free_func(encoder->cfg.port_a.gpio, encoder);
	return	err;
}

#else	/* ENCODER_USE_HALF_IRQ */

static int amlogic_irq_setup(void *handle)
{
	struct encoder *encoder = handle;
	int err = 0, irq_flags;

	/* Encoder Port A Interrupts */
	err = _amlogic_irq_setup(encoder, &encoder->cfg.port_a,
			GPIO_IRQ_FALLING, encoder_port_irq);
	if (err < 0) {
		DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_A_IRQ);
		goto err_irq_port_a;
	}

	err = _amlogic_irq_setup(encoder, &encoder->cfg.port_a,
			GPIO_IRQ_RISING, encoder_port_irq);
	if (err < 0) {
		DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_A_IRQ);
		goto err_irq_port_a;
	}

	/* Encoder Port B Interrupts */
	err = _amlogic_irq_setup(encoder, &encoder->cfg.port_b,
			GPIO_IRQ_FALLING, encoder_port_irq);
	if (err < 0) {
		DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_B_IRQ);
		goto err_irq_port_b;
	}

	err = _amlogic_irq_setup(encoder, &encoder->cfg.port_b,
			GPIO_IRQ_RISING, encoder_port_irq);
	if (err < 0) {
		DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_B_IRQ);
		goto err_irq_port_b;
	}

	/* Encoder Push button Interrupt */
	if (encoder->cfg.push_bt.gpio) {
		irq_flags = encoder->cfg.push_bt.active ?
				GPIO_IRQ_FALLING : GPIO_IRQ_RISING;
		err = _amlogic_irq_setup(encoder, &encoder->cfg.push_bt,
				irq_flags, encoder_push_bt_irq);
		if (err < 0) {
			DRV_ERROR_STATE(encoder->cfg_state,
						DRV_ERR_PUSH_BT_IRQ);
			goto err_irq_push_bt;
		}
	}
	return	0;

err_irq_push_bt:
	if (encoder->irq_free_func)
		encoder->irq_free_func(encoder->cfg.push_bt.gpio, encoder);
err_irq_port_b:
	if (encoder->irq_free_func)
		encoder->irq_free_func(encoder->cfg.port_b.gpio, encoder);
err_irq_port_a:
	if (encoder->irq_free_func)
		encoder->irq_free_func(encoder->cfg.port_a.gpio, encoder);
	return	err;
}
#endif	/* ENCODER_USE_HALF_IRQ */

/*----------------------------------------------------------------------------*/
static int amlogic_pullupdn(void *port_data, const char *port_str,
			void *handle)
{
	struct encoder *encoder = handle;
	struct device *dev = &encoder->pdev->dev;
	struct port_info *port = port_data;
	int ret = 0, pullupdn;

	if (port->pullen) {
		/* 0:pull down, 1:pull up */
		pullupdn = (port->active == 0) ? 1 : 0;
#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
		ret = gpio_set_pullup(port->gpio, pullupdn);
#else
		ret = amlogic_set_pull_up_down(port->gpio, pullupdn, port_str);
#endif
		if (ret)
			dev_err(dev, "gpio %s(%d), pullupdn = %d error!\n",
				port_str, port->gpio, pullupdn);
	}
	return	ret;
}

/*----------------------------------------------------------------------------*/
#else
/*----------------------------------------------------------------------------*/
static void generic_irq_free(int gpio, void *handle)
{
	struct encoder *encoder = handle;

	if (gpio)
		free_irq(gpio_to_irq(gpio), encoder);
}

/*----------------------------------------------------------------------------*/
static int generic_irq_setup(void *handle)
{
	struct encoder *encoder = handle;
	int err = 0;

	encoder->cfg.port_a.irq = gpio_to_irq(encoder->cfg.port_a.gpio);
	if (encoder->cfg.port_a.irq < 0) {
		DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_A_IRQ);
		return -EINVAL;
	}
	encoder->cfg.port_a.irq_flags =
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;

	err = request_irq(encoder->cfg.port_a.irq, encoder_port_irq,
			encoder->cfg.port_a.irq_flags, DRIVER_NAME, encoder);
	if (err) {
		DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_A_IRQ);
		return -EINVAL;
	}

	encoder->cfg.port_b.irq = gpio_to_irq(encoder->cfg.port_b.gpio);
	if (encoder->cfg.port_b.irq < 0) {
		err = -EINVAL;
		DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_B_IRQ);
		goto	err_irq_port_b;
	}
	encoder->cfg.port_b.irq_flags =
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;

	err = request_irq(encoder->cfg.port_b.irq, encoder_port_irq,
			encoder->cfg.port_b.irq_flags, DRIVER_NAME, encoder);
	if (err) {
		DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_B_IRQ);
		goto	err_irq_port_b;
	}

	if (encoder->cfg.push_bt.gpio) {
		encoder->cfg.push_bt.irq =
			gpio_to_irq(encoder->cfg.push_bt.gpio);
		if (encoder->cfg.push_bt.irq < 0) {
			err = -EINVAL;
			DRV_ERROR_STATE(encoder->cfg_state,
						DRV_ERR_PUSH_BT_IRQ);
			goto	err_irq_push_bt;
		}
		encoder->cfg.push_bt.irq_flags =
			encoder->cfg.push_bt.active ?
				IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;

		err = request_irq(encoder->cfg.push_bt.irq,
				encoder_push_bt_irq,
				encoder->cfg.push_bt.irq_flags,
				DRIVER_NAME, encoder);
		if (err) {
			DRV_ERROR_STATE(encoder->cfg_state,
						DRV_ERR_PUSH_BT_IRQ);
			goto	err_irq_push_bt;
		}
	}

	return	0;
err_irq_push_bt:
	if (encoder->irq_free_func)
		encoder->irq_free_func(encoder->cfg.port_b.gpio, encoder);
err_irq_port_b:
	if (encoder->irq_free_func)
		encoder->irq_free_func(encoder->cfg.port_a.gpio, encoder);
	return	err;
}

/*----------------------------------------------------------------------------*/
#if defined(CONFIG_MACH_ODROIDXU3)
static int exynos5422_pullupdn(void *port_data, const str *port_str,
				void *handle)
{
	struct encoder *encoder = handle;
	struct device *dev = &encoder->pdev->dev;
	struct port_info *port = port_data;
	int ret = 0, pullupdn;

	if (port->pullen) {
		/* 0:pull none, 1:pull down, 2:pull up */
		pullupdn = (port->active == 0) ? 1 : 2;
		ret = s3c_gpio_setpull(port->gpio, pullupdn);
		if (ret)
			dev_err(dev, "gpio %s(%d), pullupdn = %d error!\n",
				port_str, port->gpio, pullupdn);
	}
	return	ret;
}
#endif
/*----------------------------------------------------------------------------*/
#endif
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int encoder_gpio_setup(struct encoder *encoder,
			struct port_info *cur_port_info,
			struct port_info *new_port_info,
			const char *port_str)
{
	struct device *dev = &encoder->pdev->dev;

	if (new_port_info->gpio) {
		if (gpio_request(new_port_info->gpio, port_str)) {
			dev_err(dev, "%s : gpio(%d) request fail!\n",
				port_str, new_port_info->gpio);
			return	-EINVAL;
		} else {
			gpio_direction_input(new_port_info->gpio);
			cur_port_info->gpio   = new_port_info->gpio;
			cur_port_info->active = new_port_info->active;
			cur_port_info->pullen = new_port_info->pullen;
			if (encoder->pullupdn_func)
				encoder->pullupdn_func(cur_port_info,
							port_str, encoder);
		}
	} else
		dev_err(dev, "warring : encoder %s not define!\n", port_str);

	return	0;
}

/*----------------------------------------------------------------------------*/
static void encoder_clear_setup(struct encoder *encoder)
{
	/* interrupt clear */
	if (encoder->irq_free_func) {
		encoder->irq_free_func(encoder->cfg.port_a.gpio, encoder);
		encoder->irq_free_func(encoder->cfg.port_b.gpio, encoder);
		encoder->irq_free_func(encoder->cfg.push_bt.gpio, encoder);
	}
	/* request gpio free */
	if (encoder->cfg.port_a.gpio)
		gpio_free(encoder->cfg.port_a.gpio);
	if (encoder->cfg.port_b.gpio)
		gpio_free(encoder->cfg.port_b.gpio);
	if (encoder->cfg.push_bt.gpio)
		gpio_free(encoder->cfg.push_bt.gpio);

	/* configuration erase */
	memset(&encoder->cfg, 0x00, sizeof(struct encoder_config));

	/* driver state init */
	encoder->cfg_state = DRV_ERR_NONE;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int encoder_setup(struct encoder *encoder, struct encoder_config *newcfg)
{
	if (encoder->cfg_state == DRV_ERR_NONE)
		encoder_clear_setup(encoder);

	encoder->cfg_state = DRV_ERR_NONE;

	if (encoder_gpio_setup(encoder, &encoder->cfg.port_a,
				&newcfg->port_a, "port_a")) {
		DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_A);
		goto err_port_a;
	}

	if (encoder_gpio_setup(encoder, &encoder->cfg.port_b,
				&newcfg->port_b, "port_b")) {
		DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PORT_B);
		goto err_port_b;
	}

	if (encoder_gpio_setup(encoder, &encoder->cfg.push_bt,
				&newcfg->push_bt, "push_bt")) {
		DRV_ERROR_STATE(encoder->cfg_state, DRV_ERR_PUSH_BT);
		goto err_push_bt;
	}

	encoder->cfg.cw_keycode      = newcfg->cw_keycode;
	encoder->cfg.ccw_keycode     = newcfg->ccw_keycode;
	encoder->cfg.push_bt_keycode = newcfg->push_bt_keycode;

	if (encoder->irq_setup_func) {
		if (encoder->irq_setup_func(encoder))
			goto err_cfg;
	}

	/* setup complete */
	encoder->cfg_state = DRV_ERR_NONE;
	return	0;
err_cfg:
	if (newcfg->push_bt.gpio)
		gpio_free(newcfg->push_bt.gpio);
err_push_bt:
	if (newcfg->port_b.gpio)
		gpio_free(newcfg->port_b.gpio);
err_port_b:
	if (newcfg->port_a.gpio)
		gpio_free(newcfg->port_a.gpio);
err_port_a:
	memset(&encoder->cfg, 0x00, sizeof(struct encoder_config));
	return -EINVAL;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static long encoder_misc_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct encoder *encoder =
		container_of(file->private_data, struct encoder, misc);
	u32	*state = (u32 *)arg;
	int	ret = 0;

	switch (cmd) {
	case	ENCODER_IOCSREG:
		ret = encoder_setup(encoder, (struct encoder_config *)arg);
		break;
	case	ENCODER_IOCGREG:
		*state = encoder->cfg_state;
		break;
	default:
		dev_err(&encoder->pdev->dev,
			"%s : unknown message\n", __func__);
		break;
	}
	return	ret;
}

/*----------------------------------------------------------------------------*/
#if defined(CONFIG_COMPAT)
static long encoder_misc_compat_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	return	encoder_misc_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

/*----------------------------------------------------------------------------*/
static int encoder_misc_open(struct inode *inode, struct file *file)
{
	return	0;
}

/*----------------------------------------------------------------------------*/
static const struct file_operations	encoder_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= encoder_misc_open,
	.unlocked_ioctl = encoder_misc_ioctl,
#if defined(CONFIG_COMPAT)
	.compat_ioctl	= encoder_misc_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static int encoder_misc_setup(struct device *dev, struct encoder *encoder)
{
	encoder->misc.minor = MISC_DYNAMIC_MINOR;
	encoder->misc.name  = DRIVER_NAME;
	encoder->misc.fops  = &encoder_misc_fops;
	encoder->misc.mode  = S_IWUGO | S_IRUGO;

	return	misc_register(&encoder->misc);
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int encoder_input_setup(struct device *dev, struct encoder *encoder)
{
	int i;

	encoder->input = devm_input_allocate_device(dev);
	if (!encoder->input) {
		dev_err(dev, "input devm_kzalloc().\n");
		return	-ENOMEM;
	}

	encoder->input->dev.parent	= dev;
	encoder->input->name		= DRIVER_NAME;
	encoder->input->id.bustype	= BUS_HOST;
	encoder->input->id.vendor	= 0x16B4;
	encoder->input->id.product	= 0x0701;
	encoder->input->id.version	= 0x0001;

	set_bit(EV_KEY,  encoder->input->evbit);

	for (i = 0; i < KEY_MAX; i++)
		set_bit(i & KEY_MAX, encoder->input->keybit);

	input_set_drvdata(encoder->input, encoder);

	return	input_register_device(encoder->input);
}

/*----------------------------------------------------------------------------*/
#if defined(ODROID_ENCODER_DEBUG)
static void encoder_setup_for_test(struct encoder *encoder)
{
	struct encoder_config newcfg;

	memset(&newcfg, 0x00, sizeof(struct encoder_config));

	/* Header PIN Number 16 */
	newcfg.port_a.gpio   = GPIO_PORT_A;
	newcfg.port_a.active = PORT_LEVEL;
	newcfg.port_a.pullen = PULL_UPDN_EN;
	/* Header PIN Number 18 */
	newcfg.port_b.gpio   = GPIO_PORT_B;
	newcfg.port_b.active = PORT_LEVEL;
	newcfg.port_b.pullen = PULL_UPDN_EN;
	/* Header PIN Number 22 */
	newcfg.push_bt.gpio   = GPIO_PUSH_BT;
	newcfg.push_bt.active = PUSH_BT_LEVEL;
	newcfg.push_bt.pullen = PULL_UPDN_EN;

	newcfg.ccw_keycode	= ENCODEER_CCW_KEYCODE;
	newcfg.cw_keycode	= ENCODEER_CW_KEYCODE;
	newcfg.push_bt_keycode	= ENCODEER_PUSH_BT_KEYCODE;

	if (encoder_setup(encoder, &newcfg)) {
		dev_err(&encoder->pdev->dev,
			"%s : setup error!\n", __func__);
	}
	dev_info(&encoder->pdev->dev,
		"%s : setup complete!\n", __func__);
}
#endif
/*----------------------------------------------------------------------------*/
static int encoder_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct encoder *encoder;
	int err = 0;

	dev_info(dev, "%s\n", __func__);

	encoder = devm_kzalloc(dev, sizeof(struct encoder), GFP_KERNEL);
	if (!encoder) {
		dev_err(dev, "encoder devm kzmalloc().\n");
		return -ENOMEM;
	}

	err = encoder_input_setup(dev, encoder);
	if (err) {
		dev_err(dev, "encoder_input_setup().\n");
		return	err;
	}

	err = encoder_misc_setup(dev, encoder);
	if (err) {
		dev_err(dev, "encoder_misc_setup().\n");
		return	err;
	}

	INIT_WORK(&encoder->work, encoder_work_handler);
	encoder->work_q = create_singlethread_workqueue(DRIVER_NAME);
	if (!encoder->work_q) {
		dev_err(dev, "error work_q create!\n");
		return	-ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, encoder);
	encoder->pdev = pdev;

	dev_info(dev, "%s success!!\n", __func__);

#if defined(CONFIG_ARCH_MESON64_ODROIDC2) \
	|| defined(CONFIG_MACH_MESON8B_ODROIDC)
	encoder->irq_setup_func = amlogic_irq_setup;
	encoder->irq_free_func = amlogic_irq_free;
#else
	encoder->irq_setup_func = generic_irq_setup;
	encoder->irq_free_func = generic_irq_free;
#endif

#if defined(CONFIG_ARCH_MESON64_ODROIDC2) \
	|| defined(CONFIG_MACH_MESON8B_ODROIDC)
	encoder->pullupdn_func = amlogic_pullupdn;
#endif
#if defined(CONFIG_MACH_ODROIDXU3)
	encoder->pullupdn_func = exynos5422_pullupdn;
#endif

#if defined(ODROID_ENCODER_DEBUG)
	/* test encoder enable */
	encoder_setup_for_test(encoder);
#endif
	return 0;
}

/*----------------------------------------------------------------------------*/
static int encoder_remove(struct platform_device *pdev)
{
	struct encoder *encoder = dev_get_drvdata(&pdev->dev);

	cancel_work_sync(&encoder->work);
	destroy_workqueue(encoder->work_q);

	if (encoder->cfg_state)
		encoder_clear_setup(encoder);

	input_unregister_device(encoder->input);

	misc_deregister(&encoder->misc);

	return 0;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static const struct of_device_id encoder_of_match[] = {
	{ .compatible = DRIVER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(of, encoder_of_match);

static struct platform_driver encoder_driver = {
	.probe  = encoder_probe,
	.remove = encoder_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
		.of_match_table = of_match_ptr(encoder_of_match),
	},
};

module_platform_driver(encoder_driver);

MODULE_AUTHOR("Hardkernel Co.,Ltd");
MODULE_DESCRIPTION("odroid-encoder driver");
MODULE_LICENSE("GPL");
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
