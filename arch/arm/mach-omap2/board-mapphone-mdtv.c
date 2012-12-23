/*
 * linux/arch/arm/mach-omap2/board-MAPPHONE-sensors.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <plat/mux.h>
#include <plat/gpio.h>
#include <plat/keypad.h>

#include "linux/i2c/lp3907_i2c.h"

#define MAPPHONE_MDTV_INT_GPIO			38
#define MAPPHONE_MDTV_PWDN_GPIO			53
#define MAPPHONE_MDTV_RESET_N_GPIO		54
#define MAPPHONE_MDTV_REG_EN_GPIO		21

static int mapphone_lp3907_init(void)
{
	printk(KERN_INFO "mapphone_lp3907_init()");
	return 0;
}

static void mapphone_lp3907_exit(void)
{
}

static int mapphone_lp3907_power_on(void)
{
	printk(KERN_INFO "mapphone_lp3907_power_on()\n");
	/* SPI pin control */
	omap_cfg_reg(F1_34XX_MDTV_INT_ON);

	/* EN_T is high */
	gpio_set_value(MAPPHONE_MDTV_REG_EN_GPIO, 1);
	msleep(6);

	return 0;
}

static int mapphone_lp3907_power_off(void)
{
	printk(KERN_INFO "mapphone_lp3907_power_off()\n");
	/* SPI pin control */
	omap_cfg_reg(F1_34XX_MDTV_INT_OFF);

	/* EN_T is low */
	gpio_set_value(MAPPHONE_MDTV_REG_EN_GPIO, 0);
	msleep(6);

	return 0;
}

struct lp3907_platform_data mapphone_lp3907_data = {
	.init = mapphone_lp3907_init,
	.exit = mapphone_lp3907_exit,
	.power_on = mapphone_lp3907_power_on,
	.power_off = mapphone_lp3907_power_off,
};

/*
*	TDMB module initialize.
*/
void __init mapphone_mdtv_init(void)
{
	/* MTV_INT pin */
	gpio_request(MAPPHONE_MDTV_INT_GPIO, "sms1130 int");
	gpio_direction_input(MAPPHONE_MDTV_INT_GPIO);

	/* MTV_PWDN pin - low */
	gpio_request(MAPPHONE_MDTV_PWDN_GPIO, "sms1130 pwdn");
	gpio_direction_output(MAPPHONE_MDTV_PWDN_GPIO, 0);

	/* MTV_RST_N pin - low */
	gpio_request(MAPPHONE_MDTV_RESET_N_GPIO, "sms1130 reset");
	gpio_direction_output(MAPPHONE_MDTV_RESET_N_GPIO, 0);

	/* MTV_REG_EN pin - low */
	gpio_request(MAPPHONE_MDTV_REG_EN_GPIO, "lp3907 en");
	gpio_direction_output(MAPPHONE_MDTV_REG_EN_GPIO, 0);

	printk(KERN_INFO "[TDMB] mapphone_mdtv_init()\n");
}
