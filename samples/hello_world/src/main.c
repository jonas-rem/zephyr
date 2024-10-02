/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <stdio.h>

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec step = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec direction = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

int main(void)
{
	int err;

	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	if (!gpio_is_ready_dt(&step)) {
		printf("The step pin GPIO port is not ready.\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&direction)) {
		printf("The direction pin GPIO port is not ready.\n");
		return 0;
	}

	err = gpio_pin_configure_dt(&step, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		printf("Configuring step GPIO pin failed: %d\n", err);
		return 0;
	}
	err = gpio_pin_configure_dt(&direction, GPIO_OUTPUT_INACTIVE);

	if (err != 0) {
		printf("Configuring direction GPIO pin failed: %d\n", err);
		return 0;
	}

	err = gpio_pin_set_dt(&direction, 0);

	while (1) {
		gpio_pin_set_dt(&step, 1);
		k_sleep(K_MSEC(2));
		gpio_pin_set_dt(&step, 0);
		k_sleep(K_MSEC(2));
	}

	return 0;
}
