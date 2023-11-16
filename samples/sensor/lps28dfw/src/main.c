/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2023 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include <zephyr/shell/shell.h>

#define N_SAMPLES 100

/*
 * getting up to 10 proximity sensors from the device tree as
 * preparation for multiple proximity sensors; starting at prox_sensor0
 */
#define PRESSURE_SENSORS_ALIASES(i) DT_ALIAS(_CONCAT(press, i))
#define PRESSURE_DEVICES(i, _) \
	IF_ENABLED(DT_NODE_EXISTS(PRESSURE_SENSORS_ALIASES(i)), \
	(DEVICE_DT_GET(PRESSURE_SENSORS_ALIASES(i)),))

static const struct device *press_devs[] = {
	LISTIFY(2, PRESSURE_DEVICES, ())
};


struct press_values {
	int cnt;
	uint64_t ts[N_SAMPLES];
	double pressure[N_SAMPLES];
};

static int add_sample(struct press_values *psen, double pressure)
{
	if (psen->cnt == N_SAMPLES) {
		return -1;
	}

	psen->pressure[psen->cnt] = pressure;
	psen->ts[psen->cnt] = k_ticks_to_us_floor64(k_uptime_ticks());
	psen->cnt++;

	return 0;
}

static int print_flush_samples(struct press_values *psen_1,
			       struct press_values *psen_2)
{
	int i;
	printf("Sample,Ts1,Pressure1,Ts2,Pressure2\n");

	for (i = 0; i < N_SAMPLES; i++) {
		printf("%d, %lld, %f, %lld, %f\n", i,
		       psen_1->ts[i], psen_1->pressure[i],
		       psen_2->ts[i], psen_2->pressure[i]);
	}

	printf("[DONE]\n");

	psen_1->cnt = 0;
	psen_2->cnt = 0;

	return 0;
}

static void process_sample(const struct device *dev)
{
	static struct press_values psen_1;
	static struct press_values psen_2;
	struct press_values *psen;

	struct sensor_value tmp_pressure;
	struct sensor_value attr = {
		.val1 = 0,
	};
	//printk("Sample ready\n");

	if (sensor_sample_fetch(dev) < 0) {
		printf("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &tmp_pressure) < 0) {
		printf("Cannot read LPS28DFW pressure channel\n");
		return;
	}

	if (dev == press_devs[0]) {
		psen = &psen_1;
	} else {
		psen = &psen_2;
	}

	add_sample(psen, sensor_value_to_double(&tmp_pressure));

	if (psen->cnt == N_SAMPLES) {
		sensor_attr_set(dev, SENSOR_CHAN_ALL,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &attr);
	}
	if (psen_1.cnt == N_SAMPLES && psen_2.cnt == N_SAMPLES) {
		print_flush_samples(&psen_1, &psen_2);
	}
}

static void lps28dfw_handler(const struct device *dev,
			    const struct sensor_trigger *trig)
{
	process_sample(dev);
}

SHELL_SUBCMD_SET_CREATE(sub_section_get_samples, (pressure));

static int get_lps28dfw_samples(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct sensor_value attr = {
		.val1 = 200,
	};
	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};

	for (int i = 0; i < ARRAY_SIZE(press_devs); i++) {
		if (sensor_attr_set(press_devs[i], SENSOR_CHAN_ALL,
				    SENSOR_ATTR_SAMPLING_FREQUENCY, &attr) < 0) {
			printf("Cannot configure sampling rate\n");
			return 0;
		}
		if (sensor_trigger_set(press_devs[i], &trig, lps28dfw_handler) < 0) {
			printf("Cannot configure trigger\n");
			return 0;
		}
		printk("Configured for triggered collection at %u Hz\n",
		       attr.val1);
	}

	return 0;
}

/* Create a set of subcommands for "pressure cm1". */
SHELL_SUBCMD_SET_CREATE(sub_section_get_samples1, (pressure, get_samples));

/* Add command to the set. Subcommand set is identify by parent shell command. */
SHELL_SUBCMD_ADD((pressure), get_samples, &sub_section_get_samples1,
		  "Get 1000 samples from LPS28DFW",
		  get_lps28dfw_samples, 1, 0);

SHELL_CMD_REGISTER(pressure, &sub_section_get_samples,
		   "Commands for getting pressure samples", NULL);


int main(void)
{
	if (!device_is_ready(press_devs[0])) {
		printk("sensor: device 0 not ready.\n");
		return 0;
	}
	if (!device_is_ready(press_devs[1])) {
		printk("sensor: device 1 not ready.\n");
		return 0;
	}

	return 0;
}
