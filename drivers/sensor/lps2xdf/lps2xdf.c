/* ST Microelectronics LPS2XDF pressure and temperature sensor
 *
 * Copyright (c) 2023 STMicroelectronics
 * Copyright (c) 2023 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lps22df.pdf
 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "lps2xdf.h"

#define LPS2XDF_SWRESET_WAIT_TIME_US 50

LOG_MODULE_REGISTER(LPS2XDF, CONFIG_SENSOR_LOG_LEVEL);

static const uint16_t lps2xdf_map[] = {0, 1, 4, 10, 25, 50, 75, 100, 200};

#if DT_HAS_COMPAT_STATUS_OKAY(st_lps22df)
static inline int lps22df_mode_set_odr_raw(const struct device *dev, uint8_t odr)
{
	const struct lps2xdf_config *const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps22df_md_t md;

	md.odr = odr;
	md.avg = cfg->avg;
	md.lpf = cfg->lpf;

	return lps22df_mode_set(ctx, &md);
}
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(st_lps28dfw)
static inline int lps28dfw_mode_set_odr_raw(const struct device *dev, uint8_t odr)
{
	const struct lps2xdf_config *const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps28dfw_md_t md;

	md.odr = odr;
	md.avg = cfg->avg;
	md.lpf = cfg->lpf;
	md.fs = cfg->fs;

	return lps28dfw_mode_set(ctx, &md);
}
#endif

static int lps2xdf_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;
	const struct lps2xdf_config *const cfg = dev->config;
	const struct lps2xdf_chip_api *chip_api = cfg->chip_api;

	for (odr = 0; odr < ARRAY_SIZE(lps2xdf_map); odr++) {
		if (freq == lps2xdf_map[odr]) {
			break;
		}
	}

	if (odr == ARRAY_SIZE(lps2xdf_map)) {
		LOG_DBG("bad frequency");
		return -EINVAL;
	}

	if (chip_api->mode_set_odr_raw(dev, odr)) {
		LOG_DBG("failed to set sampling rate");
		return -EIO;
	}

	return 0;
}

static int lps2xdf_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_ALL) {
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return lps2xdf_odr_set(dev, val->val1);
	default:
		LOG_DBG("operation not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static inline void lps2xdf_press_convert(const struct device *dev,
					 struct sensor_value *val,
					 int32_t raw_val)
{
	const struct lps2xdf_config *const cfg = dev->config;
	int32_t press_tmp = raw_val >> 8; /* raw value is left aligned (24 msb) */
	int divider;

	/* Pressure sensitivity is:
	 * - 4096 LSB/hPa for Full-Scale of 260 - 1260 hPa:
	 * - 2048 LSB/hPa for Full-Scale of 260 - 4060 hPa:
	 * Also convert hPa into kPa
	 */
	if (cfg->fs == 0) {
		divider = 40960;
	} else {
		divider = 20480;
	}
	val->val1 = press_tmp / divider;

	/* For the decimal part use (3125 / 128) as a factor instead of
	 * (1000000 / 40960) to avoid int32 overflow
	 */
	val->val2 = (press_tmp % divider) * 3125 / 128;
}


static inline void lps2xdf_temp_convert(struct sensor_value *val, int16_t raw_val)
{
	/* Temperature sensitivity is 100 LSB/deg C */
	val->val1 = raw_val / 100;
	val->val2 = ((int32_t)raw_val % 100) * 10000;
}

static int lps2xdf_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct lps2xdf_data *data = dev->data;

	if (chan == SENSOR_CHAN_PRESS) {
		lps2xdf_press_convert(dev, val, data->sample_press);
	} else if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		lps2xdf_temp_convert(val, data->sample_temp);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int lps2xdf_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct lps2xdf_config *const cfg = dev->config;
	const struct lps2xdf_chip_api *chip_api = cfg->chip_api;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	return chip_api->sample_fetch(dev, chan);
}

#if DT_HAS_COMPAT_STATUS_OKAY(st_lps22df)
static int lps22df_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct lps2xdf_data *data = dev->data;
	const struct lps2xdf_config *const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps22df_data_t raw_data;

	if (lps22df_data_get(ctx, &raw_data) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	data->sample_press = raw_data.pressure.raw;
	data->sample_temp = raw_data.heat.raw;

	return 0;
}
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(st_lps28dfw)
static int lps28dfw_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct lps2xdf_data *data = dev->data;
	const struct lps2xdf_config *const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps28dfw_data_t raw_data;
	lps28dfw_md_t md;

	md.fs = cfg->fs;

	if (lps28dfw_data_get(ctx, &md, &raw_data) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	data->sample_press = raw_data.pressure.raw;
	data->sample_temp = raw_data.heat.raw;

	return 0;
}
#endif

static const struct sensor_driver_api lps2xdf_driver_api = {
	.attr_set = lps2xdf_attr_set,
	.sample_fetch = lps2xdf_sample_fetch,
	.channel_get = lps2xdf_channel_get,
#if CONFIG_LPS2XDF_TRIGGER
	.trigger_set = lps2xdf_trigger_set,
#endif
};

#if DT_HAS_COMPAT_STATUS_OKAY(st_lps22df)
static const struct lps2xdf_chip_api st_lps22df_chip_api = {
	.mode_set_odr_raw = lps22df_mode_set_odr_raw,
	.sample_fetch = lps22df_sample_fetch,
#if CONFIG_LPS2XDF_TRIGGER
	.handle_interrupt = lps22df_handle_interrupt,
	.trigger_set = lps22df_trigger_set,
#endif
};
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(st_lps28dfw)
static const struct lps2xdf_chip_api st_lps28dfw_chip_api = {
	.mode_set_odr_raw = lps28dfw_mode_set_odr_raw,
	.sample_fetch = lps28dfw_sample_fetch,
#if CONFIG_LPS2XDF_TRIGGER
	.handle_interrupt = lps28dfw_handle_interrupt,
	.trigger_set = lps28dfw_trigger_set,
#endif
};
#endif


#if DT_HAS_COMPAT_STATUS_OKAY(st_lps22df)
static int st_lps22df_init(const struct device *dev)
{
	const struct lps2xdf_config *const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps22df_id_t id;
	lps22df_stat_t status;
	uint8_t tries = 10;
	int ret;

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lps22df, i3c)
	if (cfg->i3c.bus != NULL) {
		struct lps2xdf_data *data = dev->data;
		/*
		 * Need to grab the pointer to the I3C device descriptor
		 * before we can talk to the sensor.
		 */
		data->i3c_dev = i3c_device_find(cfg->i3c.bus, &cfg->i3c.dev_id);
		if (data->i3c_dev == NULL) {
			LOG_ERR("Cannot find I3C device descriptor");
			return -ENODEV;
		}
	}
#endif

	if (lps22df_id_get(ctx, &id) < 0) {
		LOG_ERR("%s: Not able to read dev id", dev->name);
		return -EIO;
	}

	if (id.whoami != LPS22DF_ID) {
		LOG_ERR("%s: Invalid chip ID 0x%02x", dev->name, id.whoami);
		return -EIO;
	}

	LOG_DBG("%s: chip id 0x%x", dev->name, id.whoami);

	/* Restore default configuration */
	if (lps22df_init_set(ctx, LPS22DF_RESET) < 0) {
		LOG_ERR("%s: Not able to reset device", dev->name);
		return -EIO;
	}

	do {
		if (!--tries) {
			LOG_DBG("sw reset timed out");
			return -ETIMEDOUT;
		}
		k_usleep(LPS2XDF_SWRESET_WAIT_TIME_US);

		if (lps22df_status_get(ctx, &status) < 0) {
			return -EIO;
		}
	} while (status.sw_reset);

	/* Set bdu and if_inc recommended for driver usage */
	if (lps22df_init_set(ctx, LPS22DF_DRV_RDY) < 0) {
		LOG_ERR("%s: Not able to set device to ready state", dev->name);
		return -EIO;
	}

	if (ON_I3C_BUS(cfg)) {
		lps22df_bus_mode_t bus_mode;

		/* Select bus interface */
		bus_mode.filter = LPS22DF_AUTO;
		bus_mode.interface = LPS22DF_SEL_BY_HW;
		lps22df_bus_mode_set(ctx, &bus_mode);
	}

	/* set sensor default odr */
	LOG_DBG("%s: odr: %d", dev->name, cfg->odr);
	ret = lps22df_mode_set_odr_raw(dev, cfg->odr);
	if (ret < 0) {
		LOG_ERR("%s: Failed to set odr %d", dev->name, cfg->odr);
		return ret;
	}

#ifdef CONFIG_LPS2XDF_TRIGGER
	if (lps2xdf_init_interrupt(dev, DEVICE_VARIANT_LPS22DF) < 0) {
		LOG_ERR("Failed to initialize interrupt.");
		return -EIO;
	}
#endif

	return 0;
}
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(st_lps28dfw)
static int st_lps28dfw_init(const struct device *dev)
{
	const struct lps2xdf_config *const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps28dfw_id_t id;
	lps28dfw_stat_t status;
	uint8_t tries = 10;
	int ret;

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lps28dfw, i3c)
	if (cfg->i3c.bus != NULL) {
		struct lps2xdf_data *data = dev->data;
		/*
		 * Need to grab the pointer to the I3C device descriptor
		 * before we can talk to the sensor.
		 */
		data->i3c_dev = i3c_device_find(cfg->i3c.bus, &cfg->i3c.dev_id);
		if (data->i3c_dev == NULL) {
			LOG_ERR("Cannot find I3C device descriptor");
			return -ENODEV;
		}
	}
#endif

	if (lps28dfw_id_get(ctx, &id) < 0) {
		LOG_ERR("%s: Not able to read dev id", dev->name);
		return -EIO;
	}

	if (id.whoami != LPS28DFW_ID) {
		LOG_ERR("%s: Invalid chip ID 0x%02x", dev->name, id.whoami);
		return -EIO;
	}

	LOG_DBG("%s: chip id 0x%x", dev->name, id.whoami);

	/* Restore default configuration */
	if (lps28dfw_init_set(ctx, LPS28DFW_RESET) < 0) {
		LOG_ERR("%s: Not able to reset device", dev->name);
		return -EIO;
	}

	do {
		if (!--tries) {
			LOG_DBG("sw reset timed out");
			return -ETIMEDOUT;
		}
		k_usleep(LPS2XDF_SWRESET_WAIT_TIME_US);

		if (lps28dfw_status_get(ctx, &status) < 0) {
			return -EIO;
		}
	} while (status.sw_reset);

	/* Set bdu and if_inc recommended for driver usage */
	if (lps28dfw_init_set(ctx, LPS28DFW_DRV_RDY) < 0) {
		LOG_ERR("%s: Not able to set device to ready state", dev->name);
		return -EIO;
	}

	if (ON_I3C_BUS(cfg)) {
		lps28dfw_bus_mode_t bus_mode;

		/* Select bus interface */
		bus_mode.filter = LPS28DFW_AUTO;
		bus_mode.interface = LPS28DFW_SEL_BY_HW;
		lps28dfw_bus_mode_set(ctx, &bus_mode);
	}

	/* set sensor default odr */
	LOG_DBG("%s: odr: %d", dev->name, cfg->odr);
	ret = lps28dfw_mode_set_odr_raw(dev, cfg->odr);
	if (ret < 0) {
		LOG_ERR("%s: Failed to set odr %d", dev->name, cfg->odr);
		return ret;
	}

#ifdef CONFIG_LPS2XDF_TRIGGER
	if (lps2xdf_init_interrupt(dev, DEVICE_VARIANT_LPS28DFW) < 0) {
		LOG_ERR("Failed to initialize interrupt.");
		return -EIO;
	}
#endif

	return 0;
}
#endif


#ifdef CONFIG_LPS2XDF_TRIGGER
#define LPS2XDF_CFG_IRQ(inst) .gpio_int = GPIO_DT_SPEC_INST_GET(inst, drdy_gpios),
#else
#define LPS2XDF_CFG_IRQ(inst)
#endif /* CONFIG_LPS2XDF_TRIGGER */

#define LPS2XDF_CONFIG_COMMON(inst, name)                                      \
	.odr = DT_INST_PROP(inst, odr),                                        \
	.lpf = DT_INST_PROP(inst, lpf),                                        \
	.avg = DT_INST_PROP(inst, avg),                                        \
	.drdy_pulsed = DT_INST_PROP(inst, drdy_pulsed),                        \
	.chip_api = &name##_chip_api,                                          \
	IF_ENABLED(DT_NODE_HAS_COMPAT(DT_DRV_INST(inst), st_lps28dfw),         \
		   (.fs = DT_INST_PROP(inst, fs),))                            \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, drdy_gpios),                    \
		   (LPS2XDF_CFG_IRQ(inst)))

#define LPS2XDF_SPI_OPERATION (SPI_WORD_SET(8) | SPI_OP_MODE_MASTER |          \
			       SPI_MODE_CPOL | SPI_MODE_CPHA)

#define LPS2XDF_CONFIG_SPI(inst, name)                                         \
{                                                                              \
	STMEMSC_CTX_SPI(&lps2xdf_config_##name##_##inst.stmemsc_cfg),          \
	.stmemsc_cfg = {                                                       \
		.spi = SPI_DT_SPEC_INST_GET(inst, LPS2XDF_SPI_OPERATION, 0),   \
	},                                                                     \
	LPS2XDF_CONFIG_COMMON(inst, name)                                      \
}

#define LPS2XDF_CONFIG_I2C(inst, name)                                         \
{                                                                              \
	STMEMSC_CTX_I2C(&lps2xdf_config_##name##_##inst.stmemsc_cfg),          \
	.stmemsc_cfg = {                                                       \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                             \
	},                                                                     \
	LPS2XDF_CONFIG_COMMON(inst, name)                                      \
}

#define LPS2XDF_CONFIG_I3C(inst, name)                                         \
{                                                                              \
	STMEMSC_CTX_I3C(&lps2xdf_config_##name##_##inst.stmemsc_cfg),          \
	.stmemsc_cfg = {                                                       \
		.i3c = &lps2xdf_data_##name##_##inst.i3c_dev,                  \
	},                                                                     \
	.i3c.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),                           \
	.i3c.dev_id = I3C_DEVICE_ID_DT_INST(inst),                             \
	LPS2XDF_CONFIG_COMMON(inst, name)                                      \
}

#define LPS2XDF_CONFIG_I3C_OR_I2C(inst, name)                                  \
	COND_CODE_0(DT_INST_PROP_BY_IDX(inst, reg, 1),                         \
		    (LPS2XDF_CONFIG_I2C(inst, name)),                          \
		    (LPS2XDF_CONFIG_I3C(inst, name)))

#define LPS2XDF_DEFINE(inst, name)                                                           \
	static struct lps2xdf_data lps2xdf_data_##name##_##inst;                             \
	static const struct lps2xdf_config lps2xdf_config_##name##_##inst = COND_CODE_1(     \
		DT_INST_ON_BUS(inst, spi),                                                   \
		(LPS2XDF_CONFIG_SPI(inst, name)),                                            \
		(COND_CODE_1(DT_INST_ON_BUS(inst, i3c),                                      \
			     (LPS2XDF_CONFIG_I3C_OR_I2C(inst, name)),                        \
			     (LPS2XDF_CONFIG_I2C(inst, name)))));                            \
											     \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, name##_init, NULL, &lps2xdf_data_##name##_##inst, \
				     &lps2xdf_config_##name##_##inst, POST_KERNEL,           \
				     CONFIG_SENSOR_INIT_PRIORITY, &lps2xdf_driver_api);

#define DT_DRV_COMPAT st_lps22df
DT_INST_FOREACH_STATUS_OKAY_VARGS(LPS2XDF_DEFINE, DT_DRV_COMPAT)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT st_lps28dfw
DT_INST_FOREACH_STATUS_OKAY_VARGS(LPS2XDF_DEFINE, DT_DRV_COMPAT)
#undef DT_DRV_COMPAT
