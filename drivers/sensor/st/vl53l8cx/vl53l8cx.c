/*
 * Copyright (c) 2026 Bastien Auneau <bastien.auneau@while-true.fr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_vl53l8cx

#include "vl53l8cx_api.h"
#include "vl53l8cx_platform.h"
#include "vl53l8cx_types.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(VL53L8CX, CONFIG_SENSOR_LOG_LEVEL);


static vl53l8cx_status_t vl53l8cx_read_sensor(struct vl53l8cx_data *drv_data)
{
	int ret;

	// TODO
	//vl53l8cx_get_ranging_data(...)

	//ret = VL53L1_GetRangingMeasurementData(&drv_data->vl53l8cx, &drv_data->data);
	//if (ret != VL53L1_ERROR_NONE) {
	//	LOG_ERR("VL53L1_GetRangingMeasurementData return error (%d)", ret);
	//	return ret;
	//}
//
	//ret = VL53L1_ClearInterruptAndStartMeasurement(&drv_data->vl53l8cx);
	//if (ret != VL53L1_ERROR_NONE) {
	//	LOG_ERR("VL53L1_ClearInterruptAndStartMeasurement return error (%d)", ret);
	//	return ret;
	//}

	return VL53L8CX_STATUS_OK;
}

#ifdef CONFIG_VL53L1X_INTERRUPT_MODE
static void vl53l1x_worker(struct k_work *work)
{
	struct vl53l1x_data *drv_data = CONTAINER_OF(work, struct vl53l1x_data, work);

	vl53l1x_read_sensor(drv_data);
}

static void vl53l1x_gpio_callback(const struct device *dev,
		struct gpio_callback *cb, uint32_t pins)
{
	struct vl53l1x_data *drv_data = CONTAINER_OF(cb, struct vl53l1x_data, gpio_cb);

	k_work_submit(&drv_data->work);
}

static int vl53l1x_init_interrupt(const struct device *dev)
{
	struct vl53l1x_data *drv_data = dev->data;
	const struct vl53l1x_config *config = dev->config;
	int ret;

	drv_data->dev = dev;

	if (!gpio_is_ready_dt(&config->gpio1)) {
		LOG_ERR("%s: device %s is not ready", dev->name, config->gpio1.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->gpio1, GPIO_INPUT | GPIO_PULL_UP);
	if (ret < 0) {
		LOG_ERR("[%s] Unable to configure GPIO interrupt", dev->name);
		return -EIO;
	}

	gpio_init_callback(&drv_data->gpio_cb,
					vl53l1x_gpio_callback,
					BIT(config->gpio1.pin));

	ret = gpio_add_callback(config->gpio1.port, &drv_data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to set gpio callback!");
		return -EIO;
	}

	drv_data->work.handler = vl53l1x_worker;

	return 0;
}
#endif

static int vl53l8cx_sample_fetch(const struct device *dev,
		enum sensor_channel chan)
{
#if 0
	struct vl53l8cx_data *drv_data = dev->data;
	VL53L1_Error ret;

	__ASSERT_NO_MSG((chan == SENSOR_CHAN_ALL)
			|| (chan == SENSOR_CHAN_DISTANCE));

	/* Will immediately stop current measurement */
	ret = VL53L1_StopMeasurement(&drv_data->vl53l8cx);
	if (ret != VL53L1_ERROR_NONE) {
		LOG_ERR("VL53L1_StopMeasurement return error (%d)", ret);
		return -EBUSY;
	}

#ifdef CONFIG_vl53l8cx_INTERRUPT_MODE
	const struct vl53l8cx_config *config = dev->config;

	ret = gpio_pin_interrupt_configure_dt(&config->gpio1, GPIO_INT_EDGE_TO_INACTIVE);
	if (ret < 0) {
		LOG_ERR("[%s] Unable to config interrupt", dev->name);
		return -EIO;
	}
#endif

	ret = VL53L1_StartMeasurement(&drv_data->vl53l8cx);
	if (ret != VL53L1_ERROR_NONE) {
		LOG_ERR("[%s] VL53L1_StartMeasurement return error (%d)", dev->name, ret);
		return -EBUSY;
	}
#endif
	return 0;
}

static int vl53l8cx_channel_get(const struct device *dev,
		enum sensor_channel chan,
		struct sensor_value *val)
{
#if 0
	struct vl53l8cx_data *drv_data = dev->data;
	VL53L1_Error ret;

	if (chan != SENSOR_CHAN_DISTANCE) {
		return -ENOTSUP;
	}

	/* Calling VL53L1_WaitMeasurementDataReady regardless of using interrupt or
	 * polling method ensures user does not have to consider the time between
	 * calling fetch and get.
	 */
	ret = VL53L1_WaitMeasurementDataReady(&drv_data->vl53l8cx);
	if (ret != VL53L1_ERROR_NONE) {
		LOG_ERR("[%s] VL53L1_WaitMeasurementDataReady return error (%d)", dev->name, ret);
		return -EBUSY;
	}

	if (IS_ENABLED(CONFIG_vl53l8cx_INTERRUPT_MODE) == 0) {
		/* Using driver poling mode */
		ret = vl53l8cx_read_sensor(drv_data);
		if (ret != VL53L1_ERROR_NONE) {
			return -ENODATA;
		}
	}

	val->val1 = (int32_t)(drv_data->data.RangeMilliMeter);
	/* RangeFractionalPart not implemented in API */
	val->val2 = 0;
#endif
	return 0;
}

static int vl53l8cx_attr_get(const struct device *dev,
		enum sensor_channel chan,
		enum sensor_attribute attr,
		struct sensor_value *val)
{
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_DISTANCE);

	int ret;
#if 0

	if (attr == SENSOR_ATTR_CONFIGURATION) {
		ret = vl53l8cx_get_mode(dev, val);
	} else if (attr == SENSOR_ATTR_CALIB_TARGET) {
		ret = vl53l8cx_get_roi(dev, val);
	} else {
		return -ENOTSUP;
	}
#endif
	return ret;
}

static int vl53l8cx_attr_set(const struct device *dev,
		enum sensor_channel chan,
		enum sensor_attribute attr,
		const struct sensor_value *val)
{
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_DISTANCE);

	int ret;
#if 0

	if (attr == SENSOR_ATTR_CONFIGURATION) {
		ret = vl53l8cx_set_mode(dev, val);
	} else if (attr == SENSOR_ATTR_CALIB_TARGET) {
		ret = vl53l8cx_set_roi(dev, val);
	} else {
		return -ENOTSUP;
	}
#endif
	return ret;
}

static DEVICE_API(sensor, vl53l8cx_api_funcs) = {
	.sample_fetch = vl53l8cx_sample_fetch,
	.channel_get = vl53l8cx_channel_get,
	.attr_get = vl53l8cx_attr_get,
	.attr_set = vl53l8cx_attr_set,
};

static int vl53l8cx_driver_init(const struct device *dev)
{
	const struct vl53l8cx_config *config = dev->config;
	struct vl53l8cx_data *data = dev->data;
	// STM vl53l8cx_platform.c requires GPIOs and I2C use
	//data->vl53l8cx_config.config = config;
	//data->vl53l8cx_config.platform.address = config->i2c.addr;
	data->vl53l8cx_config.platform.config = config;
	data->vl53l8cx_config.platform.address = config->i2c.addr;
	int ret = 0;
	
	// GPIO lpn
	if (!gpio_is_ready_dt(&config->lpn)) {
		LOG_ERR("GPIO port %s not ready", config->lpn.port->name);
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&config->lpn, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("GPIO port %s failed to set low", config->lpn.port->name);
		return -ENODEV;
	}
	//k_msleep(10);
	// GPIO pwr
	if (!gpio_is_ready_dt(&config->pwr)) {
		LOG_ERR("GPIO port %s not ready", config->pwr.port->name);
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&config->pwr, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("GPIO port %s failed to set low", config->pwr.port->name);
		return -ENODEV;
	}
	//k_msleep(10);
	// I2C
	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}

	ret = VL53L8CX_Reset_Sensor(&data->vl53l8cx_config.platform);
	if (ret != 0) {
		LOG_ERR("[%s] Failed to reset", dev->name);
		return ret;
	}
	LOG_INF("[%s] is reset", dev->name);

	// ST driver upload FW to HW
	ret = vl53l8cx_init(&data->vl53l8cx_config);
	if (ret != 0) {
		LOG_ERR("[%s] Failed to init", dev->name);
		return ret;
	}

	LOG_INF("[%s] Initialized", dev->name);
	return 0;
}


#define VL53L8CX_INIT(i) \
	static const struct vl53l8cx_config vl53l8cx_config_##i = { \
		.i2c = I2C_DT_SPEC_INST_GET(i), \
		.lpn = GPIO_DT_SPEC_INST_GET_OR(i, lpn_gpios, { 0 }), \
		.pwr = GPIO_DT_SPEC_INST_GET_OR(i, pwr_gpios, { 0 }), \
	}; \
	\
	static struct vl53l8cx_data vl53l8cx_data_##i; \
	\
	SENSOR_DEVICE_DT_INST_DEFINE(\
		i, \
		vl53l8cx_driver_init, \
		NULL, \
		&vl53l8cx_data_##i, \
		&vl53l8cx_config_##i, \
		POST_KERNEL, \
		CONFIG_SENSOR_INIT_PRIORITY, \
		&vl53l8cx_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(VL53L8CX_INIT)