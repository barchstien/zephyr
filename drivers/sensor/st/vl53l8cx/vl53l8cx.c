/*
 * Copyright (c) 2026 Bastien Auneau <bastien.auneau@while-true.fr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_vl53l8cx

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

#include "vl53l8cx_api.h"
#include "vl53l8cx_platform.h"

LOG_MODULE_REGISTER(VL53L8CX, CONFIG_SENSOR_LOG_LEVEL);

struct vl53l8cx_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec lpn;
	struct gpio_dt_spec pwr;
};

struct vl53l8cx_data {
	VL53L8CX_Platform vl53l8cx;
	//VL53L1_RangingMeasurementData_t data;
	//VL53L1_DistanceModes distance_mode;

    // TODO, coz zephyr doc says that preferred flow is via interrupts
//#ifdef CONFIG_vl53l8cx_INTERRUPT_MODE
//	struct gpio_callback gpio_cb;
//	struct k_work work;
//	const struct device *dev;
//#endif
};

typedef int vl53l8cx_status_t;

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

static vl53l8cx_status_t vl53l8cx_initialize(const struct device *dev)
{
#if 0
	struct vl53l8cx_data *drv_data = dev->data;
	vl53l8cx_status_t ret;
	VL53L1_DeviceInfo_t vl53l8cx_dev_info;

	LOG_DBG("[%s] Initializing ", dev->name);

	/* Pull XSHUT high to start the sensor */
#ifdef CONFIG_vl53l8cx_XSHUT
	const struct vl53l8cx_config *const config = dev->config;

	if (config->xshut.port) {
		int gpio_ret = gpio_pin_set_dt(&config->xshut, 1);

		if (gpio_ret < 0) {
			LOG_ERR("[%s] Unable to set XSHUT gpio (error %d)", dev->name, gpio_ret);
			return -EIO;
		}
		/* Boot duration is 1.2 ms max */
		k_sleep(K_MSEC(2));
	}
#endif

	/* ONE TIME device initialization.
	 * To be called ONLY ONCE after device is brought out of reset
	 */
	ret = VL53L1_DataInit(&drv_data->vl53l8cx);
	if (ret != VL53L1_ERROR_NONE) {
		LOG_ERR("[%s] vl53l8cx_DataInit return error (%d)", dev->name, ret);
		return -ENOTSUP;
	}

	/* Do basic device init */
	ret = VL53L1_StaticInit(&drv_data->vl53l8cx);
	if (ret != VL53L1_ERROR_NONE) {
		LOG_ERR("[%s] VL53L1_StaticInit return error (%d)", dev->name, ret);
		return -ENOTSUP;
	}

	/* Get info from sensor */
	(void)memset(&vl53l8cx_dev_info, 0, sizeof(VL53L1_DeviceInfo_t));

	ret = VL53L1_GetDeviceInfo(&drv_data->vl53l8cx, &vl53l8cx_dev_info);
	if (ret != VL53L1_ERROR_NONE) {
		LOG_ERR("[%s] VL53L1_GetDeviceInfo return error (%d)", dev->name, ret);
		return -ENODEV;
	}

	LOG_DBG("[%s] vl53l8cx_GetDeviceInfo returned %d", dev->name, ret);
	LOG_DBG("   Device Name : %s", vl53l8cx_dev_info.Name);
	LOG_DBG("   Device Type : %s", vl53l8cx_dev_info.Type);
	LOG_DBG("   Device ID : %s", vl53l8cx_dev_info.ProductId);
	LOG_DBG("   ProductRevisionMajor : %d", vl53l8cx_dev_info.ProductRevisionMajor);
	LOG_DBG("   ProductRevisionMinor : %d", vl53l8cx_dev_info.ProductRevisionMinor);

	/* Set default distance mode */
	drv_data->distance_mode = VL53L1_DISTANCEMODE_LONG;
	ret = VL53L1_SetDistanceMode(&drv_data->vl53l8cx, drv_data->distance_mode);
	if (ret != VL53L1_ERROR_NONE) {
		LOG_ERR("[%s] VL53L1_SetDistanceMode return error (%d)", dev->name, ret);
		return -EINVAL;
	}
#endif
	return 0;
}

/* Mapping is 1:1 with the API.
 * From VL531X datasheet:
 *          | Max distance  | Max distance in
 *  Mode    | in dark (cm)  | strong ambient light (cm)
 * ----------------------------------------------------
 * short    | 136           | 135
 * medium   | 290           | 76
 * long     | 360           | 73
 */
static int vl53l8cx_set_mode(const struct device *dev,
		const struct sensor_value *val)
{
#if 0
	struct vl53l8cx_data *drv_data = dev->data;
	VL53L1_Error ret;

	switch (val->val1) {
	/* short */
	case 1:
	/* medium */
	case 2:
	/* long */
	case 3:
		drv_data->distance_mode = val->val1;
		break;
	default:
		drv_data->distance_mode = VL53L1_DISTANCEMODE_LONG;
		break;
	}

	ret = VL53L1_SetDistanceMode(&drv_data->vl53l8cx, drv_data->distance_mode);
	if (ret != VL53L1_ERROR_NONE) {
		LOG_ERR("[%s] VL53L1_SetDistanceMode return error (%d)", dev->name, ret);
		return -EINVAL;
	}
#endif
	return 0;
}

/*
 * The ROI is a 16x16 grid.
 * The bottom left is (0,0), top right is (15, 15), for
 * a total of 256 squares (numbered 0 through 255).
 * The default ROI is val1 = 240, val2 = 15 (the full grid).
 * See UM2356 User Manual (VL531X API doc).
 */
static int vl53l8cx_set_roi(const struct device *dev,
		const struct sensor_value *val)
{
#if 0
	struct vl53l8cx_data *drv_data = dev->data;
	VL53L1_Error ret;

	if ((val->val1 < 0) ||
	    (val->val2 < 0) ||
	    (val->val1 > 255) ||
	    (val->val2 > 255) ||
	    (val->val2 >= val->val1)) {
		return -EINVAL;
	}

	/* Map val to pUserROi */
	VL53L1_UserRoi_t pUserROi = {
		.TopLeftX = val->val1 % 16,
		.TopLeftY = (uint8_t)(val->val1 / 16),
		.BotRightX = val->val2 % 16,
		.BotRightY = (uint8_t)(val->val2 / 16),
	};

	ret = VL53L1_SetUserROI(&drv_data->vl53l8cx, &pUserROi);
	if (ret != VL53L1_ERROR_NONE) {
		LOG_ERR("[%s] VL53L1_SetUserROI return error (%d)", dev->name, ret);
		return -EINVAL;
	}
#endif
	return 0;
}

static int vl53l8cx_get_mode(const struct device *dev,
		struct sensor_value *val)
{
#if 0
	struct vl53l8cx_data *drv_data = dev->data;
	VL53L1_DistanceModes mode;
	VL53L1_Error ret;

	ret = VL53L1_GetDistanceMode(&drv_data->vl53l8cx, &mode);
	if (ret != VL53L1_ERROR_NONE) {
		LOG_ERR("[%s] VL53L1_GetDistanceMode return error (%d)", dev->name, ret);
		return -ENODATA;
	}

	/* Mapping is 1:1 with the API */
	val->val1 = (int32_t)mode;
	val->val2 = 0;
#endif
	return 0;
}

static int vl53l8cx_get_roi(const struct device *dev,
		struct sensor_value *val)
{
#if 0
	struct vl53l8cx_data *drv_data = dev->data;
	VL53L1_Error ret;
	VL53L1_UserRoi_t pUserROi;

	ret = VL53L1_GetUserROI(&drv_data->vl53l8cx, &pUserROi);
	if (ret != VL53L1_ERROR_NONE) {
		LOG_ERR("[%s] VL53L1_GetUserROI return error (%d)", dev->name, ret);
		return -ENODATA;
	}

	/* Map pUserROi to val */
	val->val1 = (int32_t)((16 * pUserROi.TopLeftY) + pUserROi.TopLeftX);
	val->val2 = (int32_t)((16 * pUserROi.BotRightY) + pUserROi.BotRightX);
#endif
	return 0;
}

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
#if 0
	int ret = 0;
	struct vl53l8cx_data *drv_data = dev->data;
	const struct vl53l8cx_config *config = dev->config;

	/* Initialize the HAL i2c peripheral */
	drv_data->vl53l8cx.i2c = &config->i2c;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}

	/* Configure gpio connected to vl53l8cx's XSHUT pin to
	 * allow deepest sleep mode
	 */
#ifdef CONFIG_vl53l8cx_XSHUT
		if (config->xshut.port) {
			ret = gpio_pin_configure_dt(&config->xshut, GPIO_OUTPUT);
			if (ret < 0) {
				LOG_ERR("[%s] Unable to configure GPIO as output", dev->name);
				return -EIO;
			}
		}
#endif

#ifdef CONFIG_vl53l8cx_INTERRUPT_MODE
		if (config->gpio1.port) {
			ret = vl53l8cx_init_interrupt(dev);
			if (ret < 0) {
				LOG_ERR("Failed to initialize interrupt!");
				return -EIO;
			}
		}
#endif

	ret = vl53l8cx_initialize(dev);
	if (ret) {
		return ret;
	}

	LOG_DBG("[%s] Initialized", dev->name);
#endif
	return 0;
}




#define VL53L8CX_INIT(i) \
	static const struct vl53l8cx_config vl53l8cx_config_##i = { \
		.i2c = I2C_DT_SPEC_INST_GET(i), \
		IF_ENABLED(CONFIG_vl53l8cx_XSHUT, ( \
		.lpn = GPIO_DT_SPEC_INST_GET_OR(i, lpn_gpios, { 0 }),)) \
		IF_ENABLED(CONFIG_vl53l8cx_INTERRUPT_MODE, ( \
		.pwr = GPIO_DT_SPEC_INST_GET_OR(i, pwr_gpios, { 0 }),)) \
	}; \
	\
	static struct vl53l8cx_data vl53l8cx_data_##i; \
	\
	SENSOR_DEVICE_DT_INST_DEFINE(i, \
				     vl53l8cx_driver_init, \
				     NULL, \
				     &vl53l8cx_data_##i, \
				     &vl53l8cx_config_##i, \
				     POST_KERNEL, \
				     CONFIG_SENSOR_INIT_PRIORITY, \
				     &vl53l8cx_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(VL53L8CX_INIT)