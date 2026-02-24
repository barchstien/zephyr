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


//////////////////////////////////////////////////////////////////////////

static int vl53l8cx_attr_set(
	const struct device *dev,
    enum sensor_channel chan,
    enum sensor_attribute attr,
    const struct sensor_value *val)
{
    struct vl53l8cx_data *data = dev->data;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_DISTANCE) {
        return -ENOTSUP;
    }

    switch (attr) {
        case SENSOR_ATTR_SAMPLING_FREQUENCY:
            if (val->val1 < 0 || val->val1 > 30) {
                LOG_ERR("Expect sample freq in [0; 30] but got: %d Hz", val->val1);
                return -EINVAL;
            }

			if (val->val1 == 0) {
				// Disable continuous mode
				vl53l8cx_set_ranging_mode(
					&data->vl53l8cx_private_config,
					VL53L8CX_RANGING_MODE_AUTONOMOUS
				);
			}
			else {
				// set freq and enable continuous mode
				vl53l8cx_set_ranging_frequency_hz(
					&data->vl53l8cx_private_config,
					val->val1
				);
				vl53l8cx_set_ranging_mode(
					&data->vl53l8cx_private_config,
					VL53L8CX_RANGING_MODE_CONTINUOUS
				);
			}

            LOG_INF("Sample freq set to %d Hz", val->val1);
            return 0;

        case SENSOR_ATTR_RESOLUTION:
			if (val->val1 == 16) {
				vl53l8cx_set_resolution (
					&data->vl53l8cx_private_config,
					VL53L8CX_RESOLUTION_4X4
				);
			}
			else if (val->val1 == 64) {
				vl53l8cx_set_resolution (
					&data->vl53l8cx_private_config,
					VL53L8CX_RESOLUTION_8X8
				);
			}
			else {
                return -EINVAL;
            }
            LOG_INF("Resolution set to %d zones", val->val1);
            return 0;

        default:
            return -ENOTSUP;
    }
}

static int vl53l8cx_attr_get(
	const struct device *dev,
    enum sensor_channel chan,
    enum sensor_attribute attr,
    struct sensor_value *val)
{
    struct vl53l8cx_data *data = dev->data;
	uint8_t tmp_u8;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_DISTANCE) {
        return -ENOTSUP;
    }

    switch (attr) {
        case SENSOR_ATTR_SAMPLING_FREQUENCY:
            vl53l8cx_get_ranging_frequency_hz(
				&data->vl53l8cx_private_config,
				&tmp_u8
			);
            val->val1 = tmp_u8;
            val->val2 = 0;
            return 0;

		case SENSOR_ATTR_RESOLUTION:
			vl53l8cx_set_resolution (
				&data->vl53l8cx_private_config,
				&tmp_u8
			);
            val->val1 = tmp_u8;
            val->val2 = 0;
            return 0;

        default:
            return -ENOTSUP;
    }
}

static const struct sensor_driver_api vl53l8cx_api_funcs = {
    .attr_set = vl53l8cx_attr_set,
    .attr_get = vl53l8cx_attr_get
};

// TODO also add sensor_decoder_api
//static const struct sensor_decoder_api my_decoder = {
//    .decode = my_decode,
//    .get_frame_count = ... // optional
//    .get_size_info = ...   // optional
//};

/////////////////////////////

//static int vl53l8cx_sensor_read()
//{
//	int ret;
//
//	// TODO
//	LOG_INF("BLoooooP");
//
//	return 0;
//}

//static DEVICE_API(sensor, vl53l8cx_api_funcs) = {
//	.sensor_read = vl53l8cx_sensor_read,
//	//.sensor_decode = vl53l8cx_sensor_decode
//};

static int vl53l8cx_driver_init(const struct device *dev)
{
	const struct vl53l8cx_config *config = dev->config;
	struct vl53l8cx_data *data = dev->data;
	// STM vl53l8cx_platform.c requires GPIOs and I2C use
	//data->vl53l8cx_config.config = config;
	//data->vl53l8cx_config.platform.address = config->i2c.addr;
	data->vl53l8cx_private_config.platform.config = config;
	data->vl53l8cx_private_config.platform.address = config->i2c.addr;
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

	ret = VL53L8CX_Reset_Sensor(&data->vl53l8cx_private_config.platform);
	if (ret != 0) {
		LOG_ERR("[%s] Failed to reset", dev->name);
		return ret;
	}
	LOG_INF("[%s] is reset", dev->name);

	// ST driver upload FW to HW
	ret = vl53l8cx_init(&data->vl53l8cx_private_config);
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