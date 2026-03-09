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

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/rtio/work.h>
#include <zephyr/sys/atomic.h>

/**
 * 1 Bytes zone count (16 or 64)
 * 8 bytes timestamp
 * VL53L8CX_ResultsData ST made, can be reduced using defines
 * 						@see modules/has/st/sensor/vl53l8cx/api
 */
#define SENSOR_RAW_DATA_LEN (1 + 8 + sizeof(VL53L8CX_ResultsData))

LOG_MODULE_REGISTER(VL53L8CX, CONFIG_SENSOR_LOG_LEVEL);

static int vl53l8cx_attr_set(const struct device *dev,
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
			LOG_INF(" -- Stop ranging");
			vl53l8cx_stop_ranging(&data->vl53l8cx_private_config);
			// Disable continuous mode
			vl53l8cx_set_ranging_mode(
				&data->vl53l8cx_private_config,
				VL53L8CX_RANGING_MODE_AUTONOMOUS
			);
		} else {
			// set freq and enable continuous mode
			vl53l8cx_set_ranging_frequency_hz(
				&data->vl53l8cx_private_config,
				val->val1
			);
			vl53l8cx_set_ranging_mode(
				&data->vl53l8cx_private_config,
				VL53L8CX_RANGING_MODE_CONTINUOUS
			);
			LOG_INF(" -- Start ranging");
			vl53l8cx_start_ranging(&data->vl53l8cx_private_config);
		}

		LOG_INF("Sample freq set to %d Hz", val->val1);
		return 0;

	case SENSOR_ATTR_RESOLUTION:
		if (val->val1 == 16) {
			vl53l8cx_set_resolution (
				&data->vl53l8cx_private_config,
				VL53L8CX_RESOLUTION_4X4
			);
			data->num_of_zone = 16;
		} else if (val->val1 == 64) {
			vl53l8cx_set_resolution (
				&data->vl53l8cx_private_config,
				VL53L8CX_RESOLUTION_8X8
			);
			data->num_of_zone = 64;
		} else {
			return -EINVAL;
		}
		LOG_INF("Resolution set to %d zones", val->val1);
		return 0;

	default:
		return -ENOTSUP;
	}
}

static int vl53l8cx_attr_get(const struct device *dev,
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
		vl53l8cx_get_resolution (
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

void vl53l8cx_submit_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	uint8_t *buf;
	uint32_t buf_len;

	int ret = rtio_sqe_rx_buf(
		iodev_sqe, 
		SENSOR_RAW_DATA_LEN, 
		SENSOR_RAW_DATA_LEN, 
		&buf, 
		&buf_len
	);
	if (ret != 0) {
		LOG_ERR("Failed to allocate buffer from mempool");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}
	const struct sensor_read_config *read_cfg = iodev_sqe->sqe.iodev->data;
	const struct device *dev = read_cfg->sensor;
	struct vl53l8cx_data *data = dev->data;

	buf[0] = data->num_of_zone;
	*(uint32_t*)(&buf[1]) = (uint32_t)atomic_get(&data->last_interrupt_timepoint);
	ret = vl53l8cx_get_ranging_data(&data->vl53l8cx_private_config, (VL53L8CX_ResultsData *)(buf + 5));
	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

static void vl53l8cx_submit(const struct device *sensor, struct rtio_iodev_sqe *iodev_sqe)
{
	struct vl53l8cx_data *data = sensor->data;
	if (false == atomic_ptr_cas(&data->pending_sqe, NULL, iodev_sqe)) {
		LOG_WRN("SQE already pending");
	}
}

static int vl53l8cx_decoder_get_frame_count(const uint8_t *buffer,
		struct sensor_chan_spec channel,
		uint16_t *frame_count)
{
	// always receiving frames 1 by 1
	*frame_count = 1;
	return 0;
}

static int vl53l8cx_decoder_get_size_info(struct sensor_chan_spec chan_spec,
					size_t *base_size,
					size_t *frame_size)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_DISTANCE:
		// lokks like base is full data (+metadata), and frame the sample itself
		//*base_size = sizeof(struct sensor_three_axis_data);
		//*frame_size = sizeof(struct sensor_three_axis_sample_data);
		// ... use same for now
		*base_size = sizeof(VL53L8CX_ResultsData) + 1 + 4;
		*frame_size = sizeof(VL53L8CX_ResultsData) + 1 + 4;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int vl53l8cx_decoder_decode(const uint8_t *buffer,
								   struct sensor_chan_spec channel,
								   uint32_t *fit_count,
								   uint16_t max_count,
								   void *data_out)
{
	uint8_t zone_cnt = buffer[0];
	const VL53L8CX_ResultsData *result_data = (VL53L8CX_ResultsData*)(&buffer[5]);
	uint8_t *buffer_out = (uint8_t*)data_out;
	int16_t *distance_mm_out = (int16_t*)(buffer_out + 5);

	
	// zone count, 16 or 64
	buffer_out[0] = zone_cnt;
	// 32 bit timestamp
	memcpy(&buffer_out[1], &buffer[1], sizeof(int32_t));
	// distance per zone
	memcpy(buffer_out + 5, (uint8_t*)result_data->distance_mm, zone_cnt * sizeof(int16_t));
	// ... replace invalid data by max distance
	for (int i=0; i<zone_cnt; i++) {
		if (result_data->nb_target_detected[i] == 0) {
			distance_mm_out[i] = MAX_DISTANCE_MM;
		}
		if (result_data->target_status[i] != 5 && result_data->target_status[i] != 9) {
			distance_mm_out[i] = MAX_DISTANCE_MM;
		}
	}

	// TODO, change data in "normalised whatnot" thingy
	// Meaning scale it to use 4096 (2^12), which is easy coz value is between 0 and 4000 mm
	// Also the "shift" should be clarified
	// |--> is it about using whole q15 and shitdt by 3, coz 12 + 3 = 15 ?	

	*fit_count += sizeof(VL53L8CX_ResultsData);
	return 1;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = vl53l8cx_decoder_get_frame_count,
	.get_size_info = vl53l8cx_decoder_get_size_info,
	.decode = vl53l8cx_decoder_decode,
};

static int vl53l8cx_get_decoder(const struct device *dev,
				    const struct sensor_decoder_api **decoder)
{
	*decoder = &SENSOR_DECODER_NAME();
	return 0;
}

static const struct sensor_driver_api vl53l8cx_api_funcs = {
	.attr_set = vl53l8cx_attr_set,
	.attr_get = vl53l8cx_attr_get,
	.get_decoder = vl53l8cx_get_decoder,
	.submit = vl53l8cx_submit
};


static void vl53l8cx_rdy_callback(const struct device *port,
								  struct gpio_callback *cb,
								  uint32_t pins)
{
	struct vl53l8cx_data *data = CONTAINER_OF(cb, struct vl53l8cx_data, rdy_cb);
	struct rtio_iodev_sqe *sqe = NULL;

	atomic_set(&data->last_interrupt_timepoint, k_uptime_get_32());
	sqe = atomic_ptr_set(&data->pending_sqe, NULL);
	if (sqe != NULL) {
		struct rtio_work_req *req = rtio_work_req_alloc();
		if (req == NULL) {
			rtio_iodev_sqe_err(sqe, -ENOMEM);
			return;
		}
		rtio_work_req_submit(req, sqe, vl53l8cx_submit_sync);
	}
}


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
	uint8_t tmp_u8 = 0;

	LOG_INF("sizeof(VL53L8CX_ResultsData): %d", sizeof(VL53L8CX_ResultsData));

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
#if 1
	// GPIO rdy (read ready)
	if (! gpio_is_ready_dt(&config->rdy)) {
		LOG_ERR("GPIO port %s not ready", config->rdy.port->name);
		return ret;
	}
	ret = gpio_pin_configure_dt(&config->rdy, GPIO_INPUT); 
	if (ret != 0) {
		LOG_ERR("GPIO port %s failed to set as input: %i", config->rdy.port->name, ret);
		return ret;
	}
	gpio_init_callback(&data->rdy_cb, vl53l8cx_rdy_callback, BIT(config->rdy.pin));
	ret = gpio_add_callback(config->rdy.port, &data->rdy_cb);
	if (ret < 0) {
		LOG_ERR("Could not add gpio callback (%d)", ret);
		return ret;
	}
	ret = gpio_pin_interrupt_configure_dt(&config->rdy, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure interrupt trigger (%d)", ret);
		return ret;
	}
#endif
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
	// Takes 2 sec @ 400KHz
	ret = vl53l8cx_init(&data->vl53l8cx_private_config);
	if (ret != 0) {
		LOG_ERR("[%s] Failed to init", dev->name);
		return ret;
	}

	// get current resolution, required to extract results
	ret = vl53l8cx_get_resolution (
		&data->vl53l8cx_private_config,
		&tmp_u8
	);
	if (ret != 0) {
		LOG_ERR("[%s] Failed get resolution", dev->name);
		return ret;
	}
	data->num_of_zone = tmp_u8;

	// TODO test ranging mode
	// default is autonomous (good for pwoer), continuous (good for perf)
	ret = vl53l8cx_set_ranging_mode (
		&data->vl53l8cx_private_config,
		VL53L8CX_RANGING_MODE_CONTINUOUS
	);
	if (ret != 0) {
		LOG_ERR("[%s] Failed to set ranging mode", dev->name);
		return ret;
	}

	// repeat count to trigger temp realted calibration
	// takes few msec
	ret = vl53l8cx_set_VHV_repeat_count (
		&data->vl53l8cx_private_config,
		150 // TODO, make it better
	);
	if (ret != 0) {
		LOG_ERR("[%s] Failed to set ranging mode", dev->name);
		return ret;
	}

	// TODO test sharpener [0;99]%, default 1%
	//ret = vl53l8cx_set_sharpener_percent (
	//	&data->vl53l8cx_private_config,
	//	0
	//);
	//if (ret != 0) {
	//	LOG_ERR("[%s] Failed to set sharpener percent", dev->name);
	//	return ret;
	//}

	// TODO test 1st/strongest target
	// default is stronger, and it's recommended for indoor (why?)
	ret = vl53l8cx_set_target_order (
		&data->vl53l8cx_private_config,
		VL53L8CX_TARGET_ORDER_CLOSEST
	);
	if (ret != 0) {
		LOG_ERR("[%s] Failed to set target order", dev->name);
		return ret;
	}

	// TODO more ?

	LOG_INF("[%s] Initialized", dev->name);
	return 0;
}

#define VL53L8CX_INIT(i) \
	static const struct vl53l8cx_config vl53l8cx_config_##i = { \
		.i2c = I2C_DT_SPEC_INST_GET(i), \
		.lpn = GPIO_DT_SPEC_INST_GET_OR(i, lpn_gpios, { 0 }), \
		.pwr = GPIO_DT_SPEC_INST_GET_OR(i, pwr_gpios, { 0 }), \
		.rdy = GPIO_DT_SPEC_INST_GET_OR(i, rdy_gpios, { 0 }), \
	}; \
	\
	static struct vl53l8cx_data vl53l8cx_data_##i = { \
		.last_interrupt_timepoint = ATOMIC_INIT(0), \
		.pending_sqe = ATOMIC_PTR_INIT(NULL) \
	}; \
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