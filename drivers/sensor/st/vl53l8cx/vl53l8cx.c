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
#include <zephyr/drivers/sensor/vl53l8cx.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/rtio/work.h>
#include <zephyr/sys/atomic.h>

/**
 * Intermediate format, input of decoder:
 * 1 Bytes zone count (16 or 64)
 * 8 bytes timestamp
 * VL53L8CX_ResultsData ST made, can be reduced using defines
 * 						@see modules/has/st/sensor/vl53l8cx/api
 */
#define SENSOR_RAW_DATA_LEN (1 + 8 + sizeof(VL53L8CX_ResultsData))

#define MAX_DISTANCE_M 4

LOG_MODULE_REGISTER(vl53l8cx, CONFIG_SENSOR_LOG_LEVEL);

static int vl53l8cx_attr_set(const struct device *dev,
							 enum sensor_channel chan,
							 enum sensor_attribute attr,
							 const struct sensor_value *val)
{
	struct vl53l8cx_inst_data *data = dev->data;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_DISTANCE) {
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (val->val1 < 1 || val->val1 > 60) {
			LOG_ERR("Sample freq shoudl be within [0; 60] for 4x4, or [0; 15] for 8x8. Got: %d Hz", val->val1);
			return -EINVAL;
		}
		vl53l8cx_set_ranging_frequency_hz(
			&data->vl53l8cx_private_config,
			val->val1
		);
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
	struct vl53l8cx_inst_data *data = dev->data;
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
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct device *dev = cfg->sensor;
	struct vl53l8cx_inst_data *data = dev->data;


	if (FIELD_GET(RTIO_SQE_CANCELED, iodev_sqe->sqe.flags)) {
		atomic_ptr_set(&data->pending_sqe, NULL);
		vl53l8cx_stop_ranging(&data->vl53l8cx_private_config);
		data->is_streaming = false;
		rtio_iodev_sqe_err(iodev_sqe, -ECANCELED);
		return;
	}

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

	buf[0] = data->num_of_zone;
	*(uint32_t*)(&buf[1]) = (uint32_t)atomic_get(&data->last_interrupt_timestamp);
	ret = vl53l8cx_get_ranging_data(&data->vl53l8cx_private_config, (VL53L8CX_ResultsData *)(buf + 5));
	rtio_iodev_sqe_ok(iodev_sqe, 0);

	// Stop ranging if it was a single shot
	if (!cfg->is_streaming) {
		vl53l8cx_stop_ranging(&data->vl53l8cx_private_config);
	}
}

static void vl53l8cx_submit(const struct device *sensor, struct rtio_iodev_sqe *iodev_sqe)
{
	struct vl53l8cx_inst_data *data = sensor->data;
    const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;

	// Set pending sqe if it's not already set
	// rtio submit happens in "data ready" interrupt handler
	if (false == atomic_ptr_cas(&data->pending_sqe, NULL, iodev_sqe)) {
		if (cfg->is_streaming) {
			LOG_WRN("SQE already streaming");
		}
		else {
			LOG_WRN("SQE already pending");
		}
		rtio_iodev_sqe_err(iodev_sqe, -EBUSY);
		return;
	}

	// stream start
	if (cfg->is_streaming && !data->is_streaming) {
		LOG_INF(" -- Start ranging stream");
		vl53l8cx_start_ranging(&data->vl53l8cx_private_config);
		data->is_streaming = true;
	}
	// one shot start
	if (!cfg->is_streaming) {
		vl53l8cx_start_ranging(&data->vl53l8cx_private_config);
	}
}

static int vl53l8cx_decoder_get_frame_count(const uint8_t *buffer,
		struct sensor_chan_spec channel,
		uint16_t *frame_count)
{
	// always receive frames 1 by 1
	*frame_count = 1;
	return 0;
}

static int vl53l8cx_decoder_get_size_info(struct sensor_chan_spec chan_spec,
					size_t *base_size,
					size_t *frame_size)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_DISTANCE:
		*base_size = sizeof(struct vl53l8cx_result_data);
		*frame_size = sizeof(struct vl53l8cx_result_data);
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
	// input
	const uint8_t zone_cnt = buffer[0];
	const uint32_t timestamp_ms = *(uint32_t*)(&buffer[1]);
	VL53L8CX_ResultsData *in_result = (VL53L8CX_ResultsData*)(&buffer[5]);
	// output
	struct vl53l8cx_result_data *out_result = (struct vl53l8cx_result_data*)data_out;

	out_result->header.base_timestamp_ns = 1e6 * (uint64_t)timestamp_ms;
	out_result->header.reading_count = 1;
	out_result->resolution = zone_cnt;
	out_result->readings[0].timestamp_delta = 0;

	// target distance is within [0; 4] meters, 
	// use double [0; 8], ie [0; 2^3]
	out_result->shift = 3;
	const int32_t anti_shift = 15 - out_result->shift;
	for (int i=0; i<zone_cnt; i++) {
		// replace invalid data by max distance
		if (in_result->nb_target_detected[i] == 0
			|| (in_result->target_status[i] != 5 && in_result->target_status[i] != 9)
		) {
			out_result->readings[0].distance[i] = MAX_DISTANCE_M << anti_shift;
			continue;
		}
		const int32_t normalised_mm = ((int32_t)in_result->distance_mm[i]) << anti_shift;
		// convert to meter
		out_result->readings[0].distance[i] = normalised_mm / 1000;
	}

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


static void vl53l8cx_rdy_callback(const struct device *dev,
								  struct gpio_callback *cb,
								  uint32_t pins)
{
	struct vl53l8cx_inst_data *data = CONTAINER_OF(cb, struct vl53l8cx_inst_data, rdy_cb);
	struct rtio_iodev_sqe *sqe = NULL;

	atomic_set(&data->last_interrupt_timestamp, k_uptime_get_32());
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
	struct vl53l8cx_inst_data *data = dev->data;
	// STM vl53l8cx_platform.c requires GPIOs and I2C use
	data->vl53l8cx_private_config.platform.config = config;
	data->vl53l8cx_private_config.platform.address = config->i2c.addr;
	int ret = 0;
	uint8_t tmp_u8 = 0;

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
	// Takes 2 sec with i2c at 400KHz
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
	// default is autonomous (good for power), continuous (good for perf)
	ret = vl53l8cx_set_ranging_mode (
		&data->vl53l8cx_private_config,
		VL53L8CX_RANGING_MODE_CONTINUOUS
	);
	if (ret != 0) {
		LOG_ERR("[%s] Failed to set ranging mode", dev->name);
		return ret;
	}

	// repeat count to trigger temp calibration "takes few msec"
	ret = vl53l8cx_set_VHV_repeat_count (
		&data->vl53l8cx_private_config,
		15 * 60 // 1min at 15Hz
	);
	if (ret != 0) {
		LOG_ERR("[%s] Failed to set VHV repeat count mode", dev->name);
		return ret;
	}

	LOG_INF("[%s] Initialized", dev->name);
	return 0;
}

#define VL53L8CX_INIT(i) \
	static const struct vl53l8cx_config vl53l8cx_config_##i = { \
		.i2c = I2C_DT_SPEC_INST_GET(i), \
		.lpn = GPIO_DT_SPEC_INST_GET(i, lpn_gpios), \
		.pwr = GPIO_DT_SPEC_INST_GET(i, pwr_gpios), \
		.rdy = GPIO_DT_SPEC_INST_GET_OR(i, rdy_gpios, { 0 }), \
	}; \
	\
	static struct vl53l8cx_inst_data vl53l8cx_inst_data_##i = { \
		.last_interrupt_timestamp = ATOMIC_INIT(0), \
		.pending_sqe = ATOMIC_PTR_INIT(NULL), \
		.is_streaming = false \
	}; \
	\
	SENSOR_DEVICE_DT_INST_DEFINE(\
		i, \
		vl53l8cx_driver_init, \
		NULL, \
		&vl53l8cx_inst_data_##i, \
		&vl53l8cx_config_##i, \
		POST_KERNEL, \
		CONFIG_SENSOR_INIT_PRIORITY, \
		&vl53l8cx_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(VL53L8CX_INIT)