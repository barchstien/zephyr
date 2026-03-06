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

LOG_MODULE_REGISTER(VL53L8CX, CONFIG_SENSOR_LOG_LEVEL);


//////////////////////////////////////////////////////////////////////////

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

// debug
//static VL53L8CX_ResultsData result_data;

void vl53l8cx_submit_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	//LOG_INF("vl53l8cx_submit_sync");

	//uint32_t min_buf_len = 64 * 2 + 1; // TODO
	uint32_t min_buf_len = sizeof(VL53L8CX_ResultsData) + 1;
	uint8_t *buf;
	uint32_t buf_len;

	int ret = rtio_sqe_rx_buf(
		iodev_sqe, 
		min_buf_len, 
		min_buf_len, 
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

	//ret = vl53l8cx_get_ranging_data(&data->vl53l8cx_private_config, &result_data);
	buf[0] = data->num_of_zone;
	ret = vl53l8cx_get_ranging_data(&data->vl53l8cx_private_config, (VL53L8CX_ResultsData *)(buf + 1));
	// TODO also check status, etc
	// TODO add num of zone in 1st byte
	// Correct data
	//for (int i=0)
	//memcpy(buf, (uint8_t*)result_data.distance_mm, data->num_of_zone * sizeof(int16_t));
	//LOG_INF("%d Read data: %d %d %d %d", 
	//	ret, result_data.distance_mm[0], result_data.distance_mm[1], result_data.distance_mm[2], result_data.distance_mm[3]);

	// ??! TODO just copy VL53L8CX_ResultsData, coz RTI should no be processing data
	// it's 1360 bytes
	// that would avoid needing a static VL53L8CX_ResultsData

	//LOG_INF("vl53l8cx_submit_sync SQE OK");
	rtio_iodev_sqe_ok(iodev_sqe, 0);
	//LOG_INF("vl53l8cx_submit_sync END");
}

static void vl53l8cx_submit(const struct device *sensor, struct rtio_iodev_sqe *iodev_sqe)
{
	struct vl53l8cx_data *data = sensor->data;
#if 0
	/* Offload execution using the Zephyr standard RTIO workqueue request */
	struct rtio_work_req *req = rtio_work_req_alloc();

	if (req == NULL) {
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	/* Dispatch into the RTIO workqueue - the caller thread continues immediately */
	rtio_work_req_submit(req, iodev_sqe, vl53l8cx_submit_sync);
#else
	// TODO ?? check if already set ?? use atomic_ptr_cas ?
	//atomic_ptr_set(&data->pending_sqe, iodev_sqe);
	if (false == atomic_ptr_cas(&data->pending_sqe, NULL, iodev_sqe)) {
		LOG_WRN("SQE already pending");
	}
	else {
		//LOG_INF("SQE pending");
	}
#endif
}


#if 1
/*
 * DECODER: Splits raw buffer into 64 Q31 integers
 */
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
		*base_size = sizeof(VL53L8CX_ResultsData) + 1;
		*frame_size = sizeof(VL53L8CX_ResultsData) + 1;
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
	//const int32_t *raw_ints = (const int32_t *)buffer;
	//q31_t *out = (q31_t *)data_out;
//
	//if (*fit_idx >= SENSOR_NUM_SAMPLES) {
	//	return 0; /* No more data to decode */
	//}
//
	///* Convert Raw int32 to Q31 (Example: 1:1 mapping) */
	//for (int i = 0; i < max_count && *fit_idx < SENSOR_NUM_SAMPLES; i++) {
	//	out[i] = raw_ints[*fit_idx];
	//	(*fit_idx)++;
	//}
	// TODO check channels, distance, status, etc
//
	///* return number of samples written */
	//return (*fit_idx);
	// TODO, use the 1st byte to give the length
	//memcpy(data_out, buffer, 64*2);
	//memcpy(buf, (uint8_t*)result_data.distance_mm, data->num_of_zone * sizeof(int16_t));
	uint8_t zone_cnt = buffer[0];
	const VL53L8CX_ResultsData *result_data = (VL53L8CX_ResultsData*)(&buffer[1]);
	memcpy(data_out, (uint8_t*)result_data->distance_mm, zone_cnt * sizeof(int16_t));
	int16_t *distance_mm_out = (int16_t*)data_out;

	for (int i=0; i<zone_cnt; i++) {
		if (result_data->nb_target_detected[i] == 0) {
			//LOG_WRN("no target-------------");
			distance_mm_out[i] = 8000;
		}
		if (result_data->target_status[i] != 5 && result_data->target_status[i] != 9) {
			//LOG_WRN("bad status ===================");
			// if bad status, use max
			distance_mm_out[i] = 8000;
		}
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

#endif


static void vl53l8cx_rdy_callback(const struct device *port,
								  struct gpio_callback *cb,
								  uint32_t pins)
{
	//struct vl53l8cx_data *data = CONTAINER_OF(cb, struct vl53l8cx_data, gpio_cb);
	struct vl53l8cx_data *data = CONTAINER_OF(cb, struct vl53l8cx_data, rdy_cb);
	//atomic_ptr_t sqe = NULL;
	struct rtio_iodev_sqe *sqe = NULL;

	//LOG_INF("INT start");
	sqe = atomic_ptr_set(&data->pending_sqe, NULL);
	if (sqe != NULL) {
		struct rtio_work_req *req = rtio_work_req_alloc();
		if (req == NULL) {
			rtio_iodev_sqe_err(sqe, -ENOMEM);
			return;
		}
		rtio_work_req_submit(req, sqe, vl53l8cx_submit_sync);
	}
	
	//LOG_INF("INT read ready ! num_of_zone: %d", data->num_of_zone);
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

	data->pending_sqe = ATOMIC_PTR_INIT(NULL);

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