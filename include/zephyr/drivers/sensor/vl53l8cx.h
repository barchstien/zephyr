/*
 * Copyright (c) 2026 Bastien Auneau <bastien.auneau@while-true.fr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_VL53L8CX_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_VL53L8CX_H_

#include <zephyr/drivers/sensor_data_types.h>
#include "vl53l8cx_api.h"

/** 
 * Minimum size required to read from HW
 * 1 byte (zone cnt 16/64) + 4 bytes timestamp msec + ST defined struct size
 * @see vl53l8cx_platform.h to reduce size of ST defined struct
 */
#define VL53L8CX_SENSOR_READ_BLOCK_SIZE (1 + 4 + sizeof(VL53L8CX_ResultsData))

#define SENSOR_ATTR_VL53L8CX_RESOLUTION_4X4 VL53L8CX_RESOLUTION_4X4
#define SENSOR_ATTR_VL53L8CX_RESOLUTION_8X8 VL53L8CX_RESOLUTION_8X8

/**
 * Custom sensor attributes
 */
enum sensor_attribute_vl53l8cx {
	SENSOR_ATTR_VL53L8CX_RANGING_MODE = SENSOR_ATTR_PRIV_START,
	SENSOR_ATTR_VL53L8CX_POWER_MODE,
	SENSOR_ATTR_VL53L8CX_TARGET_ORDER,
	SENSOR_ATTR_VL53L8CX_VHV_REPEAT_COUNT
};

#define SENSOR_ATTR_VL53L8CX_RANGING_AUTONOMOUS VL53L8CX_RANGING_MODE_AUTONOMOUS
#define SENSOR_ATTR_VL53L8CX_RANGING_CONTINUOUS VL53L8CX_RANGING_MODE_CONTINUOUS
#define SENSOR_ATTR_VL53L8CX_POWER_WAKE_UP VL53L8CX_POWER_MODE_WAKEUP
#define SENSOR_ATTR_VL53L8CX_POWER_SLEEP VL53L8CX_POWER_MODE_SLEEP
#define SENSOR_ATTR_VL53L8CX_POWER_DEEP_SLEEP VL53L8CX_POWER_MODE_DEEP_SLEEP
#define SENSOR_ATTR_VL53L8CX_TARGET_CLOSEST VL53L8CX_TARGET_ORDER_CLOSEST
#define SENSOR_ATTR_VL53L8CX_TARGET_STRONGEST VL53L8CX_TARGET_ORDER_STRONGEST

struct vl53l8cx_result_data {
	struct sensor_data_header header;
	int8_t shift;
	uint8_t resolution; // 16 or 64 zone
	struct vl53l8cx_result_sample_data {
        uint32_t timestamp_delta;
		q15_t distance[64]; 
	} readings[1]; 
} __attribute__((__packed__));

#endif // ZEPHYR_INCLUDE_DRIVERS_SENSOR_VL53L8CX_H_