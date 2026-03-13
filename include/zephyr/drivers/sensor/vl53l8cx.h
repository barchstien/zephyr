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
 * 1 byte (num of zone) + 4 bytes timestamp msec + ST defined struct size
 * @see vl53l8cx_platform.h to reduce size of ST defined struct
 */
#define VL53L8CX_SENSOR_READ_BLOCK_SIZE (1 + 4 + sizeof(VL53L8CX_ResultsData))

// TODO add attributes:
//  - autonomous vs continuous mode
//  - power modes
//  - target order

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