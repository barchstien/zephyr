/*
 * Copyright (c) 2026 Bastien Auneau <bastien.auneau@while-true.fr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "vl53l8cx_api.h"
#include "vl53l8cx_platform.h"

/**
 * Config for Zephyr Sensor Device
 * Used in SENSOR_DEVICE_DT_INST_DEFINE
 */
struct vl53l8cx_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec lpn;
	struct gpio_dt_spec pwr;
    struct gpio_dt_spec rdy;
};

/**
 * Private data to Zephyr Sensor Device
 * Used in SENSOR_DEVICE_DT_INST_DEFINE
 */
struct vl53l8cx_data {
    /**
     * STM defined driver structure that "contains the sensor configuration"
     * @warning "User MUST not manually change these field, except for the sensor address"
     * @see vl53l8cx_api.h
     */
    VL53L8CX_Configuration vl53l8cx_private_config;

    /**
     * Call back for INT, aka ready to read
     */
    struct gpio_callback rdy_cb;

    struct rtio_iodev_sqe *sqe;

    // Max 30Hz 4x4, max 15Hz 8x8
    // already in attributes
    //uint8_t sample_freq;
	
    // TODO, coz zephyr doc says that preferred flow is via interrupts
//#ifdef CONFIG_vl53l8cx_INTERRUPT_MODE
//	struct gpio_callback gpio_cb;
//	struct k_work work;
//	const struct device *dev;
//#endif
};
