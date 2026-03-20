/*
 * Copyright (c) 2026 Bastien Auneau <bastien.auneau@while-true.fr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/dsp/types.h>

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
struct vl53l8cx_inst_data {
    /**
     * STM defined driver structure that "contains the sensor configuration"
     * @warning "User MUST not manually change these field, except for the sensor address"
     * @see vl53l8cx_api.h
     */
    VL53L8CX_Configuration vl53l8cx_private_config;

    uint8_t num_of_zone;

    /** Submitted upon interrupt */
    atomic_ptr_t pending_sqe;

    /** Taken on interrupt, to stamp incoming smaple */
    atomic_t last_interrupt_timestamp;

    bool is_streaming;

#ifdef CONFIG_VL53L8CX_INTERRUPT
    /** Call back for INT, aka ready to read */
    struct gpio_callback rdy_cb;
#else
    struct k_work_delayable ready_poll_work;
#endif
};
