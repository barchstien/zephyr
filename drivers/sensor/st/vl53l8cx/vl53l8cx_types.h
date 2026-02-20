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
};

/**
 * Private data to Zephyr Sensor Device
 * Used in SENSOR_DEVICE_DT_INST_DEFINE
 */
struct vl53l8cx_data {
    /**
     * STM defined driver structure to "be filled with customer's platform"
     * Used for platform dependent i2c and gpio calls
     * @see vl53l8cx_platform.h
     */
	//VL53L8CX_Platform vl53l8cx_platform; <---- already part of VL53L8CX_Configuration
    
    /**
     * STM defined driver structure that "contains the sensor configuration"
     * @see vl53l8cx_api.h
     */
    VL53L8CX_Configuration vl53l8cx_config;
	
    // TODO, coz zephyr doc says that preferred flow is via interrupts
//#ifdef CONFIG_vl53l8cx_INTERRUPT_MODE
//	struct gpio_callback gpio_cb;
//	struct k_work work;
//	const struct device *dev;
//#endif
};

typedef int vl53l8cx_status_t;