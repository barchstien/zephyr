/** 
  * ST Microelectronics VL53L8CX ToF sensor
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * SPDX-License-Identifier: Apache-2.0
  *
  * Datasheet:
  * https://www.st.com/resource/en/datasheet/vl53l8cx.pdf
  */

#include "vl53l8cx_platform.h"
#include "vl53l8cx_types.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(VL53L8CX_PLATFORM, CONFIG_SENSOR_LOG_LEVEL);

uint8_t VL53L8CX_RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	return VL53L8CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L8CX_WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	return VL53L8CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L8CX_WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	int ret;
	const uint32_t CHUNK_MAX_SIZE = 64;
	// reserve 2 bytes for register address
	const uint32_t DATA_WRITE_MAX_SIZE = CHUNK_MAX_SIZE - 2;
	uint8_t buffer[CHUNK_MAX_SIZE];
	uint32_t i = 0;

	// Write by chunks, to reduce RAM use
	while (i < size) {
		const uint16_t reg_addr = RegisterAdress + i;
		uint32_t chunk_size = CHUNK_MAX_SIZE;
		if (size - i < DATA_WRITE_MAX_SIZE) {
			// last chunk isn't always full
			chunk_size = size - i + 2;
		}
		// prefix data with reg address, MSB first
		buffer[0] = (uint8_t)((reg_addr & 0xff00) >> 8);
		buffer[1] = (uint8_t)(reg_addr & 0x00ff);
		memcpy(
			&buffer[2], 
			p_values + i, 
			chunk_size - 2
		);

		ret = i2c_write_dt(&p_platform->config->i2c, buffer, chunk_size);
		if (ret != 0) {
			LOG_ERR("Failed to to write to i2c: %d", ret);
			return 255;
		}
		i += DATA_WRITE_MAX_SIZE;
	}
	
	return 0;
}

uint8_t VL53L8CX_RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	int ret;

	RegisterAdress = sys_cpu_to_be16(RegisterAdress);
	ret = i2c_write_read_dt(
		&p_platform->config->i2c, 
		(uint8_t *)(&RegisterAdress), 
		2, // index length
		p_values, 
		size
	);

	if (ret != 0) {
		LOG_ERR("Failed to write_read from i2c: %d", ret);
		return 255;
	}
	return 0;
}

uint8_t VL53L8CX_Reset_Sensor(
		VL53L8CX_Platform *p_platform)
{
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */

	/* Set pin LPN to LOW */
	gpio_pin_set_dt(&p_platform->config->lpn, 0);
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	/* Set pin CORE_1V8 to LOW */
	gpio_pin_set_dt(&p_platform->config->pwr, 0);
	VL53L8CX_WaitMs(p_platform, 10);

	/* Set pin LPN to HIGH */
	gpio_pin_set_dt(&p_platform->config->lpn, 1);
	/* Set pin AVDD to HIGH */
	/* Set pin VDDIO to HIGH */
	/* Set pin CORE_1V8 to HIGH */
	gpio_pin_set_dt(&p_platform->config->pwr, 1);
	VL53L8CX_WaitMs(p_platform, 10);

	return 0;
}

void VL53L8CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i;
	uint32_t* uint32_buffer = (uint32_t*)buffer;
	
	/* Example of possible implementation using <string.h> */
	//for(i = 0; i < size; i = i + 4) 
	//{
	//	tmp = (
	//	  buffer[i]<<24)
	//	|(buffer[i+1]<<16)
	//	|(buffer[i+2]<<8)
	//	|(buffer[i+3]);
	//	
	//	memcpy(&(buffer[i]), &tmp, 4);
	//}

	// Considering above example, 
	// and usage of the current function (see STN provided C code)
	// size is always a multiple of 4
	// swap 4 by 4 so platform may benefit from HW acceleration
	for(i = 0; i < size/4; i++) {
		uint32_buffer[i] = BSWAP_32(uint32_buffer[i]);
	}
}	

uint8_t VL53L8CX_WaitMs(
		VL53L8CX_Platform *p_platform,
		uint32_t TimeMs)
{
	/* Need to be implemented by customer. This function returns 0 if OK */
	k_msleep(TimeMs);
	return 0;
}
