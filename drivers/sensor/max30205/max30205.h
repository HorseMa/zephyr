/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_MAX30205_H__
#define __SENSOR_MAX30205_H__

#include <device.h>
#include <misc/util.h>
#include <zephyr/types.h>
#include <gpio.h>

#define SYS_LOG_DOMAIN "MAX30205"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SENSOR_LEVEL
#include <logging/sys_log.h>

#define MAX30205_I2C_ADDR			CONFIG_MAX30205_I2C_ADDR

#define MAX30205_REG_TEMP		0X00
#define MAX30205_REG_CFG		0x01
#define MAX30205_REG_THYST		0X02
#define MAX30205_REG_TOS		0x03
#define MAX30205_TEMP_STEP		0.00390625

struct max30205_data {
	struct device *i2c;
	u16_t temp_degc;
	u16_t adc_12bit;
};

#endif /* __SENSOR_MAX30205__ */
