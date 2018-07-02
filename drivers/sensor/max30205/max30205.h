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

#define MAX30205_I2C_ADDR			0x5F
#define MAX30205_AUTOINCREMENT_ADDR	BIT(7)

#define MAX30205_REG_WHO_AM_I		0x0F
#define MAX30205_CHIP_ID			0xBC

#define MAX30205_REG_CTRL1		0x20
#define MAX30205_PD_BIT			BIT(7)
#define MAX30205_BDU_BIT			BIT(3)
#define MAX30205_ODR_SHIFT		0

#define MAX30205_REG_CTRL3		0x22
#define MAX30205_DRDY_EN			BIT(2)

#define MAX30205_REG_DATA_START		0x28
#define MAX30205_REG_CONVERSION_START	0x30

static const char * const max30205_odr_strings[] = {
	"1", "7", "12.5"
};

struct max30205_data {
	struct device *i2c;
	s16_t rh_sample;
	s16_t t_sample;

	u8_t h0_rh_x2;
	u8_t h1_rh_x2;
	u16_t t0_degc_x8;
	u16_t t1_degc_x8;
	s16_t h0_t0_out;
	s16_t h1_t0_out;
	s16_t t0_out;
	s16_t t1_out;

#ifdef CONFIG_MAX30205_TRIGGER
	struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

#if defined(CONFIG_MAX30205_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_MAX30205_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_MAX30205_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
	struct device *dev;
#endif

#endif /* CONFIG_MAX30205_TRIGGER */
};

#ifdef CONFIG_MAX30205_TRIGGER
int max30205_trigger_set(struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int max30205_init_interrupt(struct device *dev);
#endif

#endif /* __SENSOR_MAX30205__ */
