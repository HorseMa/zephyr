/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <i2c.h>
#include <init.h>
#include <misc/__assert.h>
#include <misc/byteorder.h>
#include <sensor.h>
#include <string.h>

#include "max30205.h"

static int max30205_channel_get(struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct max30205_data *drv_data = dev->driver_data;
	s32_t conv_val;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_AMBIENT_TEMP);

	/*
	 * see "Interpreting humidity and temperature readings" document
	 * for more details
	 */
	conv_val = drv_data->temp_degc;
			/* convert temperature x8 to degrees Celsius */
			val->val1 = conv_val;
			val->val2 = 0;

	return 0;
}

static int max30205_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct max30205_data *drv_data = dev->driver_data;
	u8_t buf[2];

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
	printk("%s\n",__func__);
	buf[0] = 0;
	if(i2c_write(drv_data->i2c, buf, 1, MAX30205_I2C_ADDR) < 0)
	{
		printk("Failed to write pointer byte\n");
	}
	if(i2c_read(drv_data->i2c, buf,
			   2, MAX30205_I2C_ADDR) < 0) {
		printk("Failed to fetch data sample.");
		return -EIO;
	}
	/*if (i2c_reg_read16(drv_data->i2c, MAX30205_I2C_ADDR,
			   MAX30205_REG_TEMP,
			   buf) < 0) {
		printk("Failed to fetch data sample.");
		return -EIO;
	}*/

	drv_data->temp_degc = buf[1] | (buf[0] << 8);

	return 0;
}

static const struct sensor_driver_api max30205_driver_api = {
	.sample_fetch = max30205_sample_fetch,
	.channel_get = max30205_channel_get,
};

int max30205_init(struct device *dev)
{
	struct max30205_data *drv_data = dev->driver_data;
	printk("%s\n",__func__);
	drv_data->i2c = device_get_binding(CONFIG_MAX30205_I2C_MASTER_DEV_NAME);
	if (drv_data->i2c == NULL) {
		printk("Could not get pointer to %s device.",
			    CONFIG_MAX30205_I2C_MASTER_DEV_NAME);
		return -EINVAL;
	}

	return 0;
}

struct max30205_data max30205_driver;

DEVICE_AND_API_INIT(max30205, CONFIG_MAX30205_NAME, max30205_init, &max30205_driver,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &max30205_driver_api);
