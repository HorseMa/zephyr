/* Bosch ADS1X9X inertial measurement unit driver, trigger implementation
 *
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <sensor.h>
#include <gpio.h>

#include "ads1x9x.h"

static void ads1x9x_handle_anymotion(struct device *dev)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	struct sensor_trigger anym_trigger = {
		.type = SENSOR_TRIG_DELTA,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	if (ads1x9x->handler_anymotion) {
		ads1x9x->handler_anymotion(dev, &anym_trigger);
	}
}

static void ads1x9x_handle_drdy(struct device *dev, u8_t status)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	struct sensor_trigger drdy_trigger = {
		.type = SENSOR_TRIG_DATA_READY,
	};

#if !defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
	if (ads1x9x->handler_drdy_acc && (status & ADS1X9X_STATUS_ACC_DRDY)) {
		drdy_trigger.chan = SENSOR_CHAN_ACCEL_XYZ;
		ads1x9x->handler_drdy_acc(dev, &drdy_trigger);
	}
#endif

#if !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
	if (ads1x9x->handler_drdy_gyr && (status & ADS1X9X_STATUS_GYR_DRDY)) {
		drdy_trigger.chan = SENSOR_CHAN_GYRO_XYZ;
		ads1x9x->handler_drdy_gyr(dev, &drdy_trigger);
	}
#endif
}

static void ads1x9x_handle_interrupts(void *arg)
{
	struct device *dev = (struct device *)arg;

	union {
		u8_t raw[6];
		struct {
			u8_t dummy; /* spi related dummy byte */
			u8_t status;
			u8_t int_status[4];
		};
	} buf;

	if (ads1x9x_read(dev, ADS1X9X_REG_STATUS, buf.raw, sizeof(buf)) < 0) {
		return;
	}

	if ((buf.int_status[0] & ADS1X9X_INT_STATUS0_ANYM) &&
	    (buf.int_status[2] & (ADS1X9X_INT_STATUS2_ANYM_FIRST_X |
				  ADS1X9X_INT_STATUS2_ANYM_FIRST_Y |
				  ADS1X9X_INT_STATUS2_ANYM_FIRST_Z))) {
		ads1x9x_handle_anymotion(dev);
	}

	if (buf.int_status[1] & ADS1X9X_INT_STATUS1_DRDY) {
		ads1x9x_handle_drdy(dev, buf.status);
	}

}

#ifdef CONFIG_ADS1X9X_TRIGGER_OWN_THREAD
static K_THREAD_STACK_DEFINE(ads1x9x_thread_stack, CONFIG_ADS1X9X_THREAD_STACK_SIZE);
static struct k_thread ads1x9x_thread;

static void ads1x9x_thread_main(void *arg1, void *unused1, void *unused2)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	struct device *dev = (struct device *)arg1;
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;

	while (1) {
		k_sem_take(&ads1x9x->sem, K_FOREVER);
		ads1x9x_handle_interrupts(dev);
	}
}
#endif

#ifdef CONFIG_ADS1X9X_TRIGGER_GLOBAL_THREAD
static void ads1x9x_work_handler(struct k_work *work)
{
	struct ads1x9x_device_data *ads1x9x =
		CONTAINER_OF(work, struct ads1x9x_device_data, work);

	ads1x9x_handle_interrupts(ads1x9x->dev);
}
#endif

extern struct ads1x9x_device_data ads1x9x_data;

static void ads1x9x_gpio_callback(struct device *port,
				 struct gpio_callback *cb, u32_t pin)
{
	struct ads1x9x_device_data *ads1x9x =
		CONTAINER_OF(cb, struct ads1x9x_device_data, gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pin);

#if defined(CONFIG_ADS1X9X_TRIGGER_OWN_THREAD)
	k_sem_give(&ads1x9x->sem);
#elif defined(CONFIG_ADS1X9X_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&ads1x9x->work);
#endif
}

static int ads1x9x_trigger_drdy_set(struct device *dev,
				   enum sensor_channel chan,
				   sensor_trigger_handler_t handler)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	u8_t drdy_en = 0;

#if !defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
	if (chan == SENSOR_CHAN_ACCEL_XYZ) {
		ads1x9x->handler_drdy_acc = handler;
	}

	if (ads1x9x->handler_drdy_acc) {
		drdy_en = ADS1X9X_INT_DRDY_EN;
	}
#endif

#if !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
	if (chan == SENSOR_CHAN_GYRO_XYZ) {
		ads1x9x->handler_drdy_gyr = handler;
	}

	if (ads1x9x->handler_drdy_gyr) {
		drdy_en = ADS1X9X_INT_DRDY_EN;
	}
#endif

	if (ads1x9x_reg_update(dev, ADS1X9X_REG_INT_EN1,
			      ADS1X9X_INT_DRDY_EN, drdy_en) < 0) {
		return -EIO;
	}

	return 0;
}

#if !defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
static int ads1x9x_trigger_anym_set(struct device *dev,
				   sensor_trigger_handler_t handler)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	u8_t anym_en = 0;

	ads1x9x->handler_anymotion = handler;

	if (handler) {
		anym_en = ADS1X9X_INT_ANYM_X_EN |
			  ADS1X9X_INT_ANYM_Y_EN |
			  ADS1X9X_INT_ANYM_Z_EN;
	}

	if (ads1x9x_reg_update(dev, ADS1X9X_REG_INT_EN0,
			      ADS1X9X_INT_ANYM_MASK, anym_en) < 0) {
		return -EIO;
	}

	return 0;
}

static int ads1x9x_trigger_set_acc(struct device *dev,
				  const struct sensor_trigger *trig,
				  sensor_trigger_handler_t handler)
{
	if (trig->type == SENSOR_TRIG_DATA_READY) {
		return ads1x9x_trigger_drdy_set(dev, trig->chan, handler);
	} else if (trig->type == SENSOR_TRIG_DELTA) {
		return ads1x9x_trigger_anym_set(dev, handler);
	}

	return -ENOTSUP;
}

int ads1x9x_acc_slope_config(struct device *dev, enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	u8_t acc_range_g, reg_val;
	u32_t slope_th_ums2;

	if (attr == SENSOR_ATTR_SLOPE_TH) {
		if (ads1x9x_byte_read(dev, ADS1X9X_REG_ACC_RANGE, &reg_val) < 0) {
			return -EIO;
		}

		acc_range_g = ads1x9x_acc_reg_val_to_range(reg_val);

		slope_th_ums2 = val->val1 * 1000000 + val->val2;

		/* make sure the provided threshold does not exceed range / 2 */
		if (slope_th_ums2 > (acc_range_g / 2 * SENSOR_G)) {
			return -EINVAL;
		}

		reg_val = 512 * (slope_th_ums2 - 1) / (acc_range_g * SENSOR_G);

		if (ads1x9x_byte_write(dev, ADS1X9X_REG_INT_MOTION1,
				      reg_val) < 0) {
			return -EIO;
		}
	} else { /* SENSOR_ATTR_SLOPE_DUR */
		/* slope duration is measured in number of samples */
		if (val->val1 < 1 || val->val1 > 4) {
			return -ENOTSUP;
		}

		if (ads1x9x_reg_field_update(dev, ADS1X9X_REG_INT_MOTION0,
					    ADS1X9X_ANYM_DUR_POS,
					    ADS1X9X_ANYM_DUR_MASK,
					    val->val1) < 0) {
			return -EIO;
		}
	}

	return 0;
}
#endif

#if !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
static int ads1x9x_trigger_set_gyr(struct device *dev,
				  const struct sensor_trigger *trig,
				  sensor_trigger_handler_t handler)
{
	if (trig->type == SENSOR_TRIG_DATA_READY) {
		return ads1x9x_trigger_drdy_set(dev, trig->chan, handler);
	}

	return -ENOTSUP;
}
#endif

int ads1x9x_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
#if !defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
	if (trig->chan == SENSOR_CHAN_ACCEL_XYZ) {
		return ads1x9x_trigger_set_acc(dev, trig, handler);
	}
#endif
#if !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
	if (trig->chan == SENSOR_CHAN_GYRO_XYZ) {
		return ads1x9x_trigger_set_gyr(dev, trig, handler);
	}
#endif
	return -ENOTSUP;
}

int ads1x9x_trigger_mode_init(struct device *dev)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;

	const struct ads1x9x_device_config *cfg = dev->config->config_info;

	ads1x9x->gpio = device_get_binding((char *)cfg->gpio_port);
	if (!ads1x9x->gpio) {
		SYS_LOG_DBG("Gpio controller %s not found.", cfg->gpio_port);
		return -EINVAL;
	}

#if defined(CONFIG_ADS1X9X_TRIGGER_OWN_THREAD)
	k_sem_init(&ads1x9x->sem, 0, UINT_MAX);

	k_thread_create(&ads1x9x_thread, ads1x9x_thread_stack,
			CONFIG_ADS1X9X_THREAD_STACK_SIZE,
			ads1x9x_thread_main, dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ADS1X9X_THREAD_PRIORITY), 0, 0);
#elif defined(CONFIG_ADS1X9X_TRIGGER_GLOBAL_THREAD)
	ads1x9x->work.handler = ads1x9x_work_handler;
	ads1x9x->dev = dev;
#endif

	/* map all interrupts to INT1 pin */
	if (ads1x9x_word_write(dev, ADS1X9X_REG_INT_MAP0, 0xf0ff) < 0) {
		SYS_LOG_DBG("Failed to map interrupts.");
		return -EIO;
	}

	gpio_pin_configure(ads1x9x->gpio, cfg->int_pin,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			   GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE);

	gpio_init_callback(&ads1x9x->gpio_cb,
			   ads1x9x_gpio_callback,
			   BIT(cfg->int_pin));

	gpio_add_callback(ads1x9x->gpio, &ads1x9x->gpio_cb);
	gpio_pin_enable_callback(ads1x9x->gpio, cfg->int_pin);

	return ads1x9x_byte_write(dev, ADS1X9X_REG_INT_OUT_CTRL,
				 ADS1X9X_INT1_OUT_EN | ADS1X9X_INT1_EDGE_CTRL);
}
