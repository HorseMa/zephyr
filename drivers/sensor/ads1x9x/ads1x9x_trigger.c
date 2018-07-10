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
#include "ads1x9x.h"
#include "ECGInterface.h"
#include "ADS1x9xTI.h"
#include "ADS1x9x_ECG_Processing.h"
#include "ADS1x9x_RESP_Processing.h"

extern uint8_t SPI_Rx_Data_Flag,  SPI_Rx_buf[], SPI_Rx_Count, SPI_Rx_exp_Count;

static void ads1x9x_handle_interrupts(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	const u8_t READ_CONTINOUS_BUF[512] = {0};

	const struct spi_buf buf[1] = {
		{
			.buf = READ_CONTINOUS_BUF,
			.len = SPI_Rx_exp_Count
		}
	};
	const struct spi_buf_set tx = {
		.buffers = buf,
		.count = 1
	};

	const struct spi_buf_set rx = {
		.buffers = SPI_Rx_buf,
		.count = 1
	};

	spi_transceive(ads1x9x->spi, &ads1x9x->spi_cfg, &tx, &rx);

	ADS1x9x_Parse_data_packet();
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

int ads1x9x_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
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
