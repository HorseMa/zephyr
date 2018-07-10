/* Bosch ADS1X9X inertial measurement unit driver
 *
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * http://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-ADS1X9X-DS000-07.pdf
 */

#include <init.h>
#include <sensor.h>
#include <misc/byteorder.h>
#include <kernel.h>
#include <misc/__assert.h>

#include "ads1x9x.h"
#include "ECGInterface.h"
#include "ADS1x9xTI.h"
#include "ADS1x9x_ECG_Processing.h"
#include "ADS1x9x_RESP_Processing.h"

struct ads1x9x_device_data ads1x9x_data;

static int ads1x9x_transceive(struct device *dev, u8_t reg,
			     bool write, void *data, size_t length)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	const struct spi_buf buf[2] = {
		{
			.buf = &reg,
			.len = 1
		},
		{
			.buf = data,
			.len = length
		}
	};
	const struct spi_buf_set tx = {
		.buffers = buf,
		.count = data ? 2 : 1
	};

	if (!write) {
		const struct spi_buf_set rx = {
			.buffers = buf,
			.count = 2
		};

		return spi_transceive(ads1x9x->spi, &ads1x9x->spi_cfg, &tx, &rx);
	}

	return spi_write(ads1x9x->spi, &ads1x9x->spi_cfg, &tx);
}
int ads1x9x_read(struct device *dev, u8_t reg_addr, u8_t *data, u8_t len)
{
	return ads1x9x_transceive(dev, reg_addr | BIT(7), false, data, len);
}

int ads1x9x_byte_read(struct device *dev, u8_t reg_addr, u8_t *byte)
{
	return ads1x9x_transceive(dev, reg_addr | BIT(7), false, byte, 1);
}

static int ads1x9x_word_read(struct device *dev, u8_t reg_addr, u16_t *word)
{
	if (ads1x9x_transceive(dev, reg_addr | BIT(7), false, word, 2) != 0) {
		return -EIO;
	}

	*word = sys_le16_to_cpu(*word);

	return 0;
}

int ads1x9x_byte_write(struct device *dev, u8_t reg_addr, u8_t byte)
{
	return ads1x9x_transceive(dev, reg_addr & 0x7F, true, &byte, 1);
}

int ads1x9x_word_write(struct device *dev, u8_t reg_addr, u16_t word)
{
	u8_t tx_word[2] = {
		(u8_t)(word & 0xff),
		(u8_t)(word >> 8)
	};

	return ads1x9x_transceive(dev, reg_addr & 0x7F, true, tx_word, 2);
}

static int ads1x9x_pmu_set(struct device *dev, union ads1x9x_pmu_status *pmu_sts)
{
	return 0;
}

static int ads1x9x_attr_set(struct device *dev, enum sensor_channel chan,
		    enum sensor_attribute attr, const struct sensor_value *val)
{
	return 0;
}

static int ads1x9x_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	return 0;
}

static void ads1x9x_to_fixed_point(s16_t raw_val, u16_t scale,
				  struct sensor_value *val)
{
	s32_t converted_val;

	/*
	 * maximum converted value we can get is: max(raw_val) * max(scale)
	 *	max(raw_val) = +/- 2^15
	 *	max(scale) = 4785
	 *	max(converted_val) = 156794880 which is less than 2^31
	 */
	converted_val = raw_val * scale;
	val->val1 = converted_val / 1000000;
	val->val2 = converted_val % 1000000;
}

static void ads1x9x_channel_convert(enum sensor_channel chan,
				   u16_t scale,
				   u16_t *raw_xyz,
				   struct sensor_value *val)
{
	int i;
	u8_t ofs_start, ofs_stop;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_GYRO_X:
		ofs_start = ofs_stop = 0;
		break;
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_GYRO_Y:
		ofs_start = ofs_stop = 1;
		break;
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_GYRO_Z:
		ofs_start = ofs_stop = 2;
		break;
	default:
		ofs_start = 0; ofs_stop = 2;
		break;
	}

	for (i = ofs_start; i <= ofs_stop ; i++, val++) {
		ads1x9x_to_fixed_point(raw_xyz[i], scale, val);
	}
}

#if !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
static inline void ads1x9x_gyr_channel_get(struct device *dev,
					  enum sensor_channel chan,
					  struct sensor_value *val)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;

	ads1x9x_channel_convert(chan, ads1x9x->scale.gyr,
			       ads1x9x->sample.gyr, val);
}
#endif

#if !defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
static inline void ads1x9x_acc_channel_get(struct device *dev,
					  enum sensor_channel chan,
					  struct sensor_value *val)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;

	ads1x9x_channel_convert(chan, ads1x9x->scale.acc,
			       ads1x9x->sample.acc, val);
}
#endif

static int ads1x9x_temp_channel_get(struct device *dev, struct sensor_value *val)
{
	u16_t temp_raw = 0;
	s32_t temp_micro = 0;
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;

	if (ads1x9x->pmu_sts.raw == 0) {
		return -EINVAL;
	}

	if (ads1x9x_word_read(dev, ADS1X9X_REG_TEMPERATURE0, &temp_raw) < 0) {
		return -EIO;
	}

	/* the scale is 1/2^9/LSB = 1953 micro degrees */
	temp_micro = ADS1X9X_TEMP_OFFSET * 1000000ULL + temp_raw * 1953ULL;

	val->val1 = temp_micro / 1000000ULL;
	val->val2 = temp_micro % 1000000ULL;

	return 0;
}

static int ads1x9x_channel_get(struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	switch (chan) {
#if !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		ads1x9x_gyr_channel_get(dev, chan, val);
		return 0;
#endif
#if !defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		ads1x9x_acc_channel_get(dev, chan, val);
		return 0;
#endif
	case SENSOR_CHAN_DIE_TEMP:
		return ads1x9x_temp_channel_get(dev, val);
	default:
		SYS_LOG_DBG("Channel not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api ads1x9x_api = {
	.attr_set = ads1x9x_attr_set,
#ifdef CONFIG_ADS1X9X_TRIGGER
	.trigger_set = ads1x9x_trigger_set,
#endif
	.sample_fetch = ads1x9x_sample_fetch,
	.channel_get = ads1x9x_channel_get,
};

int ads1x9x_init(struct device *dev)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	u8_t val = 0;
	printk("ads1x9x init!!!!!\r\n");
	k_busy_wait(1000);
	ECGInterface_Init();
	ECGInterface_StreamData();
	ADS1x9x_Filtered_ECG();
	ads1x9x->spi = device_get_binding(CONFIG_ADS1X9X_SPI_PORT_NAME);
	if (!ads1x9x->spi) {
		SYS_LOG_DBG("SPI master controller not found: %d.",
			    CONFIG_ADS1X9X_SPI_PORT_NAME);
		return -EINVAL;
	}

	ads1x9x->spi_cfg.operation = (((SPI_WORD_SET(8) | SPI_MODE_CPHA) & (~SPI_MODE_CPOL)) | SPI_TRANSFER_MSB);
	ads1x9x->spi_cfg.frequency = CONFIG_ADS1X9X_SPI_BUS_FREQ;
	ads1x9x->spi_cfg.slave = CONFIG_ADS1X9X_SLAVE;

	/* reboot the chip */
	if (ads1x9x_byte_write(dev, ADS1X9X_REG_CMD, ADS1X9X_CMD_SOFT_RESET) < 0) {
		SYS_LOG_DBG("Cannot reboot chip.");
		return -EIO;
	}

	k_busy_wait(1000);

	/* do a dummy read from 0x7F to activate SPI */
	if (ads1x9x_byte_read(dev, 0x7F, &val) < 0) {
		SYS_LOG_DBG("Cannot read from 0x7F..");
		return -EIO;
	}

	k_busy_wait(100);

	if (ads1x9x_byte_read(dev, ADS1X9X_REG_CHIPID, &val) < 0) {
		SYS_LOG_DBG("Failed to read chip id.");
		return -EIO;
	}

	if (val != ADS1X9X_CHIP_ID) {
		SYS_LOG_DBG("Unsupported chip detected (0x%x)!", val);
		return -ENODEV;
	}

	/* compass not supported, yet */
	ads1x9x->pmu_sts.mag = ADS1X9X_PMU_SUSPEND;

	/*
	 * The next command will take around 100ms (contains some necessary busy
	 * waits), but we cannot do it in a separate thread since we need to
	 * guarantee the BMI is up and running, before the app's main() is
	 * called.
	 */
	if (ads1x9x_pmu_set(dev, &ads1x9x->pmu_sts) < 0) {
		SYS_LOG_DBG("Failed to set power mode.");
		return -EIO;
	}

#ifdef CONFIG_ADS1X9X_TRIGGER
	if (ads1x9x_trigger_mode_init(dev) < 0) {
		SYS_LOG_DBG("Cannot set up trigger mode.");
		return -EINVAL;
	}
#endif

	dev->driver_api = &ads1x9x_api;

	return 0;
}

const struct ads1x9x_device_config ads1x9x_config = {
#if defined(CONFIG_ADS1X9X_TRIGGER)
	.gpio_port = CONFIG_ADS1X9X_GPIO_DEV_NAME,
	.int_pin = CONFIG_ADS1X9X_READY_GPIO_PIN_NUM,
#endif
};

DEVICE_INIT(ads1x9x, CONFIG_ADS1X9X_NAME, ads1x9x_init, &ads1x9x_data,
	    &ads1x9x_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY);
