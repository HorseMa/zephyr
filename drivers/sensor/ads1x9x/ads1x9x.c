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

int ads1x9x_reg_field_update(struct device *dev, u8_t reg_addr,
			    u8_t pos, u8_t mask, u8_t val)
{
	u8_t old_val;

	if (ads1x9x_byte_read(dev, reg_addr, &old_val) < 0) {
		return -EIO;
	}

	return ads1x9x_byte_write(dev, reg_addr,
				 (old_val & ~mask) | ((val << pos) & mask));
}

static int ads1x9x_pmu_set(struct device *dev, union ads1x9x_pmu_status *pmu_sts)
{
	struct {
		u8_t cmd;
		u16_t delay_us; /* values taken from page 82 */
	} cmds[] = {
		{ADS1X9X_CMD_PMU_MAG | pmu_sts->mag, 350},
		{ADS1X9X_CMD_PMU_ACC | pmu_sts->acc, 3200},
		{ADS1X9X_CMD_PMU_GYR | pmu_sts->gyr, 55000}
	};
	size_t i;

	for (i = 0; i < ARRAY_SIZE(cmds); i++) {
		union ads1x9x_pmu_status sts;
		bool pmu_set = false;

		if (ads1x9x_byte_write(dev, ADS1X9X_REG_CMD, cmds[i].cmd) < 0) {
			return -EIO;
		}

		/*
		 * Cannot use a timer here since this is called from the
		 * init function and the timeouts were not initialized yet.
		 */
		k_busy_wait(cmds[i].delay_us);

		/* make sure the PMU_STATUS was set, though */
		do {
			if (ads1x9x_byte_read(dev, ADS1X9X_REG_PMU_STATUS,
					     &sts.raw) < 0) {
				return -EIO;
			}

			if (i == 0) {
				pmu_set = (pmu_sts->mag == sts.mag);
			} else if (i == 1) {
				pmu_set = (pmu_sts->acc == sts.acc);
			} else {
				pmu_set = (pmu_sts->gyr == sts.gyr);
			}

		} while (!pmu_set);
	}

	/* set the undersampling flag for accelerometer */
	return ads1x9x_reg_field_update(dev, ADS1X9X_REG_ACC_CONF,
				       ADS1X9X_ACC_CONF_US, ADS1X9X_ACC_CONF_US,
				       pmu_sts->acc != ADS1X9X_PMU_NORMAL);
}

#if defined(CONFIG_ADS1X9X_GYRO_ODR_RUNTIME) ||\
	defined(CONFIG_ADS1X9X_ACCEL_ODR_RUNTIME)
/*
 * Output data rate map with allowed frequencies:
 * freq = freq_int + freq_milli / 1000
 *
 * Since we don't need a finer frequency resolution than milliHz, use u16_t
 * to save some flash.
 */
struct {
	u16_t freq_int;
	u16_t freq_milli; /* User should convert to uHz before setting the
			      * SENSOR_ATTR_SAMPLING_FREQUENCY attribute.
			      */
} ads1x9x_odr_map[] = {
	{0,    0  }, {0,     780}, {1,     562}, {3,    120}, {6,   250},
	{12,   500}, {25,    0  }, {50,    0  }, {100,  0  }, {200, 0  },
	{400,  0  }, {800,   0  }, {1600,  0  }, {3200, 0  },
};

static int ads1x9x_freq_to_odr_val(u16_t freq_int, u16_t freq_milli)
{
	size_t i;

	/* An ODR of 0 Hz is not allowed */
	if (freq_int == 0 && freq_milli == 0) {
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(ads1x9x_odr_map); i++) {
		if (freq_int < ads1x9x_odr_map[i].freq_int ||
		    (freq_int == ads1x9x_odr_map[i].freq_int &&
		     freq_milli <= ads1x9x_odr_map[i].freq_milli)) {
			return i;
		}
	}

	return -EINVAL;
}
#endif

#if defined(CONFIG_ADS1X9X_ACCEL_ODR_RUNTIME)
static int ads1x9x_acc_odr_set(struct device *dev, u16_t freq_int,
			      u16_t freq_milli)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	int odr = ads1x9x_freq_to_odr_val(freq_int, freq_milli);

	if (odr < 0) {
		return odr;
	}

	/* some odr values cannot be set in certain power modes */
	if ((ads1x9x->pmu_sts.acc == ADS1X9X_PMU_NORMAL &&
	     odr < ADS1X9X_ODR_25_2) ||
	    (ads1x9x->pmu_sts.acc == ADS1X9X_PMU_LOW_POWER &&
	    odr < ADS1X9X_ODR_25_32) || odr > ADS1X9X_ODR_1600) {
		return -ENOTSUP;
	}

	return ads1x9x_reg_field_update(dev, ADS1X9X_REG_ACC_CONF,
				       ADS1X9X_ACC_CONF_ODR_POS,
				       ADS1X9X_ACC_CONF_ODR_MASK,
				       (u8_t) odr);
}
#endif

static const struct ads1x9x_range ads1x9x_acc_range_map[] = {
	{2,	ADS1X9X_ACC_RANGE_2G},
	{4,	ADS1X9X_ACC_RANGE_4G},
	{8,	ADS1X9X_ACC_RANGE_8G},
	{16,	ADS1X9X_ACC_RANGE_16G},
};
#define ADS1X9X_ACC_RANGE_MAP_SIZE	ARRAY_SIZE(ads1x9x_acc_range_map)

static const struct ads1x9x_range ads1x9x_gyr_range_map[] = {
	{2000,	ADS1X9X_GYR_RANGE_2000DPS},
	{1000,	ADS1X9X_GYR_RANGE_1000DPS},
	{500,	ADS1X9X_GYR_RANGE_500DPS},
	{250,	ADS1X9X_GYR_RANGE_250DPS},
	{125,	ADS1X9X_GYR_RANGE_125DPS},
};
#define ADS1X9X_GYR_RANGE_MAP_SIZE	ARRAY_SIZE(ads1x9x_gyr_range_map)

#if defined(CONFIG_ADS1X9X_ACCEL_RANGE_RUNTIME) ||\
	defined(CONFIG_ADS1X9X_GYRO_RANGE_RUNTIME)
static s32_t ads1x9x_range_to_reg_val(u16_t range,
				       const struct ads1x9x_range *range_map,
				       u16_t range_map_size)
{
	int i;

	for (i = 0; i < range_map_size; i++) {
		if (range <= range_map[i].range) {
			return range_map[i].reg_val;
		}
	}

	return -EINVAL;
}
#endif

static s32_t ads1x9x_reg_val_to_range(u8_t reg_val,
				       const struct ads1x9x_range *range_map,
				       u16_t range_map_size)
{
	int i;

	for (i = 0; i < range_map_size; i++) {
		if (reg_val == range_map[i].reg_val) {
			return range_map[i].range;
		}
	}

	return -EINVAL;
}

s32_t ads1x9x_acc_reg_val_to_range(u8_t reg_val)
{
	return ads1x9x_reg_val_to_range(reg_val, ads1x9x_acc_range_map,
				       ADS1X9X_ACC_RANGE_MAP_SIZE);
}

s32_t ads1x9x_gyr_reg_val_to_range(u8_t reg_val)
{
	return ads1x9x_reg_val_to_range(reg_val, ads1x9x_gyr_range_map,
				       ADS1X9X_GYR_RANGE_MAP_SIZE);
}

static int ads1x9x_do_calibration(struct device *dev, u8_t foc_conf)
{
	if (ads1x9x_byte_write(dev, ADS1X9X_REG_FOC_CONF, foc_conf) < 0) {
		return -EIO;
	}

	if (ads1x9x_byte_write(dev, ADS1X9X_REG_CMD, ADS1X9X_CMD_START_FOC) < 0) {
		return -EIO;
	}

	k_busy_wait(250000); /* calibration takes a maximum of 250ms */

	return 0;
}

#if defined(CONFIG_ADS1X9X_ACCEL_RANGE_RUNTIME)
static int ads1x9x_acc_range_set(struct device *dev, s32_t range)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	s32_t reg_val = ads1x9x_range_to_reg_val(range,
						  ads1x9x_acc_range_map,
						  ADS1X9X_ACC_RANGE_MAP_SIZE);

	if (reg_val < 0) {
		return reg_val;
	}

	if (ads1x9x_byte_write(dev, ADS1X9X_REG_ACC_RANGE, reg_val & 0xff) < 0) {
		return -EIO;
	}

	ads1x9x->scale.acc = ADS1X9X_ACC_SCALE(range);

	return 0;
}
#endif

#if !defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
/*
 * Accelerometer offset scale, taken from pg. 79, converted to micro m/s^2:
 *	3.9 * 9.80665 * 1000
 */
#define ADS1X9X_ACC_OFS_LSB		38246
static int ads1x9x_acc_ofs_set(struct device *dev, enum sensor_channel chan,
			      const struct sensor_value *ofs)
{
	u8_t reg_addr[] = {
		ADS1X9X_REG_OFFSET_ACC_X,
		ADS1X9X_REG_OFFSET_ACC_Y,
		ADS1X9X_REG_OFFSET_ACC_Z
	};
	int i;
	s32_t ofs_u;
	s8_t reg_val;

	/* we need the offsets for all axis */
	if (chan != SENSOR_CHAN_ACCEL_XYZ) {
		return -ENOTSUP;
	}

	for (i = 0; i < 3; i++, ofs++) {
		/* convert ofset to micro m/s^2 */
		ofs_u = ofs->val1 * 1000000ULL + ofs->val2;
		reg_val = ofs_u / ADS1X9X_ACC_OFS_LSB;

		if (ads1x9x_byte_write(dev, reg_addr[i], reg_val) < 0) {
			return -EIO;
		}
	}

	/* activate accel HW compensation */
	return ads1x9x_reg_field_update(dev, ADS1X9X_REG_OFFSET_EN,
				       ADS1X9X_ACC_OFS_EN_POS,
				       BIT(ADS1X9X_ACC_OFS_EN_POS), 1);
}

static int  ads1x9x_acc_calibrate(struct device *dev, enum sensor_channel chan,
				 const struct sensor_value *xyz_calib_value)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	u8_t foc_pos[] = {
		ADS1X9X_FOC_ACC_X_POS,
		ADS1X9X_FOC_ACC_Y_POS,
		ADS1X9X_FOC_ACC_Z_POS,
	};
	int i;
	u8_t reg_val = 0;

	/* Calibration has to be done in normal mode. */
	if (ads1x9x->pmu_sts.acc != ADS1X9X_PMU_NORMAL) {
		return -ENOTSUP;
	}

	/*
	 * Hardware calibration is done knowing the expected values on all axis.
	 */
	if (chan != SENSOR_CHAN_ACCEL_XYZ) {
		return -ENOTSUP;
	}

	for (i = 0; i < 3; i++, xyz_calib_value++) {
		s32_t accel_g;
		u8_t accel_val;

		accel_g = sensor_ms2_to_g(xyz_calib_value);
		if (accel_g == 0) {
			accel_val = 3;
		} else if (accel_g == 1) {
			accel_val = 1;
		} else if (accel_g == -1) {
			accel_val = 2;
		} else {
			accel_val = 0;
		}
		reg_val |= (accel_val << foc_pos[i]);
	}

	if (ads1x9x_do_calibration(dev, reg_val) < 0) {
		return -EIO;
	}

	/* activate accel HW compensation */
	return ads1x9x_reg_field_update(dev, ADS1X9X_REG_OFFSET_EN,
				       ADS1X9X_ACC_OFS_EN_POS,
				       BIT(ADS1X9X_ACC_OFS_EN_POS), 1);
}

static int ads1x9x_acc_config(struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	switch (attr) {
#if defined(CONFIG_ADS1X9X_ACCEL_RANGE_RUNTIME)
	case SENSOR_ATTR_FULL_SCALE:
		return ads1x9x_acc_range_set(dev, sensor_ms2_to_g(val));
#endif
#if defined(CONFIG_ADS1X9X_ACCEL_ODR_RUNTIME)
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return ads1x9x_acc_odr_set(dev, val->val1, val->val2 / 1000);
#endif
	case SENSOR_ATTR_OFFSET:
		return ads1x9x_acc_ofs_set(dev, chan, val);
	case SENSOR_ATTR_CALIB_TARGET:
		return ads1x9x_acc_calibrate(dev, chan, val);
#if defined(CONFIG_ADS1X9X_TRIGGER)
	case SENSOR_ATTR_SLOPE_TH:
	case SENSOR_ATTR_SLOPE_DUR:
		return ads1x9x_acc_slope_config(dev, attr, val);
#endif
	default:
		SYS_LOG_DBG("Accel attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}
#endif /* !defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND) */

#if defined(CONFIG_ADS1X9X_GYRO_ODR_RUNTIME)
static int ads1x9x_gyr_odr_set(struct device *dev, u16_t freq_int,
			      u16_t freq_milli)
{
	int odr = ads1x9x_freq_to_odr_val(freq_int, freq_milli);

	if (odr < 0) {
		return odr;
	}

	if (odr < ADS1X9X_ODR_25 || odr > ADS1X9X_ODR_3200) {
		return -ENOTSUP;
	}

	return ads1x9x_reg_field_update(dev, ADS1X9X_REG_GYR_CONF,
				       ADS1X9X_GYR_CONF_ODR_POS,
				       ADS1X9X_GYR_CONF_ODR_MASK,
				       (u8_t) odr);
}
#endif

#if defined(CONFIG_ADS1X9X_GYRO_RANGE_RUNTIME)
static int ads1x9x_gyr_range_set(struct device *dev, u16_t range)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	s32_t reg_val = ads1x9x_range_to_reg_val(range,
						ads1x9x_gyr_range_map,
						ADS1X9X_GYR_RANGE_MAP_SIZE);

	if (reg_val < 0) {
		return reg_val;
	}

	if (ads1x9x_byte_write(dev, ADS1X9X_REG_GYR_RANGE, reg_val) < 0) {
		return -EIO;
	}

	ads1x9x->scale.gyr = ADS1X9X_GYR_SCALE(range);

	return 0;
}
#endif

#if !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
/*
 * Gyro offset scale, taken from pg. 79, converted to micro rad/s:
 *		0.061 * (pi / 180) * 1000000, where pi = 3.141592
 */
#define ADS1X9X_GYR_OFS_LSB		1065
static int ads1x9x_gyr_ofs_set(struct device *dev, enum sensor_channel chan,
			      const struct sensor_value *ofs)
{
	struct {
		u8_t lsb_addr;
		u8_t msb_pos;
	} ofs_desc[] = {
		{ADS1X9X_REG_OFFSET_GYR_X, ADS1X9X_GYR_MSB_OFS_X_POS},
		{ADS1X9X_REG_OFFSET_GYR_Y, ADS1X9X_GYR_MSB_OFS_Y_POS},
		{ADS1X9X_REG_OFFSET_GYR_Z, ADS1X9X_GYR_MSB_OFS_Z_POS},
	};
	int i;
	s32_t ofs_u;
	s16_t val;

	/* we need the offsets for all axis */
	if (chan != SENSOR_CHAN_GYRO_XYZ) {
		return -ENOTSUP;
	}

	for (i = 0; i < 3; i++, ofs++) {
		/* convert offset to micro rad/s */
		ofs_u = ofs->val1 * 1000000ULL + ofs->val2;

		val = ofs_u / ADS1X9X_GYR_OFS_LSB;

		/*
		 * The gyro offset is a 10 bit two-complement value. Make sure
		 * the passed value is within limits.
		 */
		if (val < -512 || val > 512) {
			return -EINVAL;
		}

		/* write the LSB */
		if (ads1x9x_byte_write(dev, ofs_desc[i].lsb_addr,
				      val & 0xff) < 0) {
			return -EIO;
		}

		/* write the MSB */
		if (ads1x9x_reg_field_update(dev, ADS1X9X_REG_OFFSET_EN,
					    ofs_desc[i].msb_pos,
					    0x3 << ofs_desc[i].msb_pos,
					    (val >> 8) & 0x3) < 0) {
			return -EIO;
		}
	}

	/* activate gyro HW compensation */
	return ads1x9x_reg_field_update(dev, ADS1X9X_REG_OFFSET_EN,
				       ADS1X9X_GYR_OFS_EN_POS,
				       BIT(ADS1X9X_GYR_OFS_EN_POS), 1);
}

static int ads1x9x_gyr_calibrate(struct device *dev, enum sensor_channel chan)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;

	ARG_UNUSED(chan);

	/* Calibration has to be done in normal mode. */
	if (ads1x9x->pmu_sts.gyr != ADS1X9X_PMU_NORMAL) {
		return -ENOTSUP;
	}

	if (ads1x9x_do_calibration(dev, BIT(ADS1X9X_FOC_GYR_EN_POS)) < 0) {
		return -EIO;
	}

	/* activate gyro HW compensation */
	return ads1x9x_reg_field_update(dev, ADS1X9X_REG_OFFSET_EN,
				       ADS1X9X_GYR_OFS_EN_POS,
				       BIT(ADS1X9X_GYR_OFS_EN_POS), 1);
}

static int ads1x9x_gyr_config(struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	switch (attr) {
#if defined(CONFIG_ADS1X9X_GYRO_RANGE_RUNTIME)
	case SENSOR_ATTR_FULL_SCALE:
		return ads1x9x_gyr_range_set(dev, sensor_rad_to_degrees(val));
#endif
#if defined(CONFIG_ADS1X9X_GYRO_ODR_RUNTIME)
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return ads1x9x_gyr_odr_set(dev, val->val1, val->val2 / 1000);
#endif
	case SENSOR_ATTR_OFFSET:
		return ads1x9x_gyr_ofs_set(dev, chan, val);

	case SENSOR_ATTR_CALIB_TARGET:
		return ads1x9x_gyr_calibrate(dev, chan);

	default:
		SYS_LOG_DBG("Gyro attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}
#endif /* !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND) */

static int ads1x9x_attr_set(struct device *dev, enum sensor_channel chan,
		    enum sensor_attribute attr, const struct sensor_value *val)
{
	switch (chan) {
#if !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		return ads1x9x_gyr_config(dev, chan, attr, val);
#endif
#if !defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		return ads1x9x_acc_config(dev, chan, attr, val);
#endif
	default:
		SYS_LOG_DBG("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

#if defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
#	define ADS1X9X_SAMPLE_BURST_READ_ADDR	ADS1X9X_REG_DATA_ACC_X
#	define ADS1X9X_DATA_READY_BIT_MASK	(1 << 7)
#else
#	define ADS1X9X_SAMPLE_BURST_READ_ADDR	ADS1X9X_REG_DATA_GYR_X
#	define ADS1X9X_DATA_READY_BIT_MASK	(1 << 6)
#endif
static int ads1x9x_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	size_t i;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	ads1x9x->sample.raw[0] = 0;

	while ((ads1x9x->sample.raw[0] & ADS1X9X_DATA_READY_BIT_MASK) == 0) {
		if (ads1x9x_transceive(dev, ADS1X9X_REG_STATUS | (1 << 7), false,
				      ads1x9x->sample.raw, 1) < 0) {
			return -EIO;
		}
	}

	if (ads1x9x_transceive(dev, ADS1X9X_SAMPLE_BURST_READ_ADDR | (1 << 7),
			      false, ads1x9x->sample.raw, ADS1X9X_BUF_SIZE) < 0) {
		return -EIO;
	}

	/* convert samples to cpu endianness */
	for (i = 0; i < ADS1X9X_SAMPLE_SIZE; i += 2) {
		u16_t *sample =
			(u16_t *) &ads1x9x->sample.raw[i];

		*sample = sys_le16_to_cpu(*sample);
	}

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
	s32_t acc_range, gyr_range;

	ads1x9x->spi = device_get_binding(CONFIG_ADS1X9X_SPI_PORT_NAME);
	if (!ads1x9x->spi) {
		SYS_LOG_DBG("SPI master controller not found: %d.",
			    CONFIG_ADS1X9X_SPI_PORT_NAME);
		return -EINVAL;
	}

	ads1x9x->spi_cfg.operation = SPI_WORD_SET(8) | SPI_MODE_CPHA & (~SPI_MODE_CPOL) | SPI_TRANSFER_MSB;
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

	/* set default PMU for gyro, accelerometer */
	ads1x9x->pmu_sts.gyr = ADS1X9X_DEFAULT_PMU_GYR;
	ads1x9x->pmu_sts.acc = ADS1X9X_DEFAULT_PMU_ACC;
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

	/* set accelerometer default range */
	if (ads1x9x_byte_write(dev, ADS1X9X_REG_ACC_RANGE,
				ADS1X9X_DEFAULT_RANGE_ACC) < 0) {
		SYS_LOG_DBG("Cannot set default range for accelerometer.");
		return -EIO;
	}

	acc_range = ads1x9x_acc_reg_val_to_range(ADS1X9X_DEFAULT_RANGE_ACC);

	ads1x9x->scale.acc = ADS1X9X_ACC_SCALE(acc_range);

	/* set gyro default range */
	if (ads1x9x_byte_write(dev, ADS1X9X_REG_GYR_RANGE,
			      ADS1X9X_DEFAULT_RANGE_GYR) < 0) {
		SYS_LOG_DBG("Cannot set default range for gyroscope.");
		return -EIO;
	}

	gyr_range = ads1x9x_gyr_reg_val_to_range(ADS1X9X_DEFAULT_RANGE_GYR);

	ads1x9x->scale.gyr = ADS1X9X_GYR_SCALE(gyr_range);

	if (ads1x9x_reg_field_update(dev, ADS1X9X_REG_ACC_CONF,
				    ADS1X9X_ACC_CONF_ODR_POS,
				    ADS1X9X_ACC_CONF_ODR_MASK,
				    ADS1X9X_DEFAULT_ODR_ACC) < 0) {
		SYS_LOG_DBG("Failed to set accel's default ODR.");
		return -EIO;
	}

	if (ads1x9x_reg_field_update(dev, ADS1X9X_REG_GYR_CONF,
				    ADS1X9X_GYR_CONF_ODR_POS,
				    ADS1X9X_GYR_CONF_ODR_MASK,
				    ADS1X9X_DEFAULT_ODR_GYR) < 0) {
		SYS_LOG_DBG("Failed to set gyro's default ODR.");
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
	.int_pin = CONFIG_ADS1X9X_GPIO_PIN_NUM,
#endif
};

DEVICE_INIT(ads1x9x, CONFIG_ADS1X9X_NAME, ads1x9x_init, &ads1x9x_data,
	    &ads1x9x_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY);
