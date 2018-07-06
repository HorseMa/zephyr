/* Bosch ADS1X9X inertial measurement unit header
 *
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ADS1X9X_H_
#define _ADS1X9X_H_

#include <gpio.h>
#include <spi.h>
#include <misc/util.h>

/* registers */
#define ADS1X9X_REG_CHIPID		0x00
#define ADS1X9X_REG_ERR			0x02
#define ADS1X9X_REG_PMU_STATUS		0x03
#define ADS1X9X_REG_DATA_MAG_X		0x04
#define ADS1X9X_REG_DATA_MAG_Y		0x06
#define ADS1X9X_REG_DATA_MAG_Z		0x08
#define ADS1X9X_REG_DATA_RHALL		0x0A
#define ADS1X9X_REG_DATA_GYR_X		0x0C
#define ADS1X9X_REG_DATA_GYR_Y		0x0E
#define ADS1X9X_REG_DATA_GYR_Z		0x10
#define ADS1X9X_REG_DATA_ACC_X		0x12
#define ADS1X9X_REG_DATA_ACC_Y		0x14
#define ADS1X9X_REG_DATA_ACC_Z		0x16
#define ADS1X9X_REG_SENSORTIME0		0x18
#define ADS1X9X_REG_SENSORTIME1		0x19
#define ADS1X9X_REG_SENSORTIME2		0x1A
#define ADS1X9X_REG_STATUS		0x1B
#define ADS1X9X_REG_INT_STATUS0		0x1C
#define ADS1X9X_REG_INT_STATUS1		0x1D
#define ADS1X9X_REG_INT_STATUS2		0x1E
#define ADS1X9X_REG_INT_STATUS3		0x1F
#define ADS1X9X_REG_TEMPERATURE0		0x20
#define ADS1X9X_REG_TEMPERATURE1		0x21
#define ADS1X9X_REG_FIFO_LENGTH0		0x22
#define ADS1X9X_REG_FIFO_LENGTH1		0x23
#define ADS1X9X_REG_FIFO_DATA		0x24
#define ADS1X9X_REG_ACC_CONF		0x40
#define ADS1X9X_REG_ACC_RANGE		0x41
#define ADS1X9X_REG_GYR_CONF		0x42
#define ADS1X9X_REG_GYR_RANGE		0x43
#define ADS1X9X_REG_MAG_CONF		0x44
#define ADS1X9X_REG_FIFO_DOWNS		0x45
#define ADS1X9X_REG_FIFO_CONFIG0		0x46
#define ADS1X9X_REG_FIFO_CONFIG1		0x47
#define ADS1X9X_REG_MAG_IF0		0x4B
#define ADS1X9X_REG_MAG_IF1		0x4C
#define ADS1X9X_REG_MAG_IF2		0x4D
#define ADS1X9X_REG_MAG_IF3		0x4E
#define ADS1X9X_REG_MAG_IF4		0x4F
#define ADS1X9X_REG_INT_EN0		0x50
#define ADS1X9X_REG_INT_EN1		0x51
#define ADS1X9X_REG_INT_EN2		0x52
#define ADS1X9X_REG_INT_OUT_CTRL		0x53
#define ADS1X9X_REG_INT_LATCH		0x54
#define ADS1X9X_REG_INT_MAP0		0x55
#define ADS1X9X_REG_INT_MAP1		0x56
#define ADS1X9X_REG_INT_MAP2		0x57
#define ADS1X9X_REG_INT_DATA0		0x58
#define ADS1X9X_REG_INT_DATA1		0x59
#define ADS1X9X_REG_INT_LOWHIGH0		0x5A
#define ADS1X9X_REG_INT_LOWHIGH1		0x5B
#define ADS1X9X_REG_INT_LOWHIGH2		0x5C
#define ADS1X9X_REG_INT_LOWHIGH3		0x5D
#define ADS1X9X_REG_INT_LOWHIGH4		0x5E
#define ADS1X9X_REG_INT_MOTION0		0x5F
#define ADS1X9X_REG_INT_MOTION1		0x60
#define ADS1X9X_REG_INT_MOTION2		0x61
#define ADS1X9X_REG_INT_MOTION3		0x62
#define ADS1X9X_REG_INT_TAP0		0x63
#define ADS1X9X_REG_INT_TAP1		0x64
#define ADS1X9X_REG_INT_ORIENT0		0x65
#define ADS1X9X_REG_INT_ORIENT1		0x66
#define ADS1X9X_REG_INT_FLAT0		0x67
#define ADS1X9X_REG_INT_FLAT1		0x68
#define ADS1X9X_REG_FOC_CONF		0x69
#define ADS1X9X_REG_CONF			0x6A
#define ADS1X9X_REG_IF_CONF		0x6B
#define ADS1X9X_REG_PMU_TRIGGER		0x6C
#define ADS1X9X_REG_SELF_TEST		0x6D
#define ADS1X9X_REG_NV_CONF		0x70
#define ADS1X9X_REG_OFFSET_ACC_X		0x71
#define ADS1X9X_REG_OFFSET_ACC_Y		0x72
#define ADS1X9X_REG_OFFSET_ACC_Z		0x73
#define ADS1X9X_REG_OFFSET_GYR_X		0x74
#define ADS1X9X_REG_OFFSET_GYR_Y		0x75
#define ADS1X9X_REG_OFFSET_GYR_Z		0x76
#define ADS1X9X_REG_OFFSET_EN		0x77
#define ADS1X9X_REG_STEP_CNT0		0x78
#define ADS1X9X_REG_STEP_CNT1		0x79
#define ADS1X9X_REG_STEP_CONF0		0x7A
#define ADS1X9X_REG_STEP_CONF1		0x7B
#define ADS1X9X_REG_CMD			0x7E

/* bitfields */

/* ADS1X9X_REG_ERR */
#define ADS1X9X_ERR_FATAL		BIT(0)
#define ADS1X9X_ERR_CODE			BIT(1)
#define ADS1X9X_ERR_CODE_MASK		(0xf << 1)
#define ADS1X9X_ERR_I2C_FAIL		BIT(5)
#define ADS1X9X_ERR_DROP_CMD		BIT(6)
#define ADS1X9X_ERR_MAG_DRDY		BIT(7)

/* ADS1X9X_REG_PMU_STATUS */
#define ADS1X9X_PMU_STATUS_MAG_MASK	0x3
#define ADS1X9X_PMU_STATUS_MAG_POS	0
#define ADS1X9X_PMU_STATUS_GYR_POS	2
#define ADS1X9X_PMU_STATUS_GYR_MASK	(0x3 << 2)
#define ADS1X9X_PMU_STATUS_ACC_POS	4
#define ADS1X9X_PMU_STATUS_ACC_MASK	(0x3 << 4)

#define ADS1X9X_PMU_SUSPEND		0
#define ADS1X9X_PMU_NORMAL		1
#define ADS1X9X_PMU_LOW_POWER		2
#define ADS1X9X_PMU_FAST_START		3

/* ADS1X9X_REG_STATUS */
#define ADS1X9X_STATUS_GYR_SELFTEST	BIT(1)
#define ADS1X9X_STATUS_MAG_MAN_OP	BIT(2)
#define ADS1X9X_STATUS_FOC_RDY		BIT(3)
#define ADS1X9X_STATUS_NVM_RDY		BIT(4)
#define ADS1X9X_STATUS_MAG_DRDY		BIT(5)
#define ADS1X9X_STATUS_GYR_DRDY		BIT(6)
#define ADS1X9X_STATUS_ACC_DRDY		BIT(7)

/* ADS1X9X_REG_INT_STATUS0 */
#define ADS1X9X_INT_STATUS0_STEP		BIT(0)
#define ADS1X9X_INT_STATUS0_SIGMOT	BIT(1)
#define ADS1X9X_INT_STATUS0_ANYM		BIT(2)
#define ADS1X9X_INT_STATUS0_PMU_TRIG	BIT(3)
#define ADS1X9X_INT_STATUS0_D_TAP	BIT(4)
#define ADS1X9X_INT_STATUS0_S_TAP	BIT(5)
#define ADS1X9X_INT_STATUS0_ORIENT	BIT(6)
#define ADS1X9X_INT_STATUS0_FLAT		BIT(7)

/* ADS1X9X_REG_INT_STATUS1 */
#define ADS1X9X_INT_STATUS1_HIGHG	BIT(2)
#define ADS1X9X_INT_STATUS1_LOWG		BIT(3)
#define ADS1X9X_INT_STATUS1_DRDY		BIT(4)
#define ADS1X9X_INT_STATUS1_FFULL	BIT(5)
#define ADS1X9X_INT_STATUS1_FWM		BIT(6)
#define ADS1X9X_INT_STATUS1_NOMO		BIT(7)

/* ADS1X9X_REG_INT_STATUS2 */
#define ADS1X9X_INT_STATUS2_ANYM_FIRST_X BIT(0)
#define ADS1X9X_INT_STATUS2_ANYM_FIRST_Y BIT(1)
#define ADS1X9X_INT_STATUS2_ANYM_FIRST_Z BIT(2)
#define ADS1X9X_INT_STATUS2_ANYM_SIGN	BIT(3)
#define ADS1X9X_INT_STATUS2_TAP_FIRST_X	BIT(4)
#define ADS1X9X_INT_STATUS2_TAP_FIRST_Y	BIT(5)
#define ADS1X9X_INT_STATUS2_TAP_FIRST_Z	BIT(6)
#define ADS1X9X_INT_STATUS2_TAP_SIGN	BIT(7)

/* ADS1X9X_REG_INT_STATUS3 */
#define ADS1X9X_INT_STATUS3_HIGH_FIRST_X BIT(0)
#define ADS1X9X_INT_STATUS3_HIGH_FIRST_Y BIT(1)
#define ADS1X9X_INT_STATUS3_HIGH_FIRST_Z BIT(2)
#define ADS1X9X_INT_STATUS3_HIGH_SIGN	BIT(3)
#define ADS1X9X_INT_STATUS3_ORIENT_1_0	BIT(4)
#define ADS1X9X_INT_STATUS3_ORIENT_2	BIT(6)
#define ADS1X9X_INT_STATUS3_FLAT		BIT(7)

/* ADS1X9X_REG_ACC_CONF */
#define ADS1X9X_ACC_CONF_ODR_POS		0
#define ADS1X9X_ACC_CONF_ODR_MASK	0xF
#define ADS1X9X_ACC_CONF_BWP_POS		4
#define ADS1X9X_ACC_CONF_BWP_MASK	(0x7 << 4)
#define ADS1X9X_ACC_CONF_US		BIT(7)

/* ADS1X9X_REG_GYRO_CONF */
#define ADS1X9X_GYR_CONF_ODR_POS	0
#define ADS1X9X_GYR_CONF_ODR_MASK	0xF
#define ADS1X9X_GYR_CONF_BWP_POS	4
#define ADS1X9X_GYR_CONF_BWP_MASK	(0x3 << 4)

/* ADS1X9X_REG_OFFSET_EN */
#define ADS1X9X_GYR_OFS_EN_POS		7
#define ADS1X9X_ACC_OFS_EN_POS		6
#define ADS1X9X_GYR_MSB_OFS_Z_POS	4
#define ADS1X9X_GYR_MSB_OFS_Z_MASK	(BIT(4) | BIT(5))
#define ADS1X9X_GYR_MSB_OFS_Y_POS	2
#define ADS1X9X_GYR_MSB_OFS_Y_MASK	(BIT(2) | BIT(3))
#define ADS1X9X_GYR_MSB_OFS_X_POS	0
#define ADS1X9X_GYR_MSB_OFS_X_MASK	(BIT(0) | BIT(1))

/* ADS1X9X_REG_CMD */
#define ADS1X9X_CMD_START_FOC		3
#define ADS1X9X_CMD_PMU_ACC		0x10
#define ADS1X9X_CMD_PMU_GYR		0x14
#define ADS1X9X_CMD_PMU_MAG		0x18
#define ADS1X9X_CMD_SOFT_RESET		0xB6

/* ADS1X9X_REG_FOC_CONF */
#define ADS1X9X_FOC_ACC_Z_POS		0
#define ADS1X9X_FOC_ACC_Y_POS		2
#define ADS1X9X_FOC_ACC_X_POS		4
#define ADS1X9X_FOC_GYR_EN_POS		6

/* ADS1X9X_REG_INT_MOTION0 */
#define ADS1X9X_ANYM_DUR_POS		0
#define ADS1X9X_ANYM_DUR_MASK		0x3

/* ADS1X9X_REG_INT_EN0 */
#define ADS1X9X_INT_FLAT_EN		BIT(7)
#define ADS1X9X_INT_ORIENT_EN		BIT(6)
#define ADS1X9X_INT_S_TAP_EN		BIT(5)
#define ADS1X9X_INT_D_TAP_EN		BIT(4)
#define ADS1X9X_INT_ANYM_Z_EN		BIT(2)
#define ADS1X9X_INT_ANYM_Y_EN		BIT(1)
#define ADS1X9X_INT_ANYM_X_EN		BIT(0)
#define ADS1X9X_INT_ANYM_MASK		(BIT(0) | BIT(1) | BIT(2))

/* ADS1X9X_REG_INT_EN1 */
#define ADS1X9X_INT_FWM_EN		BIT(6)
#define ADS1X9X_INT_FFULL_EN		BIT(5)
#define ADS1X9X_INT_DRDY_EN		BIT(4)
#define ADS1X9X_INT_LOWG_EN		BIT(3)
#define ADS1X9X_INT_HIGHG_Z_EN		BIT(2)
#define ADS1X9X_INT_HIGHG_Y_EN		BIT(1)
#define ADS1X9X_INT_HIGHG_X_EN		BIT(0)
#define ADS1X9X_INT_HIGHG_MASK		(BIT(0) | BIT(1) | BIT(2))

/* ADS1X9X_REG_INT_EN2 */
#define ADS1X9X_INT_STEP_DET_EN		BIT(3)
#define ADS1X9X_INT_STEP_NOMO_Z_EN	BIT(2)
#define ADS1X9X_INT_STEP_NOMO_Y_EN	BIT(1)
#define ADS1X9X_INT_STEP_NOMO_X_EN	BIT(0)
#define ADS1X9X_INT_STEP_NOMO_MASK	(BIT(0) | BIT(1) | BIT(2))

/* ADS1X9X_REG_INT_OUT_CTRL */
#define ADS1X9X_INT2_OUT_EN		BIT(7)
#define ADS1X9X_INT2_OD			BIT(6)
#define ADS1X9X_INT2_LVL			BIT(5)
#define ADS1X9X_INT2_EDGE_CTRL		BIT(4)
#define ADS1X9X_INT1_OUT_EN		BIT(3)
#define ADS1X9X_INT1_OD			BIT(2)
#define ADS1X9X_INT1_LVL			BIT(1)
#define ADS1X9X_INT1_EDGE_CTRL		BIT(0)

/* other */
#define ADS1X9X_CHIP_ID			0xD1
#define ADS1X9X_TEMP_OFFSET		23

/* allowed ODR values */
enum ads1x9x_odr {
	ADS1X9X_ODR_25_32 = 1,
	ADS1X9X_ODR_25_16,
	ADS1X9X_ODR_25_8,
	ADS1X9X_ODR_25_4,
	ADS1X9X_ODR_25_2,
	ADS1X9X_ODR_25,
	ADS1X9X_ODR_50,
	ADS1X9X_ODR_100,
	ADS1X9X_ODR_200,
	ADS1X9X_ODR_400,
	ADS1X9X_ODR_800,
	ADS1X9X_ODR_1600,
	ADS1X9X_ODR_3200,
};

/* Range values for accelerometer */
#define ADS1X9X_ACC_RANGE_2G		0x3
#define ADS1X9X_ACC_RANGE_4G		0x5
#define ADS1X9X_ACC_RANGE_8G		0x8
#define ADS1X9X_ACC_RANGE_16G		0xC

/* Range values for gyro */
#define ADS1X9X_GYR_RANGE_2000DPS	0
#define ADS1X9X_GYR_RANGE_1000DPS	1
#define ADS1X9X_GYR_RANGE_500DPS		2
#define ADS1X9X_GYR_RANGE_250DPS		3
#define ADS1X9X_GYR_RANGE_125DPS		4

#define ADS1X9X_ACC_SCALE(range_g)	((2 * range_g * SENSOR_G) / 65536LL)
#define ADS1X9X_GYR_SCALE(range_dps)\
				((2 * range_dps * SENSOR_PI) / 180LL / 65536LL)

/* default settings, based on menuconfig options */

/* make sure at least one sensor is active */
#if defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND) &&\
		defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
#error "Error: You need to activate at least one sensor!"
#endif

#if defined(CONFIG_ADS1X9X_ACCEL_PMU_RUNTIME) ||\
		defined(CONFIG_ADS1X9X_ACCEL_PMU_NORMAL)
#	define ADS1X9X_DEFAULT_PMU_ACC		ADS1X9X_PMU_NORMAL
#elif defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
#	define ADS1X9X_DEFAULT_PMU_ACC		ADS1X9X_PMU_SUSPEND
#else
#	define ADS1X9X_DEFAULT_PMU_ACC		ADS1X9X_PMU_LOW_POWER
#endif

#if defined(CONFIG_ADS1X9X_GYRO_PMU_RUNTIME) ||\
		defined(CONFIG_ADS1X9X_GYRO_PMU_NORMAL)
#	define ADS1X9X_DEFAULT_PMU_GYR		ADS1X9X_PMU_NORMAL
#elif defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
#	define ADS1X9X_DEFAULT_PMU_GYR		ADS1X9X_PMU_SUSPEND
#else
#	define ADS1X9X_DEFAULT_PMU_GYR		ADS1X9X_PMU_FAST_START
#endif

#if defined(CONFIG_ADS1X9X_ACCEL_RANGE_RUNTIME) ||\
		defined(CONFIG_ADS1X9X_ACCEL_RANGE_2G)
#	define ADS1X9X_DEFAULT_RANGE_ACC		ADS1X9X_ACC_RANGE_2G
#elif defined(CONFIG_ADS1X9X_ACCEL_RANGE_4G)
#	define ADS1X9X_DEFAULT_RANGE_ACC		ADS1X9X_ACC_RANGE_4G
#elif defined(CONFIG_ADS1X9X_ACCEL_RANGE_8G)
#	define ADS1X9X_DEFAULT_RANGE_ACC		ADS1X9X_ACC_RANGE_8G
#else
#	define ADS1X9X_DEFAULT_RANGE_ACC		ADS1X9X_ACC_RANGE_16G
#endif

#if defined(CONFIG_ADS1X9X_GYRO_RANGE_RUNTIME) ||\
		defined(CONFIG_ADS1X9X_GYRO_RANGE_2000DPS)
#	define ADS1X9X_DEFAULT_RANGE_GYR		ADS1X9X_GYR_RANGE_2000DPS
#elif defined(CONFIG_ADS1X9X_GYRO_RANGE_1000DPS)
#	define ADS1X9X_DEFAULT_RANGE_GYR		ADS1X9X_GYR_RANGE_1000DPS
#elif defined(CONFIG_ADS1X9X_GYRO_RANGE_500DPS)
#	define ADS1X9X_DEFAULT_RANGE_GYR		ADS1X9X_GYR_RANGE_500DPS
#elif defined(CONFIG_ADS1X9X_GYRO_RANGE_250DPS)
#	define ADS1X9X_DEFAULT_RANGE_GYR		ADS1X9X_GYR_RANGE_250DPS
#else
#	define ADS1X9X_DEFAULT_RANGE_GYR		ADS1X9X_GYR_RANGE_125DPS
#endif

#if defined(CONFIG_ADS1X9X_ACCEL_ODR_RUNTIME) ||\
		defined(CONFIG_ADS1X9X_ACCEL_ODR_100)
#	define ADS1X9X_DEFAULT_ODR_ACC		8
#elif defined(CONFIG_ADS1X9X_ACCEL_ODR_25_32)
#	define ADS1X9X_DEFAULT_ODR_ACC		1
#elif defined(CONFIG_ADS1X9X_ACCEL_ODR_25_16)
#	define ADS1X9X_DEFAULT_ODR_ACC		2
#elif defined(CONFIG_ADS1X9X_ACCEL_ODR_25_8)
#	define ADS1X9X_DEFAULT_ODR_ACC		3
#elif defined(CONFIG_ADS1X9X_ACCEL_ODR_25_4)
#	define ADS1X9X_DEFAULT_ODR_ACC		4
#elif defined(CONFIG_ADS1X9X_ACCEL_ODR_25_2)
#	define ADS1X9X_DEFAULT_ODR_ACC		5
#elif defined(CONFIG_ADS1X9X_ACCEL_ODR_25)
#	define ADS1X9X_DEFAULT_ODR_ACC		6
#elif defined(CONFIG_ADS1X9X_ACCEL_ODR_50)
#	define ADS1X9X_DEFAULT_ODR_ACC		7
#elif defined(CONFIG_ADS1X9X_ACCEL_ODR_200)
#	define ADS1X9X_DEFAULT_ODR_ACC		9
#elif defined(CONFIG_ADS1X9X_ACCEL_ODR_400)
#	define ADS1X9X_DEFAULT_ODR_ACC		10
#elif defined(CONFIG_ADS1X9X_ACCEL_ODR_800)
#	define ADS1X9X_DEFAULT_ODR_ACC		11
#else
#	define ADS1X9X_DEFAULT_ODR_ACC		12
#endif

#if defined(CONFIG_ADS1X9X_GYRO_ODR_RUNTIME) ||\
		defined(CONFIG_ADS1X9X_GYRO_ODR_100)
#	define ADS1X9X_DEFAULT_ODR_GYR		8
#elif defined(CONFIG_ADS1X9X_GYRO_ODR_25)
#	define ADS1X9X_DEFAULT_ODR_GYR		6
#elif defined(CONFIG_ADS1X9X_GYRO_ODR_50)
#	define ADS1X9X_DEFAULT_ODR_GYR		7
#elif defined(CONFIG_ADS1X9X_GYRO_ODR_200)
#	define ADS1X9X_DEFAULT_ODR_GYR		9
#elif defined(CONFIG_ADS1X9X_GYRO_ODR_400)
#	define ADS1X9X_DEFAULT_ODR_GYR		10
#elif defined(CONFIG_ADS1X9X_GYRO_ODR_800)
#	define ADS1X9X_DEFAULT_ODR_GYR		11
#elif defined(CONFIG_ADS1X9X_GYRO_ODR_1600)
#	define ADS1X9X_DEFAULT_ODR_GYR		12
#else
#	define ADS1X9X_DEFAULT_ODR_GYR		13
#endif

/* end of default settings */

struct ads1x9x_range {
	u16_t range;
	u8_t reg_val;
};

struct ads1x9x_device_config {
#if defined(CONFIG_ADS1X9X_TRIGGER)
	const char *gpio_port;
	u8_t int_pin;
#endif
};

union ads1x9x_pmu_status {
	u8_t raw;
	struct {
		u8_t mag : 2;
		u8_t gyr : 2;
		u8_t acc : 2;
		u8_t res : 2;
	};
};

#if !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND) && \
		!defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
#	define ADS1X9X_SAMPLE_SIZE		(6 * sizeof(u16_t))
#else
#	define ADS1X9X_SAMPLE_SIZE		(3 * sizeof(u16_t))
#endif

#define ADS1X9X_BUF_SIZE			(ADS1X9X_SAMPLE_SIZE)
union ads1x9x_sample {
	u8_t raw[ADS1X9X_BUF_SIZE];
	struct {
		u8_t dummy_byte;
#if !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
		u16_t gyr[3];
#endif
#if !defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
		u16_t acc[3];
#endif
	} __packed;
};

struct ads1x9x_scale {
	u16_t acc; /* micro m/s^2/lsb */
	u16_t gyr; /* micro radians/s/lsb */
};

struct ads1x9x_device_data {
	struct device *spi;
	struct spi_config spi_cfg;
#if defined(CONFIG_ADS1X9X_TRIGGER)
	struct device *gpio;
	struct gpio_callback gpio_cb;
#endif
	union ads1x9x_pmu_status pmu_sts;
	union ads1x9x_sample sample;
	struct ads1x9x_scale scale;

#ifdef CONFIG_ADS1X9X_TRIGGER_OWN_THREAD
	struct k_sem sem;
#endif

#ifdef CONFIG_ADS1X9X_TRIGGER_GLOBAL_THREAD
	struct k_work work;
	struct device *dev;
#endif

#ifdef CONFIG_ADS1X9X_TRIGGER
#if !defined(CONFIG_ADS1X9X_ACCEL_PMU_SUSPEND)
	sensor_trigger_handler_t handler_drdy_acc;
	sensor_trigger_handler_t handler_anymotion;
#endif
#if !defined(CONFIG_ADS1X9X_GYRO_PMU_SUSPEND)
	sensor_trigger_handler_t handler_drdy_gyr;
#endif
#endif /* CONFIG_ADS1X9X_TRIGGER */
};

int ads1x9x_read(struct device *dev, u8_t reg_addr,
		u8_t *data, u8_t len);
int ads1x9x_byte_read(struct device *dev, u8_t reg_addr, u8_t *byte);
int ads1x9x_byte_write(struct device *dev, u8_t reg_addr, u8_t byte);
int ads1x9x_word_write(struct device *dev, u8_t reg_addr, u16_t word);
int ads1x9x_reg_field_update(struct device *dev, u8_t reg_addr,
			    u8_t pos, u8_t mask, u8_t val);
static inline int ads1x9x_reg_update(struct device *dev, u8_t reg_addr,
				    u8_t mask, u8_t val)
{
	return ads1x9x_reg_field_update(dev, reg_addr, 0, mask, val);
}
int ads1x9x_trigger_mode_init(struct device *dev);
int ads1x9x_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);
int ads1x9x_acc_slope_config(struct device *dev, enum sensor_attribute attr,
			    const struct sensor_value *val);
s32_t ads1x9x_acc_reg_val_to_range(u8_t reg_val);
s32_t ads1x9x_gyr_reg_val_to_range(u8_t reg_val);

#define SYS_LOG_DOMAIN "ADS1X9X"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SENSOR_LEVEL
#include <logging/sys_log.h>
#endif /* _ADS1X9X_H_ */
