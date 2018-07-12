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
u8_t ads1x9xregval[12] = {0};
static u8_t SPI_Rx_exp_Count = 0;
static uint8_t ADS1x9xR_Default_Register_Settings[15] = {

	//Device ID read Ony
	0x00,
   	//CONFIG1
	 0x02,
    //CONFIG2
     0xE0,
    //LOFF
     0xF0,
	 //CH1SET (PGA gain = 6)
     0x00,
	 //CH2SET (PGA gain = 6)
     0x00,
	 //RLD_SENS (default)
	 0x2C,
	 //LOFF_SENS (default)
	 0x0F,
    //LOFF_STAT
     0x00,
    //RESP1
     0xEA,
	//RESP2
	 0x03,
	//GPIO
     0x0C
};
static uint8_t ADS1x9x_Default_Register_Settings[15] = {

	//Device ID read Ony
	0x00,
   	//CONFIG1
	 0x02,
    //CONFIG2
     0xE0,
    //LOFF
     0xF0,
	 //CH1SET (PGA gain = 6)
     0x00,
	 //CH2SET (PGA gain = 6)
     0x00,
	 //RLD_SENS (default)
	 0x2C,
	 //LOFF_SENS (default)
	 0x0F,
    //LOFF_STAT
     0x00,
    //RESP1
     0x02,
	//RESP2
	 0x03,
	//GPIO
     0x0C
};

void ads1x9x_set_out_bytes(void)
{
	switch( ads1x9xregval[0] & 0x03)
	{
		case ADS1191_16BIT:
			SPI_Rx_exp_Count=4;		// 2 byte status + 2 bytes CH0 data
		break;

		case ADS1192_16BIT:
			SPI_Rx_exp_Count=6;		// 2 byte status + 2 bytes ch1 data + 2 bytes CH0 data
		break;

		case ADS1291_24BIT:
			SPI_Rx_exp_Count=6;		// 3 byte status + 3 bytes CH0 data
		break;

		case ADS1292_24BIT:
			SPI_Rx_exp_Count=9;		// 3 byte status + 3 bytes ch1 data + 3 bytes CH0 data
		break;
	}
}

int ads1x9x_enable_start(struct device *dev)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	const struct ads1x9x_device_config *cfg = dev->config->config_info;

	return gpio_pin_write(ads1x9x->gpio, cfg->start_pin,1);
}

int ads1x9x_disable_start(struct device *dev)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	const struct ads1x9x_device_config *cfg = dev->config->config_info;

	return gpio_pin_write(ads1x9x->gpio, cfg->start_pin,0);
}

int ads1x9x_reset(struct device *dev)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	const struct ads1x9x_device_config *cfg = dev->config->config_info;

	gpio_pin_write(ads1x9x->gpio, cfg->reset_pin,1);
	k_busy_wait(1000);
	gpio_pin_write(ads1x9x->gpio, cfg->reset_pin,0);
	k_busy_wait(1000);
	gpio_pin_write(ads1x9x->gpio, cfg->reset_pin,1);
	k_busy_wait(1000);
	return 0;
}

int ads1x9x_powerdown(struct device *dev)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	const struct ads1x9x_device_config *cfg = dev->config->config_info;

	gpio_pin_write(ads1x9x->gpio, cfg->reset_pin,0);
	k_busy_wait(1000);
	return 0;
}

int ads1x9x_read_reg(struct device *dev, u8_t reg_addr, u8_t *byte)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	u8_t tx_buf[2],tx_byte,rx_buf[2];

	tx_buf[0] = reg_addr | RREG;
	tx_buf[1] = 0x00;
	tx_byte = 0;
	const struct spi_buf buf_tx[2] = {
		{
			.buf = tx_buf,
			.len = 2
		},
		{
			.buf = &tx_byte,
			.len = 1
		}
	};
	const struct spi_buf_set tx = {
		.buffers = buf_tx,
		.count = 2
	};
	const struct spi_buf buf_rx[2] = {
		{
			.buf = rx_buf,
			.len = 2
		},
		{
			.buf = byte,
			.len = 1
		}
	};
	const struct spi_buf_set rx = {
		.buffers = buf_rx,
		.count = 2
	};

	return spi_transceive(ads1x9x->spi, &ads1x9x->spi_cfg, &tx, &rx);
}

int ads1x9x_read_all_reg(struct device *dev, u8_t *regs)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	u8_t tx_buf[2],tx_buf_dumy[12] = {0},rx_buf[2];

	tx_buf[0] = 0 | RREG;
	tx_buf[1] = 11;
	const struct spi_buf buf_tx[2] = {
		{
			.buf = tx_buf,
			.len = 2
		},
		{
			.buf = tx_buf_dumy,
			.len = 12
		}
	};
	const struct spi_buf_set tx = {
		.buffers = buf_tx,
		.count = 2
	};
	const struct spi_buf buf_rx[2] = {
		{
			.buf = rx_buf,
			.len = 2
		},
		{
			.buf = regs,
			.len = 12
		}
	};
	const struct spi_buf_set rx = {
		.buffers = buf_rx,
		.count = 2
	};

	return spi_transceive(ads1x9x->spi, &ads1x9x->spi_cfg, &tx, &rx);
}

int ads1x9x_read_data(struct device *dev, u8_t *data)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	u8_t tx_buf[9] = {0};

	const struct spi_buf buf_tx[1] = {
		{
			.buf = tx_buf,
			.len = SPI_Rx_exp_Count
		},
	};
	const struct spi_buf_set tx = {
		.buffers = buf_tx,
		.count = 1
	};
	const struct spi_buf buf_rx[1] = {
		{
			.buf = data,
			.len = SPI_Rx_exp_Count
		},
	};
	const struct spi_buf_set rx = {
		.buffers = buf_rx,
		.count = 1
	};

	return spi_transceive(ads1x9x->spi, &ads1x9x->spi_cfg, &tx, &rx);
}

int ads1x9x_write_reg(struct device *dev, u8_t reg_addr, u8_t byte)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	u8_t tx_buf[2],rx_buf[2],rx_byte;

	tx_buf[0] = reg_addr | WREG;
	tx_buf[1] = 0x00;
	const struct spi_buf buf_tx[2] = {
		{
			.buf = tx_buf,
			.len = 2
		},
		{
			.buf = &byte,
			.len = 1
		}
	};
	const struct spi_buf_set tx = {
		.buffers = buf_tx,
		.count = 2
	};
	const struct spi_buf buf_rx[2] = {
		{
			.buf = rx_buf,
			.len = 2
		},
		{
			.buf = &rx_byte,
			.len = 1
		}
	};
	const struct spi_buf_set rx = {
		.buffers = buf_rx,
		.count = 2
	};

	return spi_transceive(ads1x9x->spi, &ads1x9x->spi_cfg, &tx, &rx);
}

int ads1x9x_write_all_default_regs(struct device *dev)
{
	uint8_t Reg_Init_i;

	if ((ads1x9xregval[0] & 0X20) == 0x20)
	{
		for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
		{
			ads1x9x_write_reg(dev,Reg_Init_i,ADS1x9xR_Default_Register_Settings[Reg_Init_i]);
		}
	}
	else
	{
		for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
		{
			ads1x9x_write_reg(dev,Reg_Init_i,ADS1x9x_Default_Register_Settings[Reg_Init_i]);
		}
	}

	return 0;
}

int ads1x9x_write_cmd(struct device *dev, u8_t cmd)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;

	const struct spi_buf buf_tx[1] = {
		{
			.buf = &cmd,
			.len = 1
		}
	};
	const struct spi_buf_set tx = {
		.buffers = buf_tx,
		.count = 1
	};

	return spi_write(ads1x9x->spi, &ads1x9x->spi_cfg, &tx);
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

static void ads1x9x_channel_convert(enum sensor_channel chan,
				   u16_t scale,
				   u16_t *raw_xyz,
				   struct sensor_value *val)
{

}

static int ads1x9x_temp_channel_get(struct device *dev, struct sensor_value *val)
{
	u16_t temp_raw = 0;
	s32_t temp_micro = 0;
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;

	return 0;
}

static int ads1x9x_channel_get(struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{

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

	printk("ads1x9x init!!!!!\r\n");
	const struct ads1x9x_device_config *cfg = dev->config->config_info;

	ads1x9x->gpio = device_get_binding((char *)cfg->gpio_port);
	if (!ads1x9x->gpio) {
		SYS_LOG_DBG("Gpio controller %s not found.", cfg->gpio_port);
		return -EINVAL;
	}

	gpio_pin_configure(ads1x9x->gpio, cfg->start_pin,
			   GPIO_DIR_OUT | GPIO_PUD_PULL_UP);
	gpio_pin_write(ads1x9x->gpio, cfg->start_pin,1);

	gpio_pin_configure(ads1x9x->gpio, cfg->reset_pin,
			   GPIO_DIR_OUT | GPIO_PUD_PULL_UP);
	gpio_pin_write(ads1x9x->gpio, cfg->reset_pin,1);


	ads1x9x->spi = device_get_binding(CONFIG_ADS1X9X_SPI_PORT_NAME);
	if (!ads1x9x->spi) {
		SYS_LOG_DBG("SPI master controller not found: %d.",
			    CONFIG_ADS1X9X_SPI_PORT_NAME);
		return -EINVAL;
	}

	ads1x9x->spi_cfg.operation = (((SPI_WORD_SET(8) | SPI_MODE_CPHA) & (~SPI_MODE_CPOL)) | SPI_TRANSFER_MSB);
	ads1x9x->spi_cfg.frequency = CONFIG_ADS1X9X_SPI_BUS_FREQ;
	ads1x9x->spi_cfg.slave = CONFIG_ADS1X9X_SLAVE;

	/* compass not supported, yet */
	//ads1x9x->pmu_sts.mag = ADS1X9X_PMU_SUSPEND;

	ads1x9x_reset(dev);

 	k_busy_wait(5000);
    // Set internal clock

    //The START pin must be set high to begin conversions.
	ads1x9x_disable_start(dev);
	k_busy_wait(1000);
    ads1x9x_enable_start(dev);// Set to High
    k_busy_wait(1000);
	ads1x9x_disable_start(dev);
	k_busy_wait(1000);
    ads1x9x_write_cmd (dev, START_);          // Send 0x08 to the ADS1x9x
    k_busy_wait(1000);
	ads1x9x_write_cmd (dev, STOP);
	k_busy_wait(5000);

	ads1x9x_write_cmd (dev, SDATAC);
	k_busy_wait(10000);

	ads1x9x_read_all_reg(dev,ads1x9xregval);
	printk("regs value is :\r\n");
	for(u8_t i = 0;i < 12;i++)
		printk("%02X ",ads1x9xregval[i]);
	printk("\r\n");
   	ads1x9x_write_all_default_regs(dev);
    ads1x9x_read_all_reg(dev,ads1x9xregval);
	printk("regs value is :\r\n");
	for(u8_t i = 0;i < 12;i++)
		printk("%02X ",ads1x9xregval[i]);
	printk("\r\n");
	ads1x9x_set_out_bytes();
	//ads1x9x_powerdown(dev);
#ifdef CONFIG_ADS1X9X_TRIGGER
	if (ads1x9x_trigger_mode_init(dev) < 0) {
		SYS_LOG_DBG("Cannot set up trigger mode.");
		return -EINVAL;
	}
#endif
	ads1x9x_disable_start(dev);
	k_busy_wait(1000);
	ads1x9x_write_cmd (dev, RDATAC);
	k_busy_wait(1000);
	ads1x9x_enable_start(dev);
	dev->driver_api = &ads1x9x_api;

	return 0;
}

const struct ads1x9x_device_config ads1x9x_config = {
#if defined(CONFIG_ADS1X9X_TRIGGER)
	.gpio_port = CONFIG_ADS1X9X_GPIO_DEV_NAME,
	.int_pin = CONFIG_ADS1X9X_READY_GPIO_PIN_NUM,
	.start_pin = CONFIG_ADS1X9X_START_GPIO_PIN_NUM,
	.reset_pin = CONFIG_ADS1X9X_RESET_GPIO_PIN_NUM,
#endif
};

DEVICE_INIT(ads1x9x, CONFIG_ADS1X9X_NAME, ads1x9x_init, &ads1x9x_data,
	    &ads1x9x_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY);
