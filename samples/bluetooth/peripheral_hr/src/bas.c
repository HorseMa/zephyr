/** @file
 *  @brief BAS Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/printk.h>
#include <misc/byteorder.h>
#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/__assert.h>
#include <board.h>
#include <sensor.h>
#include "mymath.h"
#include <adc.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#if defined(CONFIG_BOARD_NRF51_ESTACK_ECG)

#include <nrfx_adc.h>
#define ADC_DEVICE_NAME		CONFIG_ADC_0_NAME
#define ADC_RESOLUTION		10
#define ADC_GAIN		ADC_GAIN_1_3
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT
#define ADC_CHANNEL_0_INPUT	NRF_ADC_CONFIG_INPUT_3
#define ADC_CHANNEL_2_INPUT	NRF_ADC_CONFIG_INPUT_3

#elif defined(CONFIG_BOARD_NRF52_PCA10040) || \
      defined(CONFIG_BOARD_NRF52840_PCA10056)

#include <nrfx_saadc.h>
#define ADC_DEVICE_NAME		CONFIG_ADC_0_NAME
#define ADC_RESOLUTION		10
#define ADC_GAIN		ADC_GAIN_1_6
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_CHANNEL_0_INPUT	NRF_SAADC_INPUT_AIN1
#define ADC_CHANNEL_2_INPUT	NRF_SAADC_INPUT_AIN2

#else
#error "Unsupported board."
#endif

#define BUFFER_SIZE  6
static s16_t m_sample_buffer[BUFFER_SIZE];

static struct bt_gatt_ccc_cfg  blvl_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t simulate_blvl;
static u8_t battery = 100;

static const struct adc_channel_cfg m_channel_0_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = 0,
	.differential     = false,
	.input_positive   = ADC_CHANNEL_0_INPUT,
};

static struct device *adc_dev;
static float battery_v = 0.0;
static uint8_t alarm_flag = false;

static struct device *init_adc(void)
{
	int ret;
	adc_dev = device_get_binding(ADC_DEVICE_NAME);

	if (adc_dev == NULL) {
		printk("Could not get ADC_0 device\n");
		return NULL;
	}

	ret = adc_channel_setup(adc_dev, &m_channel_0_cfg);
	if(ret < 0)
	{
		printk("adc_channel_setup error!!!\n");
	}
	memset(m_sample_buffer, 0, sizeof(m_sample_buffer));

	return adc_dev;
}

/*static void check_samples(int expected_count)
{
	int i;

	printk("Samples read: ");
	for (i = 0; i < BUFFER_SIZE; i++) {
		s16_t sample_value = m_sample_buffer[i];

		printk("0x%04x ", sample_value);
	}
	printk("\n");
}*/

static void blvl_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 u16_t value)
{
	simulate_blvl = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static ssize_t read_blvl(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, u16_t len, u16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(*value));
}

/* Battery Service Declaration */
static struct bt_gatt_attr attrs[] = {
	BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
	BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_blvl, NULL, &battery),
	BT_GATT_CCC(blvl_ccc_cfg, blvl_ccc_cfg_changed),
};

static struct bt_gatt_service bas_svc = BT_GATT_SERVICE(attrs);

void bas_init(void)
{
	init_adc();

	if (!adc_dev) {
		return;
	}
	bt_gatt_service_register(&bas_svc);
}

void bas_notify(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels    = BIT(0),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};
	static int loop = 0;
	if(loop != 0)
	{
		loop ++;
		if(loop == 100)
		{
			loop = 0;
		}
		return;
	}

	ret = adc_read(adc_dev, &sequence);

	if(ret < 0)
	{
		printk("adc_read() failed\n");
	}
	battery_v = (s16_t)m_sample_buffer[0] / (float)1024 * 3 * 1.2 * 710 / 200;
	//printk("battery voltage is %f\n",battery_v);
	if(battery_v >= 4.05)
	{
		battery = 100;
	}
	else if(battery_v <= 3.35)
	{
		battery = 0;
		// system off (shutdown)
	}
	else
	{
		battery = (uint8_t)((battery_v - 3.35) / (4.05 - 3.35) * 100);
		if((battery_v <= 3.45) && (alarm_flag == false))
		{
			// 低电量报警
			alarm_flag = true;
		}
	}
	//printk("battery : %d%\n",battery);
	bt_gatt_notify(NULL, &attrs[1], &battery, sizeof(battery));
}
