/** @file
 *  @brief HRS Service sample
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

#include <sensor.h>
#include <device.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include "hrs.h"
#include "ecg.h"
#include "resp.h"

static struct bt_gatt_ccc_cfg hrmc_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t simulate_hrm;
static u8_t heartrate = 90;
static u8_t resprate = 0;
static u8_t hrs_blsc;
extern unsigned long stepCount; /* 步数值 */

static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 u16_t value)
{
	simulate_hrm = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static ssize_t read_blsc(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, u16_t len, u16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &hrs_blsc,
				 sizeof(hrs_blsc));
}

/* Heart Rate Service Declaration */
static struct bt_gatt_attr attrs[] = {
	BT_GATT_PRIMARY_SERVICE(BT_UUID_HRS),
	BT_GATT_CHARACTERISTIC(BT_UUID_HRS_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg, hrmc_ccc_cfg_changed),
	BT_GATT_CHARACTERISTIC(BT_UUID_HRS_BODY_SENSOR, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_blsc, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_HRS_CONTROL_POINT, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
};

static struct bt_gatt_service hrs_svc = BT_GATT_SERVICE(attrs);
u8_t ecgrawdata[2 * 10];
static u8_t rawdatacount = 0;
static void trigger_handler(struct device *ads1x9x, struct sensor_trigger *trigger)
{
	struct sensor_value val[3];
	static int loop = 0;

	if (trigger->type != SENSOR_TRIG_DATA_READY &&
	    trigger->type != SENSOR_TRIG_DELTA) {
		printk("Ecg: trigger handler: unknown trigger type.\n");
		return;
	}

	/*if (sensor_sample_fetch(ads1x9x) < 0) {
		printk("Gyro sample update error.\n");
	}*/
	if(trigger->chan == SENSOR_CHAN_ACCEL_X)
	{
		if (sensor_channel_get(ads1x9x, SENSOR_CHAN_ACCEL_X, val) < 0) {
			printk("Cannot read ads1x9x raw channels.\n");
			return;
		}
		//printk("raw\n");
		if(!(loop ++ % 5))
		{
			//printk("%d,%d,%d\n",loop,val[0].val1,val[0].val2);
			ecgrawdata[rawdatacount * 2] = val[0].val1 / 0xff;
			ecgrawdata[rawdatacount * 2 + 1] = val[0].val2 / 0xff;
			rawdatacount ++;
			if(rawdatacount >= 10)
			{
				rawdatacount = 0;
				ecg_notify();
			}
		}
		//loop ++;


	}
	else if(trigger->chan == SENSOR_CHAN_ACCEL_Y)
	{
		if (sensor_channel_get(ads1x9x, SENSOR_CHAN_ACCEL_Y, val) < 0) {
			printk("Cannot read ads1x9x heart channels.\n");
			return;
		}
		//printk("heart\n");
		heartrate = (u8_t)val[0].val1;
		hrs_notify();
	}
	else if(trigger->chan == SENSOR_CHAN_ACCEL_Z)
	{
		if (sensor_channel_get(ads1x9x, SENSOR_CHAN_ACCEL_Z, val) < 0) {
			printk("Cannot read ads1x9x resp channels.\n");
			return;
		}
		//printk("resp\n");
		resprate = (u8_t)val[0].val1;
		resp_notify(resprate);
	}
	else
	{

	}

}

void hrs_init(u8_t blsc)
{
	hrs_blsc = blsc;
	struct device *dev;
	struct sensor_trigger trig;

	printk("ADS1X9X sample application\n");
	dev = device_get_binding("ADS1X9X");

	if (!dev) {
		printk("sensor: device not found.\n");
		return;
	}

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_X;
	if (sensor_trigger_set(dev, &trig, trigger_handler) < 0) {
		printk("Ecg: cannot set trigger.\n");
		return;
	}
	bt_gatt_service_register(&hrs_svc);
}

void hrs_notify(void)
{
	static u8_t hrm[2];

	/* Heartrate measurements simulation */
	if (!simulate_hrm) {
		return;
	}

	//heartrate++;
	/*if (heartrate == 160) {
		heartrate = 90;
	}*/

	hrm[0] = 0x06; /* uint8, sensor contact */
	hrm[1] = heartrate;

	bt_gatt_notify(NULL, &attrs[1], &hrm, sizeof(hrm));
}
