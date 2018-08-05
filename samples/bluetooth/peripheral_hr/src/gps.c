/** @file
 *  @brief GPS Service sample
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
#include "gps.h"
#include <sensor.h>
#include <device.h>
#include <math.h>

#ifndef TRUE
#define TRUE     1
#endif
#ifndef FALSE
#define FALSE    0
#endif
#define BT_UUID_GPS                       BT_UUID_DECLARE_128(0x00, 0x00, 0x8B, 0x98, 0x42, 0xC2, 0x34, 0xA0,0x5C, 0x46, 0x84, 0x38, 0x88, 0x89, 0x86, 0x75)
#define BT_UUID_GPS_MEASUREMENT           BT_UUID_DECLARE_128(0x00, 0x00, 0x8B, 0x98, 0x42, 0xC2, 0x34, 0xA0,0x5C, 0x46, 0x84, 0x38, 0x66, 0x77, 0x86, 0x75)

static struct bt_gatt_ccc_cfg hrmc_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t simulate_hrm;
static u8_t gps_blsc;

static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 u16_t value)
{
	simulate_hrm = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

/* Heart Rate Service Declaration */
static struct bt_gatt_attr attrs[] = {
	BT_GATT_PRIMARY_SERVICE(BT_UUID_GPS),
	BT_GATT_CHARACTERISTIC(BT_UUID_GPS_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg, hrmc_ccc_cfg_changed),
};

static struct bt_gatt_service gps_svc = BT_GATT_SERVICE(attrs);

void gps_init(u8_t blsc)
{
	gps_blsc = blsc;
	int ret;

	ret = bt_gatt_service_register(&gps_svc);
	if(ret < 0)
	{
		printk("register gps Service err!\n");
	}
}

void gps_notify(char *data)
{
	//static u8_t location[100] = {0};
	/* Gps measurements simulation */
	if (!simulate_hrm) {
		return;
	}
	//memset(location,0,100);
	//strcpy(location,"38.6518,104.07642");
	bt_gatt_notify(NULL, &attrs[1], data, strlen(data));
}

