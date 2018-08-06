/** @file
 *  @brief PID Service sample
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
#include "pid.h"
#include <sensor.h>
#include <device.h>
#include <math.h>

#ifndef TRUE
#define TRUE     1
#endif
#ifndef FALSE
#define FALSE    0
#endif
#define BT_UUID_PID                       BT_UUID_DECLARE_128(0x00, 0x00, 0x8B, 0x98, 0x42, 0xC2, 0x34, 0xA0,0x5C, 0x46, 0x84, 0x38, 0x12, 0x28, 0x86, 0x75)
#define BT_UUID_PID_MEASUREMENT           BT_UUID_DECLARE_128(0x00, 0x00, 0x8B, 0x98, 0x42, 0xC2, 0x34, 0xA0,0x5C, 0x46, 0x84, 0x38, 0x28, 0x12, 0x86, 0x75)

static struct bt_gatt_ccc_cfg hrmc_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t simulate_hrm;
static u8_t pid_blsc;

static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 u16_t value)
{
	simulate_hrm = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

/* Heart Rate Service Declaration */
static struct bt_gatt_attr attrs[] = {
	BT_GATT_PRIMARY_SERVICE(BT_UUID_PID),
	BT_GATT_CHARACTERISTIC(BT_UUID_PID_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg, hrmc_ccc_cfg_changed),
};

static struct bt_gatt_service pid_svc = BT_GATT_SERVICE(attrs);

void pid_init(u8_t blsc)
{
	pid_blsc = blsc;
	int ret;

	ret = bt_gatt_service_register(&pid_svc);
	if(ret < 0)
	{
		printk("register pid Service err!\n");
	}
}

extern u8_t pidrawdata[];
void pid_notify(void)
{
	/* Heartrate measurements simulation */
	if (!simulate_hrm) {
		return;
	}

	bt_gatt_notify(NULL, &attrs[1], &pidrawdata, 20 * 2);
}

