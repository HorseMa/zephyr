/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
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

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "hrs.h"
#include "dis.h"
#include "bas.h"
#include "htp.h"
#include "step.h"
#include "ecg.h"
#include "pid.h"
#include "resp.h"
#include "gps.h"

#define PORT      "GPIO_0"

struct bt_conn *default_conn;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x0d, 0x18, 0x0f, 0x18, 0x05, 0x18, 0x09, 0x18),
};

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
	} else {
		default_conn = bt_conn_ref(conn);
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	hrs_init(0x01);
	ecg_init(0x01);
	resp_init(0x01);
	gps_init(0x01);
	pid_init(0x01);
	step_init(0x01);
	htp_init(0x01);
	bas_init();
	dis_init(CONFIG_SOC, "Manufacturer");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

/*#define STACK_SIZE		1024
K_THREAD_STACK_DEFINE(stack, STACK_SIZE);
static struct k_thread thread_data;
void mpu6050(void);
*/
extern void detectbattary(void);

void main(void)
{
	int err;
	struct device *gpiob;
	u32_t value;

	printk("Press the user defined button on the board\n");
	gpiob = device_get_binding(PORT);
	if (!gpiob) {
		printk("error\n");
		return;
	}

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	/*k_thread_create(&thread_data, stack, STACK_SIZE,
		(k_thread_entry_t) mpu6050,
		NULL, NULL, NULL, K_PRIO_COOP(7), 0, 0);*/
	/* Implement notification. At the moment there is no suitable way
	 * of starting delayed work so we do it here
	 */
	while (1) {
		k_sleep(50);

		/* Heartrate measurements simulation */
		//hrs_notify();
		gpio_pin_read(gpiob, 11,&value);
		if(value == 0)
		{
			step_notify();
			htp_notify();
		}
		detectbattary();
		/* Battery level simulation */
		bas_notify();
	}
}
