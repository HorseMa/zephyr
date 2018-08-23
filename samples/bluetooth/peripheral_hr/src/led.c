/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <board.h>
#include <string.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* Change this if you have an LED connected to a custom port */
#ifndef LED0_GPIO_CONTROLLER
#define LED0_GPIO_CONTROLLER 	LED0_GPIO_PORT
#endif
#ifndef LED1_GPIO_CONTROLLER
#define LED1_GPIO_CONTROLLER 	LED1_GPIO_PORT
#endif
#ifndef LED2_GPIO_CONTROLLER
#define LED2_GPIO_CONTROLLER 	LED2_GPIO_PORT
#endif

#define PORT0	 LED0_GPIO_CONTROLLER
#define PORT1	 LED1_GPIO_CONTROLLER
#define PORT2	 LED2_GPIO_CONTROLLER


/* Change this if you have an LED connected to a custom pin */
#define LED0    LED0_GPIO_PIN
#define LED1    LED1_GPIO_PIN
#define LED2    LED2_GPIO_PIN

struct printk_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	u32_t led;
	u32_t cnt;
};
bool bt_connect_flag = false;
bool gps_work_flag = false;
bool ecg_lead_flag = false;

K_FIFO_DEFINE(printk_fifo);

void blink(const char *port, u32_t sleep_ms, u32_t led, u32_t id)
{
	int cnt = 0;
	struct device *gpio_dev;

	gpio_dev = device_get_binding(port);
	__ASSERT_NO_MSG(gpio_dev != NULL);

	gpio_pin_configure(gpio_dev, led, GPIO_DIR_OUT);

	while (1) {
		gpio_pin_write(gpio_dev, led, cnt % 2);
		if(cnt % 2)
		{
			k_sleep(100);
		}
		else
		{
			if(((bt_connect_flag) && (led == LED0)) || ((gps_work_flag) && (led == LED1)) || ((ecg_lead_flag) && (led == LED2)))
			{
				k_sleep(sleep_ms * 2);
			}
			else
			{
				k_sleep(sleep_ms);
			}
		}
		cnt++;
	}
}

void blink1(void)
{
	blink(PORT0, 1000, LED0, 0);
}

void blink2(void)
{
	blink(PORT1, 1000, LED1, 1);
}

void blink3(void)
{
	blink(PORT2, 1000, LED2, 2);
}

K_THREAD_DEFINE(blink1_id, STACKSIZE, blink1, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(blink2_id, STACKSIZE, blink2, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(blink3_id, STACKSIZE, blink3, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
