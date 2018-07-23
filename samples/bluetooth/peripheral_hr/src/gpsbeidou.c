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
#include <sensor.h>
#include <device.h>
#include "mymath.h"
#include <math.h>
#include <string.h>
#include <console.h>
#include <unistd.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

void gpsbeidou(void)
{
	#if 0
	struct device *gpio_dev;
	unsigned char *p,*pend;
	float lat;
	static unsigned char tempstr[100] = {0};

	gpio_dev = device_get_binding("GPIO_0");
	__ASSERT_NO_MSG(gpio_dev != NULL);

	gpio_pin_configure(gpio_dev, 14,
			   GPIO_DIR_IN);
	gpio_pin_configure(gpio_dev, 17,
			   GPIO_DIR_OUT |  GPIO_PUD_PULL_UP);
	while (1) {
		u32_t val = 0;
		gpio_pin_read(gpio_dev, 14, &val);
		if(val == 1)
		{
			break;
		}
		else
		{
			k_sleep(200);
		}
	}

	console_getline_init();
	gpio_pin_write(gpio_dev, 17,1);

	while (1) {
		char *s = console_getline();
		p = strstr(s,"GGA");
		if(p)
		{
			printk("got one!!!\n");
			memset(tempstr,0,100);
			p = strstr(p,",");
			p = strstr(p + 1,",");
			pend = strstr(p + 1,",");
			memcpy(tempstr,p + 1,pend - p - 1);
			printk("%s\n",tempstr);
			//lat = atof(p + 1);
			//atoi()
			//sscanf(tempstr,"%f",&lat);	// 纬度
			//printk("lat = %f\n",lat);
			memset(tempstr,0,100);
			p = strstr(pend + 1,",");
			p = p + 1;
			pend = strstr(p,",");
			memcpy(tempstr,p,pend - p); // 经度
			printk("%s\n",tempstr);
		}
	}
	#endif
}

K_THREAD_DEFINE(gpsbeidou_id, STACKSIZE, gpsbeidou, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
