#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <board.h>
#include <string.h>
#include <sensor.h>
#include <device.h>
#include "mymath.h"
#include <math.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define MAX30205_TEMP_STEP		0.00390625f

void max30205(void)
{
	struct sensor_value temp;
	struct device *dev = device_get_binding("MAX30205");

	if (dev == NULL) {
		printk("Could not get MAX30205 device\n");
		return;
	}

	while (1) {
		if (sensor_sample_fetch(dev) < 0) {
			printk("Sensor sample update error\n");
			return;
		}

		if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
			printk("Cannot read MAX30205 temperature channel\n");
			return;
		}

		/* display temperature */
		printk("Temperature:%d,%fC\n", temp.val1,(float)temp.val1 * MAX30205_TEMP_STEP);

		k_sleep(2000);
	}
}

K_THREAD_DEFINE(max30205_id, STACKSIZE, max30205, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
