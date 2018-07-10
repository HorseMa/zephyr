#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <board.h>
#include <string.h>
#include <sensor.h>
#include <device.h>

void mpu6050(void)
{
	struct sensor_value intensity[3];
	struct device *dev;

	printk("MPU6050 sample application\n");
	dev = device_get_binding("MPU6050");

	if (!dev) {
		printk("sensor: device not found.\n");
		return;
	}

	while (1) {

		if (sensor_sample_fetch(dev)) {
			printk("sensor_sample fetch failed\n");
		}

		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, intensity);

		printk("ambient light intensity without"
				" trigger is %d, %d, %d\n", intensity[0].val1, intensity[1].val1, intensity[2].val1);

		k_sleep(1000);
	}
}