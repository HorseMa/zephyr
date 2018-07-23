#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <board.h>
#include <string.h>
#include <sensor.h>
#include "mymath.h"
#include <adc.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

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

static const struct adc_channel_cfg m_channel_0_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = 0,
	.differential     = false,
	.input_positive   = ADC_CHANNEL_0_INPUT,
};

static struct device *init_adc(void)
{
	int ret;
	struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);

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

static void check_samples(int expected_count)
{
	int i;

	printk("Samples read: ");
	for (i = 0; i < BUFFER_SIZE; i++) {
		s16_t sample_value = m_sample_buffer[i];

		printk("0x%04x ", sample_value);
	}
	printk("\n");
}

void battery(void)
{
	int ret;
	float battery_v = 0.0;
	const struct adc_sequence sequence = {
		.channels    = BIT(0),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	struct device *adc_dev = init_adc();

	if (!adc_dev) {
		return;
	}
	ret = adc_read(adc_dev, &sequence);

	if(ret < 0)
	{
		printk("adc_read() failed\n");
	}
	battery_v = (s16_t)m_sample_buffer[0] / (float)1024 * 3 * 1.2 * 710 / 200;
	printk("battery voltage is %f\n",battery_v);
	check_samples(1);
}

K_THREAD_DEFINE(battery_id, STACKSIZE, battery, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
