#include <zephyr.h>
#include <board.h>
#include <device.h>
#include <gpio.h>
#include <misc/util.h>
#include <misc/printk.h>
#include <nrf_power.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define PORT      "GPIO_0"
#define PIN       0
/* Sleep time */
#define SLEEP_TIME	500
#define PULL_UP SW0_GPIO_PIN_PUD
#define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)
#define GPIO_CFG_SENSE_LOW (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)

extern int nrf_gpiote_interrupt_enable(uint32_t mask);
extern void nrf_gpiote_clear_port_event(void);

static void _system_off(void)
{
	nrf_power_system_off();
}

void button_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	printk("Button pressed at %d\n", k_cycle_get_32());
	k_busy_wait(300000);
	gpio_pin_configure(gpiob, 9,
			   GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 9, 0); // buzzer
	gpio_pin_configure(gpiob, 19,
			   GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 19, 0); // LDO
	gpio_pin_configure(gpiob, 10,
			   GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 10, 0); // LED0
	gpio_pin_configure(gpiob, 11,
			   GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 11, 0); // LED1
	gpio_pin_configure(gpiob, 21,
			   GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 21, 0); // ads start pin
	gpio_pin_configure(gpiob, 22,
			   GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 22, 0); // ads reset pin
	gpio_pin_configure(gpiob, 23,
			   GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 23, 0); // ads ready pin
	gpio_pin_configure(gpiob, 24,
			   GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 24, 0); // ads cs pin
	gpio_pin_configure(gpiob, 25,
			   GPIO_DIR_OUT); // ads mosi
	gpio_pin_write(gpiob, 25,0);
	gpio_pin_configure(gpiob, 28,
			   GPIO_DIR_OUT); // ads miso
	gpio_pin_write(gpiob, 28,0);
	gpio_pin_configure(gpiob, 29,
			   GPIO_DIR_OUT); // ads sclk
	gpio_pin_write(gpiob, 29,0);
	gpio_pin_configure(gpiob, 30,
			   GPIO_DIR_IN); // i2c sda
	//gpio_pin_write(gpiob, 30,1);
	gpio_pin_configure(gpiob, 7,
			   GPIO_DIR_IN); // i2c sda
	//gpio_pin_write(gpiob, 7,1);
	gpio_pin_configure(gpiob, PIN, GPIO_DIR_IN
			| GPIO_PUD_PULL_UP
			| GPIO_CFG_SENSE_LOW);
	nrf_gpiote_clear_port_event();
	/* Enable GPIOTE Port Event */
	nrf_gpiote_interrupt_enable(GPIOTE_INTENSET_PORT_Msk);
	NVIC_EnableIRQ(GPIOTE_IRQn);
	printk("Button pressed at %d\n", k_cycle_get_32());
	_system_off();
}

static struct gpio_callback gpio_cb;

void button(void)
{
	struct device *gpiob;

	printk("Press the user defined button on the board\n");
	gpiob = device_get_binding(PORT);
	if (!gpiob) {
		printk("error\n");
		return;
	}

	gpio_pin_configure(gpiob, PIN,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);

	gpio_init_callback(&gpio_cb, button_pressed, BIT(PIN));

	gpio_add_callback(gpiob, &gpio_cb);
	gpio_pin_enable_callback(gpiob, PIN);

	while (1) {
		u32_t val = 0;

		gpio_pin_read(gpiob, PIN, &val);
		k_sleep(SLEEP_TIME);
	}
}

K_THREAD_DEFINE(button_id, STACKSIZE, button, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
