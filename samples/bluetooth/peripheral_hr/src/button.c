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
#define KEYAPIN       21
#define KEYBPIN       01
#define KEYGPIN       11

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

void power_off(struct device *gpiob)
{
	gpio_pin_configure(gpiob, 10,
			GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 10, 1); // buzzer
	k_busy_wait(500000);
	gpio_pin_configure(gpiob, 10,
			GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 10, 0); // buzzer
	gpio_pin_configure(gpiob, 9,
			GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 9, 0); // LDO
	gpio_pin_configure(gpiob, 8,
			GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 8, 0); // LDO
	gpio_pin_configure(gpiob, 22,
			GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 22, 0); // LED0
	gpio_pin_configure(gpiob, 23,
			GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 23, 0); // LED1
	gpio_pin_configure(gpiob, 24,
			GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 24, 0); // LED2
	gpio_pin_configure(gpiob, 14,
			GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 14, 0); // ads start pin
	gpio_pin_configure(gpiob, 15,
			GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 15, 0); // ads reset pin
	gpio_pin_configure(gpiob, 20,
			GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 20, 0); // ads ready pin
	gpio_pin_configure(gpiob, 16,
			GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 16, 0); // ads cs pin
	gpio_pin_configure(gpiob, 17,
			GPIO_DIR_OUT); // ads mosi
	gpio_pin_write(gpiob, 17,0);
	gpio_pin_configure(gpiob, 19,
			GPIO_DIR_OUT); // ads miso
	gpio_pin_write(gpiob, 19,0);
	gpio_pin_configure(gpiob, 18,
			GPIO_DIR_OUT); // ads sclk
	gpio_pin_write(gpiob, 18,0);
	gpio_pin_configure(gpiob, 12,
			GPIO_DIR_IN); // i2c sda
	//gpio_pin_write(gpiob, 30,1);
	gpio_pin_configure(gpiob, 13,
			GPIO_DIR_IN); // i2c sda
	//gpio_pin_write(gpiob, 7,1);
	gpio_pin_configure(gpiob, KEYAPIN, GPIO_DIR_IN
			| GPIO_PUD_PULL_UP
			| GPIO_CFG_SENSE_LOW);
	nrf_gpiote_clear_port_event();
	/* Enable GPIOTE Port Event */
	nrf_gpiote_interrupt_enable(GPIOTE_INTENSET_PORT_Msk);
	NVIC_EnableIRQ(GPIOTE_IRQn);
	_system_off();
}

void button_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	printk("Button pressed at pin : %08x, %d\n", pins, k_cycle_get_32());
	if(pins & BIT(KEYAPIN))
	{
		k_busy_wait(300000);

		power_off(gpiob);
	}
}

static struct gpio_callback gpio_cb_keya;
static struct gpio_callback gpio_cb_keyb;
static struct gpio_callback gpio_cb_keyi2c;

void button(void)
{
	struct device *gpiob;

	printk("Press the user defined button on the board\n");
	gpiob = device_get_binding(PORT);
	if (!gpiob) {
		printk("error\n");
		return;
	}

	gpio_pin_configure(gpiob, KEYAPIN,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);

	gpio_init_callback(&gpio_cb_keya, button_pressed, BIT(KEYAPIN));

	gpio_add_callback(gpiob, &gpio_cb_keya);
	gpio_pin_enable_callback(gpiob, KEYAPIN);

	gpio_pin_configure(gpiob, KEYBPIN,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);

	gpio_init_callback(&gpio_cb_keyb, button_pressed, BIT(KEYBPIN));

	gpio_add_callback(gpiob, &gpio_cb_keyb);
	gpio_pin_enable_callback(gpiob, KEYBPIN);

	gpio_pin_configure(gpiob, KEYGPIN,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);

	gpio_init_callback(&gpio_cb_keyi2c, button_pressed, BIT(KEYGPIN));

	gpio_add_callback(gpiob, &gpio_cb_keyi2c);
	gpio_pin_enable_callback(gpiob, KEYGPIN);

	while (1) {
		//u32_t val = 0;

		//gpio_pin_read(gpiob, PIN, &val);
		k_sleep(SLEEP_TIME);
	}
}

K_THREAD_DEFINE(button_id, STACKSIZE, button, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
