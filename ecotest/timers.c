/**
 * Econet Low-level drivers - timing functions
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"
#include "hardware/structs/iobank0.h"

// Prototypes for this module
#include "econet_ll.h"

typedef struct workspace_1hz
{
	repeating_timer_t api_timer;	// Workspace for the system timer API
	unsigned clock_hz;				// Measured clock speed
	unsigned jam_secs;				// Number of seconds line has been jammed
	uint8_t clk_pin;				// Pin number for CLKIN
	uint8_t txen_pin;				// Pin number for TX_EN
} Workspace1Hz;

static Workspace1Hz ws0;		// Potentially one instance per interface


/*
	Purpose:	Callback function invoked by system timer at 1Hz
	Returns:	true (so as to get called again).
	Notes:		Also feeds the system watchdog.
*/

static bool poll_1hz(repeating_timer_t *rt)
{
	Workspace1Hz *ws = rt->user_data;
	uint32_t mask;
	
	ws->clock_hz = pwm_get_counter(pwm_gpio_to_slice_num(ws->clk_pin)) * 16;
	pwm_set_counter(pwm_gpio_to_slice_num(ws->clk_pin), 0);
//	printf("Clk pin %u hz %u\n", ws->clk_pin, ws->clock_hz);

	// Line jammed: if pin is currently low, or there has been an
	// edge in the last second, it's not jammed.

	// Edge detect and level detect is provided in the GPIO interrupt system.
	// The 32 GPIOs are spread across 4 registers with 4 bits per pin.
	mask = (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_LEVEL_LOW) 
		<< ((ws->txen_pin % 8) * 4);
	if (iobank0_hw->intr[ws->txen_pin / 8] & mask)
	{
		// interrupt status register is write-1-to-clear
		// Can't clear level bit, but it doesn't do any harm.
		iobank0_hw->intr[ws->txen_pin / 8] = mask;
		ws->jam_secs = 0;
	}
	else
	{
		ws->jam_secs++;

		printf("Line Jammed %u secs\n", ws->jam_secs);
		if (ws->jam_secs > 10)
		{
			printf("rebooting\n");
			watchdog_reboot(0,0,100);
		}
	}

	// Toggle LED
	gpio_xor_mask(1 << PICO_DEFAULT_LED_PIN);

	// Tickle the system watchdog
	watchdog_update();
	return true;
}

/*
	Purpose:	Set up watcher 1Hz polling for linejammed/no clock
	Returns:	true if setup OK, false if failed
*/

bool watcher_init(const EcoHWConfig *hw)
{
	Workspace1Hz *ws = &ws0;

	pwm_config cfg;
	// Set up the PWMs to count clock and tx enable edges.

	printf("Watcher pins %u %u\n", hw->clk_pin, hw->txen_pin);

	// Since we are only monitoring the txen pin which is pinmuxed to PIO
	// for output, don't call gpio_set_function() on the txen.
	// Conversely, clock has to be pinmuxed to PWM otherwise PWM won't
	// use the input, but PIO can spy on inputs regardless of muxing.
	gpio_set_function(hw->clk_pin, GPIO_FUNC_PWM);

	// Going to use the LED for flashing
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	// Save the pin numbers to use in the callback fn
	ws->clk_pin = hw->clk_pin;
	ws->txen_pin = hw->txen_pin;

	// TX_EN - detect rising edges so the polling function can see
	// if it has been low in the past second.  Note that we are just
	// setting up the interrupt status so it can be polled, we don't
	// actually take any interrupts (which would be too frequent).
	// In fact, this needs no configuration at all since the INTR
	// register tracks all the possible interrupts (rising/falling/level)
	// regardless of enable settings.

	// Clock, want to count the number of edges in a second.
	// Result has to fit in 16 bits, so divide by 16
	cfg = pwm_get_default_config();
	pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
	pwm_config_set_clkdiv(&cfg, 16);
	pwm_init(pwm_gpio_to_slice_num(hw->clk_pin), &cfg, true);

	if (!add_repeating_timer_ms(1000, poll_1hz, ws, &ws->api_timer))
	{
		printf("Failed add_repeating_timer_ms()\n");
		return false;
	}

	// Enable system watchdog with 2 sec timeout
	watchdog_enable(2000, 1);

	return true;
}
