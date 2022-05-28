/**
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/structs/pwm.h"
#include "hardware/pwm.h"
#include <hardware/structs/spi.h>
#include <hardware/spi.h>
#include "hardware/watchdog.h"

#include "econet_ll.h"
#include "econet_hl.h"


#if HAS_SPI_CTRL_REG
static uint8_t ctrl_reg_copy = 0;	// Copy of value sent to SPI shift reg

static void set_ctrl_bit(unsigned bitmask, bool value)
{
	if (value) ctrl_reg_copy |= bitmask;
	else ctrl_reg_copy &= ~bitmask;
	spi_get_hw(BOARD_CTRL_SPI)->dr = ctrl_reg_copy;
//	spi_write_blocking(BOARD_CTRL_SPI, &ctrl_reg_copy, 1);
}
#endif	// HAS_SPI_CTRL_REG

void board_toggle_led(void)
{
#if defined(BOARD_CTRL_BIT_LED)
	set_ctrl_bit(BOARD_CTRL_BIT_LED, (ctrl_reg_copy & BOARD_CTRL_BIT_LED) == 0);
#elif defined(PICO_DEFAULT_LED_PIN)
	gpio_xor_mask(1 << PICO_DEFAULT_LED_PIN);
#endif
}

static void set_clk_ena(bool en)
{
#if defined(ECONET_PIN_CLK_EN)
		gpio_put(ECONET_PIN_CLK_EN, en);
#elif defined(BOARD_CTRL_BIT_CLK_EN)
	set_ctrl_bit(BOARD_CTRL_BIT_CLK_EN, en);
#else
#error clock enable not defined
#endif
}

void board_enable_terminator(bool en)
{
#if defined(ECONET_PIN_TERM_EN)
	gpio_put(ECONET_PIN_TERM_EN, en);
#elif defined(BOARD_CTRL_BIT_TERM_EN)
	set_ctrl_bit(BOARD_CTRL_BIT_TERM_EN, en);
#else
#error term enable not defined
#endif
}

bool board_terminator_enabled(void)
{
#if defined(ECONET_PIN_TERM_EN)
	return gpio_get(ECONET_PIN_TERM_EN);
#elif defined(BOARD_CTRL_BIT_TERM_EN)
	return ((ctrl_reg_copy & BOARD_CTRL_BIT_TERM_EN) != 0);
#else
	return false;
#endif
}

bool board_clock_enabled(void)
{
#if defined(ECONET_PIN_CLK_EN)
	return gpio_get(ECONET_PIN_CLK_EN);
#elif defined(BOARD_CTRL_BIT_CLK_EN)
	return ((ctrl_reg_copy & BOARD_CTRL_BIT_CLK_EN) != 0);
#else
	return false;
#endif
}




// Takes rate in Hz or 0 for off, 1 to enable without changing speed
void board_enable_clock(unsigned rate)
{
	if (rate == 0)
	{
		set_clk_ena(0);
	}
	else if (rate == 1)
	{
		set_clk_ena(1);
	}
	else
	{
		uint32_t period = clock_get_hz(clk_sys) / rate;
		pwm_config cfg = pwm_get_default_config();
		// Set up the clock and turn it on
		printf("Setting clock to %u Hz (%u)\n", rate, (unsigned)period);
		pwm_config_set_wrap(&cfg, period);

		pwm_init(pwm_gpio_to_slice_num(ECONET_PIN_CLKOUT), &cfg, true);
		// 1:1 clocks are fine for short networks.
		pwm_set_gpio_level(ECONET_PIN_CLKOUT, period/2);

		set_clk_ena(1);
	}
}


void board_specific_init()
{

	// Some boards have two UART pins, some only Tx, some none at all.
#if defined(PICO_DEFAULT_UART_TX_PIN)
	stdio_uart_init_full(uart0, 2000000,PICO_DEFAULT_UART_TX_PIN,
#if defined(PICO_DEFAULT_UART_RX_PIN)
		PICO_DEFAULT_UART_RX_PIN);
#else
		-1);
#endif
#endif

	// XXXX presently to be combined with our own USB handling.
	stdio_usb_init();

	// Since we are only monitoring the txen pin which is pinmuxed to PIO
	// for output, don't call gpio_set_function() on the txen.
	// Conversely, clock has to be pinmuxed to PWM otherwise PWM won't
	// use the input, but PIO can spy on inputs regardless of muxing.
	gpio_set_function(ECONET_PIN_CLOCK, GPIO_FUNC_PWM);
	// The clock out pin has to be PWM when enabled; doesn't matter
	// otherwise.  Fortunately, this means the pinmuxing is constant
	// regardless of separate or shared clock in/out pins.
	gpio_set_function(ECONET_PIN_CLKOUT, GPIO_FUNC_PWM);

#if defined(PICO_DEFAULT_LED_PIN)
	// Going to use the LED for flashing
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

#if HAS_SPI_CTRL_REG
	// This board has control bits on an SPI-driven shift register
	gpio_set_function(PIN_BOARD_CTRL_TX, GPIO_FUNC_SPI);
	gpio_set_function(PIN_BOARD_CTRL_CLK, GPIO_FUNC_SPI);
	gpio_set_function(PIN_BOARD_CTRL_CS, GPIO_FUNC_SPI);

	// Clock wants to go as fast as reasonably possible.
	// If using 74HC594, Fmax is 4MHz at 2V or 20MHz at 4.5V so
	// about 12MHz at 3V3.  Initially using 74AHC which supports 70MHz(!).
	// 74LVC could do 130MHz.
	// So 8MHz is conservative, but still gets our byte out in a microsecond
	// and we are unlikely to do updates quicker than that.
	spi_init(BOARD_CTRL_SPI, 8000000);
	spi_set_format(BOARD_CTRL_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
	// XXX Note on the prototype where we have >16 bits of shift register
	// we can't use the hardware CS and need to drive CS as a GPIO.
	// But for Econet use where we only care about the first 8 bits it is OK.
#endif

#if defined(ECONET_PIN_CLK_EN)
	gpio_init_mask( (1<<ECONET_PIN_CLK_EN) | (1<<ECONET_PIN_TERM_EN));
	gpio_set_dir(ECONET_PIN_CLK_EN, GPIO_OUT);
	gpio_set_dir(ECONET_PIN_TERM_EN, GPIO_OUT);
#endif
}
