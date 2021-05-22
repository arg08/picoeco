/**
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/structs/pwm.h"
#include "hardware/pwm.h"

#include "econet_ll.h"

// Hardware configuration to pass to the low level drivers
static const EcoHWConfig eco_hw =
{
	.pio_no = 0,	// Arbitrarily choose PIO unit 0
	.clk_pin = 9,	// Econet clock input pin number
	.rxd_pin = 11,	// Econet RxD input pin number
	.txd_pin = 12,	// Econet TxD output pin number
	.txen_pin = 13,	// Econet Tx enable output pin number
	.cts_pin = 10,	// Econet CTS (/collision detect) input pin number
};

// These are for the auxiliary clock/terminate functions,
// not strictly part of the main econet interface and so
// implemented in this file rather than econet_ll.
#define	ECONET_PIN_CLKOUT	14	// Output clock
#define	ECONET_PIN_CLK_EN	15	// Active high enable for clock generator
#define	ECONET_PIN_TERM_EN	16	// Active high terminator enable

static char cmdline[100];
#define	PROMPT	"\npicoeco>"



// -----------------------------------------------------------------

static const char *skip_sp(const char *p)
{
	while (*p == ' ')
		p++;
	return p;
}
static const char *find_sp(const char *p)
{
	while ((*p != ' ') && (*p != '\0'))
		p++;
	return p;
}

static const char *skip_to_mandatory_param(const char *p)
{
	p = skip_sp(find_sp(p));
	if (!*p) printf("Mandatory parameter missing\n");
	return p;
}



static void execute_cmd(const char *cmd)
{
	const char *p;
	int len;

	cmd = skip_sp(cmd);
	p = find_sp(cmd);
	len = p - cmd;
	p = skip_sp(p);		// Find first param (if any)

	printf("\n");
	if (len == 0) return;	// Null cmd does nothing

	if (!strncmp(cmd, "clock", len))
	{
		if (!strncmp("off", p, 3))
		{
			gpio_put(ECONET_PIN_CLK_EN, 0);
		}
		else if (!strncmp("on", p, 2))
		{
			gpio_put(ECONET_PIN_CLK_EN, 1);
		}
		else
		{
			unsigned rate = strtoul(p, NULL, 10);
			if (rate == 0)
			{
				// Zero, or something simply not numeric
				printf("Clock is %s\n", 
				gpio_get(ECONET_PIN_CLK_EN) ? "on" : "off");
			}
			else
			{
				uint32_t period = clock_get_hz(clk_sys) / rate;
				pwm_config cfg = pwm_get_default_config();
				// Set up the clock and turn it on
				printf("Setting clock to %u Hz (%u)\n", rate, period);
				pwm_config_set_wrap(&cfg, period);

				pwm_init(pwm_gpio_to_slice_num(ECONET_PIN_CLKOUT), &cfg, true);
				// 1:1 clocks are fine for short networks.
				pwm_set_gpio_level(ECONET_PIN_CLKOUT, period/2);
				gpio_put(ECONET_PIN_CLK_EN, 1);
			}
		}
	}
	else if (!strncmp(cmd, "terminator", len))
	{
		if (!strncmp("off", p, 3))
		{
			gpio_put(ECONET_PIN_TERM_EN, 0);
		}
		else if (!strncmp("on", p, 2))
		{
			gpio_put(ECONET_PIN_TERM_EN, 1);
		}
		printf("Terminator is %s\n",
			gpio_get(ECONET_PIN_TERM_EN) ? "on" : "off");
	}
	else if (!strncmp(cmd, "peek", len))
	{
		uint32_t addr = strtoul(p, NULL, 16);
		printf("Peek %08x = %08x\n", addr, *(uint32_t*)addr);
	}
	else if (!strncmp(cmd, "assert", len))
	{
		int a, b;
		a = atoi(p);
		b = atoi(skip_sp(find_sp(p)));
		printf("Asserting %d = %d\n", a, b);

		assert(a != b);
	}
	else if (!strncmp(cmd, "bootrom", 7))
	{
		printf("Resetting to bootrom.\n");
		reset_usb_boot(0, 0);
	}
	else
	{
		printf("Commands:\n"
			" clock [<speed> | off | on ]\n"
			" terminator [ off | on ]\n"
			" peek <hexaddr>\n"
			" bootrom\n"
			);
	}
}


int main()
{
	char *cmdptr;
	stdio_init_all();

	econet_hl_init(&eco_hw);
	watcher_init(&eco_hw);

	// The remaining pins aren't part of the econet interface, so we
	// set them up here.
	gpio_set_function(ECONET_PIN_CLKOUT, GPIO_FUNC_PWM);
	// This init also sets the output value low.
	gpio_init_mask( (1<<ECONET_PIN_CLK_EN) | (1<<ECONET_PIN_TERM_EN));
	gpio_set_dir(ECONET_PIN_CLK_EN, GPIO_OUT);
	gpio_set_dir(ECONET_PIN_TERM_EN, GPIO_OUT);


	printf(PROMPT);
	cmdptr = cmdline;
	for (;;)
	{
		int c = getchar_timeout_us(100);
		if (c == '\r')
		{
			*cmdptr = '\0';
			execute_cmd(cmdline);
			cmdptr = cmdline;
			printf(PROMPT);
		}
		else if (c == 8)	// Backspace
		{
			if (cmdptr > cmdline)
			{
				cmdptr--;
				printf("\x08 \x08");	// Rub out 1 char
			}
			else putchar(7);
		}
		else if (c > 0)
		{
			if (cmdptr < (cmdline + sizeof(cmdline) - 1))
			{
				*cmdptr++ = c;
				putchar(c);
			}
			else putchar(7);
		}
	}
}
