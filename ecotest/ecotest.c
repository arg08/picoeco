/**
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
//#include <stdio_uart.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"

#include "board_specific.h"
#include "econet_ll.h"
#include "econet_hl.h"
#if HAS_1MHZ_BUS
#include "bus1mhz.h"
#endif

#include "fs.h"

// Hardware configuration to pass to the low level drivers
static const EcoHWConfig eco_hw =
{
	.pio_no = 0,	// Arbitrarily choose PIO unit 0
	.clk_pin = ECONET_PIN_CLOCK,	// Econet clock input pin number
	.rxd_pin = ECONET_PIN_RXD,		// Econet RxD input pin number
	.txd_pin = ECONET_PIN_TXD,		// Econet TxD output pin number
	.txen_pin = ECONET_PIN_TX_EN,	// Econet Tx enable output pin number
	.cts_pin = ECONET_PIN_CTS,		// Econet CTS (/collision detect) input pin
};

static char cmdline[100];

#define	PROMPT	"\n" PICO_BOARD ">"

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
			board_enable_clock(0);
		}
		else if (!strncmp("on", p, 2))
		{
			board_enable_clock(1);
		}
		else
		{
			unsigned rate = strtoul(p, NULL, 10);
			if (rate == 0)
			{
				// Zero, or something simply not numeric
				printf("Clock is %s\n", 
				board_clock_enabled() ? "on" : "off");
			}
			else
			{
				board_enable_clock(rate);
			}
		}
	}
	else if (!strncmp(cmd, "terminator", len))
	{
		if (!strncmp("off", p, 3))
		{
			board_enable_terminator(false);
		}
		else if (!strncmp("on", p, 2))
		{
			board_enable_terminator(true);
		}
		printf("Terminator is %s\n",
			board_terminator_enabled() ? "on" : "off");
	}
	else if (!strncmp(cmd, "peek", len))
	{
		uint32_t addr = strtoul(p, NULL, 16);
		printf("Peek %08lx = %08lx\n", addr, *(uint32_t*)addr);
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
	board_specific_init();

#if 0
// Temporary kludge for bus1mhz_rp2040 where the PICO_DEFAULT_UART in
// the board file doesn't seem to be respected by the stdio library
	stdio_uart_init_full(uart1, 2000000, PICO_DEFAULT_UART_TX_PIN, PICO_DEFAULT_UART_RX_PIN);
#endif

	printf("Ecotest\n");
	if (watchdog_caused_reboot()) printf("Watchdog reboot\n");


	econet_hl_init(&eco_hw);
	watcher_init(&eco_hw);
#if HAS_1MHZ_BUS
	bus1mhz_init();
#endif

	fs_init();
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
		// Our little fake Econet FS
		poll_fs();
	}
}
