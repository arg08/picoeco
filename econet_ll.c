/**
 * Econet Low-level drivers - hardware access, packet framing etc.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/pio.h"
#include "hardware/irq.h"

// Prototypes for this module
#include "econet_ll.h"

// CRC16 for frame checksums
#include "crc16.h"

// Our assembled program:
#include "econet.pio.h"

/*
	Purpose:	Exclusive handler for IRQ1  on this PIO unit
	Returns:	Nothing
	Notes:		
*/


static void __not_in_flash_func(pio_irq0_handler)(void)
{
// XXXX Current assumption of PIO0
	static uint16_t crc = 0xffff;
	static unsigned plen = 0;

	while (pio0->ints0 & PIO_IRQ0_INTS_SM0_RXNEMPTY_BITS)
	{
#if 1
printf("%08x\n", pio0->rxf[0]);
#else
		uint32_t w = pio0->rxf[0];
		if (w == 0xffffffff)
		{
			putchar('*');
			if ((plen > 1) && (crc != VALID_CRC16_FINAL)) printf("CRC?\n");
			crc = 0xffff;
			plen = 0;
		}
		else
		{
			w >>= 24;		// Get the actual data byte
			crc = crc16_add_byte(crc, w);
			plen++;
			printf("%02x", w);
		}
#endif
	}
}

/*
	Purpose:	Exclusive handler for IRQ1  on this PIO unit
	Returns:	Nothing
	Notes:		
*/

static void pio_irq1_handler(void)
{
}

// -----------------------------------------------------------------

void econet_ll_init(const EcoHWConfig *hw)
{
	pio_sm_config cfg = econet_hdlc_program_get_default_config(0);

	// Our program has to load at offset 0 in the PIO as it uses the
	// entire available space.
	// Note that if we are called multiple times for extra Econet
	// instances in the same PIO unit, this will just harmlessly
	// re-load the program over the top of the existing copy.
    pio_add_program_at_offset(hw->pio, &econet_hdlc_program, 0);

	if (econet_hdlc_program.length != 32)
	{
		printf("XXXXX code assumes program exactly 32 words long\n");
	}

	// One 'out' pin for TxD
	sm_config_set_out_pins(&cfg, hw->txd_pin, 1);
	// One side-set pin for Tx enable
	// (NB. sideset count and option flag already set by get_default_config()
	// from definition in pioasm file.)
	sm_config_set_sideset_pins(&cfg, hw->txen_pin);
	// One 'in' pin for clock
	sm_config_set_in_pins(&cfg, hw->clk_pin);
	// The conditional jump pin is RxD
	sm_config_set_jmp_pin(&cfg, hw->rxd_pin);

	// Input and output both have 8-bit thresholds, autopush and shift_right
	sm_config_set_out_shift(&cfg, true, true, 8);
	sm_config_set_in_shift(&cfg, true, true, 8);

	// XXXX CTS pin is not processed by the PIO - set up as a GPIO interrupt

	// These two output pins are consecutive on current hardware, but
	// might not be in future so we set them up individually.
	// Input pins don't need pinmux setting.
    pio_gpio_init(hw->pio, hw->txd_pin);
    pio_sm_set_consecutive_pindirs(hw->pio, hw->sm, hw->txd_pin, 1, true);
    pio_gpio_init(hw->pio, hw->txen_pin);
    pio_sm_set_consecutive_pindirs(hw->pio, hw->sm, hw->txen_pin, 1, true);

    // Load our configuration, and jump to the start of the program
    pio_sm_init(hw->pio, hw->sm, econet_hdlc_offset_rxentrypoint, &cfg);

	// Hook the interrupt handlers
	irq_set_exclusive_handler((hw->pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0,
		pio_irq0_handler);
	irq_set_exclusive_handler((hw->pio == pio0) ? PIO0_IRQ_1 : PIO1_IRQ_1,
		pio_irq1_handler);
	// We leave the interrupts permanently enabled in the NVIC,
	// but mask individual sources from time to time.
	irq_set_enabled((hw->pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0, true);
	irq_set_enabled((hw->pio == pio0) ? PIO0_IRQ_1 : PIO1_IRQ_1, true);

	// Enable the FIFO Rx interrupt
	// These enable bits are in order of sm number
	hw->pio->inte0 |= (PIO_IRQ0_INTE_SM0_RXNEMPTY_BITS << hw->sm);

	// Initialise the CRC table - doesn't mind being called more than once.
	init_crc16_table();

    // Set the state machine running
    pio_sm_set_enabled(hw->pio, hw->sm, true);
}

