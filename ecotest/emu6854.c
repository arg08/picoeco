/**
 * Emulation of 6854 presented on the 1MHz Bus and controlling the econet directly.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

// Enables sanity checking for conditions that should not be possible.
#define	SANITY_CHECKS	1

// Access to hardware
#include "board_specific.h"

// Prototypes for this module
#include "emu6854.h"

// CRC16 for frame checksums
#include "crc16.h"

// Our assembled program:
#include "econet.pio.h"

// Which SM of the 4 within the unit?  This will become a macro to
// accesse an assigned SM number when multi-channel is implemented;
// for now just hard-wire it but use this define to save having to hunt
// down the uses later.
#define	SM_NO	0

typedef enum
{
	ST_IDLE = 0,		// Idle state.
	ST_RX = 1,			// Receiving frame (flag already rx)
	ST_SKIPIDLE = 2,	// Skipping unwanted data until line goes idle
	ST_TXWAIT = 3,		// Transmitting flag fill getting ready to Tx
	ST_TX = 4,			// Transmitting frame data
	ST_RXTURN = 5,		// Waiting for reply flag after we sent a frame
} IFState;

// Workspace for one econet interface
typedef struct
{
	uint16_t rx_crc;		// Running copy of CRC for both Tx and Rx
	uint16_t tx_crc;		// Running copy of CRC for both Tx and Rx
	bool line_idle;
	bool tx_mode;
} Em6854WkSpace;

static Em6854WkSpace workspace0;	// In principle, one of these per interface

// ---------------------------------------------------------------------------
// Interrupt handler and support functions

/*
	Purpose:	Reset hardware and state machines after internal error
	Returns:	Nothing, now in ST_SKIPIDLE
	Notes:		Called after 'shouldn't happen' events, typically
				caused by excess interrupt latency.
				Not marked 'not_in_flash_function' since things
				have already gone wrong by that point - this is effectively
				initialisation code.
*/

static void reset_interface(PIO pio, Em6854WkSpace *ws)
{
    pio_sm_set_enabled(pio, SM_NO, false);
	pio_sm_clear_fifos(pio, SM_NO);
	pio_sm_exec(pio, SM_NO, pio_encode_jmp(econet_hdlc_offset_rxentrypoint));
	pio_sm_set_wrap(pio, SM_NO, econet_hdlc_wrap_target, econet_hdlc_wrap);
	// Disable Tx interrupts
	hw_clear_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_TXNFULL_BITS << SM_NO));
    pio_sm_set_enabled(pio, SM_NO, true);
}


/*
	Purpose:	Handle received flag event (RF)
	Returns:	Nothing
	Notes:		Called from PIO Rx FIFO interrupt - should get
				inlined into interrupt handler.
*/

static inline void __not_in_flash_func(event_rf)(PIO pio, Em6854WkSpace *ws)
{
//putchar('*');
				{
					// Turn the line around.
					// Note that we do this state transition first, as the
					// higher layer might invoke econet_ll_ack_rx() etc.
					// directly from the callback and so cause another
					pio_sm_set_wrap(pio, SM_NO,
						econet_hdlc_offset_tx_wrap_target,
						econet_hdlc_offset_tx_wrap_source);
					pio_sm_exec(pio, SM_NO,
						pio_encode_jmp(econet_hdlc_offset_tx_zero_bit));
						// XXXX tx_zero_bit probably not necessary given
						// XXX we now transmit a whole extra flag.
				}
}

/*
	Purpose:	Handle started-to-transmit-flag event (TF)
	Returns:	Nothing
	Notes:		Called from PIO Rx FIFO interrupt - should get
				inlined into interrupt handler.
*/

static inline void __not_in_flash_func(event_tf)(PIO pio, Em6854WkSpace *ws)
{
//putchar('^');
}

/*
	Purpose:	Handle received data event (D)
	Returns:	Nothing
	Notes:		Called from PIO Rx FIFO interrupt - should get
				inlined into interrupt handler.
*/

static inline
void __not_in_flash_func(event_d)(PIO pio, Em6854WkSpace *ws, uint32_t w)
{
}

/*
	Purpose:	Handle line-idle event (I)
	Returns:	Nothing
	Notes:		Called from PIO Rx FIFO interrupt - should get
				inlined into interrupt handler.
*/

static inline void __not_in_flash_func(event_i)(PIO pio, Em6854WkSpace *ws)
{
	// An idle event has to leave us in ST_IDLE, but varies in the
	// amount of cleaning up needed.
//putchar('i');

	if (0)
	{
		// This should shortly cause a TF event and Tx to start
		pio_sm_set_wrap(pio, SM_NO,
			econet_hdlc_offset_tx_wrap_target,
			econet_hdlc_offset_tx_wrap_source);
	}
	else
	{
		// Normal Rx wrap setting.
		pio_sm_set_wrap(pio, SM_NO, econet_hdlc_wrap_target, econet_hdlc_wrap);
	}
}

/*
	Purpose:	Handle TX FIFO interrupt
	Returns:	Nothing
	Notes:		Called from PIO interrupt handler - should get inlined.
*/

static inline void __not_in_flash_func(tx_fifo_int)(PIO pio, Em6854WkSpace *ws)
{
	uint8_t b;

	// Interrupt should only be enabled if there's actually still some
	// data to load.
	{
//		ws->crc = crc16_add_byte(ws->crc, b);
	}
		// This is the last byte so disable interrupts.
		hw_clear_bits(&pio->inte0,
			(PIO_IRQ0_INTE_SM0_TXNFULL_BITS << SM_NO));
	// Put the byte in the FIFO.
//putchar('D');
	pio->txf[SM_NO] = b;
//printf("-%02x-", b);
}

/*
	Purpose:	Exclusive handler for IRQ0  on this PIO unit
	Returns:	Nothing
	Notes:		Currently assumes pio0, SM0 - should support
				multiple interfaces each in own state machine, but
				not easy to do so without adding inefficiency for the
				usual case of only one interface.
*/

static void __not_in_flash_func(pio_irq0_handler)(void)
{
	Em6854WkSpace *ws = &workspace0;
	PIO pio = pio0;

	// RxFIFO interrupt is always enabled.
	// Values read from the FIFO are one of:
	//  - Data byte value in bits 31:24 (bits 23:0 all zeros)
	//  - Marker value for flag received (9)
	//  - Marker value for flag starting transmit (6)
	//  - Marker value for abort/line about to be idle (18 or 19).

	while (pio->ints0 & (PIO_IRQ0_INTS_SM0_RXNEMPTY_BITS << SM_NO))
	{
		uint32_t w = pio->rxf[SM_NO];
		if (w == 9) event_rf(pio, ws);
		else if (w == 6) event_tf(pio, ws);
		else if (w & 0xff)	// all other marker values, should just be 18/19
		{
			event_i(pio, ws);
		}
		else event_d(pio, ws, w); // Normal data byte in high 8 bits
	}

	// Handle Tx not empty interrupt
	// This is only enabled after the leading flag has gone out
	// and the 4 address bytes loaded in the Tx FIFO.
	while (pio->ints0 & (PIO_IRQ0_INTS_SM0_TXNFULL_BITS << SM_NO))
	{
		tx_fifo_int(pio, ws);
		__dmb();	// Ensure write to the Tx FIFO have taken effect
					// before we sample the status again.
	}
}


// -----------------------------------------------------------------

/*
	Purpose:	Initialise Econet hardware for 6854 on 1MHz bus emulation.
	Returns:	Nothing (assumed only one instance of this)
	Notes: 		
*/

void emu6854_init(const EcoHWConfig *hw)
{
	pio_sm_config cfg = econet_hdlc_program_get_default_config(0);
	PIO pio = hw->pio_no ? pio1 : pio0;


	// Our program has to load at offset 0 in the PIO as it uses the
	// entire available space.
	// Note that if we are called multiple times for extra Econet
	// instances in the same PIO unit, this will just harmlessly
	// re-load the program over the top of the existing copy.
    pio_add_program_at_offset(pio, &econet_hdlc_program, 0);

	// This is really an assert about the PIO code itself, as there's
	// a jump dropped off the bottom assuming it will wrap round.
	static_assert(sizeof(econet_hdlc_program_instructions) == (32*2),
		"Code assumes program exactly 32 words long");

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
    pio_gpio_init(pio, hw->txd_pin);
    pio_sm_set_consecutive_pindirs(pio, SM_NO, hw->txd_pin, 1, true);
    pio_gpio_init(pio, hw->txen_pin);
    pio_sm_set_consecutive_pindirs(pio, SM_NO, hw->txen_pin, 1, true);

    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, SM_NO, econet_hdlc_offset_rxentrypoint, &cfg);

	// Hook the interrupt handler
	irq_set_exclusive_handler((pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0,
		pio_irq0_handler);

	// We leave the interrupts permanently enabled in the NVIC,
	// but mask individual sources from time to time.
	irq_set_enabled((pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0, true);

	// Enable the FIFO Rx interrupt
	// These enable bits are in order of sm number
	hw_set_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_RXNEMPTY_BITS << SM_NO));

	// Initialise the CRC table - doesn't mind being called more than once.
	init_crc16_table();

    // Set the state machine running
    pio_sm_set_enabled(pio, SM_NO, true);

	// Return instance handle - pointer to our workspace
	return (void*)&workspace0;
}
