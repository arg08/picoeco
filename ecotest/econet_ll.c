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


typedef enum
{
	ST_IDLE = 0,		// Idle line interrupt has been received.
	ST_RXSCOUT = 1,		// Receiving scout frame (flag already rx)
	ST_SKIPIDLE = 2,	// Skipping unwanted data until line goes idle
	ST_TXACK1 = 3,		// Transmitting the first ack to an Rx packet
	ST_RXDATA = 4,		// Receiving packet data
	ST_TXACK2 = 5,		// Transmitting the final ack to an Rx packet
	ST_TXSCOUT = 6,		// Transmitting scout for our Tx packet
	ST_RXACK1 = 7,		// Receiving ACK to our scout
	ST_TXDATA = 8,		// Transmitting data frame
	ST_TXDATA_IM = 9,	// Transmitting data frame of immediate op 
						// that only does 2-way handshake
	ST_RXACK2 = 10		// Receiving final ack to our Tx packet
} IFState;

static const uint8_t machine_peek_data[] =
{ 0xf8, 0xff, 0x01, 0x00 };

// Workspace for one econet interface
typedef struct
{
	bool (*isforus)(uint32_t addrs);	// Callback function to see if
						// an address pattern is 'us' and so should
						// cause line claim or ignore after Rx scout.
						// Value is the 4 bytes out of the scout packet,
						// LSB = dest.  A normal station will just want to
						// check the low two bytes against the stnid, but
						// bridges etc. may want to do more.
						// In all cases, should return false for broadcasts.
	IFState state;
unsigned flag_cnt;	// Temp
	uint16_t crc;		// Running copy of CRC for both Tx and Rx
	uint32_t rx_count;	// Count of Rx bytes in current frame
	uint32_t rx_limit;	// Rx will abort if exceeding this limit
	uint32_t tx_left;	// Bytes of data to transmit in current frame
	const uint8_t *tx_ptr;	// Next byte to transmit in current frame
	uint32_t current_addrs;	// 4 address bytes for the current handshake,
						// always arranged in the order matching frames we
						// receive, regardless of the overall Tx/Rx direction.
						//  So LSB is dest stn, byte 1 dest net,
						// byte 2 source stn, MSB source net.
	// Longest legitimate scout packet is 14+CRC=16 for Broadcast
	// (BBCs also send 14 for peek/poke for some reason)
	// XXXX Will need to do something different for big broadcasts
	// as used for bridge discovery
	uint8_t rx_scout_buf[16];
} EcoWkSpace;

static EcoWkSpace workspace0;	// In principle, one of these per interface

/*
	Purpose:	Load 4 bytes of Dest/Src econet address to Tx FIFO
	Returns:	Nothing
	Notes:		Used at start of packet with FIFO empty, so there's
				always room to load them in without checking
				FIFO status.  CRC is reset and bytes accumulated into it
				as they are loaded.
				PIO wrap is set to allow PIO to drop back into sending
				flags when the data runs out at end of packet.
				FIFO Tx interrupts are enabled to transfer the remaining
				bytes (there will always be some, as even the ack packet
				has CRC after the 4-byte address prefix).

				Note that the address bytes are packed in a word to suit
				Rx frames - here for Tx we need to swap source and
				destination addresses.
*/

static void __not_in_flash_func(load_addrs_to_tx_fifo)(PIO pio,
	EcoWkSpace *ws)
{
	uint32_t addr = ws->current_addrs;
	uint32_t b;
	uint16_t crc = 0xffff;

	b = (addr >> 16) & 0xff;
	pio->txf[0] = b;
	crc = crc16_add_byte(crc, b);
	b = (addr >> 24) & 0xff;
	pio->txf[0] = b;
	crc = crc16_add_byte(crc, b);
	b = addr & 0xff;
	pio->txf[0] = b;
	crc = crc16_add_byte(crc, b);
	b = (addr >> 8) & 0xff;
	pio->txf[0] = b;
	crc = crc16_add_byte(crc, b);

	ws->crc = crc;

	// Remove the PIO program wrap that returns to Rx when Tx FIFO empty
	// (0,31) is effectively no wrap at all, just the implicit bottom to top.
	pio_sm_set_wrap(pio, 0 /*XXX*/, 0, 31);

	// Enable Tx interrupts.
	hw_set_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_TXNFULL_BITS << 0));
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
	EcoWkSpace *ws = &workspace0;
	PIO pio = pio0;

	// RxFIFO interrupt is always enabled.
	// Idle line interrupt is enabled in all states except ST_IDLE,
	// and is an error situation for all except ST_SKIPIDLE.
	// In the (unusual) case where both are active, we don't know the
	// relative timing of the idle and the words in the FIFO:
	// the FIFO contents could be the tail end of a frame after which
	// the line went idle, or it could be the line was idle (briefly)
	// and the start of a new frame got loaded into the FIFO before we
	// serviced it.  We therefore process the FIFO first, and if it
	// contains a flag we can inspect the idle IRQ bit  and take
	// appropriate action, clearing the idle IRQ if we now appear to
	// be in a new packet.  If there's no flag, then the data in the
	// FIFO is all junk anyhow (and processing it won't have caused
	// any significant action).  Then after processing the FIFO, if
	// the idle IRQ is still active then this must now be a persistent
	// idle state and can be handled like the case of the idle IRQ
	// occurring on its own.
	// Note that there's no race hazard in clearing the idle IRQ:
	// the PIO program will set it repeatedly for so long as the
	// idle state lasts.
	// TxFIFO interrupts are only enabled during the transmission of
	// the body of a frame, and there's no timing relationship with Rx
	// to be concerned about (idle can't occur during Tx).

	while (pio->ints0 & PIO_IRQ0_INTS_SM0_RXNEMPTY_BITS)
	{
		uint32_t w = pio->rxf[0];
		if (w == 9)
		{
			// Flag received
			if (ws->state == ST_IDLE)
			{
				// Opening flag of scout packet.
				ws->rx_count = 0;
				ws->rx_limit = sizeof(ws->rx_scout_buf);
				ws->crc = 0xffff;
				// Shouldn't be idle here and want to catch return to idle
				// so clear idle flag and enable IRQ if it happens again.
				pio->irq = (1<<0);		// regster is W1C
				hw_set_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_BITS));
				ws->state = ST_RXSCOUT;
			}
			else if (ws->state == ST_RXSCOUT)
			{
				// This ought to be the flag terminating the scout frame.
				// However, it could be a duplicate flag, maybe after
				// a bit of junk or the line going idle.
				// If there's less than 4 bytes, whether or not there
				// was an idle, ignore it all and reset in case this
				// flag turns out to be the start of a well-formed scout.
				if (ws->rx_count < 4)
				{
					ws->rx_count = 0;
					ws->crc = 0xffff;
					pio->irq = (1<<0);		// regster is W1C
					// No need to reset state, rx_limit, or enable IRQ
					// as we are already in ST_RXSCOUT.
				}
				else
				{
					// If we got past 4 bytes and still in RXSCOUT
					// then the PIO was set up such that this scout
					// will have triggered a line turnaround and is now
					// transmitting flags.  So we need to either
					// switch to a Tx state or else do some cleanup.
					if ((ws->crc != VALID_CRC16_FINAL) || 0 /*XXX check RxCBs etc*/)
					{
						ws->state = ST_SKIPIDLE;
						pio_sm_set_wrap(pio, 0 /*XXX*/,
							econet_hdlc_wrap_target, econet_hdlc_wrap);
						// Could consider forcing a JMP to give
						// immediate return to idle rather than
						// cleanly completing the flag.
printf("|abort crc %04x count %u\n", ws->crc, ws->rx_count);
					}
					else
					{
						// XXXXXX Assume machine peek!
						ws->state = ST_TXDATA_IM;
						ws->tx_ptr = machine_peek_data;
						ws->rx_count = 0;
						ws->rx_limit = sizeof(machine_peek_data);
ws->flag_cnt = 0;
#if 0
	// XXXX This only works because the host is slower than the PIO:
	// XXXX Could in theory have got the flag Rx but not got past the
	// XXXX test for Tx FIFO empty that triggers the flag.
	// XXXX However, it's only 2 ticks and interrupt latency can't be that low.
						load_addrs_to_tx_fifo(pio, ws);
#endif
					}
				}
			}
			else
			{
			// XXXXXXXX Flag rx in other states?
			}

			if (0)
			{
// XXXX This is tx code, not turnaround code.
				// Do line turnaround by forcing the PIO into Tx flag code
				pio_sm_exec(pio, 0 /*XXX instance no*/,
					pio_encode_jmp(econet_hdlc_offset_txflag));

				// Clear PIO wrap so that it carries on sending flags
				// until we load data into the Tx FIFO.
				// (0,31) is effectively no wrap at all, just the implicit
				// bottom to top.
				pio_sm_set_wrap(pio, 0 /*XXX*/, 0, 31);

			}
		}
		else if (w == 6)
		{
//putchar('F');
			// Tx has started generating flag(s), either at start
			// of a packet transmission after forcing JMP to txflag
			// or at the end of a Tx frame with the Tx FIFO run dry,
			// or after line-turnaround where we haven't decided what
			// to do next.
			if (ws->state == ST_TXDATA_IM)
			{
				// Will get the first one of these for the leading flag
				// of the frame, but don't need to do anything as the
				// Tx FIFO is already primed.
				// Once the packet has gone however it's time to shut down.
				if (ws->flag_cnt == 0)
				{
					// Do nothing to give BBC a bit more time
				//	load_addrs_to_tx_fifo(pio, ws);
				}
				else if (ws->flag_cnt == 1) load_addrs_to_tx_fifo(pio, ws);
				else
				{
					ws->state = ST_SKIPIDLE;
					pio_sm_set_wrap(pio, 0 /*XXX*/,
						econet_hdlc_wrap_target, econet_hdlc_wrap);
printf("ws->crc %04x\n", ws->crc);
				}
				ws->flag_cnt++;
			}
			else
			{
				// ST_TXACK1, ST_TXACK2, ST_TXSCOUT, ST_TXDATA
				// Re-program the PIO wrap mechanism to cause PIO
				// to switch back to Rx at the end of the flag
				//XXX

			}
		}
		else
		{
			// Data byte arrived.
			//
			// For SKIPIDLE, we just discard the byte and carry on
			// with minimum overhead (it's someone else's packet)
			// Similarly ST_IDLE we haven't had an opening flag yet,
			// so any data received is just glitches.
			// Tx states don't generate data in the Rx FIFO.
			// So this should be any of:
			// ST_RXSCOUT, ST_RXDATA, ST_RXACK1, ST_RXACK2
			// all of which need to at least accumulate the CRC
			if ((ws->state == ST_RXSCOUT) || (ws->state == ST_RXDATA)
				|| (ws->state == ST_RXACK1) || (ws->state == ST_RXACK2))
			{
				if (ws->rx_count >= ws->rx_limit)
				{
					// Frame too big - cancel any decision we may have made
					// to do line turnaround at the end.
					// XXXX Error reporting??
					pio_sm_set_wrap(pio, 0 /*XXX*/,
						econet_hdlc_wrap_target, econet_hdlc_wrap);
					ws->state = ST_SKIPIDLE;
				}
				w >>= 24;		// Get the actual data byte
				ws->rx_scout_buf[ws->rx_count++] = w;
				ws->crc = crc16_add_byte(ws->crc, w);

				// Make the decision at rx_count == 4 as to whether
				// we are going to turn the line round at the end
				// of the frame: at this point we have all the address bytes.
				// RXACK2 however is the final frame and never wants
				// turning round, though we do want to check the addresses
				if (ws->rx_count == 4)
				{
					uint32_t addrs = *(uint32_t*)ws->rx_scout_buf;
					bool accept;
					if (ws->state == ST_RXSCOUT)
					{
						// Ask the higher layer whether to accept
						ws->current_addrs = addrs;
						accept = ws->isforus(addrs);
					}
					// Otherwise, check if the addresses match
					else accept = (addrs == ws->current_addrs);
					if (accept)
					{
putchar('D');
						// Remove the PIO program wrap that returns to Rx
						// after flag detected.
						// (0,31) is what you get without the wrap mechanism
						pio_sm_set_wrap(pio, 0 /*XXX*/, 0, 31);

						// If this is an Rx data frame, this is the point
						// at which the user data starts.
						if (ws->state == ST_RXDATA)
						{
							// Switch to another buffer??
							// Or stash it in the scout buffer and copy it
							// out in chunks later?
							// Adjust rx_limit?
						}
					}
					else
					{
						// Decided not to accept this packet.
						// Wrap should already be in the state for
						// continued Rx
						// .pio source - this just carries on receiving
						// after a flag.
						pio_sm_set_wrap(pio, 0 /*XXX*/,
							econet_hdlc_wrap_target, econet_hdlc_wrap);
						// And we don't care about any of those bytes either
						// until the line goes idle again.
						ws->state = ST_SKIPIDLE;
					}
				}
			}
		}
	}


	// Idle IRQ still active after FIFO processing - line is really idle.
	if (pio->ints0 & PIO_IRQ0_INTS_SM0_BITS)
	{
		putchar('i');
		// Disable the interrupt again until we get more data
		hw_clear_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_BITS));
		pio_sm_set_wrap(pio, 0 /*XXX*/,
			econet_hdlc_wrap_target, econet_hdlc_wrap);
		ws->state = ST_IDLE;
	}

	// Handle Tx not empty
	while (pio->ints0 & PIO_IRQ0_INTS_SM0_TXNFULL_BITS)
	{
//printf("tx");
		uint8_t b;
		if (ws->rx_count < ws->rx_limit)
		{
			// Bytes up to rx_limit come from buffer and need to be CRCed
			b = *ws->tx_ptr++;
			ws->crc = crc16_add_byte(ws->crc, b);
		}
		// After the data comes two bytes of CRC.  Note need to use ~crc.
		else if (ws->rx_count == ws->rx_limit) b = ~ws->crc;
		else
		{
			// Final byte is hi byte of CRC - disable interrupts after this.
			b = ~ws->crc >> 8;
			hw_clear_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_TXNFULL_BITS));
		}

		pio->txf[0] = b;
		ws->rx_count++;
	}
}


// -----------------------------------------------------------------

/*
	Purpose:	Initialise one instance of Econet hardware.
	Returns:	Nothing
	Notes: 		Currently only works for a single instance, but
				some plumbing is there for multiple instances.
				Second parameter is a callback function, called on
				each scout frame Rx to ask "is this for us?".
				If it returns true, the line will be claimed at the end
				of the scout Rx, false and it's assumed to be a packet
				for some other station and all input will be skipped
				until the line goes idle.  Callback is invoked from
				interrupt handler so should return quickly and
				only do basic processing (typically just checking
				the low 16 bits against the local stnid).  There's
				the opportunity to do more (RxCB scanning) when the
				whole scout is in and the line claimed.
*/

void econet_ll_init(const EcoHWConfig *hw,
	bool (*isforus_cb)(uint32_t eco_addrs))
{
	pio_sm_config cfg = econet_hdlc_program_get_default_config(0);

	// Save the supplied callback
	workspace0.isforus = isforus_cb;
	workspace0.state = ST_IDLE;

	// Our program has to load at offset 0 in the PIO as it uses the
	// entire available space.
	// Note that if we are called multiple times for extra Econet
	// instances in the same PIO unit, this will just harmlessly
	// re-load the program over the top of the existing copy.
    pio_add_program_at_offset(hw->pio, &econet_hdlc_program, 0);

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
    pio_gpio_init(hw->pio, hw->txd_pin);
    pio_sm_set_consecutive_pindirs(hw->pio, hw->sm, hw->txd_pin, 1, true);
    pio_gpio_init(hw->pio, hw->txen_pin);
    pio_sm_set_consecutive_pindirs(hw->pio, hw->sm, hw->txen_pin, 1, true);

    // Load our configuration, and jump to the start of the program
    pio_sm_init(hw->pio, hw->sm, econet_hdlc_offset_rxentrypoint, &cfg);

	// Hook the interrupt handler
	irq_set_exclusive_handler((hw->pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0,
		pio_irq0_handler);

	// We leave the interrupts permanently enabled in the NVIC,
	// but mask individual sources from time to time.
	irq_set_enabled((hw->pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0, true);

	// Enable the FIFO Rx interrupt
	// These enable bits are in order of sm number
	hw_set_bits(&hw->pio->inte0, (PIO_IRQ0_INTE_SM0_RXNEMPTY_BITS << hw->sm));

	// Initialise the CRC table - doesn't mind being called more than once.
	init_crc16_table();

    // Set the state machine running
    pio_sm_set_enabled(hw->pio, hw->sm, true);
}

