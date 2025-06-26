/**
 * Emulation of 6854 presented on the 1MHz Bus, controlling the econet directly.
 */


// For concurrency purposes, this code assumes that the interrupts for
// 1MHz bus and for econet (which are two different PIO units)
// are configured to the same priority and hence do not preempt each other.
// That significantly simplifies logic here, but at the cost of adding the
// Econet-side IRQ latency to the 1MHz-side (with the 1MHz being timing
// critical).  For this implementation (direct emulation onto the wire)
// that's probably acceptable as the Econet-driven IRQs are simple.
// For future systems that run an internal bridge and may link multiple
// subsystems (USB, Ethernet) a more concurrent solution will probably
// be required.



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


// Address map of emulated registers
#define	REGNO_SR1	0
#define	REGNO_SR2	1
#define	REGNO_RXD	2
#define	REGNO_RXD2	3
#define	REGNO_INTON	4
#define	REGNO_STNID	5
#define	REGNO_SPARE1	6
#define	REGNO_SPARE2	7

// 6854 register constants
#define	CR1_BIT_AC		(1<<0)	// Address control, enables CR3 access
#define	CR1_BIT_RIE		(1<<1)	// Rx interrupt enable
#define	CR1_BIT_TIE		(1<<2)	// Tx interrupt enable
#define	CR1_BIT_RDSR	(1<<3)	// DMA Rx
#define	CR1_BIT_TDSR	(1<<4)
#define	CR1_BIT_RXDISC	(1<<5)	// Rx discontinue
#define	CR1_BIT_RXRST	(1<<6)	// Reset Rx
#define	CR1_BIT_TXRST	(1<<7)	// Reset Tx

#define	CR2_BIT_PSE		(1<<0)	// Prioritized status enable
#define	CR2_BIT_2BYT	(1<<1)	// Rx interrupt enable
#define	CR2_BIT_FLAG	(1<<2)	// Tx interrupt enable
#define	CR2_BIT_FC_TDRA	(1<<3)	// 1 to select FrameComplete in place of TDRA
#define	CR2_BIT_TXLAST	(1<<4)	// Tx last byte
#define	CR2_BIT_CLRRX	(1<<5)	// Clear Rx status
#define	CR2_BIT_CLRTX	(1<<6)	// Clear Tx status
#define	CR2_BIT_RTS		(1<<7)	// Control RTS (ie. claim line)

#define	SR1_BIT_RDA		(1<<0)	// Receive data available
#define	SR1_BIT_S2RQ	(1<<1)	// SR2 read req (ie. any  bits 0-6 set in SR2)
#define	SR1_BIT_LOOP	(1<<2)	// Loopback state (not IRQ causing)
#define	SR1_BIT_FD		(1<<3)	// Flag det (if enabled, not implemented here)
#define	SR1_BIT_CTS		(1<<4)	// nCTS (collision detect)
#define	SR1_BIT_TXU		(1<<5)	// Tx underrun
#define	SR1_BIT_TDRA_FC	(1<<6)	// Space available in Tx FIFO, or FrameComplete
#define	SR1_BIT_IRQ		(1<<7)	// IRQ active (sum of other bits)

#define	SR2_BIT_AP		(1<<0)	// Address present
#define	SR2_BIT_FV		(1<<1)	// Frame valid
#define	SR2_BIT_IDLE	(1<<2)	// Line idle
#define	SR2_BIT_RxABT	(1<<3)	// Received string of 1's in middle of pkt
#define	SR2_BIT_ERR		(1<<4)	// CRC fail etc
#define	SR2_BIT_DCD		(1<<5)	// DCD - ie. clock detect 1=>no clock
#define	SR2_BIT_OVRN	(1<<6)	// Rx Overrun
#define	SR2_BIT_RDA		(1<<7)	// Rx data avail (duplicate of SR1 bit0)



// Which SM of the 4 within the unit?  This could become a macro to
// accesse an assigned SM number to support multiple physical econet interfaces,
// for now just hard-wire it but use this define to save having to hunt
// down the uses later.
#define	SM_NO	0

// Workspace for one econet interface
typedef struct
{
	uint16_t crc;			// Running copy of CRC for Rx or Tx
							// Initialise to 0xffff and invert before final use

	bool line_idle;			// Remember when the PIO tells us the line is idle:
							// it won't tell us again if the BBC has reset
							// the emulated receiver and lost the info fron SR2
	bool tx_mode;			// Shows whether the PIO wrap is set for Tx or Rx

	bool inton;				// Record the inton/intoff toggle

	bool fdisc;				// Discard data until end of frame.
							// Set by writes to CR1, cleared by Rx flag/idle

	uint8_t cr1;			// Last written CR1 value
	uint8_t cr2;			// Last written CR2 value

	uint32_t tx_pending;	// up to 3 bytes waiting to go in the tx FIFO,
							// zero if empty, 9 bits per byte with 0x100 added
							// to distinguish empty from zero value byte.
							// Only one byte used in normal operations,
							// up to 3 at end of packet for the CRC

	uint8_t rx_inptr;		// Indexes to circular buffer for rx data
	uint8_t rx_outptr;		// Indexes to circular buffer for rx data
	uint8_t rx_buf[256];


} Em6854WkSpace;

static Em6854WkSpace workspace0;	// In principle, one of these per interface


// ---------------------------------------------------------------------------
// 1MHz bus transaction handling - called from bus1mhz.c in interrupt context

/*
	Purpose:	Recompute SR1 INT/S2RQ bits and drive BBC IRQ if read
	Returns:	Nothing
	Notes:		Called when one of the status bits changes.
				Very often, this will in fact cause an interrupt change,
				so the logic here doesn't bother to check for things
				not changing to short-cut the remaining processing.
*/

static void __not_in_flash_func(recompute_irq)(Em6854WkSpace *ws)
{
	bool irq;
	// First set S2RQ correctly in SR1 reflecting SR2 sources
	// NB this list does not include RDA as that appears in SR1 as well
	if (eco_1mhz_regs[REGNO_SR2] & 
		(SR2_BIT_AP | SR2_BIT_OVRN | SR2_BIT_DCD | SR2_BIT_ERR
		| SR2_BIT_RxABT | SR2_BIT_IDLE | SR2_BIT_FV) )
	{
		eco_1mhz_regs[REGNO_SR1] |= SR1_BIT_S2RQ;
	}
	else eco_1mhz_regs[REGNO_SR1] &= ~SR1_BIT_S2RQ;

	// Now set IRQ bit if enabled
	irq = (eco_1mhz_regs[REGNO_SR1] & (SR1_BIT_RDA | SR1_BIT_S2RQ)
			&& (ws->cr1 & CR1_BIT_RIE))
		|| ((eco_1mhz_regs[REGNO_SR1] & 
				(SR1_BIT_TDRA_FC | SR1_BIT_TXU | SR1_BIT_CTS | SR1_BIT_FD))
			&& (ws->cr1 & CR1_BIT_TIE));
	if (irq) eco_1mhz_regs[REGNO_SR1] |= SR1_BIT_IRQ;
	else eco_1mhz_regs[REGNO_SR2] &= ~SR1_BIT_IRQ;

	// If the INTON/INTOFF latch is off, changes to SR1[IRQ] have no effect
	// on the actual NMI pin; otherwise reflect it in there
	if (ws->inton && irq)
		set_b1m_nmi_active(0);
}

/*
	Purpose:	Host request to terminate Tx packet
	Returns:	nothing
	Notes:		Called either for the CR2 TxLast bit, or for writing to the
				Tx FIFO via the special end of packet address.
				Adds the CRC (already computed) to the transmit data, and
				resets Tx hardware to transmit the trailing flag then revert to Rx
*/

static void __not_in_flash_func(tx_end)(Em6854WkSpace *ws)
{
	PIO pio = ECONET_PIO;
	uint32_t crc, t;
	// Loading the Tx FIFO is tricky: even if it appears not full, there might
	// be a byte waiting in tx_pending if the IRQ handler hasn't had a chance
	// to run.  Conversely, there might be room in the FIFO for one or even
	// both bytes of the CRC.
	// So we add the CRC unconditionally to tx_pending and enable the IRQ
	// to sort it out.
	crc = ~ws->crc;
	t = (crc & 0xff) | ((crc & 0xff00) << 1) | 0x200100;
	if (ws->tx_pending)
		ws->tx_pending |= t << 9;
	else ws->tx_pending = t;
	hw_set_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_TXNFULL_BITS << SM_NO));
}


/*
	Purpose:	Called after BBC has read one of the emulated registers
	Returns:	Nothing.
	Notes:		BBC transaction gets the value from eco_1mhz_regs[]
				automatically by PIO/DMA.  This function gets called
				via interrupt at the end of the transaction, to implement
				any read-sensitive behaviour - setting up the value for
				the next read.  Performance is potentially very critical:
				if the BBC can read it again before we have done any
				update, it will get the wrong results.
				Things are somewhat relaxed on the BBC as the CPU can't
				can't do back-to-back I/O cycles, but would be very
				tight indeed if implementing a module for Archimedes
				for example.
*/

static inline void __not_in_flash_func(read_sr1)(Em6854WkSpace *ws)
{
	// This should have logic to make CTS sticky
	// but not needed in practice
}
static inline void __not_in_flash_func(read_sr2)(Em6854WkSpace *ws)
{
	// This should have logic to make DCD/RxIdle sticky
	// but not needed in practice
}
static inline void __not_in_flash_func(read_rxd)(Em6854WkSpace *ws)
{
}
static inline void __not_in_flash_func(read_inton)(Em6854WkSpace *ws)
{
	if (!ws->inton)
	{
		// Only drive the pin if there's an IRQ pending in SR1
		if (eco_1mhz_regs[REGNO_SR1] & SR1_BIT_IRQ)
			set_b1m_nmi_active(1);
		ws->inton = true;
	}
}
static inline void __not_in_flash_func(read_intoff)(Em6854WkSpace *ws)
{
	// This is the station ID, but the side effect we need to implement here
	// is disabling of NMI.
	if (ws->inton)
	{
		set_b1m_nmi_active(0);
		ws->inton = false;
	}
}
static inline void __not_in_flash_func(read_spare1)(Em6854WkSpace *ws)
{
}
static inline void __not_in_flash_func(read_spare2)(Em6854WkSpace *ws)
{
}

void __not_in_flash_func(emu6854_reg_read)(unsigned regno)
{
	Em6854WkSpace *ws = &workspace0;
	switch (regno)
	{
		case 0: read_sr1(ws); break;
		case 1: read_sr2(ws); break;
		case 2: read_rxd(ws); break;
		case 3: read_rxd(ws); break;
		case 4: read_inton(ws); break;
		case 5: read_intoff(ws); break;
		case 6: read_spare1(ws); break;
		case 7: read_spare2(ws); break;
	}
}

/*
	Purpose:	Called after BBC has written one of the emulated registers
	Returns:	Nothing.
	Notes:		BBC transactions are queued for us to process at the end
				of the 1MHz cycle.  Timings are potentially less critical
				than reads, as there are no read/write registers that
				allow the host to see directly the results of its own
				writes; the most critical thing here is the updates
				to the 6854 IRQ (which feeds BBC NMI).  Since NMI is
				edge-triggered, wrong timing can cause edges to be lost
				or duplicated.
*/


static inline void
	__not_in_flash_func(write_txd)(Em6854WkSpace *ws, unsigned wb)
{
	PIO pio = ECONET_PIO;

	if (!pio_sm_is_tx_fifo_full(pio, SM_NO))
	{
		// If space, just put the byte and carry on.  Note that we don't need
		// to check for tx_pending because that should only get used on
		// a second write with TXDA already turned off - in which case the
		// host can't legitimately write again until TXDA is turned back on
		// (and that process checks tx_pending)
		// XXX could do the check and declare Tx overflow error?
		pio->txf[SM_NO] = wb;

		if (pio_sm_is_tx_fifo_full(pio, SM_NO))
		{
			// If that byte filled the FIFO, turn off TDRA and enable
			// fifo interrupt to turn it back on again when space available.
			if (!(ws->cr2 & CR2_BIT_FC_TDRA))
			{
				// Only update SR1 if the TDRA/FC bit is in TDRA mode
				eco_1mhz_regs[REGNO_SR1] &= ~SR1_BIT_TDRA_FC;
				recompute_irq(ws);
			}
			hw_set_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_TXNFULL_BITS << SM_NO));
		}
	}
	else
	{
		// No space.  This can only be a 2nd write in 2-byte mode.
		// Stash the byte and wait for the interrupt to pick it up
		// Addinng an extra bit distinguishes tx_pending empty from a byte of zero
		ws->tx_pending = wb | 0x100;
	}
	// All CRC accumulation is done here, not in the fifo interrupt.
	ws->crc = crc16_add_byte(ws->crc, wb);
}

static inline void
	__not_in_flash_func(write_cr1)(Em6854WkSpace *ws, unsigned wb)
{
	unsigned delta;

	// Many of the bits in CR1 need action when they become set or when
	// they change; others are just status that we need to store.
	// So calculate a delta before storing.
	delta = ws->cr1 ^ wb;
	ws->cr1 = wb;
	// RXDISC takes effect when ever it is set on a write; the stored
	// value is immaterial, and we have to maintain a separate flag to
	// discard Rx data (reset on next flag or idle) as we mustn't turn off
	// the discarding if there's another write to CR1 before end of frame
	if (wb & CR1_BIT_RXDISC)
	{
#warning ws->fdisc should be ws->rx_count == -1
		ws->fdisc = true;
		ws->rx_inptr = ws->rx_outptr;	// Flush any buffered data
		eco_1mhz_regs[REGNO_SR2] &=
			SR2_BIT_RDA | SR2_BIT_OVRN | SR2_BIT_ERR | SR2_BIT_FV | SR2_BIT_AP;
		recompute_irq(ws);
	}
	// TIE/RIE may reauire a change to SR1 INT flag and hence the IRQ
	// asserted towards the BBC.
	if (delta & (CR1_BIT_TIE | CR1_BIT_RIE))
		recompute_irq(ws);

	// Tx and Rx resets need action on both edges
}

static inline void
	__not_in_flash_func(write_cr2)(Em6854WkSpace *ws, unsigned wb)
{
	PIO pio = ECONET_PIO;
	unsigned delta;

	// Many of the bits in CR2 need action when they become set or when
	// they change; others are just status that we need to store.
	// So calculate a delta before storing.
	delta = ws->cr2 ^ wb;
	ws->cr2 = wb;

	if (wb & delta & CR2_BIT_RTS)
	{
		// Rising RTS is a request to start Tx.
		// Falling RTS is supposed to allow Tx to continue until a closing
		// flag so no action here (hard abort requires CR1[TxRS])
		ws->crc = 0xffff;
		pio_sm_set_wrap(pio, SM_NO,
			econet_hdlc_offset_tx_wrap_target,
			econet_hdlc_offset_tx_wrap_source);
		pio_sm_exec(pio, SM_NO, pio_encode_jmp(econet_hdlc_offset_tx_zero_bit));
		// When we get the marker from the PIO that it has started transmitting
		// the opening flag, we will consider CTS to be asserted
		// and enable TDRA
		ws->tx_mode = true;
	}
}

void __not_in_flash_func(emu6854_reg_write)(unsigned regno, unsigned wval)
{
	Em6854WkSpace *ws = &workspace0;

	if (regno == 0) write_cr1(ws, wval);
	else if (regno == 2)
	{
		// CR1[AC] controls whether this is a write to CR2 or CR3.
		// We don't currently implement CR3, but mustn't interpret
		// a CR3 write spuriously as a CR2 write.
		if (!(ws->cr1 & CR1_BIT_AC)) write_cr2(ws, wval);
		// else write_cr3
	}
	else if (regno == 3) write_txd(ws, wval);
	else if (regno == 4)
	{
		// Again this could be a CR4 write
		if (!(ws->cr1 & CR1_BIT_AC))
		{
			write_txd(ws, wval);
			tx_end(ws);
		}
		// else write_cr4
	}

	// Also have here 5,6 (INTON/INTOFF - but those are supposed to be read
	// not written) and the spare registers at 6,7 which might be used
	// as an IRQ control.
}


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
	// Interrupt primarily serves to re-enable TDRA in SR1 so that the
	// host sends more data.  However, if it was in 2-byte mode and
	// had sent the 2nd byte while the FIFO was full then there may be pending
	// data to load.
	if (ws->tx_pending)
	{
		pio->txf[SM_NO] = ws->tx_pending & 0xff;	// ?? does PIO prog mind extra bits?
		// At end of packet, tx_pending may have more than 1 byte due to CRC - each
		// byte with an attached marker bit so 9 bits per byte
		ws->tx_pending >>= 9;
		if (pio_sm_is_tx_fifo_full(pio, SM_NO))
			return;	// Not yet ready to turn on TDRA
	}
	if (!(ws->cr2 & CR2_BIT_FC_TDRA))
	{
		// Only update SR1 if the TDRA/FC bit is in TDRA mode
		eco_1mhz_regs[REGNO_SR1] |= SR1_BIT_TDRA_FC;
		recompute_irq(ws);
	}
	// Disable fifo interrutpt
	hw_clear_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_TXNFULL_BITS << SM_NO));
}

/*
	Purpose:	Exclusive handler for IRQ0  on this PIO unit
	Returns:	Nothing
	Notes:		Currently assumes SM0 - could support
				multiple interfaces but not really relevant for this
				direct emulation - there can't be multiple BBCs!
*/

static void __not_in_flash_func(pio_irq0_handler)(void)
{
	Em6854WkSpace *ws = &workspace0;
	PIO pio = ECONET_PIO;

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

	// Interrupt priority - this _must_ match the priority of the 1MHz bus
	// interrupts, currently set to highest priority (zero)
	// bearing in mind that CortexM0 has only 4 distinct levels distinguished
	// by the two MSBs.
	irq_set_priority((pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0, 0);

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
}
