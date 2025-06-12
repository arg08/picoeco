/**
 * Econet Low-level drivers - hardware access, packet framing etc.
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
#include "econet_ll.h"

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

/*

Note that this is primarily a per-frame state machine: the standard
packet transmission is a 4-frame handshake (2 Tx and 2 Rx by each end),
and each state can process all 4 frames, though not for the same packet:
there are states for Rx and Tx, so for an overall packet going in one
direction each state will process two frames, and will process the other
two for packet going in the other direction.  We keep track of this by
a separate frame number varible (range 0 to 3 for scout to final ack).
For convenience, in immediate ops that use a 2-way handshake the frames
are numbered 0 and 3 (skipping 1,2).

The main events feeding this state machine are those arriving in the Rx FIFO:
  - Received Flag (RF): The PIO has just read the last zero of a flag
  - Idle (I): the PIO has seen 15 1's
  - Transmit flag (TF): the PIO has just started sending a flag.
  - Data (D): The PIO has received some data bits.  Under some circumstances
	  this includes parts of flags or idle sequences before they are
	  detected as such, and so need to be discarded.
Additional events causing transitions are:
  - Higher layer providing the Tx data (for exit from TXWAIT)
  - Higher layer cancelling the transaction (for exit from TX/TXWAIT)
  - Buffer overflow on Rx (really a special case of D)

The basic transitions are:
   State\event: (I)       (TF)       (D)          (RF)
  IDLE      (ignore)   ST_TX        (ignore)     ST_RX 
  RX        ST_IDLE    (imposs)     store+CRC    ST_SKIPIDLE/ST_TXWAIT/(ign)
  SKIPIDLE  ST_IDLE    (ignore)     (ignore)     (ignore)
  TXWAIT    (error)    ST_TX        (ignore)     (err but ignore)
  TX        (imposs)   RXTURN/SKIPIDLE  (imposs) (imposs)
  RXTURN    ST_IDLE    (error)      (ignore)      ST_RX

All of the Rx states (RX/SKIPIDLE/RXTURN) have the PIO wrap set for Rx
(hence doesn't generate TF events); the Tx states (TXWAIT/TX)
have PIO wrap set for Tx and so can only generate TF events.
ST_IDLE usually has the wrap set for Rx, but has it set to Tx if higher
layer wants to transmit, hence can generate the full range of events.
Events marked (imposs) should not be possible, as the PIO is known
to be in the wrap setting that doesn't generate that event before entering
the state.  Events marked (error) shouldn't occur in normal operation,
but could potentially occur if the ARM is slow in responding.
Some other cases of Rx events in Tx states are marked (ignore) because
they could legitimately occur as a result of a race hazard on
entry to the state, similarly for TF in SKIPIDLE state.

Detailed state descriptions
---------------------------

ST_IDLE: Default initial state.  Data is ignored (assumed glitches
on the line).  Further idle events should only be the result of
such unwanted data and are also ignored.  RF events signal arrival
of a scout packet and advance to ST_RX; must set the PIO wrap at
that point as it could have been either setting during idle.
TF event indicates transmit has started and moves to TX.
(in principle could go to TXWAIT if we wanted to allow the higher layer
to express a desire to transmit without actually providing the Tx
scout data; in practice this isn't useful).

ST_RX: This is the most complex state.  CRC is reset on entry to
the state.  Data is buffered, added to the CRC and counted.
If the data count exceeds the buffer size, exits to SKIPIDLE.
(buffer is set up on entry to the state, using a fixed scout buffer
on entry from IDLE, or requesting from the higher layer on entry
from RXTURN).  When the count reaches 4, the address bytes are
checked (by asking the higher layer if a scout, by comparison
to value recorded from previous frame otherwise).  If the addresses
mismatch, the transaction is aborted (inform higher layer) and
exits to SKIPIDLE.  RF is ignored (repeated opening flag) if the
data count is zero, exits to SKIPIDLE if the count is less than 4,
also exits to SKIPIDLE (after reporting success to higher layer)
if this is the last frame of the transaction, otherwise exits
to TXWAIT (forcing the PIO to transmit mode and informing higher
layer to think about what it wants to do next).

ST_TXWAIT: Wait before Tx after turning the line around from Rx.
Requires two conditions to move to Tx: there must have been at least
2 flags transmitted (should in theory be only one, but BBC misses
opening flag if it is too soon after Rx), and the higher layer
must have provided the data to transmit (or confirmation to do the
transmit if next frame is an ACK).
Entered with the PIO generating flag fill.  Counts flags in the
rx_count variable (initialised zero).  Availability of data is
recorded in bleft - bufptr can legitimately be NULL (and bleft=2)
for ACK packets where there is no user data, just the CRC.
frame_no is still set for the frame just received, incremented on exit.
Exits to TX, having loaded 4 address bytes into the Tx FIFO and
enabled Tx interrupt.
Data events will not occur once properly in the state, but can legitimately
arise if we were slightly slow in turning on our Tx, so are ignored.
RF events are extremely unlikely (requires both ends to be late or
improbable line noise), but similarly ignored.  I events could occur
if we were very late claiming the line - this is an error condition
and exits to SKIPIDLE with a reset of the PIO (which will cause a
subsequent I event to advance to IDLE).


ST_TX: Transmit in progress, activity driven by Tx FIFO interrupts.
Once all the data has been loaded, two bytes of CRC are also loaded,
and then the Tx interrupts are disabled.  TF event indicates that the
FIFO has run dry, at which point the PIO wrap is re-set into Rx mode
and state advances to RXTURN if there are more frames to come,
or SKIPIDLE otherwise.  NB. needs to detect the case of TF event
before all the data has been loaded, which is an error condition
to be reported to higher layer, and needs pio_sm_clear_fifos()
in case data was loaded just too late (will cause mayhem exiting
to Rx with data still in FIFO).  Both exits must reset wrap to Rx mode.

ST_RXTURN: Has just released the line ready for line turnaround by
the other end.  Expecting RF to advance to RX, can legitimately
have data from junk in the inter-frame gap so ignored.
I exits to IDLE (Tx failure case). TF means we were late in
releasing the line and likely collided with the other end's claim,
but ignored in the hope that we get away with it.
frame_no is still the frame just sent, incremented on exit.

Entry conditions:

ST_IDLE: entered from all the Rx states.  Needs to set PIO wrap
according to whether or not high level has a packet waiting to tx.

ST_RX: entered from IDLE and RXTURN.  Frame number updated: set zero
from IDLE, incremented from RXTURN.  Needs to initialise the CRC
and the byte count, find a buffer (built-in scout buffer for frames
0,1,3; callback to higher layer to get buffer for 2).
Need to increment the packet number on entry from IDLE.


ST_SKIPIDLE: entered from almost all states for error handling.
Needs to ensure PIO wrap in Rx mode.

ST_TXWAIT: entered from IDLE and Rx, with PIO already in Tx mode.
Needs to initialise counter (uses rx_count to count Tx flags),
and bleft = 0 to detect higher layer supplying data

ST_TX: Entered from IDLE and TXWAIT, with PIO already in Tx mode.
Frame number updated: from IDLE it's zero or 2 according to whether
it's a 2-way handshake (?how to tell), from TXWAIT it's 1 or 3
according to the higher layer calling _ack_rx or _rx_2way (overall Rx),
or 2 with _tx_data (overall Tx).
Needs to initialise counter and CRC, load address bytes into FIFO,
set get pointer to data and enable Tx interrupts (for a 4-byte
ACK frame, there's no data but still need to load the CRC from Tx int).
Collision detect should be primed as Tx starts.

ST_RXTURN: Need to reset PIO wrap to Rx mode.


*/



// Workspace for one econet interface
typedef struct
{
	EcoLLCallbacks callb;	// Callbacks to higher layer code
	PIO pio;			// Pointer to PIO instance in use.
	IFState state;
	uint8_t frame_no;	// Runs 0..3 for scout/ack/data/ack
	uint16_t crc;		// Running copy of CRC for both Tx and Rx
	uint32_t pkt_id;	// Increments on each complete packet processed
	uint32_t rx_count;	// Counts total bytes rx, not used for tx.
						// counts frames in ST_TXWAIT
	unsigned bleft;		// Count of bytes left in current tx/rx buffer.
						// Zero in ST_TXWAIT if waiting for data.
	uint8_t *bufptr;	// Pointer to current Rx or Tx buffer
						// Can be NULL when doing CRC or just an ack pkt

	uint32_t current_addrs;	// 4 address bytes for the current handshake,
						// always arranged in the order matching frames we
						// receive, regardless of the overall Tx/Rx direction.
						//  So LSB is dest stn, byte 1 dest net,
						// byte 2 source stn, MSB source net.
						// Gets initialised on reciept of 4th byte for Rx,
						// or at the beginning of Tx.
	const uint8_t *pend_tx;	// Scout packet waiting to start Tx
						// NULL if Tx not currently pending.
	uint32_t pend_len;	// Length of frame at pend_tx. Bit31 set if 2-way hs.

	uint8_t *hl_rx_buf;	// Values supplied from econet_ll_rx_start()
	unsigned hl_rx_len;	// giving buffer to use for Rx data

	// Longest legitimate scout packet is 14+CRC=16 for Broadcast
	// (BBCs also send 14 for peek/poke for some reason)
	// XXXX Will need to do something different for big broadcasts
	// as used for bridge discovery
	uint8_t rx_scout_buf[16];
} EcoWkSpace;

static EcoWkSpace workspace0;	// In principle, one of these per interface

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

static void reset_interface(PIO pio, EcoWkSpace *ws)
{
    pio_sm_set_enabled(pio, SM_NO, false);
	pio_sm_clear_fifos(pio, SM_NO);
	pio_sm_exec(pio, SM_NO, pio_encode_jmp(econet_hdlc_offset_rxentrypoint));
	pio_sm_set_wrap(pio, SM_NO, econet_hdlc_wrap_target, econet_hdlc_wrap);
	ws->state = ST_SKIPIDLE;
	// Disable Tx interrupts
	hw_clear_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_TXNFULL_BITS << SM_NO));
    pio_sm_set_enabled(pio, SM_NO, true);
}

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
	pio->txf[SM_NO] = b;
	crc = crc16_add_byte(crc, b);
	b = (addr >> 24) & 0xff;
	pio->txf[SM_NO] = b;
	crc = crc16_add_byte(crc, b);
	b = addr & 0xff;
	pio->txf[SM_NO] = b;
	crc = crc16_add_byte(crc, b);
	b = (addr >> 8) & 0xff;
	pio->txf[SM_NO] = b;
	crc = crc16_add_byte(crc, b);

	ws->crc = crc;

	// Enable Tx interrupts.
	hw_set_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_TXNFULL_BITS << SM_NO));
}

/*
	Purpose:	Consider exiting TXWAIT state
	Returns:	Nothing, state maybe updated
	Notes:		Called both from the TF event and from higher layer
				(maybe foreground, maybe resulting from callback)
				supplying the rx buffer.  Made a separate function
				so the same criteria are applied in all cases.
				Called either from an interrupt handler or with
				interrupts disabled.
*/

static inline void consider_txwait_exit(PIO pio, EcoWkSpace *ws)
{
	if ((ws->rx_count > 1) && ws->bleft)
	{
//putchar('X');
		ws->frame_no++;
		ws->state = ST_TX;
		// This sets up CRC, loads the 4 address bytes, and enables Tx ints.
		load_addrs_to_tx_fifo(pio, ws);
	}
//else putchar('x');
}

/*
	Purpose:	Signal failures arising in Rx
	Returns:	Nothing
	Notes:		Common processing for line going idle during Rx,
				bad CRC on Rx, runt packet Rx.
				If it's the scout frame, higher layer doesn't know about
				it yet so no report needed.  Otherwise, report as
				tx or rx failure according to which frame it is:
				failed ack reception means a Tx fail, failed data
				reception means an Rx fail.
*/

static void __not_in_flash_func(signal_rx_fail)( EcoWkSpace *ws, unsigned err)
{
printf("F%u/%u/%u\n", ws->state, ws->frame_no, err);
	if ((ws->frame_no == 1) || (ws->frame_no == 3))
		ws->callb.tx_done(err);
	else if (ws->frame_no == 2)
		ws->callb.rx_cancel(err);
}

/*
	Purpose:	Handle received flag event (RF)
	Returns:	Nothing
	Notes:		Called from PIO Rx FIFO interrupt - should get
				inlined into interrupt handler.
*/

static inline void __not_in_flash_func(event_rf)(PIO pio, EcoWkSpace *ws)
{
//putchar('*');
	if (ws->state == ST_IDLE)
	{
		// Opening flag of scout packet.
		ws->frame_no = 0;
		ws->pkt_id++;		// Start of a new packet
		ws->rx_count = 0;
		ws->bleft = sizeof(ws->rx_scout_buf);
		ws->bufptr = ws->rx_scout_buf;
		ws->crc = 0xffff;
		ws->state = ST_RX;
	}
	else if (ws->state == ST_RX)
	{
		// Extra flags at the beginning (count == 0) are legitimately ignored,
		// just extra flag-fill.  If count is less than 6, the frame is
		// invalid: we treat >1 byte as an error and abort, but there's
		// an outside possibility of noise/inter-frame junk looking
		// like a flag and giving us a runt packet ahead of a good one.
		// So after a single byte of junk we reset the count and continue,
		// assuming this flag is actually the leading flag.
		// Minimum packet is 6 (4 addrs + CRC), anything less is error.
		if (ws->rx_count < 2)
		{
			ws->bleft -= ws->rx_count;
			ws->bufptr -= ws->rx_count;
			ws->rx_count = 0;
			ws->crc = 0xffff;
		}
		else if (ws->rx_count < 6)
		{
			// Bad frame.  Skip until line goes idle.
			// Report failure to higher layer if reqd
			signal_rx_fail(ws, ERR_RUNT);
			ws->state = ST_SKIPIDLE;
		}
		else
		{
			
			// Potentially good frame - check the CRC.
			if (ws->crc != VALID_CRC16_FINAL)
			{
				// signal_rx_fail() works out if it's an overall Tx or Rx
				// that failed and calls the appropriate callback.
				signal_rx_fail(ws, ERR_CRC);
				ws->state = ST_SKIPIDLE;
			}
			else
			{
				// save rx_count for use in a moment
				unsigned count = ws->rx_count;

				// Good frame: turn the line around unless this was the
				// last frame of the transaction.
				if ((ws->frame_no == 3) || (ws->rx_scout_buf[0] == 0xff))
				{
					// Last frame (tx ack or broadcast) - all over now,
					// just wait for line to go idle
					ws->state = ST_SKIPIDLE;
				}
				else
				{
//putchar('t');
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
					ws->state = ST_TXWAIT;
					ws->rx_count = 0;		// Prepare to count flags
					ws->bleft = 0;		// Data not yet arrived.
				}
				// Inform the higher layer so it can decide what
				// to do next while we sit doing flag-fill.
				// Note that frame_no in TXWAIT is still that of
				// the frame just Rxed
				switch (ws->frame_no)
				{
					case 0:		// New scout frame
						ws->callb.rx_scout(ws->rx_scout_buf,
							count - 2,
							ws->pkt_id);
						break;
					case 1:		// Received ack of Tx scout
						ws->callb.tx_ready();
						break;
					case 2:		// Received main packet data
						ws->callb.rx_end(count - 2);
						break;
					case 3:		// Received Tx final ack
						ws->callb.tx_done(0);	// finished OK
						break;
				}
			}
		}
	}
	else if (ws->state == ST_RXTURN)
	{
		// Good exit from RXTURN - the flag has turned up and we are
		// now in ready for Rx.
		// If it was the first ACK to an Rx, higher layer provided the
		// buffer while we were in TXWAIT; however we don't use it yet
		// and instead grab the first 4 bytes into the scout buffer,
		// which we also use for the other two cases where we just expect ACK
		//  If it was the final ACK, we shouldn't
		// be here as should have gone direct from TX to SKIPIDLE.
		ws->state = ST_RX;
#if SANITY_CHECKS
		if (ws->frame_no == 3)
		{
			printf("Bad frame no %u in RXTURN\n", ws->frame_no);
			reset_interface(pio, ws);
		}
		else
#endif
		{
			ws->bleft = sizeof(ws->rx_scout_buf);
			ws->bufptr = ws->rx_scout_buf;
		}
		ws->frame_no++;
		ws->rx_count = 0;
		ws->crc = 0xffff;
		ws->state = ST_RX;
	}
}

/*
	Purpose:	Handle started-to-transmit-flag event (TF)
	Returns:	Nothing
	Notes:		Called from PIO Rx FIFO interrupt - should get
				inlined into interrupt handler.
*/

static inline void __not_in_flash_func(event_tf)(PIO pio, EcoWkSpace *ws)
{
//putchar('^');
	if (ws->state == ST_IDLE)
	{
		uint32_t w;
		// This is a transmit operation that has auto-started from idle.
		// The leading flag has already gone, so load up the address
		// bytes into the FIFO and start transmitting the data already
		// provided.
		if (!ws->pend_tx)
		{
			// Started but there's nothing to Tx!  Maybe just cancelled
			// at the wrong moment.
			reset_interface(pio, ws);
		}
		else
		{
			ws->pkt_id++;
			// Cast because pend_tx is const but bufptr shared between Rx/Tx
			// +4 because we handle the address bytes separately.
			ws->bufptr = (uint8_t*)ws->pend_tx + 4;
			if (ws->pend_len & 0x80000000)
			{
				ws->pend_len &= 0x7fffffff;
				ws->frame_no = 2;
// XXXX in this case, need to get the reply buffer
			}
			else ws->frame_no = 0;
			ws->bleft = ws->pend_len - 4;
			// Saved addresses have to match Rx order for later comparisons,
			// so need to swap low and high halfwords to swap source/dest
			w = *(const uint32_t *)ws->pend_tx;
			ws->current_addrs = (w << 16) | (w >> 16);
			ws->state = ST_TX;
			// This sets up the CRC, loads  4 addr bytes, and enables Tx ints.
			load_addrs_to_tx_fifo(pio, ws);

			// Prevent transmitting this more than once
			ws->pend_tx = NULL;
			ws->pend_len = 0;
		}
	}
	else if (ws->state == ST_TXWAIT)
	{
		// We have transmitted one (more) flag
		ws->rx_count++;
		// Exit to TX state if we've done at least 2 flags and
		// higher layer has provided a buffer to transmit
		// NB. the buffer pointer itself may be NULL if we are just
		// transmitting an ACK with no data, but in that case the
		// bleft will be 2 for the CRC.
		consider_txwait_exit(pio, ws);
		// Else keep on waiting for those conditions to be satisfied.
	}
	else if (ws->state == ST_TX)
	{
		// We are at the end of Tx and have just started transmitting
		// the trailing flag.  Need to reset the wrap controls so that
		// at the end of the flag PIO switches to Rx mode.
		pio_sm_set_wrap(pio, SM_NO, econet_hdlc_wrap_target, econet_hdlc_wrap);
		// Other end should then send us a flag (RF).
		// Sit in ST_RXTURN waiting for that to happen.
		// Unless this was the last flag, in which case just wait for idle.
		if (ws->frame_no == 3) ws->state = ST_SKIPIDLE;
		else if ((ws->frame_no == 0) && ((ws->current_addrs & 0xff) == 0xff))
		{
			// Broadcast - we're now done
			ws->state = ST_SKIPIDLE;
			ws->callb.tx_done(0);
		}
		else ws->state = ST_RXTURN;
	}
	// TF event in ST_RXTURN means we were too late changing the PIO wrap;
	// could treat as an error, but more useful to ignore and hope the
	// other end is slow so we get away with it.
	// Should be impossible in RX state, in SKIPIDLE again it can only
	// occur if we were late, but SKIPIDLE is where we would go to treat it
	// as an error so nothing to do.
#if SANITY_CHECKS
	else if ((ws->state == ST_RX) || (ws->state == ST_SKIPIDLE))
	{
		printf("TF event in state %u\n", ws->state);
	}
#endif
}

/*
	Purpose:	Handle received data event (D)
	Returns:	Nothing
	Notes:		Called from PIO Rx FIFO interrupt - should get
				inlined into interrupt handler.
*/

static inline
void __not_in_flash_func(event_d)(PIO pio, EcoWkSpace *ws, uint32_t w)
{
	// Only ST_RX wants data, all others either intentionally ignoring it
	// or should be impossible to get it.

	if (ws->state == ST_RX)
	{
		// Check for overflowing buffer.  For normal Rx (frame 2), we can
		// ask the higher layer for another one, likewise immediate op
		// with 2-way handshake (actually 2nd frame but numbered 3 here).
		// Frame 0 is a scout and frame 1 is an ACK so should never
		// overflow.
		// Rather than looking at the frame number, we check if the buffer
		// pointer is actually the scout buffer, as this distinguishes the
		// frame 3 imm. op (can get another buffer) vs 2nd ack (can't).
		if ((ws->bleft == 0)
			&& (ws->bufptr != (ws->rx_scout_buf + sizeof(ws->rx_scout_buf))))
		{
			ws->bufptr = ws->callb.rx_extra_buf(&ws->bleft);
// XXXXX Need to somehow keep at least 1 byte "in our back pocket"
// XXXX as otherwise we put the first byte of CRC in a new buffer and
// XXXX send it off before later discovering it isn't part of the packet data.
// XXX also lesser problem of all Rx buffers needing to be 2 bytes larger
// XXX than ever used
		}
		if (ws->bleft == 0)
		{
			ws->state = ST_SKIPIDLE;
			signal_rx_fail(ws, ERR_OFLO);
		}
		else
		{
			w >>= 24;		// Get the actual data byte
			*ws->bufptr++ = w;
			ws->crc = crc16_add_byte(ws->crc, w);
			ws->bleft--;
			ws->rx_count++;

			// At rx_count == 4 we have all the address bytes and can
			// check if we want to go any further: for initial scout,
			// we check if this is 'for us', for all other frames we
			// check that the addresses match the previous frames.
			// All frame types have been using the rx_scout_buf up to here.
			// NB. this is effectively the point at which we make the
			// decision to turn the line around at the end of the frame.
			if (ws->rx_count == 4)
			{
				uint32_t addrs = *(uint32_t*)ws->rx_scout_buf;
				bool accept;
				if (ws->frame_no == 0)
				{
					// Ask the higher layer whether to accept
					// and remember addresses to check against later frames
					ws->current_addrs = addrs;
					accept = ws->callb.addr_is_us(addrs);
				}
				// Otherwise, check if the addresses match
				else accept = (addrs == ws->current_addrs);

				if (!accept)
				{
					// Decided not to accept this packet.
					ws->state = ST_SKIPIDLE;
				}
				// For data frame, once we've got the addresses,
				// switch to the supplied buffer for the actual data.
				else if (ws->frame_no == 2)
				{
					ws->bufptr = ws->hl_rx_buf;	// Value provided earlier
					ws->bleft = ws->hl_rx_len;
#if SANITY_CHECKS
					if (!ws->bufptr) printf("No buffer on Rx?\n");
					ws->hl_rx_buf = NULL;
#endif
				}
			}
		}
	}
}

/*
	Purpose:	Handle line-idle event (I)
	Returns:	Nothing
	Notes:		Called from PIO Rx FIFO interrupt - should get
				inlined into interrupt handler.
*/

static inline void __not_in_flash_func(event_i)(PIO pio, EcoWkSpace *ws)
{
	// An idle event has to leave us in ST_IDLE, but varies in the
	// amount of cleaning up needed.
//putchar('i');

	if (ws->state == ST_RX)
	{
		// We've aborted while receiving a frame (could be overall Tx or Rx)
		// so may need to notify the higher layer
		signal_rx_fail(ws, ERR_IDLE);
	}
	else if (ws->state == ST_RXTURN)
	{
		// NB. frame_no not yet incremented, so it tells us which frame
		// has just been transmitted and failed to get the response
		if (ws->frame_no == 0) ws->callb.tx_done(ERR_NOTLIS);
		else if (ws->frame_no == 2) ws->callb.tx_done(ERR_IDLE);
	}
	else if (ws->state == ST_TXWAIT)
	{
		// Idle in TXWAIT means it must have happened before we got to
		// TXWAIT so we were very late in starting our Tx and the
		// other end will no longer be listening.
		// frame_no is still set for the Rx frame, so we can legitimately
		// call signal_rx_fail()
		signal_rx_fail(ws, ERR_LATE);
		// Do an aggressive reset of the system.
		reset_interface(pio, ws);
		return;		// No longer appropriate to do the idle processing
	}
	// ST_SKIPIDLE needs no special action, and idle in ST_TX really should
	// be impossible.

	// Now entering ST_IDLE
	ws->state = ST_IDLE;
	// Set wrap according to whether there's a Tx waiting to go
	if (ws->pend_tx)
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

static inline void __not_in_flash_func(tx_fifo_int)(PIO pio, EcoWkSpace *ws)
{
	uint8_t b;
	// This interrupt only enabled in ST_TX
#if SANITY_CHECKS
	if (ws->state != ST_TX) printf("Tx int in state %u\n", ws->state);
	if (ws->bleft == 0) printf("Tx ran out of data??\n");
#endif

	// Interrupt should only be enabled if there's actually still some
	// data to load.  If bufptr is set, bytes come from the buffer and
	// are put through the CRC; if NULL, the bytes to be transmitted
	// _are_ the CRC (which needs to be inverted before use).
	if (ws->bufptr)
	{
		b = *ws->bufptr++;
		ws->crc = crc16_add_byte(ws->crc, b);
		ws->bleft--;
		if (ws->bleft == 0)
		{
			// Run off the end of the buffer - ask the higher layer if 
			// there's any more, and if not switch to doing the CRC.
			ws->bufptr = ws->callb.tx_more_data(&ws->bleft);
			if (!ws->bufptr) ws->bleft = 2;		// 2 bytes of CRC
//putchar('d');
		}
	}
	else if (ws->bleft == 2)
	{
		b = ~ws->crc;		// Low byte of CRC
		ws->bleft--;
//putchar('c');
	}
	else
	{
		b = (~ws->crc) >> 8;	// High byte
		// This is the last byte so disable interrupts.
		hw_clear_bits(&pio->inte0,
			(PIO_IRQ0_INTE_SM0_TXNFULL_BITS << SM_NO));
//putchar('C');
	}
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
	EcoWkSpace *ws = &workspace0;
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

// ---------------------------------------------------------------------------
// Entry points from foreground
// All take an instance pointer, returned from econet_ll_init();


/*
	Purpose:	Ack rx scout for  4-way handshake (including 4way imm ops)
	Returns:	Nothing
	Notes:		Provides a buffer for the receive data.  If the buffer
				isn't big enough, the rx_extra_buf() callback will be used
				to get another one.
*/

void econet_ll_rx_start(void *inst, uint8_t *buf, unsigned len)
{
	EcoWkSpace *ws = (EcoWkSpace*)inst;
	uint32_t intstat = save_and_disable_interrupts();

	// Should be waiting to tx the ACK to the scout
	if (ws->state == ST_TXWAIT)
	{
		// Stash the supplied buffer for later use
		ws->hl_rx_buf = buf;
		ws->hl_rx_len = len;
		// Right now, get the ACK transmitted - just a CRC, no data
		ws->bufptr = NULL;
		ws->bleft = 2;
		// The following not strictly necessary - the buffer would be 'found'
		// on the next TF event, but that guarantees to waste 8 bit-times
		// (since the poll isn't done until just after the flag has gone)
		consider_txwait_exit(ws->pio, ws);
	}
#if SANITY_CHECKS
	else printf("rx_start in state %u\n", ws->state);
#endif
	restore_interrupts(intstat);
}

/*
	Purpose:	Acknowledge the scout for immediate op with 2-way handshake
	Returns:	Nothing
	Notes:		Supplies the data to be returned to the other station.
				data may be NULL and the length zero for ops like
				halt/continue that have a simple ack response with no data.
*/

void econet_ll_rx_2way(void *inst, const uint8_t *data, unsigned len)
{
	EcoWkSpace *ws = (EcoWkSpace*)inst;
	uint32_t intstat = save_and_disable_interrupts();

//putchar('2');
	// Should be waiting to tx the ACK to the scout
	if (ws->state == ST_TXWAIT)
	{
		// Set up to transmit the supplied data (CRC added automatically)
		ws->bufptr = (uint8_t*)data;	// Cast away const
		ws->bleft = len;
		// As it's only a 2-way handshake, tweak the frame number so
		// this is the last frame.  Note that frame_no gets incremented
		// on leaving ST_TXWAIT
		ws->frame_no = 2;
		// The following not strictly necessary - the buffer would be 'found'
		// on the next TF event, but that guarantees to waste 8 bit-times
		// (since the poll isn't done until just after the flag has gone)
		consider_txwait_exit(ws->pio, ws);
	}
#if SANITY_CHECKS
	else printf("rx_2way in state %u\n", ws->state);
#endif
	restore_interrupts(intstat);
}

/*
	Purpose:	Acknowledge the successful reception of an entire packet.
	Returns:	Nothing
	Notes:		Authorises sending of the final ack frame.
				Called in response to (or from) the rx_complete() callback.
*/

void econet_ll_rx_final(void *inst)
{
	EcoWkSpace *ws = (EcoWkSpace*)inst;
	uint32_t intstat = save_and_disable_interrupts();

	// Should be waiting to tx that final ACK
	if (ws->state == ST_TXWAIT)
	{
		// Set up to transmit just the addresses and CRC - no buffer as such
		ws->bufptr = NULL;
		ws->bleft = 2;
		// The following not strictly necessary - the buffer would be 'found'
		// on the next TF event, but that guarantees to waste 8 bit-times
		// (since the poll isn't done until just after the flag has gone)
		consider_txwait_exit(ws->pio, ws);
	}
#if SANITY_CHECKS
	else printf("rx_final in state %u\n", ws->state);
#endif

	restore_interrupts(intstat);
}


/*
	Purpose:	Cancel an Rx in progress  (other end sees not-listening)
	Returns:	Nothing
	Notes:		pkt_id is the value from rx_scout callbackto confirm which
				packet is being cancelled, in case it also fails at the
				other end around the same time - which could result in
				the wrong packet getting cancelled if a new one is just
				arriving in the meantime.
*/

void econet_ll_cancel_rx(void *inst, uint32_t pkt_id)
{
	EcoWkSpace *ws = (EcoWkSpace*)inst;
	uint32_t intstat = save_and_disable_interrupts();

	if (ws->pkt_id == pkt_id)
	{
		// If in idle or skipidle, the Rx is already over and doesn't
		// need cancelling
		if ((ws->state != ST_IDLE) && (ws->state != ST_SKIPIDLE))
		{
			// This leaves us in ST_SKIPIDLE
			reset_interface(ws->pio, ws);
		}
	}
	restore_interrupts(intstat);
}

/*
	Purpose:	Initial entrypoint to request a Tx.
	Returns:	true if OK, false if there's already a Tx queued.
	Notes:		Supplied buffer is just the scout: packet data is
				supplied later.  Buffer is NOT copied: caller to
				ensure it remains available.
				slen has top bit set to request only 2-way handshake.
*/

bool econet_ll_req_tx(void *inst, const uint8_t *scout, unsigned slen)
{
	EcoWkSpace *ws = (EcoWkSpace*)inst;

	// Can do this test outside critical region as this function is
	// the only place pend_tx gets set (and this is really a sanity check
	// anyhow).  Missing it getting cleared doesn't matter.
	if (ws->pend_tx) return false;


	uint32_t intstat = save_and_disable_interrupts();
	ws->pend_len = slen;
	ws->pend_tx = scout;

	// If already idle, set the PIO wrap to cause it to start transmitting.
	// In all other states, this will be done automatically at the point
	// of transitioning to idle state.
	if (ws->state == ST_IDLE)
	{
		// This should shortly cause a TF event and Tx to start
		pio_sm_set_wrap(ws->pio, SM_NO,
			econet_hdlc_offset_tx_wrap_target,
			econet_hdlc_offset_tx_wrap_source);
	}
	restore_interrupts(intstat);
	return true;
}

/*
	Purpose:	Supply Tx data (or at least the 1st chunk of it)
	Returns:	Nothing.
	Notes:		Called in response to the tx_started() callback to supply the
				and allow actual data tx to start.  More data can be
				supplied later via callback if it is not all available yet.
				len is the length of this chunk.
				im2way is true for an immediate op with only 2-way handshake.
*/

void econet_ll_tx_data(void *inst, const uint8_t *data, unsigned len, bool im2way)
{
	EcoWkSpace *ws = (EcoWkSpace*)inst;
	uint32_t intstat = save_and_disable_interrupts();

	// Should always be in TXWAIT - since we are driving the line, there's
	// no way for the other end to abort, only exit is if we time it out
	if (ws->state == ST_TXWAIT)
	{
		if (len)
		{
			ws->bufptr = (uint8_t*)data; // Cast to drop const as bufptr shared
			ws->bleft = len;
		}
		else
		{
			// Not clear if zero length tx data is permitted, but just using
			// bleft == 0 won't work because bleft is used as a data available
			// flag.  To actually transmit zero bytes, must go straight to CRC
			// by setting the pointer to NULL and bleft to 2.
			ws->bufptr = NULL;
			ws->bleft = 2;
		}
		// The following not strictly necessary - the buffer would be 'found'
		// on the next TF event, but that guarantees to waste 8 bit-times
		// (since the poll isn't done until just after the flag has gone)
		consider_txwait_exit(ws->pio, ws);
	}
#if SANITY_CHECKS
	else printf("tx_data in state %u\n", ws->state);
#endif
	restore_interrupts(intstat);
}

/*
	Purpose:	Cancel a requested Tx
	Returns: 	Nothing
	Notes:		Can be called any time, though depending on timing it
				may manage to cancel the tx before it even started,
				or else be too late and it's already done.

				If there is a transaction in progress, then need to check
				that this is actually a Tx and not a subsequent Rx.
				For an overall Tx, we are transmitting for frame nos 0,2
				and listening for frame_no 1,3, so frame_no & 1 says
				it's a frame we should be receiving.  Made slightly
				more complex by the fact that frame_no is incremented
				on exit from TXWAIT and RXTURN, so TXWAIT is a 'rx'
				state for this analysis, even though it's driving the line
				(and vice-versa for RXTURN).
*/

void econet_ll_cancel_tx(void *inst)
{
	EcoWkSpace *ws = (EcoWkSpace*)inst;
	uint32_t intstat = save_and_disable_interrupts();

	// If the scout is still pending, it hasn't started yet and we
	// can just get rid of it.
	if (ws->pend_tx)
	{
		ws->pend_tx = NULL;
		ws->pend_len = 0;
	}
	else if ( ( ((ws->state == ST_TX) || (ws->state == ST_RXTURN))
				&& ((ws->frame_no & 1) == 0))
			|| ( ((ws->state == ST_RX) || (ws->state == ST_TXWAIT))
				&& (ws->frame_no & 1))
			)
	{
		ws->state = ST_SKIPIDLE;
		// Set default PIO wrap - this will terminate any tx that's
		// in progress (data or flags) once Tx FIFO empty
		pio_sm_set_wrap(ws->pio, SM_NO, econet_hdlc_wrap_target,
			econet_hdlc_wrap);
		// Disable Tx interrupts in case enabled.
		hw_clear_bits(&ws->pio->inte0,
			(PIO_IRQ0_INTE_SM0_TXNFULL_BITS << SM_NO));
	}

	restore_interrupts(intstat);
}


// -----------------------------------------------------------------

/*
	Purpose:	Initialise one instance of Econet hardware.
	Returns:	Instance handle to pass back to other econet_ll_xxx functions
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

void *econet_ll_init(const EcoHWConfig *hw, const EcoLLCallbacks *callb)
{
	pio_sm_config cfg = econet_hdlc_program_get_default_config(0);
	PIO pio = hw->pio_no ? pio1 : pio0;

	// Save the supplied callbacks
	workspace0.callb = *callb;
	workspace0.state = ST_SKIPIDLE;
	workspace0.pio = pio;

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
