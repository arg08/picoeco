/**
  Econet higher layer - packet processing, RxCBs etc.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "board_specific.h"
#include "econet_ll.h"
#include "econet_hl.h"

// Temp - station ID hard-wired rather than configured.
#define OUR_STNID	8

#define	NOOF_TXCBS	4
#define	NOOF_RXCBS	16

static void *ih;		// The instance handle for the low-layer/hardware

#define	IMM_MIN		0x81		// Minimum valid ctrl value
#define	IMM_PEEK	0x81		// Peek - gets 4 byte addr, 4 byte len
#define	IMM_POKE	0x82		// Poke - gets 4 byte addr
#define	IMM_JSR		0x83		// Remote JSR - gets 4 byte addr
#define	IMM_RPC		0x84		// Remote procedure call - gets 2byte proc no
#define	IMM_OSRPC	0x85		// OS Procedure call - gets 2byte proc no
#define	IMM_HALT	0x86		// Halt - no args
#define	IMM_CONT	0x87		// Continue - no args
#define	IMM_MCPEEK	0x88		// Machine Peek - no args
#define	IMM_GETREG	0x89		// Get registers (ARM only) - no args
#define	IMM_MAX		0x89		// Maximum supported value

// Scout packet lengths, starting at IMM_MIN
static const uint8_t imm_op_scout_len[IMM_MAX - IMM_MIN + 1] =
{ 6+4+4, 6+4, 6+4, 6+2, 6+2, 6, 6, 6, 6 };
// True if only 2-way handshake
static const uint8_t imm_op_2way[IMM_MAX - IMM_MIN + 1] =
{ true, false, false, false, false, true, true, true, true};

static const uint8_t machine_peek_data[] =
{ 0xf7, 0xff, 0x01, 0x00 };

// RxCB structure defined in include file.

// Internal TxCB - same as user one but with added next_retry
// Once done, retry_cnt is zero and success/fail in ctrl
// Unused entries have buf == NULL.
typedef struct inttxcb
{
	TxCB u;					// User's TxCB
							// Buffer pointer is NULL if not active.
	uint32_t next_retry;	// time_us_32() value for next retry
	
} IntTxCB;

static RxCB rxcbs[NOOF_RXCBS];
static IntTxCB txcbs[NOOF_TXCBS];
static int max_rxcb_no = -1;	// Maximum RxCB number in use (but may be gaps)
static int max_txcb_no = -1;	// Maximum TxCB number in use

static RxCB *active_rxcb = NULL;
static uint8_t rx_ctrl;		// Saved from scout to go in RxCB if Rx completes
static uint8_t rx_port;
static uint16_t rx_stn_net;

static IntTxCB *active_txcb = NULL;
static uint8_t tx_scout[14];	// Constructed here from TxCB for current Tx

/*
	Purpose:	Callback to check address on Rx packet
	Returns:	true if this is potentially acceptable, false to discard
	Notes:		Called with first 4 bytes of a received scout,
				LSB=dest stn, dest net, src stn, MSB=src net.
				Needs to quickly return whether or not the packet is
				addressed to us (ie. we are allowed to claim the line).
				Should return true for broadcasts, assuming they are wanted.
				Full check on the acceptability of the packet comes later.
*/

static bool __not_in_flash_func(cb_addr_is_us)(uint32_t addrs)
{
	// Accept our station and all broadcasts
	if ((addrs & 0xffff) == OUR_STNID)
	{
		return true;
	}
	return (addrs & 0xff) == 0xff;	// Broadcast
}

/*
	Purpose:	Callback to handle a received scout (inc bdcast)
	Returns:	Nothing
	Notes:		Supplied buffer contains the whole thing, starting with
				the destination address.  Supplied length includes addresses
				but not the CRC.   For broadcasts (defined as the low address
				byte being 0xff), this is the end of the transaction and no
				further calls should be made.  For all other cases, one
				of the following should be called, either directly from the
				callback or some time later (the line will be held meanwhile):
				econet_ll_rx_start() - for normal 4-way handshake
				econet_ll_rx_2way() - for immediate ops with 2-way handshake
				econet_ll_cancel_rx() - if turns out not to be wanted
				The first two are for use specifically in response to this
				callback; cancel_rx() can be used at any stage during the
				reception to abort (typically causing the other end to see
				Net Error).
				The pkt_id is a sequence number used to detect the case where
				the higher layer is cancelling but in the meantime the original
				transaction has gone away (perhaps the other end timed it out)
				and another one is now active - matching the ID prevents the
				old transaction cancelling the new one by mistake.
*/
static void __not_in_flash_func(cb_rx_scout)(const uint8_t *data, unsigned len, uint32_t pkt_id)
{
	int i;
	if (data[5])
	{
		// Non-immediate Rx
		rx_ctrl = data[4];
		rx_port = data[5];
		rx_stn_net = data[2] | (data[3] << 8);

		// Look for an RxCB
		for (i = 0; i <= max_rxcb_no; i++)
		{
			// Presence of a buffer shows RxCB in use
			if (!rxcbs[i].buf) continue;
			// Non-zero ctrl shows it's already full
			if (rxcbs[i].ctrl) continue;
			// Match port unless RxCB is wild
			if ((rxcbs[i].port != 0) && (rxcbs[i].port != rx_port))
				continue;
			// Match stn unless RxCB is wild
			if ((rxcbs[i].stn_net != 0) && (rxcbs[i].stn_net != rx_stn_net))
				continue;

			// Here with a suitable RxCB
			// For broadcast, we just copy the data and job is over
			if (data[0] == 0xff)
			{
				unsigned dlen = len - 6;	// 6 for addrs+ctrl+port
				if (rxcbs[i].buflen < dlen) dlen = rxcbs[i].buflen;
				memcpy(rxcbs[i].buf, data + 6, dlen);
				rxcbs[i].buflen = dlen;
				// Mark the RxCB done - for non-bdcast, this comes later
				rxcbs[i].ctrl = rx_ctrl;
				rxcbs[i].port = rx_port;
				rxcbs[i].stn_net = rx_stn_net;
			}
			else
			{
				// Normal Rx.  Accept the packet and wait for data
				active_rxcb = &rxcbs[i];
				econet_ll_rx_start(ih, active_rxcb->buf, active_rxcb->buflen);
			}
			return;
		}
		// Here with no RxCB found.
		// No action needed if broadcast, otherwise cancel the Rx.
		if (data[0] != 0xff)
		{
			econet_ll_cancel_rx(ih, pkt_id);
		}
	}
	else if (data[0] == 0xff)
	{
		// Broadcast immed op! Just ignore these.
		return;
	}
	else
	{
		// Immediate ops
		if (data[4] == IMM_MCPEEK)
		{
			econet_ll_rx_2way(ih, machine_peek_data, sizeof(machine_peek_data));
		}
		else
		{
			econet_ll_cancel_rx(ih, pkt_id);
		}
	}
}

/*
	Purpose:	Callback to request extension Rx buffer
	Returns:	Pointer to new buffer or NULL if none
	Notes:		Called if rx data will not fit in the buffer provided by
				econet_ll_rx_start()/_2way().  Either returns a pointer to
				a new buffer and its length, or returns NULL in which case
				the rx will be cancelled.  Can be called more than once if
				lots of small buffers are used.  Conversely, implementations
				that always use one large buffer can just supply a dummy
				version of this callback that always returns NULL.
				Note that on return the previous buffer won't be touched again
				higher layer is free to start moving it.
*/

static	uint8_t* __not_in_flash_func(cb_rx_extra_buf)(unsigned *len)
{
	// This demo version doesn't use extension buffers
	return NULL;
}

/*
	Purpose:	Callback to report end of rx data
	Returns:	Nothing
	Notes:		Called when all of the Rx data has been written to buffer(s).
				Should call econet_ll_rx_final() or econet_ll_cancel_rx()
				to complete the entire operation.
*/

static void __not_in_flash_func(cb_rx_end)(unsigned total_len)
{
	if (active_rxcb)
	{
		active_rxcb->buflen = total_len;
		active_rxcb->ctrl = rx_ctrl;
		active_rxcb->port = rx_port;
		active_rxcb->stn_net = rx_stn_net;
	}
	econet_ll_rx_final(ih);
}

/*
	Purpose:	Callback to report that Rx operation has terminated early
	Returns:	Nothing
	Notes:		Called during rx when an error has occurred - CRC error or
				the other end simply going away during the transaction.
*/

static void __not_in_flash_func(cb_rx_cancel)(unsigned err)
{
	if (active_rxcb) active_rxcb = NULL;
}


/*
	Purpose:	Callback to request Tx data
	Returns:	Nothing
	Notes:		Called during Tx after initial ACK when lower layer is
				ready for Tx data. Should call econet_ll_tx_data()
				when convenient - either directly from this callback
				if data already available, or some time later.
*/

static void __not_in_flash_func(cb_tx_ready)(void)
{
	if (active_txcb)
		econet_ll_tx_data(ih, active_txcb->u.buf, active_txcb->u.buflen, false);
	else econet_ll_cancel_tx(ih);
}

/*
	Purpose:	Callback to request next buffer of data on Tx
	Returns:	Pointer to buffer or NULL if no more
	Notes:		Called when all of the data from econet_ll_tx_data() has
				been sent, to get more data.  Returns NULL when all the
				data has already been consumed, to cause end of frame.
*/
static uint8_t* __not_in_flash_func(cb_tx_more_data)(unsigned *len)
{
	// This demo version doesn't use multiple buffers so returns NULL always
	return NULL;
}

/*
	Purpose:	Callback to report Tx complete
	Returns:	Nothing
	Notes:		Called with err = 0 for successful Tx completion, otherwise
				with error code when an error has occurred - CRC error or
				the other end simply going away during the transaction.
				This terminates the whole Tx - there will be no more callbacks.
*/

static void __not_in_flash_func(cb_tx_done)(unsigned err)
{
	if (active_txcb)
	{
		if (err == 0)
		{
			// Success, so no more retries needed
			active_txcb->u.retry_cnt = 0;
			active_txcb->u.ctrl = 0;
		}
		else if (active_txcb->u.retry_cnt == 0)
		{
			// Error and run out of retries
			active_txcb->u.ctrl = err;
		}
		// Else leave the txcb with the retry time that was set up
		// when initiating the tx.

		// TxCB is no longer active.
		active_txcb = NULL;
	}
	else printf("<txd>");
}

static const EcoLLCallbacks our_callbacks =
{
	.addr_is_us = cb_addr_is_us,
	.rx_scout = cb_rx_scout,
	.rx_extra_buf = cb_rx_extra_buf,
	.rx_end = cb_rx_end,
	.rx_cancel = cb_rx_cancel,
	.tx_ready = cb_tx_ready,
	.tx_more_data = cb_tx_more_data,
	.tx_done = cb_tx_done
};

// -----------------------------------------------------------------------

static void start_next_tx(void)
{
	int i, found;
	uint32_t delta, max, now = time_us_32();

	// XXXX disable interrupts?
	if (active_txcb)
	{
		printf("start_next_tx() when active\n");
		return;
	}

	// Find the TxCB out of those active with the earliest retry time
	for (found = -1, max = 0, i = 0; i <= max_txcb_no; i++)
	{
		// Ignore ones already finished
		if (!txcbs[i].u.retry_cnt) continue;
		// Compute how long ago tx should have started
		delta = now - txcbs[i].next_retry;
		// If negative, it's not ready to transmit yet
		if (delta & 0x80000000) continue;
		if (delta > max)
		{
			max = delta;
			found = i;
		}
	}

	if (found != -1)
	{
		unsigned len;
		active_txcb = &txcbs[found];
		active_txcb->next_retry = now + active_txcb->u.retry_int;
		active_txcb->u.retry_cnt--;
		tx_scout[0] = active_txcb->u.stn_net;
		tx_scout[1] = active_txcb->u.stn_net >> 8;
		tx_scout[2] = OUR_STNID;
		tx_scout[3] = 0;		// Source net number
		tx_scout[4] = active_txcb->u.ctrl;
		tx_scout[5] = active_txcb->u.port;
		if (active_txcb->u.port == 0)
		{
			if ((active_txcb->u.ctrl < IMM_MIN)
				|| (active_txcb->u.ctrl > IMM_MAX))
			{
				active_txcb->u.ctrl = ECO_BAD_PARAM;
				return;
			}
			// It's an immediate op.  Add extra args (not all used in all cases)
			tx_scout[6] = active_txcb->u.remote_addr;
			tx_scout[7] = active_txcb->u.remote_addr >> 8;
			tx_scout[8] = active_txcb->u.remote_addr >> 16;
			tx_scout[9] = active_txcb->u.remote_addr >> 24;
			tx_scout[10] = active_txcb->u.buflen;
			tx_scout[11] = active_txcb->u.buflen >> 8;
			len = imm_op_scout_len[active_txcb->u.ctrl - IMM_MIN];
			if (imm_op_2way[active_txcb->u.ctrl - IMM_MIN])
				len |= 0x80000000;
		}
		else if ((active_txcb->u.stn_net & 0xff) == 0xff)
		{
			// Broadcast.  Fixed size, user's data goes in the scout.
			memcpy(tx_scout + 6, active_txcb->u.buf, 8);
			len = 6 + 8;
		}
		else len = 6;

		// Shouldn't ever fail since we know whether there's a tx active.
		if (!econet_ll_req_tx(ih, tx_scout, len))
		{
			active_txcb = NULL;
			printf("Tx start failed\n");
		}
		else printf("Tx start OK\n");
	}
}

// -----------------------------------------------------------------------

// API exposed to clients

// Handles are indexes into the arrays of TxCB/RxCB, but with these
// offsets added to detect misuse.
#define	TX_HANDLE_MAGIC	0x123400
#define	RX_HANDLE_MAGIC	0x567800
#define	HANDLE_MAGIC_MASK	0x00ffff00
#define	HANDLE_INDEX_MASK	0x000000ff

/*
	Purpose:	Initiate a normal transmit.
	Returns:	Tx handle, or -ve for error
	Notes:		Buffer in supplied TxCB must be non-NULL even for imm ops.
*/

int econet_tx_req(TxCB *user_txcb)
{
	int handle;
	if (!user_txcb->buf) return ECO_BAD_PARAM;
	if (!user_txcb->retry_cnt) return ECO_BAD_PARAM;

	for (handle = 0; ; handle++)
	{
		if (handle >= NOOF_TXCBS) return ECO_NO_TXCB;
		if (txcbs[handle].u.buf == NULL) break;
	}
	txcbs[handle].u = *user_txcb;
	txcbs[handle].next_retry = time_us_32();

	if (handle > max_txcb_no) max_txcb_no = handle;

	// Note that start_next_tx might actually start some other Tx
	// that is overdue rather than this one.
	if (!active_txcb) start_next_tx();

	return handle | TX_HANDLE_MAGIC;
}

/*
	Purpose:	Poll for Tx completion.
	Returns:	-ve for errors, 1 for success, 0 for not done yet
	Notes:		Once a nonzero value has been returned, the handle
				is no longer valid. 
*/

int econet_tx_poll(int tx_handle)
{
	int result;
	IntTxCB *our_txcb;
	if ((tx_handle & HANDLE_MAGIC_MASK) != TX_HANDLE_MAGIC)
		return ECO_BAD_HANDLE;
	tx_handle &= HANDLE_INDEX_MASK;
	our_txcb = &txcbs[tx_handle];

	if (our_txcb->u.buf == NULL) return ECO_BAD_HANDLE;

	// XXXX Kluge: use people polling to trigger tx retries.
	// Ought to come from some sort of timer interrupt.
	if (!active_txcb) start_next_tx();

	if (our_txcb->u.retry_cnt) return 0;	// Still waiting

	// Here if finished - result is in the ctrl code
	if (our_txcb->u.ctrl == 0) result = 1;
	else if (our_txcb->u.ctrl == ERR_NOTLIS) result = ECO_NOT_LISTEN;	
	else result = ECO_NET_ERR;

	// Mark TxCB no longer in use.
	our_txcb->u.buf = NULL;

	// If this was the highest-numbered  TxCB in use, crank down the
	// tracking variable (minimises wasted search time).
	if (tx_handle == max_txcb_no)
	{
		while ((txcbs[max_txcb_no].u.buf == NULL) && (max_txcb_no >= 0))
			max_txcb_no--;
	}

	return result;
}

/*
	Purpose:	Cancel a pending tx.
	Returns:	1 for success, -ve for error
	Notes:		After call, handle is no longer valid.
*/

int econet_tx_cancel(int tx_handle)
{
	IntTxCB *our_txcb;
	if ((tx_handle & HANDLE_MAGIC_MASK) != TX_HANDLE_MAGIC)
		return ECO_BAD_HANDLE;
	tx_handle &= HANDLE_INDEX_MASK;
	our_txcb = &txcbs[tx_handle];

	if (our_txcb->u.buf == NULL) return ECO_BAD_HANDLE;

	if (our_txcb == active_txcb)
	{
		active_txcb = NULL;
		// XXXX Should cancell more vigorously.
	}

	our_txcb->u.buf = NULL;
	our_txcb->u.retry_cnt = 0;
	
	// If this was the highest-numbered  TxCB in use, crank down the
	// tracking variable (minimises wasted search time).
	if (tx_handle == max_txcb_no)
	{
		while ((txcbs[max_txcb_no].u.buf == NULL) && (max_txcb_no >= 0))
			max_txcb_no--;
	}
	return 1;
}

/*
	Purpose:	Initiate reception
	Returns:	Handle or -ve if error.
	Notes:		Assumes user has zeroed the ctrl field of the supplied RxCB.
*/

extern int econet_rx_listen(const RxCB *user_rxcb)
{
	int handle;

	if ((!user_rxcb->buf) || user_rxcb->ctrl) return ECO_BAD_PARAM;

	for (handle = 0; ; handle++)
	{
		if (handle >= NOOF_RXCBS) return ECO_NO_RXCB;
		if (rxcbs[handle].buf == NULL) break;
	}
	rxcbs[handle] = *user_rxcb;

	// Keep track of highest RxCB number in use
	if (handle > max_rxcb_no) max_rxcb_no = handle;

	return handle | RX_HANDLE_MAGIC;
}

/*
	Purpose:	Poll for Rx completion.
	Returns:	1 for complete, 0 for still waiting, -ve for error.
	Notes:		On 1 return, supplied RxCB is filled in (otherwise unused).
				After a nonzero result, handle no longer valid.
*/
int econet_rx_poll(int handle, RxCB *user_rxcb)
{
	RxCB *our_rxcb;
	if ((handle & HANDLE_MAGIC_MASK) != RX_HANDLE_MAGIC)
		return ECO_BAD_HANDLE;
	handle &= HANDLE_INDEX_MASK;
	our_rxcb = &rxcbs[handle];

	if (our_rxcb->buf == NULL) return ECO_BAD_HANDLE;

	if (our_rxcb->ctrl == 0) return 0;	// Still waiting

	// Here if received - copy out the RxCB and clear our copy
	*user_rxcb = *our_rxcb;
	our_rxcb->buf = NULL;

	// If this was the highest-numbered  RxCB in use, crank down the
	// tracking variable (minimises wasted search time).
	if (handle == max_rxcb_no)
	{
		while ((rxcbs[max_rxcb_no].buf == NULL) && (max_rxcb_no >= 0))
			max_rxcb_no--;
	}

	return 1;
}

/*
	Purpose:	Cancel a pending Rx.
	Returns:	1 for success, -ve for error
	Notes:		After call, handle is no longer valid.
*/

int econet_rx_cancel(int handle)
{
	RxCB *our_rxcb;
	if ((handle & HANDLE_MAGIC_MASK) != RX_HANDLE_MAGIC)
		return ECO_BAD_HANDLE;
	handle &= HANDLE_INDEX_MASK;

	our_rxcb = &rxcbs[handle];

	if (our_rxcb->buf)
	{
		// Normal case - delete the buffer pointer to show not in use
		our_rxcb->buf = NULL;
	}
	else return ECO_BAD_PARAM;

	if (handle == max_rxcb_no)
	{
		while ((rxcbs[max_rxcb_no].buf == NULL) && (max_rxcb_no >= 0))
			max_rxcb_no--;
	}

	return 1;
}


// -------------------------------------------------------------------------
/*
	Purpose:	Initialise high-level functions.
	Returns:	Nothing
	Notes:		Calls through to ll_init.
*/

void econet_hl_init(const EcoHWConfig *hw)
{
	ih = econet_ll_init(hw, &our_callbacks);
}


