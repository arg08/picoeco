/**
  Econet higher layer - packet processing, RxCBs etc.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "econet_ll.h"

// Temp - station ID hard-wired rather than configured.
#define OUR_STNID	8

static void *ih;		// The instance handle for the low-layer/hardware

static const uint8_t machine_peek_data[] =
{ 0xf8, 0xff, 0x01, 0x00 };


static uint8_t tx_buf[100];
static uint32_t tx_len = 0;

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
	if (data[0] == 0xff)
	{
		// Broadcast
		return;
	}
	if (data[5])
	{
		// Non-immediata
		econet_ll_cancel_rx(ih, pkt_id);
		return;
	}
	else
	{
		// Immediate ops
		if (data[4] == 0x88)
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

static void __not_in_flash_func(cb_rx_end)(void)
{
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
	// This demo version doesn't have any critical state so nothing to do
	printf("<rxc>");
}


/*
	Purpose:	Callback to request Tx data
	Returns:	Nothing
	Notes:		Called during Tx after initial ACK when lower layere is
				ready for Tx data. Should call econet_ll_tx_data()
				when convenient - either directly from this callback
				if data already available, or some time later.
*/

static void __not_in_flash_func(cb_tx_ready)(void)
{
	econet_ll_tx_data(ih, tx_buf, tx_len, false);
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
	printf("<txd>");
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


void econet_hl_init(const EcoHWConfig *hw)
{
	ih = econet_ll_init(hw, &our_callbacks);
}


