
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "board_specific.h"
#include "econet_hl.h"

static int rx_handle = -1;
static int tx_handle = -1;

static uint8_t rx_buf[1024];
static uint8_t tx_buf[100];

static void open_rx(void)
{
	RxCB rxcb;
	rxcb.ctrl = 0;
	rxcb.port = 0x99;
	rxcb.stn_net = 0;	// Wildcard
	rxcb.buf = rx_buf;
	rxcb.buflen = sizeof(rx_buf);
	rx_handle = econet_rx_listen(&rxcb);
	if (rx_handle < 0) printf("Rx listen failed %d\n", rx_handle);

printf("rx handle %d\n", rx_handle);
}

void fs_init(void)
{
	open_rx();
}

void poll_fs(void)
{
	int i;
	RxCB rxcb;
	TxCB txcb;
	if (rx_handle >= 0)
	{
		i = econet_rx_poll(rx_handle, &rxcb);
		if (i < 0)
		{
			printf("Poll Rx error %d\n", i);
			rx_handle = -1;
		}
		else if (i > 0)
		{
			// Successful Rx
			printf("Rx packet len %u from %u.%u\n", rxcb.buflen,
				rxcb.stn_net >> 8, rxcb.stn_net & 0xff);
			printf("reply port %02x, fn code %u\n", rx_buf[0], rx_buf[1]);

			txcb.ctrl = rxcb.ctrl;
			txcb.stn_net = rxcb.stn_net;
			txcb.port = rx_buf[0];
			txcb.buf = tx_buf;
			tx_buf[0] = 0;		// Cmd code
			tx_buf[1] = 123;	// Return code
			strcpy((char*)tx_buf+2, "No fileserver here\r");
			txcb.buflen = strlen((char*)tx_buf+2) + 2;
			txcb.retry_cnt = 10;
			txcb.retry_int = 100000;	// 100ms

			tx_handle = econet_tx_req(&txcb);
			printf("Tx handle %d\n", tx_handle);
			open_rx();
		}
		// Else not received yet.
	}
	if (tx_handle >= 0)
	{
		i = econet_tx_poll(tx_handle);
		if (i)
		{
			printf("Tx poll returns %d\n", i);
			tx_handle = -1;
		}
	}
}

