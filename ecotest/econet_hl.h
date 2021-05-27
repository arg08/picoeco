/**
 * Econet High-level drivers - RxCBs, TxCBs etc.
 */

// Error returns
#define	ECO_BAD_HANDLE	-1
#define	ECO_BAD_PARAM	-2		// Bad parameters to API call
#define	ECO_NOT_LISTEN	-3		// Not listening
#define	ECO_NET_ERR		-4		// Net error
#define	ECO_NO_RXCB		-5		// No RxCB available
#define	ECO_NO_TXCB		-6		// No TxCB available

// Note: the RxCB is both a public and an internal data structure;
// TxCBs have a different internal version.

typedef struct rxcb
{
	uint8_t	ctrl;		// Must be zero when setting up listening;
						// will have ctrl byte (top bit set) when done.
	uint8_t port;		// Target port, zero wildcard
	uint16_t stn_net;	// Target station/net (stn lo byte).  Zero wildcard.
	uint8_t *buf;		// Pointer to buffer for rx data
						// (internally: NULL for RxCB not in use)
	unsigned buflen;	// Max length of buffer
} RxCB;

typedef struct txcb
{
	uint8_t ctrl;
	uint8_t port;
	uint16_t stn_net;		// Station in low byte
	uint8_t *buf;			// For imm. ops this is both source and dest
	unsigned buflen;		// For Tx it's the tx len, for imm ops it's
							// the rx buffer size (tx len implicit).
	uint32_t remote_addr;	// Needed for peek/poke/JSR/proc/osPC imm ops only.
	unsigned retry_cnt;		// Counts down to zero
	uint32_t retry_int;		// Retry interval in us.
} TxCB;

// Initiate a normal transmit.  Returns Tx handle, or -ve for error
extern int econet_tx_req(TxCB *txcb);
// Poll for Tx completion.
// Returns -ve for errors, 1 for success, 0 for not done yet
// Once a nonzero value has been returned, the handle is no longer valid 
extern int econet_tx_poll(int tx_handle);
// Cancel a pending tx. Returns 1 for success, -ve for error
// After call, handle is no longer valid.
extern int econet_tx_cancel(int tx_handle);

// Initiate reception
// Returns handle or -ve if error.
extern int econet_rx_listen(const RxCB *rxcb);

// Poll for completion.  Returns 1 for complete, with RxCB filled in,
// 0 for still waiting, -ve for error.  After a nonzero result, handle
// no longer valid.
extern int econet_rx_poll(int rx_handle, RxCB *rxcb);

// Cancel a pending tx. Returns 1 for success, -ve for error
// After call, handle is no longer valid.
extern int econet_rx_cancel(int rx_handle);

// Initialise one interface at higher level - calls through to low level.
extern void econet_hl_init(const EcoHWConfig *hw);
