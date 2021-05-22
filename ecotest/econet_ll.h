/**
 * Econet Low-level drivers - hardware access, packet framing etc.
 */

// Econet hardware setup.  Note that we need the whole of a PIO
// so can't share with other PIO functions, but can have multiple
// Econet instances in the same PIO.
typedef struct
{
	uint8_t pio_no;			// Which PIO unit to use?
	uint8_t clk_pin;	// Econet clock input pin number
	uint8_t rxd_pin;	// Econet RxD input pin number
	uint8_t txd_pin;	// Econet TxD output pin number
	uint8_t txen_pin;	// Econet Tx enable output pin number
	uint8_t cts_pin;	// Econet CTS (/collision detect) input pin number
} EcoHWConfig;


// Error codes for failing Tx/Rx
#define ERR_RUNT		1	// Too-short frame received
#define	ERR_CRC			2	// Frame received with CRC error
#define	ERR_OFLO		3	// Frame is too big for buffer
#define	ERR_IDLE		4	// Line went idle during receive
						// XXX should split to not-listening/net-error
#define	ERR_LATE		5	// Idle line due to late interrupt response


// Callbacks from Econet low-level to higher layer code.
// All of these are called in interrupt context and are expected to
// return within a couple of microseconds maximum - so should be
// __not_in_flash_func() and should not do any 'real work', just setting
// up for any such work to be done in foreground if applicable.
// On a single-interface machine the time constraint is quite lax
// (the callbacks get called where the only constraint is filling the
// Rx FIFO, so nearly 4 byte times), however on a multi-interface
// machine execution of this callback may be adding interrupt latency
// for one of the other interfaces and the worst-case requirement is
// 8 Econet bit-times.  Hence thes callbacks should ideally complete
// within 2 bit-times - 4us at the fastest plausible Econet clock.
typedef struct
{
	bool (*addr_is_us)(uint32_t addrs);
		// Called with first 4 bytes of a received scout,
		// LSB=dest stn, dest net, src stn, MSB=src net.
		// Needs to quickly return whether or not the packet is
		// addressed to us (ie. we are allowed to claim the line).
		// A normal station will just want to check low 16 bits
		// against the local station number; bridges may be more complex.
		// Should return true for broadcasts, assuming they are wanted.
		// Full check on the acceptability of the packet comes later.
	void (*rx_scout)(const uint8_t *data, unsigned len, uint32_t pkt_id);
		// Called to hand over a recieved scout packet (including bdcasts).
		// Supplied buffer contains the whole thing, starting with
		// the destination address.  Supplied length includes addresses
		// but not the CRC.   For broadcasts (defined as the low address
		// byte being 0xff), this is the end of the transaction and no
		// further calls should be made.  For all other cases, one
		// of the following should be called, either directly from the
		// callback or some time later (the line will be held meanwhile):
		//   econet_ll_rx_start() - for normal 4-way handshake
		//   econet_ll_rx_2way() - for immediate ops with 2-way handshake
		//   econet_ll_cancel_rx() - if turns out not to be wanted
		// The first two are for use specifically in response to this
		// callback; cancel_rx() can be used at any stage during the
		// reception to abort (typically causing the other end to see
		// Net Error).
		// The pkt_id is a sequence number used to detect the case where
		// the higher layer is cancelling but in the meantime the original
		// transaction has gone away (perhaps the other end timed it out)
		// and another one is now active - matching the ID prevents the
		// old transaction cancelling the new one by mistake.
	uint8_t* (*rx_extra_buf)(unsigned *len);
		// Called if rx data will not fit in the buffer provided by
		// econet_ll_rx_start()/_2way().  Either returns a pointer to
		// a new buffer and its length, or returns NULL in which case
		// the rx will be cancelled.  Can be called more than once if
		// lots of small buffers are used.  Conversely, implementations
		// that always use one large buffer can just supply a dummy
		// version of this callback that always returns NULL.
		// Note that on return the previous buffer won't be touched again
		// so higher layer is free to start moving it.
	void (*rx_end)(void);
		// Called when all of the Rx data has been written to buffer(s).
		// Should call econet_ll_rx_final() or econet_ll_cancel_rx()
		// to complete the entire operation.
	void (*rx_cancel)(unsigned err);
		// Called during rx when an error has occurred - CRC error or
		// the other end simply going away during the transaction.
	void (*tx_ready)(void);
		// Called during Tx after initial ACK to say we are ready for Tx data
		// Should call econet_ll_tx_data() when convenient - either directly
		// from this callback if data already available, or some time later.
	uint8_t* (*tx_more_data)(unsigned *len);
		// Called when all of the data from econet_ll_tx_data() has
		// been sent, to get more data.  Returns NULL when all the
		// data has already been consumed, to cause end of frame.
		// Implementations that always supply the full packet in _tx_data()
		// can simply supply a dummy version that always returns NULL.
	void (*tx_done)(unsigned err);
		// Called with err = 0 for successful Tx completion, otherwise
		// with error code when an error has occurred - CRC error or
		// the other end simply going away during the transaction.
		// This terminates the whole Tx - there will be no more callbacks.
} EcoLLCallbacks;

// These three invoked by higher layer in response to the rx_scout() callback.

// Acknowledge the scout for a normal 4-way handshake (including 4way imm ops)
// and provide a buffer for the receive data.  If the buffer isn't big
// enough, the rx_extra_buf() callback will be used to get another one.
extern void econet_ll_rx_start(void *inst, uint8_t *buf, unsigned len);

// Acknowledge the scout for an immediate op with 2-way handshake,
// and supply the data to be returned.  data may be NULL and the
// length zero for ops like halt/continue that have a simple ack
// response without any data in it.  Like normal tx, the tx_more_data()
// callback to check for extra data will be done at the end.
extern void econet_ll_rx_2way(void *inst, const uint8_t *data, unsigned len);

// Acknowledge the successful reception of an entire packet,
// authorising the sending of the final ack frame.
// Called in response to (or from) the rx_end() callback.
extern void econet_ll_rx_final(void *inst);

// Initial entrypoint to request a Tx.  Supplied buffer is just the scout:
// packet data is supplied later.
// Returns false if there's already a Tx queued.
// Scout is NOT copied - buffer pointed to is assumed to remain valid
extern bool econet_ll_req_tx(void *inst, const uint8_t *scout, unsigned slen);

// Called in response to the tx_started() callback to supply the
// data, or at least the first chunk of it (optionally later chunks
// via callback).  len is the length of this chunk.
// im2way is true if this is an immediate op with only 2-way handshake.
extern void econet_ll_tx_data(void *inst, const uint8_t *data, unsigned len, bool im2way);


// Cancel the transaction, dropping the line so other end sees not-listening.
// pkt_id is the value from rx_scout callbackto confirm which packet
// is being cancelled.
extern void econet_ll_cancel_rx(void *inst, uint32_t pkt_id);
// Similarly for Tx, though depending on timing it may manage to cancel
// the tx before it even started, or else be too late and it's already done.
extern void econet_ll_cancel_tx(void *inst);


// Initialise one interface for Econet operation at low level.
extern void *econet_ll_init(const EcoHWConfig *hw, const EcoLLCallbacks*callb);
// Initialise one interface at higher level - calls through to low level.
extern void econet_hl_init(const EcoHWConfig *hw);

extern bool watcher_init(const EcoHWConfig *hw);
