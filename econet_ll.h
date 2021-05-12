/**
 * Econet Low-level drivers - hardware access, packet framing etc.
 */

// Econet hardware setup.  Note that we need the whole of a PIO
// so can't share with other PIO functions, but can have multiple
// Econet instances in the same PIO.
typedef struct
{
	PIO pio;			// Which PIO unit to use?
	uint8_t sm;			// Which SM within that PIO?
	uint8_t clk_pin;	// Econet clock input pin number
	uint8_t rxd_pin;	// Econet RxD input pin number
	uint8_t txd_pin;	// Econet TxD output pin number
	uint8_t txen_pin;	// Econet Tx enable output pin number
	uint8_t cts_pin;	// Econet CTS (/collision detect) input pin number
} EcoHWConfig;


extern void econet_ll_init(const EcoHWConfig *hw);
