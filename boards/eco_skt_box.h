// Board configuration for the SJ socket box Econet board.

// This one actually contains a Pico rather than being a dedicated
// RP2040 board, so just add our extra definitions to the standard ones.

#include	<boards/pico.h>

#define	ECONET_PIN_TX_EN	13
#define	ECONET_PIN_TXD		12
#define	ECONET_PIN_RXD		11
#define	ECONET_PIN_CTS		10
#define	ECONET_PIN_CLOCK	9	// Input clock (used by the PIO for HDLC)
#define	ECONET_PIN_CLKOUT	14  // Output clock when enabled: separate pin
								// on this board

#define	ECONET_PIN_CLK_EN	15
#define	ECONET_PIN_TERM_EN	16

// Static assignment of on-chip resources.
// It's too much bother to do this dynamically, for no real benefit.
#define	ECONET_PIO			pio0
