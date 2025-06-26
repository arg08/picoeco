/**
 * Econet High-level drivers - RxCBs, TxCBs etc.
 */


extern void board_toggle_led(void);
extern void board_enable_terminator(bool en);
extern void board_enable_clock(unsigned rate);
extern bool board_clock_enabled(void);
extern bool board_terminator_enabled(void);
extern void board_specific_init(void);

// These are inline functions if simple GPIOs, otherwise just prototype
// the actual function in board_specific.c
#if defined(B1M_PIN_BBC_NMI)
static inline void set_b1m_nmi_active(bool en)
{
	gpio_set_dir(B1M_PIN_BBC_NMI, en);
}
#else
static void set_b1m_nmi_active(bool en);
#endif
#if defined(B1M_PIN_BBC_IRQ)
static inline void set_b1m_irq_active(bool en)
{
	gpio_set_dir(B1M_PIN_BBC_IRQ, en);
}
#else
static void set_b1m_irq_active(bool en);
#endif

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

