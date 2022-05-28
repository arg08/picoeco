
// Board configuration for the BBC micro 1MHz Bus Econet interface

#define	ECONET_PIN_TX_EN	23
#define	ECONET_PIN_TXD		24
#define	ECONET_PIN_RXD		22
#define	ECONET_PIN_CTS		29
#define	ECONET_PIN_CLOCK	21	// Input clock (used by the PIO for HDLC)
#define	ECONET_PIN_CLKOUT	21	// Output clock when enabled: same pin here

#define	HAS_SPI_CTRL_REG	1
#define	BOARD_CTRL_SPI	spi1	// To match these pin numbers
#define	PIN_BOARD_CTRL_TX	27
#define	PIN_BOARD_CTRL_CLK	26
#define	PIN_BOARD_CTRL_CS	25

#define	BOARD_CTRL_BIT_CLK_EN	(1<<4)
#define	BOARD_CTRL_BIT_TERM_EN	(1<<5)
#define	BOARD_CTRL_BIT_LED		(1<<3)
#define	BOARD_CTRL_BIT_BBC_NMI	(1<<1)
#define	BOARD_CTRL_BIT_BBC_IRQ	(1<<2)

// NB. not defining ECONET_PIN_CLK_EN, ECONET_PIN_TERM_EN, PICO_DEFAULT_LED_PIN

// --- UART ---
#define PICO_DEFAULT_UART 0
#define PICO_DEFAULT_UART_TX_PIN 0
// No RX pin available

// --- FLASH ---

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

// Static assignment of on-chip resources.
// It's too much bother to do this dynamically, for no real benefit.
#define ECONET_PIO		pio0
#define	BUS1MHZ_PIO		pio1
#define	BUS1MHZ_DMACH_A	0			// First DMA channel
#define	BUS1MHZ_DMACH_B	1			// Second DMA channel


