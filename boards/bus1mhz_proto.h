
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
#define	BOARD_CTRL_BIT_LED		(1<<3)	// Also EPROM power ena
#define	BOARD_CTRL_BIT_BBC_NMI	(1<<1)
#define	BOARD_CTRL_BIT_BBC_IRQ	(1<<2)


#define	BOARD_CTRL_BIT_A4	(1<<16)
#define	BOARD_CTRL_BIT_A17	(1<<29)

// NB. not defining ECONET_PIN_CLK_EN, ECONET_PIN_TERM_EN, PICO_DEFAULT_LED_PIN

// Pins for the 1MHz bus interface
#define	HAS_1MHZ_BUS	1

#define	B1M_PIN_BASE	1		// First pin is GPIO1
#define	B1M_PIN_COUNT	20		// Total 20 pins used (not counting nRST
								// which doesn't get a dedicated pin)

#define	B1M_PIN_D0		1
#define	B1M_PIN_D1		2
#define	B1M_PIN_D2		3
#define	B1M_PIN_D3		4
#define	B1M_PIN_D4		5
#define	B1M_PIN_D5		6
#define	B1M_PIN_D6		7
#define	B1M_PIN_D7		8
#define	B1M_PIN_A0		9
#define	B1M_PIN_A1		10
#define	B1M_PIN_A2		11
#define	B1M_PIN_A3		12
#define	B1M_PIN_A4		13
#define	B1M_PIN_A5		14
#define	B1M_PIN_A6		15
#define	B1M_PIN_A7		16
#define	B1M_PIN_nJIM	17
#define	B1M_PIN_nFRED	18
#define	B1M_PIN_RnW		19
#define	B1M_PIN_1MHZE	20


#define	ROM_PIN_D0		1
#define	ROM_PIN_D1		2
#define	ROM_PIN_D2		3
#define	ROM_PIN_D3		4
#define	ROM_PIN_D4		5
#define	ROM_PIN_D5		6
#define	ROM_PIN_D6		7
#define	ROM_PIN_D7		8
#define	ROM_PIN_A0		9
#define	ROM_PIN_A1		10
#define	ROM_PIN_A2		11
#define	ROM_PIN_A3		12

#define	ROM_PIN_nWR		16		// Only on 32-pin - A14 is nWR on 28-pin
#define	ROM_PIN_nOE		17		// Jim on 1MHz
#define	ROM_PIN_nCE		19		// RnW on 1MHz




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


