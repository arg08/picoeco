
// Board configuration for the BBC micro 1MHz Bus Econet interface

#define	ECONET_PIN_TX_EN	2
#define	ECONET_PIN_TXD		3
#define	ECONET_PIN_RXD		1
#define	ECONET_PIN_CTS		28
#define	ECONET_PIN_CLOCK	29	// Input clock (used by the PIO for HDLC)
#define	ECONET_PIN_CLKOUT	29	// Output clock when enabled: same pin here
								// Must be odd-numbered GPIO (PWM B channel)
#define	ECONET_PIN_CLK_EN	9
#define	ECONET_PIN_TERM_PUMP	8	// Voltage pump for terminator enable
#define	ECONET_PIN_CD_LEVEL	0	// GPIO0 - PWM to set collision detect level
#define	ECONET_PIN_COL_MON	27	// Analogue measurement of Tx current

#define	HAS_SPI_CTRL_REG	0

// SPI instance here wired to SD card socket
#define	PIN_SPI_MISO		4
#define	PIN_SPI_nCS			5
#define	PIN_SPI_MOSI		6
#define	PIN_SPI_CLK			7

// --- UART ---
// Pins available either for UART debug or for SPI-only SD card.
#define PICO_DEFAULT_UART 1
#define PICO_DEFAULT_UART_TX_PIN 4
#define PICO_DEFAULT_UART_RX_PIN 5


#define	PIN_LED			10
#define	PICO_DEFAULT_LED_PIN	10


// NB. not defining ECONET_PIN_CLK_EN, ECONET_PIN_TERM_EN

// Pins for the 1MHz bus interface
#define	HAS_1MHZ_BUS	1

#define	B1M_PIN_BASE	11		// First pin is GPIO11
#define	B1M_PIN_COUNT	16		// Total 16 pins used (inc NMI etc)

#define	B1M_PIN_A0_D0		16	// These first 8 pins are muxed
#define	B1M_PIN_A1_D1		17
#define	B1M_PIN_A2_D2		18
#define	B1M_PIN_A3_D3		19
#define	B1M_PIN_A4_D4		20
#define	B1M_PIN_A5_D5		21
#define	B1M_PIN_A6_D6		22
#define	B1M_PIN_A7_D7		23
#define	B1M_PIN_nJIM		24
#define	B1M_PIN_nFRED		25
#define	B1M_PIN_RnW			26

#define	B1M_PIN_1MHZE		11
#define	B1M_PIN_BBC_NMI		12
#define	B1M_PIN_BBC_IRQ		13
#define	B1M_PIN_BBC_nRST	14
#define	B1M_PIN_AD_MUX		15	// Read data when low, address when high.
								// NB. data undriven when muxed away

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


