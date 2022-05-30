/**
 * 1MHz bus interface
 */

#include <stdio.h>
#include "pico/stdlib.h"
//#include "pico/bootrom.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/dma.h"

// Our assembled program:
#include "bus1mhz.pio.h"

// Bank of registers for read purposes, accessed by DMA.
// Has to be aligned to its own size.
uint8_t eco_regs[8] __attribute__ ((aligned(8))) = 
{ 0xaa, 0x55, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};


/*
	Purpose:	Exclusive handler for IRQ0  on this PIO unit
	Returns:	Nothing
	Notes:		Currently assumes pio0, SM0 - should support
				multiple interfaces each in own state machine, but
				not easy to do so without adding inefficiency for the
				usual case of only one interface.
*/

static void __not_in_flash_func(pio_irq0_handler)(void)
{
	PIO pio = BUS1MHZ_PIO;

	// Only using interrupts on SM0
	while (pio->ints0 & (PIO_IRQ0_INTS_SM0_RXNEMPTY_BITS << 0))
	{
		uint32_t w = pio->rxf[0];	// SM 0
		printf("main rx %08lx\n", w);
	}
}

// Set up a pair of DMAs to serve one instance of the read SM
// First DMA is 32-wide, DREQ from the SM's RxFIFO, infinite count,
// source RxFIFO, dest 2nd DMA's source-plus-trigger register.
// Second DMA is 8-wide, no DREQ, count 1, dest SM's TxFIFO.
// We trigger DMA1 now and it sits waiting for DREQ; DMA2 gets
// triggered each time by DMA1.
static void setup_dma(unsigned sm)
{
	unsigned dma1, dma2;
	dma_channel_config cfg;
	PIO pio = BUS1MHZ_PIO;

	dma1 = dma_claim_unused_channel(true);
	dma2 = dma_claim_unused_channel(true);
	printf("Using DMA%u, DMA%u for SM %u\n", dma1, dma2, sm);

	// Set up DMA2 first (it's not triggered until DMA1 does so)
	cfg = dma_channel_get_default_config(dma2);
	channel_config_set_read_increment(&cfg, false);
		// write increment defaults to false
		// dreq defaults to DREQ_FORCE
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
	dma_channel_set_write_addr(dma2, &(pio->txf[sm]), false);
	dma_channel_set_trans_count(dma2, 1, false);
	dma_channel_set_config(dma2, &cfg, false);

	// Set up DMA1 and trigger it
	cfg = dma_channel_get_default_config(dma1);
	channel_config_set_read_increment(&cfg, false);
		// write increment defaults to false
	channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, false));
		// transfer size defaults to 32
	dma_channel_set_trans_count(dma1, 0xffffffff, false);
	dma_channel_set_read_addr(dma1, &(pio->rxf[sm]), false);
	dma_channel_set_write_addr(dma1, &(dma_hw->ch[dma2].al3_read_addr_trig),
		false);
	dma_channel_set_config(dma1, &cfg, true);
}

void bus1mhz_init(void)
{
	PIO pio = BUS1MHZ_PIO;
	unsigned offset;

	// Most of the pins are inputs; just the databus can be outputs
	// and so need to be pinmuxed to the PIO.
	for (offset = 0; offset < 8; offset++)
		pio_gpio_init(pio, B1M_PIN_D0 + offset);

	// Set all the pins to have pullups.
	// This is mainly so that the board can be used with the 1MHz bus
	// unplugged - high is an idle condition, and is also the natural
	// state for TTL inputs (pulling low would not work).
	for (offset = 0; offset < B1M_PIN_COUNT; offset++)
		gpio_pull_up(B1M_PIN_BASE + offset);

	// One instance of the main program - this gives us non-time-critical
	// notification of all 1MHz bus events down the Rx FIFO.
	// Put this in SM0
    offset = pio_add_program(pio, &bus1mhz_main_program);
printf("Main program at offset %d\n", offset);
	bus1mhz_main_program_init(pio, 0, offset);

	// One or more instances of the read engine, with an associated pair
	// of DMAs to transfer the data from the register block to the PIO
	offset = pio_add_program(pio, &bus1mhz_read8_program);

	// SM1 - econet registers at 0xfc28
	bus1mhz_read8_program_init(pio, 1, offset, 0xfc28, eco_regs);
	setup_dma(1);
	// XXXX SM2, SM3 for further instances


	// Hook the interrupt handler for SM0
	irq_set_exclusive_handler((pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0,
		pio_irq0_handler);

	// We leave the interrupts permanently enabled in the NVIC,
	// but mask individual sources from time to time.
	irq_set_enabled((pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0, true);

	// Enable the FIFO Rx interrupt
	// These enable bits are in order of sm number
	hw_set_bits(&pio->inte0, (PIO_IRQ0_INTE_SM0_RXNEMPTY_BITS << 0));

    // Set the state machines running
    pio_sm_set_enabled(pio, 0, true);
    pio_sm_set_enabled(pio, 1, true);

	printf("1MHz bus running\n");
}

