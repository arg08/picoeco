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
#ifdef B1M_PIN_AD_MUX
#include "bus1mhz_muxed.pio.h"
#else
#include "bus1mhzproto.pio.h"
#endif

// Bank of registers for read purposes, accessed by DMA.
// Has to be aligned to its own size.
uint8_t eco_regs[8] __attribute__ ((aligned(8))) = 
{	0x00,	// SR1
	0x24,	// SR2 - clk det in bit 5, idle in bit 2
	0x33,	// RDR
	0x44,	// RDR duplicate
	0x55,	// Int enable
	123,	// Int disable/statid
	0x77,
	0x88};


/*
	Purpose:	IRQ handler for 1MHz bus transfer reports
	Returns:	Nothing
	Notes:		SM0 of the PIO generates a word in the Rx FIFO for
				each transaction on the 1MHz bus, and this handler
				is invoked to process them - implementing writes and
				the side-effects of read-sensitive registers.
				The actual read data is supplied by the other SMs
				autonomously and they do not generate interrupts.
				BBC can't actually generate back-to-back cycles on the
				1MHz bus unless it tries to execute code there
				(which isn't going to be useful anyhow).
				So in practice with a tight loop at best about 4us
				per transfer, and there's an 8-word FIFO so we only
				need to achieve 4us on average and can take longer
				on difficult ones (subject to the demands of Econet
				if running on the same CPU).


				Value in the FIFO has 19 bits of data:
					bits 31..24 Data bits, only useful on writes.
					bit 10: 1=read cycle, 0=write cycle
					bit 9: nFred (can ignore as it's the complement of nJim)
					bit 8: nJim (can treat as an extra address bit)
					bits 0..7 Address bits A[0..7]

				NB. prototype had different layout:
					bit 18: RnW, Bit17 nFred, Bit16 nJim, Addr[7:0], Data[7:0]
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
// First DMA is 32-wide, DREQ from the SM's RxFIFO,
// source RxFIFO, dest 2nd DMA's source-plus-trigger register.
// Second DMA is 8-wide, no DREQ, count 1, dest SM's TxFIFO.
// We trigger DMA1 now and it sits waiting for DREQ; DMA2 gets
// triggered each time by DMA1.
// Ideally, DMA1 would have infinite transfer count and DMA2
// transfer count of 1.  Unfortunately, the hardware doesn't allow an
// infinite count, and 0xffffffff isn't close enough to infinity
// (2^32 transfers at 1MHz would be 4000 seconds; BBC can't actually do
// back-to-back transfers, but could potentially do that many in
// a couple of days).
// So instead we set up both units with transfer count of 1,
// and get DMA2 to re-trigger DMA1 on completion (via the chain trigger).
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
	channel_config_set_chain_to(&cfg, dma1);
	dma_channel_set_write_addr(dma2, &(pio->txf[sm]), false);
	dma_channel_set_trans_count(dma2, 1, false);
	dma_channel_set_config(dma2, &cfg, false);

	// Set up DMA1 and trigger it
	cfg = dma_channel_get_default_config(dma1);
	channel_config_set_read_increment(&cfg, false);
		// write increment defaults to false
	channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, false));
		// transfer size defaults to 32
	dma_channel_set_trans_count(dma1, 1, false);
	dma_channel_set_read_addr(dma1, &(pio->rxf[sm]), false);
	dma_channel_set_write_addr(dma1, &(dma_hw->ch[dma2].al3_read_addr_trig),
		false);
	dma_channel_set_config(dma1, &cfg, true);
}

/*
	Purpose:	Handle interrupt reporting that BBC has been reset
	Returns:	Nothing, IRQ acknowledged
	Notes:
*/

#ifdef B1M_PIN_BBC_nRST
void bbc_nrst_handler(void)
{
	// This is the only enabled GPIO interrupt at present, but be prepared
	// for more since there's only one shared vector for all pins.
	if (gpio_get_irq_event_mask(B1M_PIN_BBC_nRST))
	{
		// We only have falling edge enabled, so that must be the one to ack
		gpio_acknowledge_irq(B1M_PIN_BBC_nRST, GPIO_IRQ_EDGE_FALL);
		printf("BBC nRST!\n");
	}
	else printf("Wild IRQ\n");
}
#endif

void bus1mhz_init(void)
{
	PIO pio = BUS1MHZ_PIO;
	unsigned pin_no, offset;

	// Enble all the pins as inputs and turn on pullups.
	// This is mainly so that the board can be used with the 1MHz bus
	// unplugged - high is an idle condition, and is also the natural
	// state for TTL inputs (pulling low would not work).
	for (pin_no = B1M_PIN_BASE; pin_no < (B1M_PIN_BASE + B1M_PIN_COUNT);
		pin_no++)
	{
		gpio_pull_up(pin_no);

		pio_gpio_init(pio, pin_no);
	}
	// If wired to the CPU, NMI/IRQ/nRST are GPIOs
#ifdef B1M_PIN_NMI
	gpio_init(B1M_PIN_NMI);
	gpio_put(B1M_PIN_NMI, 0);	// Currently an input, ready to drive low
#endif
#ifdef B1M_PIN_IRQ
	gpio_init(B1M_PIN_IRQ);
	gpio_put(B1M_PIN_IRQ, 0);	// Currently an input, ready to drive low
#endif
#ifdef B1M_PIN_BBC_nRST
	gpio_init(B1M_PIN_BBC_nRST);
	gpio_pull_up(B1M_PIN_BBC_nRST);	// In case BBC not connected
	gpio_add_raw_irq_handler(B1M_PIN_BBC_nRST, bbc_nrst_handler);
	gpio_set_irq_enabled(B1M_PIN_BBC_nRST, GPIO_IRQ_EDGE_FALL, true);
	irq_set_enabled(IO_IRQ_BANK0, true);
#endif


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

