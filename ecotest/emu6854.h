/**
 * Econet Low-level drivers - hardware access, packet framing etc.
 */

// Called in interrupt context for 1MHz bus read/write transactions.
// Regno is an offset from the register base (so 0..7)
extern void emu6854_reg_read(unsigned regno);
extern void emu6854_reg_write(unsigned regno, unsigned wval);

// Registers in RAM directly read under DMA to satisfy reads from the BBC

extern uint8_t eco_1mhz_regs[8];

// Initialise one interface for direct emulated 6854 operation.
extern void emu6854_init(const EcoHWConfig *hw);

