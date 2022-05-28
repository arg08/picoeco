/**
 * Econet High-level drivers - RxCBs, TxCBs etc.
 */


extern void board_toggle_led(void);
extern void board_enable_terminator(bool en);
extern void board_enable_clock(unsigned rate);
extern bool board_clock_enabled(void);
extern bool board_terminator_enabled(void);
extern void board_specific_init(void);
