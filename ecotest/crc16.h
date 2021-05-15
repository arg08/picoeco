
#include <stdint.h>

uint16_t crc16_table[256];

extern void init_crc16_table(void);

// Initial crc parameter should be 0xffff, then function can be called again
// with further chunks of data and passing the previous result as the new
// input CRC.  Final result should be 1's complemented before use.
extern uint16_t crc16_block(uint16_t crc, const uint8_t  *pData, unsigned blk_len);


// Byte-at-a-time version
// A crc accumulator should be initialised to 0xffff at start of packet, then
// each byte passed to this function, with the return value updating
// the accumulator.  At end of packet, value to use is 1's complement
// of the accumulator value.

static inline uint16_t crc16_add_byte(uint16_t old_crc, uint8_t b)
{
	return (old_crc >> 8) ^ crc16_table[(old_crc ^ b) & 0xff];
}

//  Alternatively, on Rx the whole packet including CRC can be put through
// the algorithm and the result should be 0xf0b8
// (would be 0x0f47 if complemented)
#define	VALID_CRC16_FINAL	(0xf0b8)
