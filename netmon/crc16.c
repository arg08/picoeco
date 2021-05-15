
#include <stdint.h>
#include "crc16.h"

/*
//                                      16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
// This works out to be 0x1021, but the way the algorithm works
// lets us use 0x8408 (the reverse of the bit pattern).  The high
// bit is always assumed to be set, thus we only use 16 bits to
// represent the 17 bit value.
*/

#define CRC16_POLY 0x8408

/* This version is bit-wise and shows the underlying algorithm	*/
/* but too slow for practical use.								*/
#if UNIT_TEST
uint16_t crc16_bitwise(const uint8_t *data_p, unsigned  length)
{
	unsigned i;
	uint8_t  data;
	uint16_t crc = 0xffff;

	while (length--)
	{
		for (i = 0, data= *data_p++; i < 8; i++, data >>= 1)
		{
			if ((crc & 0x0001) ^ (data & 0x0001))
				crc = (crc >> 1) ^ CRC16_POLY;
			else  crc >>= 1;
		}
	}
	return (~crc);
}
#endif

/* init_crc16_table -------------------------------------------------------- */
/*
    Purpose:	Construct table for byte-wise CRC16 calculation
    Returns:	Nothing, global crc16_table[] initialised.
	Notes:		The table is now public, so that the code using it
				can be inlined via the header file.
*/

uint16_t crc16_table[256];

void init_crc16_table(void)
{
	int i, j;
	uint16_t crc;

	for (i = 0; i < 256; i++)
	{
		crc = i;
		for (j = 0; j < 8; j++)
    	{
			if (crc & 1)
				crc = (crc >> 1) ^ CRC16_POLY;
			else
				crc = ( crc >> 1 );
		}
		crc16_table[i] = crc;
	}
}

/* crc16_block ------------------------------------------------------------ */
/*
    Purpose:	Compute CRC16 over a block of data, by table method.
    Returns:	Working value, suitable for re-input for further blocks
    Notes:		Input value should be 0xffff for the first block,
				else return value from previous call.
				Final result is the 1's complement of the return value.
*/

uint16_t crc16_block(uint16_t crc, const uint8_t  *pData, unsigned blk_len)
{
	unsigned i, j;
	for (j = 0; j < blk_len; j++)
	{
		i = (crc ^ *pData++) & 0xff;
		crc = (crc >> 8) ^ crc16_table[i];
	}
	return crc;
}


#if UNIT_TEST
#include <stdio.h>
// Arbitrary data for testing
static const uint8_t pkt1[] = {0x04, 0xe7, 0xd7, 0x57, 0x37, 0xe0, 0x3f,
  0xf3, 0x3c, 0x8d, 0x60, 0x34, 0x12, 0x14, 0x0e, 0x10, 0xff, 0xf8, 0x00,
  0x00, 0x00, 0x7e, 0x0f};
static const uint8_t pkt2[] = "Hello there!";
static const uint8_t pkt3[] = {0xfe, 0x00, 0x02, 0x00, 0x80, 0x99};

int main(int argc, char **argv)
{
	uint16_t acc;
	unsigned count;
	printf("len %d\n", (int)sizeof(pkt1));
	printf("CRC pkt1: %04x\n", crc16_bitwise(pkt1, sizeof(pkt1)));
	printf("CRC pkt2: %04x\n", crc16_bitwise(pkt2, sizeof(pkt2)));
	printf("CRC pkt3: %04x\n", crc16_bitwise(pkt3, sizeof(pkt3)));
	printf("Correct answers are 0x0f47, 0x9fb4, 0xb27c\n");

	printf("By block method:\n");
	init_crc16_table();
	printf("CRC pkt1: %04x\n", ~crc16_block(0xffff, pkt1, sizeof(pkt1)));
	printf("CRC pkt2: %04x\n", ~crc16_block(0xffff, pkt2, sizeof(pkt2)));
	printf("CRC pkt3: %04x\n", ~crc16_block(0xffff, pkt3, sizeof(pkt3)));

	printf("By accumulator method\n");
	acc = 0xffff;
	for (count = 0; count < sizeof(pkt1); count++)
		acc = crc16_add_byte(acc, pkt1[count]);
	printf("CRC pkt1: %04x\n", ~acc);
	acc = 0xffff;
	for (count = 0; count < sizeof(pkt2); count++)
		acc = crc16_add_byte(acc, pkt2[count]);
	printf("CRC pkt2: %04x\n", ~acc);
	acc = 0xffff;
	for (count = 0; count < sizeof(pkt3); count++)
		acc = crc16_add_byte(acc, pkt3[count]);
	printf("CRC pkt3: %04x\n", ~acc);
	return 0;
}
#endif
