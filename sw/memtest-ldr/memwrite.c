#include "../lib/io_driver.h"
#include "../lib/io_driver.c"
#include "../lib/uart.h"
#include "../lib/uart.c"



// ############################################################################################
// Convert 4/8/12/16/20/24/28/32 bit hexadecimal value to ASCII string
   void long_to_hex_string(unsigned long data,    // max 32 bit data word
                           unsigned char *buffer, // buffer to store the string
						   unsigned char numbers) // number of places, max 8
// ############################################################################################
{
	unsigned char temp_char = 0;
	unsigned long temp_data = 0;

	// fit into range
	if(numbers > 8)
		numbers = 8;
	if(numbers < 1)
		numbers = 1;

	while(numbers > 0){
		// isolate one 4-bit value
		if(numbers > 1)
			temp_data = data >> ((numbers-1)*4);
		else
			temp_data = data;
		temp_data = temp_data & 0x0000000F;
		numbers--;

		// convert 4-bit value temp_data to char temp_char
		if(temp_data < 10)
			temp_char = '0' + temp_data;
		else
			temp_char = 'A' + temp_data - 10;

		// save character
		*buffer++ = temp_char;
	}

	*buffer++ = 0; // terminate string
} //long_to_hex_string



// void dumpmem (volatile unsigned long  * startAddr, unsigned long numBytes) {
	// unsigned long  offset;
	// unsigned long *data_pointer, word_buffer;
	// unsigned char str[10]="\0";
				// uart0_printf("\r\n\r\nAbort dumping by pressing any key.\r\nPress any key to continue.\r\n\r\n");
				// while(io_uart0_read_byte() == -1);
				// while(io_uart0_read_byte() != -1);
				// data_pointer = startAddr ;
				// //* data_pointer = (volatile unsigned  long *) START_ADDRESS; 
				// for (offset = 0; offset < (NUM_BYTES/4); offset=offset+1)
				// {
					// word_buffer = startAddr[offset] ;
					// long_to_hex_string ( word_buffer, str, 8);
					// uart0_printf(str);uart0_printf("  ");
					// //data_pointer=data_pointer + 4;
					// if(io_uart0_read_byte() != -1){
						// break;
						// uart0_printf("\r\n\r\nAborted!");
					// }
				// }
				// uart0_printf("\r\n\r\nDumping completed.\r\n\r\nSelect: ");
// }//dumpmem


//#define START_ADDRESS 0x00004000
#define START_ADDRESS 0x00220000
#define NUM_BYTES  2048
#define BASE_ADDRESS  (volatile unsigned long *) START_ADDRESS
#define TEST_PATTERN START_ADDRESS


int
main(void)
{
/*
		constant INT_MEM_BASE_C    : STD_LOGIC_VECTOR(31 downto 0) := x"00000000";
		constant INT_MEM_SIZE_C    : natural := 32*1024; -- byte
		constant SDRAM_MEM_BASE_C   : STD_LOGIC_VECTOR(31 downto 0):= x"00009000";
		constant SDRAM_MEM_SIZE_C   : natural := 32*1024*1024; -- byte  02009000
		constant BOOT_ROM_BASE_C   : STD_LOGIC_VECTOR(31 downto 0) := x"FFF00000";
		constant BOOT_ROM_SIZE_C   : natural := 8*1024; -- byte
*/
/*
tips:
*(volatile int *) 0x10003456 = 0x12345678; // writes the 32 bit value into memory

x = *(volatile char *) 0x10001234; // read a byte wide value

*/

unsigned long  offset, nWords, pattern, word_buffer;
volatile unsigned long  *baseAddress;
unsigned char str[10]="\0";

nWords = NUM_BYTES / 4;
 * baseAddress = (volatile unsigned  long *) START_ADDRESS;
uart0_printf("Writing test pattern to memory...\r\n");
pattern = 10;
//for (pattern = START_ADDRESS, offset = 0; offset <= nWords; offset=offset+1)
for ( offset = 0; offset <= nWords; offset=offset+1)
    {
        baseAddress[offset] = pattern;
		uart0_printf(".");
		// word_buffer = baseAddress[offset] ;
		// long_to_hex_string ( word_buffer, str, 8);
		// uart0_printf(str);uart0_printf("  ");
		 pattern=pattern+4;
    }
uart0_printf("...done writing test pattern to memory...\r\n");

//dumpmem (baseAddress , NUM_BYTES) ;
uart0_printf("Dumping memory\r\n");

for (offset = 0; offset <= nWords; offset=offset+1)
{
		word_buffer = baseAddress[offset] ;
		long_to_hex_string ( word_buffer, str, 8);
		uart0_printf(str);uart0_printf("  ");

}
uart0_printf("\r\n\r\nDumping completed.\r\n\r\nSelect: ");				for (offset = 0; offset < (NUM_BYTES/4); offset=offset+1)


asm volatile ("mov r0,     #0x0FF00000");
asm volatile ("add pc, r0, #0xF0000000"); // jump to bootloader

}

