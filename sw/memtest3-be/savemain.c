#include "../lib/storm_core.h"
#include "../lib/storm_soc_basic.h"
#include "../lib/io_driver.c"
#include "../lib/uart.c"
#include "../lib/utilities.c"
#include "memtest.h"
#include "memtest.c"
#include <stdlib.h>

#define NULL  (void *) 0

static unsigned long int hex_string_to_long ( unsigned char *hexstr, int charlen )
{
	unsigned long int hexval =  0 ;
	int i;

	for (i=0 ; i< (charlen*2) ; i++ ) {
		if (( hexstr[i] >= 48 ) & (hexstr[i] <= 57 )) 
			hexval = (hexval <<4 ) + hexstr[i] - (int) '0';
		else if (( hexstr[i] >= 65 ) & ( hexstr[i] <= 70 )) 
			hexval = (hexval <<4 ) + hexstr[i] + 10 - (int) 'A';
		else if (( hexstr[i] >= 97 ) & ( hexstr[i] <= 102)) 
			hexval = (hexval <<4 ) + hexstr[i] +10 - (int) 'a';
	} //for
 
	return hexval;
}
/*
static unsigned long int qbytes_to_long ( unsigned char *buffer )
{
	unsigned char d = buffer[0];
	unsigned char c = buffer[1];
	unsigned char b = buffer[2];
	unsigned char a = buffer[3];
	unsigned long n;


	n = a|(b<<8)|(c<<16)|(d<<24);
	return n;
}
*/

// ############################################################################################
// STORM MemTest
   int main(void)
// ############################################################################################
{
	int function_sel;
	datum result, *result_ptr;
	unsigned long int adr_buffer,  start_address;
	unsigned char  char_tmp, *char_pointer;
	
	char_tmp = 'n'; //enable mem test menu

	// show reset ack
	io_set_gpio0_port(0);
	set_syscpreg(0xC3, SYS_IO);

		// enable write-through strategy
	set_syscpreg(get_syscpreg(SYS_CTRL_0) | (1<<DC_WTHRU), SYS_CTRL_0);



	// Intro screen
	uart0_printf("\r\n\r\n\r\n+----------------------------------------------------------------+\r\n");

	uart0_printf("   < Welcome to the STORM SoC memtest >\r\n < Select an operation from the menu below or press >\r\n");
	uart0_printf( "   < the boot key for immediate application start. >\r\n\r\n");

	// Console menu
//uart0_printf(test_str); uart0_printf("\r\n");
//adr_buffer = hex_string_to_long(test_str,4);
//long_to_hex_string( adr_buffer, test_str2,8);
//uart0_printf(test_str2); uart0_printf("\r\n");


	uart0_printf(" 0 - memTestDataBus Test\r\n");
	uart0_printf(" 1 - memTestAddressBus Test\r\n");
	uart0_printf(" 2 - memTestDevice Test\r\n");
	uart0_printf(" 3 - All memTest* Tests\r\n");
	uart0_printf(" r - restart system\r\n\r\nSelect:");
//uart0_printf(test_str); uart0_printf("\r\n");
	while(1){
		// console input
		function_sel = io_uart0_read_byte();

//main_menu:

		// main functions
		switch(function_sel){

			// memTestDataBus
			// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			case '0':
				io_uart0_send_byte((char)function_sel);
				uart0_printf(" Begin Address in form of 0000ffff:  ");
				uart0_scanf(char_pointer, 8, ECHO_ON);
				start_address = (datum) hex_string_to_long ( char_pointer, 4 );
				result = memTestDataBus( &start_address );
				if ( result == 0 ) 
					uart0_printf ("\r\n memTestDataBus Passed!\r\n");
				else {
					long_to_hex_string((datum)result, char_pointer, 4);
					uart0_printf("memTestDataBus failure at:") ;
					uart0_printf(char_pointer);
					uart0_printf("\r\n");
				}

				break;

			// memTestAddressBus
			// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			case '1':
				io_uart0_send_byte((char)function_sel);
				uart0_printf(" Begin Address in form of 0000ffff:  ");
				uart0_scanf(char_pointer, 8, ECHO_ON);
				start_address = hex_string_to_long ( char_pointer, 4 );
				uart0_printf("\r\n Number of bytes to test in form of  0000ffff:  ");
				uart0_scanf(char_pointer, 8, ECHO_ON);
				uart0_printf("\r\n");
				adr_buffer = hex_string_to_long ( char_pointer, 4 );

				result =  * memTestAddressBus( &start_address, adr_buffer );
				if (result == (datum) NULL ) 
					uart0_printf (" memTestAddressBus Passed!\r\n");
				else {
					long_to_hex_string( result, char_pointer, 4);
					uart0_printf(" memTestAddressBus failure at:") ;
					uart0_printf(char_pointer);
					uart0_printf("\r\n");
				}
				break;


			// memTestDevice 
			// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			case '2':
				io_uart0_send_byte((char)function_sel);
				uart0_printf(" Begin Address in form of 0000ffff:  ");
				uart0_scanf(char_pointer, 8, ECHO_ON);
				start_address = hex_string_to_long ( char_pointer, 4 );
				uart0_printf(" Number of bytes to test in form of  0000ffff:  ");
				uart0_scanf(char_pointer, 8, ECHO_ON);
				uart0_printf("\r\n");
				adr_buffer = hex_string_to_long ( char_pointer, 4 );

				result = * memTestDevice ( &start_address, adr_buffer );
				if (result == (datum) NULL ) 
					uart0_printf (" memTestDevice  Passed!\r\n");
				else {
					long_to_hex_string(  *result_ptr, char_pointer, 4);
					uart0_printf(" memTestDevice  failure at:") ;
					uart0_printf(char_pointer);
					uart0_printf("\r\n");
				}
				break;
			
			// All memTest* 
			// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			case '3':
				io_uart0_send_byte((char)function_sel);
				uart0_printf(" Begin Address in form of 0000ffff:  ");
				uart0_scanf(char_pointer, 8, ECHO_ON);
				start_address = hex_string_to_long ( char_pointer, 4 );
				uart0_printf(" Number of bytes to test in form of  0000ffff:  ");
				uart0_scanf(char_pointer, 8, ECHO_ON);
				uart0_printf("\r\n");
				adr_buffer = hex_string_to_long ( char_pointer, 4 );
			//memTestDataBus
				result = memTestDataBus( &start_address );
				if (result == 0 ) 
					uart0_printf ("memTestDataBus Passed!\r\n");
				else {
					long_to_hex_string( (unsigned long) result, char_pointer, 4);
					uart0_printf("memTestDataBus failure at:") ;
					uart0_printf(char_pointer);
					uart0_printf("\r\n");
				}
				
			//memTestAddressBus	
				result = * memTestAddressBus( &start_address, adr_buffer );
				if (result == (datum) NULL ) 
					uart0_printf (" memTestAddressBus Passed!\r\n");
				else {
					long_to_hex_string( *result_ptr, char_pointer, 4);
					uart0_printf(" memTestAddressBus failure at:") ;
					uart0_printf(char_pointer);
					uart0_printf("\r\n");
				}

				
			//memTestDevice
				result_ptr = * memTestDevice ( &start_address, adr_buffer );
				if (result == (datum) NULL ) 
					uart0_printf (" memTestDevice  Passed!\r\n");
				else {
					long_to_hex_string(*result_ptr, char_pointer, 4);
					uart0_printf(" memTestDevice  failure at:") ;
					uart0_printf(char_pointer);
					uart0_printf("\r\n");
				}
				break;



			// restart system
			// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			case 'r':
				io_uart0_send_byte((char)function_sel);
				asm volatile ("mov r0,     #0x0FF00000");
				asm volatile ("add pc, r0, #0xF0000000"); // jump to bootloader
				while(1);
				break;

			// no input
			// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			case -1:
				break;
uart0_printf("no_input");

			// invalid selection
			// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			default:
				io_uart0_send_byte((char)function_sel);
				uart0_printf(" Invalid operation!\r\nTry again: ");
				break;

		}
//		uart0_printf (" Done testing? y or n:\r\n");
//		char_tmp = io_uart0_read_byte();
	}

//	while(1);
}
