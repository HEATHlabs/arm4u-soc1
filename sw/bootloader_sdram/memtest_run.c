// #include "../lib/io_driver.h"
// #include "../lib/io_driver.c"
// #include "../lib/uart.h"
// #include "../lib/uart.c"
#include "../memtest3/memtest.h"
#include "../memtest3/memtest.c"

#define BASE_ADDRESS  (volatile datum *) 0x01000000 //beginning of SDRAM memory
#define NUM_BYTES      32 * 1024 * 1024

void memtest_run () {
	int result = 0;
	unsigned char str[10]="\0";
	
	datum *result_ptr=NULL;
    result = memTestDataBus(BASE_ADDRESS);
    if ( result != 0)
    {
        uart0_printf("memTestDataBus FAILED\r\n");
		return (-1);
    }
    else
    {
        uart0_printf("memTestDataBus Passed\r\n");

    }
	result_ptr = memTestAddressBus(BASE_ADDRESS, NUM_BYTES);
    if ( result_ptr != NULL)
    {
       uart0_printf("memTestAddressBus FAILED ");
	   long_to_hex_string((unsigned long ) result_ptr, str, 8);
		uart0_printf(str);uart0_printf("\r\n");
		//return (-1);
    }
    else
    {
        uart0_printf("memTestAddressBus Passed\r\n");
    }

    result_ptr = memTestDevice1(BASE_ADDRESS, NUM_BYTES);
    if ( result_ptr != NULL)
    {
        uart0_printf("memTestDevice1 FAILED ");
		long_to_hex_string((unsigned long ) result_ptr, str, 8);
		uart0_printf(str);uart0_printf("\r\n");
		//return (-1);
    }
    else
    {
        uart0_printf("memTestDevice1 Passed\r\n");
		//return (0);
    }

	
    result_ptr = memTestDevice2(BASE_ADDRESS, NUM_BYTES);
    if ( result_ptr != NULL)
    {
        uart0_printf("memTestDevice2 FAILED ");
		long_to_hex_string((unsigned long ) result_ptr, str, 8);
		uart0_printf(str);uart0_printf("\r\n");
		//return (-1);
    }
    else
    {
        uart0_printf("memTestDevice2 Passed\r\n");
		//return (0);
    }
} //memtest_run()