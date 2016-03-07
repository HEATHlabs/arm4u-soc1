/**********************************************************************
 *
 * Function:    memTest()
 *
 * Description: Test a 4-k chunk of SRAM.
 *
 * Notes:       
 *
 * Returns:     0 on success.
 *              Otherwise -1 indicates failure.
 *
 **********************************************************************/
#include "../mini-libc/stdio.h"
#include "memtest.h"
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
}

int
main(void)
{
#define BASE_ADDRESS  (volatile datum *) 0x01000000
//#define NUM_BYTES     ( 32 * 1024 * 1024 ) - 0x00002000
#define NUM_BYTES    32 * 1024 
    int result = 0;
	unsigned char str[10]="\0";
	
	datum *result_ptr=NULL;
	

     printf("Starting MEMTEST tests\r\n");
     printf("BASE ADDRESS:  ");
     long_to_hex_string((unsigned long ) BASE_ADDRESS, str, 8);
     printf(str);printf("\r\n");
     printf("Number of Bytest:  ");
     long_to_hex_string((unsigned long ) NUM_BYTES, str, 8);
     printf(str);printf("\r\n");

    result = memTestDataBus(BASE_ADDRESS);
    if ( result != 0)
    {
        printf("memTestDataBus FAILED\r\n");
		return (-1);
    }
    else
    {
        printf("memTestDataBus Passed\r\n");

    }
	result_ptr = memTestAddressBus(BASE_ADDRESS, NUM_BYTES);
    if ( result_ptr != NULL)
    {
       printf("memTestAddressBus FAILED ");
	   long_to_hex_string((unsigned long ) *result_ptr, str, 8);
		printf(str);printf("\r\n");
		//return (-1);
    }
    else
    {
        printf("memTestAddressBus Passed\r\n");
    }
	
    result_ptr = memTestDevice1(BASE_ADDRESS, NUM_BYTES);
    if ( result_ptr != NULL)
    {
        printf("memTestDevice1 FAILED ");
		long_to_hex_string((unsigned long ) *result_ptr, str, 8);
		printf(str);printf("\r\n");
		return (-1);
    }
    else
    {
        printf("memTestDevice1 Passed\r\n");
		//return (0);
    }

    result_ptr = memTestDevice2(BASE_ADDRESS, NUM_BYTES);
    if ( result_ptr != NULL)
    {
        printf("memTestDevice2 FAILED ");
		long_to_hex_string((unsigned long ) *result_ptr, str, 8);
		printf(str);printf("\r\n");
		return (-1);
    }
    else
    {
        printf("memTestDevice2 Passed\r\n");
		//return (0);
    }

}   /* memTest() */





