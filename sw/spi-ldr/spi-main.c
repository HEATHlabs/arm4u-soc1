/*----------------------------------------------------------------
//                                                              //
//  spi-main.c                                               //
//                                                              //
----------------------------------------------------------------*/

#include "stdio.h"
#include "../include/amber_registers.h"

#define TINY_SPI_STATUS_TXE 0x1
#define TINY_SPI_STATUS_TXR 0x2
#define DUMMY 0x0
#define MFGID 0
#define DEVID 1
#define SPIWAIT 20
#define SPI0CS 0

typedef  unsigned long datum;

//verified
void delay(int time) // waits time * 1usec @ 40MHz
{
	time = time*40;
	while(time > 0)
	{
		asm volatile ("NOP");
		time--;
	}
}

//verified
void bit_clear ( volatile datum *addr, int bitno)
{
	*addr &= ~(1<<bitno);
}


//verified
void bit_set (volatile datum *addr, int bitno)
{
	*addr |= (1<<bitno);
}

//verified
void dump_pio_regs () {
	printf("PIO output reg: %x \n\r",*((datum *) ADR_AMBER_PIO_RGPIO_OUT));
	printf("PIO input reg: %x \n\r",*((datum *) ADR_AMBER_PIO_RGPIO_IN));
	printf("PIO output enable: %x \n\r",*((datum *) ADR_AMBER_PIO_RGPIO_OE));

}

// dump_spi_regs
// base+0  R shift register
// base+4  R buffer register
		// W buffer register
// base+8  R status
		// [1] TXR transfer ready
		// [0] TXE transter end
		// W irq enable
		// [1] TXR_EN transfer ready irq enable
		// [0] TXE_EN transter end irq enable
// base+12 W control (optional)
		// [1:0] spi mode
// base+16 W baud divider (optional)
//verified
void dump_spi_regs ()
{

	unsigned char c;
//	c= *((datum volatile *)ADR_AMBER_SPI0_RXD) ;
	printf ("Shift Reg:   %x\n\r", *((datum volatile *)ADR_AMBER_SPI0_RXD));
//	c= *((datum volatile *)ADR_AMBER_SPI0_TXD);
	printf ("Buffer Reg:  %x\n\r", *((datum volatile *)ADR_AMBER_SPI0_TXD));
//	c= *((datum volatile *)ADR_AMBER_SPI0_STATUS);
	printf ("Status Reg:  %x\n\r", *((datum volatile *)ADR_AMBER_SPI0_STATUS));
//	c= *((datum volatile *)ADR_AMBER_SPI0_CONTROL);
	printf ("Control Reg: %x \n\r",  *((datum volatile *)ADR_AMBER_SPI0_CONTROL));
	//c= *((datum *)ADR_AMBER_SPI0_BAUD_DIV);
	printf ("Buad control : %x (not readable, always 0)\n\r", (*(datum volatile *) ADR_AMBER_SPI0_BAUD_DIV) );
}




//put single byte out on SPI Bus
//verified
void out_spi( datum *spibuffer, unsigned char c)  //SPI0 only
{
//	printf("out_spi() status reg: %x\n\r",(*(datum volatile *)ADR_AMBER_SPI0_STATUS));
	while ( !((*(datum volatile *)ADR_AMBER_SPI0_STATUS) & TINY_SPI_STATUS_TXE) ) //Wait until ready to send
	{
		delay (SPIWAIT);
//		printf ("waiting...");
	}
//	printf("out_spi() status reg: %x\n\r",(*(datum volatile *)ADR_AMBER_SPI0_STATUS));

	*spibuffer = c;
}

//verified
unsigned char in_spi( datum volatile *spibuffer)  //SPI0 only
{
	unsigned char c;
	while ( !((*(datum volatile *)ADR_AMBER_SPI0_STATUS) & TINY_SPI_STATUS_TXR) ) //Wait until ready to send
	{
		delay (SPIWAIT);
//		printf ("waiting...");
	}
//	printf("in_spi() status reg: %x\n\r",(*(datum volatile *)ADR_AMBER_SPI0_STATUS));

	c = *spibuffer ;
	return c ;
}

//verified
void pio_init()   //SPI0 only
{
		//read OutputEnable reg
		bit_set ( ((datum *)ADR_AMBER_PIO_RGPIO_OE), 0);
		//Write 1 to bit 0
		bit_set ( ((datum *)ADR_AMBER_PIO_RGPIO_OUT), 0);
}


// SPI init, set baud divider
//verified
void spi_init()   //SPI0 only
{
	*(volatile datum *) ADR_AMBER_SPI0_BAUD_DIV = 0x28; // 40Mhz/40 = 1Mhz
}

// ee25xxx_read()
char ee25xxx_read ( int address, int portaddr)   //SPI0 only
{
	char mem_data;
	int HiByte, MidByte, LoByte;
	HiByte=address>>16; /*mask off lower 8 bits*/
	MidByte = (address >> 8) & 0b0000000011111111;
	LoByte=address & 0b000000000000000011111111; /*mask off 16 higher bits*/
	printf ("read H: %x, M: %x, L: %x\n\r", HiByte, MidByte, LoByte);
	bit_clear(((datum *)ADR_AMBER_PIO_RGPIO_OUT), SPI0CS); /*set  CS* low*/
	out_spi((datum *)ADR_AMBER_SPI0_TXD, 0x3); /*send out read instruction*/
	out_spi((datum *)ADR_AMBER_SPI0_TXD, HiByte); /*send out first 8 bits of address*/
	out_spi((datum *)ADR_AMBER_SPI0_TXD, MidByte); /*send out first 8 bits of address*/
	out_spi((datum *)ADR_AMBER_SPI0_TXD, LoByte); /*send out last 4 bits of address*/
	out_spi((datum *)ADR_AMBER_SPI0_TXD, DUMMY); /*send out last 4 bits of address*/
	mem_data=in_spi((datum *)ADR_AMBER_SPI0_RXD); /*read in memory data*/
	delay(15); //15us
	bit_set((datum *)ADR_AMBER_PIO_RGPIO_OUT, SPI0CS); /*set  CS* hi*/
	return mem_data;
}

/*This routine will set the write enable latch of the 25Cxxx eeprom*/
void ee25xxx_write_enable()
{
	bit_clear(((datum *)ADR_AMBER_PIO_RGPIO_OUT), SPI0CS); /*set  CS* low*/
	out_spi((datum *)ADR_AMBER_SPI0_TXD, 0x06); /*send out write enable instruction*/
	delay (10);
	bit_set(((datum *)ADR_AMBER_PIO_RGPIO_OUT), SPI0CS); /*set  CS* high*/
}

//** looks to not be needed **ee25xxx_erasemem()
void ee25xxx_erasemem()
{
	bit_clear(((datum *)ADR_AMBER_PIO_RGPIO_OUT),SPI0CS); /*set  CS* low*/
	out_spi((datum *)ADR_AMBER_SPI0_TXD, 0x60); /*send out write enable instruction*/
	delay (10);
	bit_set(((datum *)ADR_AMBER_PIO_RGPIO_OUT), SPI0CS); /*set  CS* hi*/
	delay (50000); /*need to wait until complete*/
}

// ee25xxx_getid()
//verified
char ee25xxx_getid(int addr)
{
	char mem_data=0xde;
	int HiByte, MidByte, LoByte;
	HiByte=0x00; /*mask off lower 8 bits*/
	MidByte = 0x00;
	LoByte=(0x00+addr); /*device id = 8E*/
	bit_clear(((datum *)ADR_AMBER_PIO_RGPIO_OUT), SPI0CS); /*set  CS* low*/
//	printf("write cmd\n\r");
	out_spi((datum *)ADR_AMBER_SPI0_TXD, 0xAB); /*send out read instruction*/
//	printf("write msb addr\n\r");
	out_spi((datum *)ADR_AMBER_SPI0_TXD, HiByte); /*send out first 8 bits of address*/
	printf("write mid addr\n\r");
	out_spi((datum *)ADR_AMBER_SPI0_TXD, MidByte); /*send out first 8 bits of address*/
//	printf("write lsb addr\n\r");
	out_spi((datum *)ADR_AMBER_SPI0_TXD, LoByte); /*send out last 4 bits of address*/
//	printf("write DUMMY\n\r");
	out_spi((datum *)ADR_AMBER_SPI0_TXD, DUMMY); /*dummy*/
	delay (20);
	mem_data=in_spi((datum *)ADR_AMBER_SPI0_RXD); /*read in memory data*/
	bit_set((datum *)ADR_AMBER_PIO_RGPIO_OUT, SPI0CS); /*set  CS* hi*/
	return mem_data;
}

// ee25xxx_write()
char ee25xxx_write ( int address, int portaddr, char c)
{
	int HiByte, LoByte, MidByte;
	HiByte=address>>16; /*mask off lower 8 bits*/
	MidByte = (address >> 8) & 0b0000000011111111;
	LoByte=address & 0b000000000000000011111111; /*mask off 16 higher bits*/
	printf ("write H: %x, M: %x, L: %x CH: %x \n\r", HiByte, MidByte, LoByte, c);

//	ee25xxx_write_enable(); /*set eeprom so can write to it*/
	bit_clear( ((datum *)ADR_AMBER_PIO_RGPIO_OUT), 0); /*set PD5 CS* low*/
	out_spi( ((datum *)ADR_AMBER_SPI0_TXD), 0x02); /*send out write instruction*/
	out_spi( ((datum *)ADR_AMBER_SPI0_TXD), HiByte); /*send out first 8 bits of address*/
	out_spi( ((datum *)ADR_AMBER_SPI0_TXD),MidByte); /*send out middle 8 bits of address*/
	out_spi( ((datum *)ADR_AMBER_SPI0_TXD),LoByte); /*send out last 8 bits of address*/
	out_spi( ((datum *)ADR_AMBER_SPI0_TXD), c); /*clock out the memory data*/
	delay(10); //Tbp+10us
	bit_set( ((datum *)ADR_AMBER_PIO_RGPIO_OUT), 0); /*set PD5 CS* hi*/
	delay(20); //Tbp+10us
	return c;
}


//#define PORTBASE 0x40000000
//unsigned int volatile * const port = (unsigned int *) PORTBASE;
//The variable port is a constant pointer to a volatile unsigned integer, so we can access the memory-mapped register using:
//*port = value; /* write to port */
//value = *port; /* read from port */

void main ()
{
    unsigned char c=0xde;
	int i;
	int tempreg, pattern, bitno=0;
//	datum volatile *  addr;


//	addr = (( datum *)ADR_AMBER_PIO_RGPIO_OE);
	pio_init();
	spi_init();
	printf (" PIO and SPI initialized\n\r");

	c = ee25xxx_getid(MFGID);
	printf("\n\r EEprom Get ID %x: %x\n\r",MFGID,c);

	ee25xxx_write_enable();
	printf (" EEprom Write Enabled\n\r");
	ee25xxx_erasemem();
	printf (" EEprom Chip Erase\n\r");
//	ee25xxx_write(0x00000000, 0 , 'J');
//	ee25xxx_write(0x00000001, 0 , 'e');
//	ee25xxx_write(0x00000002, 0 , 'r');
//	ee25xxx_write(0x00000003, 0 , 'e');
//	ee25xxx_write(0x00000004, 0 , 'm');
//	ee25xxx_write(0x00000005, 0 , 'y');
//	ee25xxx_write_enable();
//	c=ee25xxx_read(0x00000000, 0);
	printf("%c",c);
	c=ee25xxx_read(0x00000001, 0);
	printf("%c",c);
	c=ee25xxx_read(0x00000002, 0);
	printf("%c",c);
	c=ee25xxx_read(0x00000003, 0);
	printf("%c",c);
	c=ee25xxx_read(0x00000004, 0);
	printf("%c",c);
	c=ee25xxx_read(0x00000005, 0);
	printf("%c",c);

	asm volatile ("mov pc, #0");
	return;
}
