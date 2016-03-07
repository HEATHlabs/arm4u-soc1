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
#define SPIWAIT 20
#define SPI1CS 1

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
//	c= *((datum volatile *)ADR_AMBER_SPI1_RXD) ;
	printf ("Shift Reg:   %x\n\r", *((datum volatile *)ADR_AMBER_SPI1_RXD));
//	c= *((datum volatile *)ADR_AMBER_SPI1_TXD);
	printf ("Buffer Reg:  %x\n\r", *((datum volatile *)ADR_AMBER_SPI1_TXD));
//	c= *((datum volatile *)ADR_AMBER_SPI1_STATUS);
	printf ("Status Reg:  %x\n\r", *((datum volatile *)ADR_AMBER_SPI1_STATUS));
//	c= *((datum volatile *)ADR_AMBER_SPI1_CONTROL);
	printf ("Control Reg: %x \n\r",  *((datum volatile *)ADR_AMBER_SPI1_CONTROL));
	//c= *((datum *)ADR_AMBER_SPI1_BAUD_DIV);
	printf ("Buad control : %x (not readable, always 0)\n\r", (*(datum volatile *) ADR_AMBER_SPI1_BAUD_DIV) );
}




//put single byte out on SPI Bus
//verified
void out_spi( datum *spibuffer, unsigned char c)  //SPI1 only
{
//	printf("out_spi() status reg: %x\n\r",(*(datum volatile *)ADR_AMBER_SPI1_STATUS));
	while ( !((*(datum volatile *)ADR_AMBER_SPI1_STATUS) & TINY_SPI_STATUS_TXE) ) //Wait until ready to send
	{
		delay (SPIWAIT);
//		printf ("waiting...");
	}
//	printf("out_spi() status reg: %x\n\r",(*(datum volatile *)ADR_AMBER_SPI1_STATUS));

	*spibuffer = c;
}

//verified
unsigned char in_spi( datum volatile *spibuffer)  //SPI1 only
{
	unsigned char c;
	while ( !((*(datum volatile *)ADR_AMBER_SPI1_STATUS) & TINY_SPI_STATUS_TXR) ) //Wait until ready to send
	{
		delay (SPIWAIT);
//		printf ("waiting...");
	}
//	printf("in_spi() status reg: %x\n\r",(*(datum volatile *)ADR_AMBER_SPI1_STATUS));

	c = *spibuffer ;
	return c ;
}

//verified
void pio_init()   //SPI1 only
{
		//read OutputEnable reg
		bit_set ( ((datum *)ADR_AMBER_PIO_RGPIO_OE), 0);
		//Write 1 to bit 0
		bit_set ( ((datum *)ADR_AMBER_PIO_RGPIO_OUT), 0);
}


// SPI init, set baud divider
//verified
void spi_init()   //SPI1 only
{
	*(volatile datum *) ADR_AMBER_SPI1_BAUD_DIV = 0x28; // 40Mhz/40 = 1Mhz
}

// enc25j_read()
char enc25j_read ( int address, int portaddr)   //SPI1 only
{
	char mem_data;
	int LoByte;
	LoByte=address & 0b000000000000000011111111; /*mask off 16 higher bits*/
	printf ("write L: %x \n\r", LoByte);
	bit_clear(((datum *)ADR_AMBER_PIO_RGPIO_OUT), SPI1CS); /*set  CS* low*/
	out_spi((datum *)ADR_AMBER_SPI1_TXD, LoByte); /*send out cmd+addr of address*/
	out_spi((datum *)ADR_AMBER_SPI1_TXD, DUMMY); /*send out lastdummpy byte*/
	mem_data=in_spi((datum *)ADR_AMBER_SPI1_RXD); /*read in memory data*/
	delay(1); //15us
	bit_set((datum *)ADR_AMBER_PIO_RGPIO_OUT, SPI1CS); /*set  CS* hi*/
	return mem_data;
}

char enc25j_reset ()   //SPI1 only
{
	bit_clear(((datum *)ADR_AMBER_PIO_RGPIO_OUT), SPI1CS); /*set  CS* low*/
	out_spi((datum *)ADR_AMBER_SPI1_TXD, 0xff); /*send out cmd+addr of address*/
	delay(1); //5us
	bit_set((datum *)ADR_AMBER_PIO_RGPIO_OUT, SPI1CS); /*set  CS* hi*/

}

// enc25j_getid()
//verified
char enc25j_getid(int addr)
{
	char mem_data=0xde;
	int HiByte, MidByte, LoByte;
	HiByte=0x00; /*mask off lower 8 bits*/
	MidByte = 0x00;
	LoByte=(0x00+addr); /*device id = 8E*/
	bit_clear(((datum *)ADR_AMBER_PIO_RGPIO_OUT), SPI1CS); /*set  CS* low*/
//	printf("write cmd\n\r");
	out_spi((datum *)ADR_AMBER_SPI1_TXD, 0xAB); /*send out read instruction*/
//	printf("write msb addr\n\r");
	out_spi((datum *)ADR_AMBER_SPI1_TXD, HiByte); /*send out first 8 bits of address*/
	printf("write mid addr\n\r");
	out_spi((datum *)ADR_AMBER_SPI1_TXD, MidByte); /*send out first 8 bits of address*/
//	printf("write lsb addr\n\r");
	out_spi((datum *)ADR_AMBER_SPI1_TXD, LoByte); /*send out last 4 bits of address*/
//	printf("write DUMMY\n\r");
	out_spi((datum *)ADR_AMBER_SPI1_TXD, DUMMY); /*dummy*/
	delay (20);
	mem_data=in_spi((datum *)ADR_AMBER_SPI1_RXD); /*read in memory data*/
	bit_set((datum *)ADR_AMBER_PIO_RGPIO_OUT, SPI1CS); /*set  CS* hi*/
	return mem_data;
}

// enc25j_write()
char enc25j_write ( int address, int portaddr, char c)
{
	int LoByte;
	LoByte=address & 0b000000000000000011111111; /*mask off 16 higher bits*/
	printf ("write L: %x CH: %x \n\r", LoByte, c);

	bit_clear( ((datum *)ADR_AMBER_PIO_RGPIO_OUT), SPI1CS); /*set PD5 CS* low*/
	out_spi( ((datum *)ADR_AMBER_SPI1_TXD),LoByte); /*send out last 8 bits of address*/
	out_spi( ((datum *)ADR_AMBER_SPI1_TXD), c); /*clock out the memory data*/
	delay(10); //Tbp+10us
	bit_set( ((datum *)ADR_AMBER_PIO_RGPIO_OUT), SPI1CS); /*set PD5 CS* hi*/
	delay(1); //Tbp+10us
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
	printf ("PIO and SPI initialized\n\r");
	enc25j_reset ();
	c=enc25j_read (0x1f,1);
	printf("First read %x\n\r",c);
	enc25j_write(0x5f,1,0x00);
	printf("Write \n\r");
	c=enc25j_read (0x1f,1);
	printf("Second read %x\n\r",c);

	enc25j_write(0x5f,1,0xff);
	printf("Write \n\r");
	c=enc25j_read (0x1f,1);
	printf("Second read %x\n\r",c);

	enc25j_write(0x5f,1,0xaa);
	printf("Write \n\r");
	c=enc25j_read (0x1f,00);
	printf("Second read %x\n\r",c);

	asm volatile ("mov pc, #0");
	return;
}
