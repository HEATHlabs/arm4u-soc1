


















// ee25xxx_write()
char ee25xxx_write ( int address, int portaddr, char c)
{
	int HiByte, LoByte;
	HiByte=address>>8; /*mask off lower 8 bits*/
	LoByte=address & 0b0000000011111111; /*mask off 8 higher bits*/
	eeprom_write_enable(); /*set eeprom so can write to it*/
	bit_clear(((datum *)ADR_AMBER_PIO_RGPIO_OUT), 0); /*set PD5 CS* low*/
	out_spi(0x2, ((datum *)ADR_AMBER_SPI0_BUFFER)); /*send out write instruction*/
	out_spi(HiByte, ((datum *)ADR_AMBER_SPI0_BUFFER)); /*send out first 8 bits of address*/
	out_spi(LoByte, ((datum *)ADR_AMBER_SPI0_BUFFER)); /*send out last 4 bits of address*/
	out_spi(c, ((datum *)ADR_AMBER_SPI0_BUFFER)); /*clock out the memory data*/
	bit_set(((datum *)ADR_AMBER_PIO_RGPIO_OUT), 0); /*set PD5 CS* hi*/
	return c;
}





/


	












//	PortLoad ((volatile unsigned int *) PORTBASE)
//	addr = (datum *) ADR_AMBER_PIO_RGPIO_OE;
//	 *addr = 0xff;
//	addr = (( volatile unsigned long int *)ADR_AMBER_PIO_RGPIO_OE);


//	printf("addr: %x \n\r", addr);


	tempreg = *((datum *) ADR_AMBER_PIO_RGPIO_OE);
//	pattern = ~(0x1 << bitno);
	print ("pattern: %x \n\r", pattern);
//	*addr = (tempreg & pattern);
//	*addr = (datum) 0x0;


//	bit_clear((*((datum *) ADR_AMBER_PIO_RGPIO_OE)), 0);
	printf("* datum *ADR_AMBER_PIO_RGPIO_OE: %x \n\r",*((datum *) ADR_AMBER_PIO_RGPIO_OE));


	//printf("ADR_AMBER_PIO_RGPIO_OE: %x \n\r", ADR_AMBER_PIO_RGPIO_OE);
	//printf("ADR_AMBER_PIO_RGPIO_OE: %x \n\r",*((datum *) ADR_AMBER_PIO_RGPIO_OE));
//	dump_pio_regs();
//	dump_spi_regs();
  //printf ("Shift Reg:   %x\n\r", c);

// PIO init, enable output and set high
//	printf ("Start initialize PIO and SPI \n\r");
//	pio_init();
//	spi_init();
//	printf ("End initialize PIO and SPI \n\r");
//	dump_pio_regs();
//	dump_spi_regs();

//	printf ("EEPROM getid \n\r");
//	c = ee25xxx_getid();
//	printf("ID:  %x \n\r", c);
//	//printf ("more...\n\r");
//	dump_spi_regs();
