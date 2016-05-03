/*----------------------------------------------------------------
//                                                              //
//  amber_registers.h                                           //
//                                                              //
//  This file is part of the Amber project                      //
//  http://www.opencores.org/project,amber                      //
//                                                              //
//  Description                                                 //
//  Defines the address of all registers in the Amber system.   //
//  Must be kept synchronized with the equivalent Verilog       //
//  file, $AMBER_BASE/hw/vlog/system/register_addresses.v       //
//  which is considered the master.                             //
//                                                              //
//  Author(s):                                                  //
//      - Conor Santifort, csantifort.amber@gmail.com           //
//                                                              //
//////////////////////////////////////////////////////////////////
//                                                              //
// Copyright (C) 2010 Authors and OPENCORES.ORG                 //
//                                                              //
// This source file may be used and distributed without         //
// restriction provided that this copyright statement is not    //
// removed from the file and that any derivative work contains  //
// the original copyright notice and the associated disclaimer. //
//                                                              //
// This source file is free software; you can redistribute it   //
// and/or modify it under the terms of the GNU Lesser General   //
// Public License as published by the Free Software Foundation; //
// either version 2.1 of the License, or (at your option) any   //
// later version.                                               //
//                                                              //
// This source is distributed in the hope that it will be       //
// useful, but WITHOUT ANY WARRANTY; without even the implied   //
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU Lesser General Public License for more //
// details.                                                     //
//                                                              //
// You should have received a copy of the GNU Lesser General    //
// Public License along with this source; if not, download it   //
// from http://www.opencores.org/lgpl.shtml                     //
//                                                              //
----------------------------------------------------------------*/


#define ADR_AMBER_TEST_STATUS          0xff000000


#define ADR_AMBER_IC_IRQ0_STATUS       0x14000000
#define ADR_AMBER_IC_IRQ0_RAWSTAT      0x14000004
#define ADR_AMBER_IC_IRQ0_ENABLESET    0x14000008
#define ADR_AMBER_IC_IRQ0_ENABLECLR    0x1400000c
#define ADR_AMBER_IC_INT_SOFTSET_0     0x14000010
#define ADR_AMBER_IC_INT_SOFTCLEAR_0   0x14000014
#define ADR_AMBER_IC_FIRQ0_STATUS      0x14000020
#define ADR_AMBER_IC_FIRQ0_RAWSTAT     0x14000024
#define ADR_AMBER_IC_FIRQ0_ENABLESET   0x14000028
#define ADR_AMBER_IC_FIRQ0_ENABLECLR   0x1400002c
#define ADR_AMBER_IC_IRQ1_STATUS       0x14000040
#define ADR_AMBER_IC_IRQ1_RAWSTAT      0x14000044
#define ADR_AMBER_IC_IRQ1_ENABLESET    0x14000048
#define ADR_AMBER_IC_IRQ1_ENABLECLR    0x1400004c
#define ADR_AMBER_IC_INT_SOFTSET_1     0x14000050
#define ADR_AMBER_IC_INT_SOFTCLEAR_1   0x14000054
#define ADR_AMBER_IC_FIRQ1_STATUS      0x14000060
#define ADR_AMBER_IC_FIRQ1_RAWSTAT     0x14000064
#define ADR_AMBER_IC_FIRQ1_ENABLESET   0x14000068
#define ADR_AMBER_IC_FIRQ1_ENABLECLR   0x1400006c
#define ADR_AMBER_IC_INT_SOFTSET_2     0x14000090
#define ADR_AMBER_IC_INT_SOFTCLEAR_2   0x14000094
#define ADR_AMBER_IC_INT_SOFTSET_3     0x140000d0
#define ADR_AMBER_IC_INT_SOFTCLEAR_3   0x140000d4


#define ADR_AMBER_CT_TIMER0_STAT       0xF0000300
#define ADR_AMBER_TM_TIMER0_CTRL       0xF0000301
#define ADR_AMBER_TM_TIMER0_PER1       0xF0000302
#define ADR_AMBER_TM_TIMER0_PER2       0xF0000303
#define ADR_AMBER_TM_TIMER0_SNP1       0xF0000304
#define ADR_AMBER_TM_TIMER0_SNP2       0xF0000305


//https://www.altera.com/en_US/pdfs/literature/ug/ug_embedded_ip.pdf
// Uart0 0xf000.0100 - 0xf00.0107
//TRDY: Transmit ready x2 bit 6
//RRdy: recv ready 0x2 bit 7
#define ADR_AMBER_UART0_DRX            0xF0000100
#define ADR_AMBER_UART0_DTX            0xF0000101
#define ADR_AMBER_UART0_CR             0xF0000102
#define ADR_AMBER_UART0_FR             0xF0000103

#define ADR_JTAG_UART0_DR              0xF0000000
#define ADR_JTAG_UART0_CR              0xF0000001

#define ADR_AMBER_SPI0_RXD           0xF0000200
#define ADR_AMBER_SPI0_TXD           0xF0000201
#define ADR_AMBER_SPI0_STATUS          0xF0000202
#define ADR_AMBER_SPI0_CONTROL         0xF0000203
// Reserved                            0xF0000204
#define ADR_AMBER_SPI0_SS           0xF0000205
