/*----------------------------------------------------------------
//                                                              //
//  hello-world.c                                               //
//                                                              //
//  This file is part of the Amber project                      //
//  http://www.opencores.org/project,amber                      //
//                                                              //
//  Description                                                 //
//  Simple stand-alone example application.                     //
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

/* Note that the stdio.h referred to here is the one in
   mini-libc. This applications compiles in mini-libc
   so it can run stand-alone.
*/   
#include "stdio.h"

main () 
{
    int i;
    printf ("\nMarsohod2: Hello, World!\n");
//hex -- 4d 61 72 73 6f 68 6f 64 32 3a 20 48 65 6c 6c 6f 2c 20 57 
//hex 6f 72 6c 64 21 --

    for(i=0; i<5; i++)
	printf("* %d *\n",i);

    /* Flush out UART FIFO */
    printf ("                ");
    _testpass();
}


