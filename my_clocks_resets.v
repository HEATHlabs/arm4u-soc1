
//
// Clocks and Resets Module
//

module my_clocks_resets (
	input	wire	i_brd_rst, //from board button
	input	wire	i_brd_clk, //from board crystal
	input wire  i_memory_initialized,
//	output wire	o_sys_rst,
	output reg	o_sys_rst,
	output wire	o_sys_clk,
	output wire	o_mem_clk,
	output reg o_sdr_rst,
//	output wire o_sdr_rst,
	output reg	o_system_ready
//	output wire	o_system_ready
);
//wire pll_clk40;
//wire pll_clk80;
wire xtra_clk;
wire pll_locked;
//initial begin 

initial  begin
	   o_sdr_rst = 1;
	   o_sys_rst = 1;
	   o_system_ready = 0;
	   //pll_locked =0;
 end
//end
`ifdef ICARUS
 //for simulation purposes just use incoming freq as memory clock
 //and system clock is half of incoming freq
 reg t=0;
 always @(posedge i_brd_clk)
	t <= ~t;
 assign o_sys_clk = t;
 assign o_mem_clk = i_brd_clk;
 assign pll_locked = 1'b1;
`else
 //for real system insert altera's PLL
 my_pll my_pll_inst (
	.inclk0(i_brd_clk),
	.areset(i_brd_rst ),
	.c0(o_sys_clk), //40Mhz
	.c1(o_mem_clk), //80Mhz
	.c2(xtra_clk), //extra 10 mhz clock
	.locked(pll_locked)
	);
`endif

//enable SRDAM only after timeout, when counter reaches 0x8000
`ifdef ICARUS
	reg [17:0]cnt ;
`else
	reg [17:0]cnt ;
`endif

initial cnt = 'h00;


always @(posedge o_sys_clk or posedge i_brd_rst)

	if(i_brd_rst)
		cnt <= 0;
	else begin
`ifdef ICARUS
	if( (cnt[17]) && (o_sdr_rst) ) begin
`else
	if( (cnt[17]) && (o_sdr_rst) ) begin
`endif		
		o_sdr_rst <= 0;
		end
		else if ( (cnt[14]) && (!o_sdr_rst)) begin
			o_sys_rst <= 0 ;
		end
		else  begin
			cnt <= cnt + 1'b1;
		end
	end	
	
always @(posedge i_brd_clk)
 begin   

	o_system_ready <= (~o_sys_rst) & i_memory_initialized;
end

endmodule


