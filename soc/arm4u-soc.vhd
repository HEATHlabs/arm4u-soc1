library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library work;
--use work.STORM_core_package.all;

entity ARM4U_SOC is
	port (
			-- Global Control --
			CLK_I         : in    STD_LOGIC;
			RST_I         : in    STD_LOGIC;

			-- General purpose (console) UART --
			UART0_RXD_I   : in    STD_LOGIC; -- N16 / GPIO_123 / J2.28
			UART0_TXD_O   : out   STD_LOGIC; -- P16 / GPIO_121 / J2.26
														-- GND: J2.30
			-- General purpose (console) UART1 --
			UART1_RXD_I   : in    STD_LOGIC;  -- R11 / GPIO_19  / J2.14
			UART1_TXD_O   : out   STD_LOGIC;  -- R10 / GPIO_111 / J2.16
			                                  -- GND: J2.12

			-- SDRAM ---
			SDRAM_CLK_O	  : out   STD_LOGIC;
			SDRAM_CKE_O	  : out	  STD_LOGIC;
			SDRAM_RAS_O	  : out	  STD_LOGIC;
			SDRAM_CAS_O	  : out	  STD_LOGIC;
			SDRAM_WE_O	  : out	  STD_LOGIC;
			SDRAM_CS_O	  : out	  STD_LOGIC;
			SDRAM_DQM_O	  : out   STD_LOGIC_VECTOR(01 downto 0);
			SDRAM_BA_O	  : out   STD_LOGIC_VECTOR(01 downto 0);
			SDRAM_ADDR_O  : out   STD_LOGIC_VECTOR(12 downto 0);
			SDRAM_DQ_IO	  : inout   STD_LOGIC_VECTOR(15 downto 0);
			
			-- PWM Port 0 --
			PWM0_PORT_O   : out   STD_LOGIC_VECTOR(07 downto 0)
	     );
end ARM4U_SOC;

architecture Structure of ARM4U_SOC is

	-- Address Map --------------------------------------------------------------------
	-- -----------------------------------------------------------------------------------
		constant INT_MEM_BASE_C    : STD_LOGIC_VECTOR(31 downto 0) := x"00000000";
		constant INT_MEM_SIZE_C    : natural := 32*1024; -- byte       x00008000
		constant BOOT_ROM_BASE_C   : STD_LOGIC_VECTOR(31 downto 0) := x"00010000";
		constant BOOT_ROM_SIZE_C   : natural := 16*1024; -- byte       x00014000
		constant SDRAM_MEM_BASE_C   : STD_LOGIC_VECTOR(31 downto 0):= x"01000000";
		constant SDRAM_MEM_SIZE_C   : natural := 32*1024*1024; -- byte x02ffffff
--		constant BOOT_ROM_BASE_C   : STD_LOGIC_VECTOR(31 downto 0) := x"FFF00000";
		-- Begin of IO area ------------------------------------------------------
		constant IO_AREA_BEGIN     : STD_LOGIC_VECTOR(31 downto 0) := x"FFFF0000";
		constant GP_IO0_BASE_C     : STD_LOGIC_VECTOR(31 downto 0) := x"FFFF0000";
		constant GP_IO0_SIZE_C     : natural := 2*4; -- byte
		constant UART0_BASE_C      : STD_LOGIC_VECTOR(31 downto 0) := x"FFFF0100";
		constant UART0_SIZE_C      : natural := 2*4; -- byte
		constant SYS_TIMER0_BASE_C : STD_LOGIC_VECTOR(31 downto 0) := x"FFFF0200";
		constant SYS_TIMER0_SIZE_C : natural := 4*4; -- byte
		constant SPI0_CTRL_BASE_C  : STD_LOGIC_VECTOR(31 downto 0) := x"FFFF0300";
		constant SPI0_CTRL_SIZE_C  : natural := 8*4; -- byte
		constant I2C0_CTRL_BASE_C  : STD_LOGIC_VECTOR(31 downto 0) := x"FFFF0400";
		constant I2C0_CTRL_SIZE_C  : natural := 8*4; -- byte
		constant PWM_CTRL0_BASE_C  : STD_LOGIC_VECTOR(31 downto 0) := x"FFFF0500";
		constant PWM_CTRL0_SIZE_C  : natural := 2*4; -- byte
		constant UART1_BASE_C      : STD_LOGIC_VECTOR(31 downto 0) := x"FFFF0600";
		constant UART1_SIZE_C      : natural := 2*4; -- byte
		constant VIC_BASE_C        : STD_LOGIC_VECTOR(31 downto 0) := x"FFFFF000";
		constant VIC_SIZE_C        : natural := 64*4; -- byte
		constant IO_AREA_END       : STD_LOGIC_VECTOR(31 downto 0) := x"FFFFFFFF";
		-- End of IO area --------------------------------------------------------


	-- Architecture Constants ---------------------------------------------------------
	-- -----------------------------------------------------------------------------------
		constant BOOT_VECTOR_C       : STD_LOGIC_VECTOR(31 downto 0) := BOOT_ROM_BASE_C;
		constant BOOT_IMAGE_C        : string  := "STORM_SOC_BASIC_BL_32_8";
		constant I_CACHE_PAGES_C     : natural := 8;
		constant I_CACHE_PAGE_SIZE_C : natural := 32;
		constant D_CACHE_PAGES_C     : natural := 8;
		constant D_CACHE_PAGE_SIZE_C : natural := 32;
		constant CORE_CLOCK_C        : natural := 50000000; -- Hz
		constant RST_RTIGGER_C       : natural := CORE_CLOCK_C/2;
		constant LOW_ACTIVE_RST_C    : boolean := TRUE;
--		constant SDRAM_BURST_LEN     : natural := 1;
--		constant UART0_BAUD_C        : natural := 9600;
		constant UART0_BAUD_C        : natural := 38400;
		constant UART0_BAUD_VAL_C    : natural := CORE_CLOCK_C/(4*UART0_BAUD_C);
		constant USE_OUTPUT_GATES_C  : boolean := FALSE;
		constant UNCACHABLE_BEGIN	  : STD_LOGIC_VECTOR(31 downto 0)  := SDRAM_MEM_BASE_C;
--		constant UNCACHABLE_BEGIN	  : STD_LOGIC_VECTOR(31 downto 0)  := IO_AREA_BEGIN;
		constant UNCACHABLE_END			: STD_LOGIC_VECTOR(31 downto 0)  := IO_AREA_END;
		constant A2WB_DATA_W			: natural :=32;
		constant A2WB_ADR_W				: natural :=32;
		


	-- Global signals -----------------------------------------------------------------
	-- -----------------------------------------------------------------------------------

		-- Global Clock, Reset, Interrupt, Control --
		signal MAIN_RST           : STD_LOGIC;
		signal MAIN_RST_N         : STD_LOGIC;
		signal XMEM_CLK           : STD_LOGIC;
		signal XMEMD_CLK          : STD_LOGIC;
		signal CPU_RST           : STD_LOGIC;
		signal CLK_LOCK           : STD_LOGIC;
		signal CLK_DIV            : STD_LOGIC_VECTOR(01 downto 0) := "00"; -- just for sim
		signal MAIN_CLK           : STD_LOGIC;
		signal SAVE_RST           : STD_LOGIC;
		signal STORM_IRQ          : STD_LOGIC;
		signal STORM_FIQ          : STD_LOGIC;
		signal SYS_CTRL_O         : STD_LOGIC_VECTOR(15 downto 0);
		signal SYS_CTRL_I         : STD_LOGIC_VECTOR(15 downto 0);

		-- Wishbone Core Bus --
		signal CORE_WB_ADR_O      : STD_LOGIC_VECTOR(31 downto 0); -- address
		signal CORE_WB_CTI_O      : STD_LOGIC_VECTOR(02 downto 0); -- cycle type
		signal CORE_WB_BTE_O      : STD_LOGIC_VECTOR(01 downto 0); -- burst trans type
		signal CORE_WB_TGC_O      : STD_LOGIC_VECTOR(06 downto 0); -- cycle tag
		signal CORE_WB_SEL_O      : STD_LOGIC_VECTOR(03 downto 0); -- byte select
		signal CORE_WB_WE_O       : STD_LOGIC;                     -- write enable
		signal CORE_WB_DATA_O     : STD_LOGIC_VECTOR(31 downto 0); -- data out
		signal CORE_WB_DATA_I     : STD_LOGIC_VECTOR(31 downto 0); -- data in
		signal CORE_WB_STB_O      : STD_LOGIC;                     -- valid transfer
		signal CORE_WB_CYC_O      : STD_LOGIC;                     -- valid cycle
		signal CORE_WB_ACK_I      : STD_LOGIC;                     -- acknowledge
		signal CORE_WB_HALT_I     : STD_LOGIC;                     -- halt request
		signal CORE_WB_ERR_I      : STD_LOGIC;                     -- abnormal termination

		-- Avalon to Wishbone Bridge
		--	 Arm4u master output to Avalon Slave input
		signal CORE_AV_ADDRESS_O	: STD_LOGIC_VECTOR(AW-1 downto 0); 		-- s_av_address_i,
		signal CORE_AV_BE_O 		: STD_LOGIC_VECTOR(DW/8-1 downto 0); 	-- s_av_byteenable_i,
		signal CORE_AV_RD_O		 	: STD_LOGIC;	  						--s_av_read_i,
		signal CORE_AV_RD_DAT_I 	: STD_LOGIC_VECTOR(DW-1 downto 0); 		-- s_av_readdata_o,
		signal CORE_AV_BUST_CNT_O 	: STD_LOGIC_VECTOR(7 downto 0); 		-- s_av_burstcount_i,
		signal CORE_AV_WR_O 		: STD_LOGIC; 							-- s_av_write_i,
		signal CORE_AV_WR_DAT_O 	: STD_LOGIC_VECTOR(DW-1 downto 0); 		-- s_av_writedata_i,
		signal CORE_AV_WAIT_REQ_I 	: STD_LOGIC;	  						-- s_av_waitrequest_o,
		signal CORE_AV_RD_DAT_VALID : STD_LOGIC;		  					-- s_av_readdatavalid_o,
		
	-- // Wishbone Master Output
	-- output [AW-1:0]   wbm_adr_o,
	-- output [DW-1:0]   wbm_dat_o,
	-- output [DW/8-1:0] wbm_sel_o,
	-- output 		  wbm_we_o,
	-- output 		  wbm_cyc_o,
	-- output 		  wbm_stb_o,
	-- output [2:0] 	  wbm_cti_o,
	-- output [1:0] 	  wbm_bte_o,
	-- input [DW-1:0] 	  wbm_dat_i,
	-- input 		  wbm_ack_i,
	-- input 		  wbm_err_i,
	-- input 		  wbm_rty_i
	-- Component interface ------------------------------------------------------------
	-- -----------------------------------------------------------------------------------

		-- Internal SRAM Memory --
		signal INT_MEM_DATA_O     : STD_LOGIC_VECTOR(31 downto 0);
		signal INT_MEM_STB_I      : STD_LOGIC;
		signal INT_MEM_ACK_O      : STD_LOGIC;
		signal INT_MEM_HALT_O     : STD_LOGIC;
		signal INT_MEM_ERR_O      : STD_LOGIC;

		-- external SDRAM Memory --
		signal SDRAM_MEM_DATA_O     : STD_LOGIC_VECTOR(31 downto 0);
		signal SDRAM_MEM_STB_I      : STD_LOGIC;
		signal SDRAM_MEM_ACK_O      : STD_LOGIC;
		signal SDRAM_MEM_HALT_O     : STD_LOGIC;
		signal SDRAM_MEM_ERR_O      : STD_LOGIC;
		signal SDRAM_MEM_CTI_I      : STD_LOGIC;
		-- DQ ---
		signal internal_dqo   : STD_LOGIC_VECTOR(15 downto 0);
		signal internal_dqi   : STD_LOGIC_VECTOR(15 downto 0);
		signal internal_dqoe   : STD_LOGIC;

		-- UART 0 - miniUART --
		signal UART0_DATA_O       : STD_LOGIC_VECTOR(31 downto 0);
		signal UART0_STB_I        : STD_LOGIC;
		signal UART0_ACK_O        : STD_LOGIC;
		signal UART0_ERR_O        : STD_LOGIC;
		signal UART0_TX_IRQ       : STD_LOGIC;
		signal UART0_RX_IRQ       : STD_LOGIC;
		signal UART0_HALT_O       : STD_LOGIC;

		-- UART 1 - Amber UART --
		signal UART1_DATA_O       : STD_LOGIC_VECTOR(31 downto 0);
		signal UART1_STB_I        : STD_LOGIC;
		signal UART1_ACK_O        : STD_LOGIC;
		signal UART1_ERR_O        : STD_LOGIC;
		signal UART1_TX_IRQ       : STD_LOGIC;
		signal UART1_RX_IRQ       : STD_LOGIC;
		signal UART1_HALT_O       : STD_LOGIC;		
		-- Boot ROM --
		signal BOOT_ROM_DATA_O    : STD_LOGIC_VECTOR(31 downto 0);
		signal BOOT_ROM_STB_I     : STD_LOGIC;
		signal BOOT_ROM_ACK_O     : STD_LOGIC;
		signal BOOT_ROM_HALT_O    : STD_LOGIC;
		signal BOOT_ROM_ERR_O     : STD_LOGIC;


		-- System Timer 0 --
		signal SYS_TIMER0_DATA_O  : STD_LOGIC_VECTOR(31 downto 0);
		signal SYS_TIMER0_STB_I   : STD_LOGIC;
		signal SYS_TIMER0_ACK_O   : STD_LOGIC;
		signal SYS_TIMER0_IRQ     : STD_LOGIC;
		signal SYS_TIMER0_HALT_O  : STD_LOGIC;
		signal SYS_TIMER0_ERR_O   : STD_LOGIC;

		-- Vector Interrupt Controller --
		signal VIC_DATA_O         : STD_LOGIC_VECTOR(31 downto 0);
		signal VIC_STB_I          : STD_LOGIC;
		signal VIC_ACK_O          : STD_LOGIC;
		signal VIC_HALT_O         : STD_LOGIC;
		signal VIC_ERR_O          : STD_LOGIC;
		signal INT_LINES          : STD_LOGIC_VECTOR(31 downto 0);
		signal INT_LINES_ACK      : STD_LOGIC_VECTOR(31 downto 0);


	-- Logarithm duales ---------------------------------------------------------------
	-- -----------------------------------------------------------------------------------
		function log2(temp : natural) return natural is
			variable result : natural;
		begin
			for i in 0 to integer'high loop
				if (2**i >= temp) then
					return i;
				end if;
			end loop;
			return 0;
		end function log2;


	-- ARM4U SYSTEM TOP ENTITY --------------------------------------------------------
	-- -----------------------------------------------------------------------------------
		component arm4u_cpu
			generic(
				CACHE_BLOCK_BITWIDTH : natural := 5   -- byte address range of a block (hence C_BLOCK_SIZE = 2**BLOCK_BITWIDTH)
			);
			port(
				-- Globals
				clk   : in std_logic;
				reset : in std_logic;
				
				--Avalon Master Interface for instructions
				avm_inst_waitrequest   : in  std_logic;
				avm_inst_readdatavalid : in  std_logic;
				avm_inst_readdata      : in  std_logic_vector(31 downto 0);
				avm_inst_read          : out std_logic;
				avm_inst_burstcount    : out std_logic_vector(CACHE_BLOCK_BITWIDTH-2 downto 0);
				avm_inst_address       : out std_logic_vector(31 downto 0);
				
				--Avalon Master Interface for data
				avm_data_waitrequest   : in  std_logic;
				avm_data_readdatavalid : in  std_logic;
				avm_data_readdata      : in  std_logic_vector(31 downto 0);
				avm_data_read          : out std_logic;
				avm_data_writedata     : out std_logic_vector(31 downto 0);
				avm_data_write         : out std_logic;
				avm_data_byteen        : out std_logic_vector(3 downto 0);
				avm_data_burstcount    : out std_logic_vector(4 downto 0);
				avm_data_address       : out std_logic_vector(31 downto 0);
				
				--Interrupt interface
				inr_irq                : in  std_logic_vector(31 downto 0) := (others => '0')
			);
		end component;

	-- Altera Megawizzard PLL ---------------------------------------------------------
	-- -----------------------------------------------------------------------------------
		component SYSTEM_PLL
			port	(
						inclk0        : in  STD_LOGIC; -- external clock input
						c0	          : out STD_LOGIC; -- system clock
						c1	          : out STD_LOGIC; -- external mem clock for internal use
						c2	          : out STD_LOGIC; -- external mem clock, -3ns phase shifted
						locked        : out STD_LOGIC  -- clock stable
					);
		end component;

	-- Reset Protector ----------------------------------------------------------------
	-- -----------------------------------------------------------------------------------
		component RST_PROTECT
			generic	(
						TRIGGER_VAL   : natural := 50000000; -- trigger in sys clocks
						LOW_ACT_RST   : boolean := TRUE      -- valid reset level
					);
			port	(
						-- Interface --
						MAIN_CLK_I    : in  STD_LOGIC; -- system master clock
						EXT_RST_I     : in  STD_LOGIC; -- external reset input
						SYS_RST_O     : out STD_LOGIC;  -- system master reset
						CPU_RST_O     : out STD_LOGIC  -- system master reset
					);
		end component;

	-- Avalon to Wishbone Bridger ----------------------------------------------------------------
	-- -----------------------------------------------------------------------------------
		component avalon_to_wb_bridge
			generic	(
						DW   : natural := 32;	  -- Data width
						AW   : natural := 32      -- Address width
					);
			port	(
						wb_clk_i			: in  STD_LOGIC;
						wb_rst_i			: in  STD_LOGIC;
						
						-- Avalon Slave input
						s_av_address_i		: in  std_logic_vector(AW-1 downto 0);
						s_av_byteenable_i	: in  std_logic_vector(DW/8-1 downto 0);
						s_av_read_i			: in  STD_LOGIC;
						s_av_readdata_o		: out  std_logic_vector(W-1 downto 0);
						s_av_burstcount_i	: in  std_logic_vector(7 downto 0);
						s_av_write_i		: in  STD_LOGIC;
						s_av_writedata_i	: in  std_logic_vector(DW-1 downto 0);
						s_av_waitrequest_o	: out  STD_LOGIC;
						s_av_readdatavalid_o: out  STD_LOGIC;
						
						-- Wishbone Master Output
						wbm_adr_o			: out  std_logic_vector(AW-1 downto 0);
						bm_dat_o			: out  std_logic_vector(DW-1 downto 0);
						wbm_sel_o			: out  std_logic_vector(DW/8-1 downto 0);
						wbm_we_o			: out  STD_LOGIC;
						wbm_cyc_o			: out  STD_LOGIC;
						wbm_stb_o			: out  STD_LOGIC;
						wbm_cti_o			: out  std_logic_vector(2 downto 0);
						wbm_bte_o			: out  std_logic_vector(1 downto 0);
						wbm_dat_i			: in  std_logic_vector(DW-1 downto 0);
						wbm_ack_i			: in  STD_LOGIC;
						wbm_err_i			: in  STD_LOGIC;
						wbm_rty_i			: in  STD_LOGIC
					);
		end component;
		
	-- Internal Working Memory --------------------------------------------------------
	-- -----------------------------------------------------------------------------------
		component MEMORY
			generic	(
						MEM_SIZE      : natural := 256;  -- memory cells
						LOG2_MEM_SIZE : natural := 8;    -- log2(memory cells)
						OUTPUT_GATE   : boolean := FALSE -- output and-gate, might be necessary for some bus systems
					);
			port	(
						-- Wishbone Bus --
						WB_CLK_I      : in  STD_LOGIC; -- memory master clock
						WB_RST_I      : in  STD_LOGIC; -- high active sync reset
						WB_CTI_I      : in  STD_LOGIC_VECTOR(02 downto 0); -- cycle indentifier
						WB_TGC_I      : in  STD_LOGIC_VECTOR(06 downto 0); -- cycle tag
						WB_ADR_I      : in  STD_LOGIC_VECTOR(LOG2_MEM_SIZE-1 downto 0); -- adr in
						WB_DATA_I     : in  STD_LOGIC_VECTOR(31 downto 0); -- write data
						WB_DATA_O     : out STD_LOGIC_VECTOR(31 downto 0); -- read data
						WB_SEL_I      : in  STD_LOGIC_VECTOR(03 downto 0); -- data quantity
						WB_WE_I       : in  STD_LOGIC; -- write enable
						WB_STB_I      : in  STD_LOGIC; -- valid cycle
						WB_ACK_O      : out STD_LOGIC; -- acknowledge
						WB_HALT_O     : out STD_LOGIC; -- throttle master
						WB_ERR_O      : out STD_LOGIC  -- abnormal cycle termination
					);
		end component;
	--------------------------------------------------------------------------------------
	----SDRAM Controller
	--------------------------------------------------------------------------------------
		component wb_sdram_ctrl
			port (
                  
                -- WB bus
                    wb_clk			: in  STD_LOGIC;
                    wb_rst			: in  STD_LOGIC;
                    wb_adr_i			: in  STD_LOGIC_VECTOR(31 downto 0);
                    wb_dat_i			: in  STD_LOGIC_VECTOR(31 downto 0);
                    wb_dat_o			: out STD_LOGIC_VECTOR(31 downto 0);
                    wb_sel_i			: in  STD_LOGIC_VECTOR(3 downto 0);
                    wb_cyc_i			: in  STD_LOGIC;
                    wb_stb_i			: in  STD_LOGIC;
                    wb_we_i			: in  STD_LOGIC;
                    wb_ack_o			: out STD_LOGIC;
		    wb_cti_i			: in  STD_LOGIC_VECTOR(02 downto 0); -- cycle indentifier
		    wb_bte_i			: in  STD_LOGIC_VECTOR(01 downto 0); -- burst trans type

		
				-- Interface to SDRAMs 
                    sdram_clk			: in  STD_LOGIC;
                    sdram_rst			: in  STD_LOGIC;
                    cs_n_pad_o		: out STD_LOGIC;
						  cke_pad_o			: out STD_LOGIC;
                    we_pad_o			: out STD_LOGIC;
                    cas_pad_o			: out STD_LOGIC;
                    ras_pad_o			: out STD_LOGIC;
                    dqm_pad_o 		: out STD_LOGIC_VECTOR(1 downto 0);
                    a_pad_o 			: out STD_LOGIC_VECTOR(12 downto 0);
                    ba_pad_o			: out STD_LOGIC_VECTOR(1 downto 0);
						  dq_o				: out STD_LOGIC_VECTOR(15 downto 0);
						  dq_i				: in STD_LOGIC_VECTOR(15 downto 0);
						  dq_oe				: out STD_LOGIC
                    

				);
		end component;
	-- Simple general purpose UART ----------------------------------------------------
	-- -----------------------------------------------------------------------------------
		component MINI_UART
			generic	(
						BRDIVISOR : integer range 0 to 65535
					);
			port	(
						-- Wishbone Bus --
						WB_CLK_I      : in  STD_LOGIC; -- memory master clock
						WB_RST_I      : in  STD_LOGIC; -- high active sync reset
						WB_CTI_I      : in  STD_LOGIC_VECTOR(02 downto 0); -- cycle indentifier
						WB_TGC_I      : in  STD_LOGIC_VECTOR(06 downto 0); -- cycle tag
						WB_ADR_I      : in  STD_LOGIC;                     -- adr in
						WB_DATA_I     : in  STD_LOGIC_VECTOR(31 downto 0); -- write data
						WB_DATA_O     : out STD_LOGIC_VECTOR(31 downto 0); -- read data
						WB_SEL_I      : in  STD_LOGIC_VECTOR(03 downto 0); -- data quantity
						WB_WE_I       : in  STD_LOGIC; -- write enable
						WB_STB_I      : in  STD_LOGIC; -- valid cycle
						WB_ACK_O      : out STD_LOGIC; -- acknowledge
						WB_HALT_O     : out STD_LOGIC; -- throttle master
						WB_ERR_O      : out STD_LOGIC; -- abnormal termination

						-- Terminal signals --
						IntTx_O       : out STD_LOGIC; -- Transmit interrupt: indicate waiting for Byte
						IntRx_O       : out STD_LOGIC; -- Receive interrupt: indicate Byte received
						BR_Clk_I      : in  STD_LOGIC; -- Clock used for Transmit/Receive
						TxD_PAD_O     : out STD_LOGIC; -- Tx RS232 Line
						RxD_PAD_I     : in  STD_LOGIC  -- Rx RS232 Line
					);
		end component;
	-- Amber general purpose UART1 ----------------------------------------------------
	-- -----------------------------------------------------------------------------------
	component amber_uart
			port	(
						-- Wishbone Bus --
						i_clk      : in  STD_LOGIC; -- memory master clock
--						wb_rst_i      : in  STD_LOGIC; -- high active sync reset
--						WB_CTI_I      : in  STD_LOGIC_VECTOR(02 downto 0); -- cycle indentifier
--						WB_TGC_I      : in  STD_LOGIC_VECTOR(06 downto 0); -- cycle tag
						i_wb_adr      : in  STD_LOGIC_VECTOR(31 downto 0);  -- adr in
						i_wb_dat     : in  STD_LOGIC_VECTOR(31 downto 0); -- write data
						o_wb_dat     : out STD_LOGIC_VECTOR(31 downto 0); -- read data
						i_wb_sel      : in  STD_LOGIC_VECTOR(03 downto 0); -- data quantity
						i_wb_we       : in  STD_LOGIC; -- write enable
						i_wb_stb      : in  STD_LOGIC; -- valid cycle
						o_wb_ack      : out STD_LOGIC; -- acknowledge
--						WB_HALT_O     : out STD_LOGIC; -- throttle master
						o_wb_err      : out STD_LOGIC; -- abnormal termination
						i_wb_cyc	  : in  STD_LOGIC;
						-- Terminal signals --
						o_uart_int    : out STD_LOGIC; -- Transmit interrupt: indicate waiting for Byte
--						IntRx_O       : out STD_LOGIC; -- Receive interrupt: indicate Byte received
--						BR_Clk_I      : in  STD_LOGIC; -- Clock used for Transmit/Receive
						o_uart_txd     : out STD_LOGIC; -- Tx RS232 Line
						i_uart_rxd     : in  STD_LOGIC;  -- Rx RS232 Line
						i_uart_cts_n     : in  STD_LOGIC ; -- cts RS232 Line
						o_uart_rts_n     : out STD_LOGIC -- rts RS232 Line
					);
		end component;	
	-- Bootloader ROM -----------------------------------------------------------------
	-- -----------------------------------------------------------------------------------
		component BOOT_ROM_FILE
			generic	(
						MEM_SIZE      : natural; -- memory cells
						LOG2_MEM_SIZE : natural; -- log2(memory cells)
						OUTPUT_GATE   : boolean; -- use output gate
						INIT_IMAGE_ID : string   -- init image
					);
			port	(
						-- Wishbone Bus --
						WB_CLK_I      : in  STD_LOGIC; -- memory master clock
						WB_RST_I      : in  STD_LOGIC; -- high active sync reset
						WB_CTI_I      : in  STD_LOGIC_VECTOR(02 downto 0); -- cycle indentifier
						WB_TGC_I      : in  STD_LOGIC_VECTOR(06 downto 0); -- cycle tag
						WB_ADR_I      : in  STD_LOGIC_VECTOR(LOG2_MEM_SIZE-1 downto 0); -- adr in
						WB_DATA_I     : in  STD_LOGIC_VECTOR(31 downto 0); -- write data
						WB_DATA_O     : out STD_LOGIC_VECTOR(31 downto 0); -- read data
						WB_SEL_I      : in  STD_LOGIC_VECTOR(03 downto 0); -- data quantity
						WB_WE_I       : in  STD_LOGIC; -- write enable
						WB_STB_I      : in  STD_LOGIC; -- valid cycle
						WB_ACK_O      : out STD_LOGIC; -- acknowledge
						WB_HALT_O     : out STD_LOGIC; -- throttle master
						WB_ERR_O      : out STD_LOGIC  -- abnormal cycle termination
					);
		end component;


	-- System Timer -------------------------------------------------------------------
	-- -----------------------------------------------------------------------------------
		component TIMER
			port (
						-- Wishbone Bus --
						WB_CLK_I      : in  STD_LOGIC; -- memory master clock
						WB_RST_I      : in  STD_LOGIC; -- high active sync reset
						WB_CTI_I      : in  STD_LOGIC_VECTOR(02 downto 0); -- cycle indentifier
						WB_TGC_I      : in  STD_LOGIC_VECTOR(06 downto 0); -- cycle tag
						WB_ADR_I      : in  STD_LOGIC_VECTOR(01 downto 0); -- adr in
						WB_DATA_I     : in  STD_LOGIC_VECTOR(31 downto 0); -- write data
						WB_DATA_O     : out STD_LOGIC_VECTOR(31 downto 0); -- read data
						WB_SEL_I      : in  STD_LOGIC_VECTOR(03 downto 0); -- data quantity
						WB_WE_I       : in  STD_LOGIC; -- write enable
						WB_STB_I      : in  STD_LOGIC; -- valid cycle
						WB_ACK_O      : out STD_LOGIC; -- acknowledge
						WB_HALT_O     : out STD_LOGIC; -- throttle master
						WB_ERR_O      : out STD_LOGIC; -- abnormal termination

						-- Match Interrupt --
						INT_O         : out STD_LOGIC
				 );
		end component;

		

	-- Vector Interrupt Controller ----------------------------------------------------
	-- -----------------------------------------------------------------------------------
		component VIC
			port (
						-- Wishbone Bus --
						WB_CLK_I      : in  STD_LOGIC; -- memory master clock
						WB_RST_I      : in  STD_LOGIC; -- high active sync reset
						WB_CTI_I      : in  STD_LOGIC_VECTOR(02 downto 0); -- cycle indentifier
						WB_TGC_I      : in  STD_LOGIC_VECTOR(06 downto 0); -- cycle tag
						WB_ADR_I      : in  STD_LOGIC_VECTOR(05 downto 0); -- adr in (word boundary)
						WB_DATA_I     : in  STD_LOGIC_VECTOR(31 downto 0); -- write data
						WB_DATA_O     : out STD_LOGIC_VECTOR(31 downto 0); -- read data
						WB_SEL_I      : in  STD_LOGIC_VECTOR(03 downto 0); -- data quantity
						WB_WE_I       : in  STD_LOGIC; -- write enable
						WB_STB_I      : in  STD_LOGIC; -- valid cycle
						WB_ACK_O      : out STD_LOGIC; -- acknowledge
						WB_HALT_O     : out STD_LOGIC; -- throttle master
						WB_ERR_O      : out STD_LOGIC; -- abnormal termination

						-- INT Lines & ACK --
						IRQ_LINES_I   : in  STD_LOGIC_VECTOR(31 downto 0);
						ACK_LINES_O   : out STD_LOGIC_VECTOR(31 downto 0);

						-- Global FIQ/IRQ signal to ARM4U --
						STORM_IRQ_O   : out STD_LOGIC;
						STORM_FIQ_O   : out STD_LOGIC
				 );
		end component;

begin

-- #################################################################################################################################
-- ###  ARM4U CORE PROCESSOR                                                                                                     ###
-- #################################################################################################################################

	-- Clock Manager (PLL) ---------------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		SYSCON_CLK: SYSTEM_PLL
			port map (
						inclk0 => CLK_I,     -- external clock input
						c0     => MAIN_CLK,  -- system clock
						c1     => XMEM_CLK,  -- ext mem clock for internal use
						c2     => XMEMD_CLK, -- ext mem clock, -3ns phase shifted
						locked => CLK_LOCK   -- clock stable
					);

		CLOCK_DIVIDER: process(CLK_I)
		begin
			if rising_edge(CLK_I) then
				CLK_DIV <= Std_Logic_Vector(unsigned(CLK_DIV)+1);
			end if;
		end process CLOCK_DIVIDER;


	-- Reset Manager ---------------------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		SYSCON_RST: RST_PROTECT
			generic	map (
							TRIGGER_VAL => RST_RTIGGER_C, -- trigger in sys clocks
							LOW_ACT_RST => LOW_ACTIVE_RST_C -- valid reset level
						)
			port map (
						MAIN_CLK_I => MAIN_CLK,
						EXT_RST_I  => RST_I,
						SYS_RST_O  => SAVE_RST,
						CPU_RST_O => CPU_RST
					 );

		MAIN_RST   <= SAVE_RST or (not CLK_LOCK); -- system reset
		MAIN_RST_N <= not MAIN_RST;

		-- FOR SIMULATION --
--		SAVE_RST <= not RST_I;

		A2WB: avalon_to_wb_bridge
			generic	(
						DW  => A2WB_DATA_W,	  -- Data width
						AW   => A2WB_ADR_W      -- Address width
					);
			port	(
						wb_clk_i        => MAIN_CLK,
						wb_rst_i 		=> CPU_RST,        -- global reset input
						
						-- Avalon Slave input
						s_av_address_i    => CORE_AV_ADDRESS_O,
						s_av_byteenable_i    => CORE_AV_BE_O,
						s_av_read_i    => CORE_AV_RD_O,
						s_av_readdata_o    => CORE_AV_RD_DAT_I,
						s_av_burstcount_i    => CORE_AV_BUST_CNT_O,
						s_av_write_i    => CORE_AV_WR_O,
						s_av_writedata_i    => CORE_AV_WR_DAT_O,
						s_av_waitrequest_o    => CORE_AV_WAIT_REQ_I,
						s_av_readdatavalid_o    => CORE_AV_RD_DAT_VALID,
						
						-- Wishbone Master Output
						wbm_adr_o          => CORE_WB_ADR_O,   -- address
						bm_dat_o         => CORE_WB_DATA_O,  -- data out
						wbm_sel_o          => CORE_WB_SEL_O,   -- byte select
						wbm_we_o           => CORE_WB_WE_O,    -- write enable
						wbm_cyc_o          => CORE_WB_CYC_O,   -- valid cycle
						wbm_stb_o          => CORE_WB_STB_O,   -- valid transfer
						wbm_cti_o          => CORE_WB_CTI_O,   -- cycle type
						wbm_bte_o          => CORE_WB_BTE_O,   -- cycle type
						wbm_dat_i         => CORE_WB_DATA_I,  -- data in
						wbm_ack_i          => CORE_WB_ACK_I,   -- acknowledge
						-- wbm_rty_i			: in  STD_LOGIC
						wbm_err_i          => CORE_WB_ERR_I   -- abnormal termination
					);
		

	-- ARM4U CORE PROCESSOR --------------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		ARM4U: arm4u_cpu
					port(
						-- Globals
						clk						=> MAIN_CLK,        -- core clock input
						reset					=> CPU_RST,        -- global reset input
						
						--Avalon Master Interface for instructions
						avm_inst_waitrequest	=> CORE_AV_WAIT_REQ_I,
						avm_inst_readdatavalid	=> CORE_AV_RD_DAT_VALID,
						avm_inst_readdata		=> CORE_AV_RD_DAT_I,
						avm_inst_read			=> CORE_AV_RD_O,
						avm_inst_burstcount		=> CORE_AV_BUST_CNT_O,
						avm_inst_address		=> CORE_AV_ADDRESS_O,
						
						--Avalon Master Interface for data
						avm_data_waitrequest	=> CORE_AV_WAIT_REQ_I,
						avm_data_readdatavalid	=> CORE_AV_RD_DAT_VALID,
						avm_data_readdata		=> CORE_AV_RD_DAT_I,
						avm_data_read			=> CORE_AV_RD_O,
						avm_data_writedata		=> CORE_AV_WR_DAT_O,
						avm_data_write			=> CORE_AV_WR_O,
						avm_data_byteen			=> CORE_AV_BE_O,
						avm_data_burstcount		=> CORE_AV_BUST_CNT_O,
						avm_data_address		=> CORE_AV_ADDRESS_O,
						
						--Interrupt interface
						inr_irq (00)    		=> STORM_IRQ,       -- interrupt request
						inr_irq (01)			=> STORM_FIQ,        -- fast interrupt request
						inr_irq (31 downto 02) 	<= (others => '0')
					);



-- #################################################################################################################################
-- ###  WISHBONE FABRIC                                                                                                          ###
-- #################################################################################################################################

	-- Valid Transfer Signal Terminal ----------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		INT_MEM_STB_I     <= CORE_WB_STB_O when ((CORE_WB_ADR_O >= INT_MEM_BASE_C)    and (CORE_WB_ADR_O < Std_logic_Vector(unsigned(INT_MEM_BASE_C)    + INT_MEM_SIZE_C)))    else '0';
		SDRAM_MEM_STB_I   <= CORE_WB_STB_O when ((CORE_WB_ADR_O >= SDRAM_MEM_BASE_C)  and (CORE_WB_ADR_O < Std_logic_Vector(unsigned(SDRAM_MEM_BASE_C)  + SDRAM_MEM_SIZE_C)))  else '0';
		BOOT_ROM_STB_I    <= CORE_WB_STB_O when ((CORE_WB_ADR_O >= BOOT_ROM_BASE_C)   and (CORE_WB_ADR_O < Std_logic_Vector(unsigned(BOOT_ROM_BASE_C)   + BOOT_ROM_SIZE_C)))   else '0';
		SYS_TIMER0_STB_I  <= CORE_WB_STB_O when ((CORE_WB_ADR_O >= SYS_TIMER0_BASE_C) and (CORE_WB_ADR_O < Std_logic_Vector(unsigned(SYS_TIMER0_BASE_C) + SYS_TIMER0_SIZE_C))) else '0';
		UART0_STB_I       <= CORE_WB_STB_O when ((CORE_WB_ADR_O >= UART0_BASE_C)      and (CORE_WB_ADR_O < Std_logic_Vector(unsigned(UART0_BASE_C)      + UART0_SIZE_C)))      else '0';
		UART1_STB_I       <= CORE_WB_STB_O when ((CORE_WB_ADR_O >= UART1_BASE_C)      and (CORE_WB_ADR_O < Std_logic_Vector(unsigned(UART1_BASE_C)      + UART1_SIZE_C)))      else '0';
		VIC_STB_I         <= CORE_WB_STB_O when ((CORE_WB_ADR_O >= VIC_BASE_C)        and (CORE_WB_ADR_O < Std_logic_Vector(unsigned(VIC_BASE_C)        + VIC_SIZE_C)))        else '0';


	-- Read-Back Data Selector Terminal --------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		CORE_WB_DATA_I <=
			INT_MEM_DATA_O     when (INT_MEM_STB_I     = '1') else
			SDRAM_MEM_DATA_O   when (SDRAM_MEM_STB_I   = '1') else
			BOOT_ROM_DATA_O    when (BOOT_ROM_STB_I    = '1') else
			SYS_TIMER0_DATA_O  when (SYS_TIMER0_STB_I  = '1') else
			UART0_DATA_O       when (UART0_STB_I       = '1') else
			UART1_DATA_O       when (UART1_STB_I       = '1') else
			VIC_DATA_O         when (VIC_STB_I         = '1') else
			x"00000000";


	-- Acknowledge Terminal --------------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		CORE_WB_ACK_I  <= INT_MEM_ACK_O      or
						  BOOT_ROM_ACK_O     or
						  SDRAM_MEM_ACK_O    or  -- causes hang.
						  SYS_TIMER0_ACK_O   or
						  UART0_ACK_O        or
						  UART1_ACK_O        or
						  VIC_ACK_O          or
						  '0';


	-- Abnormal Termination Terminal -----------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		CORE_WB_ERR_I  <= INT_MEM_ERR_O      or
						  BOOT_ROM_ERR_O     or
						  SDRAM_MEM_ERR_O    or
						  SYS_TIMER0_ERR_O   or
						  UART0_ERR_O        or
						  UART1_ERR_O        or
						  VIC_ERR_O          or
						  '0';


	-- Halt Terminal ---------------------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		CORE_WB_HALT_I <= INT_MEM_HALT_O     or
						  BOOT_ROM_HALT_O    or
						  SDRAM_MEM_HALT_O   or
						  SYS_TIMER0_HALT_O  or
						  UART0_HALT_O       or
						  UART1_HALT_O       or
						  VIC_HALT_O         or
						  '0';



-- #################################################################################################################################
-- ###  SYSTEM COMPONENTS                                                                                                        ###
-- #################################################################################################################################

	-- Internal Working Memory -----------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		INTERNAL_SRAM_MEMORY: MEMORY
			generic map	(
						MEM_SIZE      => INT_MEM_SIZE_C/4,       -- memory size in 32-bit cells
						LOG2_MEM_SIZE => log2(INT_MEM_SIZE_C/4), -- log2 memory size in 32-bit cells
						OUTPUT_GATE   => USE_OUTPUT_GATES_C      -- output and-gate, might be necessary for some bus systems
						)
			port map (
						WB_CLK_I      => MAIN_CLK,
						WB_RST_I      => MAIN_RST,
						WB_CTI_I      => CORE_WB_CTI_O,
						WB_TGC_I      => CORE_WB_TGC_O,
						WB_ADR_I      => CORE_WB_ADR_O(log2(INT_MEM_SIZE_C/4)+1 downto 2), -- word boundary access
						WB_DATA_I     => CORE_WB_DATA_O,
						WB_DATA_O     => INT_MEM_DATA_O,
						WB_SEL_I      => CORE_WB_SEL_O,
						WB_WE_I       => CORE_WB_WE_O,
						WB_STB_I      => INT_MEM_STB_I,
						WB_ACK_O      => INT_MEM_ACK_O,
						WB_HALT_O     => INT_MEM_HALT_O,
						WB_ERR_O      => INT_MEM_ERR_O
					);



	-- Boot ROM Memory -------------------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		BOOT_MEMORY: BOOT_ROM_FILE
			generic map (
							MEM_SIZE      => BOOT_ROM_SIZE_C/4, -- memory size in 32-bit words
							LOG2_MEM_SIZE => log2(BOOT_ROM_SIZE_C/4), -- log2 memory size in words
							OUTPUT_GATE   => USE_OUTPUT_GATES_C, -- use output gate
							INIT_IMAGE_ID => BOOT_IMAGE_C -- init image
						)
			port map (
						-- Wishbone Bus --
						WB_CLK_I      => MAIN_CLK,
						WB_RST_I      => MAIN_RST,
						WB_CTI_I      => CORE_WB_CTI_O,
						WB_TGC_I      => CORE_WB_TGC_O,
						WB_ADR_I      => CORE_WB_ADR_O(log2(BOOT_ROM_SIZE_C/4)+1 downto 2), -- word boundary
						WB_DATA_I     => CORE_WB_DATA_O,
						WB_DATA_O     => BOOT_ROM_DATA_O,
						WB_SEL_I      => CORE_WB_SEL_O,
						WB_WE_I       => CORE_WB_WE_O,
						WB_STB_I      => BOOT_ROM_STB_I,
						WB_ACK_O      => BOOT_ROM_ACK_O,
						WB_HALT_O     => BOOT_ROM_HALT_O,
						WB_ERR_O      => BOOT_ROM_ERR_O
					);
	-- --------------------------------------------------------------------------------------------------------
	-- SDRAM Controller 0 -------------------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		SDRAM_CTRL_0: wb_sdram_ctrl
--			generic map (
--						BURST_LENGTH	=> SDRAM_BURST_LEN
--					)
			port map (
						-- Wishbone Bus --
						wb_clk        => MAIN_CLK,
						wb_rst        => MAIN_RST,
						  
						wb_adr_i      => CORE_WB_ADR_O, 
						wb_dat_i      => CORE_WB_DATA_O,
						wb_dat_o      => SDRAM_MEM_DATA_O,
						wb_sel_i      => CORE_WB_SEL_O,
						wb_cyc_i      => CORE_WB_CYC_O,
						wb_stb_i      => SDRAM_MEM_STB_I,
						wb_we_i       => CORE_WB_WE_O,
						wb_ack_o      => SDRAM_MEM_ACK_O,
						wb_cti_i	  => CORE_WB_CTI_O,
						wb_bte_i	  => CORE_WB_BTE_O,

				-- Interface to SDRAMs 
						sdram_clk		=> XMEM_CLK,
						sdram_rst		=>MAIN_RST,
						cke_pad_o		=> SDRAM_CKE_O,
						cs_n_pad_o		=> SDRAM_CS_O,
						we_pad_o		=> SDRAM_WE_O,
						cas_pad_o		=> SDRAM_CAS_O,
						ras_pad_o		=> SDRAM_RAS_O,
						dqm_pad_o		=> SDRAM_DQM_O,
						a_pad_o			=> SDRAM_ADDR_O,
						ba_pad_o		=> SDRAM_BA_O,
						dq_o			=> internal_dqo,
						dq_i			=> internal_dqi,
						dq_oe			=> internal_dqoe 
					);


		SDRAM_DQ_IO <= internal_dqo when (internal_dqoe = '1') else "ZZZZZZZZZZZZZZZZ";
		internal_dqi <= SDRAM_DQ_IO when (internal_dqoe = '0') else "ZZZZZZZZZZZZZZZZ";
		--SDRAM to deal with Pipelined request from  CPU.
		SDRAM_MEM_HALT_O <= '0' when CORE_WB_CYC_O='0' else not SDRAM_MEM_ACK_O;
		SDRAM_MEM_ERR_O <= '0';-- nothing can go wrong - never ever!

		SDRAM_CLK_O <= XMEM_CLK;
	
	-- General Purpose UART 0 ------------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		GP_UART_0: MINI_UART
			generic map	(
							BRDIVISOR => UART0_BAUD_VAL_C
						)
			port map (
						-- Wishbone Bus --
						WB_CLK_I      => MAIN_CLK,
						WB_RST_I      => MAIN_RST,
						WB_CTI_I      => CORE_WB_CTI_O,
						WB_TGC_I      => CORE_WB_TGC_O,
						WB_ADR_I      => CORE_WB_ADR_O(2),
						WB_DATA_I     => CORE_WB_DATA_O,
						WB_DATA_O     => UART0_DATA_O,
						WB_SEL_I      => CORE_WB_SEL_O,
						WB_WE_I       => CORE_WB_WE_O,
						WB_STB_I      => UART0_STB_I,
						WB_ACK_O      => UART0_ACK_O,
						WB_HALT_O     => UART0_HALT_O,
						WB_ERR_O      => UART0_ERR_O,

						-- Terminal signals --
						IntTx_O       => UART0_TX_IRQ,
						IntRx_O       => UART0_RX_IRQ,
						BR_Clk_I      => MAIN_CLK,
						TxD_PAD_O     => UART0_TXD_O,
						RxD_PAD_I     => UART0_RXD_I
					);

	-- Amber UART 1-------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		GP_UART_1: amber_uart
			port map (
						-- Wishbone Bus --
						i_clk      => MAIN_CLK,
--						wb_rst_i  =>  MAIN_RST, 
--						WB_CTI_I      : in  STD_LOGIC_VECTOR(02 downto 0); -- cycle indentifier
--						WB_TGC_I      : in  STD_LOGIC_VECTOR(06 downto 0); -- cycle tag
						i_wb_adr      => CORE_WB_ADR_O,
						i_wb_dat     => CORE_WB_DATA_O,
						o_wb_dat     => UART1_DATA_O,
						i_wb_sel      => CORE_WB_SEL_O,
						i_wb_we       => CORE_WB_WE_O,
						i_wb_stb     => UART1_STB_I,
						o_wb_ack      => UART1_ACK_O,
						o_wb_err      => UART1_ERR_O,
						i_wb_cyc      => CORE_WB_CYC_O,
						-- Terminal signals --
						o_uart_int        => UART1_RX_IRQ,
--						IntRx_O       : out STD_LOGIC; -- Receive interrupt: indicate Byte received
						o_uart_txd    => UART1_TXD_O,
						i_uart_rxd      => UART1_RXD_I,
						i_uart_cts_n   =>'0' -- cts RS232 Line
--						o_uart_rts_n     : out STD_LOGIC -- rts RS232 Line
				);
									
						
		UART1_HALT_O <= '0';-- nothing can go wrong - never ever!

	-- System Timer 0 --------------------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		SYSTEM_TIMER_0: TIMER
			port map (
						-- Wishbone Bus --
						WB_CLK_I      => MAIN_CLK,
						WB_RST_I      => MAIN_RST,
						WB_CTI_I      => CORE_WB_CTI_O,
						WB_TGC_I      => CORE_WB_TGC_O,
						WB_ADR_I      => CORE_WB_ADR_O(3 downto 2),
						WB_DATA_I     => CORE_WB_DATA_O,
						WB_DATA_O     => SYS_TIMER0_DATA_O,
						WB_SEL_I      => CORE_WB_SEL_O,
						WB_WE_I       => CORE_WB_WE_O,
						WB_STB_I      => SYS_TIMER0_STB_I,
						WB_ACK_O      => SYS_TIMER0_ACK_O,
						WB_HALT_O     => SYS_TIMER0_HALT_O,
						WB_ERR_O      => SYS_TIMER0_ERR_O,

						-- Match Interrupt --
						INT_O         => SYS_TIMER0_IRQ
				 );




	-- Vector Interrupt Controller -------------------------------------------------------------------------
	-- --------------------------------------------------------------------------------------------------------
		VECTOR_INTERRUPT_CONTROLLER: VIC
			port map (
						-- Wishbone Bus --
						WB_CLK_I      => MAIN_CLK,
						WB_RST_I      => MAIN_RST,
						WB_CTI_I      => CORE_WB_CTI_O,
						WB_TGC_I      => CORE_WB_TGC_O,
						WB_ADR_I      => CORE_WB_ADR_O(log2(VIC_SIZE_C/4)+1 downto 2),
						WB_DATA_I     => CORE_WB_DATA_O,
						WB_DATA_O     => VIC_DATA_O,
						WB_SEL_I      => CORE_WB_SEL_O,
						WB_WE_I       => CORE_WB_WE_O,
						WB_STB_I      => VIC_STB_I,
						WB_ACK_O      => VIC_ACK_O,
						WB_HALT_O     => VIC_HALT_O,
						WB_ERR_O      => VIC_ERR_O,

						-- INT Lines & ACK --
						IRQ_LINES_I   => INT_LINES,
						ACK_LINES_O   => INT_LINES_ACK,

						-- Global IRQ/FIQ signal to ARM4U --
						STORM_IRQ_O   => STORM_IRQ,
						STORM_FIQ_O   => STORM_FIQ
				 );

			-- IRQ/FIQ Lines --
			INT_LINES(00) <= SYS_TIMER0_IRQ;
			INT_LINES(01) <= GP_IO0_IRQ;
			INT_LINES(02) <= UART0_TX_IRQ;
			INT_LINES(03) <= UART0_RX_IRQ;
			INT_LINES(04) <= SPI0_CTRL_IRQ;
			INT_LINES(05) <= I2C0_CTRL_IRQ;
			INT_LINES(06) <= UART1_RX_IRQ;
			INT_LINES(31 downto 07) <= (others => '0'); -- unused



end Structure;

