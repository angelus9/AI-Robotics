


///////////////////////////////////////////////////////////////////////////////////////////////////
// Open Source Code, 
//
// File: ForthProc.v
// File history:
// Revision number: 2.0 : <10-9-2021>:
//
// Description: First working code from Demitri.  I had to change some blocking statements to non-blocking.
// 			skip_op <= true;
//
//
// Targeted device: <Family::IGLOO2> <Die::M2GL005> <Package::144 TQ>
// Authors: Don Golding & Dimitri Peynado
// 
/////////////////////////////////////////////////////////////////////////////////////////////////// 

// Don Golding & Dimitri Peynado
// AI Engineering
// donaldrgolding@gmail.com
//
//                       *** Forth Processor IN System Verilog ***
//
// A high level of abstraction Forth Processor core using the latest System Verilog synthisizable
// constructs is the design goal.  The code is very high level and easy to read, this is extremely
// important.
//
// We also want support to building high level languages on top of this Forth Processor
// extending this Forth to include AI languages such as Prolog extensions.  Forth is the perfect language
// to create new languages or incorporate desirable features of other languages.
//
// Previous chip based Forth Microprocessors include: Harris RTX2000, SHABOOM, F21,etc.

// We believe a straight forward simple design will have considerable applications
// when you need a processor included in your FPGA/ASIC design.
// FPGAs operate at 200mhz or higher, I don't know how fast this design will be, but it's speed
// should be limited to the external RAM speed when memory access is required.
// Internal logic operations should be 50-200mhz range.
//
// The preliminary specifications are:
//
//  32 bit data bus 
//  16 bit address bus
//  by editing the code in the Entity declariations, you implement 16, 32, 64, or ? designs
//
//  Return Stack levels=256
//  Data Stack levels=256 
//  Output port A is 8 lines
//  Output port B is 8 lines
// Please review it and email me with your input on either Forth design issues or
// System Verilog design issues.
// Other Contributors: Dr. Ting

//syn_ramstyle = "lsram"
//module
// spram256 user modules //
//SB_SPRAM256KA SRAM(
 //.DATAIN(DATAIN),
 //.ADDRESS(ADDRESS),
 //.MASKWREN(MASKWREN),
 //.WREN(WREN),
 //.CHIPSELECT(CHIPSELECT),
 //.CLOCK(CLOCK),
 //.STANDBY(STANDBY),
 //.SLEEP(SLEEP),
 //.POWEROFF(POWEROFF),
 //.DATAOUT(DATAOUT_A)
//) 
//end module

parameter DataWidthSystem = 31;
parameter char_buf_ptr_width = 7;

//Internal SRAM parameters
parameter Int_SRAM_size = 31;
parameter Int_SRAM_ADDR_size = 500;

//LSRAM2
parameter data_width = 9;
parameter address_width = 10;
parameter ext_ram_size = 1024;

//BootROM
parameter end_boot_ROM = 28;
parameter clock_1MHZ_divider = 6;
parameter clock_1KHZ_divider = 500;
parameter clock_500MS_divider = 2000;
parameter clock_SPI1_divider = 12;
parameter clock_SPI2_divider = 12;

//PWM
parameter PWM_reg_size = 16;
parameter PWM_range = 200;

//SPI
parameter SPI_8_reg_size = 4;// ss = low, 8 data pulses, ss = high
parameter SPI_16_reg_size = 5;//16 data pulses 

//Define System Architecture
typedef enum logic [15:0]{

//System Operators
_execute, _abort, _end_boot,  _run_boot, _create, _does, _compile, _bracket_compile, _interpret, _do_colon, _exit, _number,

//Stack Operators
_depth, _dup, _pick, _over, _swap, _rot, _equal, _zero_equal, _greater_than, _less_than,
_question_dup, _drop, _roll, _to_return, _from_return, _copy_return,
_lit,

//Memory Operators              
_8store, _8fetch, _8plus_store, _16store, _16fetch, _32store, _32fetch, _cmove, _fill,
_ROM_active, _SRAM_active, _EXSRAM_active,

//Arithmetic
_plus, _minus, _times, _divide, _max, _min, _times_mod, _divide_mod, _times_divide, _one_plus, _one_minus,

//Conditional
_if, _else,_then, _0branch, _branch,

//Loops
_begin, _again, _until, _for, _next,

//Logical
_and, _or, _xor, _zero_less_than, _zero_greater_than, _zero_equals,

//Communications
_key, _emit,

//I/O Processing                
_io_led, _io_button, _io_fetch, _io_store

} op_e;

module ForthProc

#( parameter
  address_size      = 9,//size-1
  data_size         = DataWidthSystem,//size-1
  memory_size       = 1023,//size-1
  port_size         = 7,//size-1
  code_size         = 15,//size-1
  io_size           = 7,//size-1  
  data_stack_depth  = 16,
  return_stack_depth = 8,
  boot_depth        = 50,
  ram_depth         = 500,
  high		        = 1'b1,
  low		        = 1'b0,
  LED_on			= 1'b0,
  LED_off			= 1'b1,
  true		        = 1'b1,
  false	            = 1'b0,
  dstack_start      = 0,
  read              = 1'b1, 
  write             = 1'b0
)  

//Module I/O 
(  
    input logic reset,
    input logic clk,
	output logic TX,
	input logic RX,

//Dev Board Specific I/O
input  BUTTON0,
input  BUTTON1,
input SPI1_8_in,
input SPI2_16_in,

output logic LED_G,
output logic LED_B,
output logic LED_R,

//Devices
output logic PWM_CH1,
output logic PWM_CH2,
output logic PWM_CH3,
output logic PWM_CH4,
output logic SPI1_8_out,
output logic SPI2_16_out,
output logic SPI1_8_ss,
output logic SPI2_16_ss,
output logic SPI1_8_clk,
output logic SPI2_16_clk
);

logic [Int_SRAM_size:0] IntMem [Int_SRAM_ADDR_size:0];

logic n_main_clk;
logic [3:0] active_mem;//which memory is active? ROM, INTSRAM or EXSRAM    
logic [3:0] errorcode ; 
logic	[address_size:0] mp;      // memory pointer
logic	[address_size:0] bp;      // boot ROM memory pointer
logic	[1:0] successful;
logic	[code_size:0] opcode;

logic [4:0] clock_1MHZ_ctr;
logic [10:0] clock_1KHZ_ctr = 12;//from 1MHZ clock
logic [10:0] clock_500MS_ctr = 12;//from 1KHZ clock

logic [4:0] clock_1MHZ;
logic [10:0] clock_1KHZ;//from 1MHZ clock
logic [10:0] clock_500MS;//from 1KHZ clock

//PWM Registers
logic [PWM_reg_size:0] PWM_div_counter;
logic [PWM_reg_size:0] PWM_CH1_compare_reg;
logic [PWM_reg_size:0] PWM_CH2_compare_reg;
logic [PWM_reg_size:0] PWM_CH3_compare_reg;
logic [PWM_reg_size:0] PWM_CH4_compare_reg;

//SPI Registers
logic [SPI_8_reg_size:0] SPI1_8_div_counter;
logic [7:0] SPI1_8_data_out;
logic [7:0] SPI1_8_data_in;

logic [SPI_16_reg_size:0] SPI2_16_div_counter;
logic [15:0] SPI1_16_data_out;
logic [15:0] SPI1_16_data_in;

//Circular stacks
logic [data_size:0] data_stack[data_stack_depth] ;
logic [data_size:0] return_stack[return_stack_depth];
logic [$clog2(data_stack_depth)-1:0] dp;
logic [$clog2(data_stack_depth)-1:0] rp;

//Internal Boot ROM
logic [data_size:0] boot_ROM [boot_depth];

//Lattice Internal memory
 logic [Int_SRAM_ADDR_size:0] mem[Int_SRAM_size:0]; /// memory block
 
//External Memory
logic [data_size:0] DataBus;
logic [address_size:0] AddressBus;

//Forth Registers
logic busy;
logic skip_op;
logic branch;
logic [address_size:0] branch_addr;

logic n_BUTTON0;
logic n_BUTTON1;
logic n_LED_G;
logic n_LED_B;
logic n_LED_R;

// UART registers
logic [7:0] tx_data;
logic uart_busy_tx;
logic uart_send;
logic uart_busy_rx;
logic uart_receive;
logic uart_rx_valid;
logic [7:0] uart_rx_data;

// Execution Token
logic [address_size:0] xt[256];
logic [7:0] xtrp;
logic [7:0] xtwp;
logic xt_valid;
logic xt_ready;

logic [data_size:0] number[256];
logic [7:0] nrp;
logic [7:0] nwp;

//Boot code when the processor starts...
task automatic t_init_boot_code;
	boot_ROM[0] <=  _execute;
	boot_ROM[1] <=  _lit;
	boot_ROM[2] <=  "?";
	boot_ROM[3] <=  _emit;
	boot_ROM[4] <= _branch;
	boot_ROM[5] <= 0;
	boot_ROM[6] <=  _lit;
	boot_ROM[7] <= 4;
	boot_ROM[8] <= _io_led;
	boot_ROM[9] <= _branch;
	boot_ROM[10] <= 0;
	boot_ROM[11] <=  _lit;
	boot_ROM[12] <= 1;
	boot_ROM[13] <= _io_led;
	boot_ROM[14] <= _branch;
	boot_ROM[15] <= 0;
	boot_ROM[16] <=  _lit;
	boot_ROM[17] <= 2;
	boot_ROM[18] <= _io_led;
	boot_ROM[19] <= _branch;
	boot_ROM[20] <= 0;
	boot_ROM[21] <= _number;
	boot_ROM[22] <= _branch;
	boot_ROM[23] <= 0;
	boot_ROM[24] <= _io_led;
	boot_ROM[25] <= _branch;
	boot_ROM[26] <= 0;
	boot_ROM[27] <= _lit;
	boot_ROM[28] <= 5000000;
	boot_ROM[29] <= _lit;
	boot_ROM[30] <= 1;
	boot_ROM[31] <= _minus;
	boot_ROM[32] <= _dup;
	boot_ROM[33] <= _zero_equal;
	boot_ROM[34] <= _0branch;
	boot_ROM[35] <= 29;
	boot_ROM[36] <= _drop;
	boot_ROM[37] <= _branch;
	boot_ROM[38] <= 0;
	boot_ROM[39] <= _lit;
	boot_ROM[40] <= " ";
	boot_ROM[41] <= _emit;
	boot_ROM[42] <= _lit;
	boot_ROM[43] <= "0";
	boot_ROM[44] <= _plus;
	boot_ROM[45] <= _emit;
	boot_ROM[46] <= _execute;
	boot_ROM[47] <= _plus;
endtask : t_init_boot_code 

//Demetri: can you change the Outer Interpreter code to use this RAM based dictionary?
//build dictionary for testing...
task automatic t_init_dictionary_code;
	mem[0] <= 3;					// Link to next dictionary entry
	mem[1] <= {8'd1,"r","e","d"};	// Name field (NFA)
	mem[2] <= 6;					// Boot ROM address containing code (XT/CFA)
	mem[3] <= 7;
	mem[4] <= {8'd2,"g","r","e"};
	mem[5] <= {"e","n","\0","\0"};
	mem[6] <= 11;
	mem[7] <= 11;
	mem[8] <= {8'd2,"b","l","u"};
	mem[9] <= {"e","\0","\0","\0"};
	mem[10] <= 16;
	mem[11] <= 15;
	mem[12] <= {8'd2,"d","e","l"};
	mem[13] <= {"a","y","\0","\0"};
	mem[14] <= 27;
	mem[15] <= 18;
	mem[16] <= {8'd1,"l","e","d"};
	mem[17] <= 8;
	mem[18] <= 0;
	mem[19] <= {8'd1,".","\0","\0"};
	mem[20] <= 39;
endtask : t_init_dictionary_code

// Forth Outer Interpreter
always_ff @(posedge clk) begin
	logic [7:0] byte_in;
	logic [31:0] wp;
	logic [31:0] link_addr;
	logic [7:0] ccell, cchar, cells;
	logic [3:0][31:0] word_in;
	logic [address_size:0] local_xt;
	enum logic [2:0] {IDLE, PARSE, SEARCH, GET_LINK, GET_XT, EXECUTE, NUMBER, COMPILE} state;
	
	if (reset == 1'b0) begin
		wp = '0;
		xt_valid <= 1'b0;
		state = IDLE;
		uart_receive <= 1'b1;
		xtwp = 0;
		nwp = '0;
		local_xt=0;
		word_in = '0;
		cells = '0;
		cchar = '0;
	end
	else begin
		case (state)
			IDLE : begin
				wp = '0;
				if (uart_rx_valid) begin
					uart_receive <= 1'b0;
					byte_in = uart_rx_data;
					state = PARSE;
				end
			end
			PARSE : begin
				if (byte_in inside {[10:13]}) begin//is character between <bl> and <cr>?
					xt[xtwp++] = local_xt;
					local_xt = _execute;
					word_in = '0;
					cells = '0;
					cchar = '0;
					state = EXECUTE;
				end
				else if (byte_in == " ") begin
					xt[xtwp++] = local_xt;
					local_xt = _execute;
					state = IDLE;
					word_in = '0;
					cells = '0;
					cchar = '0;
					uart_receive <= 1'b1;
				end
				else begin
					cchar++;
					case (cchar % 4)
						0 : word_in[cchar / 4][31:24] = byte_in;
						1 : word_in[cchar / 4][23:16] = byte_in;
						2 : word_in[cchar / 4][15:8] = byte_in;
						3 : word_in[cchar / 4][7:0] = byte_in;
						default :;
					endcase
					cells = 1 + (cchar / 4);
					word_in[0][31:24] = cells;
					state = GET_LINK;
				end
			end
			GET_LINK :  begin
				link_addr = mem[wp];		
				++wp;
				ccell = '0;
				state = SEARCH;
			end
			SEARCH : begin
				if (ccell < cells && mem[wp] == word_in[ccell]) begin
					// token
					++wp;
					++ccell;
				end
				else if (ccell >= cells) begin
					state = GET_XT;
				end
				else begin
					wp = link_addr;
					state = link_addr ? GET_LINK : NUMBER;
				end
			end
			GET_XT: begin
				local_xt = mem[wp];
				state = IDLE;
				uart_receive <= 1'b1;
			end			
			NUMBER : begin
				// TODO: more than one char numbers
				if (byte_in inside{["0":"9"]}) begin
					number[nwp++] = byte_in - "0";
					local_xt = 21; // CFA for "number"
				end
				else begin
					local_xt = 1; // CFA for error "word not found"
				end
				state = IDLE;
				uart_receive <= 1'b1;
			end
			EXECUTE : begin
				xt_valid <= (xtrp < xtwp); // FIFO not empty (read before write)
				if (xtrp == xtwp) begin    // Stop when FIFO empty (read at write)
					state = IDLE;
					uart_receive <= 1'b1;
				end
			end
			
			COMPILE: begin
				//xt_valid <= (xtrp < xtwp);
				//if (xtrp == xtwp) begin
					//state = IDLE;
					//uart_receive <= 1'b1;
				//end
			end
		default : state = IDLE;
		endcase
	end
end

	task automatic t_reset;
        t_init_boot_code;
        bp='0;
        dp='0;
        rp='0;
        mp ='0;
		xtrp = 0;
		nrp = '0;
        busy <= false;
        skip_op <= false;
		uart_send <= 1'b0;	
		t_init_dictionary_code;
	endtask

	task automatic t_Fetch_opcode;
		if (branch) begin
			DataBus = boot_ROM[branch_addr];
			bp = branch_addr+1;
			branch = false;
		end
		else begin
			DataBus = boot_ROM[bp];
			++bp;
		end
	
	endtask        

	//Execute opcodes task
	task automatic t_execute;
   
      opcode = DataBus[code_size:0];
	  case (opcode)
        _minus : begin
				--dp;
				data_stack[dp] = data_stack[dp] - data_stack[dp+1];
        end
        
        _plus : begin
		  --dp;
				data_stack[dp] = data_stack[dp] + data_stack[dp+1];
        end
              
		_dup : begin
			++dp;
			data_stack[dp] = data_stack[dp-1];
			
		end
		_over : begin
			data_stack[dp+1] = data_stack[dp-1];
          ++dp;
		end
        
		_drop : begin
			--dp;
		end
        
		_equal : begin
			--dp;
			data_stack[dp] = data_stack[dp+1] == data_stack[dp] ? -1 : 0;
		end
		_zero_equal : begin
			data_stack[dp] = data_stack[dp] == '0 ? -1 : 0;
		end
		_io_led : begin
			n_LED_G <= !data_stack[dp][0];
			n_LED_B <= !data_stack[dp][1];
			n_LED_R <= !data_stack[dp][2];
			--dp;
		end
		_io_button : begin
            data_stack[dp] = {BUTTON1,BUTTON0};
		end
		_lit : begin
			// TODO: allow to work for main memory too
			++dp;
			data_stack[dp] = boot_ROM[bp];
			skip_op <= true;
		end
		_and : begin
			--dp;
			data_stack[dp] = data_stack[dp] & data_stack[dp+1];
		end
		_or : begin
			--dp;
			data_stack[dp] = data_stack[dp] | data_stack[dp+1];
		end
		_0branch : begin
			branch = data_stack[dp] == 0;
			branch_addr = boot_ROM[bp];
			skip_op <= ~branch;
			--dp;
		end
		 _branch : begin
			branch = true;
			branch_addr = boot_ROM[bp];
        end    
        _emit : begin
			if (busy == false) begin
				uart_send <= 1'b1;
				busy = true;
				--dp;    
			end
			else if (uart_busy_tx && busy == true) begin
				tx_data   <= data_stack[dp+1][7:0];         
			end
			else if (!uart_busy_tx && busy == true) begin
				uart_send <= 1'b0;
				busy = false;
			end
        end        
		_execute : begin
			busy = true; // Wait until XT/CFA available
			if (xt_valid) begin
				branch_addr = xt[xtrp++];
				busy = false;
				branch = true;
			end
		end
		_number : begin
			++dp;
			data_stack[dp] = number[nrp++];
		end
		default : ;
	  endcase
      
  endtask : t_execute
 
//Forth Inner Interpreter (Fetch/Execute Unit)
always_ff @(posedge clk) begin

    if (reset == 1'b0) begin
        t_reset;
    end    
    else begin
		if (busy == false) begin
			t_Fetch_opcode;
		end
		if (skip_op == false) begin
			t_execute;
		end
		else begin
			skip_op <= false;
		end
        
        n_BUTTON0 <= BUTTON0;
        n_BUTTON1 <= BUTTON1;         
		LED_G <= n_LED_G;
 		LED_B <= n_LED_B; 
 		LED_R <= n_LED_R;		  
    end 
end

// UART RX
always_ff @(posedge clk) begin
	logic [3:0] count;
	logic [9:0] clk_count;
	logic [7:0] rxd;

	if (reset == 1'b0) begin
		count <= '0;
		clk_count <= '0;
		uart_busy_rx = 1'b0;
		uart_rx_valid <= 1'b0;
	end
	else if (uart_receive && uart_rx_valid) begin
		uart_rx_valid <= 1'b0;
	end
	else if ((uart_busy_rx || RX == 1'b0) && !uart_rx_valid) begin 
		uart_busy_rx = 1'b1;
		uart_rx_valid <= 1'b0;
		if (clk_count < 625) begin // 12MHz/625 = 19.2KHz = 19200 baud
			clk_count <= clk_count + 1;
		end
		else begin
			clk_count <= '0;
			count <= count + 1;
			if (count < 8) begin
				rxd <= {RX,rxd[7:1]};
			end
			else begin
				count <= '0;
				uart_busy_rx = 1'b0;
				uart_rx_valid <= 1'b1;
				uart_rx_data = rxd;
			end
		end
	end
end

// UART TX
always_ff @(posedge clk) begin
	logic [3:0] count;
	logic [9:0] clk_count;
	logic [7:0] txd;

	if (reset == 1'b0) begin
		TX <= 1'b1;
		count <= '0;
		clk_count <= '0;
		uart_busy_tx = 1'b0;
	end
	else if (uart_send) begin 
		uart_busy_tx = 1'b1;
		if (clk_count < 625) begin // 12MHz/625 = 19.2KHz = 19200 baud
			clk_count <= clk_count + 1;
		end
		else begin
			clk_count <= '0;
			if (count < 9) begin
				if (count == 0) begin
					TX <= 1'b0;
					txd <= tx_data;
				end
				else begin
					TX <= txd[0];
					txd <= txd >> 1;
				end
				count <= count+1;
			end
			else begin
				TX <= 1'b1;
				count <= '0;
				uart_busy_tx = 1'b0;
			end
		end
	end
end

//Process SPI1_8 out
always_ff @(posedge clk) begin
	logic i;
	
	if (reset == true) begin
		SPI1_8_div_counter = 0;
		SPI1_8_out <= low;
		SPI1_8_ss  <= high;
		SPI1_8_clk <= low;
	end
	
    else if (SPI1_8_div_counter == 0) begin
		SPI1_8_ss <= low;
    end

    else if (SPI1_8_div_counter <= 8) begin

		for (int i = 0; i < 8; i++) begin
			SPI1_8_clk <= low;
		   	SPI1_8_out <= 8'b1000000 & (SPI1_8_data_out << 1);//may need to AND bit mask 1000000 here
			SPI1_8_clk <= high;
		end
	end	
	
    else if (SPI1_8_div_counter >= 9) begin
		SPI1_8_ss <= high;	
		SPI1_8_div_counter = 0;
    end	
	
				

//		SPI1_8_out <= SPI1_8_in;
		++SPI1_8_div_counter;
end	

//Process SPI2_16 out
always_ff @(posedge clk) begin
	if (reset == true) begin
		SPI2_16_div_counter = 0;
		SPI2_16_out <= low;
		SPI2_16_out <= low;		
		SPI2_16_ss <= low;	
		SPI2_16_clk <= low;
	end	
	else begin
		SPI2_16_out <= SPI2_16_in;	
		++SPI2_16_div_counter;
	end	
end	
  
endmodule

//REFERENCE!

//F83 wordset
//http://forth.sourceforge.net/standard/fst83/fst83-12.htm

//Nucleus layer
//!  *  */  */MOD  +  +!  -  /  /MOD  0<  0=  0>  1+  1-  2+ 
//2-  2/  <  <=  >  >R  ?DUP  @  ABS  AND  C!  C@  CMOVE 
//CMOVE>  COUNT  D+  D<  DEPTH  DNEGATE  DROP  DUP  EXECUTE 
//EXIT  FILL  I  J  MAX  MIN  MOD  NEGATE  NOT  OR  OVER  PICK 
//R>  R@  ROLL  ROT  SWAP  U<  UM*  UM/MOD  XOR 

//Device layer 
//BLOCK  BUFFER  CR  EMIT  EXPECT  FLUSH  KEY  SAVE-BUFFERS 
//SPACE  SPACES  TYPE  UPDATE 

//Interpreter layer 
//#  #>  #S  #TIB  '  (  -TRAILING  .  .(  <#  >BODY  >IN 
//ABORT  BASE  BLK  CONVERT  DECIMAL  DEFINITIONS  FIND 
//FORGET  FORTH  FORTH-83  HERE  HOLD  LOAD  PAD  QUIT  SIGN 
//SPAN  TIB  U.  WORD
 
//Compiler layer 
//+LOOP  ,  ."  :  ;  ABORT"  ALLOT  BEGIN  COMPILE  CONSTANT 
//CREATE  DO  DOES>  ELSE  IF  IMMEDIATE  LEAVE  LITERAL  LOOP 
//REPEAT  STATE  THEN  UNTIL  VARIABLE  VOCABULARY  WHILE   
//[']  [COMPILE]  ] 


//End
