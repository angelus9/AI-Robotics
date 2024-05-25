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

parameter DataWidthSystem = 31;
parameter char_buf_ptr_width = 7;

//Internal SRAM parameters
parameter Int_SRAM_size = 256;
parameter Int_SRAM_ADDR_size = 31;

//LSRAM2
parameter data_width = 9;
parameter address_width = 10;
parameter ext_ram_size = 1024;

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

//ASCII Chars
parameter SPACE = 32;
parameter CR = 13;
parameter LF = 12;

//Define System Architecture
typedef enum logic [15:0]{

//System Operators
_execute, _abort, _end_boot,  _run_boot, _create, _does, _compile, _bracket_compile, _interpret, _do_colon, _exit,

//Stack Operators
_depth, _dup, _pick, _over, _swap, _rot, _equal, _zero_equal, _greater_than, _less_than,
_question_dup, _drop, _roll, _to_return, _from_return, _copy_return,
_lit,

//Memory Operators              
_8store, _8fetch, _8plus_store, _16store, _16fetch, _32store, _32fetch, _cmove, _fill,
_ROM_active, _SRAM_active, _EXSRAM_active,

//Arithmetic
_plus, _minus, _times, _divide, _max, _min, _times_mod, _divide_mod, _times_divide, _one_plus, _one_minus, _negate,

//Conditional
_if, _else,_then, _0branch, _branch,

//Loops
_begin, _again, _until, _for, _next,

//Logical
_and, _or, _xor, _not, _zero_less_than, _zero_greater_than, _zero_equals,

//Communications
_key, _emit,

// Outer interpreter
_parse,
//I/O Processing                
_io_led, _io_led_on, _io_led_off, _io_button, _io_fetch, _io_store

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
    input logic clk,
	input logic rst,
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

logic n_main_clk;
logic [3:0] active_mem;//which memory is active? ROM, INTSRAM or EXSRAM    
logic [3:0] errorcode ; 
logic	[address_size:0] mp;      // memory pointer
logic	[1:0] successful;
op_e opcode;

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

logic boot_enable;

//Demitri shouldn't this be:
//logic [data_size:0] bootROM [XTQ_START:0];
logic [data_size:0] bootROM [1000:0];//DG was 200 changed to 1000 didn't\increase LUTs
//Lattice Internal memory
 logic [Int_SRAM_size:0][Int_SRAM_ADDR_size:0] mem; /* synthesis syn_ramstyle="block_ram" */;
 
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
localparam XTQ_START = 200;
logic [address_size:0] xtrp;
logic [address_size:0] xtwp;
logic xt_valid;
logic xt_ready;

logic [7:0] nrp;
logic [7:0] nwp;

// Dictionary
logic dict_write;
logic mem_access_outer;
logic [address_size:0] mem_addr;
logic [Int_SRAM_ADDR_size:0] dict_wdata;
logic [address_size:0] wp; // dictionary pointer wp

logic reset;
logic [7:0] reset_cnt='0;
always_ff @(posedge clk) begin
	if (reset_cnt != 8'hff) begin
		reset_cnt <= reset_cnt + 1;
		reset <= 1'b0;
	end 
	else begin 
		reset <= rst;
	end
end

task automatic t_init_dictionary_code;

	//dg LED loop with emit
//	bootROM[0] <=  -55;
//	bootROM[1] <=  _io_led_on;//dg was -65
//	bootROM[2] <=  -42;
//	bootROM[3] <=  _lit;
//	bootROM[4] <=  ".";
//	bootROM[5] <=  _emit;
//	bootROM[6] <=  _io_led_off;
//	bootROM[7] <= -42;
//	bootROM[8] <=  _lit;
//	bootROM[9] <=  "|";
//	bootROM[10] <=  _emit;	
//	bootROM[11] <=  _branch;	
//	bootROM[12] <=  1;	
//Don 5/24 end program	

	//dg LED loop only
	//change line: 	bootROM[43] <= 650000; for 1 sec rate
	
	//Benchmark 17 insturctions per loop, times 1.3 Million times per second
	//  Clock is 12MHz on the Tinyvision.ai board.
	// 12 x 1,300,000 = 15.6MOPS
	
	bootROM[0] <=  _io_led_on;
	bootROM[1] <=  -6;
	bootROM[2] <=  _io_led_off;
	bootROM[3] <= -6;
	bootROM[4] <=  _branch;	
	bootROM[5] <=  0;
	bootROM[6] <= _lit;	
	bootROM[7] <= 650000;//dg was 5000000, 666666 for emit loop 650000 for 1 sec
	bootROM[8] <= _lit;
	bootROM[9] <= 1;
	bootROM[10] <= _minus;
	bootROM[11] <= _dup;
	bootROM[12] <= _zero_equal;
	bootROM[13] <= _0branch;
	bootROM[14] <= 8;
	bootROM[15] <= _drop;
	bootROM[16] <= _exit;	

//Print to terminal test - Print out "CORE I"
	//bootROM[0] <= _lit;//emit a <CR> to terminal/UART drops one character
	//bootROM[1] <= CR;
	//bootROM[2] <= _emit;
	//bootROM[3] <= _lit;
	//bootROM[4] <= "C";
	//bootROM[5] <= _emit;
	//bootROM[6] <= _lit;
	//bootROM[7] <= "O";
	//bootROM[8] <= _emit;
	//bootROM[9] <= _lit;
	//bootROM[10] <= "R";
	//bootROM[11] <= _emit;
	//bootROM[12] <= _lit;
	//bootROM[13] <= "E";
	//bootROM[14] <= _emit;		
	//bootROM[15] <= _lit;
	//bootROM[16] <= SPACE;
	//bootROM[17] <= _emit;
	//bootROM[18] <= _lit;
	//bootROM[19] <= "I";
	//bootROM[20] <= _emit;
	//bootROM[21] <= _lit;
	//bootROM[22] <= CR;
	//bootROM[23] <= _emit;

	//bootROM[24] <= _lit;	//bootROM[25] <= "o";
	//bootROM[26] <= _emit;
	//bootROM[27] <= _lit;
	//bootROM[28] <= "k";
	//bootROM[29] <= _emit;
	//bootROM[30] <= _lit;
	//bootROM[31] <= CR;
	//bootROM[32] <= _emit;
	
//Carrage Return - RealTerm doesn't need LFCR pair
	//bootROM[24] <= _emit;//
	//bootROM[25] <= _lit; //if your Terminal program needs it uncomment section
	//bootROM[26] <= CR;   //remember to renumber bootROM[x] x must be sequential
	//bootROM[27] <= _io_led_on;	
	//bootROM[28] <= _branch;	
	//bootROM[29] <= 0;

//continue number sequence from Line 357
//comment out if you use Carrage Return

	//bootROM[32] <= _io_led_on;
	//bootROM[33] <= _key;	
	//bootROM[34] <= _emit;
	//bootROM[35] <= _io_led_off;	
	//bootROM[36] <= _branch;	
	//bootROM[37] <= 32;
	//bootROM[33] <= _key;
	//bootROM[34] <= _emit;
	//bootROM[35] <= _branch;
	//bootROM[36] <= 33;
	//bootROM[36] <= -32;	//loop to bootROM[24] "goto CFA"
	
//Demitri code
	//bootROM[0] <=  -55;
	//bootROM[11] <=  _lit;
	//bootROM[12] <=  "?";
	//bootROM[13] <=  _emit;
	//bootROM[14] <= _exit;
	//bootROM[15] <= 0; 					// Link to next dictionary entry
	//bootROM[16] <= {8'd1,"l","e","d"};	// Name field (NFA)
	//bootROM[17] <= _io_led_on;				// code field
	//bootROM[18] <= _exit;					// end of code field
	//bootROM[19] <= 15;
	//bootROM[20] <= {8'd1,"r","e","d"};				
	//bootROM[21] <= _lit;
	//bootROM[22] <= 4;
	//bootROM[23] <= -17;
	//bootROM[24] <= _exit;				
	//bootROM[25] <= 19;
	//bootROM[26] <= {8'd2,"g","r","e"};
	//bootROM[27] <= {"e","n","\0","\0"};
	//bootROM[28] <= _lit;
	//bootROM[29] <= 1;
	//bootROM[30] <= -17;
	//bootROM[31] <= _exit;
	//bootROM[32] <= 25;
	//bootROM[33] <= {8'd2,"b","l","u"};
	//bootROM[34] <= {"e","\0","\0","\0"};
	//bootROM[35] <= _lit;
	//bootROM[36] <= 2;
	//bootROM[37] <= -17;
	//bootROM[38] <= _exit;
	//bootROM[39] <= 32;
	//bootROM[40] <= {8'd2,"d","e","l"};
	//bootROM[41] <= {"a","y","\0","\0"};
	//bootROM[42] <= _lit;
	//bootROM[43] <= 650000;//dg was 5000000, 666666 for emit loop 650000 for 1 sec
	//bootROM[44] <= _lit;
	//bootROM[45] <= 1;
	//bootROM[46] <= _minus;
	//bootROM[47] <= _dup;
	//bootROM[48] <= _zero_equal;
	//bootROM[49] <= _0branch;
	//bootROM[50] <= 44;
	//bootROM[51] <= _drop;
	//bootROM[52] <= _exit;
	//bootROM[53] <= -39;
	//bootROM[54] <= {8'd1,"o","k","\0","\0"};
	//bootROM[55] <= _lit;
	//bootROM[56] <= "o";
	//bootROM[57] <= _emit;
	//bootROM[58] <= _lit;
	//bootROM[59] <= "k";
	//bootROM[60] <= _emit;
	//bootROM[61] <= _lit;
	//bootROM[62] <= 13;
	//bootROM[63] <= _emit;
	//bootROM[64] <= _exit;
	//bootROM[65] <= _key;
	//bootROM[66] <= _dup;
	//bootROM[67] <= _emit;
	//bootROM[68] <= _dup;
	//bootROM[69] <= _parse;
	//bootROM[70] <= _lit;
	//bootROM[71] <= 13;
	//bootROM[72] <= _equal;
	//bootROM[73] <= _0branch;
	//bootROM[74] <= 65;
	//bootROM[75] <= _io_led;//dg turn on led
	//bootROM[76] <= _exit;
	
endtask : t_init_dictionary_code

logic [address_size:0] mem_diff;
assign boot_enable = (mem_access_outer ? wp : mp) < XTQ_START;

// memory manager
always_ff @(posedge clk or negedge reset) begin
	if (reset == 1'b0) begin
		t_init_dictionary_code;
	end
	else begin
		mem_diff = boot_enable ? '0 : -XTQ_START;
		mem_addr = (mem_access_outer ? wp : mp)+mem_diff;
		if (dict_write) begin
			mem[mem_addr] <= dict_wdata;
		end
	end
end

assign DataBus = boot_enable ? bootROM[mem_addr] : mem[mem_addr];

	logic [7:0] byte_in;
	logic [31:0] link_addr;
	logic [31:0] newest_def;
	logic [7:0] ccell, cchar, cells;
	logic [3:0][31:0] word_in;
	logic [data_size:0] local_xt;
	logic [3:0] count ;
	enum logic [3:0] {IDLE, PARSE, SEARCH, BEFORE_LINK, GET_LINK, GET_XT, ADD_EXIT, EXECUTE, NUMBER, COMPILE,
	NEW_LINK, NEW_DEF} state;
	enum logic [3:0]{INTERPRET, COLON, WORD, COMPILING, END_COMPILE} comp_state;

// Forth Outer Interpreter
//always_ff @(posedge clk) begin
	//localparam DICT_START = 53;
	//localparam ERROR_CFA = 11;
	
	//if (reset == 1'b0) begin
		//newest_def = DICT_START;
		//xt_valid <= 1'b0;
		//state = IDLE;
		//count = '0;
		//uart_receive <= 1'b1;
		//xtwp = XTQ_START;
		//xtrp = XTQ_START;
		//nwp = '0;
		//local_xt=0;
		//word_in = '0;
		//cells = '0;
		//cchar = '0;
		//mem_access_outer = 1'b0;
		//comp_state = INTERPRET;
	//end
	//else begin
		//case (state)
			//IDLE : begin

				//xt_valid <= 1'b0;
				//mem_access_outer = 1'b0;
				//dict_write = 1'b0;
				//wp = newest_def;
				//uart_receive <= 1'b1;
				//if (uart_rx_valid) begin
					//mem_access_outer = 1'b1;
					//uart_receive <= 1'b0;
					//byte_in = uart_rx_data;
					//state = PARSE;
				//end
			//end
			//PARSE : begin
				//state = IDLE;
				//if (byte_in == " " || 10 <= byte_in && byte_in <= 13) begin //is character space or between <bl> and <cr>?
					//n_LED_G <= '0;
					//n_LED_B <= '0;
					//n_LED_R <= '0;
					//if (comp_state == COLON) begin
						//comp_state = WORD;
					//end
					//else if (comp_state == WORD) begin
						//comp_state = COMPILING;
						//state = NEW_LINK;
					//end
					//else if (comp_state == END_COMPILE) begin
						//comp_state = INTERPRET;
						//xtrp = xtwp;
					//end
					//else begin
						//state = BEFORE_LINK;
					//end
				//end
				//else if (byte_in == ":") begin
					//comp_state = COLON;
				//end
				//else if (byte_in == ";") begin
					//state = ADD_EXIT;
					//comp_state = END_COMPILE;
				//end
				//else begin
					//cchar++;
					//case (cchar % 4)
						//0 : word_in[cchar / 4][31:24] = byte_in;
						//1 : word_in[cchar / 4][23:16] = byte_in;
						//2 : word_in[cchar / 4][15:8] = byte_in;
						//3 : word_in[cchar / 4][7:0] = byte_in;
						//default :;
					//endcase
					//cells = 1 + (cchar / 4);
					//word_in[0][31:24] = cells;
				//end
			//end
			//BEFORE_LINK : begin
				//++wp;
				//state = GET_LINK;
			//end
			//GET_LINK :  begin
				//link_addr = DataBus;
				//++wp;
				//ccell = '0;
				//state = SEARCH;
			//end
			//SEARCH : begin
				//if (ccell < cells && DataBus == word_in[ccell]) begin
//					 token
					//++wp;
					//++ccell;
				//end
				//else if (ccell >= cells) begin
					//local_xt = -(wp-1);
					//state = COMPILE;
				//end
				//else begin
					//wp = link_addr;
					//state = link_addr ? BEFORE_LINK : NUMBER;
				//end
			//end
			//ADD_EXIT: begin
				//wp = xtwp++;
				//dict_write = 1'b1;
				//dict_wdata = _exit;
				//state = GET_XT;
			//end
			//GET_XT: begin // Demitri 2023 Jan 9: repurposed for adding a delay for XT compilation before execute
				//dict_write = 1'b0;
				//mem_access_outer = 1'b0;
				//state = comp_state == INTERPRET ? EXECUTE : IDLE;
			//end			
			//NUMBER : begin//				 TODO: more than one char numbers
				//if (word_in[0][23:16] inside{["0":"9"]}) begin
					//local_xt = word_in[0][23:16] - "0";
					//wp = xtwp++;
					//dict_write = 1'b1;
					//dict_wdata = _lit;
				//end
				//else begin
					//local_xt = -ERROR_CFA; // CFA for error "word not found"
				//end
				//state = COMPILE;
			//end
			//EXECUTE : begin
				//xt_valid <= 1'b1;
				//if (xt_ready) begin
					//xtwp=xtrp;
					//state = IDLE;
					//uart_receive <= 1'b1;
					//xt_valid <= 1'b0;
				//end
			//end
			//COMPILE: begin
				//wp = xtwp++;	
				//dict_write = 1'b1;
				//dict_wdata = local_xt;
				//local_xt = 0;
				//word_in = '0;
				//cells = '0;
				//cchar = '0;
				//if (byte_in == " ") begin
					//state = IDLE;
					//uart_receive <= 1'b1;
				//end
				//else if (comp_state == INTERPRET) begin
					//state = ADD_EXIT;
				//end
			//end
			//NEW_LINK: begin
				//wp = xtwp++;
				//dict_write = 1'b1;
				//dict_wdata = newest_def;
				//state = NEW_DEF;
				//newest_def = wp;
				//ccell = '0;
			//end
			//NEW_DEF: begin
				//if (ccell < cells) begin
					//wp = xtwp++;
					//dict_write = 1'b1;
					//dict_wdata = word_in[ccell];
					//++ccell;
				//end
				//else begin
					//word_in = '0;
					//cells = '0;
					//cchar = '0;
					//state = IDLE;
				//end
			//end
		//default : state = IDLE;
		//endcase
	//end
//end

	task automatic t_reset;
        dp='0;
        rp='0;
        mp ='0;
		nrp = '0;
        busy = false;
        skip_op = false;
		uart_send <= 1'b0;
	endtask

	task automatic t_Fetch_opcode;
		if (branch) begin
			mp = branch_addr;
		end
		else if (busy == false) begin
			mp++;
		end
	endtask        

	//Execute opcodes task
	task automatic t_execute;
   
      if (skip_op == false && busy == false) begin
		opcode = op_e'(DataBus[code_size:0]);
	  end
	  case (opcode)
        _minus : begin
				--dp;
				data_stack[dp] = data_stack[dp] - data_stack[dp+1];
        end
        
        _plus : begin
		  --dp;
				data_stack[dp] = data_stack[dp] + data_stack[dp+1];
        end
        _one_plus : begin
			++data_stack[dp];
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
			data_stack[dp] = (data_stack[dp] == '0) ? -1 : '0;
		end
		_zero_less_than : begin
			data_stack[dp] = (data_stack[dp] < '0) ? -1 : '0;
		end
		_not : begin
			data_stack[dp] = ~data_stack[dp];
		end
		_negate : begin
			data_stack[dp] = ~data_stack[dp]+1;
		end
		_io_led_on : begin
			//n_LED_G <= data_stack[dp][0];
			//n_LED_B <= data_stack[dp][1];
			//n_LED_R <= data_stack[dp][2];
			n_LED_G <= 1'b1;
			n_LED_B <= 1'b1;
			n_LED_R <= 1'b1;
			
			--dp;
		end
		_io_led_off : begin
			//n_LED_G <= data_stack[dp][0];
			//n_LED_B <= data_stack[dp][1];
			//n_LED_R <= data_stack[dp][2];
			n_LED_G <= 1'b0;
			n_LED_B <= 1'b0;
			n_LED_R <= 1'b0;
			
			--dp;
		end
		
		_lit : begin
			if (busy == false) begin
				busy = true;
			end
			else begin
				++dp;
				data_stack[dp] = DataBus;
				busy = false;
				skip_op = true;
			end
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
			if (busy == false) begin
				busy = true;
			end
			else begin
				branch = (data_stack[dp] == '0) ? -1 : '0;
				--dp;
				branch_addr = DataBus;
				busy = false;
				skip_op = true;
			end
		end
		 _branch : begin
			if (busy == false) begin
				busy = true;
			end
			else begin
				busy = false;
				branch = true;
				branch_addr = DataBus;
			end
        end
		_key : begin
			if (busy == false) begin
				uart_receive <= 1'b1;
				busy = true;
				++dp;
			end
			else if (uart_rx_valid && busy == true) begin
				uart_receive <= 1'b0;
				data_stack[dp][7:0] = uart_rx_data;
				busy = false;
			end
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
		_parse : begin
			if (byte_in == " " || 10 <= byte_in && byte_in <= 13) begin //is character space or between <bl> and <cr>?
				if (comp_state == COLON) begin
					comp_state = WORD;
				end
				else if (comp_state == WORD) begin
					comp_state = COMPILING;
					state = NEW_LINK;
				end
				else if (comp_state == END_COMPILE) begin
					comp_state = INTERPRET;
					xtrp = xtwp;
				end
				else begin
					state = BEFORE_LINK;
				end
			end
			else if (byte_in == ":") begin
				comp_state = COLON;
			end
			else if (byte_in == ";") begin
				state = ADD_EXIT;
				comp_state = END_COMPILE;
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
			end
		end
		_exit : begin
			branch_addr = return_stack[rp];
			branch = true;
			skip_op = true;
			rp--;
		end
		default: begin
			branch_addr = -opcode;
			branch = true;
			rp++;
			return_stack[rp] = mp;
		end
	  endcase
      
  endtask : t_execute
 
//Forth Inner Interpreter (Fetch/Execute Unit)
always_ff @(posedge clk) begin

    if (reset == 1'b0) begin
        t_reset;
		n_LED_R <= 1'b0;
		n_LED_B <= 1'b0;
		n_LED_G <= 1'b0;
    end    
    else begin
		skip_op = skip_op || branch;
		branch = false;
		if (skip_op == false) begin
			t_execute;
		end
		else begin
			skip_op = false;
		end
		t_Fetch_opcode;
        n_BUTTON0 <= BUTTON0;
        n_BUTTON1 <= BUTTON1;
    end 
end

//----------------------------------------------------------------------------
//                                                                          --
//                       Instantiate RGB primitive                          --
//                                                                          --
//----------------------------------------------------------------------------
  RGB RGB_DRIVER (
    .RGBLEDEN(1'b1),
    .RGB0PWM (n_LED_G),
    .RGB1PWM (n_LED_B),
    .RGB2PWM (n_LED_R),
    .CURREN  (1'b1),
    .RGB0    (LED_G), //Actual Hardware connection
    .RGB1    (LED_B),
    .RGB2    (LED_R)
  );
  defparam RGB_DRIVER.RGB0_CURRENT = "0b000001";
  defparam RGB_DRIVER.RGB1_CURRENT = "0b000001";
  defparam RGB_DRIVER.RGB2_CURRENT = "0b000001";

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
