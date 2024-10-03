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
parameter Int_SRAM_size = 256;
parameter Int_SRAM_ADDR_size = 31;

//LSRAM2
parameter data_width = 9;
parameter address_width = 10;
parameter ext_ram_size = 1024;

//Define System Architecture
typedef enum logic [15:0]{

//System Operators
_execute, _abort, _end_boot,  _run_boot, _create, _does, _compile, _bracket_compile, _interpret, _do_colon, _exit,

//Stack Operators
_depth, _dup, _pick, _over, _swap, _rot, _equal, _zero_equal, _greater_than, _less_than,
_question_dup, _drop, _roll, _to_return, _from_return, _copy_return,
_lit,

//Memory Operators              
_cstore, _cfetch,  _store, _fetch, _cmove, _fill,
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
_key, _emit, _atoi,

//I/O Processing                
_io_led, _io_button, _io_fetch, _io_store

} op_e;

typedef struct packed {
    logic is_cfa;
	logic is_token;
	logic [DataWidthSystem:0] data;
} cell_t;

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

output logic LED_G,
output logic LED_B,
output logic LED_R
);

logic n_main_clk;
logic [3:0] active_mem;//which memory is active? ROM, INTSRAM or EXSRAM    
logic [3:0] errorcode ;
logic	[address_size:0] mp;      // memory pointer
logic	[1:0] successful;
op_e opcode;

//Circular stacks
logic [data_size:0] top_data_stack;
logic [data_size:0] data_stack[data_stack_depth]; /* synthesis syn_ramstyle="block_ram" */;
logic [data_size:0] return_stack[return_stack_depth]; /* synthesis syn_ramstyle="block_ram" */;
logic [$clog2(data_stack_depth)-1:0] dp;
logic [$clog2(data_stack_depth)-1:0] rp;

logic boot_enable;
logic [data_size:0] bootROM [200:0]; /* synthesis syn_ramstyle="block_ram" */;
//Lattice Internal memory
logic [data_size:0] [Int_SRAM_ADDR_size:0] mem; /* synthesis syn_ramstyle="block_ram" */;
 
//External Memory
cell_t DataBus;
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
logic [7:0] tx_data_outer;
logic [7:0] tx_data_inner;
logic [7:0] tx_data;
logic uart_busy_tx;
logic uart_send_outer;
logic uart_send_inner;
logic uart_send;
logic uart_busy_rx;
logic uart_receive;
logic uart_rx_valid;
logic [7:0] uart_rx_data;
assign tx_data = tx_data_outer | tx_data_inner;
assign uart_send = uart_send_outer | uart_send_inner;

// Execution Token
localparam XTQ_START = 200;
logic [address_size:0] xtrp;
logic [address_size:0] xtwp;
logic xt_valid;
logic xt_ready;

// Dictionary
logic dict_write;
logic mem_access_outer;
logic [address_size:0] mem_addr;
logic [Int_SRAM_ADDR_size:0] dict_wdata;
logic [address_size:0] wp; // dictionary pointer wp

// Power on Reset
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

//Demetri: can you change the Outer Interpreter code to use this RAM based dictionary?
//build dictionary for testing...

logic [address_size:0] mem_diff;
assign boot_enable = (mem_access_outer ? wp : mp) < XTQ_START;
// memory manager
always_ff @(posedge clk or negedge reset) begin
	if (reset == 1'b0) begin
	end
	else begin
		mem_diff = boot_enable ? '0 : -XTQ_START;
		mem_addr = (mem_access_outer ? wp : mp)+mem_diff;
		if (dict_write) begin
			mem[mem_addr] <= dict_wdata;
		end
	end
end
assign DataBus.data = boot_enable ? bootROM[mem_addr] : mem[mem_addr];

	logic [7:0] byte_in;
	logic [31:0] link_addr;
	logic [31:0] newest_def;
	logic [7:0] ccell, cchar, cells;
	logic [3:0][31:0] word_in;
	cell_t local_xt;
	logic [3:0] count ;
	enum logic [3:0] {IDLE, PARSE, SEARCH, BEFORE_LINK, GET_LINK, GET_XT, ADD_EXIT, EXECUTE, NUMBER, COMPILE,
	NEW_LINK, NEW_DEF, MEM_OP, LOAD, MEM_OP_DONE} state, i_state;
	enum logic [3:0]{INTERPRET, COLON, WORD, COMPILING, END_COMPILE} comp_state;

	logic i_data_req = 1'b0; // inner interpreter sends
	logic i_data_gnt = 1'b0; // outer interpeter sends
	enum {FETCH, STORE, I} i_data_op;
	cell_t [data_size:0] i_data;
	cell_t [address_size:0] i_data_addr;
	cell_t [data_size:0] o_rdata;
	logic i_state_valid;
	

// Forth Outer Interpreter
always_ff @(posedge clk) begin
	localparam DICT_START = 71;
	localparam ERROR_CFA = 11;
	
	if (reset == 1'b0) begin
		newest_def = DICT_START;
		xt_valid <= 1'b0;
		state = IDLE;
		count = '0;
		uart_receive <= 1'b1;
		uart_send_outer <= 1'b0;
		tx_data_outer <= '0;
		xtwp = XTQ_START;
		xtrp = XTQ_START;
		local_xt=0;
		word_in = '0;
		cells = '0;
		cchar = '0;
		mem_access_outer = 1'b0;
		comp_state = INTERPRET;
		i_data_gnt <= 1'b0;
	end
	else begin
		case (state)
			IDLE : begin
				xt_valid <= 1'b0;
				mem_access_outer = 1'b0;
				dict_write = 1'b0;
				i_data_gnt <= 1'b0;
				wp = newest_def;
				uart_receive <= 1'b1;
				if (uart_rx_valid) begin
					mem_access_outer = 1'b1;
					uart_receive <= 1'b0;
					byte_in = uart_rx_data;
					uart_send_outer <= 1'b1;
					tx_data_outer   <= byte_in;
					state = PARSE;
				end
				else begin 
					if (!uart_busy_tx) begin
						uart_send_outer <= 1'b0;
						tx_data_outer <= '0;
					end
					if (i_data_req) begin
						wp = i_data_addr;
						mem_access_outer = 1'b1;
						state = MEM_OP;
					end
					if (i_state_valid) begin
						state = i_state;
					end
				end
			end
			PARSE : begin
				state = IDLE;
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
			BEFORE_LINK : begin
				++wp;
				state = GET_LINK;
			end
			GET_LINK :  begin
				link_addr = DataBus.data;
				++wp;
				ccell = '0;
				state = SEARCH;
			end
			SEARCH : begin
				if (ccell < cells && DataBus.data == word_in[ccell]) begin
					// token
					++wp;
					++ccell;
				end
				else if (ccell >= cells) begin
					local_xt = -(wp-1);
					state = COMPILE;
				end
				else begin
					wp = link_addr;
					state = link_addr ? BEFORE_LINK : NUMBER;
				end
			end
			ADD_EXIT: begin
				wp = xtwp++;
				dict_write = 1'b1;
				dict_wdata = _exit;
				state = GET_XT;
			end
			GET_XT: begin // Demitri 2023 Jan 9: repurposed for adding a delay for XT compilation before execute
				dict_write = 1'b0;
				mem_access_outer = 1'b0;
				state = comp_state == INTERPRET ? EXECUTE : IDLE;
			end			
			NUMBER : begin
				// TODO: more than one char numbers
				if (word_in[0][23:16] inside{["0":"9"]}) begin
					local_xt = word_in[0][23:16] - "0";
					wp = xtwp++;
					dict_write = 1'b1;
					dict_wdata = _lit;
				end
				else begin
					local_xt = -ERROR_CFA; // CFA for error "word not found"
				end
				state = COMPILE;
			end
			EXECUTE : begin
				xt_valid <= 1'b1;
				if (xt_ready) begin
					xtwp=xtrp;
					state = IDLE;
					uart_receive <= 1'b1;
					xt_valid <= 1'b0;
				end
			end
			COMPILE: begin
				wp = xtwp++;	
				dict_write = 1'b1;
				dict_wdata = local_xt;
				local_xt = 0;
				word_in = '0;
				cells = '0;
				cchar = '0;
				if (byte_in == " ") begin
					state = IDLE;
					uart_receive <= 1'b1;
				end
				else if (comp_state == INTERPRET) begin
					state = ADD_EXIT;
				end
			end
			NEW_LINK: begin // "CREATE"
				wp = xtwp++;
				dict_write = 1'b1;
				dict_wdata = newest_def;
				state = NEW_DEF;
				newest_def = wp;
				ccell = '0;
			end
			NEW_DEF: begin
				if (ccell < cells) begin
					wp = xtwp++;
					dict_write = 1'b1;
					dict_wdata = word_in[ccell];
					++ccell;
				end
				else begin
					word_in = '0;
					cells = '0;
					cchar = '0;
					state = IDLE;
				end
			end
			MEM_OP : begin
				if (i_data_op == STORE) begin // STORE
					dict_write = 1'b1;
					dict_wdata = i_data;
					i_data_gnt <= 1'b1;
					state = IDLE;
				end
				else if (i_data_op == FETCH) begin
					state = LOAD;
				end
			end
			LOAD : begin
				o_rdata = DataBus.data;
				i_data_gnt <= 1'b1;
				state = MEM_OP_DONE;
				mem_access_outer = 1'b0;
			end
			MEM_OP_DONE : begin // Need to create a delay to allow proper execution
				i_data_gnt <= 1'b0;
				state = IDLE;
			end
		default : state = IDLE;
		endcase
	end
end

	task automatic t_reset;
        dp='0;
        rp='0;
        mp ='0;
        busy = false;
        skip_op = false;
		uart_send_inner <= 1'b0;
		tx_data_inner <= '0;
		i_data_req <= 1'b0;
	endtask
	
	task automatic t_push(logic [data_size:0] x);
		++dp;
		top_data_stack <= x;
		data_stack[dp] <= top_data_stack;
	endtask
	
	task automatic t_pop;
		top_data_stack <= data_stack[dp];
		--dp;
	endtask
	
	task automatic t_Fetch_opcode;
		if (branch) begin
			mp = branch_addr;
		end
		else if (busy == false) begin
			++mp;
		end
	endtask        

function bit [7:0] int_to_ascii_str(input int num);
  int temp_num = num;
  int digit;

  if (temp_num == 0) begin
    int_to_ascii_str = "0";
  end else begin
	/*
    while (temp_num > 0) begin
      digit = temp_num % 10; // Get the last digit
      result = {string'(digit + 48),result}; // Convert to ASCII and prepend
      temp_num = temp_num / 10; // Remove the last digit
    end
	*/
	digit = temp_num % 10;
	int_to_ascii_str = digit + 48;
  end
endfunction

	//Execute opcodes task
	task automatic t_execute;
   
      if (skip_op == false && busy == false) begin
		opcode = op_e'(DataBus.data);
	  end
	  case (opcode)
		_store : begin
			if (busy == false) begin
				busy = true;
				i_data_addr <= top_data_stack;
				top_data_stack <= data_stack[dp];
				--dp;
			end
			else begin
				i_data_req <= 1'b1;
				i_data <= top_data_stack;
				i_data_op <= STORE;
				if (i_data_gnt == 1'b1) begin
					top_data_stack <= data_stack[dp];
					i_data_req <= 1'b0;
					--dp;
					busy = false;
				end
			end
		end
		_fetch : begin
			if (busy == false) begin
				busy = true;
				i_data_req <= 1'b1;
				i_data_op <= FETCH;
				i_data_addr <= top_data_stack;
			end
			else begin
				if (i_data_gnt == 1'b1) begin
					i_data_req <= 1'b0;
					top_data_stack <= o_rdata;
					busy = false;
				end
			end
		end
        _minus : begin
			top_data_stack <= top_data_stack - data_stack[dp];
			--dp;
        end
        
        _plus : begin
			top_data_stack <= top_data_stack + data_stack[dp];
			--dp;
        end
        _one_plus : begin
			top_data_stack <= top_data_stack + 1;
		end			
		_dup : begin
			++dp;
			data_stack[dp] <= top_data_stack;
			
		end
		_over : begin
			++dp;
			data_stack[dp] <= top_data_stack;
			top_data_stack <= data_stack[dp-1];
		end
		_drop : begin
			top_data_stack <= data_stack[dp];
			--dp;
		end
		_equal : begin
			top_data_stack <= top_data_stack == data_stack[dp] ? -1 : 0;
			--dp;
		end
		_zero_equal : begin
			top_data_stack <= (top_data_stack == '0) ? -1 : '0;
		end
		_zero_less_than : begin
			top_data_stack <= (top_data_stack < '0) ? -1 : '0;
		end
		_not : begin
			top_data_stack <= ~top_data_stack;
		end
		_negate : begin
			top_data_stack <= ~top_data_stack+1;
		end
		_and : begin
			top_data_stack <= top_data_stack & data_stack[dp];
			--dp;
		end
		_or : begin
			top_data_stack <= top_data_stack | data_stack[dp];
			--dp;
		end
		_atoi : begin
			top_data_stack <= 48 + top_data_stack;
		end
		_io_led : begin
			n_LED_G <= top_data_stack[0];
			n_LED_B <= top_data_stack[1];
			n_LED_R <= top_data_stack[2];
			top_data_stack <= data_stack[dp];
			--dp;	
		end
		_io_button : begin
			++dp;
            top_data_stack <= {BUTTON1,BUTTON0};
			data_stack[dp] <= top_data_stack;
		end
		_lit : begin
			if (busy == false) begin
				busy = true;
			end
			else begin
				++dp;
				data_stack[dp] <= top_data_stack;
				top_data_stack <= DataBus.data;
				busy = false;
				skip_op = true;
			end
		end
		_0branch : begin
			if (busy == false) begin
				busy = true;
			end
			else begin
				branch = (top_data_stack == '0) ? -1 : '0;
				top_data_stack <= data_stack[dp];
				--dp;
				branch_addr = DataBus.data;
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
				branch_addr = DataBus.data;
			end
        end    
        _emit : begin
			if (busy == false) begin
				busy = true;
			end
			else if (!uart_busy_tx && busy == true) begin
				if (!uart_send_inner) begin
					uart_send_inner <= 1'b1;
					tx_data_inner <= top_data_stack[7:0]; 
				end
				else begin
					uart_send_inner <= 1'b0;
					tx_data_inner <= '0;
					top_data_stack <= data_stack[dp];
					--dp;
					busy = false;
				end
			end
        end        
		_execute : begin
			busy = true;
			xt_ready = 1'b0;
			if (xt_valid) begin
				branch_addr = xtrp;
				branch = true;
				busy = false;
				skip_op = true;
				rp++;
				return_stack[rp] = mp;
				xt_ready = 1'b1;
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

//Forth Inner Interpreter (Fetch/Execute Unit)
always_ff @(posedge clk) begin

    if (reset == 1'b0) begin
        t_reset;
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

initial begin
	bootROM[0] =  _execute;
	bootROM[1] =  _branch;
	bootROM[2] =  0;
	bootROM[3] =  1;
	bootROM[4] =  2;
	bootROM[5] =  3;
	bootROM[6] =  4;
	bootROM[7] =  5;
	bootROM[8] =  6;
	bootROM[9] = 7;
	bootROM[11] =  _lit;
	bootROM[12] =  "?";
	bootROM[13] =  _emit;
	bootROM[14] = _exit;
	bootROM[15] = 0; 					// Link to next dictionary entry
	bootROM[16] = {8'd1,"l","e","d"};	// Name field (NFA)
	bootROM[17] = _io_led;				// code field
	bootROM[18] = _exit;					// end of code field
	bootROM[19] = 15;
	bootROM[20] = {8'd1,"r","e","d"};				
	bootROM[21] = _lit;
	bootROM[22] = 4;
	bootROM[23] = -17;
	bootROM[24] = _exit;				
	bootROM[25] = 19;
	bootROM[26] = {8'd2,"g","r","e"};
	bootROM[27] = {"e","n","\0","\0"};
	bootROM[28] = _lit;
	bootROM[29] = 1;
	bootROM[30] = -17;
	bootROM[31] = _exit;
	bootROM[32] = 25;
	bootROM[33] = {8'd2,"b","l","u"};
	bootROM[34] = {"e","\0","\0","\0"};
	bootROM[35] = _lit;
	bootROM[36] = 2;
	bootROM[37] = -17;
	bootROM[38] = _exit;
	bootROM[39] = 32;
	bootROM[40] = {8'd2,"d","e","l"};
	bootROM[41] = {"a","y","\0","\0"};
	bootROM[42] = _lit;
	bootROM[43] = 5000000;
	bootROM[44] = _lit;
	bootROM[45] = 1;
	bootROM[46] = _minus;
	bootROM[47] = _dup;
	bootROM[48] = _zero_equal;
	bootROM[49] = _0branch;
	bootROM[50] = 44;
	bootROM[51] = _drop;
	bootROM[52] = _exit;
	bootROM[53] = 39;
	bootROM[54] = {8'd1,"!","\0","\0"};
	bootROM[55] = _store;
	bootROM[56] = _exit;
	bootROM[57] = 53;
	bootROM[58] = {8'd1,"@","\0","\0"};
	bootROM[59] = _fetch;
	bootROM[60] = _exit;
	bootROM[61] = 57;
	bootROM[62] = {8'd1,"x","\0","\0"};
	bootROM[63] = _lit;
	bootROM[64] = 250;
	bootROM[65] = _exit;
	bootROM[66] = 61;
	bootROM[67] = {8'd1,"y","\0","\0"};
	bootROM[68] = _lit;
	bootROM[69] = 251;
	bootROM[70] = _exit;
	bootROM[71] = 66;
	bootROM[72] = {8'd1,".","\0","\0"};
	bootROM[73] = _atoi;
	bootROM[74] = _emit;
	bootROM[75] = _lit;
	bootROM[76] = 32;
	bootROM[77] = _emit;
	bootROM[78] = _exit;
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
