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
//Error Codes
dstack_overflow, dstack_underflow, rstack_overflow, rstack_underflow, invalid_instruction, no_errors,
//Communications
_key, _emit,
//I/O Processing                
_io_led, _io_button, _io_fetch, _io_store
} op_e;
typedef struct packed {
  logic is_immediate;
  op_e op;
} opcode_t;
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
//Circuliar stacks
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
logic [data_size:0] DataStackDepth;
logic [data_size:0] ReturnStackDepth;
logic [data_size:0] Temp;
logic [data_size:0] Here;
logic [data_size:0] Base;
logic [data_size:0] State;
logic [4:0] data_Stack_Op;
logic busy;
logic skip_op;
logic branch;
logic [address_size:0] branch_addr;
//Loop registers?
//logic [data_size:0] I;
//logic [data_size:0] J;
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
logic [data_size:0] number;
//Boot code when the processor starts...
task automatic t_init_boot_code;
	boot_ROM[0] <=  _execute;
	boot_ROM[1] <=  _lit;
	boot_ROM[2] <=  63;
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
endtask : t_init_boot_code 
//Demetri: can you change the Outer Interpreter code to use this RAM based dictionary?
//build dictionary for testing...
task automatic t_init_dictionary_code;
	mem[0] <= 3;
	mem[1] <= {8'd3,"r","e","d"};
	mem[1] <= {8'd1,"r","e","d"};
	mem[2] <= 6;
	mem[3] <= 6;
	mem[4] <= {8'd5,"g","r","e"};
	mem[5] <= 11;
	mem[6] <= 9;
	mem[7] <= {8'd4,"b","l","u"};
	mem[8] <= 16;
	mem[9] <= 0;
	mem[10] <= {8'd5,"d","e","l"};
	mem[11] <= 27;
	mem[12] <= 0;
	mem[3] <= 7;
	mem[4] <= {8'd2,"g","r","e"};
	mem[5] <= {"e","n","\0","\0"};
	mem[6] <= 11;
	mem[7] <= 11;
	mem[8] <= {8'd2,"b","l","u"};
	mem[9] <= {"e","\0","\0","\0"};
	mem[10] <= 16;
	mem[11] <= 0;
	mem[12] <= {8'd2,"d","e","l"};
	mem[13] <= {"a","y","\0","\0"};
	mem[14] <= 27;
	mem[15] <= 0;
endtask : t_init_dictionary_code

// Forth Outer Interpreter
always_ff @(posedge clk) begin
	logic [7:0] byte_in;
	logic [31:0] wp;
	logic [31:0] wp, ccell, cchar, cells;
	logic [31:0] link_addr;
	logic [31:0] dict_size;
	logic [31:0] word_in;
	logic [3:0][31:0] word_in;
	logic [address_size:0] local_xt;
	enum {IDLE, PARSE, SEARCH, GET_LINK, GET_XT, EXECUTE, NUMBER, COMPILE} state;

	if (reset == 1'b0) begin
		wp = '0;
		xt_valid <= 1'b0;
		dict_size = 8;
		state = IDLE;
		uart_receive <= 1'b1;
		xtwp = 0;
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
					if (word_in[31:24] < 3) begin//is word count [31:24] less than 3?
						case (word_in[31:24])
							0 : word_in[23:16] = byte_in;
							1 : word_in[15:8] = byte_in;
							2 : word_in[7:0] = byte_in;
							default :;
						endcase
					end
					++word_in[31:24];
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
				if (mem[wp] == word_in) begin
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
				if (byte_in inside{["0":"9"]}) begin
					number = byte_in - "0";
					local_xt = 21;
				end
				else begin
					local_xt = 1;
				end
				state = IDLE;
				uart_receive <= 1'b1;
			end
			EXECUTE : begin
				xt_valid <= (xtrp < xtwp);
				if (xtrp == xtwp) begin
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
//assign second = data_stack[dp];//dON 10-13-21
task automatic t_reset;
    begin
        t_init_boot_code;
        errorcode=no_errors;
        bp='0;
        dp='0;
        rp='0;
        mp ='0;
		xtrp = 0;
        busy <= false;
        active_mem <= _ROM_active;
        DataStackDepth='0;
        ReturnStackDepth='0;
        skip_op <= false;
		uart_send <= 1'b0;	
		t_init_dictionary_code;
    end
  endtask
//Execute Opcodes task
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
		// Demitri Peynado 30th Sept 2021
		_io_led : begin
			//Don 12/7/2021 testing leds 
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
		_if : begin
			--dp;
        end    
        _else : begin
			--dp;        
		end
        _then : begin
			--dp;        
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
			busy = true;
			if (xt_valid) begin
				branch_addr = xt[xtrp++];
				busy = false;
				branch = true;
			end
		end
		_number : begin
			++dp;
			data_stack[dp] = number;
		end
		default : ;
	  endcase
      
  endtask : t_execute
 //Execute Opcodes task
   //task automatic t_boot_to_SRAM; 
//
    //LSRAM_WEN <= low;
    //++SRAM_CTR; 
    //LSRAM_DATA_OUT <= boot_ROM[SRAM_CTR];
    //LSRAM_WADDR <= SRAM_CTR;
    //LSRAM_WEN <= high;
    //
    ////LSRAM_RADDR <= 'd0;
    ////LSRAM_REN <= low;
//
    ////S_DATA <= 'd0;  
    ////SRAM_CTR <= 'd0; 
    ////SRAM_over_flow <= false; 
    ////SRAM_RW <= high;
    ////mp <= 'd0;
  //
    //endtask : t_boot_to_SRAM
  
  
//  assign n_main_clk = main_clk;
  
//Forth Inner Intrepreter
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

//Do we need to instantiate any of this Lattice stuff?
//pmi_complex_mult 
//#(
  //.pmi_dataa_width         ( ), // integer
  //.pmi_datab_width         ( ), // integer
  //.pmi_sign                ( ), // "on"|"off"
  //.pmi_additional_pipeline ( ), // integer
  //.pmi_input_reg           ( ), // "on"|"off"
  //.pmi_output_reg          ( ), // "on"|"off"
  //.pmi_family              ( ), // "iCE40UP" | "common"
  //.pmi_implementation      ( )  // "DSP"|"LUT"
//) <your_inst_label> (
  //.DataA_Re  ( ),  // I:
  //.DataA_Im  ( ),  // I:
  //.DataB_Re  ( ),  // I:
  //.DataB_Im  ( ),  // I:
  //.Clock     ( ),  // I:
  //.ClkEn     ( ),  // I:
  //.Aclr      ( ),  // I:
  //.Result_Re ( ),  // O:
  //.Result_Im ( )   // O:
//);
//pmi_multaddsub
//#(
  //.pmi_dataa_width         ( ), // integer
  //.pmi_datab_width         ( ), // integer
  //.pmi_sign                ( ), // "on"|"off"
  //.pmi_additional_pipeline ( ), // integer
  //.pmi_add_sub             ( ), // "add"|"sub"
  //.pmi_input_reg           ( ), // "on"|"off"
  //.pmi_output_reg          ( ), // "on"|"off"
  //.pmi_family              ( ), // "iCE40UP" | "common"
  //.pmi_implementation      ( )  // "DSP"|"LUT"
//) <your_inst_label> (
  //.DataA0 ( ),  // I:
  //.DataA1 ( ),  // I:
  //.DataB0 ( ),  // I:
  //.DataB1 ( ),  // I:
  //.Clock  ( ),  // I:
  //.ClkEn  ( ),  // I:
  //.Aclr   ( ),  // I:
  //.Result ( )   // O:
//);
//pmi_ram_dp
//#(
  //.pmi_wr_addr_depth    ( ), // integer
  //.pmi_wr_addr_width    ( ), // integer
  //.pmi_wr_data_width    ( ), // integer
  //.pmi_rd_addr_depth    ( ), // integer
  //.pmi_rd_addr_width    ( ), // integer
  //.pmi_rd_data_width    ( ), // integer
  //.pmi_regmode          ( ), // "reg"|"noreg"
  //.pmi_resetmode        ( ), // "async"|"sync"
  //.pmi_init_file        ( ), // string
  //.pmi_init_file_format ( ), // "binary"|"hex"
  //.pmi_family           ( )  // "iCE40UP"|"common"
//) <your_inst_label> (
  //.Data      ( ),  // I:
  //.WrAddress ( ),  // I:
  //.RdAddress ( ),  // I:
  //.WrClock   ( ),  // I:
  //.RdClock   ( ),  // I:
  //.WrClockEn ( ),  // I:
  //.RdClockEn ( ),  // I:
  //.WE        ( ),  // I:
  //.Reset     ( ),  // I:
  //.Q         ( )   // O:
//);
//pmi_rom 
//#(
	//.pmi_addr_depth       ( ), // integer       
    //.pmi_addr_width       ( ), // integer       
    //.pmi_data_width       ( ), // integer       
    //.pmi_regmode          ( ), // "reg"|"noreg"
    //.pmi_resetmode        ( ), // "async" | "sync"	
    //.pmi_init_file        ( ), // string		
    //.pmi_init_file_format ( ), // "binary"|"hex"    
	//.pmi_family           ( )  // "common"
//) <your_inst_label> (
	//.Address    ( ),  // I:
	//.OutClock   ( ),  // I:
	//.OutClockEn ( ),  // I:
	//.Reset      ( ),  // I:
	//.Q          ( )   // O:
//);
    //Outer Interpreter
//always_ff  @(posedge clk) begin
  //if (reset == 0) begin
        //char_buf_rd_ptr <= 0;
    //end    
    //else begin
        //valid <= false;
    //end
        //
    //if (UARTread && char_buf_rd_ptr-1 ) begin
    //
        //rdata <= char_buf[char_buf_rd_ptr];
        //char_buf_rd_ptr <= char_buf_rd_ptr + 1;
        //valid <=  true;
    //end
//end
//I need to work on this a bit, I need to add UART control lines, wdata, etc.
//always_ff  @(posedge clk) begin
  //if (reset == 0) begin
    //char_buf_rd_ptr <= 0;
    //BAUD_VAL <= 'd434;// 50mhz/115,200 = 434 (115,200 baud)
    //CSN <= low; //enable UART
    //WEN <= high;
    //OEN <= high;
  //end
    
  //else begin
    //if(TXRDY == true)
        //DATA_OUT <= wdata;
    //end
    
    //if(RXRDY == true) begin
      //rdata <= char_buf[char_buf_rd_ptr];
      //char_buf_rd_ptr <= char_buf_rd_ptr + 1;
    //end
  //end
//Read and Write to SRAM test
//always_ff  @(posedge clk || reset) begin
  //
  //if (reset == 0) begin
//
    //LSRAM_DATA_OUT <= 'd0;
    //LSRAM_WADDR <= 'd0;
    //LSRAM_RADDR <= 'd0;
    //LSRAM_REN <= low;
    //LSRAM_WEN <= low;
    //S_DATA <= 'd0;  
    //SRAM_CTR <= 'd0; 
    //SRAM_over_flow <= false; 
    //SRAM_RW <= high;
    //mp = 'd0;
   //end
    //
  //else begin
    //if (SRAM_RW == high) begin       
        //LSRAM_WADDR <= 'd0;
        //LSRAM_DATA_OUT <= 'hff; 
        //LSRAM_WEN <= high;         
        //SRAM_RW <= low; 
        //LSRAM_WEN <= low;        
    //end 
    //
    //else begin
   //
        //LSRAM_RADDR <= 'd1;
        //LSRAM_REN <= high;         
        //S_DATA <= LSRAM_DATA_IN;
        //LSRAM_REN <= low;
        //SRAM_RW <= high;         
    //end
  //end
//end
//always @(posedge clk)
  //begin
    //din <='d100;
    //
    //en <= high;
    //we <= high;
    //
    //if (en == high) begin
        //if(we == high) begin
            //mem[addr] <= din;
                //we <= low;
        //end
        //
        //else begin
            //dout <= mem[addr];
            //we <= high;
        //end    
        //end
        //en <= low;    
    //end 
//Booting processor
  //always_ff @(posedge clk)
    //begin
      //if (reset == 0) begin
        //SRAM_CTR <= 0;
        //active_mem <= _ROM_active; 
        //boot_flag <= true;
      //end
    
      //else begin
   
        //LSRAM_WEN <= high;
        //LSRAM_WADDR <= SRAM_CTR;  
        //LSRAM_DATA_OUT <= boot_ROM[SRAM_CTR];
    
            //if (SRAM_CTR >= end_boot_ROM) begin
                //active_mem <= _SRAM_active;
                    //boot_flag <= false; 
            //end        
            //else begin
                //active_mem <= _ROM_active; 
                  //boot_flag <= true;    
            //end
         //++SRAM_CTR; 
      //end         
    //end
  
//logic	[31:0] S_DATA;
//logic   S_RW;
//logic [Int_SRAM_size-1:0] SRAM_CTR;
//logic SRAM_over_flow;  
  ////Read to SRAM
  //always_ff  @(posedge clk) begin
//
  //if (reset == 0) begin
 ////       S_RW <= low;
        //C_DIN <= 'd0;
        //C_ADDR <= 'd0;
        //C_BLK <= low;//before C_WEN    
        //C_WEN <= low; 
   //end 
    //else begin  
        //if(S_RW == low) begin
            //A_ADDR <= 'd0;
            //S_DATA <= A_DOUT;
        //end
    //end   
  //end  
  
  
  
//endmodule
//module ram (WAddress, RAddress, Data, WClock, WE,
 //RE, Rclock, Q);
//input [8:0] WAddress, RAddress;
//input [31:0] Data;
//input Rclock, WClock;
//input WE, RE;
//output [31:0] Q;
//ram R_32_16 (.Data(Data), .WE(WE), .RE(RE), .WClock(WClock),
//.Rclock(Rclock), .Q(Q), .WAddress(WAddress),
//.RAddress(RAddress));
//endmodule
//Ideas code to implement later
//Masks
//Parsed numbers (LITERALS in Forth) will not require a compiled LITERAL token but
// parsed WORDS will be identified as the most significant bit will denote an
// executable WORD.  The second most significant bit will denote an IMMEDIATE
// word to be executed during compliation.
//for example...
// expand databus width from 31 to 33 bits wide
//   1111111111111111 32 bit word
// 001111111111111111 is a LITERAL
// 000011111111111111 is a LITERAL
// 100000000000101010 is an executable WORD (token) like CFA
// 110000000000101010 is an executable WORD as above plus with the IMMEDIATE bit set
  //immediateMASK     <= 31'b010000000000000,
  //OpCodeMASK        <= 31'b100000000000000
  
//module memory(
//input logic clk ,
//input logic write
//Write enable
//write ,
//input logic [3:0] address
//4-bit address
//address ,
//input logic [7:0] data_in 8-bit input bus data_in ,
//output logic [7:0] data_out); data_out 8-bit output bus
//logic [7:0] mem
//The memory array: 16 8-bit bytes
//mem [15:0];
//always_ff @(posedge clk
//Clocked
//posedge clk)
//begin
//if (write)
//mem[address] <= data_in
//Write to array when asked
//data_in;
//data_out <= mem[address]
//Always read (old) value from array
//mem[address];
//end
//endmodule
//logic [data_size:0] mem [memory_size:0];
//mem[address] <= data_in;
//data_out <= mem[address];
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
//  By editing the code in the Entity declariations, you can add serial ports, parallel
//  ports, adc's or just about anything you can imagine.
//# Peter Jackie RISC V MAP Forth functions to logicisters
//.eqv ip tp # Instruction ptr
//.eqv rp s0 # Return thread stack ptr
//.eqv lp s1 # Loop stack ptr
//.eqv dp gp # data stack ptr
//.eqv xx t0 # main working logicisters
//.eqv yy t1
//.eqv zz t2
//.eqv aa a0 # top of data stack
//.eqv bb a1 # second item
//.eqv cc a2 # third item
//.eqv dd a3 # fourth item
//.eqv index a4 # top of loop stack parameters
//.eqv limit a5 # loop limit or count
//.eqv loopip a6 # loop address
//.eqv threaded s10 #constant for decoding address
//4?d0: y=a+b;
//4?d1: y=a-b;
//4?d2: y=a*b;
//4?d3: y={4? bww, ~a};
//4?d4: y={4? d0, (a & b)};
//4?d5: y={4? d0, (a | b)};
//4?d6: y={4? d0, (a ^ b)};
//4?d7: y={4? d0, ~(a & b)};
//4?d8: y={4? d0, ~(a | b)};
//4?d9: y={4? d0, ~(a ^ b)};
//
//bit [MEM_WIDTH-1:0] mem_data; // memory might by
//
//mem_data <= 'b1010;           
//
//opcode_t ins <= opcode_t'(mem_data);
//
 //
//
//if (ins.is_immediate) begin
// Execute
//end
//
//else
//
//begin
//
//// Compile
//
//End