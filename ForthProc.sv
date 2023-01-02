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
_plus, _minus, _times, _divide, _max, _min, _times_mod, _divide_mod, _times_divide, _one_plus, _one_minus, _negate,

//Conditional
_if, _else,_then, _0branch, _branch,

//Loops
_begin, _again, _until, _for, _next,

//Logical
_and, _or, _xor, _not, _zero_less_than, _zero_greater_than, _zero_equals,

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

logic n_main_clk;
logic [3:0] active_mem;//which memory is active? ROM, INTSRAM or EXSRAM    
logic [3:0] errorcode ; 
logic	[address_size:0] mp;      // memory pointer
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

logic boot_enable;
logic [data_size:0] bootROM [200:0];
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
logic [7:0] xtrp;
logic [7:0] xtwp;
logic xt_valid;
logic xt_ready;

logic [data_size:0] number[256];
logic [7:0] nrp;
logic [7:0] nwp;

// Dictionary
logic dict_write;
logic mem_access_outer;
logic [address_size:0] mem_addr;
logic [Int_SRAM_ADDR_size:0] dict_wdata;
logic [address_size:0] wp; // dictionary pointer wp

//Demetri: can you change the Outer Interpreter code to use this RAM based dictionary?
//build dictionary for testing...
task automatic t_init_dictionary_code;
	bootROM[0] <=  _execute;
	bootROM[1] <=  _lit;
	bootROM[2] <=  "?";
	bootROM[3] <=  _emit;
	bootROM[4] <= _branch;
	bootROM[5] <= 0;
	bootROM[6] <=  _lit;
	bootROM[7] <= 4;
	bootROM[8] <= _io_led;
	bootROM[9] <= _branch;
	bootROM[10] <= 0;
	bootROM[11] <=  _lit;
	bootROM[12] <= 1;
	bootROM[13] <= _io_led;
	bootROM[14] <= _branch;
	bootROM[15] <= 0;
	bootROM[16] <=  _lit;
	bootROM[17] <= 2;
	bootROM[18] <= _io_led;
	bootROM[19] <= _branch;
	bootROM[20] <= 0;
	bootROM[21] <= _number;
	bootROM[22] <= _branch;
	bootROM[23] <= 0;
	bootROM[24] <= _io_led;
	bootROM[25] <= _branch;
	bootROM[26] <= 0;
	bootROM[27] <= _lit;
	bootROM[28] <= 5000000;
	bootROM[29] <= _lit;
	bootROM[30] <= 1;
	bootROM[31] <= _minus;
	bootROM[32] <= _dup;
	bootROM[33] <= _zero_equal;
	bootROM[34] <= _0branch;
	bootROM[35] <= 29;
	bootROM[36] <= _drop;
	bootROM[37] <= _branch;
	bootROM[38] <= 0;
	bootROM[39] <= _dup;
	bootROM[40] <= _zero_less_than;
	bootROM[41] <= _0branch;
	bootROM[42] <= 47;
	bootROM[43] <= _lit;
	bootROM[44] <= "-";
	bootROM[45] <= _emit;
	bootROM[46] <= _negate;
	bootROM[47] <= _lit;
	bootROM[48] <= "0";
	bootROM[49] <= _plus;
	bootROM[50] <= _emit;
	bootROM[51] <= _lit;
	bootROM[52] <= " ";
	bootROM[53] <= _emit;
	bootROM[54] <= _execute;
	bootROM[55] <= _plus;
	bootROM[56] <= _execute;
	bootROM[57] <= _minus;
	bootROM[58] <= _execute;
	bootROM[59] <= _lit;
	bootROM[60] <= "0";
	bootROM[61] <= _execute;
	bootROM[62] <= _negate;
	bootROM[63] <= _execute;
	bootROM[100] <= 103;				// Link to next dictionary entry
	bootROM[101] <= {8'd1,"r","e","d"};	// Name field (NFA)
	bootROM[102] <= 6;					// bootROMory address containing code (XT/CFA)
	bootROM[103] <= 107;
	bootROM[104] <= {8'd2,"g","r","e"};
	bootROM[105] <= {"e","n","\0","\0"};
	bootROM[106] <= 11;
	bootROM[107] <= 111;
	bootROM[108] <= {8'd2,"b","l","u"};
	bootROM[109] <= {"e","\0","\0","\0"};
	bootROM[110] <= 16;
	bootROM[111] <= 115;
	bootROM[112] <= {8'd2,"d","e","l"};
	bootROM[113] <= {"a","y","\0","\0"};
	bootROM[114] <= 27;
	bootROM[115] <= 118;
	bootROM[116] <= {8'd1,"l","e","d"};
	bootROM[117] <= 8;
	bootROM[118] <= 121;
	bootROM[119] <= {8'd1,".","\0","\0"};
	bootROM[120] <= 39;
	bootROM[121] <= 124;
	bootROM[122] <= {8'd1,"+","\0","\0"};
	bootROM[123] <= 55;
	bootROM[124] <= 127;
	bootROM[125] <= {8'd1,"-","\0","\0"};
	bootROM[126] <= 57;
	bootROM[127] <= 131;
	bootROM[128] <= {8'd2,"e","m","i"};
	bootROM[129] <= {"t","\0","\0","\0"};
	bootROM[130] <= 53;
	bootROM[131] <= 134;
	bootROM[132] <= {8'd1,"\"","0","\""};
	bootROM[133] <= 59;
	bootROM[134] <= 0;
	bootROM[135] <= {8'd2,"n","e","g"};
	bootROM[136] <= {"a","t","e","\0"};
	bootROM[137] <= 62;
endtask : t_init_dictionary_code

assign boot_enable = mem_addr < XTQ_START;

// memory manager
always_ff @(posedge clk or negedge reset) begin
	if (reset == 1'b0) begin
		t_init_dictionary_code;
	end
	else begin
		mem_addr <= mem_access_outer ? (wp-XTQ_START) : mp;
		if (dict_write) begin
			mem[mem_addr] <= dict_wdata;
		end
	end
end
assign DataBus = boot_enable ? bootROM[mem_addr] : mem[mem_addr];

// Forth Outer Interpreter
always_ff @(posedge clk) begin
	logic [7:0] byte_in;
	logic [31:0] link_addr;
	logic [7:0] ccell, cchar, cells;
	logic [3:0][31:0] word_in;
	logic [address_size:0] local_xt;
	enum logic [2:0] {IDLE, PARSE, SEARCH, GET_LINK, GET_XT, EXECUTE, NUMBER, COMPILE} state;
	localparam DICT_START = 100;
	
	if (reset == 1'b0) begin
		wp = DICT_START;
		xt_valid <= 1'b0;
		state = IDLE;
		uart_receive <= 1'b1;
		xtwp = XTQ_START;
		nwp = '0;
		local_xt=0;
		word_in = '0;
		cells = '0;
		cchar = '0;
	end
	else begin
		case (state)
			IDLE : begin
				mem_access_outer = 1'b0;
				dict_write = 1'b0;
				wp = DICT_START;
				if (uart_rx_valid) begin
					uart_receive <= 1'b0;
					byte_in = uart_rx_data;
					state = PARSE;
				end
			end
			PARSE : begin
				mem_access_outer = 1'b1;
				if (10 <= byte_in && byte_in <= 13) begin//is character between <bl> and <cr>?
					wp = xtwp++;
					dict_write = 1'b1;
					dict_wdata = local_xt;
					local_xt = 0;
					word_in = '0;
					cells = '0;
					cchar = '0;
					state = EXECUTE;
				end
				else if (byte_in == " ") begin
					wp = xtwp++;	
					dict_write = 1'b1;
					dict_wdata = local_xt;
					local_xt = 0;
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
				link_addr = DataBus;
				++wp;
				ccell = '0;
				state = SEARCH;
			end
			SEARCH : begin
				if (ccell < cells && DataBus == word_in[ccell]) begin
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
				local_xt = DataBus;
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
				dict_write = 1'b0;
				xt_valid <= (xtrp < xtwp); // FIFO not empty (read before write)
				if (xtrp == xtwp) begin    // Stop when FIFO empty (read at write)
					state = IDLE;
					nwp = nrp;
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
        dp='0;
        rp='0;
        mp ='0;
		xtrp = XTQ_START;
		nrp = '0;
        busy <= false;
        skip_op <= false;
		uart_send <= 1'b0;
	endtask

	task automatic t_Fetch_opcode;
		if (branch) begin
			mp = branch_addr;
			branch = false;
		end
		else begin
			mp++;
		end
	endtask        

	//Execute opcodes task
	task automatic t_execute;
   
      if (skip_op == false) begin
		opcode = DataBus[code_size:0];
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
			if (skip_op == false) begin
				skip_op = true;
			end
			else begin
				++dp;
				data_stack[dp] = DataBus;
				skip_op = false;
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
			if (skip_op == false) begin
				skip_op = true;
			end
			else begin
				branch = (data_stack[dp] == '0) ? -1 : '0;
				--dp;
				branch_addr = DataBus;
				skip_op = false;
			end
		end
		 _branch : begin
			 if (skip_op == false) begin
				skip_op = true;
			end
			else begin
				branch = true;
				branch_addr = DataBus;
				skip_op = false;
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
		_execute : begin
			busy = true; // Wait until XT/CFA available
			if (skip_op == false) begin
				skip_op = true;
				mp = xtrp++;
			end
			else begin
				if (xt_valid) begin
					branch_addr = DataBus;
					busy = false;
					branch = true;
				end
				skip_op = false;
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
		t_execute;
        
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
