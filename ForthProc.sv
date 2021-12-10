
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

parameter DataWidthSystem = 31;
parameter char_buf_ptr_width = 7;

//Internal SRAM parameters
parameter Int_SRAM_size = 31;
parameter Int_SRAM_ADDR_size = 16;

//LSRAM2
parameter data_width = 9;
parameter address_width = 10;
parameter ram_size = 1024;

//BootROM
parameter end_boot_ROM = 28;

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
  boot_depth        = 30,
  ram_depth         = 100,
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

//UART signals need Lattice version
//input logic [7:0] DATA_IN,
//output logic [7:0] DATA_OUT,
//output logic [12:0] BAUD_VAL,
//output logic WEN, OEN, CSN, BIT8, PARITY_EN, ODD_N_EVEN,
//input logic TXRDY,RXRDY, PARITY_ERR,FRAMING_ERR,OVERFLOW,

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
logic	[address_size:0] bp;      // boot ROM memory pointer
logic	[1:0] successful;
logic	[code_size:0] opcode;
 
//Circuliar stacks
logic [data_size:0] data_stack[data_stack_depth] ;
logic [data_size:0] return_stack[return_stack_depth];
logic [$clog2(data_stack_depth)-1:0] dp;
//logic [$clog2(data_stack_depth)-1:0] next_dp;//delete this? -dg 10-12-21
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

//Need to add Lattice UART here
//RS232 UART
//logic [7:0] char_buf[2**char_buf_ptr_width];
//logic [char_buf_ptr_width-1:0] char_buf_rd_ptr;
//logic [char_buf_ptr_width-1:0] char_buf_wr_ptr;
//logic [7:0] rdata;
//logic [7:0] wdata;
//logic UARTread;
//logic UARTwrite;
//logic valid;
//logic boot_flag;

//UART
//logic n_DATA_IN[7:0];
//logic n_DATA_OUT[7:0];

//Boot code when the processor starts...
task automatic t_init_boot_code;
	boot_ROM[0] <= _lit;		//next number is a literal number
	boot_ROM[1] <= 10000000;	//place on top of the stack	(TOS)						( 10000000 )
	boot_ROM[2] <= _lit;		//next number is a literal
	boot_ROM[3] <= 1;			//place on TOS											( 10000000 1 )
	boot_ROM[4] <= _minus;		//subtract TOS from second stack item					( 09999999 )
	boot_ROM[5] <= _dup;		//duplicate TOS											( 09999999 09999999 )
	boot_ROM[6] <= _zero_equal;	//is TOS equal to zero?									( 09999999 0 )
	boot_ROM[7] <= _0branch;	//if TOS zero next number fetched is a branch location
	boot_ROM[8] <= 2;			//jump to boot_ROM[2] above
	boot_ROM[9] <= _lit;		//next number is a literal number
	boot_ROM[10] <= 1;			//place on TOS											( 09999999 1 )
	boot_ROM[11] <= _io_led;	//store TOS to IO pin, LED is turned on					( 09999999 )
	boot_ROM[12] <= _lit;		//next number is a literal
	boot_ROM[13] <= 10000000;	//place on TOS											( 09999999 10000000 )
	boot_ROM[14] <= _lit;		//next number is a literal number
	boot_ROM[15] <= 1;			//place on TOS											( 09999999 10000000 1 )
	boot_ROM[16] <= _minus;		//subtract TOS from second stack item					( 09999999 09999999 )
	boot_ROM[17] <= _dup;		//duplicate TOS											( 09999999 09999999 09999999 )
	boot_ROM[18] <= _zero_equal;//is TOS equal to zero?									( 09999999 09999999 0 )
	boot_ROM[19] <= _0branch;	//if TOS zero next number fetched is a branch location 	( 09999999 09999999 )
	boot_ROM[20] <= 14;			//jump to boot_ROM[14] above							( 09999999 09999999 14 ) 
	boot_ROM[21] <= _lit;		//next number is a literal number
	boot_ROM[22] <= 0;			//place on TOS											( 09999999 09999999 0 )
	boot_ROM[23] <= _io_led;	//store TOS to IO pin, LED is turned on					( 09999999 )
	boot_ROM[24] <= _lit;		//next number is a literal number
	boot_ROM[25] <= 65;			//ASCII "A" place on TOS	                            ( 09999999 65 )
	boot_ROM[26] <= _emit;	    //output ASCII "A" to RS232 terminal					    ( 09999999 )    
	boot_ROM[27] <= _branch;	//Always branch (WHILE) to following location 			( 09999999 09999999 )
	boot_ROM[28] <= 0;			//place on top of the stack	(TOS)
endtask : t_init_boot_code 

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
        busy <= false;
        active_mem <= _ROM_active;
        DataStackDepth='0;
        ReturnStackDepth='0;
        skip_op <= false;                
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
			//n_LED_G <= LED_off;
			//n_LED_B <= LED_off;//low turns on  this is green
			//n_LED_R <= LED_on;			
			//n_LED_B <= data_stack[dp][2];
			//n_LED_G <= data_stack[dp][3];
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
			--dp;
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
//			wdata = data_stack[dp];
			--dp;             
        end
    
        _key : begin
//			data_stack[dp] = wdata;
			++dp;
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
		t_Fetch_opcode;
		if (skip_op == false) begin
			t_execute;
		end
		else begin
			skip_op <= false;
		end
        
        n_BUTTON0 <= BUTTON0;
        n_BUTTON1 <= BUTTON1;         
		//LED_G <= n_LED_G;
 		//LED_B <= n_LED_B; 
 		//LED_R <= n_LED_R;		  
    end 
end

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
  
  
  
endmodule

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

