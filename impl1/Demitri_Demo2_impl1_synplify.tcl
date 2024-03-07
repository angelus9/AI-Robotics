#-- Lattice Semiconductor Corporation Ltd.
#-- Synplify OEM project file

#device options
set_option -technology ECP5UM5G
set_option -part LFE5UM5G_85F
set_option -package BG381C
set_option -speed_grade -8

#compilation/mapping options
set_option -symbolic_fsm_compiler true
set_option -resource_sharing true

#use verilog 2001 standard option
set_option -vlog_std v2001

#map options
set_option -frequency 200
set_option -maxfan 1000
set_option -auto_constrain_io 0
set_option -disable_io_insertion false
set_option -retiming false; set_option -pipe true
set_option -force_gsr false
set_option -compiler_compatible 0
set_option -dup false

set_option -default_enum_encoding default

#simulation options


#timing analysis options



#automatic place and route (vendor) options
set_option -write_apr_constraint 1

#synplifyPro options
set_option -fix_gated_and_generated_clocks 1
set_option -update_models_cp 0
set_option -resolve_multiple_driver 0
set_option -vhdl2008 1

set_option -seqshift_no_replicate 0

#-- add_file options
set_option -include_path {D:/MyStuff/Projects/Forth_Processor2-Lattice - dg/Diamond/Demitri Demo}
add_file -verilog -vlog_std sysv {D:/MyStuff/Projects/Forth_Processor2-Lattice - dg/Diamond/Demitri Demo/impl1/source/Forth_Proc.sv}


#-- set result format/file last
project -result_file {D:/MyStuff/Projects/Forth_Processor2-Lattice - dg/Diamond/Demitri Demo/impl1/Demitri_Demo2_impl1.edi}

#-- error message log file
project -log_file {Demitri_Demo2_impl1.srf}

#-- set any command lines input by customer


#-- run Synplify with 'arrange HDL file'
project -run
