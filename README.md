# AI-Robotics
The vision of this group is to create a Forth based AI Computer where you can communicate with the system using an English conversational approach.  We will add many high level Words for string manipulation, arrays, and other structures as we develop this computer.  We will extend Forth to include AI words too.  All of this will be built into FPGAs which are really parallel computers written in Behavioral System Verilog (the Assembler).

The CORE I AI Playground Project
CORE = CISC Over RISC 
The goal of the project is to create an intelligent AI Computer which you can use like CHATGTP using an interactive terminal approach.
Why did we choose Forth as the architecture of the machine?
Chat terminal interface (Interpreter).  Type in "Words" which are executed immediately.  Create new Words based on the Words the system already knows.  You can do this interactively, one Word at a time if you choose.  Or, write your Word definitions in a text editor, copy and paste into the terminal.
Forth's Incremental Compiler works more like the human brain.  Do you learn by editing source code then compile into a binary blob then JTAG your brain?  Of course not.  Your brain learns new things by storing them in your neurons, they don't go through a Compiler first.  Forth compiles one Word at a time.  Teach it new words, the system becomes smarter incrementally like the human brain.
Easily integrate CHATGPT or Bard into the systems terminal by adding supporting Words for the AI.  Using the on-board esp32 WIFI, you can talk to an online AI Chatbot seamlessly.  In fact, ask Bard to write System Verilog code for you!  It works!
The Forth Computer is built on top of a CORE I processor.  As far as I know, this is the world's first full Computer on a Chip.  You don't need another computer for programming, it is all done inside the FPGA.
Natural Language Interface: This is free and already supported by Forth.  We can plug in a speech to text system and you can talk to it, even program using the English language.
We added 10 User registers so you can manipulate data, pass if from one Word the next without resorting to using Forth's Stack words.  This makes easier to read code.  Forth programmers can still use the Stack, the words are supported.
Words as processor op-codes: This is a very powerful feature, even a novice at FPGA programming can write System Verilog code and create their own op-codes/Words.
Finally, we implemented a BUSY FLAG in the Inner Interpreter so when the user writes a complex op-code, the processor stalls until the op-code fully completes.  Writing code for FPGAs requires a lot of simulation and testing because you have to make sure the function completes within a clock cycle, or you have to use semaphores (like BUSY FLAG).  You have to worry about setup and hold times, plus propagation delay.  You don't have to be an expert in FPGA coding.

Why custom hardware? CORE I AI Playground board.
Forth needs very little RAM to operate.  DDR4 Ram is very slow!  This is the RAM in your computer.  Typically, DDR4 ram can fetch and store data on the average of 80 nanoseconds or longer.  High speed L1 Cache SRAM can fetch and store data in 3 nanoseconds.  We have 64,000 cells of memory included on the board.
Forth can run out of Flash memory.  The Flash memory data bus is 16 bit wide.  A page can be accessed within 25 us, data within the 1K x 16 page is 45 ns.  This is similar to how DDR4 memory works.  It takes much longer to access data outside the current page.
We can use the 2G Flash memory as long term storage.  There will be a Word called: REMEMBER which saves the entire SRAM into the Flash.  The Flash is large enough to store many dozens of snapshots.  Flash can also be used to store compiled code, a Word can CALL this pre-compiled code as needed. 
Finally an SD Card is included as our removable disk drive.  Store source code, images, even pre-compiled code with unlimited storage.
An esp32 microprocessor allows the board to access the internet.  Talk to CHATGPT or Bard right in the systems terminal.
A PS2 keyboard interface and a 4 inch touch screen completes the system for a high performance Intelligent embedded or personal computer.
A USB Cable network daisy chain connectors allow you to connect together as many boards as you want.
The 25K LUT FPGA standard version can support up to dozen 32 bit CORE I co-processors as well as the main Forth Computer for example.  The 85K LUT board can over 40 32 bit CORE I co-processors.  This gives you an idea of the power parallel computer you can build.
Longer term, we will add AI Words to extend the high level language to include AI functions such as Prolog, Lisp, Expert Systems, Neural Networks and other Machine learning functions.

Getting started with the CORE  I project:  Watch the three videos on the CORE I project.

Laterest Video: Forth in space is required: https://www.youtube.com/watch?v=9FH6kltGspM

Video: Overall view, current state of development and the goals of the project...
https://www.youtube.com/watch?v=wVRH25-lrrs&t=653s

[https://www.youtube.com/watch?v=hm6YzarM08Q&t=459s](https://www.youtube.com/watch?v=hm6YzarM08Q&t=459s)

[https://www.youtube.com/watch?v=uA-QG4Hst-s&t=56s](https://www.youtube.com/watch?v=uA-QG4Hst-s&t=56s)

Order this board:

[https://www.tindie.com/.../upduino-v31-low-cost-lattice.../](https://www.tindie.com/products/tinyvision_ai/upduino-v31-low-cost-lattice-ice40-fpga-board/)

Download Software Tools:

[https://www.latticesemi.com/latticediamond](https://www.latticesemi.com/latticediamond)

Source Code Review - this is before the the full Forth Computer was completed.  It does show how the system is built.  You will be able to use 'high level Forth in a terminal program running on your computer.  Both the Interpreter and Compiler is on chip.  Compiler is the final focus now.

[https://www.youtube.com/watch?v=KXjQdKBl7ag&t=409s](https://www.youtube.com/watch?v=KXjQdKBl7ag&t=409s)

Read Starting Forth and Thinking Forth.

Starting Forth Book

[https://www.forth.com/.../uploads/2018/01/Starting-FORTH.pdf](https://www.forth.com/wp-content/uploads/2018/01/Starting-FORTH.pdf)

Thinking Forth Book

[http://ftp.twaren.net/.../distfiles/thinking-forth-color.pdf](http://ftp.twaren.net/BSD/OpenBSD/distfiles/thinking-forth-color.pdf)

Learn Behavioral System Verilog

A great book on the latest System Verilog features...

[https://www.amazon.com/RTL.../dp/1546776346/ref=sr_1_1...](https://www.amazon.com/RTL-Modeling-SystemVerilog-Simulation-Synthesis/dp/1546776346/ref=sr_1_1?crid=GV4ZALCIJZRT&keywords=systemverilog%20sutherland&qid=1677614632&sprefix=system%20verilog%20sutherland%2Caps%2C107&sr=8-1&ufe=app_do%3Aamzn1.fos.18ed3cb5-28d5-4975-8bc7-93deae8f9840)

Video series on System Verilog....

[https://www.youtube.com/watch?v=U18k9TDP5uw...](https://www.youtube.com/watch?v=U18k9TDP5uw&list=PLYdInKVfi0KZ1HMVNNcxvvWhYJMmLAq_g)

When the system is completed (August - best guess), you can design your own processor opcodes written in System Verilog running at the speed of silicon.  You can use massive parallel processing with interactive debugging.  Define your own test words (Functions) interactively with instant compilation.

RTL Modeling with SystemVerilog for Simulation and Synthesis: Using SystemVerilog for ASIC and FPGA Design

[AMAZON.COM](https://www.amazon.com/RTL-Modeling-SystemVerilog-Simulation-Synthesis/dp/1546776346/ref=sr_1_1?crid=GV4ZALCIJZRT&keywords=systemverilog%20sutherland&qid=1677614632&sprefix=system%20verilog%20sutherland%2Caps%2C107&sr=8-1&ufe=app_do%3Aamzn1.fos.18ed3cb5-28d5-4975-8bc7-93deae8f9840)
RTL Modeling with SystemVerilog for Simulation and Synthesis: Using SystemVerilog for ASIC and FPGA Design
AMAZON.COM
RTL Modeling with SystemVerilog for Simulation and Synthesis: Using SystemVerilog for ASIC and FPGA Design
This book is both a tutorial and a reference for engineers who use the SystemVerilog Hardware Description Language (HDL) to design ASICs and FPGAs. The book shows how to write SystemVerilog models at the Register Transfer Level (RTL) that simulate and synthesize correctly, with a focus on proper ...

