//-----------------------------------------------------------------------------
// Malak Ammar - 1211470
// Sarah Allahalih - 1211083
// Jana Sawalmeh - 1212467
//-----------------------------------------------------------------------------

`timescale 1ps / 1ps

module reg_file (clk, RegWr,addr_read1, addr_read2, addr_write, bus_read1 ,bus_read2, bus_write, reg_Access, reg_Data); 
	
	// define parameters for the memory
	parameter data_width = 16, address_width = 3;
	
	//define inputs in terms of parameters
	input clk; 
	input RegWr;// this is an enable for writing 
	input [(address_width-1):0] addr_read1, addr_read2, addr_write; 
	input signed [(data_width -1):0] bus_write; 
	output reg signed [(data_width -1):0] bus_read1 ,bus_read2; 	 
	
	// define register file in terms of parameters
	reg signed [(data_width-1):0] registers [ 0:((1<<address_width)-1)];
	
	// temp for testing 
	input [(address_width-1):0] reg_Access; 
	output reg signed [(data_width -1):0] reg_Data;
	
	always @(*)
		begin
			reg_Data <= registers[reg_Access];	
		end
	
	initial 
		begin
			
        	initialize_registers; // this is a task to intailize registers  
			bus_read1=0;
			bus_read2=0;
		
    		end
	
	always @(*)
			begin
				// Combinational logic 
				bus_read1 <= registers[addr_read1];	
				bus_read2 <= registers[addr_read2];			
			end
	
	
	reg disable_signal=0; // 0 : do not disable , 1 : disable 
	
	always @(posedge clk) 
	begin
			
		// check if writing on zero -> disable that 
		if (addr_write == 0)
			disable_signal=1;
		else
			disable_signal=0;
		
			
		if(RegWr && !disable_signal ) // if enabled
			begin 
				// write on clock edge
				registers[addr_write] <= bus_write;			
			end
	end 
		
	// Task to initialize registers
	
    task initialize_registers; 
		//registers = '{16'd0,16'd1,16'd2,16'd15,16'd4,16'd5,16'd6,16'd7};
	    registers[0] = 16'h10;
        registers[1] = 16'h0;
        registers[2] = 16'h9;
        registers[3] = 16'h2; 
        registers[4] = 16'h4; 
        registers[5] = 16'h5;
        registers[6] = 16'h1;
        registers[7] = 16'h2;
    endtask
	
	
endmodule  




module mux8x1 (a0, a1, a2, a3, a4,s, out);	
	
	// parameter 
	parameter bus_width =16;
	
	input [(bus_width-1):0] a0,a1,a2,a3,a4;
	input [2:0] s;
	output reg [(bus_width-1):0]  out;	
	
	always@(*)
	begin
		if (s==0)
			out = a0;
		else if (s==1)
			out =a1;
		else if (s==2)
			out =a2;
		else if (s==3)
			out =a3; 
		else if (s==4)
			out=a4;	
			
	end
	
endmodule


module mux2x1 (a0, a1,s, out);	
	
	// parameter 
	parameter bus_width =16;
	
	input [(bus_width-1):0] a0,a1;
	input s;
	output reg [(bus_width-1):0]  out;	
	
	always@(*)
	begin
		if (s==0)
			out = a0;
		else
			out=a1;
	end
	
endmodule



module adder (a, b, BTA);
	
	
	// define parameters for lengths
	parameter data_width = 16;		 
	input signed  [data_width-1:0] a, b; // assume inputs and outpust are signed here >> see this 	
	output reg signed  [data_width-1:0] BTA;
	
	always @(*)			
			BTA = a+b;

				
	
endmodule 	


module adder_2 (a, next_PC);
	
	
	// define parameters for lengths
	parameter data_width = 16;		 
	input [data_width-1:0] a; 
	output reg [data_width-1:0] next_PC;
	
	always @(*)			
			next_PC = a +1; // word addresable 
				
	
endmodule 



module extender (in,sign, out);	
	
	// parameter 
	parameter input_width =6, output_width=16;
	
	input [(input_width-1):0] in;
	input sign;
	output reg [(output_width-1):0]  out;	
	
	always@(*)
	begin
		if (sign == 1)  // Sign extension
      		out = {{output_width-input_width{in[input_width-1]}}, in};
    	
    	else  // Zero extension
      		out = {{{output_width - input_width}{1'b0}},in};
	end
	
endmodule





module instruction_memory (addr,instruction); 
	
	// define parameters for the memory
	parameter instruction_size = 16, address_width = 16;
	
	//define inputs and outputs in terms of parameters
	input [(address_width-1):0] addr; 
	output reg [(instruction_size -1):0] instruction ; 	 
	
	reg [15:0] memory [0:65535]; // 2^16

	// initialize memory with instructions
	initial begin 

		//R-Type
		
		//AND
		{memory[0]} = 16'b0000001010100000; // R-Type: Opcode=0000 (R-Type), Rd=001, Rs=010, Rt=100, Func=000  AND			 
		
		//ADD
		{memory[1]} = 16'b0000001010100001; // R-Type: Opcode=0000 (R-Type), Rd=001, Rs=010, Rt=100, Func=001  ADD			 

		//SUB
		{memory[2]} = 16'b0000001010100010; // R-Type: Opcode=0000 (R-Type), Rd=001, Rs=010, Rt=100, Func=010  SUB			 

		//SLL
		{memory[3]} = 16'b0000001010100011; // R-Type: Opcode=0000 (R-Type), Rd=001, Rs=010, Rt=100, Func=011  SLL			 
		
		//SRL
		{memory[4]} = 16'b0000001010100100; // R-Type: Opcode=0000 (R-Type), Rd=001, Rs=010, Rt=100, Func=100  SRL			 

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		//I-Type
		
		//ADDI
		{memory[5]} = 16'b0011001101000011; // I-Type: Opcode=0011 (ADDI-Type),  Rs=001, Rt= 101, Imm=000011 (signed)
		
		//ANDI
		{memory[6]} = 16'b0010001101000011; // I-Type: Opcode=0010 (ADDI-Type),  Rs=001, Rt= 101, Imm=000011 (signed)

		//Load
		{memory[7]} = 16'b0100011010000000; // LW: load word, Rs=011, Rt=010, Imm=000000 state(signed)
		
		//Store
		{memory[8]} = 16'b0101011010000000; // SW: Store word, Rs=011, Rt=010, Imm=000000 state(signed)	
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		//I-Type - Branch
		
		//BEQ
		{memory[9]} = 16'b0110001100101010; // BEQ: Branch if equal, Rs=001, Rt=100, Imm=101010 (signed)  
			
	     //FOR
		{memory[10]} = 16'b1000001111000000; // FOR: Opcode=0000 (for-Type), Rs=001, Rt=111, Imm=000000 (signed)

		
		//BNE
		{memory[11]} = 16'b0111110101001100; // BNQ: Branch if not equal, Rs=110, Rt=101, Imm=001100 (signed)   //jump to mem 11+12 = 23

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
	
		//JUMP	  
		{memory[23]} = 16'b0001000011000000; // JUMP: Opcode=0001 (jump-Type), Imm=010010 (signed)	, function=000	  // jump to 24
		
		//CALL	
		{memory[24]} = 16'b0001000011010001; // CALL: Opcode=0001 (call-Type), Imm=000000 (signed)	, function=001	  jump to 26 then give RR <- 25

		//RET
		{memory[26]} = 16'b0001000000000010; // RET: Opcode=0001 (return-Type), Imm=000000 ignored(signed)	, function=010     PC <- 19

		
		//ADDI
		{memory[25]} = 16'b0011001101000011; // I-Type: Opcode=0011 (ADDI-Type),  Rs=001, Rt= 101, Imm=000011 (signed)
			
	end 


	always @(addr)
		begin 	 	  
		
		// The instruction is 16 bits and the memory is byte addressable and little endian
		
		instruction = {memory[addr]};
		
		end	
	
endmodule 	 






module data_memory (clk,MemRd,MemWr,addr,Data_in,Data_out, read_Data,read_Access ); 
	
    // define parameters for the memory
    parameter data_size = 16, address_width = 16;
    
	//define inputs and outputs in terms of parameters
	input clk;
    input MemRd;
    input MemWr;
    input [(address_width-1):0] addr, read_Access; 
    input signed [(address_width -1):0] Data_in;
	
	// temp for testing 
	output reg  [(data_size -1):0] read_Data; 	 
	
    output reg signed [(data_size -1):0] Data_out; 	 
    
   // Define byte addressable memory  array	
	reg [15:0] memory [0:65535]; // 2^16 * 8-bit cells	
	
	
	
	// initialize data memory 
	initial 
	begin 
		
	memory[0] = 16'h0003; // 3
	memory[1] = 16'h0004; // 4
	memory[2] = 16'h0005; // 5
	memory[3] = 16'h0006; // 6
	memory[4] = 16'h0007; // 7
	memory[5] = 16'h0008; // 8
	memory[6] = 16'h0009; // 9
	
					
	end 
	
	always @(*)
		begin 
			// temp for testing
			read_Data = {  memory[read_Access]};	
		end 
   
		
	// write at positive edge of the clock 
	always @(posedge clk)
	begin 
		
		if (MemWr) begin		   			
			memory[addr] = Data_in[15:0];
			
		end
	end

	always @(*)
		begin 	 	  
			
			if (MemRd)begin
				// start reading at given addr
				Data_out = {  memory[addr]}; 
			end 
		end	
    
endmodule


module ALU (ALU_opcode, a, b, result, zero);	
	
	// define parameters for lengths
	parameter data_width = 16,opcode_length=3;	
	
	input [opcode_length-1:0] ALU_opcode;  
	//input ALU_control; 
	input signed  [data_width-1:0] a, b; // assume inputs and outpust are signed here >> see this 
	output reg signed  [data_width-1:0] result;
	// flags for output 
	output reg zero ; 
	
	
	
	//define parametrs for operations 
	parameter add_op=1, sub_op=2, and_op=0, SLL_op=3, SRL_op=4, no_op=5;
	
	//perform correct operation
	always @(*)
    begin 
				
		case (ALU_opcode)
            add_op:
				begin
					{result} = a+b;	
				end 
            sub_op:
				begin	
				{result} = a-b;
				end 
			
		   	and_op:		   
			   begin
				   {result} = a & b;	  // bitwise operation	   
		   	   end 
			 SLL_op:		   
			   begin
				   {result} = a << b;	  // bitwise operation	   
		   	   end
		      SRL_op:		   
			   begin
				   {result} = a >> b;	  // bitwise operation	   
		   	   end 
			no_op:		   
				begin   
				result= result;
				   // do nothing (reserve last outcome)
		   	   end 
				  
				  		  
            default: 	
				begin 
					result= 0;
				end
			
        endcase	
		
		// set zero flag :
		if (result==0)
			zero=1;
		else 
			zero=0;
		
		end	
		
			
endmodule 






module decode_and_multiplexing_unit (instruction, I_type_imm, addr_read1, addr_read2, addr_write, J_type_imm, opcode, func);	
	
	// define parameters for the memory
    parameter instruction_size = 16, address_width =3;

	input [(instruction_size -1):0] instruction;
    output reg signed [5:0] I_type_imm;
	output reg signed [8:0] J_type_imm;
    output reg [(address_width -1):0] addr_read1, addr_read2;
    output reg [(address_width -1):0] addr_write;
    output reg [3:0] opcode;  	
    output reg [2:0] func ;
	
   parameter AND = 4'b0000, ADD = 4'b0000, SUB = 4'b0000, SLL = 4'b0000, SRL = 4'b0000, 
             ADDI = 4'b0011, ANDI = 4'b0010, LW = 4'b0100, SW = 4'b0101, 
		    BEQ = 4'b0110,  BNE = 4'b0111, FOR = 4'b1000, JMP = 4'b0001, 
		    CALL = 4'b0001, RET = 4'b0001;  
	always @(*)
		begin 
	   opcode = instruction[15:12]; 
	   func = 3'b101; 
	   I_type_imm = 0;
	   addr_read1 = 0; 	
	   addr_read2 = 0; 
	   addr_write = 0;
	   J_type_imm = 0; 
	   case (instruction[15:12])
		   
		   AND, ADD, SUB, SLL, SRL :   //opcode = 0000
		   begin 	
			    func = instruction[2:0]; 
				addr_write = instruction[11:9];
				addr_read1 = instruction[8:6];	 
				addr_read2 = instruction[5:3];
		   end 
		   
			ADDI, ANDI: 
			begin 
				addr_write = instruction[8:6];
				addr_read1 = instruction[11:9];	 
				I_type_imm = instruction[5:0];
			end
			
		     LW: 
			begin
				addr_write = instruction[8:6];
				addr_read1 = instruction[11:9];
				I_type_imm = instruction[5:0];
			end
			
			BEQ,BNE:
			begin
			    addr_read1 = instruction[11:9];
				addr_read2 = instruction[8:6];	
				I_type_imm = instruction[5:0]; 
			end	 
			
			FOR:
			begin  
				 addr_write = instruction[8:6];
			     addr_read1 = instruction[11:9];
				 addr_read2 = instruction[8:6];	
			end	
			
		 JMP, CALL, RET:  
			 begin
			 func = instruction[2:0]; 
	
			 if (func == 0)
				 begin 
					 J_type_imm = instruction[11:3];	  // jump 
				 end
			else if (func == 1)
	             begin
			         J_type_imm = instruction[11:3];		// CALL
			 	end  
			end 
		 
		 //RET: 	
		 	//addr_read1 = 3'b111;   
		 
		 
		 SW: 
		 	begin
			 	addr_read1 = instruction[11:9];
				addr_read2 = instruction[8:6];	
				I_type_imm = instruction[5:0]; 
			end
		 
		 
		endcase 
		end 
 

endmodule

module ALU_control (func, opcode,state, ALU_opcode);	
	
   parameter AND = 4'b0000, ADD = 4'b0000, SUB = 4'b0000, SLL = 4'b0000, SRL = 4'b0000, 
             ADDI = 4'b0011, ANDI = 4'b0010, LW = 4'b0100, SW = 4'b0101, 
		    BEQ = 4'b0110,  BNE = 4'b0111, FOR = 4'b1000, JMP = 4'b0001, 
		    CALL = 4'b0001, RET = 4'b0001;  
			
			
	 input [3:0] state;  
	 
	parameter instruction_fetch = 0,
	instruction_decode = 1,  
	R_type_ALU = 2,
	R_type_completion = 3,
	I_type_ALU = 4,
	I_type_completion = 5, 
	Address_computation = 6, 
	Load_Mem_Access = 7,
	Store_Mem_Access = 8, 
	Load_completion = 9,
	Branch_Completion = 10, 
	FOR_computation = 11, 
	FOR_completion = 12;
	
	
	
	// define parameters for lengths
	parameter opcode_length=4 ,ALU_opcode_length=3, func_length=3;	
	
	input [opcode_length-1:0] opcode;  	
	input [func_length-1:0] func;  
	//input ALU_Control; ALU_opcode_length
	output reg [ALU_opcode_length-1:0] ALU_opcode;
	
	
	//define parametrs for operations 
	parameter add_op=1, sub_op=2, and_op=0, sll_op=3, srl_op=4, no_op=5; 
	
	
	always@(*)
		
		begin  
			ALU_opcode=no_op; // by default 
			
		
			// correct operation based on state
		case (state)
			
			Address_computation: 	//SW, LW
					ALU_opcode=add_op;
			
			R_type_ALU, I_type_ALU:		
			begin
				if (func == 3'b000 || opcode == ANDI)	  //and
						ALU_opcode= and_op;
			
				else if (func == 3'b001 || opcode ==ADDI )	//add
					ALU_opcode= add_op;
		
				else if (func == 3'b010)	 //sub
					ALU_opcode= sub_op;		
					
				else if (func == 3'b011)	 //sll
					ALU_opcode= sll_op;
		
				else if (func == 3'b100)
					ALU_opcode= srl_op; 	
			end
		
		
			Branch_Completion:
				ALU_opcode=sub_op;
			
			FOR_computation: 	//FOR  -> -1+Rt
				ALU_opcode=add_op; 
				
			instruction_fetch, instruction_decode, R_type_completion, I_type_completion, Load_Mem_Access, Store_Mem_Access, Load_completion:
				ALU_opcode=no_op; // by default 
	
		 default: ALU_opcode=no_op;
		
		endcase 
		end
			
endmodule 


module PC_control (opcode,func, zero,state, PCsrc, PCWrite);	
	
    parameter opcode_length=4;
	
   parameter AND = 4'b0000, ADD = 4'b0000, SUB = 4'b0000, SLL = 4'b0000, SRL = 4'b0000, 
             ADDI = 4'b0011, ANDI = 4'b0010, LW = 4'b0100, SW = 4'b0101, 
		    BEQ = 4'b0110,  BNE = 4'b0111, FOR = 4'b1000, JMP = 4'b0001, 
		    CALL = 4'b0001, RET = 4'b0001;  
    
	
	input [(opcode_length -1):0] opcode;
	input zero; 
	parameter func_length=3;
	input [func_length-1:0] func;  
	input [3:0] state; 
	
	output reg [2:0] PCsrc;
	output reg PCWrite; 
	
	parameter instruction_fetch = 0,
	instruction_decode = 1,  
	R_type_ALU = 2,
	R_type_completion = 3,
	I_type_ALU = 4,
	I_type_completion = 5, 
	Address_computation = 6, 
	Load_Mem_Access = 7,
	Store_Mem_Access = 8, 
	Load_completion = 9,
	Branch_Completion = 10, 
	FOR_computation = 11, 
	FOR_completion = 12;
	
	always@(*)
		begin  
			PCWrite = 0; 
			PCsrc = 0;
		case (state)
		instruction_decode: begin 
			if ((func == 3'b000) && (opcode == 4'b0001)) 	//jump
				begin 
				PCsrc = 3; 
				PCWrite = 1;
				end	
			 if (func == 3'b001 && opcode == 4'b0001)	//call
				 begin
				 PCsrc = 3;
				 PCWrite = 1; 
				 end
			 if (func == 3'b010 && opcode == 4'b0001)	//ret
				 begin
				 PCsrc = 4;
				 PCWrite = 1; 
				 end
			 end
		
		 R_type_completion, I_type_completion, Load_completion, Store_Mem_Access: 
		 		begin
					PCsrc = 0;
					 PCWrite = 1;
				end	 
				
		FOR_completion: 
		begin		  
			PCWrite = 1;
			if (zero == 0)		 //rt != 0
				PCsrc = 2; 	
			else if (zero == 1)    //rt == 0
				PCsrc = 0; 
			end	 		
					
		Branch_Completion: begin
			PCWrite = 1; 
		if (opcode == BEQ && zero == 1) 
			PCsrc =1; 
		else if (opcode == BEQ && zero == 0)
			PCsrc = 0; 
		else if (opcode == BNE && zero == 1)
			PCsrc = 0; 
		else if (opcode == BNE && zero == 0)
			PCsrc = 1; 
		else 
			PCsrc = 0; 
		end
			 endcase
		end	// always end 
	
	
endmodule



module Main_control (opcode, func, clk, IRwrite, RRwrite, Ext, ALU_src1, ALU_src2, WB_sel, MemRd, MemWr, RegWr, state);	
	
	
	// define parameters for the memory
    parameter opcode_length=4;
	parameter func_length=3;
	input [func_length-1:0] func;  
	
	
	input [(opcode_length -1):0] opcode; 
	input  clk; 
	output reg IRwrite, RRwrite, Ext, ALU_src1, ALU_src2, MemRd, MemWr, RegWr, WB_sel;	
	
 	output reg [3:0] state; 
	 
   parameter AND = 4'b0000, ADD = 4'b0000, SUB = 4'b0000, SLL = 4'b0000, SRL = 4'b0000, 
             ADDI = 4'b0011, ANDI = 4'b0010, LW = 4'b0100, SW = 4'b0101, 
		    BEQ = 4'b0110,  BNE = 4'b0111, FOR = 4'b1000, JMP = 4'b0001, 
		    CALL = 4'b0001, RET = 4'b0001;  
    
    
	parameter instruction_fetch = 0,
	instruction_decode = 1,  
	R_type_ALU = 2,
	R_type_completion = 3,
	I_type_ALU = 4,
	I_type_completion = 5, 
	Address_computation = 6, 
	Load_Mem_Access = 7,
	Store_Mem_Access = 8, 
	Load_completion = 9,
	Branch_Completion = 10, 
	FOR_computation = 11, 
	FOR_completion = 12;	 
	
	initial begin 
	state = instruction_fetch;
	end
	
	always@(posedge clk)				 // next state
	begin
	case (state)
		instruction_fetch : state <= instruction_decode; 
		
		instruction_decode: begin 
				
		if (opcode == ADD || opcode == AND || opcode == SUB || opcode == SLL || opcode == SRL)	  //opcode=0000
			   state <= R_type_ALU;
		  	
		else if (opcode == ADDI || opcode == ANDI) 
			   state <= I_type_ALU;
			
		else if (opcode == LW || opcode == SW)
			state <= Address_computation; 								  
			
		else if (opcode == BEQ || opcode == BNE) 
			state <=  Branch_Completion; 		   
			
		else if (opcode == FOR) 
			state <= FOR_computation; 	
			
		else begin 	   //jump, ret, call
			state <= instruction_fetch; 
			end		 
		end
		R_type_ALU : state <= R_type_completion;  
		
		R_type_completion : state <= instruction_fetch;	 
		
		I_type_ALU : state <= I_type_completion; 
		
		I_type_completion : state <= instruction_fetch; 
		
		Address_computation : begin    //load, store
							case (opcode)
								LW: state <= Load_Mem_Access; 
								SW : state <=  Store_Mem_Access; 
								endcase
							end 
		Load_Mem_Access : state <= Load_completion; 
						 			
		Load_completion : state <= instruction_fetch; 
		
		Store_Mem_Access: state <= instruction_fetch; 
		
		Branch_Completion: state <= instruction_fetch; 	
		
		FOR_computation :  state <= FOR_completion; 
		
		FOR_completion :	 state <= instruction_fetch; 
		
		endcase
	end	 
	always @(*) begin 
	 IRwrite = 0;
	 RRwrite = 0;
	 Ext = 1; 
	 ALU_src1 = 0;
	 ALU_src2 = 0;
	 WB_sel = 0;
	 MemRd = 0;
	 MemWr = 0;
	 RegWr = 0;
		
	case (state)
		instruction_fetch :IRwrite = 1;    
		
		instruction_decode:begin
						if (opcode == ANDI)
						Ext = 0; 	//unsigned extension
						if (func == 3'b001) //call
						RRwrite = 1;
						end
						
		R_type_ALU:  begin
			ALU_src1 = 0;
			ALU_src2 = 0;	
				   end 
		R_type_completion: begin
			             RegWr = 1; 
						 WB_sel = 0; 
						end
		I_type_ALU: begin
			ALU_src1 = 1;
			ALU_src2 = 0;	
					end
		
		I_type_completion: begin
			             RegWr = 1; 
						 WB_sel = 0; 
						end	   
		
	Address_computation: begin 
			ALU_src1 = 1;
			ALU_src2 = 0;	
					 end 		  
	
	Load_Mem_Access : begin
			MemRd = 1;
			MemWr = 0;
					end   
	
	Store_Mem_Access : begin
			MemRd = 0;
			MemWr = 1;
		              end
	 
	Load_completion : begin 
					RegWr = 1;
					WB_sel = 1;
					end
	Branch_Completion : begin 
			ALU_src1 = 0; 
			ALU_src2 = 0; 
					 end 
	
	FOR_computation : begin 
			ALU_src1 = 0; 
			ALU_src2 = 1; 
		             end 
	FOR_completion : begin 
					RegWr = 1;
		            end 
		endcase 
	end
endmodule	 

module zero_comparator (
    input [15:0] rt,     // 16-bit input to check
    output zero          // Output flag, 1 if rt is zero
);
    assign zero = ~|rt;  // NOR reduction operator: zero is 1 if all bits of rt are 0
endmodule


module proccessor (clk,ALU_out,state, opcode, IR, RR, PC, PCwrite, ALU_opcode, PCsrc,addr_write,RegWr,immediate , A, B , ALU_result, Inst_Address, mem_Data, reg_Data,mem_Access,reg_Access,WB_sel, I_type_imm,data_out,bus_write,addr_read1,addr_read2,ALU_src1, ALU_src2,zero, Jump_addr, cycle_count, instruction_count, ALU_instruction_count, load_instruction_count, store_instruction_count, control_instruction_count); 
	
	parameter instruction_opcode_length=4;
	
	// Parameters for ALU
    parameter data_width = 16, ALU_opcode_length = 3; // data width is the width of general purpose registers.
	// Parameters for reg_file
    parameter reg_address_width = 3;
	// Parameters for instruction_memory and data memory -> (address width is related to size of data and instruction memory)
    parameter instruction_size = 16, address_width = 16;
	// Parameters for PC
	parameter PC_width=16; 
	
	input clk;
	
	output reg [instruction_opcode_length-1:0] opcode; 
	output reg PCwrite;
	output reg signed [(data_width-1):0]ALU_out;
   	output reg [2:0] PCsrc;
	output reg [3:0] state;  
	output reg [(PC_width-1):0] PC;
	
	
	//reg clk;
	output reg [ALU_opcode_length-1:0] ALU_opcode;
	
	// wires and temporary
	wire [(PC_width-1):0] next_PC, BTA, PC_mux_wire;
	wire [(data_width-1):0] extended_I;
	wire [(instruction_size - 1):0] instruction;
	
    //wire signed [5:0] I_type_imm;
	output reg signed  [5:0] I_type_imm;
	
    wire signed [8:0] J_type_imm;
	
	
    //wire[(reg_address_width-1):0] addr_read1,addr_read2 ;
    //wire [(reg_address_width-1):0] addr_write;
	
	output reg [(reg_address_width-1):0] addr_write,addr_read1,addr_read2 ;
	
	wire signed [(data_width-1):0] bus_read1, bus_read2,  ALU_A, ALU_B, data_in_wire;
	wire signed [(PC_width-1):0 ] data_addr_wire;
	wire [15:0] save_RR;
	
	
	wire [2:0] func;
	
	output reg signed  [(data_width-1):0] data_out,bus_write;
		
	// Flags 
	//wire zero, carry, overflow, negative;
	output reg zero; 
	
	// Signals
	reg IRwrite, RRwrite, MemRd, MemWr, Ext;
	
	output reg RegWr,ALU_src1, ALU_src2;
	output reg WB_sel; 	
	// Separate zero wires for comparator and ALU
	wire zero_comparator_flag;
	wire zero_alu_flag;
	
	
	
	// Special Purpose Registers 
	
	output reg [instruction_size-1:0] IR; // return to this!!!!  
	output reg [instruction_size-1:0] RR;
	
	
	// Registers between stages
	output reg [(data_width -1):0]  immediate , A, B , ALU_result, Inst_Address;
	output reg [(data_width-1):0] cycle_count, instruction_count, ALU_instruction_count, load_instruction_count, store_instruction_count, control_instruction_count; 

	reg signed [(data_width -1):0] ALU_A_pre;

	
	initial begin 
		PC = 0;  
		cycle_count = 0;
		instruction_count = 0;
		ALU_instruction_count = 0;
		load_instruction_count = 0;
		store_instruction_count = 0;
		control_instruction_count = 0;		
		ALU_A_pre = -1;
	 end 
	 
	
	 
		
	 
	// temp for testing memory
	output reg [(data_width -1):0] mem_Data, reg_Data;
	
	input [(address_width-1):0]mem_Access;	 
	input [2:0]reg_Access;	 
	
	//----------- Datapath Connection: ----------
	
	
	assign Jump_addr = {PC[15:9],J_type_imm};	 
	
	// Instantiate the instruction_memory 
    instruction_memory #(instruction_size, address_width) instruction_memory (
        .addr(PC),
        .instruction(instruction)
    );
	
	  

	
	// Instantiate the decode_unit module
    decode_and_multiplexing_unit #(instruction_size,address_width) inst_decode_unit (
		.instruction(IR),
		.opcode(opcode),
		.func(func),
        .I_type_imm(I_type_imm),
        .J_type_imm(J_type_imm),
        .addr_read1(addr_read1),
		.addr_read2(addr_read2),
        .addr_write(addr_write)
		
    );
	
	
	// Instantiate extenders
	extender #(.input_width(6), .output_width(data_width)) I_extender (
    	.in(I_type_imm),
    	.sign(Ext),
    	.out(extended_I)
  	);
	
	  
	   
	//address for the J-type  (simlar to a concatenation unit)
	output reg [(data_width-1):0] Jump_addr;
	  
	
	
	 // Instantiate adders :
	adder_2 next_PC_adder (
    	.a(PC),
    	.next_PC(next_PC)
  	); 	
	  	  
	  
	adder BTA_adder (
    	.a(PC),
    	.b(extended_I),
    	.BTA(BTA)
  	);
	 
	

	// Instantiate the reg_file
    reg_file #(data_width, reg_address_width) reg_file (
        .clk(clk),
        .RegWr(RegWr),
        .addr_read1(addr_read1),
        .addr_read2(addr_read2),
        .addr_write(addr_write),
        .bus_write(bus_write),
        .bus_read1(bus_read1),
        .bus_read2(bus_read2),
		.reg_Access(reg_Access),
		.reg_Data(reg_Data)
    );
	
	//assign Inst_Address = A;
	
	// Instantiate mux for ALU B input
	mux2x1 ALU_mux (
    	.a0(B),
    	.a1(extended_I),
    	.s(ALU_src1),
    	.out(ALU_B)
  	);	  
	  
	// NOR Gate required as comparator
	zero_comparator comparator(ALU_B, zero_comparator_flag);
    	
	
	// Instantiate mux for ALU A input
	mux2x1 ALU_mux1 (
    	.a0(A),
    	.a1(ALU_A_pre),
    	.s(ALU_src2),
    	.out(ALU_A)
  	);	
	  

	 	  
    // Instantiate the ALU
    ALU #(data_width, ALU_opcode_length) alu (
        .ALU_opcode(ALU_opcode),
        .a(ALU_A),
        .b(ALU_B),
        .result(ALU_out),
        .zero(zero_alu_flag)
		//.ALU_control(ALU_control)
    );		
	
    // Resolve the "zero" signal as required
	//assign zero = ALU_mode ? zero_alu_flag : zero_comparator_flag;

	//module data_memory (clk,MemRd,MemWr,addr,Data_in,Data_out, read_Data,read_Access ); 
	// Instantiate the data_memory module
    data_memory #(data_width, address_width) data_memory (
        .clk(clk),
        .MemRd(MemRd),
        .MemWr(MemWr),
        .addr(ALU_out),
        .Data_in(B),
        .Data_out(data_out),
		.read_Access(ALU_out), //address
		.read_Data(mem_Data)	  //data out
		
    );
	
 
	
	// Instantiate Bus Write Mux
	mux2x1 write_mux (
    	.a0(ALU_result),
    	.a1(data_out),
    	.s(WB_sel),
    	.out(bus_write)
  	);
	
	
	// Instantiate PC Mux
	mux8x1 PC_mux (
    	.a0(next_PC),
    	.a1(BTA),
    	.a2(Inst_Address),    //for
    	.a3(Jump_addr), 
    	.a4(RR),    //rr
    	.s(PCsrc),
    	.out(PC_mux_wire)
  	); 
	
	
	// Instantiate the ALU control_unit module 
	ALU_control my_ALU (  
    .opcode(opcode), 	  
	.func(func),
	//.ALU_Control(ALU_control),
    .ALU_opcode(ALU_opcode),
	.state(state)
	);

	// Instantiate the PC control_unit module 
	PC_control my_PC (
    .opcode(opcode),
	.func(func),
	.state(state),
    .zero(zero),
	.PCsrc(PCsrc),
	.PCWrite(PCwrite)
	);
	
	// Instantiate the main control_unit module 
	Main_control my_main_control (
		.opcode(opcode),
		.func(func),
        .IRwrite(IRwrite),
		.RRwrite(RRwrite),
        .Ext(Ext),
        .ALU_src1(ALU_src1),	  	 
        .ALU_src2(ALU_src2),	  	 
		//.ALU_control(ALU_control),
        .WB_sel(WB_sel),
        .MemRd(MemRd),
        .MemWr(MemWr),
        .RegWr(RegWr),
		.clk (clk),
		.state(state)
    ); 
	
   
	  
	// Clock synchronization:
	always @ (posedge clk)
		begin
			A <= bus_read1;
			B <= bus_read2;	
			ALU_result <= ALU_out;
			immediate <= extended_I; 
			
			  
			Inst_Address <= A;
			
			// For loop
			if ((opcode == 4'b1000)) begin
		        zero <= zero_comparator_flag; 
		    end
			else begin
		        zero <= zero_alu_flag; 
			end
			
			if (IRwrite)
				IR <= instruction;

			if (RRwrite)
				RR <= next_PC;

			if(PCwrite)
				PC <= PC_mux_wire;
			
			// Performance Registers
				  
		    	cycle_count <= cycle_count + 1;

			// Check for the start of an instruction (state == 0)
			if (state == 4'b0000) begin
				instruction_count <= instruction_count + 1;
	
			end	 
			// ALU
			if ((opcode == 4'b0000 || opcode == 4'b0010 || opcode == 4'b0011) && (state == 4'b0010 || state == 4'b0100)) begin
				ALU_instruction_count <= ALU_instruction_count + 1;
	
			end	  
			// load
			if (opcode == 4'b0100 && (state == 4'b0110)) begin
				load_instruction_count <= load_instruction_count + 1;
	
			end			
			// store
			if (opcode == 4'b0101 && (state == 4'b0110)) begin
				store_instruction_count <= store_instruction_count + 1;
	
			end	
			// control
			if ((opcode == 4'b0001 && (state == 4'b0001)) || ((opcode == 4'b0110 || opcode == 4'b0111 || opcode == 4'b1000) && (state == 4'b1010 || state == 4'b1011))) begin
				control_instruction_count <= control_instruction_count + 1;
	
			end	
		end 
	

endmodule


module test_processor; 
	
	parameter instruction_opcode_length=4;	
    parameter data_width = 16, ALU_opcode_length = 3; 	
    parameter reg_address_width = 3;
    parameter instruction_size = 16, address_width = 16;
	parameter PC_width=16; 
	
	reg clk; 	
	
	
	wire [instruction_opcode_length-1:0] opcode; 
	wire PCwrite;
	wire signed [(data_width-1):0]ALU_out;
   	wire [2:0] PCsrc;
	wire [3:0] state;  
	wire [(PC_width-1):0] PC, IR, RR;
	wire  [(data_width -1):0]  immediate , A, B , ALU_result, Inst_Address;	
	
	// temp for testing 
	wire [(data_width -1):0] mem_Data, data_out,bus_write, Jump_addr;
	wire [(data_width -1):0] cycle_count, instruction_count, ALU_instruction_count, load_instruction_count, store_instruction_count, control_instruction_count;	
	reg [(address_width-1):0]mem_Access;	 
	
	wire [(data_width -1):0] reg_Data;	
	reg [2:0]reg_Access;
	
	wire WB_sel;
	//wire ALU_control;
	
	wire [5:0] I_type_imm;
	wire RegWr, ALU_src1, ALU_src2, zero;
	
	wire [2:0] addr_write,addr_read1, addr_read2 ;
	
	wire [2:0] ALU_opcode;	
	
	

	proccessor myProcessor (.clk(clk),
							.addr_read1(addr_read1),
							.addr_read2(addr_read2),
							.ALU_src1(ALU_src1),
							.ALU_src2(ALU_src2),
							.zero(zero),
							.ALU_opcode(ALU_opcode),
							.addr_write(addr_write),
							.RegWr(RegWr),
							.ALU_out(ALU_out),
							.state(state),
							.opcode(opcode), 
							.PC(PC), 
							.PCwrite(PCwrite),
							.PCsrc(PCsrc),
							.immediate(immediate),
							.A(A),
							.B(B),	  
							.Inst_Address(Inst_Address),
							.ALU_result(ALU_result),
							.mem_Access(mem_Access),
							.mem_Data(mem_Data),
							.reg_Access(reg_Access),
							.reg_Data(reg_Data), 
							.IR(IR),
							.RR(RR),
							.WB_sel(WB_sel),
							.I_type_imm(I_type_imm),
							.data_out(data_out),
							.bus_write(bus_write),
							.Jump_addr(Jump_addr),
							.cycle_count(cycle_count),
							.instruction_count(instruction_count),
							.ALU_instruction_count(ALU_instruction_count),
							.load_instruction_count(load_instruction_count),
							.store_instruction_count(store_instruction_count),
							.control_instruction_count(control_instruction_count)			
							); 
	

	// User-defined limits
	integer max_instructions = 39;  // Maximum number of instructions to execute	  
	integer instructions_count = 0;    // Tracks the number of instructions


	always #5 clk = ~clk; 
	initial 
		begin
			
		#5 
		clk =0;	


	end	   
	always @(posedge clk) begin
		if (state == 4'b0000) begin
			instructions_count = instructions_count + 1;	
		end
		if (instructions_count >= max_instructions) begin 
			 $finish; // End simulation
	end
	end
	
endmodule 	   


