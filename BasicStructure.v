`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:49:01 01/10/2021 
// Design Name: 
// Module Name:    Basic RISC-V single cycle implementation on Verilog 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module main(rst, clk, alu_z, Anode_Activate, LED_out);
input rst, clk;						// 1 button to reset, clock signal as input
output alu_z;						// An LED turned ON if result is zero
output reg[7:0] Anode_Activate;		// Anodes to control 7-segments
output reg [6:0] LED_out;			// output result to be sent on 7-segments



	// ALL modules will be called in this file. Code will be executed and results will be shown on 7-segment display
// Code segment for BCD to 7-segment Decoder. Keep this code as it is
reg [31:0] counter ;		// A 32 bit flashing counter
reg toggle;			// A variable to toggle between two 7-segments 

always @(posedge clk)
    begin
            if (!rst)begin
            if(counter>=100000) begin //100000
                 counter <= 0;
				 toggle = ~toggle; end
            else begin
                counter <= counter + 1;
				end
			end
			else begin
			     toggle<=0;
			     counter<=0;
			     end
    end 
    // anode activating signals for 8 segments, digit period of 1ms
    // decoder to generate anode signals 
    always @(*)
    begin
        case(toggle)
        1'b0: begin
            Anode_Activate = 8'b01111111; 
            // activate SEGMENT1 and Deactivate all others
              end
        1'b1: begin
            Anode_Activate = 8'b10111111; 
            // activate LED2 and Deactivate all others    
               end
        endcase
    end
    // Cathode patterns of the 7-segment 1 LED display 
wire [31:0] result;

    always @(*)
    begin
	if (toggle) begin
        case(result)				// First 4 bits of Result from ALU will be displayed on 1st segment
        32'b0000: LED_out = 7'b0000001; // "0"     
        32'b0001: LED_out = 7'b1001111; // "1" 
        32'b0010: LED_out = 7'b0010010; // "2" 
        32'b0011: LED_out = 7'b0000110; // "3" 
        32'b0100: LED_out = 7'b1001100; // "4" 
        32'b0101: LED_out = 7'b0100100; // "5" 
        32'b0110: LED_out = 7'b0100000; // "6" 
        32'b0111: LED_out = 7'b0001111; // "7" 
        32'b1000: LED_out = 7'b0000000; // "8"     
        32'b1001: LED_out = 7'b0000100; // "9"
		32'b1010: LED_out = 7'b0001000; // "A"     
        32'b1011: LED_out = 7'b1100000; // "b"     
        32'b1100: LED_out = 7'b0110001; // "C"     
        32'b1101: LED_out = 7'b1000010; // "d"     
        32'b1110: LED_out = 7'b0110000; // "E"     
        32'b1111: LED_out = 7'b0111000; // "F"     
        
        default: LED_out = 7'b0000001; // "0"
        endcase
		end
    

	// Cathode patterns of the 7-segment 2 LED display
        if(!toggle) begin	
        case(result[7:4])			// Next 4 bits of Result from ALU will be displayed on 2nd segment
        4'b0000: LED_out = 7'b0000001; // "0"     
        4'b0001: LED_out = 7'b1001111; // "1" 
        4'b0010: LED_out = 7'b0010010; // "2" 
        4'b0011: LED_out = 7'b0000110; // "3" 
        4'b0100: LED_out = 7'b1001100; // "4" 
        4'b0101: LED_out = 7'b0100100; // "5" 
        4'b0110: LED_out = 7'b0100000; // "6" 
        4'b0111: LED_out = 7'b0001111; // "7" 
        4'b1000: LED_out = 7'b0000000; // "8"     
        4'b1001: LED_out = 7'b0000100; // "9"
		4'b1010: LED_out = 7'b0001000; // "A"     
        4'b1011: LED_out = 7'b1100000; // "b"     
        4'b1100: LED_out = 7'b0110001; // "C"     
        4'b1101: LED_out = 7'b1000010; // "d"     
        4'b1110: LED_out = 7'b0110000; // "E"     
        4'b1111: LED_out = 7'b0111000; // "F"     
        
        default: LED_out = 7'b0000001; // "0"
        endcase
    end
end

	// Keep writing your code (calling lower module) here!
	
	
	wire [31:0] pc_next ,pc_target,pc_plus4, instruction ;
    reg [31:0] pc;
	wire [31:0] RD1,RD2, SrcB, ALUResult;
    wire  ALU_Src, carry, MemWrite, Regwrite, PC_Src,zero;
    wire [1:0] Imm_Src, Result_Src;
    wire [31:0] ImmExt, ReadData, Result, instr;
    wire [3:0] ALUControl;
	
	initial  // initial pc
    pc = 32'h0; 
    
    always @ (posedge clk) begin // address Generator
        if(rst)
        pc = 32'h0;
        else
        pc <= pc_next;     
    end
    
    Instruction_Memory IM(.pc(pc),.instruction(instruction)); // fetching the instruction
    
    // Calling the Control Unit
    
    Control_Unit CU( .op(instruction[6:0]),.funct3(instruction[14:12]),.funct7(instruction[30]),.zero(zero),.PC_Src(PC_Src),.Result_Src(Result_Src),.MemWrite(MemWrite),.ALU_Src(ALU_Src),.RegWrite(Regwrite),.ALUControl(ALUControl),.Imm_Src(Imm_Src));        
     
     //Register File calling       
            
    register_file R_F(.Port_A(RD1),.Port_B(RD2),.Din(Result), .Addr_A(instruction[19:15]), .Addr_B(instruction[24:20]), .Addr_Wr(instruction[11:7]),.wr(Regwrite),.Hard_wired(result), .clk(clk)); // complete
            
         // immediate Extender
         
    extend Ext(.instr(instruction[31:7]),.immsrc(Imm_Src),.immext(ImmExt)); // complete
    
    // Mux 2 into 1 for 2nd operand Selector
            
    mux2x1 mux1(.a(ImmExt),.b(RD2),.sel(ALU_Src),.o(SrcB)); // complete ScrB
    
    //ALU calling
          
    alu ALU(.A(RD1),.B(SrcB),.ALU_Sel(ALUControl),.ALU_Out(ALUResult),.CarryOut(carry),.ZeroOut(zero));
    
    //Data Memory
  
            
    Data_Memory D_M(.Data_Out(ReadData),.Data_In(RD2),.D_Addr(ALUResult),.wr(MemWrite),.clk(clk) );
    
    //Two Pc's Adders
    
    adder add1(.a(pc),.b(32'd4),.y(pc_plus4));
    adder add2(.a(pc),.b(ImmExt),.y(pc_target)); 
    
    
    // MUX 3*1 for PC_Result
            
    mux3x1 result_mux(.a(ALUResult),.b(ReadData),.c(pc_plus4),.sel(Result_Src),.o(Result));
    
    //mux for PC_ Selector
    
    mux2x1 mux2(.a(pc_target),.b(pc_plus4),.sel(PC_Src),.o(pc_next));
    
     

endmodule




//datapath ( input pc,


// A module to generate the address of next instruction
// LOGIC: 	Add 1 in program counter if its a normal instruction
//			Add address of label in PC if its a branch instruction			
// other parameters can also be added based on Datapath and Controller Inputs/Outputs
//module adress_generator (output [31:0] pc, input PC_Src, input [31:0] immext,input rst,input clk);	

//	// Write your code here. This is not the part of Phase-I
//	wire [31:0] Pc_next ,Pc_target ; 
//	always @ (posedge clk) begin
//	if (rst) begin
//	   pc = 32'h00;
//	end
//	else begin
//	adder(.a(pc),.b(32'd4),.y(Pc_next));
//	adder(.a(pc),.b(immext),.y(Pc_target));
	
//     mux2x1(.a(Pc_target),.b(Pc_next),.sel(PC_Src),.o(pc));
//	end
//endmodule


// A module that will carry the all instructions. PC value will be provided to this module and it will return the instuction
// other parameters can also be added based on Datapath and Controller Inputs/Outputs
module Instruction_Memory (input [31:0] pc,output reg [31:0] instruction);
    
    always @ (*) begin
    case(pc)                //31                   
        32'h0000: instruction = 32'b00000000110000000000010000010011;//  a = 12
        32'h0004: instruction = 32'b00000000100100000000010010010011;//4 b = 9
      //32'h0000: instruction = 32'h01900413;
      //32'h0004: instruction = 32'h01400493; 
      //32'h0008: instruction = 32'h0x40940433;
    
    32'h0008: instruction = 32'h00940c63;//32'b00000000100101000000011001100011;//8
    32'h000c: instruction =  32'h00944663;//32'b00000000100101000000011001010101;   //12 
    32'h0010: instruction = 32'h40940433;//32'b00100000100101000000010000110011;//16
    32'h0014: instruction = 32'hff5ff06f;//32'b11111111010111111111000011101111;//20
    32'h0018: instruction = 32'h408484b3;//32'b00100000100001001000010010110011;//24
    32'h001c: instruction = 32'hfedff06f;//32'b11111111010111111111000011101111;//28
    32'h0020: instruction = 32'h0000006f;//32
    //32'h0024://36
    endcase
    end
	
endmodule


// This module is called Data_Memory. It will consists of 256 byte memory unit. It will have 
// one input as 8-bit address, 1 input flag wr that will decide if you want to read memory or write memory
module Data_Memory(output reg [31:0] Data_Out, input [31:0] Data_In, input [31:0] D_Addr, input wr, input clk );
		reg [31:0] Mem [255:0];			// Data Memory
  
	// Write your code here
	always @(*) begin
	Data_Out = Mem[D_Addr]; // Data Loading
	end
	always @(posedge clk) begin // Data Storing
	if (wr)
	Mem[D_Addr] = Data_In;
	end
endmodule



// This module is called Register_Memory. It will consists of 32 registers each of 32-bits. It will have 
// one input flag wr that will decide if you want to read any register or write or update any value in register
// This module will 2 addresses and provide the data in 2 different outputs
module register_file(Port_A, Port_B, Din, Addr_A, Addr_B, Addr_Wr, wr,Hard_wired, clk); 
			output reg [31:0] Port_A, Port_B;			// Data to be provided from register to execute the instruction // reg by me
			input [31:0] Din;						// Data to be loaded in the register
			input [4:0] Addr_A, Addr_B, Addr_Wr;	// Address (or number) of register to be written or to be read
			input wr, clk;	
			output reg [31:0] Hard_wired;						// input wr flag input // by me clk
			reg [31:0] Reg_File [31:0];				// Register file

	// Write your code here
	// read
	
	always @ (*) begin  // Data Reading
	Reg_File[0] = 32'd0;
	Port_A = Reg_File[Addr_A];
	Port_B = Reg_File[Addr_B];
	Hard_wired = Reg_File[8];
	end
	
	always @(posedge clk) begin // Data Writing
	if(wr)
	   Reg_File[Addr_Wr] = Din;
	end

	
endmodule


module Control_Unit(input [6:0] op,
                    input [2:0] funct3,
                    input funct7,
                    input zero,
                    output  PC_Src,
                    output reg MemWrite,
                    output reg ALU_Src,
                    output reg RegWrite,
                    output reg [3:0] ALUControl,
                    output reg [1:0] Imm_Src,
                    output reg [1:0] Result_Src);			
	// This is the part of Phase-II
	reg jump, branch;
	reg [1:0] ALUOP;
	always @(*)
	begin 
	 case (op)
	 7'b0000011: //lw
	       begin
	           RegWrite   = 1'b1;
	           Imm_Src    = 2'b00;
	           ALU_Src    = 1'b1;
	           MemWrite   = 1'b0;
	           Result_Src = 2'b01;
	           branch     = 1'b0;
	           ALUOP      = 2'b00;
	           jump       = 1'b0;
	           end
	           
      7'b0100011: //sw
              begin
              RegWrite   = 1'b0;
              Imm_Src    = 2'b01;
              ALU_Src    = 1'b1;
              MemWrite   = 1'b1;
              Result_Src = 2'bxx;
              branch     = 1'b0;
              ALUOP      = 2'b00;
              jump       = 1'b0;
               end  
      7'b0110011: //R-type
              begin
               RegWrite   = 1'b1;
               Imm_Src    = 2'bxx;
               ALU_Src    = 1'b0;
               MemWrite   = 1'b0;
               Result_Src = 2'b00;
               branch     = 1'b0;
               ALUOP      = 2'b10;
               jump       = 1'b0;
                        end
        7'b1100011: //beq
               begin
               RegWrite   = 1'b0;
               Imm_Src    = 2'b10;
               ALU_Src    = 1'b0;
               MemWrite   = 1'b0;
               Result_Src = 2'bxx;
               branch     = 1'b1;
               ALUOP      = 2'b01;
               jump       = 1'b0;
                              end
                              
                              
                              
   7'b1010101: //blt
             begin
               RegWrite   = 1'b0;
               Imm_Src    = 2'b10;
               ALU_Src    = 1'b0;
               MemWrite   = 1'b0;
               Result_Src = 2'bxx;
               branch     = 1'b1;
               ALUOP      = 2'b01;
               jump       = 1'b0;
               end                            
                              
                              
                              
                              
	     	 7'b0010011: //I-type ALU
                 begin
                 RegWrite   = 1'b1;
                 Imm_Src    = 2'b00;
                 ALU_Src    = 1'b1;
                 MemWrite   = 1'b0;
                 Result_Src = 2'b00;
                 branch     = 1'b0;
                 ALUOP      = 2'b10;
                 jump       = 1'b0;
                              end
 7'b1101111: //jal
                  begin
                  RegWrite   = 1'b1;
                  Imm_Src    = 2'b11;
                  ALU_Src    = 1'bx;
                 MemWrite   = 1'b0;
                  Result_Src = 2'b10;
                  branch     = 1'b0;
                  ALUOP      = 2'bxx;
                 jump       = 1'b1;
                 end
 default: //jal
                                  begin
              RegWrite   = 1'bx;
              Imm_Src    = 2'bxx;
              ALU_Src    = 1'bx;
               MemWrite   = 1'bx;
              Result_Src = 2'bxx;
                branch     = 1'bx;
               ALUOP      = 2'bxx;
             jump       = 1'bx;   
	           end
	 endcase
	 
	 case (ALUOP)
	 2'b00: ALUControl = 4'b0000;
	 
	 2'b01:
	   begin
	   case (funct3)
	   3'b000:  ALUControl = 4'b0001; // sybtract
	   3'b100:  ALUControl = 4'b1110; // less than
	   endcase
       end
	
	 2'b10: begin
	 //if ((funct3 == 3'b000 && (funct7 == 0)) begin
	   case ({funct3,funct7})
	         {3'b000,1'b0}:  ALUControl = 4'b0000;
	         {3'b000,1'b1}:  ALUControl = 4'b0001;
	         {3'b001,1'b0}:  ALUControl = 4'b0001;
	         {3'b001,1'b1}:  ALUControl = 4'b0001;
	         {3'b010,1'b0}:  ALUControl = 4'b0001;
	         {3'b010,1'b1}:  ALUControl = 4'b0001;
	         {3'b011,1'b0}:  ALUControl = 4'b0001;
	         {3'b011,1'b1}:  ALUControl = 4'b0001;
	         {3'b100,1'b1}:  ALUControl = 4'b0001;
	         {3'b101,1'b0}:  ALUControl = 4'b0001;
	         {3'b101,1'b1}:  ALUControl = 4'b0001;
	         {3'b110,1'b0}:  ALUControl = 4'b0001;
	         {3'b110,1'b1}:  ALUControl = 4'b0001;
	         {3'b111,1'b0}:  ALUControl = 4'b0001;
	         {3'b111,1'b1}:  ALUControl = 4'b0001;

	    endcase
	    end
	    default: ALUControl = 4'bxxxx;
	    endcase
	    end

	wire temp;
	and (temp, branch, zero);
	or (PC_Src, jump, temp);
endmodule






// General Module of two input (5 bit) multiplexer. This multiplexer will be connected with ALU control signals
//module mux(o,a,b, sel);
//    input [4:0] a,b;			// 5 bit inputs
//	input sel;					// selection signal
//    output reg [4:0] o;			// 5 bit output

//	// write your code here!
	
//endmodule

// A two by one 32 bit multiplexer (to select the branch instruction)
module mux2x1(output  [31:0]o,		// 32 bit output
					input[31:0]a,b,		// 32 bit inputs
					input sel			// Selection Signal
			);
			
	// Write your code here!
	assign o = sel ? a : b;
	
endmodule


module mux3x1(output [31:0]o,		// 32 bit output
					input[31:0]a,b,c,	// 32 bit inputs
					input [1:0] sel			// Selection Signal
			);
			
	// Write your code here!
	assign o = sel[1] ? c: (sel[0] ? b : a);
	
endmodule

module adder(input [31:0] a,b,output [31:0] y);
         assign y = a+b;    
endmodule 



// The final module ALU which will accept the signal (Function) from Control Unit
// and two operands (either from register file or from memory (data or address),
// will perform the desired operarion and proivde the output in Result and Zero flag.
//module ALU(Result, alu_z, Op_A, Op_B, Function);
//	output [31:0] Result;		// 32 bit Result
//	output alu_z;				// Zero flag (1 if result is zero)
//	input [31:0] Op_A, Op_B;	// Two operands based on the type of instruction
//	input [3:0] Func;			// Function to be performed as per instructed by Control Unit
	
//	// Write your code here
//endmodule


module alu(
           input [31:0] A,B,  // ALU 8-bit Inputs
           input [3:0] ALU_Sel,// ALU Selection
           output [31:0] ALU_Out, // ALU 8-bit Output
           output CarryOut, // Carry Out Flag
		   output ZeroOut	// Zero Flag
		   );
    reg [31:0] ALU_Result;
    wire [32:0] tmp;
    assign ALU_Out = ALU_Result; // ALU out
    assign tmp = {1'b0,A} + {1'b0,B};
    assign CarryOut = tmp[32]; // Carryout flag
	assign ZeroOut = (ALU_Result == 0); // Zero Flag
    always @(*)
    begin
        case(ALU_Sel)
        4'b0000: // Addition
           ALU_Result = A + B ;
        4'b0001: // Subtraction
           ALU_Result = A - B ;
        4'b0010: // Multiplication
           ALU_Result = A * B;
        4'b0011: // Division
           ALU_Result = A/B;
        4'b0100: // Logical shift left
           ALU_Result = A<<B;
         4'b0101: // Logical shift right
           ALU_Result = A>>B;
         4'b0110: // Rotate left
           ALU_Result = {A[30:0],A[31]};
         4'b0111: // Rotate right
           ALU_Result = {A[0],A[31:1]};
          4'b1000: //  Logical and
           ALU_Result = A & B;
          4'b1001: //  Logical or
           ALU_Result = A | B;
          4'b1010: //  Logical xor
           ALU_Result = A ^ B;
          4'b1011: //  Logical nor
           ALU_Result = ~(A | B);
          4'b1100: // Logical nand
           ALU_Result = ~(A & B);
          4'b1101: // Logical xnor
           ALU_Result = ~(A ^ B);
          4'b1110: // Less comparison
           ALU_Result = (A<B)?32'd0:32'd1 ;
          4'b1111: // Equal comparison
            ALU_Result = (A==B)?32'd1:32'd0 ;
          default: ALU_Result = A + B ;
        endcase
    end

endmodule


module extend(input  		[31:7] instr,
              input  		[1:0]  immsrc,
              output reg 	[31:0] immext);
    always @(*) 
    case(immsrc)
         // I−type
    2'b00:     immext = {{20{instr[31]}}, instr[31:20]};
		 // S−type (stores)
    2'b01:     immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
         // B−type (branches)
    2'b10:      immext = {{20{instr[31]}}, instr[7],  instr[30:25], instr[11:8], 1'b0};                          // J−type (jal)
		// J−type (branches)
	2'b11:      immext = {{12{instr[31]}}, instr[19:12],  instr[20], instr[30:21], 1'b0};
           
	default: 	immext = 32'bx; // undefined
    endcase
endmodule

