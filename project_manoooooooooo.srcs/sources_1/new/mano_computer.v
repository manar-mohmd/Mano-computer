`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/29/2024 11:25:40 PM
// Design Name: 
// Module Name: mano_computer
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module SEQ_COUNTER(CLR, INC, CLK, OUT);

	input CLR, INC, CLK;
	output reg [2:0] OUT;
	
	initial OUT = 0;

	always @ (posedge CLK)
		begin
			if(CLR)
				OUT = 3'b000;
			else if(INC)
				OUT = OUT + 3'b001;
		end	
endmodule


module SEQ_ARCH (CLR, INC, T, D);
input [7:0] T, D;
output CLR, INC;
wire   x1, x2, y1, y2;
assign x1= T[5];
assign x2 = D[0] | D[1] | D[2];
assign y1 = x1 & x2;
assign y2 = D[4] & T[4]; 
assign CLR = y1 | y2;
assign INC = ~CLR;
endmodule
//////////////////////////                                     end SQ                               ///////////////////

module AR_REG(Q, Data, LD, CLR, INC, CLK);

	output reg [3:0] Q;
	input [3:0] Data;
	input LD, CLR, INC, CLK;
	
	initial Q = 0;
	
	always @(posedge CLK)
	
	begin
		if(LD)
			Q = Data;
			else if (CLR) 
			Q = 0;
		end
endmodule


module AR_ARCH(LD, CLR, INC, T, D, J);
input J;
input [7:0] D, T;
output LD, CLR, INC;
wire D7n;
wire a1, a2, a3;
assign D7n = ~D[7];
assign a1 =  T[0];
assign a2 = D7n & J & T[3];
assign a3 = T[2]; 
assign LD = a1 | a2 | a3;
assign CLR =0;
assign INC = 0;
endmodule

////////////////////////////////////////////      end ar               ///////////////////////////////////////////////////////////


module PROG_COUNTER(Q, INC, LD, CLR, Data, CLK);

	output reg [3:0] Q;
	input [3:0] Data;
	input INC, CLK, LD, CLR;
	
	initial Q = 0;
	
	always @(posedge CLK)
	
	begin
		 if(INC)
			Q = Q + 1;
			else if(LD)
			Q = Data;
			else if (CLR)
			Q =0;
	end
	
endmodule

module PC_ARCH ( INC, LD, CLR, D, T);
output  LD, INC, CLR;
input [7:0] T, D;

 assign LD = D[4] & T [4];
 assign INC = T[1];
 assign CLR =0;
 endmodule
 

//////////////////////////////////          end pc                               ////////////////////////////////////////////
module DATA_REG(Q, Data, LD, CLR, INC, CLK);

    initial Q = 0;
 
	output reg [7:0] Q;
	input [7:0] Data;
	input LD, CLR, INC, CLK;
	
	always @(posedge CLK)
	begin
		 if(LD)
			Q = Data;
		end	
endmodule

module DR_ARCH(LD, CLR, INC, T, D);
input [7:0] D, T;
output LD, CLR, INC;
wire d;
assign d = D[0] & D[1] & D[2];
assign LD = d | T[4];
assign CLR =0;
assign INC =0;
endmodule

/////////////////////////////////////////////     end dr                ///////////////////////////////////////


module ACC(Q, INC, Data, LD, CLK, CLR);

 	output reg [7:0] Q;
	input [7:0] Data;
	input INC, LD, CLK, CLR;
	
	 initial Q = 0;
	
	always @(posedge CLK)
	begin
		if(CLR)
			Q = 8'b0;
		else if(LD)
			Q = Data;
		else if(INC)
			Q = Q + 1;
	end	
endmodule

module ACC_ARCH (AND, ADD, LDA, COM, INC, LD, CLR, T, D, B, J);
 
 output AND, ADD, LDA, LD, CLR, COM, INC;
 input [7:0] T,D;
 input [15:0] B;
 input J;
 
 wire r;
 wire Jn;
  assign Jn = ~J;
  assign r = D[7] & Jn & T[3];
  
 assign AND = D[0] & T [5]; 
 assign ADD = D[1] & T[5];
 assign LDA  = D[2] & T[5];
 assign COM = B[9] & r;
 assign INC = B[5] & r;
 assign CLR = B[11] & r;
 assign LD = (AND | ADD | LDA | COM);
 endmodule
 /////////////////////////////////////////////                              end acc                          ///////////////////////////////////////////////////
module INS_REG(Q,Data, LD, CLR, INC, CLK);
 
     initial Q = 0;
  
     output reg [7:0] Q;
     input [7:0] Data;
     input LD, CLR, INC, CLK;
     
     always @(posedge CLK)
     begin
          if(LD)
             Q = Data;
         end    
 endmodule

module IR_ARCH(LD, CLR, INC, T, D);
 input [7:0] D, T;
 output LD, CLR, INC;
 assign LD = T[1];
 assign CLR =0;
 assign INC =0;
endmodule
/////////////////////////////////////////////////               end ir                //////////////////////////////////////////////////////////

     module ENCODER3x8(Sel, Ins);

   input [7:0] Ins;
   output [2:0] Sel;

   assign Sel =
      (Ins[7] == 1'b1) ? 3'b111:
      (Ins[6] == 1'b1) ? 3'b110:
      (Ins[5] == 1'b1) ? 3'b101:
      (Ins[4] == 1'b1) ? 3'b100:
      (Ins[3] == 1'b1) ? 3'b011:
      (Ins[2] == 1'b1) ? 3'b010:
      (Ins[1] == 1'b1) ? 3'b001:
      (Ins[0] == 1'b1) ? 3'b000: 3'bxxx;

endmodule
                      
///////////////////////////////////////////////////////            end encoder               //////////////////////////////////////////////////////////

module Decoder3to8(I, O);
	input [2:0] I;
	output [7:0] O;
	
	reg [7:0] O;
	
	always@(*)
		case (I)
			3'b000 : O<=8'b00000001;
			3'b001 : O<=8'b00000010;
			3'b010 : O<=8'b00000100;    
			3'b011 : O<=8'b00001000;
			3'b100 : O<=8'b00010000;
			3'b101 : O<=8'b00100000;
			3'b110 : O<=8'b01000000; 
			3'b111 : O<=8'b10000000;
		endcase

endmodule

///////////////////////////////////////////////////             end decoder                 ///////////////////////////////////////////////////

module D_FF_reset(Q, D, CLK,RESET);

	output reg Q;
	input D, CLK, RESET;

	always @(posedge CLK or posedge RESET) begin
		if(RESET)
			Q <= 1'b0;
		else 
			Q <= D;
	end	
endmodule

 ///////////////////////////////////////////////                end d_flip_flop              /////////////////////////////////////////////////////////////
 
 module MUX(d0,d1,d2,d3,d4,d5,d6,d7,sel,out);
 
     input [7:0] d0,d1,d2,d3,d4,d5,d6,d7;
     input [2:0] sel;
     output reg [7:0] out;
 
     always @* begin
         case(sel)
             3'b000: out = d0;
             3'b001: out = d1;
             3'b010: out = d2;   
             3'b011: out = d3;
             3'b100: out = d4;
             3'b101: out = d5;
             3'b110: out = d6;
             3'b111: out = d7;
         endcase
     end
 
 endmodule
///////////////////////////////////////////////////////////            end mux                      ///////////////////////////////////////////////////////////
 
 module BUS_ARCH(I, D, T, J);

	input  J;
	input [7:0] D;
	input [7:0] T;

	output [7:0] I;

	wire D7n = ~D[7];
	wire B1 ,B2, B3,B4;
	
	assign B1 = D[4] & T[4];
	assign B2= D[2]& T[5];
	assign B3=  D7n & J & T[3];
	assign B4= ((D[0] | D[1] | D[2]) & T[4]);
	
	
	assign I[0] = 0;                                                                  //EMPTY_NONE
	assign I[1] = B1;                                                                //AR as source
	assign I[2] = T[0];                                                             //PC as source
	assign I[3] = B2;                                                              //DR as source
	assign I[4] = 0 ;                                                             //AC as source_EMPTY
	assign I[5] = T[2] ;                                                         //IR as source
    assign I[6] = 0;                                                        //TR as source            
    assign I[7] = B3 | B4 | T[1] ;                                             //M[AR] as source

endmodule

////////////////////////////////////////                                end bus                             ///////////////////////////////////////////////

module ALU(AND, ADD, LDA, COM,E, cout, AC, DR, ACDATA);
    input AND, ADD, LDA, COM, E;  //instructions signals 
    input [7:0] AC, DR;
    
    output cout ;
    output [7:0]  ACDATA;
    
    wire [7:0] and8, add8, lda8, com8;   // turning the instructions signals into 16-bit signal to be anded 
   
    wire [7:0] AND1, AND2, AND3, AND4, SUM;         //AND gate of each instruction 
    wire CARRY;
    
    //AND OPERATION
    assign and8 = (AND ? 16'hff :16'b0);
	assign AND1 = AC & and8 & DR;
	
	//ADD OPERTION , {CARRY,SUM} is the result of a full-adder
	assign {CARRY,SUM} = AC + DR ;
	assign cout = CARRY;
	
	assign add8 = (ADD ? 8'hff :8'b0);
	assign AND2 = SUM & add8 ;
	
	//LDA OPERATION (load AC with DR content )
	assign lda8 = (LDA ? 8'hff :8'b0);
	assign AND3 = lda8 & DR ;
	
	//CMA OPERATION (Complement AC content)
	assign com8 = (COM? 8'hff : 8'b0);
	assign AND4 = (~AC) & com8;

    assign ACDATA = AND1 | AND2| AND3| AND4;   // all and gates into OR gate its result in AC
endmodule

///////////////////////////////////////////////////////             end ALU                          /////////////////////////////////////////////////////
module MEMORY (CLK, read, AR, write, INDATA, OUTDATA );
    input CLK,read,write;
    input [7:0]INDATA;
    input [3:0] AR;
    output reg [7:0] OUTDATA;
    
    reg [3:0] MEM[15:0];
        
    initial begin
    MEM[0] = 8'h78;                              // CLA
    MEM[1] = 8'h74;                             // CMA
    MEM[2] = 8'h72;                            // INC
    MEM[3] = 8'h0A;                           // AND
    MEM[4] = 8'h1B;                          // ADD
    MEM[5] = 8'h2C;                         // LDA
    MEM[6] = 8'h47;                        // BUN
    MEM[7] = 8'h8D;                       // AND INDIRECT
    MEM[8] = 8'h9E;                      // ADD INDIRECT
    MEM[9] = 8'hAF;                     // LDA INDIRECT
    MEM[10] = 8'hC4;                   // BUN INDIRECT 
    MEM[11] = 8'hFF;
    MEM[12] = 8'hFC; 
    MEM[13] = 8'h19;
    MEM[14] = 8'h09;
    MEM[15] = 8'h0B;
    
    end
    
    always@* begin
        if (write)
            MEM[AR] = INDATA;
        else if(read)
            OUTDATA = MEM[AR];
        end
        
endmodule


module MEMORY_ARCH ( READ, WRITE, T, D, J);

	input  J;
	input [7:0] D;
	input [7:0] T;
	output  READ, WRITE;

	wire Dn7;
	wire M1, M2, M3,M4, M5;

	assign Dn7 = ~D[7];

	assign M1 =  T[1];
	assign M2 = Dn7 & J & T[3];
	assign M3= D[0] | D[1] | D[2];
	assign M4= M3 & T[4];
	
	assign READ = M1 | M2 | M4;
    assign WRITE =0;
endmodule
////////////////////////////////////////////////////////                 end memory                         ///////////////////////////////////////

module decoder (value, timing_signals);
input [2:0] value;
output wire [7:0] timing_signals;
assign timing_signals =( value == 4'b0000) ? 8'h01:
                       ( value == 4'b0001) ? 8'h02:
                       ( value == 4'b0010) ? 8'h04:
                       ( value == 4'b0011) ? 8'h08:
                       ( value == 4'b0100) ? 8'h10:
                       ( value == 4'b0101) ? 8'h20:
                       ( value == 4'b0110) ? 8'h40:                   
                       ( value == 4'b0111) ? 8'h80: 8'hxx;
                                            
                      endmodule
///////////////////////////////////                                  end decoder                                 ///////////////////////////////////////////


module CONTROL_UNIT (CLK, T, AC, IR, 
CLRSEQ, INCSEQ, LDAR, CLRAR, INCAR,
 INCPC, LDPC, CLRPC, LDDR, CLRDR, INCDR,
AND, ADD, LDA, COM, INCAC, LDAC, CLRAC,
LDIR, CLRIR, INCIR, READ, WRITE, E, I, J, D); 

input CLK;
	input [7:0] AC, IR;
	input [7:0] T, D;
	output LDAR, CLRAR, INCAR,
           CLRPC, INCPC, LDPC,            
           LDDR, CLRDR, INCDR,
           LDAC, CLRAC,INCAC,
           LDIR, CLRIR, INCIR, 
           READ, WRITE,
           COM, LDA, ADD, AND,
           CLRSEQ, INCSEQ, 
           E,J;
    output [7:0] I;
    wire [7:0] D;
        wire [3:0] B = IR [3:0];
        wire [2:0] OPCODE = IR [6:4];
        wire J = IR[7];
        wire r;
        wire Jn;
        assign Jn =J;
        assign r = D[7] & T [3] & Jn;
                     
                  //// DECODER   
                     
          Decoder3to8 decode( OPCODE, D[7:0]);         /// I,O   
          
          ///////// AR CONTROL
          AR_ARCH ar(LDAR, CLRAR, INCAR, T, D, J);
          
          /////////// PC CONTROL 
          PC_ARCH pc( INCPC, LDPC, CLRPC, D, T);
           
           /////////// DR CONTROL 
           DR_ARCH dr(LDDR, CLRDR, INCDR, T, D);
           
           /////// AC CONTROL
           ACC_ARCH ac(AND, ADD, LDA, COM, INCAC, LDAC, CLRAC, T, D, B, J );
           
           ///////////////  IR CONTROL
           IR_ARCH ir     ( LDIR, CLRIR, INCIR, T, D);
           
           /////////////// MEMORY CONTROL
           MEMORY_ARCH mem( READ, WRITE, T, D, J);
           
           /////////////// SEQ CONTROL
           SEQ_ARCH seq(CLRSEQ, INCSEQ, T, D);
           
           ///////////////// BUS CONTROL  
          BUS_ARCH bus(I, D, T, J);  
                     
                    
       
        endmodule
        
        ////////////////////////                END CONTROL UNIT            ///////////////////////////////////////
        
        
        
        
         module MAIN(input CLK,
             /*
             , output E,
             output reg[7:0] AC,DR
             */
             output wire [7:0] DR, AC, IR, MEM,
             output wire [3:0] PC, AR,
             output wire [7:0] D,
             output wire [7:0] T,
             output wire [2:0] OUTSEQ,
             output wire [7:0] Data,
             output wire [2:0] sel, Sel,
             output wire [7:0] I ,
             output wire J,
             output wire E
             );
             
            wire    LDAR, CLRAR, INCAR,
                   INCPC, LDPC, CLRPC,                                                       
                   LDDR,CLRDR, INCDR,
                   INCAC, LDAC,CLRAC,
                   LDIR, CLRIR, INCIR,
                   READ, WRITE,       
                   AND,ADD, LDA, COM,
                   CLRSEQ,INCSEQ;
            
            
            
            wire  cout; 
            assign E =cout ; 
            /////////////////////////
            wire [15:0] m0, m1, m2, m3, m4 ,m5,m6, m7;  // (reg) input wires of the bus selection 
            
            //MEMORy
 MEMORY ram(CLK,READ,AR,WRITE,Data,MEM);   //module RAM(CLK,read,AR,write,INDATA,OUTDATA );
                                    assign m7 = MEM;
                                                                                                            
            // AR REG-                                                   
AR_REG ar  (AR,Data[3:0],LDAR,CLRAR, INCAR, CLK);    //instatiating AR reg.|PC(Q, INR, Data, Load, CLK, CLR);                                            
            assign m1 = {4'b0000, AR}; 
            
            // PC REG 
PROG_COUNTER pc         (PC,INCPC, LDPC, CLRPC, Data[3:0],CLK); 
            assign m2 = {4'b0000, PC};
            
            // DR ERG -
DATA_REG dr         (DR,Data,LDDR, CLRDR, INCDR, CLK);                          
            assign m3=DR;
            
            
            // ALU & AC REG 
            wire [7:0] ACDATA;                                                   ///ACDATA is INPUT OF AC reg. coming from ALU as an output not common bus
 ALU alu       (AND,ADD,LDA,COM,E,cout ,AC, DR, ACDATA);
ACC ac      (AC,INCAC,ACDATA,LDAC,CLK,CLRAC);                  //AC(Q, INR, Data, LD, CLK, CLR);
            assign m4 = AC;    
            
            
            // IR & IR control
            
 INS_REG ir        (IR,Data,LDIR,CLRIR,INCIR, CLK);                  
                           
            assign m5 = IR;                              
            
            // CONTROL UNIT 
 CONTROL_UNIT cu      (CLK, T, AC, IR,
                       CLRSEQ, INCSEQ,
                        LDAR, CLRAR, INCAR,
                        INCPC, LDPC,CLRPC,
                        LDDR, CLRDR, INCDR,
                        AND,ADD, LDA, COM,
                        INCAC, LDAC, CLRAC,
                         LDIR, CLRIR, INCIR, 
                         READ, WRITE,
                        E,I, J, D);                                                                                                                                                                                                           
                                      
                                         
        
     /////////BUS 
ENCODER3x8 encoder    (Sel,I);                                            
 MUX mux       (m0,m1,m2,m3,m4,m5,m6,m7,sel,Data);  // MUX(d0,d1,d2,d3,d4,d5,d6,d7,sel,out);
                        
            // Sequence COUNTER 
                                                    // INCREMENT SC
            assign INCSEQ = ~CLRSEQ;                           
 SEQ_COUNTER seq       (CLRSEQ,INCSEQ ,CLK,OUTSEQ);  
 decoder deco      (OUTSEQ, T);                   
             
                        
        
                                                       
        endmodule

                                
        
                                                   
        
        
        