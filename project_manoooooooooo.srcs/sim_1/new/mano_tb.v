`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/05/2024 11:03:06 PM
// Design Name: 
// Module Name: mano_tb
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


module MAIN_tb;
wire [7:0] DR, AC, IR, MEM;
wire [3:0] PC, AR;
wire [7:0] T,D ;
wire [2:0] OUTSEQ;
wire [7:0] Data;
wire [2:0] Sel;
wire [7:0] I;
wire J;
wire E;
reg CLK;

MAIN mano( CLK, DR, AC, IR, MEM, 
PC,AR,
T, D,
OUTSEQ,
Sel,
I,
J,
E
);

initial CLK = 0;
always #5 CLK = ~ CLK;
   
endmodule

