`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: J. Callenes
// 
// Create Date: 06/07/2018 04:21:54 PM
// Design Name: 
// Module Name: ProgCount
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

import cpu_sizes::*;

module ProgCount(
    input PC_CLK,
    input PC_RST,
    input PC_LD,
    input logic [31:0] PC_IN,
    output logic [31:0][INSTR_WINDOW-1:0] PC_OUT=0
    );
    
    integer i = 0;
    always_ff @(posedge PC_CLK)
    begin
        if (PC_RST == 1'b1)
            begin
                begin
                for (i=0; i<INSTR_WINDOW;i++)
                begin
                    PC_OUT[i] <= 0+(4*i); // reset to initial instruction window
                end
            end
        else if (PC_LD == 1'b1)
            begin
                for (i=0; i<INSTR_WINDOW;i++)
                begin
                    PC_OUT[i] <= PC_IN+(4*i); // set all necessary PC values
                end
            end
    end
    
endmodule
