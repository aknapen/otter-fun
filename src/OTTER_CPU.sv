`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  Alexander Knapen
// 
// Create Date: 06/24/2021
// Design Name: Pipelined OTTER CPU
// Module Name: OTTER_CPU
// Project Name: Chelsea
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

package cpu_sizes;
    // instruction window size
    parameter INSTR_WINDOW = 2;
endpackage

import cpu_sizes::*;

module OTTER_MCU(
    input CLK,
    input INTR,
    input EXT_RESET,
    input [31:0] IOBUS_IN,
    output [31:0] IOBUS_OUT,
    output [31:0] IOBUS_ADDR,
    output logic IOBUS_WR,
);           
    //=======================  BEGIN FETCH STAGE ===========================//

    logic pc_wr; // pc enable signal
    logic [31:0][INSTR_WINDOW-1:0] pc_in, pc_out;

    ProgCount PC(.PC_CLK(CLK), .PC_RST(EXT_RESET), .PC_LD(pc_wr), 
                 .PC_DIN(pc_in), .PC_DOUT(pc_out));

    pc_in = pc_in + 4*(INSTR_WINDOW-1); // increments base of instr window

    //=======================  END FETCH STAGE ===========================//
    logic [31:0][INSTR_WINDOW-1:0] dec_pc;

    always_ff @(posedge CLK) // shift PC values into next stage
    begin
        dec_pc <= pc_out;
    end
            
endmodule
