`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: J. Callenes
// 
// Create Date: 01/27/2019 08:37:11 AM
// Design Name: 
// Module Name: bram_dualport
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.10 - (Keefe Johnson, 1/14/2020) Corrected memory size comments.
//                 Commented out unused modules.
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

import cpu_sizes::*;
                                                                                                                            //func3
module OTTER_mem_byte(
    parameter ACTUAL_WIDTH=14;  //32KB     16K x 32
    parameter NUM_COL = 4;
    parameter COL_WIDTH = 8;
    
    input [31:0][INSTR_WINDOW-1:0] MEM_ADDR1;     //Instruction Memory Port
    input [31:0] MEM_ADDR2;     //Data Memory Port
    input MEM_CLK;
    input [31:0] MEM_DIN2;
    input MEM_WRITE2;
    input MEM_READ1;
    input MEM_READ2;
    input [31:0] IO_IN;
    input [1:0] MEM_SIZE;
    input MEM_SIGN;
    output logic [31:0][INSTR_WINDOW-1:0] MEM_DOUT1;
    output logic [31:0] MEM_DOUT2;
    output logic IO_WR;
);
    logic saved_mem_sign;
    logic [1:0] saved_mem_size;
    logic [31:0] saved_mem_addr2;
    
    wire [ACTUAL_WIDTH-1:0][INSTR_WINDOW-1:0] memAddr1;
    wire [ACTUAL_WIDTH-1:0] memAddr2;
    logic memWrite2;  
    logic [31:0] memOut2;
    logic [31:0] ioIn_buffer=0;
    logic [NUM_COL-1:0] weA;

    always_comb begin
        integer i;
        for (i=0;i<INSTR_WINDOW;i++)
        begin
            memAddr1[i] = MEM_ADDR1[ACTUAL_WIDTH+1:2][i];
        end
    end

    assign memAddr2 =MEM_ADDR2[ACTUAL_WIDTH+1:2];
    
    (* rom_style="{distributed | block}" *) 
    (* ram_decomp = "power" *) logic [31:0] memory [0:2**ACTUAL_WIDTH-1];
    
    initial begin
        $readmemh("otter_memory.mem", memory, 0, 2**ACTUAL_WIDTH-1);
    end 
    

    always_comb
    begin
        case(MEM_SIZE)
                0:  weA = 4'b1 << MEM_ADDR2[1:0];   //sb
                1:  weA =4'b0011 << MEM_ADDR2[1:0];  //sh      //Not supported if across word boundary
                2:  weA=4'b1111;                    //sw        //Not supported if across word boundary
                default: weA=4'b0000;
        endcase
    end
    integer i,j;
    always_ff @(posedge MEM_CLK) begin
        //PORT 2  //Data
        if(memWrite2)
        begin
            j=0;
            for(i=0;i<NUM_COL;i=i+1) begin
                if(weA[i]) begin
                        case(MEM_SIZE)
                            0: memory[memAddr2][i*COL_WIDTH +: COL_WIDTH] <= MEM_DIN2[7:0]; //MEM_DIN2[(3-i)*COL_WIDTH +: COL_WIDTH];
                            1: begin 
                                    memory[memAddr2][i*COL_WIDTH +: COL_WIDTH] <= MEM_DIN2[j*COL_WIDTH +: COL_WIDTH];
                                    j=j+1;
                               end
                            2: memory[memAddr2][i*COL_WIDTH +: COL_WIDTH] <= MEM_DIN2[i*COL_WIDTH +: COL_WIDTH];
                            default:  memory[memAddr2][i*COL_WIDTH +: COL_WIDTH] <= MEM_DIN2[i*COL_WIDTH +: COL_WIDTH];
                        endcase
                end
            end
         end
        if(MEM_READ2)
            memOut2 <= memory[memAddr2]; 
            
        //PORT 1  //Instructions
        if(MEM_READ1)
        begin
            integer i;
            for (i=0;i<INSTR_WINDOW; i++;)
            begin
                MEM_DOUT1[i] = memory[memAddr1[i]];
            end
        end
            // MEM_DOUT1 <= memory[memAddr1];
//        begin
//            case (LD_HAZ)
//                0: MEM_DOUT1 <= memory[memAddr1];  
//                1: MEM_DOUT1 <= 32'b0;
//            endcase
//        end
        
        saved_mem_size <= MEM_SIZE;
        saved_mem_sign <= MEM_SIGN;
        saved_mem_addr2 <=MEM_ADDR2;
    end
    
    always_ff @(posedge MEM_CLK)
        if(MEM_READ2)
            ioIn_buffer<=IO_IN;       
 
//===  Second cycle ==== Post Processing ==============================
    logic [31:0] memOut2_sliced=32'b0;
   
    always_comb
    begin
            memOut2_sliced=32'b0;
  
            case({saved_mem_sign,saved_mem_size})
                0: case(saved_mem_addr2[1:0])
                        3:  memOut2_sliced = {{24{memOut2[31]}},memOut2[31:24]};      //lb     //endianess
                        2:  memOut2_sliced = {{24{memOut2[23]}},memOut2[23:16]};
                        1:  memOut2_sliced = {{24{memOut2[15]}},memOut2[15:8]};
                        0:  memOut2_sliced = {{24{memOut2[7]}},memOut2[7:0]};
                   endcase
                        
                1: case(saved_mem_addr2[1:0])
                        3: memOut2_sliced = {{16{memOut2[31]}},memOut2[31:24]};      //lh   //spans two words, NOT YET SUPPORTED!
                        2: memOut2_sliced = {{16{memOut2[31]}},memOut2[31:16]};
                        1: memOut2_sliced = {{16{memOut2[23]}},memOut2[23:8]};
                        0: memOut2_sliced = {{16{memOut2[15]}},memOut2[15:0]};
                   endcase
                2: case(saved_mem_addr2[1:0])
                        1: memOut2_sliced = memOut2[31:8];   //spans two words, NOT YET SUPPORTED!
                        0: memOut2_sliced = memOut2;      //lw     
                   endcase
                4: case(saved_mem_addr2[1:0])
                        3:  memOut2_sliced = {24'd0,memOut2[31:24]};      //lbu
                        2:  memOut2_sliced = {24'd0,memOut2[23:16]};
                        1:  memOut2_sliced = {24'd0,memOut2[15:8]};
                        0:  memOut2_sliced = {24'd0,memOut2[7:0]};
                   endcase
                5: case(saved_mem_addr2[1:0])
                        3: memOut2_sliced = {16'd0,memOut2};      //lhu //spans two words, NOT YET SUPPORTED!
                        2: memOut2_sliced = {16'd0,memOut2[31:16]};
                        1: memOut2_sliced = {16'd0,memOut2[23:8]};
                        0: memOut2_sliced = {16'd0,memOut2[15:0]};
                   endcase
            endcase
    end
 
    always_comb begin
        if(saved_mem_addr2 >= 32'h11000000)      
            MEM_DOUT2 = ioIn_buffer;  
        else 
            MEM_DOUT2 = memOut2_sliced;   
    end 

    always_comb begin
        IO_WR=0;
        if(MEM_ADDR2 >= 32'h11000000)
        begin       
            if(MEM_WRITE2) IO_WR = 1;
            memWrite2=0; 
        end
        else begin 
            memWrite2=MEM_WRITE2;
        end    
    end 
        
endmodule