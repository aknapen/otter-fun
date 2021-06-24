`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: Pipelined OTTER CPU
// Module Name: OTTER_CPU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.10 - (Keefe Johnson, 1/14/2020) Added serial programmer.
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

 package cpu_types;
       typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011,
           NOP      = 7'b0000000
       } opcode_t;
       
       // struct for storing a given pipeline stage's instruction and PC value\
       typedef struct packed{
           opcode_t opcode;
           logic [4:0] rs1_addr;
           logic [4:0] rs2_addr;
           logic [4:0] rd_addr;
           logic rs1_used;
           logic rs2_used;
           logic rd_used;
           logic [3:0] alu_fun;
           logic memWrite;
           logic memRead2;
           logic regWrite;
           logic [1:0] rf_wr_sel;
           logic [2:0] mem_type;  //sign, size
           logic ld_haz;
           logic [31:0] pc;
           logic invalid;
           logic br_taken;
       } instr_t;

   endpackage

module OTTER_MCU(input CLK,
                input INTR,
                input EXT_RESET,  // CHANGED RESET TO EXT_RESET FOR PROGRAMMER
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR,
                input PROG_RX,  // ADDED PROG_RX FOR PROGRAMMER
                output PROG_TX  // ADDED PROG_TX FOR PROGRAMMER
);           
    
    import cpu_types::*;
      
    // ************************ BEGIN PROGRAMMER ************************ 

    wire RESET;
    wire [31:0] s_prog_ram_addr;
    wire [31:0] s_prog_ram_data;
    wire s_prog_ram_we;
    wire s_prog_mcu_reset;
    wire [31:0] mem_addr_after;
    wire [31:0] mem_data_after;
    wire [1:0] mem_size_after;
    wire mem_sign_after;
    wire mem_we_after;

    programmer #(.CLK_RATE(50), .BAUD(115200), .IB_TIMEOUT(200),
                 .WAIT_TIMEOUT(500))
        programmer(.clk(CLK), .rst(EXT_RESET), .srx(PROG_RX), .stx(PROG_TX),
                   .mcu_reset(s_prog_mcu_reset), .ram_addr(s_prog_ram_addr),
                   .ram_data(s_prog_ram_data), .ram_we(s_prog_ram_we));

    // ************************ END PROGRAMMER ************************ 

    instr_t DE_EX_instr;
    instr_t EX_MEM_instr;
    instr_t MEM_WB_instr;
        
    logic [31:0] IF_ID_pc; // transmits the PC between the IF and ID stages
    logic [31:0] WB_rfIn; // connect to din for the reg file
    logic jb_taken; // indicates if a jump or branch has been taken
    
    logic [31:0] MEM_aluResult; // holds the ALU result value inside the memory stage
    logic [31:0] WB_aluResult; // holds the ALU result value inside the writeback stage
    
    logic [31:0] mem_data; // MEM_DOUT2 wire
    
    wire intTaken;

    //=======================  BEGIN FETCH STAGE ===========================//

    // Inputs to IF module
    logic [31:0] jalr_pc, branch_pc, jump_pc; 
    logic [2:0] pc_source;
    logic ld_haz; // used to detect a load-use hazard

    // Outputs from IF module
    logic [31:0] fetch_pc;
    logic memRead1;

    InstructionFetch IF(.*); 

    //=======================  END FETCH STAGE ===========================//

    // PC REGISTER
    always_ff @(posedge CLK) // transfers PC from fetch to decode
    begin
        IF_ID_pc <= fetch_pc; // delay PC from fetch stage
    end
    
    
//======================= BEGIN DECODE STAGE ===========================//

    logic [31:0] IR;
    logic [31:0] DE_A, DE_B; // ALU  A and B that will be sent to the Execute Stage
    logic [31:0] rs2; // Need this later for mem
    logic [31:0] I_immed;
    instr_t instr;
    
    InstructionDecode ID(.*);


//======================= END DECODE STAGE ===========================//

    logic [31:0] EX_A, EX_B, EX_IR, EX_RS2, EX_I_immed;
    always_ff @(posedge CLK) // to push ALU inputs and instruction from Decode to Execute stage
        begin
            EX_A <= DE_A;
            EX_B <= DE_B;
            EX_IR <= IR;
            EX_RS2 <= rs2; // Need this later for mem
            EX_I_immed <= I_immed;
            DE_EX_instr <= instr; // push instruction from decode to execute stage
        end
      
//======================= BEGIN EXECUTE STAGE ===========================//

    logic [31:0] aluResult, MEM_RS2;
    logic int_pc;
    
    Execute EX(.*);    

//======================= END EXECUTE STAGE ===========================//
    
    logic [31:0] MEM_DIN2, MEM_I_immed;
    instr_t toMem; // intermediary instruction struct
    
    always_comb // logic to signal whether an instruction was the "source" of a load-use hazard
    begin
        toMem = DE_EX_instr;
        if (ld_haz) toMem.ld_haz = 1;
    end
        
    always_ff @(posedge CLK) // to push intsr_t through the pipeline stages
    begin
        EX_MEM_instr <= toMem;
        EX_MEM_instr.br_taken <= jb_taken;
        MEM_aluResult <= aluResult;   
        MEM_DIN2 <= MEM_RS2; // Need this for din2 into memory
        MEM_I_immed <= EX_I_immed; // Need this for CSR
    end
    
//======================= BEGIN MEMORY STAGE ===========================//
    
    Memory MEM(.*);
    
//======================= END MEMORY STAGE ===========================//

    logic [31:0] WB_I_immed;
    
    always_ff @(posedge CLK) // to push intsr_t through the pipeline stages and result of the memory
    begin
        MEM_WB_instr <= EX_MEM_instr;
        WB_aluResult <= MEM_aluResult;
        WB_I_immed <= MEM_I_immed;
    end
                               
//======================= BEGIN WRITEBACK STAGE ===========================//
    
    // Technically, the writeback stage uses the register file since it writes back to the registers 
    
    //Creates 4-to-1 multiplexor used to select reg write back data
    // Mult4 to 1 ( PC+4, CSR reg, dout2 from mem, alu result, sel = wb_sel from decoder, out = finished register in
    
    // WB_rfIn is connected to the reg file 
    
        WriteBack WB(.*);

    // ************************ BEGIN PROGRAMMER ************************ 

    assign mem_addr_after = s_prog_ram_we ? s_prog_ram_addr : MEM_aluResult;  // 2:1 mux
    assign mem_data_after = s_prog_ram_we ? s_prog_ram_data : MEM_DIN2;  // 2:1 mux
    assign mem_size_after = s_prog_ram_we ? 2'b10 : EX_MEM_instr.mem_type[1:0];  // 2:1 mux
    assign mem_sign_after = s_prog_ram_we ? 1'b0 : EX_MEM_instr.mem_type[2];  // 2:1 mux
    assign mem_we_after = s_prog_ram_we | EX_MEM_instr.memWrite;  // or gate
    assign RESET = s_prog_mcu_reset | EXT_RESET;  // or gate

    // ************************ END PROGRAMMER ************************               
                           
    // ^ CHANGED aluResult to mem_addr_after FOR PROGRAMMER
    // ^ CHANGED B to mem_data_after FOR PROGRAMMER
    // ^ CHANGED memWrite to mem_we_after FOR PROGRAMMER
    // ^ CHANGED IR[13:12] to mem_size_after FOR PROGRAMMER
    // ^ CHANGED IR[14] to mem_sign_after FOR PROGRAMMER
     

    //MMIO /////////////////////////////////////////////////////           
    assign IOBUS_ADDR = mem_addr_after;  // CHANGED FROM aluResult TO mem_addr_after FOR PROGRAMMER
    assign IOBUS_OUT = mem_data_after;  // CHANGED FROM B TO mem_data_after FOR PROGRAMMER 
         
            
endmodule
