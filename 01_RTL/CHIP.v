//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,                                                         //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata,                                                       //
    // finnish procedure                                                                        //
        output              o_finish,                                                           //
    // cache                                                                                    //
        input               i_cache_finish,                                                     //
        output              o_proc_finish                                                       //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any declaration

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, next_PC;
        wire mem_cen, mem_wen;
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        wire mem_stall;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submodules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (),          
        .rs1    (),                
        .rs2    (),                
        .rd     (),                 
        .wdata  (),             
        .rdata1 (),           
        .rdata2 ()
    );

    // Control Unit


    // Imm Gen


    // ALU


    // MULDIV Unit


    // Imm Mux


    // PC BranchType Mux


    // WB Mux

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
        end
        else begin
            PC <= next_PC;
        end
    end
endmodule

module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i];
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;
            for (i = 1; i < word_depth; i = i + 1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i = 1; i < word_depth; i = i + 1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module ControlUnit #(
        parameter BIT_W  = 32,
        parameter INST_W = 7,
        parameter R_type = 7'b0110011,
        parameter I_type = 7'b0010011,
        parameter S_type = 7'b0100011,
        parameter B_type = 7'b1100011,
        parameter U_type = 7'b0010111,
        parameter Load   = 7'b0000011,
        parameter Jal    = 7'b1101111,
        parameter Jalr   = 7'b1100111,
        parameter Ecall  = 7'b1110011
    )(
        input [INST_W-1:0] i_opcode,
        input [6:0] i_func7,
        input [3:0] i_func3,
        output [1:0] o_BranchType,  // BranchTypeType Type, 1: BType, 2: Jal, 3: Jalr
        output o_Branch,            // Branch or not
        output o_MemRead,           // To Data Memory
        output o_MemToReg,          // To WB mux
        output [3:0] o_ALU_opcode,  // To ALU Control Unit
        output o_MULDIV_opcode,     // To MULDIV Unit
        output o_MemWrite,          // To Data Memory
        output o_ALUSrc,            // To Rs2 and Imm selection mux
        output o_RegWrite,          // To Registers
        output o_finish,            // Finish signal
    )
    // regs declaration and assign to outputs
        reg [1:0] BranchType;
        assign o_BranchType = BranchType;

        reg Branch;
        assign o_Branch = branch;

        reg MemRead;
        assign o_MemRead = MemRead;

        reg MemToReg;
        assign o_MemToReg = MemToReg;

        reg [3:0] ALU_opcode;
        assign o_ALU_opcode = ALU_opcode;

        reg MULDIV_opcode;
        assign o_MULDIV_opcode = MULDIV_opcode;

        reg MemWrite;
        assign o_MemWrite = MemWrite;

        reg ALUSrc;
        assign o_ALUSrc = ALUSrc;

        reg RegWrite;
        assign o_RegWrite = RegWrite;

        reg finish;
        assign o_finish = finish;

        always @(*) begin
            case(i_opcode)
                R_type  : begin
                    
                end
                I_type  : begin
                end
                S_type  : begin
                end
                B_type  : begin
                end
                U_type  : begin
                end
                Load    : begin
                end
                Jal     : begin
                end
                Jalr    : begin
                end
                Ecall   : begin
                end
                default : begin
                    BranchType      = 2'd0;
                    Branch          = 1'b0;
                    MemRead         = 1'b0; 
                    MemToReg        = 1'b0;
                    ALU_opcode      = 4'd15;
                    MULDIV_opcode   = 1'b0;
                    MemWrite
                    ALUSrc
                    RegWrite
                    finish
                end
            endcase
        end

endmodule

// module ALU #(
//         parameter ADD    = 4'd0,
//         parameter SUB    = 4'd1,
//         parameter AND    = 4'd2,
//         parameter OR     = 4'd3,
//         parameter SLT    = 4'd4,
//         parameter SRA    = 4'd5,
//         parameter XOR    = 4'd6,
//         parameter SLL    = 4'd7,
//         parameter AUIPC  = 4'd8,
//         parameter JAL    = 4'd9,
//         parameter JALR   = 4'd10,
//         parameter ECALL  = 4'd11,
//         parameter DATA_W = 32
//     )(
//         input [DATA_W-1:0] i_PC,            // PC
//         input [3:0] i_ALU_opcode,           // opcode
//         input [DATA_W-1:0] i_A, i_B,        // input data
//         input swap_BranchType_answer,           // BranchType
//         output [DATA_W-1:0] o_ALU_result,   // output data
//         output o_zero                       // pc change and gate
//     );

// endmodule

module MULDIV_unit(
    // TODO: port declaration
    );
    // Todo: HW2
endmodule

module Cache #(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
            input i_proc_finish,
            output o_cache_finish,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W*4-1:0]  o_mem_wdata,
            input [BIT_W*4-1:0] i_mem_rdata,
            input i_mem_stall,
            output o_cache_available,
        // others
        input  [ADDR_W-1: 0] i_offset
    );

    assign o_cache_available = 0; // change this value to 1 if the cache is implemented

    //------------------------------------------//
    //          default connection              //
    assign o_mem_cen = i_proc_cen;              //
    assign o_mem_wen = i_proc_wen;              //
    assign o_mem_addr = i_proc_addr;            //
    assign o_mem_wdata = i_proc_wdata;          //
    assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
    assign o_proc_stall = i_mem_stall;          //
    //------------------------------------------//

    // Todo: BONUS

endmodule