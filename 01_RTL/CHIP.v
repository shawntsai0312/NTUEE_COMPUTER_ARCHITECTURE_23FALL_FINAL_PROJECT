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
    //FSM that tell PC to stop
    parameter S_single_cycle= 2'd0;     //  1 cycle
    parameter S_write_mem = 2'd1;       //  5 cycles
    parameter S_read_mem = 2'd2;        //  9 cycles
    parameter S_mul = 2'd3;             // 32 cycles

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
    // PC
    reg [BIT_W-1:0] PC, PC_nxt;
    reg PCcontrol;

    wire mem_cen, mem_wen;
    wire [BIT_W-1:0] mem_addr, memwdata, memrdata;
    wire mem_stall;
    wire MemRead, MemWrite;
    wire [1:0] BranchType;
    wire MemToReg, ALUSrc;
    wire [3:0] ALU_opcode;

    // ALU inputs
    wire [31:0] rs1Data, rs2Data;
    wire [31:0] imm;
    wire [31:0] aluIn2;

    // ALU and MULDIV
    wire zero;
    wire InvertZeroAns;
    wire [31:0] ALU_Result;
    wire [BIT_W-1:0] MULDIV_Result;
    wire [BIT_W-1:0] result;
    wire selectMULDIV ;
    
    wire [31:0] wdata;
    wire RegWrite;
    

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
    assign o_IMEM_addr = PC;
    assign o_IMEM_cen = 1;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file regFile(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (RegWrite & ~PCcontrol),          
        .rs1    (i_IMEM_data[19:15]),                
        .rs2    (i_IMEM_data[24:20]),                 
        .rd     (i_IMEM_data[11:7]),                 
        .wdata  (wdata),             
        .rdata1 (rs1Data),           
        .rdata2 (rs2Data)
    );
    assign o_DMEM_wdata = rs2Data;

    // Imm Gen
    immGenerator immGen(
        .inst (i_IMEM_data[31:0]),
        .out_immediate(imm[31:0])
    );

    // ALU input mux
    mux aluInputMux(
        .input_1(rs2Data),
        .input_2(imm),
        .control(ALUSrc),
        .o_result(aluIn2)
    );

    // Control Unit
    ControlUnit controlUnit(
        .i_opcode(i_IMEM_data[6:0]),
        .i_func7(i_IMEM_data[31:25]),
        .i_func3(i_IMEM_data[14:12]),
        .o_BranchType(BranchType),
        .o_InvertZeroAns(InvertZeroAns),
        .o_MemRead(MemRead),
        .o_MemToReg(MemToReg),
        .o_ALU_opcode(ALU_opcode),
        .o_selectMULDIV(selectMULDIV ),
        .o_MemWrite(MemWrite),
        .o_ALUSrc(ALUSrc),
        .o_RegWrite(RegWrite),
        .o_finish(o_proc_finish)
    );
    assign o_DMEM_cen = MemRead;
    assign o_DMEM_wen = MemWrite;
    assign o_finish = i_cache_finish;

    // ALU
    ALU alu(
        .i_PC(PC),
        .i_ALU_opcode(ALU_opcode),
        .i_A(rs1Data),
        .i_B(aluIn2),
        .i_InvertZeroAns(InvertZeroAns),
        .o_ALU_Result(ALU_Result),
        .o_zero(zero)
    );

    // MULDIV Unit
    MULDIVUnit mulDivUnit(
        .i_clk(i_clk),
        .i_A(rs1Data),
        .i_B(aluIn2),
        .i_ALU_opcode(ALU_opcode),
        .o_data(MULDIV_Result),
        .o_PCwait(PCwait)
    );

    // Result Mux
    mux resultMux(
        .input_1(ALU_Result),
        .input_2(MULDIV_Result),
        .control(selectMULDIV ),
        .o_result(result)
    );
    assign o_DMEM_addr = result;

    // WB Mux
    mux wbMux(
        .input_1(result),
        .input_2(i_DMEM_rdata),
        .control(MemToReg),
        .o_result(wdata)
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit

    // PC control
    always @(*) begin
        PCcontrol = i_DMEM_stall | PCwait;
        case(BranchType)
            2'd0 : PC_nxt = PCcontrol ? PC : (PC + 4);      // no branch
            2'd1 : PC_nxt = zero ? (PC + imm) : (PC + 4);   // B-type
            2'd2 : PC_nxt = PC + imm;                       // Jal
            2'd3 : PC_nxt = rs1Data + imm;                  // Jalr
        endcase
    end

    // next PC
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)   PC <= 32'h00010000; // Do not modify this value!!!
        else            PC <= PC_nxt;
        // $display("PC           = %h", PC);
        // $display("rs1Data      = %d", rs1Data);
        // $display("aluIn2       = %d", aluIn2);
        // $display("ALUopcode    = %d", ALU_opcode);
        // $display("ALUResult    = %d", ALU_Result);
        // $display("result       = %d", result);
        // $display("i_DMEM_rdata = %h", i_DMEM_rdata);
        // $display("write_data   = %d", wdata);
        // $display("i_IMEM_data  = %h", i_IMEM_data);
        // $display("\n");
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
        input [2:0] i_func3,
        output [1:0] o_BranchType,  // 1: BType, 2: Jal, 3: Jalr
        output o_InvertZeroAns,     // InvertZeroAns : beq = 0, bne = 1; blt = 0, bge = 1; ALU only support beq and blt
        output o_MemRead,           // To Data Memory
        output o_MemToReg,          // To WB mux
        output [3:0] o_ALU_opcode,  // To ALU Control Unit
        output o_selectMULDIV,      // To MULDIV Unit
        output o_MemWrite,          // To Data Memory
        output o_ALUSrc,            // To Rs2 and Imm selection mux
        output o_RegWrite,          // To Registers
        output o_finish             // Program Finish
    );
    // regs declaration and assign to outputs
        reg [1:0] BranchType;
        assign o_BranchType = BranchType;

        reg InvertZeroAns;
        assign o_InvertZeroAns = InvertZeroAns;

        reg MemRead;
        assign o_MemRead = MemRead;

        reg MemToReg;
        assign o_MemToReg = MemToReg;

        reg [3:0] ALU_opcode;
        assign o_ALU_opcode = ALU_opcode;

        reg selectMULDIV ;
        assign o_selectMULDIV = selectMULDIV ;

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
                BranchType      = 2'd0;
                InvertZeroAns   = 1'b0;
                MemRead         = 1'b0; 
                MemToReg        = 1'b0;
                case(i_func7)
                    7'b0000000 : begin
                        case(i_func3)
                            3'b000 : ALU_opcode = 4'd0;  // add
                            3'b111 : ALU_opcode = 4'd2;  // and 
                            3'b110 : ALU_opcode = 4'd3;  // or
                            3'b100 : ALU_opcode = 4'd8;  // xor
                            default: ALU_opcode = 4'd15; // ALU does nothing
                        endcase
                    end
                    7'b0100000 : begin
                        case(i_func3)
                            3'b000 : ALU_opcode = 4'd1;  // sub
                            default: ALU_opcode = 4'd15; // ALU does nothing
                        endcase
                    end
                    7'b0000001 : begin
                        case (i_func3)
                            3'b000:  ALU_opcode = 4'd6;  // mul
                            3'b100:  ALU_opcode = 4'd7;  // div
                            default: ALU_opcode = 4'd15; // ALU does nothing
                        endcase
                    end
                    default : ALU_opcode = 4'd15;        // ALU does nothing
                endcase
                case(i_func7)
                    7'b0000001 : selectMULDIV  = 1'b1;   // mul or div
                    default    : selectMULDIV  = 1'b0;   // MULDIV unit does nothing
                endcase
                MemWrite        = 1'b0;
                ALUSrc          = 1'b0;
                RegWrite        = 1'b1;
                finish          = 1'b0;
            end
            I_type  : begin
                BranchType      = 2'd0;
                InvertZeroAns   = 1'b0;
                MemRead         = 1'b0; 
                MemToReg        = 1'b0;
                case(i_func3)
                    3'b000  : ALU_opcode = 4'd0;                // addi => add
                    3'b010  : ALU_opcode = 4'd0;                // slti => slt
                    3'b101  : begin
                        case(i_func7)
                            7'b0100000 : ALU_opcode = 4'd5;     // srai => sra
                            default    : ALU_opcode = 4'd15;    // ALU does nothing
                        endcase
                    end
                    3'b001  : begin
                        case(i_func7)
                            7'b0000000 : ALU_opcode = 4'd9;     // slli => ALU sll
                            default    : ALU_opcode = 4'd15;    // ALU does nothing
                        endcase
                    end
                    default : ALU_opcode = 4'd15;               // ALU does nothing
                endcase
                selectMULDIV    = 1'b0;
                MemWrite        = 1'b0;
                ALUSrc          = 1'b1;
                RegWrite        = 1'b1;
                finish          = 1'b0;
            end
            S_type  : begin
                BranchType      = 2'd0;
                InvertZeroAns   = 1'b0;
                MemRead         = 1'b1; 
                MemToReg        = 1'b1;
                case(i_func3)
                    3'b010  : ALU_opcode = 4'd0;  // add
                    default : ALU_opcode = 4'd15; // ALU does nothing
                endcase
                selectMULDIV    = 1'b0;
                MemWrite        = 1'b0;
                ALUSrc          = 1'b1;
                RegWrite        = 1'b0;
                finish          = 1'b0;
            end
            B_type  : begin
                BranchType      = 2'd1;
                InvertZeroAns   = 1'b0;
                case(i_func3)
                    3'b000  : InvertZeroAns = 1'b0; // beq (ALU support sub, if subAns==0, then it is equal)
                    3'b001  : InvertZeroAns = 1'b1; // bne (ALU does not support, invert the answer of beq)
                    3'b100  : InvertZeroAns = 1'b0; // blt (ALU support slt, if sltAns==1, then it is equal)
                    3'b101  : InvertZeroAns = 1'b1; // bge (ALU does not support, invert the answer of blt)
                    default : InvertZeroAns = 1'b0;
                endcase
                MemRead         = 1'b0; 
                MemToReg        = 1'b0;
                case(i_func3)
                    3'b000  : ALU_opcode = 4'd1;  // sub
                    3'b001  : ALU_opcode = 4'd1;  // sub
                    3'b100  : ALU_opcode = 4'd4;  // slt
                    3'b101  : ALU_opcode = 4'd4;  // slt
                    default : ALU_opcode = 4'd15; // ALU does nothing
                endcase
                selectMULDIV    = 1'b0;
                MemWrite        = 1'b0;
                ALUSrc          = 1'b1;
                RegWrite        = 1'b1;
                finish          = 1'b0;
            end
            Load    : begin
                BranchType      = 2'd0;
                InvertZeroAns   = 1'b0;
                MemRead         = 1'b1; 
                MemToReg        = 1'b1;
                case(i_func3)
                    3'b010  : ALU_opcode = 4'd0;  // add
                    default : ALU_opcode = 4'd15; // ALU does nothing
                endcase
                selectMULDIV    = 1'b0;
                MemWrite        = 1'b0;
                ALUSrc          = 1'b1;
                RegWrite        = 1'b1;
                finish          = 1'b0;
            end
            U_type  : begin
                BranchType      = 2'd0;
                InvertZeroAns   = 1'b0;
                MemRead         = 1'b0; 
                MemToReg        = 1'b0;
                ALU_opcode      = 4'd10;
                selectMULDIV    = 1'b0;
                MemWrite        = 1'b0;
                ALUSrc          = 1'b1;
                RegWrite        = 1'b1;
                finish          = 1'b0;
            end
            Jal     : begin
                BranchType      = 2'd2;
                InvertZeroAns   = 1'b0;
                MemRead         = 1'b0; 
                MemToReg        = 1'b0;
                ALU_opcode      = 4'd11;
                selectMULDIV    = 1'b0;
                MemWrite        = 1'b0;
                ALUSrc          = 1'b1;
                RegWrite        = 1'b1;
                finish          = 1'b0;
            end
            Jalr    : begin
                BranchType      = 2'd3;
                InvertZeroAns   = 1'b0;
                MemRead         = 1'b0; 
                MemToReg        = 1'b0;
                ALU_opcode      = 4'd12;
                selectMULDIV    = 1'b0;
                MemWrite        = 1'b0;
                ALUSrc          = 1'b1;
                RegWrite        = 1'b1;
                finish          = 1'b0;
            end
            Ecall   : begin
                BranchType      = 2'd0;
                InvertZeroAns   = 1'b0;
                MemRead         = 1'b0; 
                MemToReg        = 1'b0;
                ALU_opcode      = 4'd13; // ALU stops
                selectMULDIV    = 1'b0;
                MemWrite        = 1'b0;
                ALUSrc          = 1'b0;
                RegWrite        = 1'b0;
                finish          = 1'b1;
            end
            default : begin
                BranchType      = 2'd0;
                InvertZeroAns   = 1'b0;
                MemRead         = 1'b0; 
                MemToReg        = 1'b0;
                ALU_opcode      = 4'd15; // ALU does nothing
                selectMULDIV    = 1'b0;
                MemWrite        = 1'b0;
                ALUSrc          = 1'b0;
                RegWrite        = 1'b0;
                finish          = 1'b0;
            end
        endcase
    end
endmodule

module ALU #(
        parameter ADD    = 4'd0,
        parameter SUB    = 4'd1,
        parameter AND    = 4'd2,
        parameter OR     = 4'd3,
        parameter SLT    = 4'd4,
        parameter SRA    = 4'd5,
        parameter XOR    = 4'd8,
        parameter SLL    = 4'd9,
        parameter AUIPC  = 4'd10,
        parameter JAL    = 4'd11,
        parameter JALR   = 4'd12,
        parameter ECALL  = 4'd13,
        parameter DATA_W = 32
    )(
        input [DATA_W-1:0] i_PC,            // PC
        input [3:0] i_ALU_opcode,           // ALU opcode from Control unit
        input [DATA_W-1:0] i_A,             // input data A
        input [DATA_W-1:0] i_B,             // input data B
        input i_InvertZeroAns,              // InvertZeroAns : beq = 0, bne = 1; blt = 0, bge = 1; ALU only support beq and blt
        output [DATA_W-1:0] o_ALU_Result,   // output ALU result
        output o_zero                       // zero to pc control (branch)
    );

    // input and output handling
        reg [DATA_W-1:0] PC;
        reg [3:0] ALU_opcode;
        reg [DATA_W-1:0] operand_a;
        reg [DATA_W-1:0] operand_b;
        reg InvertZeroAns;
        
        always @(*) begin
            PC = i_PC;
            ALU_opcode = i_ALU_opcode;
            operand_a = i_A;
            operand_b = i_B;
            InvertZeroAns = i_InvertZeroAns;
        end

        reg [DATA_W-1:0] result;
        assign o_ALU_Result = result;

        reg zero;
        assign o_zero = zero;

        reg [DATA_W-1:0] temp;

    always @(*) begin
        temp = 0;
        case(ALU_opcode)
            ADD     : begin
                zero = 0;
                result = operand_a + operand_b;
            end
            SUB     : begin
                result = operand_a - operand_b;
                zero = InvertZeroAns ? (result[DATA_W-1:0] != 0) : (result[DATA_W-1:0] == 0);
            end
            AND     : begin
                zero = 0;
                result = operand_a & operand_b;
            end
            OR      : begin
                zero = 0;
                result = operand_a | operand_b;
            end
            SLT     : begin
                temp[DATA_W-1:0] = operand_a[DATA_W-1:0] - operand_b[DATA_W-1:0];
                result = temp[DATA_W-1];
                zero = InvertZeroAns ? (!result[0]) : (result[0]);
            end
            SRA     : begin
                zero = 0;
                result = $signed(operand_a) >>> operand_b;
            end
            XOR     : begin
                zero = 0;
                result = operand_a ^ operand_b;
            end
            SLL     : begin
                zero = 0;
                result = operand_a << operand_b;
            end
            AUIPC   : begin
                zero = 0;
                result = PC + operand_b;
            end
            JAL     : begin
                zero = 0;
                result = PC + 4;
            end
            JALR    : begin
                zero = 0;
                result = PC + 4;
            end
            ECALL   : begin
                zero = 0;
                result = 0;
            end
            default : begin
                zero = 0;
                result = 0;
            end
        endcase
    end  
endmodule

module immGenerator#(
        parameter BIT_W  = 32,
        parameter R_type = 7'b0110011,
        parameter I_type = 7'b0010011,
        parameter S_type = 7'b0100011,
        parameter B_type = 7'b1100011,
        parameter Load   = 7'b0000011,
        parameter U_type = 7'b0010111,
        parameter Jal    = 7'b1101111,
        parameter Jalr   = 7'b1100111,
        parameter Ecall  = 7'b1110011
    )(
        input  [31:0]  inst,
        output [31:0]  out_immediate
    );
    reg [31:0] immediate;
    assign out_immediate = immediate;

    always@(*)begin
        immediate[31:0] = 0;
        case(inst[6:0])
            I_type  : begin
                case (inst[14:12])
                    3'b001  : begin
                        immediate[4:0] = inst[24:20];
                        if(inst[24])    immediate[31:5] = 27'b111_1111_1111_1111_1111_1111_1111;
                        else            immediate[31:5] = 0;
                    end 
                    3'b101  : begin
                        immediate[4:0] = inst[24:20];
                        if(inst[24])    immediate[31:5] = 27'b111_1111_1111_1111_1111_1111_1111;
                        else            immediate[31:5] = 0;
                    end
                    default : begin
                        immediate[11:0] = inst[31:20];
                        if(inst[31])    immediate[31:12] = 20'b1111_1111_1111_1111_1111;
                        else            immediate[31:12] = 0;
                    end
                endcase
            end
            S_type  : begin
                immediate[4:0] = inst[11:7];
                immediate[11:5] = inst[31:25];
                if(inst[31])    immediate[31:12] = 20'b1111_1111_1111_1111_1111;
                else            immediate[31:12] = 0;
            end
            B_type  : begin
                immediate[4:1] = inst[11:8];
                immediate[11] = inst[7];
                immediate[10:5] = inst[30:25];
                immediate[12] = inst[31];
                if (inst[31])   immediate[31:13] = 19'b111_1111_1111_1111_1111;
                else            immediate[31:13] = 0;
            end
            U_type  : begin
                immediate[31:12] = inst[31:12];
                immediate[11:0] = 0;
            end
            Jal     : begin
                immediate[0] = 0;
                immediate[20] = inst[31];
                immediate[10:1] = inst[30:21];
                immediate[11] = inst[20];
                immediate[19:12] = inst[19:12];
                if (inst[31])   immediate[31:21] = 11'b111_1111_1111;
                else            immediate[31:21] = 0;
            end
            Jalr    : begin
                immediate[11:0 ] = inst[31:20];
                if (inst[31])   immediate[31:12] = 20'b1111_1111_1111_1111_1111;
                else            immediate[31:12] = 0;
            end
            Load    : begin
                immediate[11:0] = inst[31:20];
                if(inst[31])    immediate[31:12] = 20'b1111_1111_1111_1111_1111;
                else            immediate[31:12] = 0;
            end
            Ecall   : immediate[31:0] = 0;
            default : immediate[31:0] = 0;
        endcase
    end
endmodule

module mux(
    input [31:0] input_1,
    input [31:0] input_2,
    input control,
    output [31:0] o_result
    );
    reg [31:0] result;
    assign o_result = result;

    always @(*) begin
        if(control)  result = input_2;
        else         result = input_1;
    end
endmodule

module MULDIVUnit#(
        parameter MUL    = 4'd6,
        parameter DIV    = 4'd7,
        parameter DATA_W = 32
    )(
    // TODO: port declaration
    input                       i_clk,   // clock
    // input                       i_rst_n, // reset

    // input                       i_valid, // input valid signal
    input [DATA_W - 1 : 0]      i_A,     // input operand A
    input [DATA_W - 1 : 0]      i_B,     // input operand B
    input [         3 : 0]      i_ALU_opcode,  // instruction

    output [DATA_W - 1 : 0]     o_data,  // output value   O_MULDIV_Result
    output                      o_PCwait   // output valid signal
    );
    // Todo: HW2
    // Wires & Regs
    // state
    reg  [         1: 0] state, state_nxt; // remember to expand the bit width if you want to add more states!
    // load input
    reg  [  DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  [  DATA_W-1: 0] operand_b, operand_b_nxt;
    reg  muldivrst_n;
    reg  temp;
    reg  PCwait;
    // reg  [         2: 0] inst, inst_nxt;

    reg [          5: 0] counter, counter_nxt;//0-63 
    //output
    reg  [2*DATA_W-1: 0] out, out_nxt;
    reg oDone, oDone_nxt;
    reg [2*DATA_W-1:0] temp1, temp2;
    // Wire Assignments
    // Todo
    assign o_data = out_nxt;
    assign o_done = oDone;
    assign o_PCwait = PCwait;

    always @(*) begin
        operand_a_nxt = i_A;
        operand_b_nxt = i_B;
        muldivrst_n = (i_ALU_opcode == MUL) || (i_ALU_opcode == DIV);
    end
    always @ (*) begin
        case(i_ALU_opcode)
            MUL : begin
                if (counter == 1) begin 
                    if (out[0] == 0) out_nxt = out >> 1;
                    else begin
                        out_nxt = out >> 1;
                        out_nxt[63:31] = operand_b + out[63:32];
                    end
                end
                else begin
                    if(i_A[0])  out_nxt = {1'b0, i_B[31:0], i_A[31:1]};
                    else out_nxt = {33'b0, i_A[31:1]};
                end
            end
            DIV : begin
                if (counter < 31) begin
                    if (out[63:32] >= operand_b) begin
                        {temp, out_nxt[63:33]} = out[63:32] - operand_b;
                        out_nxt[32:0] = {out[31:0], 1'b1};
                    end
                    else begin
                        out_nxt = {out[62:0], 1'b0};
                    end
                end
                else if (counter == 0) begin
                    if (i_A[31] < i_B) out_nxt = {30'b0, i_A, 2'b0};
                    else out_nxt = {31'b0, i_A[30:0], 2'b01};
                end
                else begin
                    if (out[63:32] >= operand_b) begin
                        out_nxt[63:32] = out[63:32] - operand_b;
                        out_nxt[31: 0] = {out[30:0], 1'b1};
                    end
                    else begin
                        out_nxt = {out[63:32], out[30:0], 1'b0};
                    end
                end
            end
            default:    begin
                out_nxt = 0;
            end
        endcase
    end
    always @(*) begin
        if (counter < 31 && muldivrst_n) begin
            counter_nxt = counter + 1;
            PCwait = 1;
        end
        else begin
            counter_nxt = 0;
            PCwait      = 0;
        end
    end
    // Todo: Sequential always block
    always @(posedge i_clk) begin
        // $display("cnt = %d, unit_on = %d, PCwait= %d, MUL_result_nxt=%d, MUL_result = %d ",cnt,unit_on,PCwait,result_nxt,result);
        if (!muldivrst_n) begin
            operand_a   <= 0;
            operand_b   <= 0;
            counter     <= 0;
            out         <= 0;
        end
        else begin
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            counter     <= counter_nxt;
            out         <= out_nxt;
        end
    end
endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input  i_proc_cen,
            input  i_proc_wen,
            input  [ADDR_W-1:0]  i_proc_addr,
            input  [BIT_W-1: 0]  i_proc_wdata,
            output [BIT_W-1: 0]  o_proc_rdata,
            output o_proc_stall,
            input  i_proc_finish,
            output o_cache_finish,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1: 0]  o_mem_addr,
            output [BIT_W*4-1:0]  o_mem_wdata,
            input  [BIT_W*4-1:0]  i_mem_rdata,
            input  i_mem_stall,
            output o_cache_available,
        //others
            input  [ADDR_W-1: 0] i_offset
    );

    assign o_cache_available = 1; // change this value to 1 if the cache is implemented

    // //------------------------------------------//
    // //          default connection              //
    // assign o_mem_cen = i_proc_cen;              //
    // assign o_mem_wen = i_proc_wen;              //
    // assign o_mem_addr = i_proc_addr;            //
    // assign o_mem_wdata = i_proc_wdata;          //
    // assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
    // assign o_proc_stall = i_mem_stall;          //
    // //------------------------------------------//

    // Todo: BONUS
        //parameters:
        parameter S_IDLE       = 3'd0;
        parameter S_FIND       = 3'd1;
        parameter S_WB         = 3'd2;
        parameter S_ALLO       = 3'd3;
        parameter S_FINISH     = 3'd4;
        parameter block_number = 16;
        //regs
        reg [2:0] state, state_nxt;
        reg mem_cen, mem_wen;
        reg cen, cen_nxt, wen, wen_nxt;
        reg proc_stall;
        reg [4*BIT_W-1:0] data [0:block_number-1], data_nxt[0:block_number-1];
        reg [23:0] tag [0:block_number-1], tag_nxt[0:block_number-1];//i_addr[31:8]
        reg valid [0:block_number-1], valid_nxt[0:block_number-1];
        reg dirty [0:block_number-1], dirty_nxt[0:block_number-1];
        reg [3:0] index, index_nxt;
        reg [1:0] offset, offset_nxt;
        reg [3:0] counter;
        reg finish;
        reg hit;
        reg [ADDR_W-1:0] proc_addr, real_addr;

        //wire assignment
        assign o_mem_cen   = mem_cen;
        assign o_mem_wen   = mem_wen;
        assign o_mem_addr  = proc_addr + i_offset;
        assign o_mem_wdata = data[index];

        assign o_proc_stall   = proc_stall;
        assign o_proc_rdata   = data[index][ADDR_W*offset +: ADDR_W];
        assign o_cache_finish = finish;

        integer i;

        //always blocks
        always @(*) begin
            real_addr = i_proc_addr - i_offset;
            if (state == S_FINISH) begin
                index = counter;
                offset = 0;
                proc_addr = {tag[index], index, offset, 2'b00};
            end
            else if (state == S_WB) begin
                index = real_addr[7:4];
                offset = 0;
                proc_addr = {tag[index], index, offset, 2'b00};
            end
            else begin 
                proc_addr = real_addr;
                index = proc_addr[7:4];
                offset = proc_addr[3:2];
            end
            hit = (tag[index] == proc_addr[31:8]) & valid[index];
        end

        always @(*) begin
            case(state)
                S_IDLE: begin
                    finish = 0;
                    if (i_proc_finish) begin
                        state_nxt = S_FINISH;
                    end
                    else begin
                        if (i_proc_cen) begin
                            state_nxt = S_FIND;
                        end
                        else begin
                            state_nxt = S_IDLE;
                        end
                    end
                end
                S_FIND: begin
                    finish = 0;
                    if(hit) state_nxt = S_IDLE;
                    else begin
                        if (dirty[index]) begin
                            state_nxt = S_WB;
                        end
                        else state_nxt = S_ALLO;
                    end 
                end
                S_WB:   begin
                    finish = 0;
                    if(!i_mem_stall) begin
                        state_nxt = S_ALLO;
                    end
                    else state_nxt = S_WB;
                end
                S_ALLO: begin
                    finish = 0;
                    if(!i_mem_stall) begin
                        state_nxt = S_FIND;
                    end
                    else state_nxt = S_ALLO;
                end
                S_FINISH:   begin
                    if (counter == (block_number-1) && !i_mem_stall) begin //counter == block_number means all data is stored
                        state_nxt = S_IDLE;
                        finish = 1;
                    end
                    else begin
                        state_nxt = S_FINISH;
                        finish = 0;
                    end
                    
                end
                default: begin
                    state_nxt = state;
                    finish = 0;
                end
            endcase
        end

        always @(*) begin
            for (i = 0; (i < block_number) ; i = i + 1) begin
                dirty_nxt[i] = 0;
                tag_nxt[i] = 0;
                valid_nxt[i] = 0;
                data_nxt[i] = 0;
            end
            case(state)
                S_IDLE: begin
                    dirty_nxt[index] = dirty[index];
                    mem_cen = 0;
                    mem_wen = 0;
                    proc_stall = i_proc_cen;
                    cen_nxt = i_proc_cen;
                    wen_nxt = i_proc_wen;
                    tag_nxt[index] = tag[index];
                    valid_nxt[index] = valid[index];
                    data_nxt[index] = data[index];
                end
                S_FIND: begin
                    mem_cen = 0;
                    mem_wen = 0;
                    cen_nxt = cen;
                    wen_nxt = wen;
                    tag_nxt[index] = tag[index];
                    valid_nxt[index] = valid[index];
                    if (hit) begin
                        if (!wen) begin
                            data_nxt[index] = data[index];
                            dirty_nxt[index] = dirty[index];
                        end
                        else begin
                            dirty_nxt[index] = 1;
                            data_nxt[index] = data[index];
                            data_nxt[index][ADDR_W*offset +: ADDR_W] = i_proc_wdata;
                        end
                        proc_stall = 0;
                    end
                    else begin
                        proc_stall = 1;
                        data_nxt[index] = data[index];
                        if(wen) begin
                            dirty_nxt[index] = 1;
                        end
                        else begin
                            dirty_nxt[index] = dirty[index];
                        end
                    end
                end
                S_WB:   begin
                    data_nxt[index] = data[index];
                    dirty_nxt[index] = dirty[index];
                    mem_cen = 1;
                    mem_wen = 1;
                    proc_stall = 1;
                    cen_nxt = cen;
                    wen_nxt = wen;
                    tag_nxt[index] = tag[index];
                    valid_nxt[index] = valid[index];
                end
                S_ALLO: begin
                    data_nxt[index] = i_mem_rdata;
                    dirty_nxt[index] = dirty[index];
                    mem_cen = 1;
                    mem_wen = 0;
                    proc_stall = 1;
                    cen_nxt = cen;
                    wen_nxt = wen;
                    tag_nxt[index] = proc_addr[31:8];
                    valid_nxt[index] = 1;
                end
                S_FINISH:   begin
                    data_nxt[index] = data[index];
                    cen_nxt = cen;
                    wen_nxt = wen;
                    tag_nxt[index] = tag[index];
                    valid_nxt[index] = valid[index];
                    if (dirty[index]) begin
                        dirty_nxt[index] = 0;
                        mem_cen = 1;
                        mem_wen = 1;
                    end
                    else begin
                        dirty_nxt[index] = 0;
                        mem_cen = 0;
                        mem_wen = 0;    
                    end
                    proc_stall = 1;
                end
                default:    begin
                    data_nxt[index] = 0;
                    dirty_nxt[index] = dirty[index];
                    mem_cen = 0;
                    mem_wen = 0;
                    proc_stall = 0;
                    cen_nxt = 0;
                    wen_nxt = 0;
                    tag_nxt[index] = tag[index];
                    valid_nxt[index] = valid[index];
                end
            endcase
        end
        //finish counter

        always @(posedge i_clk) begin
            if (state == S_FINISH) begin
                if (!i_mem_stall) begin
                    counter <= counter + 1;
                end
                else begin
                    counter <= counter;
                end
            end
            else counter <= 0;
        end

        //sequential part
        always @(posedge i_clk or negedge i_rst_n) begin
            if (!i_rst_n) begin
                // reset
                state <= S_IDLE;
                for (i = 0; i < block_number; i = i + 1)begin
                    data[i] <= 0;
                    valid[i] <= 0;
                    tag[i] <= 0;
                    dirty[i] <= 0;
                
                end
                cen <= 0;
                wen <= 0; 
            end
            else begin
                state <= state_nxt;
                if (cen) begin
                    data[index] <= data_nxt[index];
                    valid[index] <= valid_nxt[index];
                    tag[index] <= tag_nxt[index];
                    dirty[index] <= dirty_nxt[index];
                end
                cen <= cen_nxt;
                wen <= wen_nxt;
            end
        end
endmodule