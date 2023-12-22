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
    parameter S_single_cycle= 2'd0;
    parameter S_write_mem = 2'd1;//5cycle
    parameter S_read_mem = 2'd2;//9cycle
    parameter S_mul = 2'd3;//32cycle

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, next_PC;
        wire mem_cen, mem_wen;
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        wire mem_stall;
        wire MemRead_select, MemWrite_select;
        wire [1:0] Branch_select;
        wire MemtoReg_select, ALUsrc_select,Regwrite_select;
        wire [3:0] ALU_opcode;
        wire swap_branch_answer;
        wire [31:0] rs1,rs2;
        wire [31:0] immediate;
        wire [31:0] rs2_select;
        wire zero;
        wire [31:0] ALU_result;
        wire [31:0] write_data_back_reg;
        reg PC_control; //set high to stop pc+4 (pc_wait | i_dmem_stall)
        wire [BIT_W - 1 :0] MULDIV_result;
        wire pc_wait;//muldiv unit's
        wire [BIT_W-1:0] single_ALU;
        wire Muldiv_ALU_choose;
        reg imem_cen,imem_cen_nxt;
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
    assign o_IMEM_addr = PC;
    assign o_IMEM_cen = 1; //imem_cen;//???不確定
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (Regwrite_select & ~PC_control),          
        .rs1    (i_IMEM_data[19:15]),                
        .rs2    (i_IMEM_data[24:20]),                 
        .rd     (i_IMEM_data[11:7]),                 
        .wdata  (write_data_back_reg),             
        .rdata1 (rs1),           
        .rdata2 (rs2)
    );

    imm_generator reg2(               //register generate immediate
        .inst (i_IMEM_data[31:0]),
        .out_immediate(immediate[31:0])
    );
    mux reg3(               //mutliplexer for rs2 and immediate
        .input_1(rs2),
        .input_2(immediate),
        .control(ALUsrc_select),
        .o_result(rs2_select)
    );
    MULDIV_unit reg5(
        .i_clk(i_clk),
        .i_A(rs1),
        .i_B(rs2_select),
        .i_ALU_opcode(ALU_opcode),
        .o_data(MULDIV_result),
        .o_pc_wait(pc_wait)
    );
    mux control(
        .input_1(single_ALU),
        .input_2(MULDIV_result),
        .control(Muldiv_ALU_choose),
        .o_result(ALU_result)
        );
    mux reg6(
        .input_1(ALU_result),
        .input_2(i_DMEM_rdata),
        .control(MemtoReg_select),
        .o_result(write_data_back_reg)
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

 always @(*)begin
        imem_cen = 1;
        // if (!PC_control) begin
        //     imem_cen_nxt = 0;
        // end
    end
    //PC control
    always @(*) begin
        PC_control = i_DMEM_stall | pc_wait;
        case(Branch_select)
            2'b00:  begin //no branch
                next_PC = PC_control ? PC : (PC + 4);
            end
            2'b01:  begin //B-type
                if(zero) begin 
                    next_PC = PC + immediate;
                end
                else next_PC = PC + 4;
            end
            2'b10: begin //JAL
                next_PC = PC + immediate;
            end
            2'b11: begin //JALR
                next_PC = rs1 + immediate;
            end
        endcase
    end

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
module  imm_generator#(
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
    reg    [31:0]  immediate;
    assign out_immediate = immediate;
    always@(*)begin
        immediate[31:0] = 0;
    case(inst[6:0])
        I_type:begin
            case (inst[14:12])
                3'b001:begin
                    immediate[4:0] = inst[24:20];
                    if(inst[24] == 1) immediate[31:5] = 27'b111111111111111111111111111;
                    else immediate[31:5] = 0;
                    // immediate[31:5] = (inst[24]==1)?20'b111111111111111111111111111:0;

                end 
                3'b101:begin
                    immediate[4:0] = inst[24:20];
                    if(inst[24] == 1) immediate[31:5] = 27'b111111111111111111111111111;
                    else immediate[31:5] = 0;
                    // immediate[31:5] = (inst[24]==1)?20'b111111111111111111111111111:0;
                end
                default:begin
                    immediate[11:0] = inst[31:20];
                    if(inst[31] == 1) immediate[31:12] = 20'b11111111111111111111;
                    else immediate[31:12] = 0;
                    // immediate[31:12] = (inst[31]==1)? 20'b11111111111111111111 : 0;
                end
        endcase
        end
        S_type:begin
            
            immediate[4:0] = inst[11:7];
            immediate[11:5] = inst[31:25];
            if(inst[31] == 1) immediate[31:12] = 20'b11111111111111111111;
            else immediate[31:12] = 0;
            // immediate[31:12] = (inst[31]==1)? 20'b11111111111111111111:0;
        end
        B_type:begin
            // immediate[0] = 0;//dont care
            immediate[4:1] = inst[11:8];
            immediate[11] = inst[7];
            immediate[10:5] = inst[30:25];
            immediate[12] = inst[31];
            if (inst[31] == 1) immediate[31:13] = 19'b1111111111111111111;
            else immediate[31:13] = 0;
            // immediate[31:13] = (instruction[31]==1)? 19'b1111111111111111111: 0;  
        end
        U_type:begin
            immediate[31:12] = inst[31:12];
            immediate[11: 0] = 12'b0;
        end
        Jal:begin
            immediate[0    ] = 0;
            immediate[20   ] = inst[31];
            immediate[10: 1] = inst[30:21];
            immediate[11   ] = inst[20];
            immediate[19:12] = inst[19:12];
            if (inst[31] == 1) immediate[31:21] = 11'b11111111111;
            else immediate[31:21] = 0;
            // immediate[31:21] = (instruction[31]==1)? 11'b11111111111:0;
        end
        Jalr:begin
            immediate[11:0 ] = inst[31:20];
            if (inst[31] == 1) immediate[31:12] = 20'b11111111111111111111;
            else immediate[31:12] = 0;
            // immediate[31:12] = (instruction[31]==1)? 20'b11111111111111111111 : 0;
        end
        Load:begin
            immediate[11:0] = inst[31:20];
            if(inst[31] == 1) immediate[31:12] = 20'b11111111111111111111;
            else immediate[31:12] = 0;
        end
        Ecall:begin
            immediate[31:0] = 0;
        end
        default: immediate[31:0] = 0;
    endcase
    
    end
endmodule

module mux(input_1,input_2,control,o_result);
    input[31:0] input_1,input_2;
    input control;
    output[31:0] o_result;
    reg [31:0] result;
    assign o_result = result;
    always@(*)begin
        // case(control)
        //     1'b0: result = input_1;
        //     1'b1: result = input_2;
        // endcase
        if(control)  result = input_2;
        else         result = input_1;
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

module MULDIV_unit#(
    parameter DATA_W = 32
)
    (
    // TODO: port declaration
    input                       i_clk,   // clock
    // input                       i_rst_n, // reset

    // input                       i_valid, // input valid signal
    input [DATA_W - 1 : 0]      i_A,     // input operand A
    input [DATA_W - 1 : 0]      i_B,     // input operand B
    input [         3 : 0]      i_ALU_opcode,  // instruction

    output [DATA_W - 1 : 0]     o_data,  // output value   O_MULDIV_result
    output                      o_pc_wait   // output valid signal
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
    reg  pc_wait;
    // reg  [         2: 0] inst, inst_nxt;

    reg [          5: 0] counter, counter_nxt;//0-63 
    //output
    reg  [2*DATA_W-1: 0] out, out_nxt;
    reg oDone, oDone_nxt;
    reg [2*DATA_W-1:0] temp1, temp2;
// Wire Assignments
    // Todo
    assign o_adta = out_nxt;
    assign o_done = oDone;
    assign o_pc_wait = pc_wait;


     always @(*) begin
        operand_a_nxt = i_A;
        operand_b_nxt = i_B;
        if ((i_ALU_opcode == 4'b0110) || (i_ALU_opcode == 4'b0111)) begin 
            muldivrst_n = 1;
        end
        else begin
            muldivrst_n = 0;
        end
    end
    always @ (*) begin
        case(i_ALU_opcode)
            4'b0110: begin
                if (counter == 1) begin 
                    if (out[0] == 0) out_nxt = out >> 1;
                    else begin
                        out_nxt = out >> 1;
                        out_nxt[63:31] = operand_b + out[63:32];
                    end
                end
                else begin
                    if(i_A[0] == 1)  out_nxt = {1'b0, i_B[31:0], i_A[31:1]};
                    else out_nxt = {33'b0, i_A[31:1]};
                end
            end
            4'b0111:begin
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
            pc_wait = 1;
        end
        else begin
            counter_nxt = 0;
            pc_wait     = 0;
        end
    end
    // Todo: Sequential always block
    always @(posedge i_clk) begin
        // $display("cnt = %d, unit_on = %d, pc_wait= %d, MUL_result_nxt=%d, MUL_result = %d ",cnt,unit_on,pc_wait,result_nxt,result);
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

// module Cache #(
//         parameter BIT_W = 32,
//         parameter ADDR_W = 32
//     )(
//         input i_clk,
//         input i_rst_n,
//         // processor interface
//             input i_proc_cen,
//             input i_proc_wen,
//             input [ADDR_W-1:0] i_proc_addr,
//             input [BIT_W-1:0]  i_proc_wdata,
//             output [BIT_W-1:0] o_proc_rdata,
//             output o_proc_stall,
//             input i_proc_finish,
//             output o_cache_finish,
//         // memory interface
//             output o_mem_cen,
//             output o_mem_wen,
//             output [ADDR_W-1:0] o_mem_addr,
//             output [BIT_W*4-1:0]  o_mem_wdata,
//             input [BIT_W*4-1:0] i_mem_rdata,
//             input i_mem_stall,
//             output o_cache_available,
//         // others
//         input  [ADDR_W-1: 0] i_offset
//     );

//     assign o_cache_available = 0; // change this value to 1 if the cache is implemented

//     //------------------------------------------//
//     //          default connection              //
//     assign o_mem_cen = i_proc_cen;              //
//     assign o_mem_wen = i_proc_wen;              //
//     assign o_mem_addr = i_proc_addr;            //
//     assign o_mem_wdata = i_proc_wdata;          //
//     assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
//     assign o_proc_stall = i_mem_stall;          //
//     //------------------------------------------//

//     // Todo: BONUS

// endmodule


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