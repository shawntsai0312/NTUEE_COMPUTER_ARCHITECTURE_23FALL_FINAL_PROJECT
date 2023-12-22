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
        input  [BIT_W-1:0]  i_DMEM_rdata,      //data memoy read                                                //
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
    assign o_DMEM_wdata = rs2;
    Op_control reg1(
        .opcode (i_IMEM_data[6:0]),
        .func7 (i_IMEM_data[31:25]),
        .func3 (i_IMEM_data[14:12]),
        .o_Branch_select(Branch_select), //01 for branch, 10 for jal, 11 for jalr
        .o_MemRead_select(o_DMEM_cen),   //ok
        .o_MemWrite_select(o_DMEM_wen),  //ok
        .o_MemtoReg_select(MemtoReg_select), //ok
        .o_ALUsrc_select(ALUsrc_select), //ok
        .o_Regwrite_select(Regwrite_select), 
        .o_ALU_opcode(ALU_opcode),   //ok
        .o_swap_branch_answer(swap_branch_answer),
        .o_finish(o_proc_finish),
        .o_Muldiv_ALU_choose(Muldiv_ALU_choose)
    );
    assign o_DMEM_cen = MemRead_select;
    assign o_DMEM_wen = MemWrite_select;
    assign o_finish = i_cache_finish;
    imm_generator reg2(               //register generate immediate
        .instruction (i_IMEM_data[31:0]),
        .o_immediate(immediate[31:0])
    );
    mux reg3(               //mutliplexer for rs2 and immediate
        .rs1(rs2),
        .rs2(immediate),
        .judge(ALUsrc_select),
        .o_result(rs2_select)
    );
    ALU alu1(
        .i_PC(PC),
        .i_A (rs1),
        .i_B (rs2_select),
        .swap_branch_answer(swap_branch_answer),
        .i_ALU_opcode(ALU_opcode),
        .o_ALU_result(single_ALU),
        .o_zero(zero)
    );
    MULDIV_unit reg5(
        .i_clk(i_clk),
        .i_A(rs1),
        .i_B(rs2_select),
        .i_ALU_opcode(ALU_opcode),
        .o_MULDIV_result(MULDIV_result),
        .o_pc_wait(pc_wait)
    );
    assign o_DMEM_addr=ALU_result;
    mux choose(
        .rs1(single_ALU),
        .rs2(MULDIV_result),
        .judge(Muldiv_ALU_choose),
        .o_result(ALU_result)
        );
    mux reg6(
        .rs1(ALU_result),
        .rs2(i_DMEM_rdata),
        .judge(MemtoReg_select),
        .o_result(write_data_back_reg)
    );
    
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
    //sequential part
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            //imem_cen <= 0;
        end
        else begin
            PC <= next_PC;
            //imem_cen <= imem_cen_nxt;
        end
        // $display("PC           = %h", PC);
        // $display("rs1          = %d", rs1);
        // $display("rs2_select   = %d", rs2_select);
        // $display("ALUopcode    = %d", ALU_opcode);
        // $display("ALUResult    = %d", ALU_result);
        // $display("result       = %d", ALU_result);
        // $display("i_DMEM_rdata = %h", i_DMEM_rdata);
        // $display("write_data   = %d", write_data_back_reg);
        // $display("i_IMEM_data  = %h", i_IMEM_data);
        // $display("\n");
    end
endmodule
module mux(rs1,rs2,judge,o_result);
    input[31:0] rs1,rs2;
    input judge;
    output[31:0] o_result;
    reg [31:0] result;
    assign o_result = result;
    always@(*)begin
        case(judge)
            1'b0: result = rs1;
            1'b1: result = rs2;
        endcase
    end
endmodule
module Op_control#(
    parameter BIT_W=32,
    parameter instruction_W = 7,
    parameter R_type = 7'b0110011,
    parameter I_type = 7'b0010011,
    parameter S_type = 7'b0100011,
    parameter B_type = 7'b1100011,
    parameter Load = 7'b0000011,
    parameter U_type = 7'b0010111,
    parameter Jal = 7'b1101111,
    parameter Jalr = 7'b1100111,
    parameter Ecall = 7'b1110011
)(
    input [instruction_W-1:0] opcode,
    input [6:0] func7,
    input [2:0] func3,
    output o_MemRead_select,o_MemWrite_select,o_MemtoReg_select,
    output [1:0] o_Branch_select, //01 for B-type, 10 for jal, 11 for jalr
    output o_ALUsrc_select,o_Regwrite_select,
    output [3:0] o_ALU_opcode,
    output o_swap_branch_answer,
    output o_finish,
    output o_Muldiv_ALU_choose
);
    reg MemRead_select,MemWrite_select,MemtoReg_select,ALUsrc_select,Regwrite_select;
    reg [1:0] Branch_select;
    reg[3:0] ALU_opcode;
    reg swap_branch_answer,finish;
    reg Muldiv_ALU_choose;
    assign o_Branch_select = Branch_select;
    assign o_MemRead_select = MemRead_select;
    assign o_MemWrite_select = MemWrite_select;
    assign o_MemtoReg_select = MemtoReg_select;
    assign o_ALUsrc_select = ALUsrc_select;
    assign o_Regwrite_select = Regwrite_select;
    assign o_swap_branch_answer = swap_branch_answer;
    assign o_finish = finish;
    assign o_ALU_opcode = ALU_opcode;
    assign o_Muldiv_ALU_choose = Muldiv_ALU_choose;
    always @(*)begin
        swap_branch_answer = 0;
        case(opcode)
            R_type:begin                                    
                 Branch_select = 2'b00;                          
                 MemRead_select = 0;                         
                 MemWrite_select = 0;                        
                 MemtoReg_select = 0;                        
                 ALUsrc_select = 0;                          
                 Regwrite_select = 1;
                 finish = 0;                       
                case (func7)
                    7'b0000000:begin
                        Muldiv_ALU_choose =0;
                        case(func3)
                            3'b000:  ALU_opcode = 4'b0000; //add
                            3'b111:  ALU_opcode = 4'b0010; //and 
                            3'b110:  ALU_opcode = 4'b0011; //or
                            3'b100:  ALU_opcode = 4'b1000; //xor
                            default: ALU_opcode = 4'b1111;
                        endcase
                        
                    end
                    7'b0100000:begin
                        Muldiv_ALU_choose =0;
                        case(func3)
                            3'b000:  ALU_opcode = 4'b0001; //sub
                            default:ALU_opcode = 4'b1111;
                        endcase
                    end
                    7'b0000001:begin
                        Muldiv_ALU_choose =1;
                        case (func3)
                            3'b000:  ALU_opcode = 4'b0110; //mul
                            3'b100:  ALU_opcode = 4'b0111;  //div
                            default: ALU_opcode = 4'b1111;
                        endcase
                    end
                    default: begin
                        Muldiv_ALU_choose =0;
                        ALU_opcode = 4'b1111;
                    end
                endcase
            end
            I_type:begin
                Muldiv_ALU_choose =0;
                 Branch_select = 2'b00;
                 MemRead_select = 0;
                 MemWrite_select = 0;
                 MemtoReg_select = 0;
                 ALUsrc_select = 1;
                 Regwrite_select = 1;
                 finish = 0; 
                case(func3)
                    3'b000:  ALU_opcode = 4'b0000; //addi
                    3'b010:  ALU_opcode = 4'b0100; //slti
                    3'b101: begin
                        case(func7)
                            7'b0100000:  ALU_opcode = 4'b0101; //srai
                            default: ALU_opcode = 4'b1111;
                        endcase
                    end
                    3'b001:begin
                        case(func7)
                            7'b0000000:  ALU_opcode = 4'b1001;//slli
                            default: ALU_opcode = 4'b1111;
                        endcase
                    end
                    default:ALU_opcode = 4'b1111;
                endcase
            end
            S_type: begin
                Muldiv_ALU_choose =0;
                 Branch_select = 2'b00;
                 MemRead_select = 1;
                 MemWrite_select = 1;
                 MemtoReg_select = 0;
                 ALUsrc_select = 1;
                 Regwrite_select = 0;
                 finish = 0; 
                case(func3)
                    3'b010:  ALU_opcode=4'b0000;
                    default:ALU_opcode = 4'b1111;
                endcase
            end
            B_type:begin
                Muldiv_ALU_choose =0;
                 Branch_select = 2'b01;
                 MemRead_select = 0;
                 MemWrite_select = 0;
                 MemtoReg_select = 0;
                 ALUsrc_select = 0;
                 Regwrite_select = 0;
                 finish = 0; 
                case(func3)
                    3'b000: begin
                         ALU_opcode = 4'b0001; //beq
                         swap_branch_answer = 0;
                    end
                    3'b001:begin
                         ALU_opcode =4'b0001;  //bne
                         swap_branch_answer = 1;
                    end
                    3'b100:begin
                         ALU_opcode =4'b0100; //blt
                         swap_branch_answer = 0;
                    end
                    3'b101:begin
                         ALU_opcode =4'b0100; //bge
                         swap_branch_answer = 1;
                    end
                    default:begin
                        ALU_opcode = 4'b1111;
                        swap_branch_answer = 0;  
                    end
                endcase
            end
            Load: begin
                Muldiv_ALU_choose =0;
                 Branch_select = 2'b00;
                 MemRead_select = 1;
                 MemWrite_select = 0;
                 MemtoReg_select = 1;
                 ALUsrc_select = 1;
                 Regwrite_select = 1;
                 finish = 0; 
                case(func3)
                    3'b010:  ALU_opcode =4'b0000; //lw
                    default:ALU_opcode = 4'b1111;
                endcase
            end
            U_type:begin
                Muldiv_ALU_choose =0;
                 Branch_select = 2'b00;
                 MemRead_select = 0;
                 MemWrite_select = 0;
                 MemtoReg_select = 0;
                 ALUsrc_select = 1;
                 Regwrite_select = 1;
                 ALU_opcode = 4'b1010;
                 finish = 0; 
            end
            Jal:begin
                Muldiv_ALU_choose =0;
                 Branch_select = 2'b10;
                 MemRead_select = 0;
                 MemWrite_select = 0;
                 MemtoReg_select = 0;
                 ALUsrc_select = 1;
                 Regwrite_select = 1;
                 ALU_opcode = 4'b1011;
                 finish = 0; 
            end
            Jalr:begin
                Muldiv_ALU_choose =0;
                 Branch_select = 2'b11;
                 MemRead_select = 0;
                 MemWrite_select = 0;
                 MemtoReg_select = 0;
                 ALUsrc_select = 1;
                 Regwrite_select = 1;
                 ALU_opcode = 4'b1011;
                 finish = 0; 
            end
            Ecall:begin
                Muldiv_ALU_choose =0;
                 Branch_select = 0;
                 MemRead_select = 0;
                 MemWrite_select = 0;
                 MemtoReg_select = 0;
                 ALUsrc_select = 0;
                 Regwrite_select = 0;
                 ALU_opcode = 4'b1101;
                 finish = 1; 
            end
            default:    begin  
                Muldiv_ALU_choose =0;               
                Branch_select = 0;
                MemRead_select = 0;
                MemWrite_select = 0;
                MemtoReg_select = 0;
                ALUsrc_select = 0;
                Regwrite_select = 0;
                ALU_opcode = 4'b1111;
                finish = 0; 
            end
        endcase
    end
endmodule
//----------------------------------------------------------------
//ALU_opcode 0000 add 
//           0001 sub
//           0010 AND
//           0011 OR
//           0100 Slt   (compare which is larger)
//           0101 SRA   (shift right)
//           0110 MUL
//           0111 DIV
//           1000 XOR
//           1001 sll (shift left)
//           1010 auipc
//           1011 Jal
//           1100 Jalr
//           1101 Ecall
//----------------------------------------------------------------
module ALU #(
    parameter ADD = 4'b0000,
    parameter SUB = 4'b0001,
    parameter AND = 4'b0010,
    parameter OR = 4'b0011,
    parameter Slt = 4'b0100,
    parameter SRA = 4'b0101,
    parameter XOR = 4'b1000,
    parameter SLL = 4'b1001,
    parameter auipc = 4'b1010,
    parameter Jal = 4'b1011,
    parameter Jalr = 4'b1100,
    parameter Ecall = 4'b1101,
    parameter DATA_W = 32
)(
    input [DATA_W - 1:0] i_PC,
    input [3:0] i_ALU_opcode,
    input [DATA_W - 1:0] i_A, i_B,
    input swap_branch_answer, // determine if branch 
    output [DATA_W - 1 : 0]   o_ALU_result,  // output value
    output o_zero   // whether to utilize the pc change and gate 
);

    reg [DATA_W-1:0] ALU_result, operand_a, operand_b;
    reg zero;
    reg [31:0] outcheck;
    assign o_ALU_result = ALU_result;
    assign o_zero = zero;
    always @(*) begin
        operand_a = i_A;
        operand_b = i_B;
    end

    
    always@(*) begin
        outcheck = 0;
        case(i_ALU_opcode)
            ADD: begin
                zero = 0;
                ALU_result = operand_a + operand_b;
                if (operand_a[DATA_W-1] == operand_b[DATA_W-1]) begin
                    if(ALU_result[DATA_W-1] != operand_a[DATA_W-1])begin
                        if(operand_a[DATA_W-1] == 0) ALU_result = 64'h7FFFFFFF;
                        else ALU_result = 64'h80000000;
                    end
                end
            end
            SUB:    begin
                ALU_result = operand_a - operand_b;
                if (swap_branch_answer == 0)begin
                    zero = (ALU_result ==0);
                end
                else begin
                    zero = ~(ALU_result == 0);
                end
                if (operand_a[DATA_W-1] != operand_b[DATA_W-1]) begin
                    if(ALU_result[DATA_W-1] != operand_a[DATA_W-1])begin
                        if(operand_a[DATA_W-1] == 0) ALU_result = 64'h7FFFFFFF;
                        else ALU_result = 64'h80000000;
                    end
                end
            end
            AND:    begin
                zero = 0;
                ALU_result = (operand_a & operand_b);
            end
            OR:     begin
                zero = 0;
                ALU_result = (operand_a | operand_b);
            end
            Slt:    begin   //signed
               if(operand_a[31]==1 && operand_b[31]==0)begin
                            ALU_result[31:0] = 1;
                        end
                        else if(operand_a[31]==0 && operand_b[31]==1)begin
                            ALU_result[31:0]  = 0;
                        end
                        else begin
                            outcheck[31:0] = operand_a[31:0] -operand_b[31:0];
                            if  (outcheck[31]==1)begin
                              ALU_result[31:0] =1;
                            end
                            else begin
                              ALU_result[31:0] =0;
                            end
                        end
                if (swap_branch_answer == 0)begin
                    zero = (ALU_result==1);
                end
                else begin
                    zero = ~(ALU_result==1);
                end
            end
            SRA:    begin
                zero = 0;
                ALU_result = $signed(operand_a[DATA_W-1:0])>>>$signed(operand_b[DATA_W-1:0]);
            end
            XOR:    begin
                zero = 0;
                ALU_result = (operand_a[DATA_W-1:0] ^ operand_b[DATA_W-1:0]);
            end
            SLL:    begin
                zero = 0;
                ALU_result = $signed(operand_a) << $signed(operand_b);
            end
            auipc:  begin
                zero = 0;
                ALU_result = i_PC + operand_b;
                $display(i_PC);
                $display(operand_b);
                $display(ALU_result);
            end
            Jal:    begin
                zero = 0;
                ALU_result = i_PC + 32'd4;
            end
            Jalr:   begin
                zero = 0;
                ALU_result = i_PC + 32'd4;
            end
            Ecall:  begin
                zero = 0;
                ALU_result = 0;
            end
            default:    begin
                zero = 0;
                ALU_result = 0;
            end       
        endcase
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
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule
module  imm_generator#(
    parameter BIT_W=32,
    parameter R_type = 7'b0110011,
    parameter I_type = 7'b0010011,
    parameter S_type = 7'b0100011,
    parameter B_type = 7'b1100011,
    parameter Load = 7'b0000011,
    parameter U_type = 7'b0010111,
    parameter Jal = 7'b1101111,
    parameter Jalr = 7'b1100111,
    parameter Ecall = 7'b1110011
)(
    input [31:0] instruction,
    output [31:0] o_immediate
);
    reg[31:0] immediate;
    assign o_immediate = immediate;
    always@(*)begin
    case(instruction[6:0])
        I_type:begin
            case (instruction[14:12])
                3'b001:begin
                    immediate[4:0] = instruction[24:20];
                    immediate[31:5] = (instruction[24]==1)?27'b111111111111111111111111111:0;

                end 
                3'b101:begin
                    immediate[4:0] = instruction[24:20];
                    immediate[31:5] = (instruction[24]==1)?27'b111111111111111111111111111:0;
                end
                default:begin
                    immediate[11:0] = instruction[31:20];
                    immediate[31:12] = (instruction[31]==1)? 20'b11111111111111111111 : 0;
                end
        endcase
        end
        S_type:begin
            
             immediate[4:0] = instruction[11:7];
             immediate[11:5] = instruction[31:25];
             immediate[31:12] = (instruction[31]==1)? 20'b11111111111111111111:0;
        end
        B_type:begin
             immediate[0] = 0;//dont care
             immediate[4:1] = instruction[11:8];
             immediate[11] = instruction[7];
             immediate[10:5] = instruction[30:25];
             immediate[12] = instruction[31];
             immediate[31:13] = (instruction[31]==1)? 19'b1111111111111111111: 0;  
        end
        U_type:begin
             immediate[31:12] = instruction[31:12];
             immediate[11:0] = 12'b0;
        end
        Jal:begin
             immediate[0] = 0;
             immediate[20] = instruction[31];
             immediate[10:1] = instruction[30:21];
             immediate[11] = instruction[20];
             immediate[19:12] = instruction[19:12];
             immediate[31:21] = (instruction[31]==1)? 11'b11111111111:0;
        end
        Jalr:begin
             immediate[11:0] = instruction[31:20];
             immediate[31:12] = (instruction[31]==1)? 20'b11111111111111111111 : 0;
        end
        Load:begin
             immediate[11:0] = instruction[31:20];
             immediate[31:12] = (instruction[31]==1)? 20'b11111111111111111111 : 0;
        end
        Ecall:begin
            immediate[31:0] = 0;
        end
        default: immediate[31:0] = 0;
    endcase
    
    end
endmodule
module MULDIV_unit#(  //TODO
    parameter DATA_W = 32
)
(
     input                       i_clk,   // clock 自己決定要不要clock 
    // input                       i_rst_n, // reset

    // input                       i_valid, // input valid signal
    input [DATA_W - 1 : 0]      i_A,     // input operand A
    input [DATA_W - 1 : 0]      i_B,     // input operand B
    input [         3 : 0]      i_ALU_opcode,  // instruction
    output [DATA_W - 1 : 0]   o_MULDIV_result,  // output value
    output o_pc_wait // told pc to wait
);

    reg [5:0] cnt, cnt_nxt;
    reg  [DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  [DATA_W-1: 0] operand_b, operand_b_nxt;
    reg  [2*DATA_W-1: 0] result, result_nxt;
    reg unit_on, pc_wait, temp;

    assign o_MULDIV_result = result_nxt[DATA_W - 1 : 0];
    assign o_pc_wait = pc_wait;
    //load input
    always @(*) begin
        operand_a_nxt = i_A;
        operand_b_nxt = i_B;
        if ((i_ALU_opcode == 4'b0110) || (i_ALU_opcode == 4'b0111)) begin // may have problem here.
            unit_on = 1;
        end
        else begin
            unit_on = 0;
        end
    end
    always @ (*) begin
        case(i_ALU_opcode)
            4'b0110: begin
                if (cnt == 0) begin 
                    if(i_A[0] == 0) result_nxt = {33'b0, i_A[31:1]};
                    else begin
                        result_nxt = {1'b0, i_B[31:0], i_A[31:1]}; 
                    end
                end
                else begin
                    if (result[0] == 1) begin
                        result_nxt = result >> 1;
                        result_nxt[63:31] = operand_b + result[63:32];
                    end
                    else begin
                        result_nxt = result >> 1;
                    end
                end
            end
            4'b0111:begin
                if (cnt == 0) begin
                    if (i_A[31] >= i_B) begin
                        result_nxt = {31'b0, i_A[30:0], 2'b01};
                    end
                    else result_nxt = {30'b0, i_A, 2'b0};
                end
                else if (cnt < 31) begin
                    if (result[63:32] >= operand_b) begin
                        {temp, result_nxt[63:33]} = result[63:32] - operand_b;
                        result_nxt[32:0] = {result[31:0], 1'b1};
                    end
                    else begin
                        result_nxt = {result[62:0], 1'b0};
                    end
                end
                else begin
                    if (result[63:32] >= operand_b) begin
                        result_nxt[63:32] = result[63:32] - operand_b;
                        result_nxt[31:0] = {result[30:0], 1'b1};
                    end
                    else begin
                        result_nxt = {result[63:32], result[30:0], 1'b0};
                    end
                end
            end
            default:    begin
                result_nxt = 0;
            end
        endcase
    end
    always @(*) begin
        if (cnt < 31 && unit_on) begin
            cnt_nxt = cnt + 1;
            pc_wait = 1;
        end
        else begin
            cnt_nxt = 0;
            pc_wait = 0;
        end
    end
    always @(posedge i_clk) begin
        // $display("cnt = %d, unit_on = %d, pc_wait= %d, MUL_result_nxt=%d, MUL_result = %d ",cnt,unit_on,pc_wait,result_nxt,result);
        if (!unit_on) begin
            operand_a   <= 0;
            operand_b   <= 0;
            cnt         <= 0;
            result      <= 0;
        end
        else begin
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            cnt         <= cnt_nxt;
            result      <= result_nxt;
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
    parameter S_IDLE = 3'd0;
    parameter S_FIND = 3'd1;
    parameter S_WB = 3'd2;
    parameter S_ALLO = 3'd3;
    parameter S_FINISH = 3'd4;
    parameter block_number = 16;
    //regs
    reg [2:0] state, state_nxt;
    reg hit;
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
    reg [ADDR_W-1:0] proc_addr, real_addr;

    //wire assignment
    assign o_mem_cen = mem_cen;
    assign o_mem_wen = mem_wen;
    assign o_mem_addr = proc_addr + i_offset;
    assign o_mem_wdata = data[index];//syntax??

    assign o_proc_stall = proc_stall;
    assign o_proc_rdata = data[index][ADDR_W*offset +: ADDR_W];
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
            for (i = 0; i < block_number; i = i+1)begin
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