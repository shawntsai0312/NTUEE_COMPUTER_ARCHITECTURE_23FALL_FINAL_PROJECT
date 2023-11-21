module memory #(
        parameter BIT_W = 32,
        parameter SIZE = 4096,
        parameter ADDR_W = 32,
        parameter OS = 32'h0001_0000
    )(
        input   i_clk,
        input   i_rst_n,
        input   i_cen,
        input   i_wen,
        input   [ADDR_W-1:0]    i_addr,
        input   [BIT_W*4-1:0]   i_wdata,
        output  [BIT_W*4-1:0]   o_rdata,
        output  o_stall,
        input   [ADDR_W-1:0]    i_offset,
        input   [ADDR_W-1:0]    i_ubound,
        input                   i_cache
    );

    integer i;

    reg [3:0]   delay_cnt;

    // ======================================
    // Control signal
    // ======================================

    reg                     cen, wen, cen_last;
    reg     [ADDR_W-1:0]    addr;
    reg     [BIT_W-1:0]     wdata;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            cen <= 0;
            wen <= 0;
            addr <= 0;
            wdata <= 0;
        end
        else begin
            if ((cen && wen && delay_cnt == 5) || (cen && delay_cnt == 10)) begin
                cen <= 0;
                wen <= 0;
                addr <= 0;
                wdata <= 0;
            end
            else if (i_cen && delay_cnt == 0) begin
                cen <= i_cen;
                wen <= i_wen;
                addr <= i_addr;
                wdata <= i_wdata;
            end
            
        end
    end

    // ======================================
    // Delay counter
    // ======================================

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            delay_cnt <= 0;
        end
        else begin
            if (cen && wen && delay_cnt == 5) begin
                delay_cnt <= 0;
            end
            else if (cen && delay_cnt == 10) begin
                delay_cnt <= 0;
            end
            else if (cen) begin
                delay_cnt <= delay_cnt+1;
            end
            else if (delay_cnt > 0 && delay_cnt < 10) begin
                delay_cnt <= delay_cnt+1;
            end
            else begin
                delay_cnt <= 0;
            end
        end
    end

    // ======================================
    // Mem
    // ======================================

    wire                addr_invalid;
    wire    [ADDR_W:0]  addr_true;
    wire    [ADDR_W-1:0]addr_idx;
    wire    [ADDR_W-1:0]addr_mem;

    reg     [BIT_W-1:0] mem[0:SIZE-1], mem_nxt[0:SIZE-1];

    assign addr_true = addr-i_offset;
    assign addr_invalid = addr_true[ADDR_W] || (addr >= i_ubound);
    assign addr_idx = (addr_invalid)? 0: addr_true;

    assign addr_mem = addr_idx>>2;

    always @* begin
        for (i=0; i<SIZE; i=i+1) begin
            mem_nxt[i] = mem[i];
        end
        
        if (cen && wen && (!addr_invalid) && delay_cnt == 4) begin
            mem_nxt[addr_mem] = wdata[0+:BIT_W];
            if(i_cache) begin
                mem_nxt[addr_mem+1] = wdata[BIT_W*1+:BIT_W];
                mem_nxt[addr_mem+2] = wdata[BIT_W*2+:BIT_W];
                mem_nxt[addr_mem+3] = wdata[BIT_W*3+:BIT_W];
            end
            
        end
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            for (i=0; i<SIZE; i=i+1) begin
                mem[i] <= 0;
            end
        end
        else begin
            for (i=0; i<SIZE; i=i+1) begin
                mem[i] <= mem_nxt[i];
            end
        end
    end

    // ======================================
    // rdata
    // ======================================

    wire     [BIT_W*4-1:0] rdata;

    assign rdata = {mem[addr_mem+3], mem[addr_mem+2], mem[addr_mem+1], mem[addr_mem]};
    assign o_rdata = (cen && delay_cnt == 10 && (!addr_invalid))? rdata: {{BIT_W}{1'bz}};

    // ======================================
    // Stall
    // ======================================

    assign o_stall = i_wen ? ((cen && delay_cnt == 5) ? 0 : (i_cen|cen)) : ((cen && delay_cnt == 10)? 0: (i_cen|cen));

    // always @(posedge i_clk) begin
    //     cen_last <= cen;
    // end

endmodule
