// DO NOT MODIFY THE TESTBENCH
`timescale 1 ns/10 ps

`define CYCLE 10          // Do not change this value!!!
`define END_CYCLE 10000 // You can modify your maximum cycles

`include "../00_TB/memory.v"
`define SIZE_DATA 1024  // You can change the size
`define SIZE_STACK 32  // You can change the size

`ifdef I1
    `define MEM_INST "../00_TB/Pattern/I1/mem_I.dat"
    `define MEM_DATA "../00_TB/Pattern/I1/mem_D.dat"
    `define MEM_GOLDEN "../00_TB/Pattern/I1/golden.dat"
`elsif I2
    `define MEM_INST "../00_TB/Pattern/I2/mem_I.dat"
    `define MEM_DATA "../00_TB/Pattern/I2/mem_D.dat"
    `define MEM_GOLDEN "../00_TB/Pattern/I2/golden.dat"
`elsif I3
    `define MEM_INST "../00_TB/Pattern/I3/mem_I.dat"
    `define MEM_DATA "../00_TB/Pattern/I3/mem_D.dat"
    `define MEM_GOLDEN "../00_TB/Pattern/I3/golden.dat"
`elsif IH
    `define MEM_INST "../00_TB/Pattern/IH/mem_I.dat"
    `define MEM_DATA "../00_TB/Pattern/IH/mem_D.dat"
    `define MEM_GOLDEN "../00_TB/Pattern/IH/golden.dat"
`else
    `define MEM_INST "../00_TB/Pattern/I0/mem_I.dat"
    `define MEM_DATA "../00_TB/Pattern/I0/mem_D.dat"
    `define MEM_GOLDEN "../00_TB/Pattern/I0/golden.dat"
`endif

module Final_tb;

    reg             clk, rst_n ;
    
    wire            cache_wen, cache_cen, cache_stall;
    wire    [31:0]  cache_addr;
    wire    [31:0]  cache_wdata;
    wire    [31:0]  cache_rdata;

    wire            DMEM_wen, DMEM_cen, DMEM_stall, SMEM_stall, mem_stall;
    wire    [31:0]  DMEM_addr;
    wire    [127:0]  DMEM_wdata;
    wire    [127:0]  DMEM_rdata;
    
    wire    [31:0]  IMEM_addr;
    reg     [31:0]  IMEM_data;
    wire            IMEM_cen;
    reg     [31:0]  mem_inst[0:1023];

    wire    [3:0]   inst_type;

    reg     [31:0]  DMEM_golden [0:`SIZE_DATA-1];
    reg     [31:0]  mem_data;
    reg     [31:0]  mem_inst_offset;
    reg     [31:0]  mem_data_offset;
    reg     [31:0]  mem_stack_offset;
    wire    [31:0]  mem_I_addr;
    wire            cache_available;
    wire            cache_finish;
    wire            proc_finish;

    wire            finish;

    integer i, cyc;
    
    integer eof, DMEM_OS;
    reg eof_find;

    integer error_num;

    assign mem_stall = DMEM_stall | SMEM_stall;
    
    CHIP chip0(
        // clock
            .i_clk          (clk),
            .i_rst_n        (rst_n),
        // instruction memory
            .i_IMEM_data    (IMEM_data),
            .o_IMEM_addr    (IMEM_addr),
            .o_IMEM_cen     (IMEM_cen),
        // data memory
            .i_DMEM_stall   (cache_stall),
            .i_DMEM_rdata   (cache_rdata),
            .o_DMEM_cen     (cache_cen),
            .o_DMEM_wen     (cache_wen),
            .o_DMEM_addr    (cache_addr),
            .o_DMEM_wdata   (cache_wdata),
        // finnish procedure
            .o_finish       (finish),
        // cache
            .i_cache_finish (cache_finish),
            .o_proc_finish  (proc_finish)
    );

    memory #(.SIZE(`SIZE_DATA)) DMEM(
        .i_clk      (clk),
        .i_rst_n    (rst_n),
        .i_cen      (DMEM_cen),
        .i_wen      (DMEM_wen),
        .i_addr     (DMEM_addr),
        .i_wdata    (DMEM_wdata),
        .o_rdata    (DMEM_rdata),
        .o_stall    (DMEM_stall),
        .i_offset   (mem_data_offset),
        .i_ubound   (mem_stack_offset),
        .i_cache    (cache_available)
    );

    memory #(.SIZE(`SIZE_STACK)) SMEM(
        .i_clk      (clk),
        .i_rst_n    (rst_n),
        .i_cen      (DMEM_cen),
        .i_wen      (DMEM_wen),
        .i_addr     (DMEM_addr),
        .i_wdata    (DMEM_wdata),
        .o_rdata    (DMEM_rdata),
        .o_stall    (SMEM_stall),
        .i_offset   (mem_stack_offset),
        .i_ubound   (32'hbffffff0),
        .i_cache    (cache_available)
    );

    Cache cache(
        // clock
            .i_clk          (clk),
            .i_rst_n        (rst_n),
        // processor interface
            .o_proc_stall   (cache_stall),
            .o_proc_rdata   (cache_rdata),
            .i_proc_cen     (cache_cen),
            .i_proc_wen     (cache_wen),
            .i_proc_addr    (cache_addr),
            .i_proc_wdata   (cache_wdata),
            .i_proc_finish  (proc_finish),
            .o_cache_finish (cache_finish),
        // memory interface
            .o_mem_cen      (DMEM_cen),
            .o_mem_wen      (DMEM_wen),
            .o_mem_addr     (DMEM_addr),
            .o_mem_wdata    (DMEM_wdata),
            .i_mem_rdata    (DMEM_rdata),
            .i_mem_stall    (mem_stall),
            .o_cache_available (cache_available),
        // others
            .i_offset (mem_data_offset)
    );

    // Initialize the data memory
    initial begin
        $fsdbDumpfile("Final.fsdb");            
        $fsdbDumpvars(0,Final_tb,"+mda");

        $display("------------------------------------------------------------\n");
        $display("START!!! Simulation Start .....\n");
        $display("------------------------------------------------------------\n");
        
        clk = 1;
        rst_n = 1'b1;
        cyc = 0;
        mem_inst_offset = 32'h00010000;
        mem_stack_offset = 32'hbffffff0 - `SIZE_STACK*4;
        eof_find = 0;

        $readmemh (`MEM_INST, mem_inst); // initialize data in mem_I

        for (i=0; i<`SIZE_DATA; i=i+1) begin
            if ((mem_inst[i] === 32'bx) && !eof_find) begin
                eof = mem_inst_offset + i*4;
                eof_find = 1;
            end
        end

        mem_data_offset = eof;

        #(`CYCLE*0.5) rst_n = 1'b0;
        #(`CYCLE*2.0) rst_n = 1'b1;
                
        for (i=0; i<`SIZE_DATA; i=i+1) begin
            mem_inst[i] = 32'h0000_0073;
        end
        $readmemh (`MEM_INST, mem_inst); // initialize data in mem_I
        
        for (i=0; i<`SIZE_DATA; i=i+1) begin
            DMEM.mem[i] = 0;
        end

        for (i=0; i<`SIZE_DATA; i=i+1) begin
            DMEM_golden[i] = 0;
        end
        $readmemh (`MEM_DATA, DMEM.mem); // initialize data in mem_D
        $readmemh (`MEM_GOLDEN, DMEM_golden); // initialize data in mem_D
    end

    initial begin
        IMEM_data = 0;
    end

    assign mem_I_addr = (IMEM_addr - mem_inst_offset)>>2;

    always @(negedge clk) begin
        IMEM_data = IMEM_cen ? mem_inst[mem_I_addr] : IMEM_data;
    end

    initial begin
        #(`CYCLE*`END_CYCLE)
        // pass_fig;
        $display("============================================================\n");
        $display("Simulation time is longer than expected.");
        $display("The test result is .....FAIL :(\n");
        $display("============================================================\n");
        
        $finish;
    end
    
    initial begin
        // @(IMEM_addr === eof);
        // #(`CYCLE*10)
        @(finish === 1);
        error_num = 0;
        for (i=0; i<`SIZE_DATA; i=i+1) begin
            mem_data = DMEM.mem[i];
            if (mem_data !== DMEM_golden[i]) begin
                if (error_num == 0)
                    $display("Error!");
                error_num = error_num + 1;
                $display("  Addr = 0x%8d  Golden: 0x%8h  Your ans: 0x%8h", (mem_data_offset + i*4), DMEM_golden[i], mem_data);
            end
        end
        if (error_num > 0) begin
            fail_fig;
            $display(" ");
            $display("============================================================\n");
            $display("There are total %4d errors in the data memory", error_num);
            $display("The test result is .....FAIL :(\n");
            $display("============================================================\n");
        end
        else begin
            pass_fig;
            $display("============================================================\n");
            $display("Success!");
            $display("The test result is .....PASS :)");
            $display("Total execution cycle : %32d", cyc);
            $display("============================================================\n");
        end

        $finish;
    end
        
    always #(`CYCLE*0.5) clk = ~clk;

    always @(negedge clk) begin
        cyc = cyc + 1;
    end

    task fail_fig;
        begin
        $display("                                                              ...                                                                                               ");
        $display("                                              ........'''''',:llo;                                                                                              ");
        $display("                                     .,;::::clodddoooollllooollllc.                                                                                             ");
        $display("                                   .:ddoodooodkOOxddoooooooooooolc;.                                                                                            ");
        $display("                                 'cxOkddxkxdodxxddddooooooooooddol::;.                                                                                          ");
        $display("                               .:xO0Odldxxxdoodooddooooooolllooddoc;;,.                                                                                         ");
        $display("                              .oxkO0Odlododdddddddddddoolcccccllooc:;;c'                                                                                        ");
        $display("                             'oxxkO00kxxxxxxdodddddddoolllloolccclc:;ckd.                                                                                       ");
        $display("                            ,dxxxxkOOOOkkkxdooddooolllok0000Okdollllcokk,                                                                                       ");
        $display("                           .oxxxxxkkxxxxxxdooollc;,.'',:looolldkkkkxxxdd;                                                                                       ");
        $display("                          ,odddddddddddddddollc;,'....'',,,,,;:odxxkkOOkd,                                                                                      ");
        $display("                        'lxxdddoooddoooddollc:;,'.........'',,:cclldxxOOOko,                                                                                    ");
        $display("                     .,cdxxxxxxdoodddoooolc:;,,''......'',;::::cccccldk0KKK0x'                                                                                  ");
        $display("                  .':ldddddddxxxddddxxdoolc::;,,''''''',;codxdlllcccloOKXXXXXk.                                                                                 ");
        $display("                .;lodddddddddxxxxxxdddddolccc:;,,''...',:ldk00kdollllox0KXXXKl.                                                                                 ");
        $display("              .,looooodddddddxxxkkxxddddoollc::;,,'....',:codxkOkxxdddxkxdl:.                                                            .;;.                   ");
        $display("             'lddoddoooodoodddxxxkkkxxxdddollcc:;,'''..'',,;:cloddddc'...                                                              .:OWWKo.                 ");
        $display("           .:odxxxdoddddooooodddxxkkkxxxxxddolc::;,,,''''',,,;::ccc:.                                                                 ;kNWWWWNo                 ");
        $display("          .lxxkkkkxdoooddodoooodddxxxxxxxxxxddollc:;;,,,,,,,',,,;;;;.                                                               ;xKXNNNW0l.                 ");
        $display("         .lxkkkkkkkkxdooooooolooooodddxxxxxxdddollc::;;;,,,,,,,,;,,,.                                                             ,xKX00KX0l.                   ");
        $display("        .cxxkOkkxkkkkkdooooolllllcllloooooodooollcc::;,,,''',,,,,,,'.                                                           'xXNNXKNKo.                     ");
        $display("        ,odxkOOkkxxkkkkxxdollccc::::::ccccclllccc::;;,,''.''',,,,,''.                                                         'dXWWWWNKo.                       ");
        $display("       .ldddxkOOOkkxxkkkkxdoolcc::::;;;;;;;;;:::::;,,''...'',,,,,'''.                                                       .oKWWWWWXd;.''.                     ");
        $display("      .lxddddxkkOOOkxxxxxxxddolccc::;;;,,,,,,,,;;;,,'''...'',,,,''''.                                                     .lKWWWWWN0dclodxdc'                   ");
        $display("     .ckxdddddxxk00Okxxxddooooolcccc:::;;,,,,'''',,'''....''',,'''''.                                                   .c0NWWWWN0kkxddxddOOx:.                 ");
        $display("     cOOxddddddxkO0Okxxdddoolccc:cc::::::;;;,,,,,'''..''''''''''''''.                                                  ,kNWWWWN0xddddddddxOOxxc.                ");
        $display("    :O0OOkxddddxxkOOkkxxxddol:;;;;;;;;;,;;;,,,,,,''''''''''''''''',,.                                                'dXWWWWXOxoodddddxxkkkxdddc'               ");
        $display("   ,k0O0OOkxxxddxxkOkkkxxxddoc;;,,,,,,,,''''',,,,,'''''''''''''',,,,.                                              .lKWWWWKkocloddddooxxddddddxxdc;'..          ");
        $display("   :kOOOOOkkxxxdxxxkkkkxxxxxdlc;,,,,,,'''''',,,''''''''''''',,;;,,,,.                                             'dKWWWKkddxdc;,;:cllooddddxxdddddddc'         ");
        $display("  .lxxkkOOkkkxxxxxxxkkkxxxddddol:;,,,,,'''',,,''''''''''''',;;;;;,,,.                                           .lxdkXXOxlcc:,...',;:loddxxxxddddddxkxd;.       ");
        $display("  'dkxxkkkkkkxxxxdxxxkkkxxxddddol::;,,,,,,,,,,,,,,''',,,,',,;::::::,.                                         .:xxk0xdolc::,'...',;cloddxkxxddoodxkkkkkxc.      ");
        $display("  'dkxxkkkkkkkkxxxxxxkkkkxxxxdddolc::;;,,,;;;;;;;;;,,,,,,,;::cccc::,.                                       .x0xk0kdoooddoc;''',;clodxxxxxdoooodkkkkkkkkd;      ");
        $display("  .lkxxkkkkxxkxxxxxxxxkkkkxxxxxddoollc:;;;;;;;;;;;;;;,,,,;::::::::c;                                       .dNWkc;clllc:;,,',;:cloddddollcllodxkkkxkOOkxdl.     ");
        $display("   ,xkkkkkkxkkxxxxxxxxkkkkxxxxxddddooolc::::::;;;;;;,,;;;:::;;;;::l:.                                     ..;oo' .,,,,,'',,;;:cllcc:::::cllodxxxxxkOOkxdddl.    ");
        $display("   .oOOOOkkxkkkkxxxxxxkkOkkxxxxddddddoolc:::;;;;;;;;:::::;;;;;;:::c:.                                   ..''.   .',,,,''.'',,,;;;;;;::cccclloddxxkOOkxdddxkl.   ");
        $display("    'dkkkxxxkkOOkkxxxxkkOOOOkxxddddddollcc::;::::::cc::::;;,,;;;;;c:.                                 ..''.    ..''','''''''',,,,;;;;;::;;:codxxkOOkxdddxkOOc   ");
        $display("     .cxxxxxxkkkOOOkkkkOOO00Okxdddddoolc::::ccccccccllooc:,,,,,;;;c:.                               ..'..      .,,,,''''''''',,,''',,,,,,;:odxxxkkkxxxxxkOO0k,  ");
        $display("      .:xkkkkkkkkkkOOOO00KK0Okxdddooollc::cllllllllodkOxoc;,',,,,;::.                             ..'..        ',,;;,,''''''''',,''',,,,;coddxxkkkxxxxxkkOOkxc. ");
        $display("        ,dkkkkkkkkkOOOO0KKK0kxxddooollc:;:lollcllloxOOOkdl:;,',;,;:;.                            .'..          .:::::;,,,,,,,,,,,,,,,,;:codddxxkkxxdxxkkkkkxko. ");
        $display("         .:xkkkkOOOOOO000KK0kxxddoolllc:;:lllcclloxOOkkkxdc;,,,;,;:c:;,'..                     ....            .::::::;,,,,;;;;;;;;;;:clodxxxxkkkxxxxxkkkkkxkl. ");
        $display("           .;oOOOOOOOO00KKK0kxxddoollcc:;:llccclldxk00Okkxl:,,;;;;:clloolc:;,.....           ....              'c:;;:::;;,;;;;;:::::clodddxxxkkkxxxxxxkkkkkkx;  ");
        $display("              ,ok0OOOO0KKKK0kxxxddoolcc:;::::::ccclldkkkkdoc;;;;;;;:;;coooolc:::;;,..      ....                'c:;;;;;:::::;;;;;:ccldddddxxkOOkxxxxkkkkxkOOo.  ");
        $display("                .:dO000KKK0Okxxxxxdolcc:;;;;;::::::cclodxxdoc:::;;;,   ...',;:okkdclo'   ....                  'c;;;,,;:llccc::::::clodddxxk00OkkkkkOkkxxxxl.   ");
        $display("                   .:okKKKK0Okxkkxdollc:;;;;,,;;;;;:::clodxdolc::;;;.         .;c;.''. ....                    ':;,,',:lxkxllllllc::loooddxk0K0OOOOkkkkkkx:.    ");
        $display("                      .,lx0XKOxxkxdoollc:::;;,,,,,,;;;;:coxxdolc:;;::.               ....                    ..;:;;,,;ldkOOxolllll::clloodxk0K00OOOOkkkkd,      ");
        $display("                          .;ldxxxxxdollc:cllc::;;,,,''',,:ldolccc:;;:c:;,'...       ...                ..',:clll:;;;,:oxkOOkdllccc:;clloddxk0K00OOOOOko,.       ");
        $display("                              'lxkxdollc:cl:,......     ..... ..;:;::::cloolc:;'...''               ',:ccclc:;,,;;;;;ldxkxolccc:::;:cllodxxk0KK000Oxl'          ");
        $display("                               .:xkxdolcc;.                      ';;;;;;;:ccc:,'..;ox:.            .;,::'..    ';;:cldxdlc::;;;;;;;:clodxxxk0KK0xl,.            ");
        $display("                                .lOdoolcc'                         .,;,,,,;,'',,;:cloc.                      .':::cldxdc:;;;,,,;;:::llodxxk0Odc'.               ");
        $display("                                 :xdollcc'                           .,::;;;'';:::::,.                ...',;::::::;,;;,......'',:lcclodxxdc,.                   ");
        $display("                                 :xdoolcc;.                            ';::clc::;::.               'lc:ccll::;;;,.               .;cloxkl.                      ");
        $display("                                .okxdolcc:.                              ...''....                 'cc::::;,;;,.                  ,clodd'                       ");
        $display("                                 :kkddllcc'                                                         .;::cc::;.                    ,clodo.                       ");
        $display("                                 .dkdolccc;.                                                          ...'..                     .:cloxx'                       ");
        $display("                                 .cxdlc:;;;.                                                                                     'ccldko.                       ");
        $display("                                  ,ddoc;;;:;.                                                                                   .,::cox:                        ");
        $display("                                  .:oolc::;::,.                                                                                .,:;;coo.                        ");
        $display("                                    .'cl:clclc.                                                                               .;cc:cc;.                         ");
        $display("                                         ....                                                                                   ....                            ");
        $display("                                                                                                                                                                ");
        $display("                                                                                                                                                                ");
        $display("                                                                                                                                                                ");
        $display("                                                                                                                                                                ");
        $display("                                                                                                                                                                ");
        $display("                                .;'            .::;;;;;;;,.         .';cccc:'.       .;:;.      .;:.   .;:,      .,:;:.           ',.                           ");
        $display("                             .,.:Kd.'.         lWWWNNNNNNNXk;     .lONWWNNNWN0d'     cNMWO,     lWMd   lNMO'    ;OWWKl.        '''kO'.'                         ");
        $display("                             'xk0NXkkl.        oMMKc'''';xNMK;   ;0WW0c,..':kNMXOc   lNMMMK:    oMMd   lWMO'  ,kNMKo.         .okONXkkd.                        ");
        $display("                              'd0O00:.         oMMO.     :XMK,  'OMWk.      .lNMMX:  lNMWWMXl.  oMMd   lWMO''xNMXo.            .l00OKo.                         ");
        $display("                              .l; 'l;          oMMNkdddxkXW0;   cNMN:        '0MMWx. lNMOo0MNd. oMMd   lWMXOXMMM0,             .cc..cc.                         ");
        $display("                                               oMMN0OOOO0NWKd'  lWMN:        '0MWMx. lNMO.'kWWk'oMMd   lWMMMXxOWW0;                                             ");
        $display("                                               oMMO'    .'xWM0' ,KMWo.       :XMMNl  lNMO. .dNMKKMMd   lWMNx' .dNMXl.                                           ");
        $display("                                               oMMO'     'xWM0, .lNMNx'    .lKMWNx.  lNMO.  .lXMMMMd   lWM0'   .oNMWx.                                          ");
        $display("                                               oMMN0OOOO0XWW0:    ;kNMN0kkOXWW0c,.   lNMO.    ;KMMMd   lWM0'     cXMWO,                                         ");
        $display("                                               ,dddddddddol;.       ,cdxkkxdl;.      ,odc.     'odd;   ,ddc.      ,dddc.                                        ");
        $display("                                                                                                                                                                ");
        $display("                                                                                                                                                                ");
        end
    endtask

    task pass_fig;
        begin
        $display("                                                                              ..                                                                                ");
        $display("                                                                            .//(((.            *(#.                                                             ");
        $display("                                                                            /////((/,...   .,(((#(                                                              ");
        $display("                                                                            /(*,*(//(############(.                                                             ");
        $display("                                                                           *(/(//(##########((#%%%%%%%%%%(,                                                          ");
        $display("                                                                          /(/(##########(#%%##(###//#%%%%#.                                                        ");
        $display("                                                                        /######%%%%%%%%%%%%#*../,/#%%%%%%#//..88/                                                        ");
        $display("                                                                      *##(###%%%%%%%%%%%%%%%%%%######%%%%%%%%#%%%%%%888%%                                                        ");
        $display("                                                                     /#(((#(#####%%%%%%%%%%%%%%%%%%%%%%%%%%%%*   ..888,                                                       ");
        $display("                                                                   .((((######%%%%%%%%#%%%%#########(/*,.,/#88.                                                       ");
        $display("                                                                  ,(#############%%%%%%%%#####(*,,,,,,,/%%88#                                                        ");
        $display("                                                                 /##########%%%%#####%%%%%%############%%%%%%8(                                                         ");
        $display("                                                                (######%%%%%%%%%%%%%%###%%##############%%%%%%%%%%#.                                                         ");
        $display("                                                              .(#######%%%%%%%%%%%%%%#%%%%##%%%%%%###########%%%%%%8#                                                          ");
        $display("                                                             ,########%%%%%%%%%%%%%%###################%%%%%%%%8%%(,                                                        ");
        $display("                                                        .*/((#########%%%%%%##%%%%%%%%%%###%%############%%%%%%%%8%%####*                                                     ");
        $display("                                 .,**/(##((/,.  ,*(((#(((############%%%%%%%%%%%%%%%%##%%%%##############%%%%%%%%%%%%%%#######(*                                                 ");
        $display("                         .*################((########################%%%%%%%%%%%%####%%%%#############%%%%%%%%%%%%%%###########((((##############(/,.                          ");
        $display("                    ./(####%%%%%%%%%%%%%%%%%%%%#################################%%%%%%%%%%%%%%%%%%%%%%%%%%%%#########%%#%%%%%%%%############################%%%%%%%%%%#####(,                     ");
        $display("                 ,(########%%%%%%%%%%%%%%%%%%%%%%%%%%%%###############################%%%%#####%%%%%%%%%%###########%%%%%%#####%%#################%%%%%%%%%%%%%%%%%%%%%%########(.                  ");
        $display("              ./##%%%%%%%%%%%%####%%%%%%%%##%%%%###%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%###############%%###########################%%%%%%%%######%%%%%%%%%%%%##%%%%%%%%%%%%%%%%%%%%%%%%####%%%%%%%%%%%%##*                 ");
        $display("             *##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###%%#%%#########%%#%%%%##########################################%%####%%####%%%%%%###%%%%######%%%%%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%%#(                ");
        $display("           .###%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%#%%####%%####%%%%#%%%%%%%%%%%%%%%%%%#################################################%%%%#%%%%#####%%########%%%%%%%%%%#%%%%%%%%%%%%#%%%%%%%%%%%%%%%%##               ");
        $display("           (#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#######%%####%%#######%%%%###%%###############################%%#%%###%%#%%%%####################%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##/              ");
        $display("          *###%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#######%%%%#####%%##########%%%%##########%%#########################%%##%%%%#%%%%%%##########%%########%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##(.             ");
        $display("          /#%%%%%%%%%%%%%%%%%%%%%%%%%%#%%##%%%%############%%%%%%%%###%%%%###%%%%##%%##%%####%%#######%%%%################%%%%%%%%#############%%%%%%##%%########%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#(.             ");
        $display("         /##%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%#%%%%########%%##############%%%%#%%%%%%%%%%##########%%##%%%%#%%%%##%%%%%%%%########%%%%%%%%###############%%##########%%%%##%%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%#(.             ");
        $display("        (###%%%%%%%%%%%%%%%%%%%%%%%%%%##%%%%%%%%#############%%%%#####%%%%#######%%###############%%%%%%%%#%%%%%%%%%%#########%%################%%###########%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%#(,             ");
        $display("       /####%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###########################%%%%%%%%%%%%########%%#####%%##%%%%%%%%%%%%##((####%%%%#%%#%%#####%%#################%%%%%%%%###%%%%%%%%%%%%%%%%%%%%%%%%###(.            ");
        $display("      /#####%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%%%%%%#############%%%%%%##############%%#####%%%%%%%%######%%###%%%%%%%%%%#########%%%%%%%%%%%%%%%%##%%##%%###############%%%%%%##%%%%%%%%%%%%%%%%%%%%%%%%%%%%####,            ");
        $display("     *(#######%%%%%%%%%%%%%%%%%%%%##%%%%#%%###############%%#############%%#%%%%%%%%%%%%%%#%%%%%%#%%%%%%%%%%%%#%%%%%%%%%%####((####%%%%%%%%%%%%######################%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####*            ");
        $display("     ((#########%%%%%%%%%%%%%%%%###%%#%%##########################%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%#########(((###%%%%%%%%%%%%###################%%###%%%%%%%%%%##%%%%%%%%%%%%%%%%%%%%######/            ");
        $display("    ,#########%%##%%%%%%%%%%%%%%%%%%%%%%#######################%%%%#%%####%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###(####%%%%%%%%%%########################%%%%#%%%%##%%%%%%%%%%%%%%%%#######(/            ");
        $display("    ,########%%#%%%%#%%%%%%%%##%%%%%%############%%%%###############%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%%%%%%%%%%%%%%%%%%%%%%%%###(((####################(((#########%%######%%#%%%%%%%%%%#%%#######*            ");
        $display("    ,######%%##%%#%%#%%%%########%%####(/*/#######################%%%%%%%%%%%%%%%%%%%%%%####%%%%%%####%%%%%%###((#####################################%%%%%%##########%%######*            ");
        $display("     (######%%#%%####%%%%###########(      .(########(################################################################%%%%#((########%%%%%%#####%%#%%%%%%%%######,            ");
        $display("    .############################.        /#%%######################################################%%%%%%%%%%#%%%%%%#####%%%%#  ./(#####%%#######%%%%#%%%%#%%%%####/             ");
        $display("   .########%%####%%##%%###%%#########(,       /####%%##########%%%%%%%%%%#%%##############################%%%%%%%%%%####%%%%%%####%%%%(      *(###########%%###%%#%%####/              ");
        $display("  .(####%%%%%%#######%%%%%%###%%%%##%%#######(       .#####%%#%%#####%%%%%%%%%%%%%%#################%%%%%%%%%%%%%%############%%%%%%##%%%%####%%(        *######################/              ");
        $display("  ,(####%%######%%#%%##########%%##%%#####/        /#########%%##%%%%%%%%%%#######################%%########%%%%%%%%%%%%###%%%%%%%%%%##(        ###############%%########(.             ");
        $display("  .(##########%%######################(.        /#############%%%%%%######################%%%%%%############%%##%%%%%%%%%%%%#/       *#######%%%%%%%%#####%%####%%%%###*             ");
        $display("   /############%%###%%#########%%#####((*         (###################################%%%%%%%%########%%%%%%%%%%####%%%%%%%%%%#,      *################%%%%%%########,             ");
        $display("   ,####%%########%%%%%%##################*         .#############%%%%######################%%%%%%####%%%%%%%%%%%%%%%%####%%%%%%%%%%#,     .(##########################(.             ");
        $display("    ####%%#%%%%###%%%%###%%#####%%%%%%##%%######*         .###############%%###################%%%%%%%%%%########%%%%%%#####%%%%%%%%##.     ,(######%%##########%%########/              ");
        $display("    /####%%%%%%%%%%%%########%%%%%%%%%%%%%%%%%%#####(,         .#############%%%%%%%%%%%%%%%%%%###############%%######%%%%%%##%%###########/      /################%%%%%%########*              ");
        $display("     (##%%%%%%%%%%%%%%#######%%%%%%%%%%%%%%%%%%%%%%%%###*          .#############%%%%%%%%%%%%%%%%%%%%%%#############%%%%%%######%%%%############(       (######%%##%%%%%%######%%%%##%%####.              ");
        $display("      ,#%%%%%%%%%%%%##%%%%#####%%%%%%%%%%%%%%%%%%##((             ./(###############%%%%%%%%%%%%%%############%%%%%%###%%%%####%%%%%%%%%%#####(        /######%%%%%%%%%%%%%%%%######%%%%%%%%##/               ");
        $display("        /#%%%%%%%%%%%%%%######%%%%%%%%%%########(,            ,(#######%%%%%%%%%%%%%%######%%%%%%############%%%%##################/          ,(###%%%%%%%%%%%%%%%%%%######%%%%%%%%#(                ");
        $display("         .(#%%%%#%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%%######(/.          /####################%%%%##%%%%%%#############################(         .(#####%%%%%%%%%%%%%%####%%%%%%%%%%%%%%/                 ");
        $display("           ,(#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###(((*       ,######%%%%#####################################%%%%%%%%#########       ,#((###%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#,                  ");
        $display("             .(%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####((/.     *######%%#######%%################(,    *(####%%%%%%%%%%###%%%%%%%%##,    /((((##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#.                    ");
        $display("               *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#(((/.    ,(#%%%%%%%%%%%%%%%%%%%%#######%%%%%%%%#####(.        ,####%%%%%%%%%%##%%%%%%##/   ./(((####%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%(                      ");
        $display("                 (%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%######(,     .##%%%%%%%%%%%%%%%%####%%%%#%%%%%%%%%%%%###*           *####%%%%%%%%##%%%%##/    *(((#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%(                       ");
        $display("                  .#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#(//((/,       (###%%%%%%%%%%###%%%%%%%%%%%%%%%%%%##*             .(#%%%%%%%%%%%%#%%##((     ,(#######%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/                        ");
        $display("                    ,%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%####(/          *(##%%%%%%%%%%%%%%%%%%%%%%%%###(,                /##%%%%####%%##(      ,/(//(#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8%%*                         ");
        $display("                      .#%%#%%%%8%%%%%%###%%%%%%%%%%%%##.           /(##%%%%#####%%%%%%##(,                  *((#%%%%%%%%%%#(/         .(####%%%%%%%%%%##%%%%88%%#%%*                           ");
        $display("                         ,(//#%%%%%%%%%%%%##/*(/.            *##(##%%%%####(((,                   .(#######(###/         ,###%%%%%%###%%%%%%%%#//,                             ");
        $display("                                 .,,.                  ,###%%##%%%%##%%###,                   (##%%%%%%#%%%%#####              .(###(,                                   ");
        $display("                                                         (#%%%%%%%%%%%%%%%%%%%%#*                  .#%%%%%%%%%%#%%%%%%%%#/                                                         ");
        $display("                                                         .##%%%%%%%%%%%%%%%%%%(                   .(%%%%%%%%%%%%%%%%##,                                                          ");
        $display("                                                          *(###%%%%%%%%##,                    /#%%%%%%%%%%##*                                                            ");
        $display("                                                           ,(#%%%%%%%%%%#,                     ,#%%%%%%%%#/.                                                             ");
        $display("                                                             *###%%#,                       *####/,                                                              ");
        $display("                                                             *##%%%%%%%%#(,                     ..,******,                                                          ");
        $display("");       
        end
    endtask
endmodule