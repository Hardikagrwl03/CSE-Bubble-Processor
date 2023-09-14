module instruction_memory(clk, rst, address, mode, write, write_data, out);

    input clk;
    input rst;
    input [7:0] address;
    input write;
    input [31:0] write_data;
    input mode;

    output reg signed [31:0] out;

    reg [31:0] mem[255:0];
    reg [4:0] add_reg;
    integer i;

    initial begin
        mem[0] <= 32'b000001_00010_00011_00000_00000_000000;
        mem[1] <= 32'b010010_00111_00101_00000_00000_000000;
        mem[2] <= 32'b001000_10000_10001_00000_00000_000000;
        mem[3] <= 32'b001001_00100_01000_00000_00000_000000;
        mem[4] <= 32'b100111_01001_01111_00000_00000_000000;
        mem[5] <= 32'b000000_00000_00000_00000_00000_000000;
        mem[6] <= 32'b000000_00000_00000_00000_00000_000000;
    end

    always @(posedge clk) begin
        add_reg <= address;
    end
    always @(mem[add_reg]) begin
        out = mem[add_reg];
    end
    always @(posedge clk) begin
        if (write && mode == 1'b0) begin
            mem[address] = write_data;
        end
    end
    always @(posedge rst) begin
        mem[address] = 1'b0;
    end

endmodule

module instruction_decode(opcode, mux_branch, mux_load, mux_ALU2);
    input [5:0] opcode;
    output mux_branch, mux_load, mux_ALU2;
    assign mux_branch = opcode[5] & ~opcode[4]; 
    // j-type instruction have 10 as the most significant bits of opcode, and for them we need to branch
        
    assign mux_load = (opcode == 6'b010001)?1:0;
    // if load instruction the store register value from memory else from the ALU output
    
    assign mux_ALU2 = opcode[5] | opcode[4]; 
    // r-type instruction have 00 as the most significant bits of opcode and for them we need the second operand of ALU from gp register
endmodule

/* ALU operations modules */

module and_mips(r3,r1,r2);

    input [31:0] r1,r2;
    output [31:0] r3;

    genvar i;

    generate 
        for(i=0;i<32;i=i+1) begin :and_for
            and a0(r3[i],r1[i],r2[i]);
        end
    endgenerate
endmodule

module or_mips(r3,r1,r2);

    input [31:0] r1,r2;
    output [31:0] r3;
    genvar i;

    generate 
        for(i=0;i<32;i=i+1) begin :and_for
            or or0(r3[i],r1[i],r2[i]);
        end
    endgenerate
endmodule

module full_adder (a,b,cin,s,cout); 

    input a,b,cin;

    output cout,s;

    wire cout,cin,t1,t2,t3;
    wire a,b,s;

    xor(t1,a,b);
    xor(s,cin,t1);
    and(t2,cin,t1);
    and(t3,a,b);
    or(cout,t2,t3);
endmodule

module add(r3,r1,r2);
    
    input [31:0] r1,r2;    
    output [31:0] r3;

    wire [32:0] c;
    assign c[0] = 1'b0;
    genvar i;
    generate
        for(i=0; i<32; i=i+1) begin
            full_adder add0(r1[i], r2[i], c[i], r3[i], c[i+1]);
        end
    endgenerate
endmodule

module full_subtractor (a,b,cin,s,cout); 
    input a,b,cin;
    output cout,s;
    wire cout,cin,t1,t2,t3, t4, t5, t6;
    wire a,b,s;

    xor g0(t1,a,b);
    xor g1(s,cin,t1);

    not g2(t2, a);
    and g3(t3, t2, b);

    not g4(t4, t1);

    and g5(t5,t4,cin);
    or g6(cout,t3,t5);

endmodule

module sub_mips(r3,r1,r2);
    
    input [31:0] r1,r2;    
    output [31:0] r3;

    wire [32:0] c;
    assign c[0] = 1'b0;
    genvar i;
    generate
        for(i=0; i<32; i=i+1) begin
            full_subtractor sub0(r1[i], r2[i], c[i], r3[i], c[i+1]);
        end
    endgenerate
endmodule

module sll_mips(r3,r1,r2);
    input [31:0] r1;
    input [31:0] r2;
    output [31:0] r3;

    assign r3 = r1 << r2;
endmodule

module srl_mips(r3,r1,r2);
    input [31:0] r1;
    input [31:0] r2;
    output [31:0] r3;

    assign r3 = r1 >> r2;
endmodule

module slt_mips(r3,r1,r2);
    input [31:0] r1;
    input [31:0] r2;
    output [31:0] r3;

    assign r3[0] = r1 < r2;
endmodule

module equal_mips(r3,r1,r2);
    input [31:0] r1;
    input [31:0] r2;
    output [31:0] r3;

    assign r3[0] = r1 == r2;

endmodule

module ALU(opcode, operand1, operand2, result);
    input [5:0] opcode;
    input [31:0] operand1;
    input [31:0] operand2;
    output [31:0] result;
    wire [31:0] all_out [63:0];

    add add0(all_out[6'b000001],operand1,operand2);
    sub_mips sub0(all_out[6'b000010],operand1,operand2);
    and_mips and0(all_out[6'b000101],operand1,operand2);
    or_mips or0(all_out[6'b000110],operand1,operand2);
    sll_mips sll0(all_out[6'b001000],operand1,operand2);
    srl_mips srl0(all_out[6'b001001],operand1,operand2);
    slt_mips slt0(all_out[6'b000111],operand1,operand2);
    equal_mips eq0(all_out[6'b100000],operand1,operand2);
    //Add
    assign all_out[6'b000011] = all_out[6'b000001];
    assign all_out[6'b010010] = all_out[6'b000001];
    assign all_out[6'b010011] = all_out[6'b000001];

    //Sub
    assign all_out[6'b000100] = all_out[6'b000010];

    //And
    assign all_out[6'b010011] = all_out[6'b000101];

    //Or
    assign all_out[6'b010101] = all_out[6'b000110];

    //Slt
    assign all_out[6'b010110] = all_out[6'b000111];

    //Bne
    assign all_out[6'b100000] = ~ all_out[6'b100000];

    //Bgt
    assign all_out[6'b100010] = (~all_out[6'b000111])&( ~ all_out[6'b100000]) ;
    assign all_out[6'b100011] = (~all_out[6'b000111]) ;
    assign all_out[6'b100100] = (all_out[6'b000111]) ;
    assign all_out[6'b100101] = (all_out[6'b000111]|all_out[6'b100000]) ;

    assign all_out[6'b010000] =  {32{1'b1}};
    assign all_out[6'b010001] =  {32{1'b1}};

    assign all_out[6'b100110] =  {32{1'b1}};
    assign all_out[6'b100111] =  {{31{1'b0}}, 1'b1};
    assign all_out[6'b101000] =  {32{1'b1}};

    assign result = all_out[opcode];

endmodule

module mux2_32(s,a,b,out);
    input s;
    input [31:0] a;
    input [31:0] b;
    output [31:0] out;
    wire [31:0] s32;
    assign s32 = {32{s}};
    assign out = (~ s32 & a) | (s32 & b);
endmodule

module brancher(pc, inst_fetch, jump_address, mux_branch, ALU_out, new_pc, pc_plus_1);
    input [31:0] pc;
    input [31:0] inst_fetch;
    input [31:0] jump_address;
    input mux_branch;
    input [31:0] ALU_out;
    output [31:0] new_pc;
    output [31:0] pc_plus_1;
    wire [31:0] branch_address;
    wire to_branch;
    wire [31:0] pc1;
    wire is_jr;
    assign is_jr = inst_fetch[31:26] == 6'b100111;
    add pc_add_1(pc_plus_1, pc, 1);
    add pc_plus_1_plus_address(branch_address, pc_plus_1, {6'b000000, inst_fetch[25:0]});
    and a0(to_branch, ALU_out[0], mux_branch);
    mux2_32 branch_mux(to_branch, pc_plus_1, branch_address, pc1);
    mux2_32 jr_mux(is_jr, pc1, jump_address, new_pc);
endmodule

module processor(clk);
    input clk;
            reg [31:0] pc;
            initial begin
                pc <= 8'b00000000;
            end
    reg [31:0] registers[31:0];
    wire [31:0] inst_fetch;
    wire mux_branch, mux_load, mux_ALU2;
    wire [31:0] ALU_oper1;
    wire [31:0] ALU_oper2;
    wire [31:0] ALU_out;
    wire [31:0] new_pc;
    wire [31:0] pc_plus_1;
    wire [31:0] write_data;
    wire is_jal;
    instruction_memory i0(clk, 1'b0, pc[7:0], 1'b1, 1'b0, 0, inst_fetch);
    instruction_decode id0(inst_fetch[31:26], mux_branch, mux_load, mux_ALU2);    
    
    assign ALU_oper1 = registers[inst_fetch[25:21]];
    mux2_32 MUX_ALU_OPERAND2(mux_ALU2, registers[inst_fetch[20:16]], {16'b0000_0000_0000_0000, inst_fetch[15:0]}, ALU_oper2);
    
    ALU ALU0(inst_fetch[31:26], ALU_oper1, ALU_oper2, ALU_out);

    brancher brancher0(pc, inst_fetch, ALU_oper1, mux_branch, ALU_out, new_pc, pc_plus_1);

    assign is_jal = inst_fetch[31:26] == 6'b101000;
    mux2_32 mux_jal(is_jal, pc_plus_1, ALU_out, write_data);
    initial begin
        registers[2] <= 4;
        registers[3] <= 61;
        registers[7] <= 31;
        registers[5] <= 21;
        registers[4] <= 75;
        registers[8] <= 3;
        registers[9] <= 1;
        registers[17] <= 5;
        registers[16] <= 2;
        registers[15] <= 5;
        registers[14] <= 5;
    end

    always@(posedge clk) begin
        registers[5'b000010] <= inst_fetch; // Instruction fetch
        #5
        if (is_jal == 1'b1) begin
            registers[5'b00000] <= write_data;
        end
        $display("operand1 = %d, operand2 = %d, output = %d, clock=", ALU_oper1, ALU_oper2, ALU_out, clk);
        // $display("pc = %d, inst = %b, mux_branch = %d", pc, registers[5'b00001], mux_branch);
        pc <= new_pc;
    end
endmodule
    

module tb;
    reg clk;
    initial begin
        clk <= 1'b0;
        forever begin
            #5 clk <= ~clk;
        end
    end
    processor p0(clk);
    
    initial begin
        #100
        $finish;
    end

endmodule
/*
module tb;
    reg clk;
    reg s;
    reg [31:0] a;
    reg [31:0] b;
    wire [31:0] out;

    initial begin
        clk <= 1'b0;
        forever begin
            #5 clk <= ~clk;
        end
    end
    mux2_32 m0(s,a,b,out);
    initial begin
        $monitor("a = %d, b = %d, s = %b, out = %d", a,b,s,out);
        a <= 123451;
        b <= 42156;
        s <= 0;
        #10
        s <= 1;
        #10
        b <= 421561;
        a <= 1234;
        #10
        s <= 0;
        #10
        s <= 1;
        
        #100
        $finish;
    end
endmodule
*/
/*
module tb;
    reg clk;
    reg s;
    reg [31:0] a;
    reg [31:0] b;
    wire [31:0] out;

    initial begin
        clk <= 1'b0;
        forever begin
            #5 clk <= ~clk;
        end
    end
    mux2_32 m0(s,a,b,out);

    initial begin
        
        #100
        $finish;
    end
endmodule
*/
