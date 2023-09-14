    module instruction_memory(clk, rst, address, mode, write_data, out);

        input clk;
        input rst;
        input [7:0] address;
        input [31:0] write_data;
        input mode;
        output [31:0] out;

        reg [31:0] mem[255:0];
        reg [7:0] add_reg;
        reg [31:0] out;
        integer i = 1;

        initial begin
        mem[0]  = 32'b11010101111000110010000000000000;
        mem[1]  = 32'b11010101111000110010000000000000;
        mem[2]  = 32'b11010101111000110010000000000000;
        mem[3]  = 32'b01000100101000000000000000001000;
        mem[4]  = 32'b01001000010000110010000000000011;
        mem[5]  = 32'b10011100010000110010000000000000;
        mem[6]  = 32'b10000100000000110010000000000000;
        mem[7]  = 32'b11010100000000110010000000000000;
        mem[8]  = 32'b10010100000000110010000000000000;
        mem[9]  = 32'b11010100000000110010000000000000;
        mem[10] = 32'b11010100000000110010000000000000;
        mem[11] = 32'b11010100000000110010000000000000;
        mem[12] = 32'b11010100000000110010000000000000;
        mem[13] = 32'b11010100000000110010000000000000;
        mem[14] = 32'b11010100000000110010000000000000;
        mem[15] = 32'b11010100000000110010000000000000;
        mem[16] = 32'b11010100000000110010000000000000;
        mem[17] = 32'b11010100000000110010000000000000;
        mem[18] = 32'b11010100000000110010000000000000;
        mem[19] = 32'b11010100000000110010000000000000;
        mem[20] = 32'b11010100000000110010000000000000;
        mem[21] = 32'b11010100000000110010000000000000;
        mem[22] = 32'b11010100000000110010000000000000;
        mem[23] = 32'b11010100000000110010000000000000;
        mem[24] = 32'b11010100000000110010000000000000;
        end
        
        always @(posedge clk) begin
            add_reg <= address;
        end

        always @(mem[add_reg]) begin
            out <= mem[add_reg];
        end

        always @(posedge clk) begin
            if(mode == 1'b0) begin
                mem[address] = write_data;
            end
        end

        always @(posedge rst) begin
            mem[address] = 1'b0;
        end

    endmodule

    module data_memory(clk, rst, address, mode, write_data, out);

        input clk;
        input rst;
        input [7:0] address;
        input [31:0] write_data;
        input mode;
        output [31:0] out;

        reg [31:0] mem[255:0];
        reg [7:0] add_reg;
        reg [31:0] out;
        integer i = 1;

        initial begin
            mem[0] = 5;
            mem[1] = 53;
            mem[2] = 10;
            mem[3] = 7;
            mem[4] = 14;
            mem[5] = 22;
        end
        
        always @(posedge clk) begin
            #50
            add_reg = address;
            out = mem[add_reg];
            if(mode == 1'b0) begin
                mem[address] = write_data;
            end
            $display("%d %d %d %d %d %d", mem[0], mem[1], mem[2], mem[3], mem[4], mem[5]);
            $display("%d %d %d", mode, address, write_data, out);

        end

        always @(posedge clk) begin
            
        end

        always @(posedge rst) begin
            mem[address] = 1'b0;
        end

    endmodule



    module decode_opcode(opcode,operator);
        input wire [5:0] opcode;
        output wire [3:0] operator;
        input wire clk;
        assign operator = opcode[5]?(opcode[4]?4'b1000:4'b0100):(opcode[4]?4'b0010:4'b0001);
    endmodule


    module andnew(o1,in1,in2);
        input wire [31:0] in1,in2;
        output wire [31:0] o1;
        genvar i;
        generate 
            for(i=0;i<32;i=i+1) begin
                assign o1[i] = in2[i] & in1[i];
            end
        endgenerate
    endmodule

    module ornew(o1,in1,in2);

        input wire [31:0] in1,in2;
        output wire [31:0] o1;

        genvar i;

        generate 
            for(i=0;i<32;i=i+1) begin
                assign o1[i] = in2[i] | in1[i];
            end
        endgenerate
    endmodule

    module fulladder(a, b, cin, sum, cout);
        input wire a, b, cin;
        output wire sum, cout;
        assign sum = a ^ b ^ cin;  
        assign cout = (a & b) | (cin & (a ^ b)); 

    endmodule

    module addnew(o,a,b);
        input [31:0] a,b;    
        output [31:0] o;
        wire [32:0] c;
        assign c[0] = 1'b0;
        genvar i;
        generate
            for(i=0; i<32; i=i+1) begin
                fulladder addit(a[i], b[i], c[i], o[i], c[i+1]);
            end
        endgenerate
    endmodule

    module fullsubtractor(a, b, c_in, diff, c_out);
        input wire a, b, c_in;
        output wire diff, c_out;
        assign diff = a ^ b ^ c_in;
        assign c_out = (~a & (b ^ c_in)) | (b & c_in);
    endmodule

    module subnew(o1,in1,in2);
        input [31:0] in1,in2;    
        output [31:0] o1;
        wire [32:0] c;
        assign c[0] = 1'b0;
        genvar i;
        generate
            for(i=0; i<32; i=i+1) begin
                fullsubtractor subit(in1[i], in2[i], c[i], o1[i], c[i+1]);
            end
        endgenerate
    endmodule

    module sllnew(o1,in1,in2);
        input [31:0] in1, in2;
        output [31:0] o1;
        assign o1 = in1 << in2;
    endmodule

    module srlnew(o1,in1,in2);
        input [31:0] in1, in2;
        output [31:0] o1;
        assign o1 = in1 >> in2;
    endmodule

    module sltnew(o1,in1,in2);
        input [31:0] in1, in2;
        output [31:0] o1;
        assign o1[0] = in1 < in2;
    endmodule


    module brancher(pc, inst_fetch, jump_address, enable, ALU_out, new_pc, pc_plus_1);
        input [31:0] pc;
        input [31:0] inst_fetch;
        input [31:0] jump_address;
        input enable;
        input [31:0] ALU_out;
        output [31:0] new_pc;
        output [31:0] pc_plus_1;
        wire [31:0] branch_address;
        wire to_branch;
        wire [31:0] pc1;
        wire is_jr;
        assign branch_address = {{27{1'b0}}, inst_fetch[25:21]};
        assign is_jr = inst_fetch[31:26] == 6'b101000;
        addnew pc_add_1(pc_plus_1, pc, 1);
        and a0(to_branch, ALU_out[0], enable);
        mux2_32 branch_mux(to_branch, pc_plus_1, branch_address, pc1);
        mux2_32 jr_mux(is_jr, pc1, jump_address, new_pc);
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

    module ALU(opcode, var1, var2, alu_out);
        input [5:0] opcode;
        input [31:0] var1, var2;
        output [31:0] alu_out;
        wire [31:0] result [63:0];

        addnew addit(result[6'b000001],var1,var2);    //add
        subnew subit(result[6'b000010],var1,var2);    //sub
        assign result[6'b000011] = result[6'b000001]; //addu
        assign result[6'b000100] = result[6'b000010]; //subu
        assign result[6'b110101] = result[6'b000001]; //addi
        assign result[6'b110110] = result[6'b000001]; //addui
        andnew andit(result[6'b000111],var1,var2);    //and
        ornew orit(result[6'b001000],var1,var2);      //or
        assign result[6'b111001] = result[6'b000111]; //andi
        assign result[6'b111010] = result[6'b001000]; //ori
        sllnew sllit(result[6'b001011],var1,var2);    //sll
        srlnew srlit(result[6'b001100],var1,var2);    //srl
        sltnew sltit(result[6'b001101],var1,var2);    //slt
        assign result[6'b111110] = result[6'b001101]; //slti
        assign result[6'b100001] = var1 == var2;                            //beq
        assign result[6'b100010] = ~result[6'b100001];                      //bne
        assign result[6'b100011] = (~result[6'b001101]|~result[6'b100001]); //bgt
        assign result[6'b100100] = (result[6'b100011]|result[6'b100001]);   //bgte
        assign result[6'b100101] = result[6'b001101];                       //ble
        assign result[6'b100110] = (result[6'b001101]|result[6'b100001]) ;  //bleq
        assign result[6'b100111] = {32{1'b1}};              //j
        assign result[6'b101000] = {{31{1'b0}}, 1'b1};      //jr
        assign result[6'b101001] = {32{1'b1}};              //jal

        assign alu_out = result[opcode];
        
    endmodule



    module command_runner;
        reg clk;
        reg [31:0] registers[31:0];
        reg [31:0] pc;
        initial begin
            pc <= 8'b00000000;
        end
        reg [4:0] counter;
        wire [3:0] type;
        wire [5:0] opcode;
        wire [31:0] alu_input1;
        wire [31:0] alu_input2;
        wire [31:0] alu_output;
        wire [31:0] new_pc;
        wire [31:0] pc_plus_1;
        wire mem_write;
        wire [31:0] mem_write_data;
        wire [7:0] mem_address;
        wire [31:0] mem_read;

        wire [31:0] reg_write_data;
        wire [7:0] reg_address;

        initial begin
            clk=0;
        end
        
        always #200 clk=~clk;
        
        initial begin
            counter = 5'b00000;
            registers[2] <= 7;
            registers[3] <= 2;
            registers[7] <= 31;
            registers[5] <= 21;
            registers[4] <= 75;
            registers[8] <= 3;
            registers[9] <= 1;
            registers[17] <= 5;
            registers[16] <= 2;
            registers[15] <= 5;
            registers[14] <= 5;
            #4000 $finish;
        end

        wire [31:0] instr;
        
        instruction_memory instr1(clk, 1'b0, pc[7:0], 1'b1, 0, instr);
        data_memory data1(clk, 1'b0, mem_address, mem_write, mem_write_data, mem_read);
        assign opcode = instr[31:26];
        decode_opcode dec1(opcode, type);
        assign alu_input1 = registers[instr[20:16]];
        assign alu_input2 = type[3]?{16'b0000000000000000,instr[15:0]}:registers[instr[15:11]];
        ALU alutry(opcode, alu_input1, alu_input2, alu_output);
        brancher brancher0(pc, instr, alu_input1, type[2], alu_output, new_pc, pc_plus_1);

        assign mem_write = (instr[31:26] == 6'b010001);
        assign mem_address = registers[instr[5:0]];
        assign mem_write_data = registers[instr[25:21]];

        assign reg_address = instr[25:21];
        assign reg_write = ((instr[31:30] == 2'b00) | (instr[31:26] == 6'b010010))|(instr[31:30] == 2'b11);    
        assign jal = instr[31:26] == 6'b100011;
        mux2_32 reg_load(type[1], alu_output, mem_read, reg_write_data);

        always @(negedge clk) begin
            registers[5'b00001] = instr;
            #100
            if (jal == 1'b1 )begin
                registers[5'b00000] <= pc_plus_1;
            end else if (reg_write == 1'b1) begin
                registers[reg_address] <= reg_write_data;
            end
            // $display("%b", instr);
            // $display("%b", type);
            // $display("%b", type[3]);
            // $display("%d", alu_input1);
            // $display("%d", alu_input2);
            // $display("%b", alu_output);
            $display("%d", pc);
            $display("%d %d %d", mem_write, mem_address, mem_write_data, mem_read);
            // $display("%d", registers[2]);
            pc <= new_pc; 
        end

        always @(posedge clk) begin
        end
    endmodule
