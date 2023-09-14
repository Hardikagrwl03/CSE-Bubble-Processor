module Instruction_fetch(clk,Curr_Instruction);
input clk;
output reg [31:0] Curr_Instruction;
reg [4:0] pc = 5'b00000;
reg [31:0] veda_mem [31:0];

initial begin

    veda_mem[0] = 32'b00000000001000100000000000100000;
    veda_mem[1] = 32'b00000000010000010000000000100010;
    veda_mem[2] = 32'b00000000001000100000000000100001;
    veda_mem[3] = 32'b00000000010000010000000000100011;
    veda_mem[4] = 32'b00100000001000000000001111101000;
    veda_mem[5] = 32'b00100100001000000000001111101000;
    veda_mem[6] = 32'b00000000001000100000000000100100;
    veda_mem[7] = 32'b00000000001000100000000000100101;
    veda_mem[8] = 32'b00110000001000000000001111101000;
    veda_mem[9] = 32'b00110100001000000000001111101000;
    veda_mem[10] = 32'b00000000000000010000001010000000;
    veda_mem[11] = 32'b00000000000000010000001010000010;
    veda_mem[12] = 32'b10001100001000000000000000001010;
    veda_mem[13] = 32'b10101100001000000000000000001010;
    veda_mem[14] = 32'b00010000000000010000000000001010;
    veda_mem[15] = 32'b00010100000000010000000000001010;
    veda_mem[16] = 32'b00011100001000100000000000001010;
    veda_mem[17] = 32'b00111100001000100000000000001010;
    veda_mem[18] = 32'b00011000001000100000000000001010;
    veda_mem[19] = 32'b01111100001000100000000000001010;
    veda_mem[20] = 32'b00001000000000000000000000000010;
    veda_mem[21] = 32'b00000000000000000000000000001000;
    veda_mem[22] = 32'b00001100000000000000000000001010;
    veda_mem[23] = 32'b00000000001000100000000000101010;
    veda_mem[24] = 32'b00101000010000010000000001100100;
    
end

always @(posedge clk) begin
    
    if(pc<25)begin
        Curr_Instruction = veda_mem[pc];
        // $display("%b",Curr_Instruction);
    end
    pc = pc + 1;
end

endmodule


module Decode_Instruction;
reg clk;
reg [5:0] Opcode;
reg [5:0] Funct;



wire [31:0] Curr_Instruction;

Instruction_fetch uut(clk,Curr_Instruction);

initial begin
clk = 0;
forever #10 clk = ~clk;
end

initial begin
#800 $finish;

end

always @(negedge clk)
begin

    Opcode =  Curr_Instruction[31:26];
    Funct = Curr_Instruction[5:0];

    case(Opcode)
        6'b000000: case(Funct)
        