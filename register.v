module regfile(clk, rst, address, mode, write_enable, write_data, out);

    input clk, rst, [4:0] address,[31:0] write_data, write_enable, mode;
    output [31:0] out;

    reg [31:0] mem[31:0];
    reg [4:0] add_reg;
    reg [31:0] out;

     initial begin
        for(i = 0; i < 32; i = i + 1) begin
            mem[i] <= 0;
        end
    end

    always @(posedge clk) begin
        add_reg <= address;
    end

    always @(mem[add_reg]) begin
        out <= mem[add_reg];
    end

    always @(posedge clk) begin
        if(@mode)

