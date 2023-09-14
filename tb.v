`timescale 1ns/1ps
`include "instruction_fetch.v"

module tb;
    reg clk;
    initial begin
        clk <= 1'b0;
        forever begin
            #200 clk <= ~clk;
        end
    end
    processor p0(clk);
    
    initial begin
        #4000
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