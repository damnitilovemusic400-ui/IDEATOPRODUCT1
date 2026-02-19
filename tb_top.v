`timescale 1ns/1ps
module tb_top;
    reg clk = 0;
    reg rst = 1;
    wire [15:0] pixel;
    wire uart_tx;

    always #5 clk = ~clk;

    top uut(.clk(clk), .rst(rst), .uart_rx(1'b1), .uart_tx(uart_tx), .pixel_out(pixel));

    initial begin
        $display("TB START");
        #20 rst = 0;
        #2000 $display("pixel sample: %h", pixel);
        $finish;
    end
endmodule
