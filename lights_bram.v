// Simple synchronous BRAM-like module for lights state (16-bit words)
module lights_bram #(
    parameter WIDTH = 16,
    parameter DEPTH = 2048
) (
    input clk,
    input we,
    input [31:0] addr,
    input [WIDTH-1:0] din,
    output reg [WIDTH-1:0] dout
);
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    initial begin
        if ($fopen("lights.mem") == 0) begin
            $display("lights.mem not found (simulation)");
        end
        $readmemh("lights.mem", mem);
    end
    always @(posedge clk) begin
        if (we && addr < DEPTH)
            mem[addr] <= din;
        if (addr < DEPTH)
            dout <= mem[addr];
        else
            dout <= 0;
    end
endmodule
