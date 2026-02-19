// Simple dual-port ROM wrapper for framebuffer (16-bit words)
module framebuffer_rom #(
    parameter WIDTH = 16,
    parameter DEPTH = 307200
) (
    input clk,
    input [31:0] addr,
    output reg [WIDTH-1:0] data
);
    // memory
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    initial begin
        // external .mem file expected at simulation time: framebuffer.mem
        if ($fopen("framebuffer.mem") == 0) begin
            $display("framebuffer.mem not found (simulation)");
        end
        $readmemh("framebuffer.mem", mem);
    end
    always @(posedge clk) begin
        if (addr < DEPTH)
            data <= mem[addr];
        else
            data <= 0;
    end
endmodule
