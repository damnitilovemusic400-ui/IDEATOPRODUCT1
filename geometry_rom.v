// Geometry ROM (32-bit entries) wrapper
module geometry_rom #(
    parameter WIDTH = 32,
    parameter DEPTH = 8192
) (
    input clk,
    input [31:0] addr,
    output reg [WIDTH-1:0] data
);
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    initial begin
        if ($fopen("geometry32.mem") == 0) begin
            $display("geometry32.mem not found (simulation)");
        end
        $readmemh("geometry32.mem", mem);
    end
    always @(posedge clk) begin
        if (addr < DEPTH)
            data <= mem[addr];
        else
            data <= 0;
    end
endmodule
