// Top-level wrapper for integration. Minimal signals for simulation/synthesis skeleton.
module top (
    input clk,
    input rst,
    // host UART pins (abstract)
    input uart_rx,
    output uart_tx,
    // VGA-like outputs simplified
    output [15:0] pixel_out
);
    // instantiate framebuffer ROM
    wire [15:0] fb_data;
    reg [31:0] fb_addr;
    framebuffer_rom fb_rom(.clk(clk), .addr(fb_addr), .data(fb_data));

    // lights BRAM
    wire [15:0] lights_dout;
    reg [31:0] lights_addr;
    reg lights_we;
    reg [15:0] lights_din;
    lights_bram lights(.clk(clk), .we(lights_we), .addr(lights_addr), .din(lights_din), .dout(lights_dout));

    assign pixel_out = fb_data;
    assign uart_tx = 1'b1;

    // simple address counter for simulation
    always @(posedge clk) begin
        if (rst) fb_addr <= 0;
        else fb_addr <= fb_addr + 1;
    end
    always @(posedge clk) begin
        if (rst) begin lights_addr <= 0; lights_we <= 0; end
    end
endmodule
