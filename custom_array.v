module binary_cell (
    input clk,
    input sel,       // Select this cell
    input rw_bar,    // Active-low: 1 = write, 0 = read
    input di,        // Data In
    output reg do    // Data Out
);
    reg q;

    always @(posedge clk) begin
        if (sel && rw_bar) begin
            q <= di;    // Store data
        end
    end

    always @(*) begin
        if (sel && !rw_bar) begin
            do = q;     // Read data
        end else begin
            do = 1'b0;  // Otherwise, output 0
        end
    end
endmodule
module memory_row (
    input clk,
    input sel,
    input rw_bar,
    input [2:0] di,
    output [2:0] do
);
    binary_cell c0 (.clk(clk), .sel(sel), .rw_bar(rw_bar), .di(di[0]), .do(do[0]));
    binary_cell c1 (.clk(clk), .sel(sel), .rw_bar(rw_bar), .di(di[1]), .do(do[1]));
    binary_cell c2 (.clk(clk), .sel(sel), .rw_bar(rw_bar), .di(di[2]), .do(do[2]));
endmodule
module decoder_2to4 (
    input [1:0] sel,
    input en,
    output [3:0] d_out
);
    assign d_out[0] = en & (~sel[1] & ~sel[0]);
    assign d_out[1] = en & (~sel[1] &  sel[0]);
    assign d_out[2] = en & ( sel[1] & ~sel[0]);
    assign d_out[3] = en & ( sel[1] &  sel[0]);
endmodule
module memory_4x3 (
    input clk,
    input en,              // Global enable
    input rw_bar,          // Active-low write/read
    input [1:0] sel,       // Row select
    input [2:0] data_in,
    output [2:0] data_out
);
    wire [3:0] row_enable;
    wire [2:0] do0, do1, do2, do3;

    decoder_2to4 dec (.sel(sel), .en(en), .d_out(row_enable));

    memory_row r0 (.clk(clk), .sel(row_enable[0]), .rw_bar(rw_bar), .di(data_in), .do(do0));
    memory_row r1 (.clk(clk), .sel(row_enable[1]), .rw_bar(rw_bar), .di(data_in), .do(do1));
    memory_row r2 (.clk(clk), .sel(row_enable[2]), .rw_bar(rw_bar), .di(data_in), .do(do2));
    memory_row r3 (.clk(clk), .sel(row_enable[3]), .rw_bar(rw_bar), .di(data_in), .do(do3));

    // Read: Output from selected row using OR gates (Falstad-style)
    assign data_out = (rw_bar == 0) ? 
                      (row_enable[0] ? do0 :
                       row_enable[1] ? do1 :
                       row_enable[2] ? do2 :
                       row_enable[3] ? do3 : 3'b000) : 3'b000;

endmodule
`timescale 1ns/1ps

module tb_memory_4x3;
    reg clk, en, rw_bar;
    reg [1:0] sel;
    reg [2:0] data_in;
    wire [2:0] data_out;

    memory_4x3 uut (
        .clk(clk),
        .en(en),
        .rw_bar(rw_bar),
        .sel(sel),
        .data_in(data_in),
        .data_out(data_out)
    );

    always #5 clk = ~clk;

    initial begin
        $display("Starting 4x3 Binary Memory Cell Test");
        $monitor("T=%0t | EN=%b RW̅=%b SEL=%b | DI=%b -> DO=%b", 
                  $time, en, rw_bar, sel, data_in, data_out);

        clk = 0;
        en = 1;

        // Write to all rows
        rw_bar = 1; // Write mode
        sel = 2'b00; data_in = 3'b101; #10;
        sel = 2'b01; data_in = 3'b110; #10;
        sel = 2'b10; data_in = 3'b011; #10;
        sel = 2'b11; data_in = 3'b000; #10;

        // Read from all rows
        rw_bar = 0; // Read mode
        sel = 2'b00; #10;
        sel = 2'b01; #10;
        sel = 2'b10; #10;
        sel = 2'b11; #10;

        $finish;
    end
endmodule
