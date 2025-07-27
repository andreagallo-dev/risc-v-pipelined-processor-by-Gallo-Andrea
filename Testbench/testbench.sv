//-----------------------------------------------------------------------------
// Module:  testbench
// Author:  Gallo Andrea 2359271
//
// Description:
// This is a testbench for the pipelined RISC-V R32I processor, instantiated
// as 'dut'. The testbench's primary functions are:
//   1. Clock and Reset Generation: It generates a 100 MHz clock and an
//      active-high reset signal, which is asserted for 2 clock cycles at
//      the beginning of the simulation.
//   2. Test Execution: After de-asserting reset, it allows the DUT to run
//      for a fixed duration of 200 clock cycles. It assumes that the
//      instruction memory within the DUT is pre-loaded with a test program.
//   3. Results Verification: At the end of the simulation, it performs a
//      white-box check by directly accessing the DUT's internal register
//      file and printing the final state of all 32 GPRs to the console.
//   4. Debugging Support: It generates a VCD (Value Change Dump) file
//      named 'waveout.vcd' for waveform analysis with tools like GTKWave.
//-----------------------------------------------------------------------------

`timescale 1ns/1ps

module RISCV_pipeline_tb;
    logic clk;
    logic rst;
    wire [31:0] wd_tb;
    wire [31:0] address_tb;
    wire mem_w_tb;

    topmodule dut (
        .clk(clk),
        .rst(rst),
        .write_data(wd_tb),
        .address(address_tb),
        .write_mem(mem_w_tb)
    );

    localparam CLK_PERIOD = 10;

    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    initial begin
        integer i; 
        $dumpfile("gtkwave/waveout.vcd");
        $dumpvars(0, dut);
        
        rst = 1'b1;
        #(CLK_PERIOD * 2);

        rst = 1'b0;
        #(CLK_PERIOD * 200);

        $display("\n----------------------------------------------------------------");
        for (i = 0; i < 32; i = i + 1) begin
            $display("x%02d .------ hex: 0x%08h .------ dec (signed): %11d", 
                     i, 
                     dut.rf_u.rf_data[i], 
                     $signed(dut.rf_u.rf_data[i]) ); 
        end
        $display("----------------------------------------------------------------\n");
        $finish;
    end

endmodule