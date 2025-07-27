//-----------------------------------------------------------------------------
// Module:  RegisterFile
// Author:  Gallo Andrea 2359271
//
// Description:
//   A 32x32-bit register file for the RISC-V CPU.
//   It features two asynchronous read ports (rd1, rd2) and one synchronous
//   write port (wd3). Register x0 is hardwired to zero.
//-----------------------------------------------------------------------------
module regfile (
    // Inputs
    input  logic        i_clk,       // Clock signal
    input  logic        i_reg_write, // Write enable signal for the write port
    input  logic [4:0]  i_addr_rd1,  // Address for read port 1
    input  logic [4:0]  i_addr_rd2,  // Address for read port 2
    input  logic [4:0]  i_addr_wr,   // Address for the write port
    input  logic [31:0] i_data_wr,   // Data to be written

    // Outputs
    output logic [31:0] o_data_rd1,  // Data from read port 1
    output logic [31:0] o_data_rd2   // Data from read port 2
);

    // The core storage element: 32 registers, each 32 bits wide.
    logic [31:0] rf_data[31:0];

    // Initialization block to ensure all registers start at 0.
    // This is for simulation purposes.
    initial begin
        integer i;
        for (i = 0; i < 32; i = i + 1) begin
            rf_data[i] = 32'b0;
        end
    end

    // --- Synchronous Write Port ---
    // Writes occur on the rising edge of the clock if write enable is high.
    // This is the standard, robust, and correct way to design a synchronous system.
    always_ff @(posedge i_clk) begin
        if (i_reg_write && (i_addr_wr != 5'b0)) begin
            rf_data[i_addr_wr] <= i_data_wr;
        end
    end

    // --- Asynchronous Read Ports ---
    // Reading register x0 (address 0) always returns 0.
    assign o_data_rd1 = (i_addr_rd1 == 5'b0) ? 32'b0 : rf_data[i_addr_rd1];
    assign o_data_rd2 = (i_addr_rd2 == 5'b0) ? 32'b0 : rf_data[i_addr_rd2];

endmodule