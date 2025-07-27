//-----------------------------------------------------------------------------
// Module:  DataMemory
// Author:  Gallo Andrea 2359271
//
// Description:
//   A simple 32-bit wide, word-addressable memory with synchronous write
//   and asynchronous read. It supports byte-enables for partial word writes,
//   which is essential for RISC-V SB, SH instructions.
//-----------------------------------------------------------------------------
module data_mem #(
    parameter MEM_WORDS = 64 // Number of 32-bit words in the memory
)(
    // Interface Ports
    input  logic                   i_clk,
    input  logic                   i_write_en,    // Main write enable for the current cycle
    input  logic [31:0]            i_addr,        // 32-bit byte address
    input  logic [31:0]            i_write_data,  // 32-bit data to be written
    input  logic [3:0]             i_byte_en,     // 4-bit byte enable mask (1 per byte)
    output logic [31:0]            o_read_data    // 32-bit data read from memory
);

    localparam ADDR_WIDTH = $clog2(MEM_WORDS);

    // Memory array declaration. This models a simple SRAM.
    logic [31:0] mem_array [MEM_WORDS];

    // Simulation-only initialization block.
    // This ensures the memory starts in a known, zeroed state to avoid
    // reading 'x' (unknown)
    initial begin
        for (integer i = 0; i < MEM_WORDS; i = i + 1) begin
            mem_array[i] = 32'b0;
        end
    end


    wire [ADDR_WIDTH-1:0] word_addr = i_addr[ADDR_WIDTH+1:2];

    // --- Synchronous Write Logic ---
    // The memory is updated on the rising edge of the clock.
    // A for-loop is used to handle byte-enables cleanly.
    always_ff @(posedge i_clk) begin
        if (i_write_en) begin            
            for (int i = 0; i < 4; i++) begin
                if (i_byte_en[i])     mem_array[word_addr][8*i +: 8] <= i_write_data[8*i +: 8];
            end
        end
    end

    // --- Asynchronous Read Logic ---
    // The read operation is combinational. The output data reflects the
    // content of the currently addressed memory location.
    assign o_read_data = mem_array[word_addr];

endmodule